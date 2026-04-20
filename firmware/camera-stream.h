#pragma once
#include "hal/hal.h"

#if HAS_CAMERA

#include "esp_camera.h"
#include <WiFi.h>

// MJPEG stream runs on a separate async server to avoid blocking the main
// WebServer.  Port 81 is the de-facto standard for ESP32-CAM streams.
#define STREAM_PORT 81

static WiFiServer streamServer(STREAM_PORT);

// ── Camera Power & Rate Control ──────────────────────────────────────────────
// cameraEnabled: user switch (persisted in NVS via main firmware)
// cameraMoving:  set true while a movement command is active
static volatile bool cameraEnabled = true;
static volatile bool cameraMoving  = false;
static volatile bool cameraInitialised = false;

// Frame intervals: 50ms (~20fps) when moving, 1000ms (1fps) when idle
#define CAM_INTERVAL_ACTIVE_MS   50
#define CAM_INTERVAL_IDLE_MS   1000

static void cameraPowerDown() {
#if defined(CAM_PIN_PWDN) && CAM_PIN_PWDN >= 0
  digitalWrite(CAM_PIN_PWDN, HIGH);  // HIGH = power down for OV2640
#endif
}

static void cameraPowerUp() {
#if defined(CAM_PIN_PWDN) && CAM_PIN_PWDN >= 0
  digitalWrite(CAM_PIN_PWDN, LOW);   // LOW = active
  delay(5);  // Stabilisation time
#endif
}

// ── Camera Initialisation ────────────────────────────────────────────────────
static bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = CAM_PIN_D0;
  config.pin_d1       = CAM_PIN_D1;
  config.pin_d2       = CAM_PIN_D2;
  config.pin_d3       = CAM_PIN_D3;
  config.pin_d4       = CAM_PIN_D4;
  config.pin_d5       = CAM_PIN_D5;
  config.pin_d6       = CAM_PIN_D6;
  config.pin_d7       = CAM_PIN_D7;
  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_pclk     = CAM_PIN_PCLK;
  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_sccb_sda = CAM_PIN_SIOD;
  config.pin_sccb_scl = CAM_PIN_SIOC;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  // Use lower resolution for minimal latency over WiFi
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_QVGA;  // 320×240 — low latency
    config.jpeg_quality = 14;              // good balance of size vs quality
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320×240
    config.jpeg_quality = 16;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DBG_PRINTF("Camera init failed: 0x%x\n", err);
    return false;
  }

  // Optional: tweak sensor settings for better indoor quality
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, config.frame_size);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }

  DBG_PRINTLN("Camera initialised");
  cameraInitialised = true;
  return true;
}

static void deinitCamera() {
  if (!cameraInitialised) return;
  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    DBG_PRINTF("Camera deinit warning: 0x%x\n", err);
  }
  cameraInitialised = false;
}

static bool reinitCameraWithRetries(uint8_t attempts) {
  deinitCamera();
  for (uint8_t i = 0; i < attempts; i++) {
    if (initCamera()) {
      // Probe one frame to ensure sensor + DMA path are alive.
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) {
        esp_camera_fb_return(fb);
        return true;
      }
      deinitCamera();
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
  return false;
}

// ── MJPEG Stream ─────────────────────────────────────────────────────────────
// Serves a multipart MJPEG stream to a single connected client at a time.
// Called from loop(); non-blocking when no client is connected.

static const char* STREAM_BOUNDARY  = "sesameframe";
static const char* STREAM_CONTENT   = "multipart/x-mixed-replace;boundary=sesameframe";
static const char* STREAM_PART      = "\r\n--sesameframe\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static WiFiClient streamClient;

// Camera stream task — runs on Core 0 alongside the WiFi stack.
// Accepts one MJPEG client at a time; adapts frame rate to movement state.
static void cameraStreamTask(void *arg) {
  (void)arg;
  bool wasPoweredDown = false;
  uint8_t fbFailCount = 0;

  for (;;) {
    // If camera is disabled by user, power down and wait
    if (!cameraEnabled) {
      if (!wasPoweredDown) {
        if (streamClient && streamClient.connected()) {
          streamClient.stop();
        }
        deinitCamera();
        cameraPowerDown();
        wasPoweredDown = true;
        DBG_PRINTLN("Camera powered down");
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // Re-enable after power down
    if (wasPoweredDown) {
      cameraPowerUp();
      // Allow power rail/sensor clock to settle before re-init.
      vTaskDelay(pdMS_TO_TICKS(250));

      if (!reinitCameraWithRetries(3)) {
        DBG_PRINTLN("Camera reinit failed after wake; retrying...");
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }

      wasPoweredDown = false;
      fbFailCount = 0;
      DBG_PRINTLN("Camera powered up and reinitialised");
    }

    // Accept a new client if none is connected
    if (!streamClient || !streamClient.connected()) {
      streamClient = streamServer.accept();
      if (!streamClient) {
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      DBG_PRINTLN("Stream client connected");
      streamClient.setNoDelay(true);
      streamClient.println("HTTP/1.1 200 OK");
      streamClient.printf("Content-Type: %s\r\n", STREAM_CONTENT);
      streamClient.println("Access-Control-Allow-Origin: *");
      streamClient.println("Connection: close");
      streamClient.println();
    }

    // Grab a frame and send it
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      fbFailCount++;
      if (fbFailCount >= 20) {
        DBG_PRINTLN("Camera frame failures detected; attempting recovery");
        if (cameraEnabled) {
          reinitCameraWithRetries(2);
        }
        fbFailCount = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    fbFailCount = 0;

    char partHeader[80];
    int headerLen = snprintf(partHeader, sizeof(partHeader), STREAM_PART, fb->len);

    if (streamClient.connected()) {
      streamClient.write((const uint8_t*)partHeader, headerLen);
      streamClient.write(fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);

    if (!streamClient.connected()) {
      streamClient.stop();
      DBG_PRINTLN("Stream client disconnected");
    }

    // Adaptive rate: fast when moving, slow when idle
    uint32_t interval = cameraMoving ? CAM_INTERVAL_ACTIVE_MS : CAM_INTERVAL_IDLE_MS;
    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

static void startCameraStream() {
  streamServer.begin();
  // Pin camera stream task to Core 0 (WiFi core) so it doesn't compete
  // with servo/animation code on Core 1.
  xTaskCreatePinnedToCore(cameraStreamTask, "cam_stream", 4096, NULL, 2, NULL, 0);
  DBG_PRINTF("Camera stream task started on Core 0, port %d\n", STREAM_PORT);
}

#endif // HAS_CAMERA
