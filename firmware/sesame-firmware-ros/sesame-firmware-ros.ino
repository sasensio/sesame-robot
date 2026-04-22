// =============================================================================
// Sesame Robot — micro-ROS Firmware
// =============================================================================
//
// This firmware variant replaces the HTTP/captive-portal stack with a
// micro-ROS node that communicates with a host computer running:
//   • micro-ROS Agent (UDP bridge, port 8888)
//   • sesame_aruco ArUco detection node
//   • sesame_aruco navigation node
//
// Flash this sketch to an AI-Thinker ESP32-CAM (BOARD_ESP32CAM) or a
// compatible ESP32-S3-CAM board (BOARD_ESP32S3CAM).  Do NOT flash it to
// Distro V1/V2 or Lolin S2 Mini boards — they have no camera.
//
// Build with:
//   pio run -e esp32cam_ros   (or esp32s3cam_ros)
//
// WiFi + Agent configuration
// ──────────────────────────
// Defaults below are compile-time overrides.  The firmware also reads the
// same NVS namespace ("wifi") used by the standard firmware, so credentials
// already saved there are reused automatically.
//
// To configure via serial (115200 baud):
//   set ssid   <yourSSID>
//   set pass   <yourPassword>
//   set agent  <agentIP>
//   set port   <agentPort>
//   save
//
// ROS Topics
// ──────────
//   Publishes:
//     /camera/image_raw   sensor_msgs/CompressedImage   (JPEG, ~5–15 KB)
//     /sesame/status      std_msgs/String               (JSON)
//
//   Subscribes:
//     /sesame/cmd         std_msgs/String   ("forward","backward","left",
//                                           "right","rest","stand","wave",
//                                           "dance","swim","point","pushup",
//                                           "bow","cute","freaky","worm",
//                                           "shake","shrug","dead","crab",
//                                           "stop")
//     /sesame/pose        geometry_msgs/PoseStamped     (from ArUco node)
//
// Architecture
// ────────────
//   Core 0: camera_task — grabs JPEG frames, copies to shared double-buffer
//   Core 1: ros_spin_task (priority 5) — micro-ROS executor + publishers
//           movement_task (priority 2)  — executes pose commands, OLED anim
// =============================================================================

// ---------------------------------------------------------------------------
// Compile-time configuration (overridden by NVS at runtime)
// ---------------------------------------------------------------------------
#define DEFAULT_WIFI_SSID    ""          // Your network SSID
#define DEFAULT_WIFI_PASS    ""          // Your network password
#define DEFAULT_AGENT_IP     "192.168.1.100"
#define DEFAULT_AGENT_PORT   8888

// Camera frame size for micro-ROS publishing.
// QVGA (320×240) JPEG is typically 5–15 KB which fits in the default
// micro-ROS transport buffers after configuring colcon.meta.
// Drop to FRAMESIZE_QQVGA (160×120) if you see serialisation errors.
#define CAM_FRAMESIZE        FRAMESIZE_QVGA
#define CAM_JPEG_QUALITY     20          // Lower = better quality / larger file

// Maximum JPEG buffer size (bytes) allocated in PSRAM.
// Must be >= typical JPEG output at the chosen resolution.
#define MAX_JPEG_SIZE        (24 * 1024)

// Interval between camera publishes (ms).  100 ms ≈ 10 fps.
#define CAM_PUBLISH_INTERVAL_MS  100

// Interval between /sesame/status publishes (ms).
#define STATUS_PUBLISH_INTERVAL_MS  1000

// Set to 1 to enable Serial debug output.
#define ENABLE_SERIAL        0

// Set to 1 to automatically release PCA9685 servo outputs while resting.
#define ENABLE_SERVO_AUTO_RELEASE  1

// ---------------------------------------------------------------------------
// Debug macros
// ---------------------------------------------------------------------------
#if ENABLE_SERIAL
  #define DBG_BEGIN(baud)      Serial.begin(baud)
  #define DBG_PRINT(...)       Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)     Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)      Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN(baud)      do {} while (0)
  #define DBG_PRINT(...)       do {} while (0)
  #define DBG_PRINTLN(...)     do {} while (0)
  #define DBG_PRINTF(...)      do {} while (0)
#endif

// ---------------------------------------------------------------------------
// Board HAL and firmware headers
// ---------------------------------------------------------------------------
#include "hal/hal.h"
#include "face-bitmaps.h"
#include "movement-sequences.h"

// ---------------------------------------------------------------------------
// Arduino / ESP-IDF system headers
// ---------------------------------------------------------------------------
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#if USE_PCA9685
  #include <Adafruit_PWMServoDriver.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#if HAS_CAMERA
  #include "esp_camera.h"
#endif

// ---------------------------------------------------------------------------
// micro-ROS headers
// ---------------------------------------------------------------------------
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/compressed_image.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/pose_stamped.h>

// ---------------------------------------------------------------------------
// OLED
// ---------------------------------------------------------------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_I2C_ADDR  0x3C

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static bool oledAvailable = false;

// ---------------------------------------------------------------------------
// Servo driver
// ---------------------------------------------------------------------------
#if USE_PCA9685
  static Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);
#endif

static int8_t   servoSubtrim[8]  = {0, 0, 0, 0, 0, 0, 0, 0};
static uint16_t servoMinUs[8]    = {500, 500, 500, 500, 500, 500, 500, 500};
static uint16_t servoMaxUs[8]    = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
static uint8_t  servoMinAngle[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t  servoMaxAngle[8] = {180, 180, 180, 180, 180, 180, 180, 180};
static int16_t  servoFrameBuf[8] = {90, 90, 90, 90, 90, 90, 90, 90};

// Animation constants (can be tuned at runtime via serial)
int frameDelay        = 100;
int walkCycles        = 10;
int motorCurrentDelay = 20;

// ---------------------------------------------------------------------------
// Face animation state
// ---------------------------------------------------------------------------
String currentFaceName       = "default";
const unsigned char* const* currentFaceFrames      = nullptr;
uint8_t currentFaceFrameCount  = 0;
uint8_t currentFaceFrameIndex  = 0;
unsigned long lastFaceFrameMs  = 0;
int     faceFps                = 8;
FaceAnimMode currentFaceMode   = FACE_ANIM_LOOP;
int8_t  faceFrameDirection     = 1;
bool    faceAnimFinished       = false;
int     currentFaceFps         = 0;
bool    idleActive             = false;
bool    idleBlinkActive        = false;
unsigned long nextIdleBlinkMs  = 0;
uint8_t idleBlinkRepeatsLeft   = 0;

// ---------------------------------------------------------------------------
// Face entry tables (same as main firmware)
// ---------------------------------------------------------------------------
struct FaceEntry {
  const char*                name;
  const unsigned char* const* frames;
  uint8_t                    maxFrames;
};

static const uint8_t MAX_FACE_FRAMES = 6;

#define MAKE_FACE_FRAMES(name) \
  const unsigned char* const face_##name##_frames[] = { \
    epd_bitmap_##name, epd_bitmap_##name##_1, epd_bitmap_##name##_2, \
    epd_bitmap_##name##_3, epd_bitmap_##name##_4, epd_bitmap_##name##_5 \
  };

#define X(name) MAKE_FACE_FRAMES(name)
FACE_LIST
#undef X
#undef MAKE_FACE_FRAMES

const FaceEntry faceEntries[] = {
#define X(name) { #name, face_##name##_frames, MAX_FACE_FRAMES },
  FACE_LIST
#undef X
  { "default", face_defualt_frames, MAX_FACE_FRAMES }
};

struct FaceFpsEntry { const char* name; uint8_t fps; };
const FaceFpsEntry faceFpsEntries[] = {
  { "walk", 1 },  { "rest", 1 },  { "swim", 1 },   { "dance", 1 },
  { "wave", 1 },  { "point", 5 }, { "stand", 1 },   { "cute", 1 },
  { "pushup", 1 },{ "freaky", 1 },{ "bow", 1 },     { "worm", 1 },
  { "shake", 1 }, { "shrug", 1 }, { "dead", 2 },    { "crab", 1 },
  { "idle", 1 },  { "idle_blink", 7 }, { "default", 1 },
  { "happy", 1 },       { "talk_happy", 1 },
  { "sad", 1 },         { "talk_sad", 1 },
  { "angry", 1 },       { "talk_angry", 1 },
  { "surprised", 1 },   { "talk_surprised", 1 },
  { "sleepy", 1 },      { "talk_sleepy", 1 },
  { "love", 1 },        { "talk_love", 1 },
  { "excited", 1 },     { "talk_excited", 1 },
  { "confused", 1 },    { "talk_confused", 1 },
  { "thinking", 1 },    { "talk_thinking", 1 },
};

// ---------------------------------------------------------------------------
// Shared command state (read by movement_task, written by ROS subscriber)
// ---------------------------------------------------------------------------
String currentCommand = "";
static SemaphoreHandle_t cmd_mutex = nullptr;

// Helper used by movement functions to read currentCommand safely
static inline String getCommand() {
  String cmd;
  if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    cmd = currentCommand;
    xSemaphoreGive(cmd_mutex);
  }
  return cmd;
}

static inline void setCommand(const String& cmd) {
  if (xSemaphoreTake(cmd_mutex, portMAX_DELAY) == pdTRUE) {
    currentCommand = cmd;
    xSemaphoreGive(cmd_mutex);
  }
}

// ---------------------------------------------------------------------------
// Camera frame double-buffer
// ---------------------------------------------------------------------------
#if HAS_CAMERA
static uint8_t*          g_cam_buf       = nullptr;  // allocated in PSRAM
static size_t            g_cam_size      = 0;
static volatile bool     g_cam_ready     = false;
static SemaphoreHandle_t cam_mutex       = nullptr;
static volatile bool     cameraInitialised = false;
#endif

// ---------------------------------------------------------------------------
// Pose received from ArUco node
// ---------------------------------------------------------------------------
struct RobotPose {
  float x, y, z;
  float qx, qy, qz, qw;
  bool  valid;
};
static volatile RobotPose g_robot_pose = {0, 0, 0, 0, 0, 0, 1, false};

// ---------------------------------------------------------------------------
// WiFi / agent configuration
// ---------------------------------------------------------------------------
static char g_wifi_ssid[64]   = DEFAULT_WIFI_SSID;
static char g_wifi_pass[64]   = DEFAULT_WIFI_PASS;
static char g_agent_ip[20]    = DEFAULT_AGENT_IP;
static uint16_t g_agent_port  = DEFAULT_AGENT_PORT;

// ---------------------------------------------------------------------------
// micro-ROS entities
// ---------------------------------------------------------------------------
static rcl_allocator_t    allocator;
static rclc_support_t     support;
static rcl_node_t         node;
static rclc_executor_t    executor;

// Publishers
static rcl_publisher_t    cam_publisher;
static rcl_publisher_t    status_publisher;

// Subscribers
static rcl_subscription_t cmd_subscriber;
static rcl_subscription_t pose_subscriber;

// Message buffers (statically allocated — no heap for ROS messages)
static sensor_msgs__msg__CompressedImage cam_msg;
static uint8_t cam_msg_data_buf[MAX_JPEG_SIZE];
static char cam_msg_frame_id[]    = "camera";
static char cam_msg_format[]      = "jpeg";

static std_msgs__msg__String      status_msg;
static char status_data_buf[256];

static std_msgs__msg__String      cmd_msg;
static char cmd_data_buf[64];

static geometry_msgs__msg__PoseStamped pose_msg;
static char pose_frame_id_buf[32] = "map";

// micro-ROS agent lifecycle state
enum RosState : uint8_t {
  ROS_WAITING_AGENT,
  ROS_AGENT_AVAILABLE,
  ROS_AGENT_CONNECTED,
  ROS_AGENT_DISCONNECTED,
};
static volatile RosState ros_state = ROS_WAITING_AGENT;

// ---------------------------------------------------------------------------
// Helper macro for micro-ROS error checking (soft — logs + continues)
// ---------------------------------------------------------------------------
#define RCCHECK(fn) \
  do { \
    rcl_ret_t _rc = (fn); \
    if (_rc != RCL_RET_OK) { \
      DBG_PRINTF("micro-ROS error %d at line %d\n", (int)_rc, __LINE__); \
    } \
  } while (0)

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void updateFaceBitmap(const unsigned char* bitmap);
void setFace(const String& faceName);
void setFaceMode(FaceAnimMode mode);
void setFaceWithMode(const String& faceName, FaceAnimMode mode);
void updateAnimatedFace();
void delayWithFace(unsigned long ms);
void enterIdle();
void exitIdle();
void updateIdleBlink();
int  getFaceFpsForName(const String& faceName);
bool pressingCheck(const String& cmd, int ms);
uint8_t countFrames(const unsigned char* const* frames, uint8_t maxFrames);
void scheduleNextIdleBlink(unsigned long minMs, unsigned long maxMs);
#if USE_PCA9685
void flushServoFrame();
void setFrame(int r1, int r2, int l1, int l2, int r4, int r3, int l3, int l4);
void enableServoOutputs();
void disableServoOutputs();
void markServoActivity();
#if ENABLE_SERVO_AUTO_RELEASE
void maybeAutoReleaseServos();
#endif
#endif
void setServoAngle(uint8_t channel, int angle);
void loadConfigFromNVS();
void saveConfigToNVS();
void oledShowStatus(const char* line1, const char* line2 = nullptr);
bool rosCreateEntities();
void rosDestroyEntities();
void publishStatus();
static void rosSpinTask(void* arg);
static void movementTask(void* arg);
#if HAS_CAMERA
bool initCamera();
static void cameraTask(void* arg);
#endif

// =============================================================================
// OLED helpers
// =============================================================================

void oledShowStatus(const char* line1, const char* line2) {
  if (!oledAvailable) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2) {
    display.setCursor(0, 12);
    display.println(line2);
  }
  display.display();
}

// =============================================================================
// Face animation (identical to main firmware, minus WebServer calls)
// =============================================================================

void updateFaceBitmap(const unsigned char* bitmap) {
  if (!oledAvailable) return;
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, 128, 64, SSD1306_WHITE);
  display.display();
}

uint8_t countFrames(const unsigned char* const* frames, uint8_t maxFrames) {
  if (!frames || !frames[0]) return 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < maxFrames; i++) {
    if (!frames[i]) break;
    count++;
  }
  return count;
}

int getFaceFpsForName(const String& faceName) {
  for (size_t i = 0; i < sizeof(faceFpsEntries) / sizeof(faceFpsEntries[0]); i++) {
    if (faceName.equalsIgnoreCase(faceFpsEntries[i].name)) {
      return faceFpsEntries[i].fps;
    }
  }
  return faceFps;
}

void setFace(const String& faceName) {
  if (faceName == currentFaceName && currentFaceFrames) return;
  currentFaceName        = faceName;
  currentFaceFrameIndex  = 0;
  lastFaceFrameMs        = 0;
  faceFrameDirection     = 1;
  faceAnimFinished       = false;
  currentFaceFps         = getFaceFpsForName(faceName);
  currentFaceFrames      = face_defualt_frames;
  currentFaceFrameCount  = countFrames(face_defualt_frames, MAX_FACE_FRAMES);
  for (size_t i = 0; i < sizeof(faceEntries) / sizeof(faceEntries[0]); i++) {
    if (faceName.equalsIgnoreCase(faceEntries[i].name)) {
      currentFaceFrames     = faceEntries[i].frames;
      currentFaceFrameCount = countFrames(faceEntries[i].frames, faceEntries[i].maxFrames);
      break;
    }
  }
  if (currentFaceFrameCount == 0) {
    currentFaceFrames     = face_defualt_frames;
    currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);
    currentFaceName       = "default";
    currentFaceFps        = getFaceFpsForName(currentFaceName);
  }
  if (currentFaceFrameCount > 0 && currentFaceFrames[0]) {
    updateFaceBitmap(currentFaceFrames[0]);
  }
}

void setFaceMode(FaceAnimMode mode) {
  currentFaceMode    = mode;
  faceFrameDirection = 1;
  faceAnimFinished   = false;
}

void setFaceWithMode(const String& faceName, FaceAnimMode mode) {
  setFaceMode(mode);
  setFace(faceName);
}

void updateAnimatedFace() {
  if (!oledAvailable || !currentFaceFrames || currentFaceFrameCount <= 1) return;
  if (currentFaceMode == FACE_ANIM_ONCE && faceAnimFinished) return;
  unsigned long now      = millis();
  int fps                = max(1, (currentFaceFps > 0 ? currentFaceFps : faceFps));
  unsigned long interval = 1000UL / fps;
  if (now - lastFaceFrameMs < interval) return;
  lastFaceFrameMs = now;
  if (currentFaceMode == FACE_ANIM_LOOP) {
    currentFaceFrameIndex = (currentFaceFrameIndex + 1) % currentFaceFrameCount;
  } else if (currentFaceMode == FACE_ANIM_ONCE) {
    if (currentFaceFrameIndex + 1 >= currentFaceFrameCount) {
      currentFaceFrameIndex = currentFaceFrameCount - 1;
      faceAnimFinished = true;
    } else {
      currentFaceFrameIndex++;
    }
  } else { // BOOMERANG
    if (faceFrameDirection > 0) {
      if (currentFaceFrameIndex + 1 >= currentFaceFrameCount) {
        faceFrameDirection = -1;
        if (currentFaceFrameIndex > 0) currentFaceFrameIndex--;
      } else {
        currentFaceFrameIndex++;
      }
    } else {
      if (currentFaceFrameIndex == 0) {
        faceFrameDirection = 1;
        if (currentFaceFrameCount > 1) currentFaceFrameIndex++;
      } else {
        currentFaceFrameIndex--;
      }
    }
  }
  updateFaceBitmap(currentFaceFrames[currentFaceFrameIndex]);
}

void scheduleNextIdleBlink(unsigned long minMs, unsigned long maxMs) {
  nextIdleBlinkMs = millis() + (unsigned long)random(minMs, maxMs);
}

void enterIdle() {
  idleActive           = true;
  idleBlinkActive      = false;
  idleBlinkRepeatsLeft = 0;
  setFaceWithMode("idle", FACE_ANIM_BOOMERANG);
  scheduleNextIdleBlink(3000, 7000);
}

void exitIdle() {
  idleActive      = false;
  idleBlinkActive = false;
}

void updateIdleBlink() {
  if (!idleActive) return;
  if (!idleBlinkActive) {
    if (millis() >= nextIdleBlinkMs) {
      idleBlinkActive = true;
      if (idleBlinkRepeatsLeft == 0 && random(0, 100) < 30) idleBlinkRepeatsLeft = 1;
      setFaceWithMode("idle_blink", FACE_ANIM_ONCE);
    }
    return;
  }
  if (currentFaceMode == FACE_ANIM_ONCE && faceAnimFinished) {
    idleBlinkActive = false;
    setFaceWithMode("idle", FACE_ANIM_BOOMERANG);
    if (idleBlinkRepeatsLeft > 0) {
      idleBlinkRepeatsLeft--;
      scheduleNextIdleBlink(120, 220);
    } else {
      scheduleNextIdleBlink(3000, 7000);
    }
  }
}

// =============================================================================
// Servo / PCA9685 (identical to main firmware)
// =============================================================================

#if USE_PCA9685
static bool    servoFrameDirty      = false;
static bool    servoOutputsEnabled  = true;
static unsigned long lastServoActivityMs = 0;
#define PCA9685_LED0_ON_L 0x06

#if ENABLE_SERVO_AUTO_RELEASE
static const unsigned long SERVO_AUTO_RELEASE_MS = 1500;
#endif

void enableServoOutputs() {
  if (servoOutputsEnabled) return;
  servoOutputsEnabled = true;
  servoFrameDirty     = true;
}

void disableServoOutputs() {
  if (!servoOutputsEnabled) return;
  for (uint8_t ch = 0; ch < 8; ch++) pwmDriver.setPWM(ch, 0, 4096);
  servoOutputsEnabled = false;
  servoFrameDirty     = false;
}

void markServoActivity() { lastServoActivityMs = millis(); }

#if ENABLE_SERVO_AUTO_RELEASE
void maybeAutoReleaseServos() {
  if (servoOutputsEnabled && getCommand() == "" &&
      currentFaceName.equalsIgnoreCase("rest") &&
      (millis() - lastServoActivityMs) >= SERVO_AUTO_RELEASE_MS) {
    disableServoOutputs();
  }
}
#endif

void flushServoFrame() {
  if (!servoFrameDirty || !servoOutputsEnabled) return;
  uint8_t buf[32];
  for (uint8_t ch = 0; ch < 8; ch++) {
    int16_t angle  = constrain(servoFrameBuf[ch],
                               (int16_t)servoMinAngle[ch],
                               (int16_t)servoMaxAngle[ch]);
    uint16_t us    = map(angle, 0, 180, servoMinUs[ch], servoMaxUs[ch]);
    uint16_t ticks = map(us, 500, 2500, PCA9685_SERVOMIN, PCA9685_SERVOMAX);
    buf[ch * 4 + 0] = 0;
    buf[ch * 4 + 1] = 0;
    buf[ch * 4 + 2] = ticks & 0xFF;
    buf[ch * 4 + 3] = (ticks >> 8) & 0x0F;
  }
  Wire.beginTransmission(PCA9685_I2C_ADDR);
  Wire.write(PCA9685_LED0_ON_L);
  Wire.write(buf, 32);
  Wire.endTransmission();
  servoFrameDirty = false;
}

void setFrame(int r1, int r2, int l1, int l2, int r4, int r3, int l3, int l4) {
  enableServoOutputs();
  servoFrameBuf[R1] = constrain(r1 + servoSubtrim[R1], 0, 180);
  servoFrameBuf[R2] = constrain(r2 + servoSubtrim[R2], 0, 180);
  servoFrameBuf[L1] = constrain(l1 + servoSubtrim[L1], 0, 180);
  servoFrameBuf[L2] = constrain(l2 + servoSubtrim[L2], 0, 180);
  servoFrameBuf[R4] = constrain(r4 + servoSubtrim[R4], 0, 180);
  servoFrameBuf[R3] = constrain(r3 + servoSubtrim[R3], 0, 180);
  servoFrameBuf[L3] = constrain(l3 + servoSubtrim[L3], 0, 180);
  servoFrameBuf[L4] = constrain(l4 + servoSubtrim[L4], 0, 180);
  markServoActivity();
  servoFrameDirty = true;
  flushServoFrame();
}
#endif // USE_PCA9685

void setServoAngle(uint8_t channel, int angle) {
  if (channel >= 8) return;
  int adjusted = constrain(angle + servoSubtrim[channel],
                           (int)servoMinAngle[channel],
                           (int)servoMaxAngle[channel]);
#if USE_PCA9685
  enableServoOutputs();
  markServoActivity();
  servoFrameBuf[channel] = adjusted;
  servoFrameDirty        = true;
#else
  (void)adjusted; // non-camera boards not supported in this firmware
#endif
}

// =============================================================================
// delayWithFace — replaces WebServer calls with vTaskDelay yields
// =============================================================================
void delayWithFace(unsigned long ms) {
#if USE_PCA9685
  flushServoFrame();
#endif
  unsigned long start = millis();
  while ((unsigned long)(millis() - start) < ms) {
    updateAnimatedFace();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// =============================================================================
// pressingCheck — thread-safe version using cmd_mutex
// =============================================================================
bool pressingCheck(const String& cmd, int ms) {
#if USE_PCA9685
  flushServoFrame();
#endif
  unsigned long start = millis();
  while ((unsigned long)(millis() - start) < (unsigned long)ms) {
    updateAnimatedFace();
    String current = getCommand();
    if (current != cmd) {
      runStandPose(1);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return (getCommand() == cmd);
}

// =============================================================================
// NVS configuration helpers
// =============================================================================
void loadConfigFromNVS() {
  Preferences prefs;
  // Reuse "wifi" namespace for WiFi credentials (compatible with main firmware)
  prefs.begin("wifi", true);
  if (prefs.isKey("ssid")) {
    String s = prefs.getString("ssid", "");
    if (s.length() > 0 && s.length() < (int)sizeof(g_wifi_ssid)) {
      s.toCharArray(g_wifi_ssid, sizeof(g_wifi_ssid));
    }
    String p = prefs.getString("pass", "");
    if (p.length() < (int)sizeof(g_wifi_pass)) {
      p.toCharArray(g_wifi_pass, sizeof(g_wifi_pass));
    }
  }
  prefs.end();

  // Agent configuration stored in "ros" namespace
  prefs.begin("ros", true);
  if (prefs.isKey("agent_ip")) {
    String ip = prefs.getString("agent_ip", DEFAULT_AGENT_IP);
    ip.toCharArray(g_agent_ip, sizeof(g_agent_ip));
    g_agent_port = (uint16_t)prefs.getUInt("agent_port", DEFAULT_AGENT_PORT);
  }
  // Servo calibration
  prefs.end();

  Preferences cal;
  cal.begin("servocal", true);
  if (cal.getBytesLength("minUs") == 16) {
    cal.getBytes("minUs", servoMinUs, 16);
    cal.getBytes("maxUs", servoMaxUs, 16);
  }
  if (cal.getBytesLength("subtrim") == 8) {
    cal.getBytes("subtrim", servoSubtrim, 8);
  }
  if (cal.getBytesLength("angMin") == 8) {
    cal.getBytes("angMin", servoMinAngle, 8);
    cal.getBytes("angMax", servoMaxAngle, 8);
  }
  cal.end();
}

void saveConfigToNVS() {
  Preferences prefs;
  prefs.begin("ros", false);
  prefs.putString("agent_ip",   g_agent_ip);
  prefs.putUInt("agent_port",   g_agent_port);
  prefs.end();

  prefs.begin("wifi", false);
  prefs.putString("ssid",    g_wifi_ssid);
  prefs.putString("pass",    g_wifi_pass);
  prefs.putBool("enabled",   strlen(g_wifi_ssid) > 0);
  prefs.end();
}

// =============================================================================
// Camera initialisation (reused from camera-stream.h logic)
// =============================================================================
#if HAS_CAMERA
bool initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = CAM_PIN_D0;
  cfg.pin_d1       = CAM_PIN_D1;
  cfg.pin_d2       = CAM_PIN_D2;
  cfg.pin_d3       = CAM_PIN_D3;
  cfg.pin_d4       = CAM_PIN_D4;
  cfg.pin_d5       = CAM_PIN_D5;
  cfg.pin_d6       = CAM_PIN_D6;
  cfg.pin_d7       = CAM_PIN_D7;
  cfg.pin_xclk     = CAM_PIN_XCLK;
  cfg.pin_pclk     = CAM_PIN_PCLK;
  cfg.pin_vsync    = CAM_PIN_VSYNC;
  cfg.pin_href     = CAM_PIN_HREF;
  cfg.pin_sccb_sda = CAM_PIN_SIOD;
  cfg.pin_sccb_scl = CAM_PIN_SIOC;
  cfg.pin_pwdn     = CAM_PIN_PWDN;
  cfg.pin_reset    = CAM_PIN_RESET;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    cfg.frame_size   = CAM_FRAMESIZE;
    cfg.jpeg_quality = CAM_JPEG_QUALITY;
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    cfg.frame_size   = FRAMESIZE_QQVGA;
    cfg.jpeg_quality = 25;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    DBG_PRINTF("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }

  DBG_PRINTLN("Camera initialised");
  cameraInitialised = true;
  return true;
}

// Camera task — runs on Core 0 alongside the WiFi stack.
// Grabs JPEG frames and copies them into the shared g_cam_buf.
static void cameraTask(void* arg) {
  (void)arg;

  for (;;) {
    if (!cameraInitialised) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (fb->len > 0 && fb->len <= MAX_JPEG_SIZE && g_cam_buf) {
      if (xSemaphoreTake(cam_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        memcpy(g_cam_buf, fb->buf, fb->len);
        g_cam_size  = fb->len;
        g_cam_ready = true;
        xSemaphoreGive(cam_mutex);
      }
    }

    esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(CAM_PUBLISH_INTERVAL_MS));
  }
}
#endif // HAS_CAMERA

// =============================================================================
// micro-ROS subscriber callbacks
// =============================================================================

// /sesame/cmd — called in ros_spin_task context
static void cmdCallback(const void* msgin) {
  const std_msgs__msg__String* msg =
      (const std_msgs__msg__String*)msgin;
  if (!msg || !msg->data.data) return;

  String cmd = String(msg->data.data);
  DBG_PRINT("ROS cmd: ");
  DBG_PRINTLN(cmd);

  if (cmd == "stop") {
    setCommand("");
  } else {
    setCommand(cmd);
  }
}

// /sesame/pose — called in ros_spin_task context
static void poseCallback(const void* msgin) {
  const geometry_msgs__msg__PoseStamped* msg =
      (const geometry_msgs__msg__PoseStamped*)msgin;
  if (!msg) return;

  g_robot_pose.x  = (float)msg->pose.position.x;
  g_robot_pose.y  = (float)msg->pose.position.y;
  g_robot_pose.z  = (float)msg->pose.position.z;
  g_robot_pose.qx = (float)msg->pose.orientation.x;
  g_robot_pose.qy = (float)msg->pose.orientation.y;
  g_robot_pose.qz = (float)msg->pose.orientation.z;
  g_robot_pose.qw = (float)msg->pose.orientation.w;
  g_robot_pose.valid = true;
}

// =============================================================================
// micro-ROS entity lifecycle
// =============================================================================

bool rosCreateEntities() {
  allocator = rcl_get_default_allocator();

  rcl_ret_t rc = rclc_support_init(&support, 0, nullptr, &allocator);
  if (rc != RCL_RET_OK) return false;

  rc = rclc_node_init_default(&node, "sesame_robot", "", &support);
  if (rc != RCL_RET_OK) return false;

  // ── Camera publisher ─────────────────────────────────────────────────────
  rc = rclc_publisher_init_best_effort(
      &cam_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
      "/camera/image_raw");
  if (rc != RCL_RET_OK) return false;

  // ── Status publisher ──────────────────────────────────────────────────────
  rc = rclc_publisher_init_best_effort(
      &status_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/sesame/status");
  if (rc != RCL_RET_OK) return false;

  // ── Command subscriber ────────────────────────────────────────────────────
  rc = rclc_subscription_init_default(
      &cmd_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/sesame/cmd");
  if (rc != RCL_RET_OK) return false;

  // ── Pose subscriber ───────────────────────────────────────────────────────
  rc = rclc_subscription_init_default(
      &pose_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
      "/sesame/pose");
  if (rc != RCL_RET_OK) return false;

  // ── Executor (2 subscriptions) ────────────────────────────────────────────
  rc = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (rc != RCL_RET_OK) return false;

  RCCHECK(rclc_executor_add_subscription(
      &executor, &cmd_subscriber, &cmd_msg,
      &cmdCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &pose_subscriber, &pose_msg,
      &poseCallback, ON_NEW_DATA));

  DBG_PRINTLN("micro-ROS entities created");
  return true;
}

void rosDestroyEntities() {
  rcl_publisher_fini(&cam_publisher,    &node);
  rcl_publisher_fini(&status_publisher, &node);
  rcl_subscription_fini(&cmd_subscriber,  &node);
  rcl_subscription_fini(&pose_subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  DBG_PRINTLN("micro-ROS entities destroyed");
}

// Publish /sesame/status as a JSON string
void publishStatus() {
  int n = snprintf(status_data_buf, sizeof(status_data_buf),
      "{\"cmd\":\"%s\",\"face\":\"%s\","
      "\"uptime\":%lu,\"ip\":\"%s\","
      "\"pose_valid\":%s}",
      currentCommand.c_str(),
      currentFaceName.c_str(),
      millis() / 1000UL,
      WiFi.localIP().toString().c_str(),
      g_robot_pose.valid ? "true" : "false");
  status_msg.data.size = (size_t)n;
  rcl_publish(&status_publisher, &status_msg, nullptr);
}

// =============================================================================
// ros_spin_task — micro-ROS executor + publishing (Core 1, high priority)
// =============================================================================
static void rosSpinTask(void* arg) {
  (void)arg;

  // Initialise static message memory
  cam_msg.data.data     = cam_msg_data_buf;
  cam_msg.data.size     = 0;
  cam_msg.data.capacity = MAX_JPEG_SIZE;

  cam_msg.format.data     = cam_msg_format;
  cam_msg.format.size     = strlen(cam_msg_format);
  cam_msg.format.capacity = sizeof(cam_msg_format);

  cam_msg.header.frame_id.data     = cam_msg_frame_id;
  cam_msg.header.frame_id.size     = strlen(cam_msg_frame_id);
  cam_msg.header.frame_id.capacity = sizeof(cam_msg_frame_id);

  status_msg.data.data     = status_data_buf;
  status_msg.data.size     = 0;
  status_msg.data.capacity = sizeof(status_data_buf);

  cmd_msg.data.data     = cmd_data_buf;
  cmd_msg.data.size     = 0;
  cmd_msg.data.capacity = sizeof(cmd_data_buf);

  pose_msg.header.frame_id.data     = pose_frame_id_buf;
  pose_msg.header.frame_id.size     = strlen(pose_frame_id_buf);
  pose_msg.header.frame_id.capacity = sizeof(pose_frame_id_buf);

  unsigned long lastStatusMs = 0;

  for (;;) {
    switch (ros_state) {

      case ROS_WAITING_AGENT:
        if (rmw_uros_ping_agent(200, 1) == RMW_RET_OK) {
          ros_state = ROS_AGENT_AVAILABLE;
          oledShowStatus("ROS agent found", g_agent_ip);
          DBG_PRINTLN("micro-ROS agent found");
        } else {
          vTaskDelay(pdMS_TO_TICKS(500));
        }
        break;

      case ROS_AGENT_AVAILABLE:
        if (rosCreateEntities()) {
          ros_state = ROS_AGENT_CONNECTED;
          oledShowStatus("ROS connected", WiFi.localIP().toString().c_str());
          DBG_PRINTLN("micro-ROS connected");
        } else {
          rosDestroyEntities();
          ros_state = ROS_WAITING_AGENT;
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        break;

      case ROS_AGENT_CONNECTED:
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          DBG_PRINTLN("micro-ROS agent lost");
          rosDestroyEntities();
          ros_state = ROS_AGENT_DISCONNECTED;
          setCommand("");
          oledShowStatus("ROS agent lost", "Reconnecting...");
          break;
        }

        // Spin the executor (process incoming messages)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Publish camera frame if one is ready
#if HAS_CAMERA
        if (g_cam_ready && g_cam_buf) {
          if (xSemaphoreTake(cam_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (g_cam_size <= MAX_JPEG_SIZE) {
              memcpy(cam_msg_data_buf, g_cam_buf, g_cam_size);
              cam_msg.data.size = g_cam_size;
              g_cam_ready = false;
            }
            xSemaphoreGive(cam_mutex);

            // Set timestamp (seconds + nanoseconds from millis)
            uint64_t now_ms = (uint64_t)millis();
            cam_msg.header.stamp.sec     = (int32_t)(now_ms / 1000ULL);
            cam_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000ULL) * 1000000ULL);
            rcl_publish(&cam_publisher, &cam_msg, nullptr);
          }
        }
#endif

        // Publish status periodically
        if (millis() - lastStatusMs >= STATUS_PUBLISH_INTERVAL_MS) {
          publishStatus();
          lastStatusMs = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
        break;

      case ROS_AGENT_DISCONNECTED:
        rosDestroyEntities();
        ros_state = ROS_WAITING_AGENT;
        vTaskDelay(pdMS_TO_TICKS(1000));
        break;
    }
  }
}

// =============================================================================
// movement_task — servo poses + OLED animation (Core 1, low priority)
// =============================================================================
static void movementTask(void* arg) {
  (void)arg;

  // Startup rest pose
  setFace("rest");

  for (;;) {
    updateAnimatedFace();
    updateIdleBlink();
#if USE_PCA9685
    flushServoFrame();
#if ENABLE_SERVO_AUTO_RELEASE
    maybeAutoReleaseServos();
#endif
#endif

    String cmd = getCommand();

    if (cmd != "") {
      if      (cmd == "forward")  runWalkPose();
      else if (cmd == "backward") runWalkBackward();
      else if (cmd == "left")     runTurnLeft();
      else if (cmd == "right")    runTurnRight();
      else if (cmd == "rest")     { runRestPose();  if (getCommand() == "rest")  setCommand(""); }
      else if (cmd == "stand")    { runStandPose(1);if (getCommand() == "stand") setCommand(""); }
      else if (cmd == "wave")     runWavePose();
      else if (cmd == "dance")    runDancePose();
      else if (cmd == "swim")     runSwimPose();
      else if (cmd == "point")    runPointPose();
      else if (cmd == "pushup")   runPushupPose();
      else if (cmd == "bow")      runBowPose();
      else if (cmd == "cute")     runCutePose();
      else if (cmd == "freaky")   runFreakyPose();
      else if (cmd == "worm")     runWormPose();
      else if (cmd == "shake")    runShakePose();
      else if (cmd == "shrug")    runShrugPose();
      else if (cmd == "dead")     runDeadPose();
      else if (cmd == "crab")     runCrabPose();
      else {
        // Unknown command — clear it
        setCommand("");
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

// =============================================================================
// Serial configuration CLI
// =============================================================================
#if ENABLE_SERIAL
static void handleSerialCLI() {
  static char buf[128];
  static int  pos = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (pos == 0) continue;
      buf[pos] = '\0';
      pos = 0;

      // Parse "set <key> <value>"
      if (strncmp(buf, "set ssid ", 9) == 0) {
        strncpy(g_wifi_ssid, buf + 9, sizeof(g_wifi_ssid) - 1);
        Serial.println("SSID set");
      } else if (strncmp(buf, "set pass ", 9) == 0) {
        strncpy(g_wifi_pass, buf + 9, sizeof(g_wifi_pass) - 1);
        Serial.println("Password set");
      } else if (strncmp(buf, "set agent ", 10) == 0) {
        strncpy(g_agent_ip, buf + 10, sizeof(g_agent_ip) - 1);
        Serial.println("Agent IP set");
      } else if (strncmp(buf, "set port ", 9) == 0) {
        g_agent_port = (uint16_t)atoi(buf + 9);
        Serial.println("Agent port set");
      } else if (strcmp(buf, "save") == 0) {
        saveConfigToNVS();
        Serial.println("Config saved. Reboot to apply.");
      } else if (strcmp(buf, "status") == 0) {
        Serial.printf("SSID:  %s\nAgent: %s:%u\nWiFi:  %s\nROS:   %d\n",
            g_wifi_ssid, g_agent_ip, g_agent_port,
            WiFi.localIP().toString().c_str(),
            (int)ros_state);
      } else if (strcmp(buf, "reboot") == 0) {
        ESP.restart();
      } else {
        Serial.println("Commands: set ssid/pass/agent/port <val>, save, status, reboot");
      }
    } else if (pos < (int)sizeof(buf) - 1) {
      buf[pos++] = c;
    }
  }
}
#endif // ENABLE_SERIAL

// =============================================================================
// setup()
// =============================================================================
void setup() {
  // Reduce brownout threshold to survive servo inrush current
  REG_SET_FIELD(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_DBROWN_OUT_THRES, 7);

  DBG_BEGIN(115200);
  randomSeed(micros());

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

#if defined(CAM_PIN_LED) && CAM_PIN_LED >= 0
  pinMode(CAM_PIN_LED, OUTPUT);
  digitalWrite(CAM_PIN_LED, LOW);
#endif

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    oledAvailable = false;
  } else {
    oledAvailable = true;
    oledShowStatus("Sesame ROS", "Starting...");
  }

  // Load NVS config
  loadConfigFromNVS();

  DBG_PRINTF("Connecting to %s\n", g_wifi_ssid);
  oledShowStatus("Connecting WiFi", g_wifi_ssid);

  // WiFi station mode
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("sesame-robot");
  WiFi.begin(g_wifi_ssid, g_wifi_pass);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    DBG_PRINT(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    DBG_PRINTLN("\nWiFi connection failed. Check credentials (serial: set ssid/pass, save).");
    oledShowStatus("WiFi failed!", "Check serial cfg");
    // Keep retrying in the background; micro-ROS spin loop will wait for agent
  } else {
    DBG_PRINTF("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    oledShowStatus("WiFi connected", WiFi.localIP().toString().c_str());
  }

  // Configure micro-ROS UDP transport
  IPAddress agent;
  if (!agent.fromString(g_agent_ip)) {
    DBG_PRINTLN("Invalid agent IP — using default");
    agent.fromString(DEFAULT_AGENT_IP);
  }
  set_microros_udp_transports(agent, g_agent_port);
  DBG_PRINTF("micro-ROS agent: %s:%u\n", g_agent_ip, g_agent_port);

  // PCA9685 servo driver
#if USE_PCA9685
  pwmDriver.begin();
  pwmDriver.setOscillatorFrequency(27000000);
  pwmDriver.setPWMFreq(50);
  pwmDriver.setOutputMode(true);
#endif
  delay(10);

  // Camera
#if HAS_CAMERA
  cam_mutex = xSemaphoreCreateMutex();
  g_cam_buf = (uint8_t*)ps_malloc(MAX_JPEG_SIZE);
  if (!g_cam_buf) {
    DBG_PRINTLN("PSRAM alloc failed — trying heap");
    g_cam_buf = (uint8_t*)malloc(MAX_JPEG_SIZE);
  }
  if (initCamera()) {
    oledShowStatus("Camera ready", "");
  }
#endif

  // Command mutex
  cmd_mutex = xSemaphoreCreateMutex();

  // Initial face (no servo movement)
  setFace("rest");

  // Launch tasks
  //   camera_task: Core 0, priority 3
  //   ros_spin_task: Core 1, priority 5  (preempts movement_task on yields)
  //   movement_task: Core 1, priority 2

#if HAS_CAMERA
  xTaskCreatePinnedToCore(cameraTask,    "cam_task",  4096, nullptr, 3, nullptr, 0);
#endif
  xTaskCreatePinnedToCore(rosSpinTask,   "ros_task",  8192, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(movementTask,  "move_task", 8192, nullptr, 2, nullptr, 1);

  DBG_PRINTLN("Sesame micro-ROS firmware started");
}

// =============================================================================
// loop() — only used for the serial CLI when ENABLE_SERIAL = 1
// =============================================================================
void loop() {
#if ENABLE_SERIAL
  handleSerialCLI();
#endif
  vTaskDelay(pdMS_TO_TICKS(100));
}
