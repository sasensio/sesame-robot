// ── Board Selection ───────────────────────────────────────────────────────────
// Uncomment the line that matches your hardware before flashing.
// Leave all lines commented out to default to the Lolin S2 Mini.
//
// #define BOARD_ESP32CAM        // AI-Thinker ESP32-CAM  (uses PCA9685 servos)
// #define BOARD_ESP32S3CAM      // Generic ESP32-S3-CAM  (uses PCA9685 servos)
// #define BOARD_DISTRO_V2       // Sesame Distro Board V2 (ESP32-S3)
// #define BOARD_DISTRO_V1       // Sesame Distro Board V1 (ESP32-WROOM32)
// ─────────────────────────────────────────────────────────────────────────────

// Set to 0 to disable all Serial output (saves RAM/CPU when no UART connected)
#define ENABLE_SERIAL 0

// Set to 1 to automatically release PCA9685 servo outputs while resting.
// This reduces idle servo power draw by removing holding torque.
#define ENABLE_SERVO_AUTO_RELEASE 1

#if ENABLE_SERIAL
  #define DBG_BEGIN(baud) Serial.begin(baud)
  #define DBG_PRINT(...)  Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN(baud) do {} while(0)
  #define DBG_PRINT(...)  do {} while(0)
  #define DBG_PRINTLN(...) do {} while(0)
  #define DBG_PRINTF(...) do {} while(0)
#endif

#include "hal/hal.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Preferences.h>
#if USE_PCA9685
  #include <Adafruit_PWMServoDriver.h>
#else
  #include <ESP32Servo.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "face-bitmaps.h"
#include "movement-sequences.h"
#include "captive-portal.h"
#include "camera-stream.h"

// --- Access Point Configuration ---
// This is the network the Robot will create
#define AP_SSID  "Sesame-Controller"
#define AP_PASS  "12345678" // Must be at least 8 characters

// --- Station Mode Configuration ---
// Stored in NVS, configurable from the web UI Settings panel.
// These defaults are used on first boot (before any NVS data exists).
String staSSID = "";
String staPass = "";
bool   staEnabled = false;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDR 0x3C

// DNS Server for Captive Portal
DNSServer dnsServer;
const byte DNS_PORT = 53;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledAvailable = false;
WebServer server(80);

// Global state for animations
String currentCommand = "";
String currentFaceName = "default";
const unsigned char* const* currentFaceFrames = nullptr;
uint8_t currentFaceFrameCount = 0;
uint8_t currentFaceFrameIndex = 0;
unsigned long lastFaceFrameMs = 0;
int faceFps = 8;
FaceAnimMode currentFaceMode = FACE_ANIM_LOOP;
int8_t faceFrameDirection = 1;
bool faceAnimFinished = false;
int currentFaceFps = 0;
bool idleActive = false;
bool idleBlinkActive = false;
unsigned long nextIdleBlinkMs = 0;
uint8_t idleBlinkRepeatsLeft = 0;

// WiFi Info Scrolling
unsigned long lastInputTime = 0;
bool firstInputReceived = false;
bool showingWifiInfo = false;
int wifiScrollPos = 0;
unsigned long lastWifiScrollMs = 0;
String wifiInfoText = "";

// Network Mode
bool networkConnected = false;
IPAddress networkIP;
String deviceHostname = "sesame-robot";

// Camera LED state (used only when CAM_PIN_LED is available on a board).
bool cameraLedOn = false;

#if HAS_CAMERA
// Keep camera at active frame interval briefly after movement stops.
static unsigned long cameraFastUntilMs = 0;
static const unsigned long CAMERA_POST_MOVE_FAST_MS = 30000;
#endif

// ── Servo / PCA9685 ──────────────────────────────────────────────────────────
// Pin numbers correspond to ESP32 GPIO pins and differ between boards.
// For PCA9685 boards (ESP32-CAM, ESP32-S3-CAM) the driver is initialised in
// setup(); servo channels 0-7 map directly to PCA9685 outputs 0-7.
// For direct-PWM boards the servoPins array selects the GPIO for each channel.
// ─────────────────────────────────────────────────────────────────────────────
#if USE_PCA9685
  Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);
#else
  Servo servos[8];
  // Sesame Distro Board V2 Pinout
  //const int servoPins[8] = {4, 5, 6, 7, 15, 16, 17, 18};

  // Sesame Distro Board Pinout
  //const int servoPins[8] = {15, 2, 23, 19, 4, 16, 17, 18};

  // Lolin S2 Mini Pinout
  const int servoPins[8] = {1, 2, 4, 6, 8, 10, 13, 14};
#endif

// Subtrim values for each servo (offset in degrees)
int8_t servoSubtrim[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Per-channel calibration limits (microseconds).
// Adjust these per-servo to define the pulse width endpoints.
// angle 0° → servoMinUs[ch], angle 180° → servoMaxUs[ch].
//                             R1    R2    L1    L2    R4    R3    L3    L4
uint16_t servoMinUs[8]   = { 500,  500,  500,  500,  500,  500,  500,  500};
uint16_t servoMaxUs[8]   = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};

// Per-channel angle limits (degrees) — collision avoidance.
// Commanded angles are clamped to this range before µs mapping.
//                             R1   R2   L1   L2   R4   R3   L3   L4
uint8_t servoMinAngle[8] = {  0,   0,   0,   0,   0,   0,   0,   0};
uint8_t servoMaxAngle[8] = {180, 180, 180, 180, 180, 180, 180, 180};

// Current servo positions (degrees) — updated by setFrame/setServoAngle
int16_t servoFrameBuf[8] = {90,90,90,90,90,90,90,90};

// Animation constants
int frameDelay = 100;
int walkCycles = 10;
int motorCurrentDelay = 20; // ms delay between motor movements to prevent over-current

struct FaceEntry {
  const char* name;
  const unsigned char* const* frames;
  uint8_t maxFrames;
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

struct FaceFpsEntry {
  const char* name;
  uint8_t fps;
};

const FaceFpsEntry faceFpsEntries[] = {
  { "walk", 1 },
  { "rest", 1 },
  { "swim", 1 },
  { "dance", 1 },
  { "wave", 1 },
  { "point", 5 },
  { "stand", 1 },
  { "cute", 1 },
  { "pushup", 1 },
  { "freaky", 1 },
  { "bow", 1 },
  { "worm", 1 },
  { "shake", 1 },
  { "shrug", 1 },
  { "dead", 2 },
  { "crab", 1 },
  { "idle", 1 },
  { "idle_blink", 7 },
  { "default", 1 },
  // Conversational faces (manually controlled by Python - no auto-animation)
  { "happy", 1 },
  { "talk_happy", 1 },
  { "sad", 1 },
  { "talk_sad", 1 },
  { "angry", 1 },
  { "talk_angry", 1 },
  { "surprised", 1 },
  { "talk_surprised", 1 },
  { "sleepy", 1 },
  { "talk_sleepy", 1 },
  { "love", 1 },
  { "talk_love", 1 },
  { "excited", 1 },
  { "talk_excited", 1 },
  { "confused", 1 },
  { "talk_confused", 1 },
  { "thinking", 1 },
  { "talk_thinking", 1 },
};


// Prototypes
void setServoAngle(uint8_t channel, int angle);
void updateFaceBitmap(const unsigned char* bitmap);
void setFace(const String& faceName);
void setFaceMode(FaceAnimMode mode);
void setFaceWithMode(const String& faceName, FaceAnimMode mode);
void updateAnimatedFace();
void delayWithFace(unsigned long ms);
void enterIdle();
void exitIdle();
void updateIdleBlink();
int getFaceFpsForName(const String& faceName);
bool pressingCheck(const String& cmd, int ms);
#if USE_PCA9685 && ENABLE_SERVO_AUTO_RELEASE
void maybeAutoReleaseServos();
#endif
void handleGetSettings();
void handleSetSettings();
void handleGetStatus();
void handleApiCommand();
void updateWifiInfoScroll();
void recordInput();

bool cameraLedAvailable() {
#if defined(CAM_PIN_LED) && CAM_PIN_LED >= 0
  return true;
#else
  return false;
#endif
}

void applyCameraLed(bool on) {
#if defined(CAM_PIN_LED) && CAM_PIN_LED >= 0
  digitalWrite(CAM_PIN_LED, on ? HIGH : LOW);
#endif
}

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleCommandWeb() {
  // We send 200 OK immediately so the web browser doesn't hang waiting for animation to finish
  if (server.hasArg("pose")) {
    currentCommand = server.arg("pose");
    recordInput();
    exitIdle();
    server.send(200, "text/plain", "OK"); 
  } 
  else if (server.hasArg("go")) {
    currentCommand = server.arg("go");
    recordInput();
    exitIdle();
    server.send(200, "text/plain", "OK");
  } 
  else if (server.hasArg("stop")) {
    currentCommand = "";
    recordInput();
    server.send(200, "text/plain", "OK");
  }
  else if (server.hasArg("reboot")) {
    server.send(200, "text/plain", "Rebooting...");
    delay(200);
    ESP.restart();
  }
  else if (server.hasArg("motor") && server.hasArg("value")) {
    int motorNum = server.arg("motor").toInt();
    int servoIdx = servoNameToIndex(server.arg("motor"));
    int angle = server.arg("value").toInt();
    if (motorNum >= 1 && motorNum <= 8 && angle >= 0 && angle <= 180) {
      setServoAngle(motorNum - 1, angle); // Convert 1-based to 0-based index
      recordInput();
      server.send(200, "text/plain", "OK");
    } else if (servoIdx != -1 && angle >= 0 && angle <= 180) {
      setServoAngle(servoIdx, angle);
      recordInput();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid motor or angle");
    }
  }
  else {
    server.send(400, "text/plain", "Bad Args");
  }
}

void handleGetSettings() {
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{\"frameDelay\":%d,\"walkCycles\":%d,\"motorCurrentDelay\":%d,"
    "\"faceFps\":%d,\"staSSID\":\"%s\",\"staEnabled\":%s,\"networkConnected\":%s,"
    "\"cameraEnabled\":%s",
    frameDelay, walkCycles, motorCurrentDelay, faceFps,
    staSSID.c_str(), staEnabled ? "true" : "false", networkConnected ? "true" : "false",
#if HAS_CAMERA
    cameraEnabled ? "true" : "false"
#else
    "false"
#endif
    );
  if (networkConnected) {
    n += snprintf(buf + n, sizeof(buf) - n, ",\"networkIP\":\"%s\"", networkIP.toString().c_str());
  }
  n += snprintf(buf + n, sizeof(buf) - n,
    ",\"cameraLed\":%s,\"cameraLedAvailable\":%s",
    cameraLedOn ? "true" : "false",
    cameraLedAvailable() ? "true" : "false");
  snprintf(buf + n, sizeof(buf) - n, "}");
  server.send(200, "application/json", buf);
}

void handleSetSettings() {
  if (server.hasArg("frameDelay")) frameDelay = server.arg("frameDelay").toInt();
  if (server.hasArg("walkCycles")) walkCycles = server.arg("walkCycles").toInt();
  if (server.hasArg("motorCurrentDelay")) motorCurrentDelay = server.arg("motorCurrentDelay").toInt();
  if (server.hasArg("faceFps")) faceFps = (int)max(1L, server.arg("faceFps").toInt());
#if HAS_CAMERA
  if (server.hasArg("camera")) {
    cameraEnabled = (server.arg("camera") == "1");
    // Persist camera state to NVS
    Preferences prefs;
    prefs.begin("wifi", false);
    prefs.putBool("camOn", cameraEnabled);
    prefs.end();
  }
#endif
  if (server.hasArg("camLed")) {
    cameraLedOn = (server.arg("camLed") == "1");
    applyCameraLed(cameraLedOn);
  }
  server.send(200, "text/plain", "OK");
}

void handleSetWifi() {
  if (server.hasArg("ssid")) staSSID = server.arg("ssid");
  if (server.hasArg("pass")) staPass = server.arg("pass");
  staEnabled = (staSSID.length() > 0);
  // Save to NVS
  Preferences prefs;
  prefs.begin("wifi", false);
  prefs.putString("ssid", staSSID);
  prefs.putString("pass", staPass);
  prefs.putBool("enabled", staEnabled);
  prefs.end();
  server.send(200, "text/plain", "OK — reboot to connect");
}

void loadWifiFromNVS() {
  Preferences prefs;
  prefs.begin("wifi", true);
  if (prefs.isKey("ssid")) {
    staSSID   = prefs.getString("ssid", "");
    staPass   = prefs.getString("pass", "");
    staEnabled = prefs.getBool("enabled", false);
  }
#if HAS_CAMERA
  cameraEnabled = prefs.getBool("camOn", true);
#endif
  prefs.end();
}

void handleGetCalibration() {
  char buf[320];
  int n = snprintf(buf, sizeof(buf), "{\"min\":[%u,%u,%u,%u,%u,%u,%u,%u],"
    "\"max\":[%u,%u,%u,%u,%u,%u,%u,%u],"
    "\"angleMin\":[%u,%u,%u,%u,%u,%u,%u,%u],"
    "\"angleMax\":[%u,%u,%u,%u,%u,%u,%u,%u],"
    "\"pos\":[%d,%d,%d,%d,%d,%d,%d,%d]}",
    servoMinUs[0],servoMinUs[1],servoMinUs[2],servoMinUs[3],servoMinUs[4],servoMinUs[5],servoMinUs[6],servoMinUs[7],
    servoMaxUs[0],servoMaxUs[1],servoMaxUs[2],servoMaxUs[3],servoMaxUs[4],servoMaxUs[5],servoMaxUs[6],servoMaxUs[7],
    servoMinAngle[0],servoMinAngle[1],servoMinAngle[2],servoMinAngle[3],servoMinAngle[4],servoMinAngle[5],servoMinAngle[6],servoMinAngle[7],
    servoMaxAngle[0],servoMaxAngle[1],servoMaxAngle[2],servoMaxAngle[3],servoMaxAngle[4],servoMaxAngle[5],servoMaxAngle[6],servoMaxAngle[7],
    servoFrameBuf[0],servoFrameBuf[1],servoFrameBuf[2],servoFrameBuf[3],servoFrameBuf[4],servoFrameBuf[5],servoFrameBuf[6],servoFrameBuf[7]);
  server.send(200, "application/json", buf);
}

void handleSetCalibration() {
  char key[8];
  for (int i = 0; i < 8; i++) {
    snprintf(key, sizeof(key), "min%d", i);
    if (server.hasArg(key)) {
      int v = server.arg(key).toInt();
      if (v >= 500 && v < 2500) servoMinUs[i] = v;
    }
    snprintf(key, sizeof(key), "max%d", i);
    if (server.hasArg(key)) {
      int v = server.arg(key).toInt();
      if (v > 500 && v <= 2500) servoMaxUs[i] = v;
    }
    snprintf(key, sizeof(key), "amin%d", i);
    if (server.hasArg(key)) {
      int v = server.arg(key).toInt();
      if (v >= 0 && v < 180) servoMinAngle[i] = v;
    }
    snprintf(key, sizeof(key), "amax%d", i);
    if (server.hasArg(key)) {
      int v = server.arg(key).toInt();
      if (v > 0 && v <= 180) servoMaxAngle[i] = v;
    }
  }
  server.send(200, "text/plain", "OK");
}

void saveCalToNVS() {
  Preferences prefs;
  prefs.begin("servocal", false);
  prefs.putBytes("minUs", servoMinUs, 16);
  prefs.putBytes("maxUs", servoMaxUs, 16);
  prefs.putBytes("subtrim", servoSubtrim, 8);
  prefs.putBytes("angMin", servoMinAngle, 8);
  prefs.putBytes("angMax", servoMaxAngle, 8);
  prefs.end();
}

void loadCalFromNVS() {
  Preferences prefs;
  prefs.begin("servocal", true);
  if (prefs.getBytesLength("minUs") == 16) {
    prefs.getBytes("minUs", servoMinUs, 16);
    prefs.getBytes("maxUs", servoMaxUs, 16);
  }
  if (prefs.getBytesLength("subtrim") == 8) {
    prefs.getBytes("subtrim", servoSubtrim, 8);
  }
  if (prefs.getBytesLength("angMin") == 8) {
    prefs.getBytes("angMin", servoMinAngle, 8);
    prefs.getBytes("angMax", servoMaxAngle, 8);
  }
  prefs.end();
}

void handleSaveCalibration() {
  saveCalToNVS();
  server.send(200, "text/plain", "OK");
}

void handleResetCalibration() {
  for (int i = 0; i < 8; i++) {
    servoMinUs[i] = 500;
    servoMaxUs[i] = 2500;
    servoMinAngle[i] = 0;
    servoMaxAngle[i] = 180;
    servoSubtrim[i] = 0;
  }
  saveCalToNVS();
  server.send(200, "text/plain", "Calibration reset to factory defaults");
}

void handleDebug() {
  char buf[512];
  int n = snprintf(buf, sizeof(buf), "{\"servos\":[");
  for (int i = 0; i < 8; i++) {
    n += snprintf(buf + n, sizeof(buf) - n,
      "%s{\"name\":\"%s\",\"ch\":%d,\"frameBuf\":%d,"
      "\"minUs\":%u,\"maxUs\":%u,\"minAngle\":%u,\"maxAngle\":%u,\"subtrim\":%d}",
      i ? "," : "", ServoNames[i], i, servoFrameBuf[i],
      servoMinUs[i], servoMaxUs[i], servoMinAngle[i], servoMaxAngle[i], servoSubtrim[i]);
  }
  snprintf(buf + n, sizeof(buf) - n, "]}");
  server.send(200, "application/json", buf);
}

// API endpoint for network clients to get robot status
void handleGetStatus() {
  char buf[384];
  int n = snprintf(buf, sizeof(buf),
    "{\"currentCommand\":\"%s\",\"currentFace\":\"%s\","
    "\"networkConnected\":%s,\"apIP\":\"%s\","
    "\"apClients\":%d,\"uptime\":%lu",
    currentCommand.c_str(), currentFaceName.c_str(),
    networkConnected ? "true" : "false", WiFi.softAPIP().toString().c_str(),
    WiFi.softAPgetStationNum(), millis() / 1000UL);
  if (networkConnected) {
    n += snprintf(buf + n, sizeof(buf) - n,
      ",\"networkIP\":\"%s\",\"rssi\":%d,\"ssid\":\"%s\"",
      networkIP.toString().c_str(), WiFi.RSSI(), staSSID.c_str());
  }
  snprintf(buf + n, sizeof(buf) - n, "}");
  server.send(200, "application/json", buf);
}

// API endpoint for network clients to send commands (JSON-based)
void handleApiCommand() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
    return;
  }
  
  String body = server.arg("plain");
  
  DBG_PRINTLN("API Command received:");
  DBG_PRINTLN(body);
  
  // Check for face-only command (no movement)
  int faceOnlyStart = body.indexOf("\"face\":\"");
  if (faceOnlyStart == -1) {
    faceOnlyStart = body.indexOf("\"face\": \"");
  }
  
  // If we have a face but no command field, it's face-only
  bool faceOnly = (faceOnlyStart > 0 && body.indexOf("\"command\":") == -1 && body.indexOf("\"command\": ") == -1);
  
  String command = "";
  String face = "";
  
  // Parse face
  if (faceOnlyStart > 0) {
    faceOnlyStart = body.indexOf("\"", faceOnlyStart + 6) + 1;
    int faceEnd = body.indexOf("\"", faceOnlyStart);
    if (faceEnd > faceOnlyStart) {
      face = body.substring(faceOnlyStart, faceEnd);
      DBG_PRINT("Parsed face: ");
      DBG_PRINTLN(face);
    }
  }
  
  // Parse command (if not face-only)
  if (!faceOnly) {
    int cmdStart = body.indexOf("\"command\":\"");
    if (cmdStart == -1) {
      cmdStart = body.indexOf("\"command\": \"");
    }
    
    if (cmdStart == -1) {
      DBG_PRINTLN("Error: command field not found");
      server.send(400, "application/json", "{\"error\":\"Missing command field\"}");
      return;
    }
    
    cmdStart = body.indexOf("\"", cmdStart + 10) + 1;
    int cmdEnd = body.indexOf("\"", cmdStart);
    
    if (cmdEnd <= cmdStart) {
      DBG_PRINTLN("Error: invalid command format");
      server.send(400, "application/json", "{\"error\":\"Invalid command format\"}");
      return;
    }
    
    command = body.substring(cmdStart, cmdEnd);
    DBG_PRINT("Parsed command: ");
    DBG_PRINTLN(command);
  }
  
  // Set face if provided
  if (face.length() > 0) {
    setFace(face);
  }
  
  // If face-only, just acknowledge
  if (faceOnly) {
    recordInput();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Face updated\"}");
    return;
  }
  
  // Execute command
  if (command == "stop") {
    currentCommand = "";
    recordInput();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Command stopped\"}");
  } else {
    currentCommand = command;
    recordInput();
    exitIdle();
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Command executed\"}");
  }
}

void setup() {
  // Lower brownout detector to least-sensitive threshold (2.43V) instead of default 2.80V.
  // Servo inrush current causes brief VCC dips that trigger the default level.
  REG_SET_FIELD(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_DBROWN_OUT_THRES, 7);

  DBG_BEGIN(115200);
  randomSeed(micros());
  
  // I2C Init for ESP32
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

#if defined(CAM_PIN_LED) && CAM_PIN_LED >= 0
  pinMode(CAM_PIN_LED, OUTPUT);
  applyCameraLed(false);
#endif

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    DBG_PRINTLN(F("SSD1306 not found — OLED disabled."));
    oledAvailable = false;
  } else {
    oledAvailable = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println(F("Setting up WiFi..."));
    display.display();
  }

  // --- WIFI CONFIGURATION ---
  loadWifiFromNVS();

  // Try to connect to saved network if configured
  if (staEnabled && staSSID.length() > 0) {
    DBG_PRINTLN("Attempting to connect to network: " + staSSID);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setHostname(deviceHostname.c_str());
    WiFi.begin(staSSID.c_str(), staPass.c_str());
    
    // Wait up to 10 seconds for connection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      DBG_PRINT(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      networkConnected = true;
      networkIP = WiFi.localIP();
      DBG_PRINTLN();
      DBG_PRINT("Connected to network! IP: ");
      DBG_PRINTLN(networkIP);
    } else {
      DBG_PRINTLN();
      DBG_PRINTLN("Failed to connect to network. Running in AP-only mode.");
      WiFi.mode(WIFI_AP);
    }
  } else {
    WiFi.mode(WIFI_AP);
    DBG_PRINTLN("Network mode disabled. Running in AP-only mode.");
  }
  
  // --- ACCESS POINT CONFIGURATION ---
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress myIP = WiFi.softAPIP();
  
  DBG_PRINT("AP Created. IP: ");
  DBG_PRINTLN(myIP);

  // Build WiFi info text for scrolling
  if (networkConnected) {
    wifiInfoText = "AP: " + String(AP_SSID) + " (" + myIP.toString() + ")  |  Network: " + staSSID + " (" + networkIP.toString() + ") or " + deviceHostname + ".local  |  ";
  } else {
    wifiInfoText = "Connect to WiFi: " + String(AP_SSID) + "  |  Pass: " + String(AP_PASS) + "  |  IP: " + myIP.toString() + "  |  Captive Portal will auto-open!  |  ";
  }
  
  // Initialize input tracking
  lastInputTime = millis();
  firstInputReceived = false;
  showingWifiInfo = false;

  // Start mDNS responder for local network discovery
  if (MDNS.begin(deviceHostname.c_str())) {
    DBG_PRINTLN("mDNS responder started");
    DBG_PRINT("Access controller at: http://");
    DBG_PRINT(deviceHostname);
    DBG_PRINTLN(".local");
    MDNS.addService("http", "tcp", 80);
  } else {
    DBG_PRINTLN("Error setting up mDNS responder!");
  }

  // Start DNS Server for Captive Portal
  // This redirects ALL domain requests to the ESP32's IP
  dnsServer.start(DNS_PORT, "*", myIP);

  // Web Server Routes
  server.on("/", handleRoot);
  server.on("/cmd", handleCommandWeb);
  server.on("/getSettings", handleGetSettings);
  server.on("/setSettings", handleSetSettings);
  server.on("/getCal", handleGetCalibration);
  server.on("/setCal", handleSetCalibration);
  server.on("/saveCal", handleSaveCalibration);
  server.on("/resetCal", handleResetCalibration);
  server.on("/debug", handleDebug);
  server.on("/setWifi", handleSetWifi);
  
  // API endpoints for network communication
  server.on("/api/status", handleGetStatus);
  server.on("/api/command", handleApiCommand);
  
  // Catch-all route for captive portal
  // This ensures any URL redirects to the controller page
  server.onNotFound(handleRoot);
  
  server.begin();

  // --- OTA Setup ---
  ArduinoOTA.setHostname(deviceHostname.c_str());
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    DBG_PRINTLN("OTA Start: " + type);
    if (oledAvailable) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("OTA Update..."));
      display.display();
    }
  });
  ArduinoOTA.onEnd([]() {
    DBG_PRINTLN("\nOTA End");
    if (oledAvailable) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("OTA Done!"));
      display.println(F("Rebooting..."));
      display.display();
    }
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int pct = progress / (total / 100);
    DBG_PRINTF("OTA Progress: %u%%\r", pct);
    if (oledAvailable) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("OTA Update..."));
      display.setCursor(0, 20);
      display.printf("%u%%", pct);
      display.drawRect(0, 40, 128, 12, SSD1306_WHITE);
      display.fillRect(2, 42, (124 * pct) / 100, 8, SSD1306_WHITE);
      display.display();
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DBG_PRINTF("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) DBG_PRINTLN("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) DBG_PRINTLN("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) DBG_PRINTLN("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) DBG_PRINTLN("Receive Failed");
    else if (error == OTA_END_ERROR) DBG_PRINTLN("End Failed");
  });
  ArduinoOTA.begin();
  DBG_PRINTLN("OTA ready");

  // Load saved calibration from NVS
  loadCalFromNVS();

  // PWM / Servo Init
#if USE_PCA9685
  pwmDriver.begin();
  // Use the measured oscillator frequency for accurate pulse widths.
  // 27 MHz is a typical value for the PCA9685 on-board oscillator;
  // calibrate with an oscilloscope if precise timing is required.
  pwmDriver.setOscillatorFrequency(27000000);
  pwmDriver.setPWMFreq(50);  // 50 Hz servo standard
  pwmDriver.setOutputMode(true);  // Totem-pole output for servos
#else
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  for (int i = 0; i < 8; i++) {
    servos[i].setPeriodHertz(50);
    // Map 0-180 to approx 732-2929us
    servos[i].attach(servoPins[i], 732, 2929);
  }
#endif
  delay(10);
  
  // Show rest face on startup without moving motors
  setFace("rest");

  // Camera init and stream server (only on camera-equipped boards)
#if HAS_CAMERA
  if (initCamera()) {
    startCameraStream();
  }
#endif
  
  DBG_PRINTLN(F("HTTP server & Captive Portal started."));
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  // Process DNS requests for captive portal
  dnsServer.processNextRequest();
  
  server.handleClient();
  updateAnimatedFace();
  updateIdleBlink();
  updateWifiInfoScroll();
#if USE_PCA9685
  flushServoFrame();
#endif

  if (currentCommand != "") {
#if HAS_CAMERA
    cameraMoving = true;
    cameraFastUntilMs = millis() + CAMERA_POST_MOVE_FAST_MS;
#endif
    String cmd = currentCommand;
    if (cmd == "forward") runWalkPose();
    else if (cmd == "backward") runWalkBackward();
    else if (cmd == "left") runTurnLeft();
    else if (cmd == "right") runTurnRight();
    else if (cmd == "rest") { runRestPose(); if (currentCommand == "rest") currentCommand = ""; }
    else if (cmd == "stand") { runStandPose(1); if (currentCommand == "stand") currentCommand = ""; }
    else if (cmd == "wave") runWavePose();
    else if (cmd == "dance") runDancePose();
    else if (cmd == "swim") runSwimPose();
    else if (cmd == "point") runPointPose();
    else if (cmd == "pushup") runPushupPose();
    else if (cmd == "bow") runBowPose();
    else if (cmd == "cute") runCutePose();
    else if (cmd == "freaky") runFreakyPose();
    else if (cmd == "worm") runWormPose();
    else if (cmd == "shake") runShakePose();
    else if (cmd == "shrug") runShrugPose();
    else if (cmd == "dead") runDeadPose();
    else if (cmd == "crab") runCrabPose();
  }
#if HAS_CAMERA
  else {
    cameraMoving = ((int32_t)(cameraFastUntilMs - millis()) > 0);
  }
#endif

#if USE_PCA9685 && ENABLE_SERVO_AUTO_RELEASE
  maybeAutoReleaseServos();
#endif
  
  // Serial CLI for debugging (can be used to diagnose servo position issues and wiring)
#if ENABLE_SERIAL
  if (Serial.available()) {
    static char command_buffer[32];
    static byte buffer_pos = 0;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer_pos > 0) {
        command_buffer[buffer_pos] = '\0';
        int motorNum, angle;
        recordInput();
        if(strcmp(command_buffer, "run walk") == 0 || strcmp(command_buffer, "rn wf") == 0) { currentCommand = "forward"; runWalkPose(); currentCommand = ""; }
        else if(strcmp(command_buffer, "rn wb") == 0) { currentCommand = "backward"; runWalkBackward(); currentCommand = ""; }
        else if(strcmp(command_buffer, "rn tl") == 0) { currentCommand = "left"; runTurnLeft(); currentCommand = ""; }
        else if(strcmp(command_buffer, "rn tr") == 0) { currentCommand = "right"; runTurnRight(); currentCommand = ""; }
        else if(strcmp(command_buffer, "run rest") == 0 || strcmp(command_buffer, "rn rs") == 0) runRestPose();
        else if(strcmp(command_buffer, "run stand") == 0 || strcmp(command_buffer, "rn st") == 0) runStandPose(1);
        else if(strcmp(command_buffer, "rn wv") == 0) { currentCommand = "wave"; runWavePose(); }
        else if(strcmp(command_buffer, "rn dn") == 0) { currentCommand = "dance"; runDancePose(); }
        else if(strcmp(command_buffer, "rn sw") == 0) { currentCommand = "swim"; runSwimPose(); }
        else if(strcmp(command_buffer, "rn pt") == 0) { currentCommand = "point"; runPointPose(); }
        else if(strcmp(command_buffer, "rn pu") == 0) { currentCommand = "pushup"; runPushupPose(); }
        else if(strcmp(command_buffer, "rn bw") == 0) { currentCommand = "bow"; runBowPose(); }
        else if(strcmp(command_buffer, "rn ct") == 0) { currentCommand = "cute"; runCutePose(); }
        else if(strcmp(command_buffer, "rn fk") == 0) { currentCommand = "freaky"; runFreakyPose(); }
        else if(strcmp(command_buffer, "rn wm") == 0) { currentCommand = "worm"; runWormPose(); }
        else if(strcmp(command_buffer, "rn sk") == 0) { currentCommand = "shake"; runShakePose(); }
        else if(strcmp(command_buffer, "rn sg") == 0) { currentCommand = "shrug"; runShrugPose(); }
        else if(strcmp(command_buffer, "rn dd") == 0) { currentCommand = "dead"; runDeadPose(); }
        else if(strcmp(command_buffer, "rn cb") == 0) { currentCommand = "crab"; runCrabPose(); }
        else if (strcmp(command_buffer, "subtrim") == 0 || strcmp(command_buffer, "st") == 0) {
          DBG_PRINTLN("Subtrim values:");
          for (int i = 0; i < 8; i++) {
            DBG_PRINT("Motor "); DBG_PRINT(i); DBG_PRINT(": ");
            if (servoSubtrim[i] >= 0) DBG_PRINT("+");
            DBG_PRINTLN(servoSubtrim[i]);
          }
        }
        else if (strcmp(command_buffer, "subtrim save") == 0 || strcmp(command_buffer, "st save") == 0) {
          DBG_PRINTLN("Copy and paste this into your code:");
          DBG_PRINT("int8_t servoSubtrim[8] = {");
          for (int i = 0; i < 8; i++) {
            DBG_PRINT(servoSubtrim[i]);
            if (i < 7) DBG_PRINT(", ");
          }
          DBG_PRINTLN("};");
        }
        else if (strncmp(command_buffer, "subtrim reset", 13) == 0 || strncmp(command_buffer, "st reset", 8) == 0) {
          for (int i = 0; i < 8; i++) servoSubtrim[i] = 0;
          DBG_PRINTLN("All subtrim values reset to 0");
        }
        else if (strncmp(command_buffer, "subtrim ", 8) == 0 || strncmp(command_buffer, "st ", 3) == 0) {
          const char* params = (command_buffer[1] == 't') ? command_buffer + 3 : command_buffer + 8;
          int trimMotor, trimValue;
          if (sscanf(params, "%d %d", &trimMotor, &trimValue) == 2) {
            if (trimMotor >= 0 && trimMotor < 8) {
              if (trimValue >= -90 && trimValue <= 90) {
                servoSubtrim[trimMotor] = trimValue;
                DBG_PRINT("Motor "); DBG_PRINT(trimMotor); DBG_PRINT(" subtrim set to ");
                if (trimValue >= 0) DBG_PRINT("+");
                DBG_PRINTLN(trimValue);
              } else {
                DBG_PRINTLN("Subtrim value must be between -90 and +90");
              }
            } else {
              DBG_PRINTLN("Invalid motor number (0-7)");
            }
          }
        }
        // --- Calibration (per-servo min/max angle) ---
        else if (strcmp(command_buffer, "cal") == 0) {
          DBG_PRINTLN("Calibration (us + angle limits):");
          for (int i = 0; i < 8; i++) {
            DBG_PRINT("  "); DBG_PRINT(ServoNames[i]); DBG_PRINT(" (ch"); DBG_PRINT(i);
            DBG_PRINT("): us="); DBG_PRINT(servoMinUs[i]);
            DBG_PRINT("-"); DBG_PRINT(servoMaxUs[i]);
            DBG_PRINT("  angle="); DBG_PRINT(servoMinAngle[i]);
            DBG_PRINT("-"); DBG_PRINTLN(servoMaxAngle[i]);
          }
        }
        else if (strcmp(command_buffer, "cal save") == 0) {
          DBG_PRINTLN("Copy and paste this into your code:");
          DBG_PRINT("uint16_t servoMinUs[8]   = {");
          for (int i = 0; i < 8; i++) { DBG_PRINT(servoMinUs[i]); if (i < 7) DBG_PRINT(", "); }
          DBG_PRINTLN("};");
          DBG_PRINT("uint16_t servoMaxUs[8]   = {");
          for (int i = 0; i < 8; i++) { DBG_PRINT(servoMaxUs[i]); if (i < 7) DBG_PRINT(", "); }
          DBG_PRINTLN("};");
        }
        else if (strncmp(command_buffer, "cal reset", 9) == 0) {
          for (int i = 0; i < 8; i++) { servoMinUs[i] = 500; servoMaxUs[i] = 2500; servoMinAngle[i] = 0; servoMaxAngle[i] = 180; }
          DBG_PRINTLN("All calibration reset to 500-2500us, 0-180deg");
        }
        else if (strncmp(command_buffer, "cal ", 4) == 0) {
          int calCh, calMin, calMax;
          if (sscanf(command_buffer + 4, "%d %d %d", &calCh, &calMin, &calMax) == 3) {
            if (calCh >= 0 && calCh < 8 && calMin >= 500 && calMax <= 2500 && calMin < calMax) {
              servoMinUs[calCh] = calMin;
              servoMaxUs[calCh] = calMax;
              DBG_PRINT(ServoNames[calCh]); DBG_PRINT(" cal set to min=");
              DBG_PRINT(calMin); DBG_PRINT("us max="); DBG_PRINT(calMax); DBG_PRINTLN("us");
            } else {
              DBG_PRINTLN("Usage: cal <ch 0-7> <min 500-2499> <max 501-2500>");
            }
          }
        }
        else if (strncmp(command_buffer, "all ", 4) == 0) {
             if (sscanf(command_buffer + 4, "%d", &angle) == 1) {
                 for (int i = 0; i < 8; i++) setServoAngle(i, angle);
                 DBG_PRINT("All servos set to "); DBG_PRINTLN(angle);
             }
        }
        else if (sscanf(command_buffer, "%d %d", &motorNum, &angle) == 2) {
             if (motorNum >= 0 && motorNum < 8) {
                 setServoAngle(motorNum, angle);
                 DBG_PRINT("Servo "); DBG_PRINT(motorNum); DBG_PRINT(" set to "); DBG_PRINTLN(angle);
             } else {
                 DBG_PRINTLN("Invalid motor number (0-7)");
             }
        }
        buffer_pos = 0;
      }
    } else if (buffer_pos < sizeof(command_buffer) - 1) {
      command_buffer[buffer_pos++] = c;
    }
  }
#endif
}

// Function to update the robot's face
void updateFaceBitmap(const unsigned char* bitmap) {
  if (!oledAvailable) return;
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, 128, 64, SSD1306_WHITE);
  display.display();
}

uint8_t countFrames(const unsigned char* const* frames, uint8_t maxFrames) {
  if (frames == nullptr || frames[0] == nullptr) return 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < maxFrames; i++) {
    if (frames[i] == nullptr) break;
    count++;
  }
  return count;
}

void setFace(const String& faceName) {
  if (faceName == currentFaceName && currentFaceFrames != nullptr) return;

  currentFaceName = faceName;
  currentFaceFrameIndex = 0;
  lastFaceFrameMs = 0;
  faceFrameDirection = 1;
  faceAnimFinished = false;
  currentFaceFps = getFaceFpsForName(faceName);

  currentFaceFrames = face_defualt_frames;
  currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);

  for (size_t i = 0; i < (sizeof(faceEntries) / sizeof(faceEntries[0])); i++) {
    if (faceName.equalsIgnoreCase(faceEntries[i].name)) {
      currentFaceFrames = faceEntries[i].frames;
      currentFaceFrameCount = countFrames(faceEntries[i].frames, faceEntries[i].maxFrames);
      break;
    }
  }

  if (currentFaceFrameCount == 0) {
    currentFaceFrames = face_defualt_frames;
    currentFaceFrameCount = countFrames(face_defualt_frames, MAX_FACE_FRAMES);
    currentFaceName = "default";
    currentFaceFps = getFaceFpsForName(currentFaceName);
  }

  if (currentFaceFrameCount > 0 && currentFaceFrames[0] != nullptr) {
    updateFaceBitmap(currentFaceFrames[0]);
  }
}

void setFaceMode(FaceAnimMode mode) {
  currentFaceMode = mode;
  faceFrameDirection = 1;
  faceAnimFinished = false;
}

void setFaceWithMode(const String& faceName, FaceAnimMode mode) {
  setFaceMode(mode);
  setFace(faceName);
}

int getFaceFpsForName(const String& faceName) {
  for (size_t i = 0; i < (sizeof(faceFpsEntries) / sizeof(faceFpsEntries[0])); i++) {
    if (faceName.equalsIgnoreCase(faceFpsEntries[i].name)) {
      return faceFpsEntries[i].fps;
    }
  }
  return faceFps;
}

void updateAnimatedFace() {
  if (oledAvailable == 0 ||currentFaceFrames == nullptr || currentFaceFrameCount <= 1) return;
  if (currentFaceMode == FACE_ANIM_ONCE && faceAnimFinished) return;

  unsigned long now = millis();
  int fps = max(1, (currentFaceFps > 0 ? currentFaceFps : faceFps));
  unsigned long interval = 1000UL / fps;
  if (now - lastFaceFrameMs >= interval) {
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
    } else {
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
}

void delayWithFace(unsigned long ms) {
#if USE_PCA9685
  flushServoFrame();
#endif
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateAnimatedFace();
    server.handleClient();
    dnsServer.processNextRequest();
    delay(5);
  }
}

void scheduleNextIdleBlink(unsigned long minMs, unsigned long maxMs) {
  unsigned long now = millis();
  unsigned long interval = (unsigned long)random(minMs, maxMs);
  nextIdleBlinkMs = now + interval;
}

void enterIdle() {
  idleActive = true;
  idleBlinkActive = false;
  idleBlinkRepeatsLeft = 0;
  setFaceWithMode("idle", FACE_ANIM_BOOMERANG);
  scheduleNextIdleBlink(3000, 7000);
}

void exitIdle() {
  idleActive = false;
  idleBlinkActive = false;
}

void updateIdleBlink() {
  if (!idleActive) return;

  if (!idleBlinkActive) {
    if (millis() >= nextIdleBlinkMs) {
      idleBlinkActive = true;
      if (idleBlinkRepeatsLeft == 0 && random(0, 100) < 30) {
        idleBlinkRepeatsLeft = 1; // double blink
      }
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

// ====== HELPERS ======

#if USE_PCA9685
// Frame buffer for batched PCA9685 updates.
// All 8 channels are written in a single I2C transaction for perfect sync.
static bool    servoFrameDirty = false;
static bool    servoOutputsEnabled = true;
static unsigned long lastServoActivityMs = 0;

#if ENABLE_SERVO_AUTO_RELEASE
// Auto-release timeout while in rest pose and no command is active.
static const unsigned long SERVO_AUTO_RELEASE_MS = 1500;
#endif

// PCA9685 LED0_ON_L register address (auto-increments through all channels)
#define PCA9685_LED0_ON_L 0x06

void enableServoOutputs() {
  if (servoOutputsEnabled) return;
  servoOutputsEnabled = true;
  servoFrameDirty = true;
}

void disableServoOutputs() {
  if (!servoOutputsEnabled) return;
  for (uint8_t ch = 0; ch < 8; ch++) {
    // FULL_OFF bit disables output pulses on this channel.
    pwmDriver.setPWM(ch, 0, 4096);
  }
  servoOutputsEnabled = false;
  servoFrameDirty = false;
}

void markServoActivity() {
  lastServoActivityMs = millis();
}

#if ENABLE_SERVO_AUTO_RELEASE
void maybeAutoReleaseServos() {
  if (servoOutputsEnabled && currentCommand == "" &&
      currentFaceName.equalsIgnoreCase("rest") &&
      (millis() - lastServoActivityMs) >= SERVO_AUTO_RELEASE_MS) {
    disableServoOutputs();
  }
}
#endif

void flushServoFrame() {
  if (!servoFrameDirty || !servoOutputsEnabled) return;
  // Build a 32-byte payload: 8 channels × 4 registers (ON_L, ON_H, OFF_L, OFF_H)
  uint8_t buf[32];
  for (uint8_t ch = 0; ch < 8; ch++) {
    int16_t angle = constrain(servoFrameBuf[ch], (int16_t)servoMinAngle[ch], (int16_t)servoMaxAngle[ch]);
    uint16_t us = map(angle, 0, 180, servoMinUs[ch], servoMaxUs[ch]);
    uint16_t ticks = map(us, 500, 2500, PCA9685_SERVOMIN, PCA9685_SERVOMAX);
    buf[ch * 4 + 0] = 0;                       // ON_L  (pulse starts at tick 0)
    buf[ch * 4 + 1] = 0;                       // ON_H
    buf[ch * 4 + 2] = ticks & 0xFF;            // OFF_L
    buf[ch * 4 + 3] = (ticks >> 8) & 0x0F;     // OFF_H
  }
  // Single I2C burst — all channels update on the same STOP condition.
  Wire.beginTransmission(PCA9685_I2C_ADDR);
  Wire.write(PCA9685_LED0_ON_L);
  Wire.write(buf, 32);
  Wire.endTransmission();
  servoFrameDirty = false;
}

// Set a complete frame of all 8 servos and flush immediately.
// Order: R1, R2, L1, L2, R4, R3, L3, L4 (matches ServoName enum).
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
#endif

void setServoAngle(uint8_t channel, int angle) { 
  if (channel < 8) {
    int adjustedAngle = constrain(angle + servoSubtrim[channel], (int)servoMinAngle[channel], (int)servoMaxAngle[channel]);
#if USE_PCA9685
  enableServoOutputs();
  markServoActivity();
    servoFrameBuf[channel] = adjustedAngle;
    servoFrameDirty = true;
#else
    int us = map(adjustedAngle, 0, 180, servoMinUs[channel], servoMaxUs[channel]);
    servos[channel].writeMicroseconds(us);
    // Stagger direct-PWM writes to limit simultaneous servo inrush current.
    delayWithFace(motorCurrentDelay);
#endif
  }
}

bool pressingCheck(const String& cmd, int ms) {
#if USE_PCA9685
  flushServoFrame();
#endif
  unsigned long start = millis();
  while (millis() - start < ms) {
    server.handleClient();
    dnsServer.processNextRequest();
    updateAnimatedFace();
    if (currentCommand != cmd) {
      runStandPose(1);
      return false;
    }
    yield();
  }
  return true;
}

void recordInput() {
  lastInputTime = millis();
  if (!firstInputReceived) {
    firstInputReceived = true;
    showingWifiInfo = false;
  }
}

void updateWifiInfoScroll() {
  // Don't show WiFi info if first input has been received
  if (firstInputReceived) {
    if (showingWifiInfo) {
      showingWifiInfo = false;
      // Restore the current face
      if (currentFaceFrames != nullptr && currentFaceFrameCount > 0) {
        updateFaceBitmap(currentFaceFrames[currentFaceFrameIndex]);
      }
    }
    return;
  }
  
  unsigned long now = millis();
  
  // Check if 30 seconds have passed without input
  if (!showingWifiInfo && (now - lastInputTime >= 30000)) {
    showingWifiInfo = true;
    wifiScrollPos = 0;
    lastWifiScrollMs = now;
  }
  
  if (!showingWifiInfo) return;
  if (!oledAvailable) return;
  
  // Update scroll every 150ms
  if (now - lastWifiScrollMs >= 150) {
    lastWifiScrollMs = now;
    
    // Clear and redraw with current face in background
    display.clearDisplay();
    
    // Draw the face bitmap in the background
    if (currentFaceFrames != nullptr && currentFaceFrameCount > 0) {
      display.drawBitmap(0, 0, currentFaceFrames[currentFaceFrameIndex], 128, 64, SSD1306_WHITE);
    }
    
    // Draw black bar for text background on top row
    display.fillRect(0, 0, 128, 10, SSD1306_BLACK);
    
    // Draw scrolling text
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.setCursor(-wifiScrollPos, 1);
    display.print(wifiInfoText);
    display.setTextWrap(true);
    
    display.display();
    
    // Advance scroll position
    wifiScrollPos += 2;
    if (wifiScrollPos >= (int)(wifiInfoText.length() * 6)) {
      wifiScrollPos = 0;
    }
  }
}
