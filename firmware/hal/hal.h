#pragma once

// =============================================================================
// HAL — Board Selector
// =============================================================================
//
// Define exactly ONE board macro before including this header, or pass it as
// a compiler flag (e.g. -DBOARD_ESP32CAM in your build system / arduino-cli
// extra flags).
//
// Supported boards
// ────────────────
//   BOARD_ESP32CAM      AI-Thinker ESP32-CAM (OV2640, PCA9685 servos)
//   BOARD_ESP32S3CAM    Generic ESP32-S3-CAM module (placeholder)
//
// Legacy boards (pin constants defined inline below, existing behaviour kept)
// ────────────────────────────────────────────────────────────────────────────
//   BOARD_DISTRO_V1     Sesame Distro Board V1 (ESP32-WROOM32)
//   BOARD_DISTRO_V2     Sesame Distro Board V2 (ESP32-S3)
//   BOARD_LOLIN_S2_MINI Lolin S2 Mini (default if nothing is defined)
// =============================================================================

#if defined(BOARD_ESP32CAM)
  #include "board_esp32cam.h"

#elif defined(BOARD_ESP32S3CAM)
  #include "board_esp32s3cam.h"

#elif defined(BOARD_DISTRO_V2)
  // Sesame Distro Board V2 (ESP32-S3)
  #define I2C_SDA      8
  #define I2C_SCL      9
  #define USE_PCA9685  0

#elif defined(BOARD_DISTRO_V1)
  // Sesame Distro Board V1 (ESP32-WROOM32)
  #define I2C_SDA      21
  #define I2C_SCL      22
  #define USE_PCA9685  0

#else
  // Default: Lolin S2 Mini (ESP32-S2)
  #define BOARD_LOLIN_S2_MINI
  #define I2C_SDA      33
  #define I2C_SCL      35
  #define USE_PCA9685  0

#endif

// Provide safe defaults for any optional symbols not set by a board header.
#ifndef USE_PCA9685
  #define USE_PCA9685 0
#endif

#ifndef HAS_CAMERA
  #define HAS_CAMERA 0
#endif

#ifndef CAM_PIN_LED
  #define CAM_PIN_LED -1
#endif
