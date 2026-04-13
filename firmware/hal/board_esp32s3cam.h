#pragma once

// =============================================================================
// HAL: ESP32-S3-CAM (placeholder — update for your specific module)
// =============================================================================
//
// Board: Generic ESP32-S3-CAM module (e.g., FREENOVE ESP32-S3-WROOM,
//        LILYGO T-Camera S3, or similar)
// Arduino IDE board target: "ESP32S3 Dev Module" (adjust as needed)
//
// ⚠  PLACEHOLDER — verify GPIO numbers against your exact board schematic
//    before use.  Camera, PSRAM, and flash pin assignments vary between
//    ESP32-S3-CAM vendors.  Replace the I2C_SDA / I2C_SCL values below with
//    the correct pins for your module if they differ.
//
// ── I2C Bus ──────────────────────────────────────────────────────────────────
// Many ESP32-S3-CAM boards expose a dedicated I2C header or use GPIO 8/9 for
// their on-board peripherals (camera SCCB, IMU, etc.).  GPIO 8 and GPIO 9 are
// chosen here as a common default on ESP32-S3 dev-kit style layouts.
//
// Check whether your board already has pull-ups on these lines; if it does,
// external pull-ups for the PCA9685 are not required.
//
// ── SD-Card Pins (typical, preserved) ───────────────────────────────────────
// On most ESP32-S3-CAM modules the SD card is wired to the SDMMC peripheral:
//   SD_CLK  → GPIO 39   SD_CMD  → GPIO 38
//   SD_D0   → GPIO 40   SD_D1   → GPIO 41
//   SD_D2   → GPIO 42   SD_D3   → GPIO 43
// These pins are NOT touched by this HAL.  Verify against your board's
// schematic — exact assignments differ by vendor.
//
// ── Servo Driver ─────────────────────────────────────────────────────────────
// As with the ESP32-CAM, the camera peripheral consumes LEDC timer resources.
// A PCA9685 I2C PWM driver is used for all servo channels.
//
// PCA9685 pulse-width mapping (50 Hz / 4096 ticks):
//   SERVOMIN 150 ticks ≈  732 µs  (matches existing firmware MIN_PULSE)
//   SERVOMAX 600 ticks ≈ 2929 µs  (matches existing firmware MAX_PULSE)
// =============================================================================

// ── I2C ──────────────────────────────────────────────────────────────────────
// Default ESP32-S3 I2C pins — update if your board uses different GPIOs.
#define I2C_SDA 8
#define I2C_SCL 9

// ── PCA9685 servo driver ─────────────────────────────────────────────────────
#define USE_PCA9685        1
#define PCA9685_I2C_ADDR   0x40

// Pulse-width tick counts for the PCA9685 at 50 Hz (4096 ticks = 20 ms).
// Formula: ticks = pulse_µs / (20000 / 4096) = pulse_µs * 4096 / 20000
// Chosen to match the 732–2929 µs range used by the existing ESP32Servo code.
#define PCA9685_SERVOMIN   150   //  ≈  732 µs  (150 * 20000 / 4096)
#define PCA9685_SERVOMAX   600   //  ≈ 2929 µs  (600 * 20000 / 4096)
