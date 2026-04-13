#pragma once

// =============================================================================
// HAL: ESP32-CAM (AI-Thinker)
// =============================================================================
//
// Board: AI-Thinker ESP32-CAM (ESP32 module with OV2640 camera)
// Arduino IDE board target: "AI Thinker ESP32-CAM"
//
// ── I2C Bus ──────────────────────────────────────────────────────────────────
// The AI-Thinker ESP32-CAM connects the OV2640 camera's SCCB (Serial Camera
// Control Bus) to GPIO 26 (SIOD / SDA) and GPIO 27 (SIOC / SCL).
// SCCB is I2C-compatible; the bus is only active during camera sensor init /
// occasional register writes, leaving it free for other peripherals the rest
// of the time.
//
// Rationale for reusing the camera SCCB bus:
//   • GPIO 26 and GPIO 27 are already pulled up on-board and are the only
//     two-wire interface pins exposed on the module that carry no SD-card duty.
//   • The OV2640 sensor sits at I2C address 0x30 (7-bit). The PCA9685 default
//     address is 0x40 — no conflict.
//   • All four-bit SDIO SD-card pins (GPIO 2, 4, 12, 13, 14, 15) are left
//     entirely untouched, preserving future SD-card support.
//
// ── SD-Card Pins (preserved, not used by this HAL) ──────────────────────────
//   SD_DATA0 → GPIO 2   SD_DATA1 → GPIO 4
//   SD_DATA2 → GPIO 12  SD_DATA3 → GPIO 13
//   SD_CLK   → GPIO 14  SD_CMD   → GPIO 15
//
// ── Servo Driver ─────────────────────────────────────────────────────────────
// The camera occupies all four LEDC timer groups that ESP32Servo needs, so
// direct PWM servo output is not possible on this board.  A PCA9685 I2C PWM
// driver is used instead.
//
// PCA9685 pulse-width mapping (50 Hz / 4096 ticks):
//   SERVOMIN 150 ticks ≈  732 µs  (matches existing firmware MIN_PULSE)
//   SERVOMAX 600 ticks ≈ 2929 µs  (matches existing firmware MAX_PULSE)
//
// ── Strapping / Boot Pins (do not reassign) ──────────────────────────────────
//   GPIO  0 — boot-mode select (LOW = download mode)
//   GPIO  2 — must be LOW or floating at boot (SD_DATA0 / pull-down)
//   GPIO 12 — must be LOW at boot when PSRAM is present (1.8 V select)
//   GPIO 15 — must be HIGH at boot (SD_CMD / internal pull-up)
// =============================================================================

// ── I2C ──────────────────────────────────────────────────────────────────────
// Reuses camera SCCB bus; shared with OV2640 sensor (addr 0x30) at no conflict.
#define I2C_SDA 26
#define I2C_SCL 27

// ── PCA9685 servo driver ─────────────────────────────────────────────────────
#define USE_PCA9685        1
#define PCA9685_I2C_ADDR   0x40

// Pulse-width tick counts for the PCA9685 at 50 Hz (4096 ticks = 20 ms).
// Formula: ticks = pulse_µs / (20000 / 4096) = pulse_µs * 4096 / 20000
// Chosen to match the 732–2929 µs range used by the existing ESP32Servo code.
#define PCA9685_SERVOMIN   150   //  ≈  732 µs  (150 * 20000 / 4096)
#define PCA9685_SERVOMAX   600   //  ≈ 2929 µs  (600 * 20000 / 4096)
