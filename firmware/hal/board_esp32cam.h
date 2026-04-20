#pragma once

// =============================================================================
// HAL: ESP32-CAM (AI-Thinker)
// =============================================================================
//
// Board: AI-Thinker ESP32-CAM (ESP32 module with OV2640 camera)
// Arduino IDE board target: "AI Thinker ESP32-CAM"
//
// ── I2C Bus ──────────────────────────────────────────────────────────────────
// GPIO 26 (SIOD) and GPIO 27 (SIOC) are the OV2640 camera's SCCB lines.
// They are routed only to the camera FFC connector on the PCB and are NOT
// brought out to the external J2/J3 header pins, so an external I2C device
// cannot be connected to them.
//
// The accessible header pins on the AI-Thinker ESP32-CAM are:
//   J3: 5V  GND  IO12  IO13  IO15  IO14  IO2  IO4
//   J2: 3V3 GND  IO16  IO0   IO3   IO1   GND  5V
//
// GPIO pins excluded from I2C use:
//   GPIO  0 — boot-mode strapping (must be HIGH; LOW = download mode)
//   GPIO  1 — serial TX (UART0)
//   GPIO  2 — SD_DATA0, strapping pin (must be LOW / floating at boot)
//   GPIO  3 — serial RX (UART0)
//   GPIO  4 — SD_DATA1; drives the on-board flash LED HIGH
//   GPIO 12 — SD_DATA2, strapping pin (must be LOW at boot w/ 4 MB PSRAM)
//   GPIO 16 — connected to PSRAM /CS on this module; do not reassign
//
// Remaining header pins eligible for I2C:
//   GPIO 13 — SD_DATA3    (no boot constraint)
//   GPIO 14 — SD_CLK      (no boot constraint)           ← chosen SDA
//   GPIO 15 — SD_CMD      (must be HIGH at boot) ← chosen SCL
//             Internal pull-up keeps GPIO 15 HIGH during boot,
//             which is compatible with the I2C SCL idle state.
//
// Trade-off: SD_CLK and SD_CMD are consumed by I2C, so the SD card cannot
// operate simultaneously.  The four SD data lines (GPIO 2, 4, 12, 13) are
// preserved and SD can be re-enabled by swapping back to ESP32Servo/direct PWM.
//
// ── SD-Card Pins ─────────────────────────────────────────────────────────────
//   SD_DATA0 → GPIO 2   SD_DATA1 → GPIO 4
//   SD_DATA2 → GPIO 12  SD_DATA3 → GPIO 13  ← preserved
//   SD_CLK   → GPIO 14                       ← used as I2C SDA
//   SD_CMD   → GPIO 15                       ← used as I2C SCL
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
// GPIO 14 (SD_CLK) as SDA and GPIO 15 (SD_CMD) as SCL — both physically
// accessible on the J3 header; no critical boot-strapping conflicts.
#define I2C_SDA 14
#define I2C_SCL 15

// ── PCA9685 servo driver ─────────────────────────────────────────────────────
#define USE_PCA9685        1
#define PCA9685_I2C_ADDR   0x40

// Pulse-width tick counts for the PCA9685 (27 MHz oscillator, prescale 131).
// Actual period ≈ 20.024 ms → 4.889 µs/tick.
// Conservative range matching MF90 datasheet: 500–2500 µs.
#define PCA9685_SERVOMIN   102   //  ≈  500 µs  (102 × 4.889 µs)
#define PCA9685_SERVOMAX   511   //  ≈ 2500 µs  (511 × 4.889 µs)

// ── Camera (OV2640) ──────────────────────────────────────────────────────────
#define HAS_CAMERA         1
#define CAM_PIN_PWDN       32
#define CAM_PIN_RESET      -1
#define CAM_PIN_XCLK        0
#define CAM_PIN_SIOD       26
#define CAM_PIN_SIOC       27
#define CAM_PIN_D7         35
#define CAM_PIN_D6         34
#define CAM_PIN_D5         39
#define CAM_PIN_D4         36
#define CAM_PIN_D3         21
#define CAM_PIN_D2         19
#define CAM_PIN_D1         18
#define CAM_PIN_D0          5
#define CAM_PIN_VSYNC      25
#define CAM_PIN_HREF       23
#define CAM_PIN_PCLK       22

// On-board white LED (flash LED) on AI-Thinker ESP32-CAM.
#define CAM_PIN_LED         4
