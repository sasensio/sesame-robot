# Sesame Robot — Hardware Abstraction Layer (HAL)

This folder contains board-specific pin and peripheral configuration headers for the Sesame Robot firmware.  Including `hal/hal.h` from the main sketch pulls in the correct settings for whichever board is selected.

---

## How to select a board

Define a board macro **before** `hal/hal.h` is processed.  The easiest way is to add a `#define` at the very top of `sesame-firmware-main.ino` (above any `#include`):

```cpp
// ── Pick exactly one ──────────────────────────────────────────────────
// #define BOARD_ESP32CAM        // AI-Thinker ESP32-CAM
// #define BOARD_ESP32S3CAM      // Generic ESP32-S3-CAM (placeholder)
// #define BOARD_DISTRO_V2       // Sesame Distro Board V2 (ESP32-S3)
// #define BOARD_DISTRO_V1       // Sesame Distro Board V1 (ESP32-WROOM32)
// (default = Lolin S2 Mini when nothing is defined)
```

Alternatively, pass the flag through your build system (arduino-cli, PlatformIO, etc.):

```
arduino-cli compile --build-property "compiler.cpp.extra_flags=-DBOARD_ESP32CAM" ...
```

---

## Supported boards

### AI-Thinker ESP32-CAM (`BOARD_ESP32CAM`)

**Arduino IDE target:** *AI Thinker ESP32-CAM*

#### I2C bus

GPIO 26 (SIOD) and GPIO 27 (SIOC) are the OV2640 camera SCCB lines.  They are routed only to the camera FFC connector on the PCB and **are not exposed on the external J2/J3 header pins**, so an external device cannot be physically connected to them.

The accessible header GPIO pins on this board are:

| Header | Pins |
|--------|------|
| J3 | 5V · GND · IO12 · IO13 · IO15 · IO14 · IO2 · IO4 |
| J2 | 3V3 · GND · IO16 · IO0 · IO3(RX) · IO1(TX) · GND · 5V |

After excluding boot-strapping pins (GPIO 0, 2, 12), serial lines (GPIO 1, 3), the flash-LED pin (GPIO 4), and the PSRAM CS pin (GPIO 16), the only viable I2C candidates are **GPIO 13, 14, and 15**.

Chosen mapping:

| Signal  | GPIO | Rationale |
|---------|------|-----------|
| I2C SDA | **14** | SD_CLK — no boot constraint, physically accessible on J3 |
| I2C SCL | **15** | SD_CMD — internal pull-up keeps it HIGH at boot, matching I2C idle state |

> **Trade-off:** GPIO 14 (SD_CLK) and GPIO 15 (SD_CMD) are consumed by I2C, so the SD card cannot operate simultaneously. The four SD data lines — GPIO 2, 4, 12, and 13 — are preserved. SD support can be re-enabled by switching the board back to a direct-PWM configuration.

#### Servo driver

The camera peripheral occupies all four LEDC timer groups that `ESP32Servo` requires, so direct PWM output is not available on this board.  A **PCA9685** I2C 16-channel PWM driver is used for all eight servo channels instead.

PCA9685 pulse-width tick counts at 50 Hz (4096 ticks = 20 ms period):

| Define           | Ticks | Pulse width | Formula                     |
|------------------|-------|-------------|------------------------------|
| `PCA9685_SERVOMIN` | 150 | ≈ 732 µs  | `150 × 20000 / 4096` µs     |
| `PCA9685_SERVOMAX` | 600 | ≈ 2929 µs | `600 × 20000 / 4096` µs     |

These match the `732–2929 µs` range used by `ESP32Servo` on legacy boards.

#### SD-card pins

| Signal    | GPIO | Status with this HAL |
|-----------|------|----------------------|
| SD_DATA0  |  2   | Preserved            |
| SD_DATA1  |  4   | Preserved            |
| SD_DATA2  | 12   | Preserved            |
| SD_DATA3  | 13   | Preserved            |
| SD_CLK    | 14   | **Used as I2C SDA**  |
| SD_CMD    | 15   | **Used as I2C SCL**  |

#### Strapping pins (do not reassign)

| GPIO | Function                                    |
|------|---------------------------------------------|
|  0   | Boot-mode select (LOW = download mode)      |
|  2   | Must be LOW / floating at boot              |
| 12   | Must be LOW at boot (PSRAM 1.8 V select)    |
| 15   | Must be HIGH at boot (internal pull-up)     |

#### Pin summary

| Signal         | GPIO | Notes                                      |
|----------------|------|--------------------------------------------|
| I2C SDA        |  14  | J3 header — SD_CLK repurposed              |
| I2C SCL        |  15  | J3 header — SD_CMD repurposed              |
| CAM LED        |   4  | On-board flash LED (web-toggle capable)    |
| PCA9685 addr   | 0x40 | Default A0–A5 = LOW                        |
| Servo driver   | PCA9685 channels 0–7                       ||

---

### Generic ESP32-S3-CAM (`BOARD_ESP32S3CAM`) — placeholder

**Arduino IDE target:** *ESP32S3 Dev Module* (adjust for your specific module)

> ⚠ **This is a placeholder configuration.**  GPIO assignments vary significantly between ESP32-S3-CAM vendors (FREENOVE, LILYGO T-Camera S3, etc.).  Verify `I2C_SDA` and `I2C_SCL` against your board's schematic before use.

#### I2C bus

Default pins: **GPIO 8 (SDA)** and **GPIO 9 (SCL)** — a common I2C mapping on ESP32-S3 dev-kit layouts.  Update `board_esp32s3cam.h` if your board differs.

#### Servo driver

Same as ESP32-CAM: PCA9685 at address **0x40**.

#### SD-card pins (typical, preserved)

Typical SDMMC wiring on ESP32-S3-CAM modules — verify against your board:

| Signal   | GPIO |
|----------|------|
| SD_CLK   | 39   |
| SD_CMD   | 38   |
| SD_D0    | 40   |
| SD_D1    | 41   |
| SD_D2    | 42   |
| SD_D3    | 43   |

---

### Legacy boards (existing behaviour, no PCA9685)

These boards use `ESP32Servo` for direct PWM output.  Their pin settings are kept inline in `hal.h` to preserve the existing firmware behaviour without requiring separate board files.

| Board                  | Macro             | I2C SDA | I2C SCL |
|------------------------|-------------------|---------|---------|
| Sesame Distro Board V2 | `BOARD_DISTRO_V2` |  8      |  9      |
| Sesame Distro Board V1 | `BOARD_DISTRO_V1` | 21      | 22      |
| Lolin S2 Mini (default)| *(none)*          | 33      | 35      |

The `servoPins[]` array for these boards is still defined in the main sketch.

---

## PCA9685 wiring

```
ESP32-CAM (J3 header)   PCA9685 breakout
──────────────────────  ──────────────────────
GPIO 14 (SDA)  ──────►  SDA
GPIO 15 (SCL)  ──────►  SCL
3.3 V  ────────────────  VCC  (logic supply)
GND    ────────────────  GND
                         V+   ← external 5–6 V servo supply
A0–A5  all LOW (default address 0x40)
```

Connect servo signal wires to PCA9685 output channels 0–7, matching the `ServoName` enum (`R1`=0 … `L4`=7).

---

## Required libraries (PCA9685 boards)

Install via the Arduino IDE Library Manager:

- **Adafruit PWM Servo Driver Library** (`Adafruit_PWMServoDriver`)
