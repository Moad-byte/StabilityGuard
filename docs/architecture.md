# StabilityGuard — System Architecture Documentation

**Version:** 2.1.1  
**Verified Simulation Target:** Raspberry Pi Pico W (RP2040)  
**Live Simulation URL:** [Wokwi Project 478987065565532161](https://wokwi.com/projects/478987065565532161)  
**Physical Hardware Target (Phase 2):** Raspberry Pi Pico W / Pico 2 W  

---

## 1. System Overview

StabilityGuard is an embedded safety and tilt-monitoring system designed for robotic platforms, industrial mobile machinery, and micro-mobility vehicles. The system continuously samples raw 3-axis acceleration, validates the gravitational vector, filters mechanical jitter, and classifies platform stability into four mutually exclusive operational states:

```
[ SAFE ] <=======> [ WARNING ] <=======> [ DANGER ]
   ^                     ^                   ^
   |                     |                   |
   +---------------------+-------------------+
                         |
                         v
                [ SENSOR_FAULT ]
```

---

## 2. Architectural Separation of Concerns

A primary design requirement is the strict architectural boundary between:
1. **Portable Core Engine (`StabilityGuard.h`)**: Pure C++98/11 logic. Contains zero Arduino dependencies, zero register-level hardware dependencies, and uses integer arithmetic for magnitude checks and square root, while still relying on floating-point arithmetic for interpolation and EMA filtering. Native compilation and target simulation require separate verification.
2. **Firmware Actuator & I2C Driver (`sketch.ino`)**: Manages the hardware peripherals (MPU6050, SSD1306 OLED, LEDs, piezo buzzer, and pushbuttons) using non-blocking asynchronous timing.

```
+-------------------------------------------------------------------------+
|                              sketch.ino                                 |
|  +--------------------+  +--------------------+  +-------------------+  |
|  |  MPU6050 I2C (0x68)|  |  SSD1306 OLED(0x3C)|  | LEDs & Piezo Buzzer| |
|  +---------+----------+  +---------^----------+  +---------^---------+  |
|            | Raw Counts            | Telemetry             | Actuation  |
+------------|-----------------------|-----------------------|------------+
             |                       |                       |
             v                       |                       |
+------------------------------------+-----------------------+------------+
|                        StabilityGuard.h                                 |
|  1. Vector Sanity Check: 0.5g <= sqrt(x^2 + y^2 + z^2) <= 1.5g          |
|  2. Integer-Assisted Tilt Angle: isqrt64 + 46-Point Tangent Interpolation|
|  3. Exponential Moving Average: alpha = 0.35                            |
|  4. Hysteresis Classifier: 12° / 15° / 26° / 30°                        |
|  5. Debounce Confirmation: 3 samples escalate, 5 samples de-escalate    |
|  6. Fault Management: 5-sample recovery with direct state classification|
+-------------------------------------------------------------------------+
```

---

## 3. Sensor Acquisition & Shared I²C Subsystem

The MPU6050 6-axis IMU and SSD1306 OLED display share the primary I²C bus controller on the Raspberry Pi Pico:
* **SDA**: GPIO 4 (Pico Pin 6)
* **SCL**: GPIO 5 (Pico Pin 7)
* **I²C Clock**: 100 kHz standard mode

### MPU6050 Configuration & Register Map
Upon boot, `initMPU()` configures the sensor:
* `WHO_AM_I` (`0x75`): Validated for `0x68` (standard MPU6050) or `0x70` (MPU6500/9250).
* `PWR_MGMT_1` (`0x6B`): Written with `0x00` to disable internal sleep mode and start oscillator.
* `ACCEL_CONFIG` (`0x1C`): Written with `0x00` to select `+/-2g` full-scale sensitivity ($16,384 \text{ LSB/g}$).
* `CONFIG` (`0x1A`): Written with `0x03` to configure the Digital Low Pass Filter (DLPF) with $44 \text{ Hz}$ bandwidth and $4.9 \text{ ms}$ delay to attenuate high-frequency mechanical vibration.

### Bus Arbitration & Stop Condition
Earlier revisions suffered from bus lockups caused by repeated-start frames (`Wire.endTransmission(false)`). Phase 1 standardized all register pointer writes to issue an explicit I²C STOP (`Wire.endTransmission(true)`). This completes each register-pointer write before the following sensor read, which worked reliably in the verified reference simulation.

### Resting Zero-Reference Calibration Subsystem
To account for physical PCB mounting misalignment without corrupting sensor reads:
* **Separation of Hardware vs. Calibrated Reads**:
  - `readRawHardwareAccel(raw_x, raw_y, raw_z)` directly queries MPU6050 I²C registers without applying stored offsets.
  - `readRawAccel(ax, ay, az)` applies stored offsets: `ax -= calib_offset_x`, etc.
* **Idempotent Multi-Sample Averaging**:
  The `c` command samples 20 readings over 400 ms using `readRawHardwareAccel()`. Because calibration operates on raw hardware data rather than already-offset data, repeated execution does not subtract the previous offsets a second time; sensor noise and changing conditions can still affect calibration values.
* **Tilt Safety Guard**:
  If the platform is tilted by $> 2500$ counts (~$8.7^\circ$), the calibration routine rejects the command and retains prior offsets to prevent an operator from inadvertently masking an active tilt hazard.

---

## 4. Integer-Assisted Tilt Calculation

To reduce computational cost and reliance on floating-point square-root or trigonometric functions:

1. **Horizontal Magnitude**:
   $$h = \sqrt{x^2 + y^2}$$
   Implemented via 64-bit integer square root (`isqrt64`) using Newton-Raphson approximation.
2. **Angle Interpolation**:
   $$\theta = \arctan2(h, z)$$
   Evaluated using a 46-point precomputed table of $\text{round}(16384 \cdot \tan(\theta))$ for $\theta \in [0^\circ, 45^\circ]$ combined with linear interpolation and cotangent reflection for $[45^\circ, 90^\circ]$ and $[90^\circ, 180^\circ]$.
   * Accuracy should be assessed with the provided test vectors on each target.
   * Zero division protection: Explicit guards for $h = 0$ and $z = 0$.
   * The engine validates calculated angle bounds and enters `SENSOR_FAULT` for invalid results; floating-point behavior must still be checked on each target.

---

## 5. Decision Engine State Machine & Hysteresis

| Current State | Escalation Condition | De-escalation Condition | Confirmation Delay |
|---|---|---|---|
| **SAFE** | Filtered Tilt $\ge 15.0^\circ$ | N/A | 3 samples ($300 \text{ ms}$) |
| **WARNING** | Filtered Tilt $\ge 30.0^\circ$ | Filtered Tilt $< 12.0^\circ$ | Escalation: 3 samples; De-escalation: 5 samples |
| **DANGER** | N/A | Filtered Tilt $< 26.0^\circ$ | 5 samples ($500 \text{ ms}$) |
| **SENSOR_FAULT** | Overrides any state immediately | 5 consecutive valid samples | 5 samples ($500 \text{ ms}$) |

* **Hysteresis Band**:
  * Warning Band: $12.0^\circ$ (exit) to $15.0^\circ$ (entry) $\implies 3.0^\circ$ safety buffer.
  * Danger Band: $26.0^\circ$ (exit) to $30.0^\circ$ (entry) $\implies 4.0^\circ$ safety buffer.
* **Tilted Fault Recovery**:
  When recovering from `SENSOR_FAULT`, the engine does not default to `SAFE`. Instead, it evaluates `classify(filtered_tilt_deg)` after 5 valid samples. If the sensor is still at $40^\circ$, it transitions directly to `DANGER`.

---

## 6. Output Actuation Subsystems

### LEDs
* **SAFE**: Green LED ON (GPIO 12).
* **WARNING**: Yellow LED ON (GPIO 11).
* **DANGER**: Red LED ON (GPIO 10).
* **SENSOR_FAULT**: Yellow & Red LEDs flashing synchronously at $350 \text{ ms}$ interval.

### Non-Blocking Piezo Buzzer
To prevent hardware timer collisions with I²C, the buzzer is controlled via asynchronous timestamp checking:
* **DANGER Alert**: $1000 \text{ Hz}$, $90 \text{ ms}$ tone every $600 \text{ ms}$.
* **SENSOR_FAULT Alert**: $420 \text{ Hz}$, $120 \text{ ms}$ tone every $1000 \text{ ms}$.
* **SAFE / WARNING**: Completely silent.

### SSD1306 OLED (128x64)
* Refreshes every $250 \text{ ms}$ ($4 \text{ Hz}$) to minimize I²C bus utilization.
* Displays header, numerical tilt angle with unit, textual state, graphical severity meter, and fault reason.
* Non-critical: If display fails to initialize or disconnects, core safety classification continues without interruption.
