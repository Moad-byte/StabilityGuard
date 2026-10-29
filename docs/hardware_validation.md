# Physical Hardware Verification — Raspberry Pi Pico 2 W

StabilityGuard was integrated and bench-tested on a **Raspberry Pi Pico 2 W (RP2350)**. The pre-existing Wokwi simulation uses a **Pico W (RP2040)**; both variants are retained as separate builds.

## Hardware configuration

| Device | Connection | Details |
|---|---|---|
| Raspberry Pi Pico 2 W | Main MCU | Physical hardware |
| MPU6050-labeled GY-521 motion module | I²C GP4 (SDA), GP5 (SCL) | Bus address `0x68`; observed `WHO_AM_I = 0x72` |
| SSD1306 OLED | I²C GP4 / GP5 | Bus address `0x3C`, 128×32 physical display |
| Green / yellow / red LEDs | GP12 / GP11 / GP10 | Each uses its own 330 Ω current-limiting resistor |
| Low-current passive piezo buzzer | GP13 | Audible alerts |
| Momentary fault-injection button | GP14 to GND | Active-low with internal pull-up |

## What was verified on the physical bench

The builder reports successful checks of:

- Pico 2 W USB upload and execution.
- MPU sensor response on I²C address `0x68`, and live acceleration/tilt readings.
- OLED response at I²C address `0x3C`, real-time angle display, and state display.
- SAFE / WARNING / DANGER transitions with matching green / yellow / red LEDs.
- Buzzer alarm output.
- Button-triggered SENSOR_FAULT and recovery after release.
- Full assembly functioning together using the hardware-specific firmware.

These observations are physical bench checks; they are not a calibrated metrology study, environmental qualification, or a claim of deployment-ready industrial protection.

## Sensor-ID compatibility finding

The module on this build communicates at I²C address **`0x68`** but returned a `WHO_AM_I` register value of **`0x72`**, rather than the `0x68` commonly reported by genuine MPU6050 devices. This is why an earlier hardware-integration candidate rejected initialization with `READ_FAILURE` despite the I²C device responding. The physical hardware sketch provisionally accepts the stable, nonstandard ID; it logs a warning and retains the existing acceleration plausibility checks. The exact chip identity has not been established.

## Firmware separation

- **Simulation:** `src/` remains the tested **Pico W** Wokwi source and 128×64 OLED configuration.
- **Physical build:** [`hardware/pico2w/StabilityGuard_Hardware_Pico2W/`](../hardware/pico2w/StabilityGuard_Hardware_Pico2W/) contains the physical **Pico 2 W** Arduino sketch and matching `StabilityGuard.h`. The core header is byte-identical to the simulation header supplied for this update.
- **Physical display:** SSD1306 **128×32** layout.
- Physical device startup includes an MPU reset/wake sequence and additional device-ID compatibility logic.

## Reproduce the physical setup

1. Install Arduino IDE 2, Earle Philhower's Raspberry Pi Pico/RP2040/RP2350 board support, and the Adafruit SSD1306/GFX libraries.
2. Select **Raspberry Pi Pico 2 W**, and the appropriate USB port.
3. With USB disconnected, verify all wiring follows the table above, including 3.3 V power, common ground, LED resistors and correct switch polarity.
4. Open `hardware/pico2w/StabilityGuard_Hardware_Pico2W/StabilityGuard_Hardware_Pico2W.ino` in Arduino IDE; upload and open Serial Monitor at **115200 baud**.
5. Confirm boot self-test, sensor initialization and live telemetry; verify SAFE, WARNING, DANGER, injected fault and recovery, and the sound/LED outputs.

## Real hardware evidence

![Physical StabilityGuard assembled on two breadboards](../media/hardware/hardware_overview.jpg)

| SAFE | WARNING |
|:---:|:---:|
| ![Green LED and SAFE OLED](../media/hardware/safe_state.jpg) | ![Yellow LED and WARNING OLED](../media/hardware/warning_state.jpg) |

| DANGER | SENSOR_FAULT |
|:---:|:---:|
| ![Red LED and DANGER OLED](../media/hardware/danger_state.jpg) | ![Fault indication on the OLED](../media/hardware/sensor_fault_state.jpg) |

[Watch the real hardware demonstration](../media/hardware/StabilityGuard_Hardware_Demo.mp4)
