# StabilityGuard — Media & Evidence Directory

This directory contains visual assets, simulation captures, and telemetry visualizations from the verified Wokwi simulation.

---

## Directory Inventory

| File | Type | Description |
|---|:---:|---|
| **`banner.png`** | PNG Banner | Repository header cover with engineering typography and IMU vector accents |
| **`01_circuit.png`** | Simulation Screenshot | Complete Wokwi breadboard circuit: Raspberry Pi Pico W, MPU6050, SSD1306 OLED, LEDs, buzzer, and button |
| **`01_commands.png`** | Terminal Capture | Interactive Serial Monitor command menu and test injection interface |
| **`02_safe.png`** | State Evidence | **SAFE** state operation: Green LED active, OLED reading `0.0 deg`, clean self-test |
| **`03_warning.png`** | State Evidence | **WARNING** state operation: Yellow LED active, OLED reading `20.0 deg` |
| **`04_danger.png`** | State Evidence | **DANGER** state operation: Red LED active, OLED reading `40.0 deg`, audible alert |
| **`05_sensor_fault.png`** | State Evidence | **SENSOR_FAULT** active: Excessive simulated acceleration rejected as `INVALID_VECTOR`; fault indication visible |
| **`telemetry_plot.svg`** | Telemetry Visualization | Vector plot generated from the automated test log showing filtered vs. raw tilt and state zones |
| **`actual_wokwi_telemetry.csv`** | Telemetry Stream | 200 actual CSV-formatted readings extracted from `evidence/automated_test_log.txt` (Pico W automated cycle) |
