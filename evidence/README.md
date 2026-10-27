# StabilityGuard — Automated Test Evidence

This directory archives raw simulation logs and execution evidence captured from the verified Wokwi simulation on Raspberry Pi Pico W.

---

## 1. Automated Test Run (`automated_test_log.txt`)

* **Simulation Platform**: [Wokwi Simulation Project 478987065565532161](https://wokwi.com/projects/478987065565532161)
* **Controller**: Raspberry Pi Pico W (`board-pi-pico-w`)
* **Trigger Command**: Serial command `'a'` (Automated Demonstration Sequence)
* **Sampling Rate**: 10.0 Hz (100 ms period)
* **Total Duration**: 19.90 seconds (200 logged telemetry frames)

### Observed State Transitions

| Timestamp | Previous State | New State | Filtered Tilt | Raw Target Tilt | Debounce Confirmation |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **32,800 ms** | `SENSOR_FAULT` | `SAFE` | 0.00° | 0.00° | 5 consecutive valid samples on level boot |
| **36,900 ms** | `SAFE` | `WARNING` | 18.48° | 19.99° | 3 consecutive samples $\ge 15.0^\circ$ |
| **40,700 ms** | `WARNING` | `DANGER` | 36.44° | 40.01° | 3 consecutive samples $\ge 30.0^\circ$ |
| **45,000 ms** | `DANGER` | `WARNING` | 20.97° | 19.99° | 5 consecutive samples $< 26.0^\circ$ |
| **48,900 ms** | `WARNING` | `SAFE` | 1.51° | 0.00° | 5 consecutive samples $< 12.0^\circ$ |

---

## 2. Telemetry Visualization (`automated_test_log_plot.svg`)

The vector plot illustrates the dynamic response of the Exponential Moving Average filter ($\alpha = 0.35$), the $3^\circ$–$4^\circ$ hysteresis bands, and the asymmetric debounce confirmation delays.
