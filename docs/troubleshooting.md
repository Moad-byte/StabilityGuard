# StabilityGuard — Troubleshooting & Engineering Post-Mortem

**Version:** 2.1.1  
**Verified Reference Platform:** Raspberry Pi Pico W (`board-pi-pico-w`) in Wokwi Simulation  
**Live Simulation URL:** [Wokwi Project 478987065565532161](https://wokwi.com/projects/478987065565532161)  

---

## 1. Issue Summary Matrix

| Issue ID | Symptom | Root Cause | Verified Resolution |
|---|---|---|---|
| **BUG-01** | `# SELFTEST raw(0,0,16384): FAIL stage=ANGLE_MATH_INVALID` on `board-pi-pico-2w` | Board-specific angle-validation failure observed on Wokwi Pico 2 W; exact emulator/compiler cause remains unconfirmed | Standardized Phase 1 verified simulation on **Raspberry Pi Pico W (`board-pi-pico-w`)**, which uses soft-float emulation |
| **BUG-02** | Garbage serial output and memory crash (`3500...zt?/ à?/ c¬...ÿÿÿÿ`) | 3-argument `tone(pin, freq, duration)` uses hardware timer alarm interrupts colliding with UART/I²C | Replaced with non-blocking timestamp polling (`tone(pin, freq)` + manual `noTone()`) |
| **BUG-03** | `# I2C READ FAILED; retry in 1.5s` and `# MPU WHO_AM_I read failed` | `Wire.endTransmission(false)` repeated-start condition leaving bus in indeterminate state | Switched to explicit STOP condition `Wire.endTransmission(true)` |
| **BUG-04** | OLED disabled to isolate intermittent I2C bus lockups | Consequence of BUG-02 and BUG-03 corrupting the shared I²C bus | Resolved root causes; OLED safely re-enabled and verified at 4 Hz refresh |
| **BUG-05** | Repeated `'c'` serial calibration caused zero-offset drift and loss of offset | Calibration queried `readRawAccel()` which already subtracted stored offsets | Created `readRawHardwareAccel()` to sample raw sensor registers directly with an $8.7^\circ$ tilt guard |

---

## 2. Deep Dive: Pico 2 W Simulation Angle Validation Failure (BUG-01)

### Symptom
On initial boot with `board-pi-pico-2w` in Wokwi, the built-in self-test immediately failed:
```text
=== StabilityGuard v2.0 RAW-INTEGER DIAGNOSTIC ===
# SELFTEST raw(0,0,16384): FAIL stage=ANGLE_MATH_INVALID magnitudeSquared=268435456
# MPU WHO_AM_I=0x68
# MPU READY
100,SENSOR_FAULT,0.00,0.00,0,INVALID_VECTOR
```
Every single reading was rejected as `ANGLE_MATH_INVALID`, keeping the system permanently stuck in `SENSOR_FAULT`.

### Investigation Notes (hypotheses, not confirmed root cause)
1. `diagram.json` specified `"type": "board-pi-pico-2w"`, which represents the Raspberry Pi Pico 2 W (RP2350 microcontroller with ARM Cortex-M33 cores).
2. The Arduino-Pico toolchain compiles for Cortex-M33 using the hardware floating-point ABI:
   `-mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard`
3. Under hard-float ABI, functions like `sqrtf()` compile directly into hardware instructions:
   `vsqrt.f32 s0, s0`
   and float comparisons compile to:
   `vcmp.f32 s0, #0.0`
   `vmrs APSR_nzcv, fpscr`
4. In this project, the Pico 2 W simulator returned invalid angle validation results; incomplete FPU emulation is one possible explanation, not a proven diagnosis.
5. No isolated minimal reproduction or intermediate numerical trace is provided to prove which arithmetic operation failed.
6. The `ANGLE_MATH_INVALID` diagnostic indicates an angle-bounds check failed; the actual intermediate value was not captured in the supplied evidence.

### Engineering Solution
The calculation path uses integer square root and a tangent lookup table to limit the amount of floating-point math, but still uses `float` for interpolation and filtering:
1. **Integer Square Root**:
   ```cpp
   static inline uint32_t isqrt64(uint64_t n) {
     if (n == 0) return 0;
     uint64_t x = n;
     uint64_t y = (x + 1) / 2;
     while (y < x) { x = y; y = (x + n / x) / 2; }
     return (uint32_t)x;
   }
   ```
2. **Tangent Interpolation Table**:
   Precomputed 46 entries of $\text{round}(16384 \cdot \tan(\theta))$ for $\theta \in [0^\circ, 45^\circ]$.
   Sector $0^\circ$ to $45^\circ$ is interpolated using $h / z$.
   Sector $45^\circ$ to $90^\circ$ is interpolated using $z / h$ (cotangent reflection).
   Sector $90^\circ$ to $180^\circ$ is reflected around $180^\circ$.
3. **Verification**:
   Self-test immediately passed:
   `# SELFTEST raw(0,0,16384): PASS stage=OK magnitudeSquared=268435456`
   This successful test applies to the Pico W reference simulation only; a Pico 2 W compatibility fix has not been verified.

---

## 3. Deep Dive: Timer Alarm Interrupt Collisions (BUG-02)

### Symptom
At 3500 ms of continuous simulation, the serial output suddenly corrupted with garbage characters, followed by a total I²C lockup:
```text
3500,SENSOR_FAULT,0.00,0.00zt?/ à?/ c¬
*O - Ü" µ:
ÜM È KlíàÄ1  1ÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿ,0,READ_FAILURE
# MPU WHO_AM_I read failed
```

### Root Cause Analysis
1. The firmware used `tone(BUZZER_PIN, 420, 120)` to generate periodic beeps.
2. In the Arduino-Pico core, providing a third argument (`duration`) configures an asynchronous hardware timer alarm (`hardware_alarm`) on the RP2040/RP2350.
3. When the alarm expires, an interrupt service routine (ISR) fires in the background to call `noTone()`.
4. In Wokwi's JavaScript emulation environment, an asynchronous timer alarm firing while a `Wire.requestFrom()` transaction or `Serial.print()` write is actively buffering causes re-entrant state corruption and stack misalignment.
5. This corrupted the UART transmission buffer (spewing garbage characters) and stalled the I²C state machine.

### Engineering Solution
Eliminated background timer alarm interrupts completely by implementing an asynchronous polling pattern in `loop()`:
```cpp
void soundAlarm(unsigned long now, unsigned int freqHz, unsigned int durationMs) {
  tone(BUZZER_PIN, freqHz); // 2-argument form: continuous PWM, NO timer alarm
  buzzerStopMillis = now + durationMs;
}

void updateActuators(State s, unsigned long now) {
  if (buzzerStopMillis > 0 && now >= buzzerStopMillis) {
    noTone(BUZZER_PIN);
    buzzerStopMillis = 0;
  }
  // ...
}
```
Result: The timestamp-based alarm stop avoids a duration argument in the `tone` call; the available stable runs do not establish a universal reliability guarantee.

---

## 4. Deep Dive: I²C Repeated-Start Arbitration (BUG-03)

### Symptom
Periodic `# I2C READ FAILED; retry in 1.5s` errors occurred during register reads.

### Root Cause Analysis
`readRawAccel()` used `Wire.endTransmission(false)` to generate a repeated-start condition before calling `Wire.requestFrom()`. In simulated I²C controllers, if the slave model does not release SDA before the repeated start SCL edge, the bus enters a lockup condition.

### Engineering Solution
Changed `Wire.endTransmission(false)` to `Wire.endTransmission(true)`. The MPU6050 internal register pointer retains its position upon receiving an I²C STOP. The subsequent `Wire.requestFrom(MPU_ADDR, 6)` successfully reads registers `0x3B` through `0x40` in the working Pico W simulation. Compatibility with other targets is not established by these tests.
