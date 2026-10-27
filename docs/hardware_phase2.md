# StabilityGuard — Phase 2 Physical Hardware Implementation Guide

**Target Board:** Raspberry Pi Pico W / Pico 2 W (Pin-compatible RP2040 / RP2350)  
**Sensors & Display:** MPU6050 (GY-521 breakout), SSD1306 0.91-inch 128x32 I²C OLED  
**Actuators:** Green/Yellow/Red 5mm LEDs, Active/Passive Piezo Buzzer, Tactile Pushbutton  

---

## 1. Safety Rules & Electrical Specifications

> [!CAUTION]
> The Raspberry Pi Pico 2 W GPIO pins operate at **3.3V logic level** and are **NOT 5V tolerant**.
> Never connect 5V (VBUS) to any GPIO pin or sensor VCC pin unless the sensor module includes an onboard 3.3V low-dropout (LDO) regulator and level shifters.

1. **Power Supply**: Connect MPU6050 VCC and SSD1306 VCC to **Pico Pin 36 (3V3 OUT)**.
2. **Current Limitations**: Each RP2350 GPIO pin should supply $\le 12 \text{ mA}$ (nominal $4 \text{ mA}$ to $8 \text{ mA}$).
   * Red LED ($V_f \approx 2.0\text{V}$): $(3.3\text{V} - 2.0\text{V}) / 330\Omega \approx 3.9 \text{ mA}$ (Safe).
   * Yellow LED ($V_f \approx 2.1\text{V}$): $(3.3\text{V} - 2.1\text{V}) / 330\Omega \approx 3.6 \text{ mA}$ (Safe).
   * Green LED ($V_f \approx 3.0\text{V}$): $(3.3\text{V} - 3.0\text{V}) / 330\Omega \approx 0.9 \text{ mA}$ (Safe).
3. **Piezo Buzzer**:
   * If using a passive piezo buzzer: direct drive via GPIO13 with a series $100\Omega$ resistor is safe.
   * If using an active magnetic buzzer ($> 30\text{ mA}$): use an NPN transistor (2N2222/2N3904) with a base resistor and flyback diode.

---

## 2. Complete Physical Breadboard Wiring Plan

### Raspberry Pi Pico 2 W Pinout Mapping

| Component | Component Pin | Pico 2 W Physical Pin | Pico GPIO / Function | Wire Color | Notes |
|---|---|---|---|---|---|
| **MPU6050** | VCC | Pin 36 | `3V3 OUT` | Red | Powers sensor from 3.3V rail |
| **MPU6050** | GND | Pin 38 | `GND` | Black | Ground reference |
| **MPU6050** | SDA | Pin 6 | `GPIO 4` | Blue | Shared I²C0 SDA |
| **MPU6050** | SCL | Pin 7 | `GPIO 5` | Purple | Shared I²C0 SCL |
| **MPU6050** | AD0 | Pin 38 | `GND` | Black | Hardwires I²C address to `0x68` |
| **MPU6050** | INT | *Not connected* | N/A | - | Unused in Phase 1 polling |
| **SSD1306 OLED** | VCC | Pin 36 | `3V3 OUT` | Red | Shared 3.3V rail |
| **SSD1306 OLED** | GND | Pin 38 | `GND` | Black | Shared Ground rail |
| **SSD1306 OLED** | SDA | Pin 6 | `GPIO 4` | Blue | Shared I²C0 SDA |
| **SSD1306 OLED** | SCL | Pin 7 | `GPIO 5` | Purple | Shared I²C0 SCL |
| **Green LED** | Anode (+) | Resistor $\implies$ Pin 16 | `GPIO 12` | Green | Via $330\Omega$ resistor |
| **Green LED** | Cathode (-) | Pin 18 | `GND` | Black | Flat edge / shorter lead to GND |
| **Yellow LED** | Anode (+) | Resistor $\implies$ Pin 15 | `GPIO 11` | Yellow | Via $330\Omega$ resistor |
| **Yellow LED** | Cathode (-) | Pin 18 | `GND` | Black | Shorter lead to GND |
| **Red LED** | Anode (+) | Resistor $\implies$ Pin 14 | `GPIO 10` | Red | Via $330\Omega$ resistor |
| **Red LED** | Cathode (-) | Pin 18 | `GND` | Black | Shorter lead to GND |
| **Piezo Buzzer** | Positive (+) | Pin 17 | `GPIO 13` | Orange | Directly or via $100\Omega$ resistor |
| **Piezo Buzzer** | Negative (-) | Pin 18 | `GND` | Black | Ground rail |
| **Fault Button** | Terminal 1 | Pin 19 | `GPIO 14` | White | Active LOW (Pico internal pull-up) |
| **Fault Button** | Terminal 2 | Pin 23 | `GND` | Black | Ground rail |

---

## 3. Physical OLED 128x32 Layout Adaptation

The simulation used a 128x64 display, but our physical hardware module is a compact **0.91-inch 128x32 OLED**.

### Firmware Modification for Physical 128x32 Display:
In `sketch.ino`, replace line 43:
```cpp
// Simulated Wokwi version (128x64):
// Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// Physical Hardware version (128x32):
Adafruit_SSD1306 oled(128, 32, &Wire, -1);
```

### Compact 128x32 Render Function:
```cpp
void renderOLED_128x32(const Result &r, unsigned long now) {
  if (!oledReady || (now - lastDisplayTime < DISPLAY_INTERVAL_MS)) return;
  lastDisplayTime = now;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // Line 1 (Header & State): "SG2.1 | SAFE"
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(F("SG2.1 | "));
  oled.print(stateName(r.state));

  // Line 2 (Large Tilt Readout): " 18.5 deg"
  oled.setTextSize(2);
  oled.setCursor(0, 12);
  if (r.valid) {
    oled.print(r.filtered_tilt_deg, 1);
  } else {
    oled.print(F("--.-"));
  }
  oled.setTextSize(1);
  oled.print(F(" deg"));

  // Line 3 (Fault indicator or status): "FAULT: NONE"
  oled.setCursor(75, 24);
  if (r.fault != Fault::NONE) {
    oled.print(faultName(r.fault));
  } else {
    oled.print(F("[OK]"));
  }

  oled.display();
}
```

---

## 4. Step-by-Step Bring-up Procedure

### Step 1: Toolchain Setup
1. Open Arduino IDE (v2.x recommended).
2. Install the **Raspberry Pi Pico/RP2040/RP2350** board package by Earle Philhower:
   * Additional Board Manager URL:
     `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
3. Tools $\implies$ Board $\implies$ Raspberry Pi RP2350 $\implies$ **Raspberry Pi Pico 2 W**.
4. Install required libraries via Library Manager:
   * **Adafruit SSD1306** (v2.5.7 or newer)
   * **Adafruit GFX Library** (v1.11.5 or newer)

### Step 2: I²C Bus Scanner (Smoke Test)
Before running StabilityGuard, upload this simple I²C scanner to verify wiring:
```cpp
#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n--- I2C Bus Scanner (GP4/GP5) ---");

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
}

void loop() {
  byte count = 0;
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at address: 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("No I2C devices found!");
  delay(3000);
}
```
**Expected Scanner Output**:
```text
Device found at address: 0x3C  <-- SSD1306 OLED
Device found at address: 0x68  <-- MPU6050 IMU
```

### Step 3: Zero-Reference Calibration
1. Place the breadboard on a verified flat, level surface.
2. In the Serial Monitor (115200 baud), type `'c'` and press Enter.
3. The firmware averages 20 samples and stores the resting offsets `(calib_offset_x, calib_offset_y, calib_offset_z)`.
4. Subsequent tilt calculations will read exactly `0.00°` on level ground.

### Step 4: Physical Tilt Verification
1. Slowly rotate the breadboard along the X-axis:
   * Tilting past $15^\circ$: Yellow LED lights up (`WARNING`).
   * Tilting past $30^\circ$: Red LED lights up, buzzer emits alarm beeps (`DANGER`).
   * Tilting back to $< 26^\circ$: Returns to `WARNING`.
   * Tilting back to $< 12^\circ$: Returns to `SAFE`, Green LED lights up.
2. Press and hold the pushbutton:
   * Yellow & Red LEDs flash immediately, buzzer emits fault chirp (`SENSOR_FAULT`, `INJECTED`).
   * Release button: System recovers to `SAFE` after exactly $500 \text{ ms}$ (5 samples).
