/**
 * @file sketch.ino
 * @brief StabilityGuard v2.1.1 — Pico 2 W physical integration test
 *
 * Microcontroller: Raspberry Pi Pico 2 W (physical board / Arduino-Pico core)
 *
 * Hardware Peripherals & Pinout:
 *   - MPU6050 6-DoF IMU: GPIO4 (SDA), GPIO5 (SCL), I2C Address 0x68
 *   - SSD1306 128x32 OLED: GPIO4 (SDA), GPIO5 (SCL), I2C Address 0x3C
 *   - Green LED (SAFE): GPIO12
 *   - Yellow LED (WARNING): GPIO11
 *   - Red LED (DANGER): GPIO10
 *   - Piezo Buzzer: GPIO13
 *   - Fault Injection Pushbutton: GPIO14 (Active LOW, internal pull-up)
 *
 * Educational prototype. Not a certified safety device.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "StabilityGuard.h"

using namespace stabilityguard;

// ============================================================================
// HARDWARE DEFINITIONS & CONSTANTS
// ============================================================================

static const uint8_t MPU_ADDR   = 0x68;
static const uint8_t OLED_ADDR  = 0x3C;

static const uint8_t RED_PIN    = 10;
static const uint8_t YELLOW_PIN = 11;
static const uint8_t GREEN_PIN  = 12;
static const uint8_t BUZZER_PIN = 13;
static const uint8_t BUTTON_PIN = 14;

static const unsigned long SAMPLE_INTERVAL_MS  = 100; // 10 Hz sampling rate
static const unsigned long DISPLAY_INTERVAL_MS = 250; // 4 Hz display refresh
static const unsigned long RETRY_INTERVAL_MS   = 1500; // 1.5s I2C retry delay

static const bool ENABLE_OLED = true;

// Maximum acceptable resting bias for zero-reference calibration (~8.7° tilt or ~2500 counts)
static const int16_t MAX_CALIB_OFFSET_COUNTS = 2500;

// ============================================================================
// SYSTEM OBJECTS & GLOBAL STATE
// ============================================================================

Guard guard;
Adafruit_SSD1306 oled(128, 32, &Wire, -1);

bool imuReady   = false;
bool oledReady  = false;

unsigned long lastSampleTime   = 0;
unsigned long lastInitAttempt  = 0;
unsigned long lastDisplayTime  = 0;
unsigned long lastAlarmBeep    = 0;
unsigned long buzzerStopMillis = 0;

State previousReportedState = State::SENSOR_FAULT;

// Calibration offsets (uncalibrated hardware counts)
int16_t calib_offset_x = 0;
int16_t calib_offset_y = 0;
int16_t calib_offset_z = 0;

// Interactive Scenario Injection Harness (for repeatable Wokwi live testing)
enum class InjectionMode : uint8_t {
  REAL_SENSOR,
  OVERRIDE_LEVEL,     // 0° (SAFE)
  OVERRIDE_MODERATE,  // ~20° (WARNING)
  OVERRIDE_SEVERE,    // ~40° (DANGER)
  AUTO_DEMO_SEQUENCE  // SAFE -> WARNING -> DANGER -> WARNING -> SAFE
};

InjectionMode currentInjectionMode = InjectionMode::REAL_SENSOR;
unsigned long demoSequenceStart = 0;

// ============================================================================
// LOW-LEVEL I2C COMMUNICATION
// ============================================================================

bool writeMPURegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

bool readMPURegister(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)MPU_ADDR, 1) != 1) {
    return false;
  }

  int b = Wire.read();
  if (b < 0) return false;

  val = (uint8_t)b;
  return true;
}

bool initMPU() {
  uint8_t who = 0;
  if (!readMPURegister(0x75, who)) {
    Serial.println(F("# [MPU] WHO_AM_I read failed"));
    return false;
  }

  Serial.print(F("# [MPU] WHO_AM_I reported: 0x"));
  Serial.println(who, HEX);

  // A genuine MPU6050 reports 0x68. Some modules sold as MPU6050
  // report a nonstandard 0x70 or 0x72. Support for these is provisional:
  // demand repeatable ID reads, then validate real accelerometer samples.
  if (who != 0x68 && who != 0x70 && who != 0x72) {
    Serial.println(F("# [MPU] Unrecognized device ID; sensor disabled."));
    return false;
  }
  for (int i = 0; i < 2; ++i) {
    delay(10);
    uint8_t repeatID = 0;
    if (!readMPURegister(0x75, repeatID) || repeatID != who) {
      Serial.println(F("# [MPU] Inconsistent device ID; check wiring/part."));
      return false;
    }
  }
  if (who != 0x68) {
    Serial.println(F("# [MPU] WARNING: Nonstandard device ID; validate raw sensor readings."));
  }

  // Physical Pico 2 W verified initialization sequence:
  // reset, wait, wake, wait, then configure sensor.
  if (!writeMPURegister(0x6B, 0x80)) return false;
  delay(150);
  if (!writeMPURegister(0x6B, 0x00)) return false;
  delay(100);

  // Set +/-2g accelerometer full-scale range (16,384 LSB/g)
  if (!writeMPURegister(0x1C, 0x00)) return false;

  // Set Digital Low Pass Filter to ~44 Hz bandwidth
  if (!writeMPURegister(0x1A, 0x03)) return false;

  Serial.println(F("# [MPU] Initialization OK, sensor ready."));
  return true;
}

// Reads uncalibrated raw hardware register counts directly from MPU6050
bool readRawHardwareAccel(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H register

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) {
    return false;
  }

  uint8_t buf[6];
  for (int i = 0; i < 6; ++i) {
    int b = Wire.read();
    if (b < 0) return false;
    buf[i] = (uint8_t)b;
  }

  raw_x = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  raw_y = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
  raw_z = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

  return true;
}

// Reads calibrated accelerometer values (subtracts stored zero-reference offsets)
bool readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  if (!readRawHardwareAccel(ax, ay, az)) {
    return false;
  }

  ax -= calib_offset_x;
  ay -= calib_offset_y;
  az -= calib_offset_z;

  return true;
}

// ============================================================================
// NON-BLOCKING BUZZER & LED ACTUATORS
// ============================================================================

void soundAlarm(unsigned long now, unsigned int freqHz, unsigned int durationMs) {
  tone(BUZZER_PIN, freqHz);
  buzzerStopMillis = now + durationMs;
}

void updateActuators(State s, unsigned long now) {
  // Non-blocking buzzer stop
  if (buzzerStopMillis > 0 && now >= buzzerStopMillis) {
    noTone(BUZZER_PIN);
    buzzerStopMillis = 0;
  }

  const bool faultFlash = ((now / 350) % 2 == 0);

  // Discrete LED outputs
  digitalWrite(GREEN_PIN, (s == State::SAFE) ? HIGH : LOW);

  digitalWrite(
    YELLOW_PIN,
    (s == State::WARNING || (s == State::SENSOR_FAULT && faultFlash)) ? HIGH : LOW
  );

  digitalWrite(
    RED_PIN,
    (s == State::DANGER || (s == State::SENSOR_FAULT && faultFlash)) ? HIGH : LOW
  );

  // Audible Alarm Patterns
  if (s == State::DANGER) {
    // High-urgency periodic beep: 1000 Hz, 90ms pulse every 600ms
    if (now - lastAlarmBeep >= 600) {
      soundAlarm(now, 1000, 90);
      lastAlarmBeep = now;
    }
  } else if (s == State::SENSOR_FAULT) {
    // Fault tone: 420 Hz, 120ms pulse every 1000ms
    if (now - lastAlarmBeep >= 1000) {
      soundAlarm(now, 420, 120);
      lastAlarmBeep = now;
    }
  } else {
    // SAFE or WARNING: silent
    if (buzzerStopMillis == 0) {
      noTone(BUZZER_PIN);
    }
  }
}

// ============================================================================
// OLED DISPLAY RENDERER (128x32 PHYSICAL SCREEN)
// ============================================================================

void renderOLED(const Result &r, unsigned long now) {
  if (!oledReady || (now - lastDisplayTime < DISPLAY_INTERVAL_MS)) return;
  lastDisplayTime = now;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(F("STABILITYGUARD"));
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  if (r.state == State::SENSOR_FAULT || !r.valid) {
    oled.setCursor(0, 12);
    oled.print(F("SENSOR FAULT"));
    oled.setCursor(0, 23);
    oled.print(faultName(r.fault));
  } else {
    oled.setTextSize(2);
    oled.setCursor(0, 13);
    oled.print(r.filtered_tilt_deg, 1);

    oled.setTextSize(1);
    oled.setCursor(82, 18);
    oled.print(stateName(r.state));
  }
  oled.display();
}

// ============================================================================
// INTERACTIVE SERIAL COMMANDS & CALIBRATION HARNESS
// ============================================================================

void printHelpMenu() {
  Serial.println(F("\n=== StabilityGuard Interactive Test Commands ==="));
  Serial.println(F("  '0' or 'l' : Force LEVEL orientation (0 deg, SAFE)"));
  Serial.println(F("  '1' or 'w' : Force MODERATE tilt (~20 deg, WARNING)"));
  Serial.println(F("  '2' or 'd' : Force SEVERE tilt (~40 deg, DANGER)"));
  Serial.println(F("  'a'        : Run AUTOMATED live sequence (SAFE->WARN->DANGER->WARN->SAFE)"));
  Serial.println(F("  'r'        : Return to REAL MPU6050 physical/simulated readings"));
  Serial.println(F("  'c'        : Perform zero-reference resting calibration"));
  Serial.println(F("  '?'        : Display this help menu"));
  Serial.println(F("================================================\n"));
}

void calibrateZeroReference() {
  Serial.println(F("# [CALIB] Starting resting zero-reference calibration..."));
  Serial.println(F("# [CALIB] Keep sensor flat and stationary for 500ms..."));

  int32_t sum_raw_x = 0;
  int32_t sum_raw_y = 0;
  int32_t sum_raw_z = 0;
  int count = 0;

  // Sample pure uncalibrated hardware readings directly to avoid offset accumulation
  for (int i = 0; i < 20; ++i) {
    int16_t hx, hy, hz;
    if (readRawHardwareAccel(hx, hy, hz)) {
      sum_raw_x += hx;
      sum_raw_y += hy;
      sum_raw_z += (hz - 16384); // deviation from ideal +1.0g on Z-axis
      count++;
    }
    delay(20);
  }

  if (count < 10) {
    Serial.println(F("# [CALIB] Calibration failed: insufficient valid sensor reads."));
    return;
  }

  int16_t measured_bias_x = (int16_t)(sum_raw_x / count);
  int16_t measured_bias_y = (int16_t)(sum_raw_y / count);
  int16_t measured_bias_z = (int16_t)(sum_raw_z / count);

  // Safety sanity check: prevent accidentally calibrating away a severe tilt
  if (abs(measured_bias_x) > MAX_CALIB_OFFSET_COUNTS || abs(measured_bias_y) > MAX_CALIB_OFFSET_COUNTS) {
    Serial.println(F("# [CALIB] REJECTED: Platform is tilted (>8 deg)."));
    Serial.println(F("# [CALIB] Calibration aborted to prevent masking unsafe tilt."));
    return;
  }

  // Idempotently store absolute resting bias
  calib_offset_x = measured_bias_x;
  calib_offset_y = measured_bias_y;
  calib_offset_z = measured_bias_z;

  Serial.print(F("# [CALIB] Success! Stored zero-offsets: X="));
  Serial.print(calib_offset_x);
  Serial.print(F(" Y="));
  Serial.print(calib_offset_y);
  Serial.print(F(" Z="));
  Serial.println(calib_offset_z);
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n' || ch == ' ') continue;

    switch (ch) {
      case '0':
      case 'l':
        currentInjectionMode = InjectionMode::OVERRIDE_LEVEL;
        Serial.println(F("# [INJECT] Forcing Level Orientation (0 deg -> SAFE)"));
        break;
      case '1':
      case 'w':
        currentInjectionMode = InjectionMode::OVERRIDE_MODERATE;
        Serial.println(F("# [INJECT] Forcing Moderate Tilt (~20 deg -> WARNING)"));
        break;
      case '2':
      case 'd':
        currentInjectionMode = InjectionMode::OVERRIDE_SEVERE;
        Serial.println(F("# [INJECT] Forcing Severe Tilt (~40 deg -> DANGER)"));
        break;
      case 'a':
        currentInjectionMode = InjectionMode::AUTO_DEMO_SEQUENCE;
        demoSequenceStart = millis();
        Serial.println(F("# [INJECT] Starting Automated Transition Cycle:"));
        Serial.println(F("#   0-4s: SAFE | 4-8s: WARNING | 8-12s: DANGER | 12-16s: WARNING | 16-20s: SAFE"));
        break;
      case 'r':
        currentInjectionMode = InjectionMode::REAL_SENSOR;
        Serial.println(F("# [INJECT] Resumed Real MPU6050 Measurements."));
        break;
      case 'c':
        calibrateZeroReference();
        break;
      case '?':
      case 'h':
        printHelpMenu();
        break;
      default:
        break;
    }
  }
}

// ============================================================================
// SELF-TEST
// ============================================================================

void runSelfTest() {
  Guard testGuard;
  Result r;
  for (int i = 0; i < 5; ++i) {
    r = testGuard.update(Sample(0, 0, 16384, true, Fault::NONE));
  }

  Serial.print(F("# [SELFTEST] Level (0,0,16384): "));
  Serial.print((r.valid && r.state == State::SAFE) ? F("PASS") : F("FAIL"));
  Serial.print(F(" stage="));
  Serial.print(testGuard.diagnosticStage());
  Serial.print(F(" magSq="));
  Serial.println((long)testGuard.magnitudeSquaredRaw());
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 1500)) {
    delay(10);
  }

  Serial.println(F("=== StabilityGuard v2.1.1 Pico 2 W Hardware Integration ==="));

  runSelfTest();

  // Initialize shared I2C bus on GPIO4 (SDA) and GPIO5 (SCL)
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire.setClock(100000); // 100 kHz standard mode

  lastInitAttempt = millis();
  imuReady = initMPU();

  if (ENABLE_OLED) {
    oledReady = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (oledReady) {
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE);
      oled.setTextSize(1);
      oled.setCursor(0, 3);
      oled.println(F("STABILITYGUARD"));
      oled.setCursor(0, 17);
      oled.println(F("INITIALIZING"));
      oled.display();
    } else {
      Serial.println(F("# [OLED] SSD1306 not detected on 0x3C; continuing headless."));
    }
  }

  printHelpMenu();

  // Print standardized CSV Telemetry Header
  Serial.println(F("time_ms,state,filtered_tilt_deg,raw_tilt_deg,valid,fault_reason,ax,ay,az,mag_mg"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // Process incoming user commands
  handleSerialCommands();

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    // Retry MPU initialization periodically if communication was lost
    if (!imuReady && (currentInjectionMode == InjectionMode::REAL_SENSOR)) {
      if (now - lastInitAttempt >= RETRY_INTERVAL_MS) {
        lastInitAttempt = now;
        imuReady = initMPU();
      }
    }

    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 16384;
    bool readSuccess = false;

    // Determine sample acquisition mode
    if (currentInjectionMode == InjectionMode::OVERRIDE_LEVEL) {
      ax = 0; ay = 0; az = 16384;
      readSuccess = true;
    } else if (currentInjectionMode == InjectionMode::OVERRIDE_MODERATE) {
      ax = -5603; ay = 0; az = 15401; // ~20.0 deg
      readSuccess = true;
    } else if (currentInjectionMode == InjectionMode::OVERRIDE_SEVERE) {
      ax = -10535; ay = 0; az = 12550; // ~40.0 deg
      readSuccess = true;
    } else if (currentInjectionMode == InjectionMode::AUTO_DEMO_SEQUENCE) {
      unsigned long elapsed = now - demoSequenceStart;
      if (elapsed < 4000) {
        ax = 0; ay = 0; az = 16384; // 0-4s: Level (0°) -> SAFE
      } else if (elapsed < 8000) {
        ax = -5603; ay = 0; az = 15401; // 4-8s: Moderate (~20°) -> WARNING
      } else if (elapsed < 12000) {
        ax = -10535; ay = 0; az = 12550; // 8-12s: Severe (~40°) -> DANGER
      } else if (elapsed < 16000) {
        ax = -5603; ay = 0; az = 15401; // 12-16s: Moderate (~20°) -> WARNING
      } else if (elapsed < 20000) {
        ax = 0; ay = 0; az = 16384; // 16-20s: Level (0°) -> SAFE
      } else {
        Serial.println(F("# [INJECT] Automated demonstration cycle finished."));
        currentInjectionMode = InjectionMode::REAL_SENSOR;
      }
      readSuccess = true;
    } else {
      // Physical sensor reading
      readSuccess = imuReady && readRawAccel(ax, ay, az);
    }

    // Physical hardware fault-injection pushbutton (Active LOW on GP14)
    bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

    // Pass validated sample into the safety engine
    Result r = guard.update(
      Sample(
        ax,
        ay,
        az,
        readSuccess && !buttonPressed,
        buttonPressed ? Fault::INJECTED : Fault::READ_FAILURE
      )
    );

    // Log state transitions with full context
    if (r.state != previousReportedState) {
      Serial.print(F("# TRANSITION: ["));
      Serial.print(now);
      Serial.print(F(" ms] "));
      Serial.print(stateName(previousReportedState));
      Serial.print(F(" -> "));
      Serial.print(stateName(r.state));
      Serial.print(F(" (Angle: "));
      Serial.print(r.filtered_tilt_deg, 2);
      Serial.print(F(" deg, Fault: "));
      Serial.print(faultName(r.fault));
      Serial.println(F(")"));
      previousReportedState = r.state;
    }

    // Standardized CSV Telemetry Line
    Serial.print(now);
    Serial.print(',');
    Serial.print(stateName(r.state));
    Serial.print(',');
    Serial.print(r.filtered_tilt_deg, 2);
    Serial.print(',');
    Serial.print(r.raw_tilt_deg, 2);
    Serial.print(',');
    Serial.print(r.valid ? 1 : 0);
    Serial.print(',');
    Serial.print(faultName(r.fault));
    Serial.print(',');
    Serial.print(ax);
    Serial.print(',');
    Serial.print(ay);
    Serial.print(',');
    Serial.print(az);
    Serial.print(',');
    Serial.println(guard.accelerationMagnitude_mg());

    // Render OLED output
    renderOLED(r, now);

    // Mark sensor as offline on read error
    if (!readSuccess && (currentInjectionMode == InjectionMode::REAL_SENSOR)) {
      if (imuReady) {
        Serial.println(F("# [I2C] Sensor read transaction failed; scheduling retry."));
      }
      imuReady = false;
    }
  }

  // Update physical actuators (LEDs & non-blocking buzzer)
  updateActuators(guard.result().state, now);
}
