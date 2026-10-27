/*
  StabilityGuard Phase 1 - Baseline Build v2.0
  Raspberry Pi Pico / Pico W / Pico 2 W in Wokwi
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "StabilityGuard.h"

using namespace stabilityguard;

// ----------------------------------
// Addresses and pins
// ----------------------------------
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t OLED_ADDR = 0x3C;

static const uint8_t RED_PIN = 10;
static const uint8_t YELLOW_PIN = 11;
static const uint8_t GREEN_PIN = 12;
static const uint8_t BUZZER_PIN = 13;
static const uint8_t BUTTON_PIN = 14;

static const unsigned long SAMPLE_MS = 100;

static const bool ENABLE_OLED = true;

// ----------------------------------
// System objects
// ----------------------------------
Guard guard;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

bool imuReady = false;
bool oledReady = false;

unsigned long lastSample = 0;
unsigned long lastInitAttempt = 0;
unsigned long lastDisplay = 0;
unsigned long lastBeep = 0;
unsigned long beepUntil = 0;

State previousState = State::SENSOR_FAULT;

// ----------------------------------
// MPU6050 register write
// ----------------------------------
bool writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

// ----------------------------------
// MPU6050 register read
// ----------------------------------
bool readMPURegister(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(true) != 0) {
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

// ----------------------------------
// Initialize MPU6050
// ----------------------------------
bool initMPU() {
  uint8_t who = 0;

  if (!readMPURegister(0x75, who)) {
    Serial.println("# MPU WHO_AM_I read failed");
    return false;
  }

  Serial.print("# MPU WHO_AM_I=0x");
  Serial.println(who, HEX);

  if (who != 0x68 && who != 0x70) {
    return false;
  }

  // Wake the sensor
  if (!writeMPU(0x6B, 0x00)) return false;

  // Configure +/-2g range
  if (!writeMPU(0x1C, 0x00)) return false;

  // Configure digital low-pass filter
  if (!writeMPU(0x1A, 0x03)) return false;

  Serial.println("# MPU READY");
  return true;
}

// ----------------------------------
// Read RAW accelerometer values
// ----------------------------------
bool readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) {
    return false;
  }

  uint8_t bytes[6];
  for (int i = 0; i < 6; ++i) {
    int b = Wire.read();
    if (b < 0) return false;
    bytes[i] = (uint8_t)b;
  }

  ax = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
  ay = (int16_t)(((uint16_t)bytes[2] << 8) | bytes[3]);
  az = (int16_t)(((uint16_t)bytes[4] << 8) | bytes[5]);

  return true;
}

// ----------------------------------
// Non-blocking buzzer handler
// ----------------------------------
void playBeep(unsigned long now, unsigned int freq, unsigned int duration) {
  tone(BUZZER_PIN, freq);
  beepUntil = now + duration;
}

void updateBuzzer(State s, unsigned long now) {
  if (beepUntil > 0 && now >= beepUntil) {
    noTone(BUZZER_PIN);
    beepUntil = 0;
  }

  if (s == State::DANGER) {
    if (now - lastBeep >= 600) {
      playBeep(now, 1000, 90);
      lastBeep = now;
    }
  } else if (s == State::SENSOR_FAULT) {
    if (now - lastBeep >= 1000) {
      playBeep(now, 420, 120);
      lastBeep = now;
    }
  } else {
    if (beepUntil == 0) {
      noTone(BUZZER_PIN);
    }
  }
}

// ----------------------------------
// LEDs and buzzer
// ----------------------------------
void indicators(State s, unsigned long now) {
  const bool flash = (now / 350) % 2 == 0;

  digitalWrite(GREEN_PIN, s == State::SAFE ? HIGH : LOW);

  digitalWrite(
    YELLOW_PIN,
    (s == State::WARNING || (s == State::SENSOR_FAULT && flash)) ? HIGH : LOW
  );

  digitalWrite(
    RED_PIN,
    (s == State::DANGER || (s == State::SENSOR_FAULT && flash)) ? HIGH : LOW
  );

  updateBuzzer(s, now);
}

// ----------------------------------
// OLED display
// ----------------------------------
void displayResult(const Result &r, unsigned long now) {
  if (!oledReady || now - lastDisplay < 300) {
    return;
  }

  lastDisplay = now;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  oled.setCursor(0, 0);
  oled.print("STABILITYGUARD P1");

  oled.drawLine(0, 12, 127, 12, SSD1306_WHITE);

  oled.setTextSize(2);
  oled.setCursor(0, 18);

  if (r.valid) {
    oled.print(r.filtered_tilt_deg, 1);
  } else {
    oled.print("--.-");
  }

  oled.print(" deg");

  oled.setTextSize(1);
  oled.setCursor(0, 43);
  oled.print("STATE: ");
  oled.print(stateName(r.state));

  oled.setCursor(0, 55);
  oled.print("FAULT: ");
  oled.print(faultName(r.fault));

  oled.display();
}

// ----------------------------------
// Built-in safety-engine self-test
// ----------------------------------
void engineSelfTest() {
  Guard test;
  Result r;

  for (int i = 0; i < 5; ++i) {
    r = test.update(Sample(0, 0, 16384, true, Fault::NONE));
  }

  Serial.print("# SELFTEST raw(0,0,16384): ");
  Serial.print((r.valid && r.state == State::SAFE) ? "PASS" : "FAIL");
  Serial.print(" stage=");
  Serial.print(test.diagnosticStage());
  Serial.print(" magnitudeSquared=");
  Serial.println((long)test.magnitudeSquaredRaw());
}

// ----------------------------------
// Setup
// ----------------------------------
void setup() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && millis() - start < 1500) {
    delay(10);
  }

  Serial.println("=== StabilityGuard v2.0 RAW-INTEGER DIAGNOSTIC ===");

  engineSelfTest();

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire.setClock(100000);

  lastInitAttempt = millis();
  imuReady = initMPU();

  if (ENABLE_OLED) {
    oledReady = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (oledReady) {
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE);
      oled.setTextSize(1);
      oled.setCursor(10, 24);
      oled.println("STABILITYGUARD");
      oled.setCursor(10, 36);
      oled.println("INITIALIZING...");
      oled.display();
    }
  }

  Serial.println("time_ms,state,filtered_tilt_deg,raw_tilt_deg,valid,fault_reason");
}

// ----------------------------------
// Main loop
// ----------------------------------
void loop() {
  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;

    if (!imuReady && now - lastInitAttempt >= 1500) {
      lastInitAttempt = now;
      imuReady = initMPU();
    }

    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;

    bool sensorOK = imuReady && readRawAccel(ax, ay, az);
    bool injected = (digitalRead(BUTTON_PIN) == LOW);

    Result r = guard.update(
      Sample(
        ax,
        ay,
        az,
        sensorOK && !injected,
        injected ? Fault::INJECTED : Fault::READ_FAILURE
      )
    );

    static unsigned debugLines = 0;
    if (debugLines++ < 12) {
      Serial.print("# RAW_COUNTS x=");
      Serial.print(ax);
      Serial.print(" y=");
      Serial.print(ay);
      Serial.print(" z=");
      Serial.println(az);

      Serial.print("# ENGINE stage=");
      Serial.print(guard.diagnosticStage());
      Serial.print(" magSq=");
      Serial.print((long)guard.magnitudeSquaredRaw());
      Serial.print(" sensorOK=");
      Serial.print(sensorOK ? "YES" : "NO");
      Serial.print(" button=");
      Serial.println(injected ? "PRESSED" : "RELEASED");
    }

    if (r.state != previousState) {
      Serial.print("# TRANSITION -> ");
      Serial.println(stateName(r.state));
      previousState = r.state;
    }

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
    Serial.println(faultName(r.fault));

    displayResult(r, now);

    if (!sensorOK) {
      if (imuReady) {
        Serial.println("# I2C READ FAILED; retry in 1.5s");
      }
      imuReady = false;
    }
  }

  indicators(guard.result().state, now);
}
