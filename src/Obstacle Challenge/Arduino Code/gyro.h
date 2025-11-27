#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ======================================================================
//                          IMU GLOBALS
// ======================================================================
BNO08x imu;

const int PIN_RST = -1;
const int PIN_INT = -1;

static inline float rad2deg(float r) { return r * 180.0f / PI; }
static inline float quatToYawDeg(float w, float x, float y, float z) {
  float s = 2.0f * (w * z + x * y);
  float c = 1.0f - 2.0f * (y * y + z * z);
  return rad2deg(atan2f(s, c));
}

uint8_t currentAddr = 0x4A;
const uint16_t reportIntervalMs = 20;
const uint32_t noDataTimeoutMs = 1000;
uint32_t lastEventMs = 0;

// Yaw values (shared with EV3 / rest of code)
volatile float accumYaw = 0.0f;
float yawOffset = 0.0f;
float lastYaw = 0.0f;
bool firstYawSet = false;

// ======================================================================
//                        INTERNAL IMU HELPERS
// ======================================================================
inline void hwResetIfAvailable() {
  if (PIN_RST >= 0) {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(5);
    digitalWrite(PIN_RST, HIGH);
    delay(300);
  } else {
    delay(300);
  }
}

// Use GAME ROTATION VECTOR (gyro + accel, NO magnetometer)
inline bool configureReports() {
  // SparkFun: enableReport(sensorId, interval_us)
  return imu.enableReport(SH2_GAME_ROTATION_VECTOR, reportIntervalMs * 1000UL);
}

inline bool startBNO() {
  bool ok = imu.begin(currentAddr, Wire, PIN_INT, PIN_RST);
  if (!ok) return false;

  configureReports();

  firstYawSet = false;
  accumYaw = 0;
  lastYaw = 0;

  lastEventMs = millis();
  return true;
}

inline void recover() {
  Serial.println(F("[WARN] IMU lost — recovering"));

  hwResetIfAvailable();

  if (!startBNO()) {
    currentAddr = (currentAddr == 0x4A ? 0x4B : 0x4A);
    hwResetIfAvailable();
    startBNO();
  }
}

// ======================================================================
//                                SETUP
// ======================================================================
inline void Gyro_setup() {
  Serial.println("Initializing IMU...");

  Wire.begin();
  Wire.setClock(400000);

  hwResetIfAvailable();

  if (!startBNO()) {
    Serial.println(F("Trying IMU address 0x4B..."));
    currentAddr = 0x4B;
    hwResetIfAvailable();
    startBNO();
  }
}

// ======================================================================
//                                UPDATE
// ======================================================================
inline void Gyro_update() {

  if (!imu.isConnected()) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 1000) {
      lastTry = millis();
      recover();
    }
    delay(1);
    return;
  }

  bool gotAny = false;

  while (imu.getSensorEvent()) {
    gotAny = true;

    if (imu.wasReset()) {
      Serial.println("IMU reset detected");
      configureReports();
      firstYawSet = false;
      accumYaw = 0;
      lastYaw = 0;
    }

    // *** use GAME ROTATION VECTOR instead of ROTATION_VECTOR ***
    if (imu.sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {

      float x = imu.getQuatI();
      float y = imu.getQuatJ();
      float z = imu.getQuatK();
      float w = imu.getQuatReal();
      float rawYaw = quatToYawDeg(w, x, y, z);

      if (!firstYawSet) {
        yawOffset = rawYaw;
        accumYaw = 0;
        lastYaw = 0;
        firstYawSet = true;
      }

      float yaw = rawYaw - yawOffset;

      while (yaw > 180) yaw -= 360;
      while (yaw < -180) yaw += 360;

      float delta = yaw - lastYaw;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;

      accumYaw += delta;
      lastYaw = yaw;
    }
  }

  if (gotAny)
    lastEventMs = millis();
  else if (millis() - lastEventMs > noDataTimeoutMs)
    recover();

  delay(1);
}

#endif
