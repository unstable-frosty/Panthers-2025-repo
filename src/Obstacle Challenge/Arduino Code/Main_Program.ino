#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "gyro.h"
#include "openmv.h"
#include "ev3com.h"

// ======================================================================
//                                 setup
// ======================================================================
#define DEBUG 
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("\n=== Nano ESP32: IMU on A4/A5, EV3 on D12/D11 ==="));

  // -------- EV3 I2C slave on Wire1 (D12/D11) --------
  // 100kHz is safest for EV3; adjust if needed.
  EV3Wire.begin((uint8_t)SLAVE_ADDRESS, EV3_SDA, EV3_SCL, 100000);
  EV3Wire.onReceive(receiveData);
  EV3Wire.onRequest(sendData);
  Serial.println(F("EV3 I2C slave ready @ 0x04 on D12/D11"));

  // -------- IMU I2C master on Wire (A4/A5) --------
  Wire.begin();           // use board’s default SDA/SCL (A4/A5 on Nano layout)
  Wire.setClock(400000);  // 400k if your wiring is short/clean; 100k if long
  hwResetIfAvailable();

  if (!startBNO()) {
    Serial.println(F("BNO085 not detected at 0x4A; trying 0x4B..."));
    currentAddr = 0x4B;
    hwResetIfAvailable();
    if (!startBNO()) {
      Serial.println(F("BNO085 not detected at either address."));
    } else {
      Serial.println(F("BNO085 detected at 0x4B."));
    }
  } else {
    Serial.print(F("BNO085 detected at 0x"));
    Serial.println(currentAddr, HEX);
  }

  Serial.println(F("Streaming zeroed, accumulated yaw; EV3 reads 4-byte float.\n"));


  // ======================================================================
  //                                  OPENMV INIT
  // ======================================================================
  delay(300);
  Serial.println("\n--- Arduino Setup Start ---");
  SerialOpenMV.begin(19200, SERIAL_8N1, OpenMV_RX_PIN, OpenMV_TX_PIN);

  lastDataTime = millis();
  Serial.println("SerialOpenMV initialized.");
}

// ======================================================================
//                                  loop
// ======================================================================
void loop() {
  // Handle optional EV3 commands (e.g., re-zero)
  if (cmdReady) {
    int cmd = lastCmd;
    cmdReady = false;
    if (cmd == 0x01) {
      firstYawSet = false;
      accumYaw = 0.0f;
      lastYaw = 0.0f;
      Serial.println(F("[CMD] Re-zero yaw"));
    }
  }

  // IMU connected?
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

  // Drain all pending IMU events
  while (imu.getSensorEvent()) {
    gotAny = true;

    if (imu.wasReset()) {
      #ifdef DEBUG
      Serial.println(F("[INFO] IMU reset. Re-configuring reports + re-zero..."));
      #endif
      configureReports();
      firstYawSet = false;
      accumYaw = 0.0f;
      lastYaw = 0.0f;
    }

    if (imu.sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float x = imu.getQuatI();
      float y = imu.getQuatJ();
      float z = imu.getQuatK();
      float w = imu.getQuatReal();
      float rawYaw = quatToYawDeg(w, x, y, z);

      // First valid reading defines zero
      if (!firstYawSet) {
        yawOffset = rawYaw;
        lastYaw = 0.0f;
        accumYaw = 0.0f;
        firstYawSet = true;
      }

      // Zeroed yaw
      float yaw = rawYaw - yawOffset;

      // Normalize to -180..180
      while (yaw > 180.0f) yaw -= 360.0f;
      while (yaw < -180.0f) yaw += 360.0f;

      // Delta with wrap-around handling
      float delta = yaw - lastYaw;
      if (delta > 180.0f) delta -= 360.0f;
      if (delta < -180.0f) delta += 360.0f;

      // Accumulate (unbounded)
      accumYaw += delta;
      lastYaw = yaw;

      // Optional debug
      static uint32_t lastPrint = 0;
      if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        Serial.print(F("AccumYaw: "));
        Serial.print(accumYaw, 2);
        Serial.print(F(" deg | Yaw: "));
        Serial.println(yaw, 2);
      }
    }
  }

  if (gotAny) {
    lastEventMs = millis();
  } else if (millis() - lastEventMs > noDataTimeoutMs) {
    recover();
  }

  delay(1);
  //OPENMV SIDE
// ======== "TRY" WRAPPER START ========
  bool ok = true;  // if anything fails → we set ok = false

  // Read incoming bytes
  while (SerialOpenMV.available() > 0) {

    char c = SerialOpenMV.read();
    lastDataTime = millis();

    if (c == '\n') {
      dataReady = true;
      break;
    }

    if (inputString.length() > 120) {
      ok = false;
      reportError("Buffer overflow - corrupted packet");
      inputString = "";
      break;
    }

    if (isPrintable(c) || c == '-' || c == ',' || isDigit(c)) {
      inputString += c;
    }
  }

  // If something failed during read → act like a catch
  if (!ok) {
    inputString = "";
    dataReady = false;
    return;  // leave loop safely
  }

  // Process received message
  if (dataReady) {
    if (!safeProcessLine(inputString)) {
      reportError("Corrupted packet - parse failed");
    }
    inputString = "";
    dataReady = false;
  }

  // Timeout = OpenMV stopped sending
  if (millis() - lastDataTime > timeoutInterval) {
    reportError("OpenMV stopped sending data (timeout)");
    handleReconnect();
    lastDataTime = millis();
  }
  // ======== "TRY" WRAPPER END ========
}
