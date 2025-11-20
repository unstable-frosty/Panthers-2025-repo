#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ---------------------- EV3 <-> Arduino (I2C Slave on Wire1) ----------------------
#define SLAVE_ADDRESS 0x04
#define EV3_SDA 12   // EV3 bus SDA -> D12
#define EV3_SCL 11   // EV3 bus SCL -> D11

TwoWire EV3Wire = TwoWire(1); // second I2C controller for EV3 link (slave)

// ---------------------- BNO085 on default I2C (Wire, A4/A5) -----------------------
BNO08x imu;
const int PIN_RST = -1;  // wire if you have it, else -1
const int PIN_INT = -1;  // optional INT (recommended), else -1

static inline float rad2deg(float r) { return r * 180.0f / PI; }
static float quatToYawDeg(float w, float x, float y, float z) {
  float s = 2.0f * (w * z + x * y);
  float c = 1.0f - 2.0f * (y * y + z * z);
  return rad2deg(atan2f(s, c));
}

// ---- IMU settings ----
uint8_t currentAddr = 0x4A;                // 0x4B if PS0 is HIGH
const uint16_t reportIntervalMs = 20;      // 50 Hz
const uint32_t noDataTimeoutMs = 1000;
uint32_t lastEventMs = 0;

// ---- yaw tracking (shared with EV3 callbacks) ----
volatile float accumYaw = 0.0f;            // unbounded
float yawOffset = 0.0f;
float lastYaw = 0.0f;
bool firstYawSet = false;

// ---------------------- EV3 command state ----------------------
volatile int lastCmd = 0;
volatile bool cmdReady = false;

// ======================================================================
//                           BNO085 helpers (on Wire)
// ======================================================================
void hwResetIfAvailable() {
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

bool configureReports() {
  // Or: imu.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalMs * 1000UL);
  return imu.enableRotationVector(reportIntervalMs);
}

bool startBNO() {
  // Begin on default Wire (A4/A5). INT/RST if you wired them.
  bool ok = imu.begin(currentAddr, Wire, (PIN_INT >= 0 ? PIN_INT : -1), (PIN_RST >= 0 ? PIN_RST : -1));
  if (!ok) return false;
  if (!configureReports()) {
    Serial.println(F("Failed to enable Rotation Vector."));
  }
  lastEventMs = millis();
  // re-zero on each fresh start
  firstYawSet = false;
  accumYaw = 0.0f;
  lastYaw   = 0.0f;
  return true;
}

void recover() {
  Serial.println(F("\n[WARN] No IMU data; recovering..."));
  hwResetIfAvailable();
  if (!startBNO()) {
    currentAddr = (currentAddr == 0x4A) ? 0x4B : 0x4A;
    hwResetIfAvailable();
    if (!startBNO()) {
      Serial.println(F("[ERR] Re-init failed. Check wiring/power."));
    }
  } else {
    Serial.println(F("[OK] Recovered."));
  }
}

// ======================================================================
//                      EV3 I2C slave callbacks (Wire1)
// ======================================================================
void receiveData(int byteCount) {
  while (EV3Wire.available() > 0) {
    lastCmd = EV3Wire.read();
    cmdReady = true;
  }
  // Commands: 0x01 -> re-zero yaw
}

// void sendData() {
//   int16_t centi = (int16_t)roundf(accumYaw * 100.0f);  // deg * 100, signed
//   uint8_t buf[2];
//   // Little-endian: LSB first, then MSB
//   buf[0] = (uint8_t)(centi & 0xFF);
//   buf[1] = (uint8_t)((centi >> 8) & 0xFF);
//   EV3Wire.write(buf, 2);
// }
void sendData() {
  // 32-bit signed centi-degrees (deg * 100)
  int32_t centi = (int32_t)roundf(accumYaw * 100.0f);

  uint8_t buf[4];
  // Little-endian: least significant byte first
  buf[0] = (uint8_t)(centi & 0xFF);
  buf[1] = (uint8_t)((centi >> 8) & 0xFF);
  buf[2] = (uint8_t)((centi >> 16) & 0xFF);
  buf[3] = (uint8_t)((centi >> 24) & 0xFF);

  EV3Wire.write(buf, 4);
}


// ======================================================================
//                                 setup
// ======================================================================
void setup() {

  Serial.begin(115200);
  //while (!Serial) {}
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println(F("\n=== Nano ESP32: IMU on A4/A5, EV3 on D12/D11 ==="));
 
  // -------- EV3 I2C slave on Wire1 (D12/D11) --------
  // 100kHz is safest for EV3; adjust if needed.
  EV3Wire.begin((uint8_t)SLAVE_ADDRESS, EV3_SDA, EV3_SCL, 100000);
  EV3Wire.onReceive(receiveData);
  EV3Wire.onRequest(sendData);
  Serial.println(F("EV3 I2C slave ready @ 0x04 on D12/D11"));

  // -------- IMU I2C master on Wire (A4/A5) --------
  Wire.begin();            // use board’s default SDA/SCL (A4/A5 on Nano layout)
  Wire.setClock(400000);   // 400k if your wiring is short/clean; 100k if long
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
      lastYaw  = 0.0f;
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
      Serial.println(F("[INFO] IMU reset. Re-configuring reports + re-zero..."));
      configureReports();
      firstYawSet = false;
      accumYaw = 0.0f;
      lastYaw  = 0.0f;
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
        lastYaw   = 0.0f;
        accumYaw  = 0.0f;
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
}
