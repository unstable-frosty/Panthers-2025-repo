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