#include "HardwareSerial.h"

// Pin definitions
#define OpenMV_RX_PIN D4
#define OpenMV_TX_PIN D5

HardwareSerial SerialOpenMV(2);

String inputString = "";
bool dataReady = false;
byte sig = 6;
int x_C = 0;

unsigned long lastDataTime = 0;
const unsigned long timeoutInterval = 2000;

// ---------- OUR CATCH SYSTEM ----------
void reportError(const char* msg) {
  Serial.print("[ERROR] ");
  Serial.println(msg);
}
// --------------------------------------
// ---------- RECONNECT ----------
void handleReconnect() {
  Serial.println("--- Attempting recovery ---");

  while (SerialOpenMV.available() > 0) {
    SerialOpenMV.read();
  }

  inputString = "";
  dataReady = false;

  Serial.println("Waiting for next OpenMV packet...");
}
bool safeParseIntegers(String data, int16_t *values, uint8_t expected) {
  uint8_t index = 0;
  char buf[128];
  data.toCharArray(buf, sizeof(buf));

  char *ptr = strtok(buf, ",");
  while (ptr != NULL && index < expected) {
    values[index++] = atoi(ptr);
    ptr = strtok(NULL, ",");
  }

  return index == expected;  // false = parsing error
}


// ---------- SAFE PARSER (our pseudo-try/catch) ----------
bool safeProcessLine(String line) {
  line.trim();
  if (line.length() == 0) return false;

  if (line.startsWith("O,")) {
    line.remove(0, 2);
    int16_t values[5];
    if (!safeParseIntegers(line, values, 5)) return false;

    Serial.println("---- OBSTACLE DATA ----");
    Serial.println(values[0]);
    Serial.println(values[1]);
    Serial.println(values[2]);
    Serial.println(values[3]);
    Serial.println(values[4]);
    sig = values[4];
    //x_CO = values[0];
    return true;
  }

  if (line.startsWith("MAG,")) {
    line.remove(0, 4);
    int16_t values[4];

    if (!safeParseIntegers(line, values, 4)) return false;

    Serial.println("---- MAGENTA DATA ----");
    Serial.println(values[0]);
    Serial.println(values[1]);
    Serial.println(values[2]);
    Serial.println(values[3]);
    return true;
  }

  // Unknown packet
  return false;
}

