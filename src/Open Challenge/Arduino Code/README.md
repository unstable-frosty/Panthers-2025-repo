# Arduino → EV3 Gyro Bridge (BNO08x IMU)

This project turns an Arduino-compatible board (such as an ESP32) into a **virtual EV3 Gyro Sensor**. It reads orientation data from a **BNO08x IMU** and sends a **continuous yaw angle** to the LEGO Mindstorms EV3 over I²C. In practice, this allows you to use a modern, high-performance IMU as if it were an official EV3 gyro sensor, but with several advantages: it supports unlimited rotation (a continuous multi-turn angle), provides a high update rate, includes automatic error recovery, and offers a simple re-zero command. This makes it especially well suited for WRO and other open-category competitions where robustness and repeatability over long runs are critical.

---

# 1. Overview

At a high level, this firmware acts as a **protocol translator** between a modern IMU and the LEGO EV3 brick. On one side, it communicates with the **BNO08x** IMU using I²C and consumes its rotation vector (quaternion) output. On the other side, it behaves like a simple I²C sensor from the perspective of the EV3, exposing only a 4-byte signed value that represents the current heading.

The firmware continuously reads quaternion orientation from the BNO08x and converts that quaternion into a yaw angle in degrees. It then integrates small changes in yaw over time to produce a continuous turning angle that can grow beyond the usual −180°…+180° or −360° ranges. This continuous angle is maintained internally and, whenever the EV3 performs an I²C read, the firmware returns the current value as a 32-bit integer representing yaw in centi-degrees (angle × 100). From the EV3’s point of view, this looks like a highly capable gyro sensor that never “wraps around” during multi-turn rotations.

---

# 2. Hardware & Pin Configuration

```cpp
#define SLAVE_ADDRESS 0x04   // I²C address seen by the EV3

#define EV3_SDA 12           // EV3 I²C SDA pin on the Arduino/ESP32
#define EV3_SCL 11           // EV3 I²C SCL pin on the Arduino/ESP32

TwoWire EV3Wire = TwoWire(1); // I²C instance used for EV3 side

BNO08x imu;
uint8_t currentAddr = 0x4A;   // BNO08x I²C address (fallback to 0x4B)
const uint16_t reportIntervalMs = 20;
const uint32_t noDataTimeoutMs = 1000;
uint32_t lastEventMs = 0;
```

These definitions set up the core configuration of the system. The `SLAVE_ADDRESS` constant determines which I²C address the EV3 will use when talking to this board; from the EV3’s perspective, this looks like the address of a standard I²C sensor. The `EV3_SDA` and `EV3_SCL` constants specify which pins on the Arduino or ESP32 are physically connected to the EV3 sensor port for I²C communication, so they must match your wiring.

The `EV3Wire` object creates a dedicated I²C bus instance for the EV3 side, completely separate from the I²C bus used to talk to the IMU. This separation improves reliability and makes debugging easier because traffic to and from the EV3 cannot interfere directly with traffic to the IMU. The `imu` object represents the BNO08x sensor itself, while `currentAddr` stores the I²C address currently in use for that IMU, defaulting to `0x4A` but allowing a fallback to `0x4B` if necessary.

Finally, the timing-related variables `reportIntervalMs`, `noDataTimeoutMs`, and `lastEventMs` control how often the IMU provides data and how long the firmware will tolerate silence before concluding that communication has failed. Together, they form the foundation for the sensor’s automatic recovery mechanism.

---

# 3. Yaw State & Quaternion Helper Functions

```cpp
volatile float accumYaw = 0.0f;
float yawOffset = 0.0f;
float lastYaw = 0.0f;
bool firstYawSet = false;

volatile int lastCmd = 0;
volatile bool cmdReady = false;

static inline float rad2deg(float r) { return r * 180.0f / PI; }

static float quatToYawDeg(float w, float x, float y, float z) {
  float s = 2.0f * (w * z + x * y);
  float c = 1.0f - 2.0f * (y * y + z * z);
  return rad2deg(atan2f(s, c));
}
```

These variables and helper functions maintain the yaw-tracking state and handle quaternion math. The variable `accumYaw` holds the continuous yaw angle in degrees and can grow beyond a single revolution, representing the total net rotation since initialization or the last reset. The `yawOffset` value stores an initial yaw reference so that the first valid IMU reading becomes the logical zero angle for the robot. The `lastYaw` variable remembers the previous wrapped yaw reading, and `firstYawSet` indicates whether a valid yaw reference has been established yet.

The variables `lastCmd` and `cmdReady` implement a small command mailbox for the EV3. Whenever the EV3 writes a command byte to this sensor over I²C, the `receiveData()` callback stores it in `lastCmd` and sets `cmdReady` to `true`. The main loop then checks this flag and handles the command in a safe, non-interrupt context.

The function `rad2deg()` is a simple utility that converts radians to degrees, which keeps the rest of the code easier to read. The `quatToYawDeg()` function is responsible for extracting a yaw angle (in degrees) from a quaternion `(w, x, y, z)` provided by the IMU. It computes two intermediate values, `s` and `c`, using standard quaternion-to-Euler conversion formulas, and then calls `atan2f(s, c)` to obtain a yaw angle in radians before converting it to degrees.

---

# 4. IMU Reset, Configuration & Recovery

This part of the firmware is responsible for keeping the IMU alive and usable throughout a match, even when the environment is electrically noisy or cables are slightly unreliable. In competitive robotics, it is common for robots to experience brownouts, jerky cable movement, transient disconnections, or voltage dips when multiple motors start or stop at once. Without explicit handling of these cases, the IMU might freeze, reset internally, or stop producing data, which would make the yaw reading useless. The code in this section detects such problems and attempts to recover from them automatically.

## 4.1 Hardware Reset Placeholder

```cpp
void hwResetIfAvailable() {
  delay(300);  // placeholder for real hardware reset line if available
}
```

The `hwResetIfAvailable()` function is a placeholder for a potential hardware reset line connected to the BNO08x. Some breakout boards expose a dedicated reset pin that can be toggled by the microcontroller. Even if your board does not currently use a physical reset signal, this function provides a central place to add one later. For now, it simply introduces a brief delay, giving the IMU time to power-cycle or restart cleanly if a reset has occurred through other means.

## 4.2 IMU Report Configuration

```cpp
bool configureReports() {
  return imu.enableRotationVector(reportIntervalMs);
}
```

The IMU supports many different output report types, but this firmware is specifically interested in the **rotation vector**, which provides orientation as a fused quaternion. The `configureReports()` function enables this report at the interval specified by `reportIntervalMs`, which in this case is 20 milliseconds, corresponding to a 50 Hz update rate. This frequency is fast enough for smooth heading control on an EV3 robot, yet not so fast that it overloads the I²C bus or the microcontroller.

## 4.3 Starting the IMU (Initialization Sequence)

```cpp
bool startBNO() {
  bool ok = imu.begin(currentAddr, Wire, -1, -1);
  if (!ok) return false;

  configureReports();
  lastEventMs = millis();

  firstYawSet = false;
  accumYaw = 0.0f;
  lastYaw = 0.0f;

  return true;
}
```

The `startBNO()` function encapsulates the process of bringing the IMU into an operational state. It first calls `imu.begin()` with the current I²C address. If this fails, the function immediately returns `false`, signaling to the caller that the IMU could not be reached. If the call succeeds, the function then enables the rotation vector report, records the current time in `lastEventMs`, and resets the yaw state by clearing `accumYaw`, `lastYaw`, and `firstYawSet`. This ensures that each successful IMU startup begins from a known, consistent orientation reference.

## 4.4 Recovery Logic (Self-Healing)

```cpp
void recover() {
  hwResetIfAvailable();
  if (!startBNO()) {
    currentAddr = (currentAddr == 0x4A) ? 0x4B : 0x4A;
    hwResetIfAvailable();
    startBNO();
  }
}
```

The `recover()` function implements the self-healing behavior of the system. When called, it first performs a hardware reset delay by calling `hwResetIfAvailable()`, and then attempts to start the IMU using `startBNO()`. If that attempt fails, the code assumes that the IMU might be configured at the alternate I²C address, and it toggles `currentAddr` between `0x4A` and `0x4B`. It then calls the reset placeholder again and performs another attempt to start the IMU. In practice, this sequence allows the system to recover from transient faults, cable issues, or incorrect address assumptions without manual intervention. For a competition robot, this kind of automatic recovery greatly reduces the chance of losing a run due to a temporary sensor glitch.

---

# 5. EV3 I²C Protocol — Receiving Commands

```cpp
void receiveData(int byteCount) {
  while (EV3Wire.available() > 0) {
    lastCmd = EV3Wire.read();
    cmdReady = true;
  }
}
```

From the EV3’s point of view, this board behaves like a simple I²C sensor. The EV3 can write one or more bytes to the sensor as commands and can read bytes back as sensor values. The `receiveData()` function is registered as the I²C `onReceive` callback for the EV3-facing bus, so it is invoked automatically every time the EV3 writes data to this device.

Inside the function, the code reads all available bytes from the EV3 and stores the last one it sees in `lastCmd`. This design assumes that the EV3 will send a single command code per transaction, which is typical for EV3-style sensors. By keeping only the most recent byte and ignoring any others, the code remains simple and avoids needing a complex protocol parser. After reading the byte, the function sets `cmdReady` to `true`, indicating to the main loop that a new command is waiting to be processed. The main loop then reads `lastCmd` and reacts accordingly, for example by re-zeroing the yaw when it receives a specific command code.

This approach keeps the interrupt-like callback very short and avoids doing any heavy processing while I²C communication is in progress. That, in turn, helps prevent bus lock-ups or interference with other sensors that might be sharing the EV3’s I²C lines.

---

# 6. EV3 I²C Protocol — Sending Yaw Data

```cpp
void sendData() {
  int32_t centi = (int32_t)roundf(accumYaw * 100.0f);

  uint8_t buf[4];
  buf[0] = (uint8_t)(centi & 0xFF);
  buf[1] = (uint8_t)((centi >> 8) & 0xFF);
  buf[2] = (uint8_t)((centi >> 16) & 0xFF);
  buf[3] = (uint8_t)((centi >> 24) & 0xFF);

  EV3Wire.write(buf, 4);
}
```

The `sendData()` function is the counterpart to `receiveData()` and is registered as the `onRequest` callback for the EV3-side I²C bus. The EV3 triggers this callback whenever it performs a read operation on the sensor. At that moment, the firmware must supply the current yaw value formatted as four bytes.

First, the function multiplies the continuous yaw angle `accumYaw` by 100 and rounds it to the nearest integer, producing a signed 32-bit value in centi-degrees. This representation preserves two decimal places of precision without requiring floating point on the EV3 side. It then packs this 32-bit integer into a four-byte buffer in little-endian order, placing the least significant byte at index zero. Finally, it writes those four bytes onto the I²C bus so the EV3 can read them in a single transaction.

On the EV3, reconstructing the original yaw value is straightforward: the program combines the four bytes into a signed 32-bit integer, then divides by 100.0 to recover the angle in degrees. Because this value comes from `accumYaw`, it represents the same continuous heading that the firmware maintains internally and can grow beyond ±360° as the robot rotates multiple times.

---

# 7. setup() — Initializing Everything

```cpp
void setup() {
  Serial.begin(115200);

  EV3Wire.begin(SLAVE_ADDRESS, EV3_SDA, EV3_SCL, 100000);
  EV3Wire.onReceive(receiveData);
  EV3Wire.onRequest(sendData);

  Wire.begin();
  Wire.setClock(400000);

  hwResetIfAvailable();

  if (!startBNO()) {
    currentAddr = 0x4B;
    hwResetIfAvailable();
    startBNO();
  }
}
```

The `setup()` function configures both the EV3-facing I²C bus and the IMU-facing I²C bus, prepares serial debugging, and initializes the BNO08x IMU. It first opens a serial port at 115200 baud, which is extremely useful for debugging and verifying sensor behavior during development.

Next, it initializes the EV3-facing I²C bus by calling `EV3Wire.begin()` with the slave address and the SDA/SCL pin assignments. The chosen clock rate is 100 kHz, which is compatible with the EV3’s sensor port. The code then attaches the `receiveData()` callback for EV3 write operations and the `sendData()` callback for EV3 read operations, enabling the sensor to react automatically whenever the EV3 communicates.

For the IMU, the firmware uses the primary `Wire` I²C bus, which is initialized with `Wire.begin()` and set to 400 kHz using `Wire.setClock(400000)`. This higher speed helps achieve smooth and timely updates from the BNO08x. After a brief hardware reset delay, the code calls `startBNO()` to initialize the IMU at the default address. If that attempt fails, the address is switched to `0x4B`, and the IMU is reset and started again. This provides robustness in case the BNO08x module is configured at a different address than expected.

---

# 8. Main Loop — Commands, IMU Events & Yaw Integration

The main loop of the firmware orchestrates three core responsibilities: it handles any pending commands from the EV3, it processes all available events from the IMU, and it maintains a watchdog timer that detects when the IMU has stopped sending data. Together, these operations keep the continuous yaw estimate up to date and ensure that the sensor recovers from faults automatically.

## 8.1 Processing EV3 Commands

```cpp
void loop() {

  // 1. Handle EV3 command(s)
  if (cmdReady) {
    int cmd = lastCmd;
    cmdReady = false;

    if (cmd == 0x01) { // Re-zero yaw
      firstYawSet = false;
      accumYaw = 0.0f;
      lastYaw  = 0.0f;
    }
  }
```

At the start of each iteration, the loop checks whether a new command has arrived from the EV3 by looking at `cmdReady`. If this flag is set, it copies `lastCmd` into a local variable, clears the flag, and then interprets the command. In this example, the command `0x01` is reserved for re-zeroing the yaw. When that command is received, the firmware clears the yaw state, setting `firstYawSet` to `false` and resetting both `accumYaw` and `lastYaw` to zero. On the next valid IMU reading, the current physical orientation will become the new zero reference. This is particularly useful in competition runs where the robot is aligned to a wall or a line before starting.

## 8.2 Checking IMU Connectivity

```cpp
  // 2. Check IMU connectivity
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
```

After processing commands, the loop checks whether the IMU is still connected by calling `imu.isConnected()`. If the IMU appears disconnected, the code throttles recovery attempts so that it only tries to recover once per second, using a static timestamp `lastTry`. Within this disconnected state, the function calls `recover()` when the interval has elapsed, then briefly delays and returns early. This prevents the rest of the loop from using invalid IMU data and provides a clean reentry once the IMU is back online.

## 8.3 Reading IMU Events and Handling Resets

```cpp
  // 3. Read all available IMU events
  while (imu.getSensorEvent()) {
    gotAny = true;

    // 3a. IMU has reset - reconfigure and zero again
    if (imu.wasReset()) {
      configureReports();
      firstYawSet = false;
      accumYaw = 0.0f;
      lastYaw  = 0.0f;
    }

    // 4. Process rotation vector (quaternion → yaw)
    if (imu.sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float rawYaw = quatToYawDeg(
        imu.getQuatReal(),
        imu.getQuatI(),
        imu.getQuatJ(),
        imu.getQuatK()
      );
```

The loop then enters a `while` loop that repeatedly calls `imu.getSensorEvent()` to pull all pending events from the IMU. This ensures that no events pile up and that the heading estimate remains responsive. Whenever at least one event is retrieved, the `gotAny` flag is set to `true` so the watchdog knows that data is still flowing.

If the IMU indicates that it has reset (for example, due to an internal watchdog or power dip), `imu.wasReset()` will return `true`. In this case, the firmware re-enables the rotation vector reports by calling `configureReports()` and resets the yaw state, effectively treating the IMU reset as a fresh start. For each rotation vector event, the quaternion is converted into a raw yaw angle using `quatToYawDeg()`, which becomes the input to the yaw integration logic.

## 8.4 Yaw Initialization & Continuous Integration

```cpp
      // First valid sample sets yaw offset
      if (!firstYawSet) {
        yawOffset = rawYaw;
        lastYaw   = 0.0f;
        accumYaw  = 0.0f;
        firstYawSet = true;
      }

      // 5. Normalize yaw to [-180, 180]
      float yaw = rawYaw - yawOffset;
      while (yaw > 180) yaw -= 360;
      while (yaw < -180) yaw += 360;

      // Compute delta, handling wrap-around
      float delta = yaw - lastYaw;
      if (delta > 180)  delta -= 360;
      if (delta < -180) delta += 360;

      // Integrate into continuous yaw
      accumYaw += delta;
      lastYaw = yaw;
    }
  }
```

The yaw processing logic begins by checking whether a valid yaw reference has been established. On the very first valid IMU sample, `firstYawSet` is still `false`, so the code stores the current `rawYaw` as `yawOffset` and resets `lastYaw` and `accumYaw` to zero. It then sets `firstYawSet` to `true`, so subsequent samples will be treated as relative angles around this initial reference.

For each subsequent event, the code subtracts `yawOffset` from `rawYaw` to obtain a relative yaw angle. It then normalizes this yaw to the range from −180° to +180° by adding or subtracting 360° as needed. This normalization makes the angle more numerically stable and easier to work with.

Next, the code computes the difference `delta` between the current yaw and the last yaw. Because yaw wraps around at ±180°, a naïve difference might appear large even when the actual movement was small. To correct this, the code adjusts `delta` whenever it exceeds 180° in magnitude by adding or subtracting 360°, effectively undoing the wrap-around. Finally, it adds `delta` to `accumYaw`, producing an integrated continuous angle, and stores the current yaw as `lastYaw` for the next iteration. Over time, this approach builds a continuous yaw value that can track multiple full rotations in either direction.

## 8.5 Watchdog: No Data Recovery

```cpp
  // 6. Watchdog: no data → attempt recovery
  if (gotAny) {
    lastEventMs = millis();
  } else if (millis() - lastEventMs > noDataTimeoutMs) {
    recover();
  }

  delay(1);
}
```

At the end of the loop, the firmware updates the watchdog. If at least one IMU event was received during this iteration, it updates `lastEventMs` with the current time. If no events were received and the time since `lastEventMs` exceeds `noDataTimeoutMs`, the firmware assumes that the IMU has stopped sending data and calls `recover()` to restart it. A small delay of one millisecond is added to yield to other tasks and to avoid a tight, busy loop. This watchdog mechanism helps protect against silent failures where the IMU is still “connected” but no longer reporting data.

---

# 9. Yaw Normalization & Infinite Rotation (Detailed)

Internally, the BNO08x provides yaw as a wrapped angle, typically confined to the range between −180° and +180°. This is sufficient if you only care about the current physical orientation, but it is not enough when you want to know how many times a robot has rotated or to maintain a continuous heading value over long paths. The firmware addresses this gap by computing the difference between successive yaw measurements and integrating these differences over time.

On the first valid sample, the firmware records the initial yaw as `yawOffset` and treats this as the zero direction. Every subsequent raw yaw value is shifted by this offset, then normalized back into the −180°…+180° range. The difference between the current normalized yaw and the previous one is then computed. If that difference appears to be larger than 180° in magnitude, the code adjusts it by adding or subtracting 360°, which corrects for the wrap-around. That corrected difference is then added to `accumYaw`. Mathematically, the continuous yaw is the sum of all these corrected deltas, so as the robot spins multiple times, `accumYaw` grows accordingly and can represent multi-turn rotations without ever resetting.

---

# 10. Using the Sensor on the EV3 Side

From the EV3’s perspective, this device behaves like a simple I²C sensor that returns a four-byte signed integer. To read the current angle, the EV3 program performs an I²C read of four bytes from the configured sensor address. These bytes should be combined into a signed 32-bit integer using little-endian order (the first byte is the least significant). Once reconstructed, the value represents the angle in centi-degrees, so dividing it by 100.0 yields the yaw in standard degrees.

To reset the angle on demand, the EV3 writes a single byte with value `0x01` to the same I²C address. The firmware interprets this as a re-zero command, clears the yaw state, and treats the next valid IMU reading as the new zero direction. In most EV3 programming environments, such as EV3-G, Python on EV3, EV3Dev, or Pybricks, these reads and writes can be implemented using low-level I²C primitives, and once wrapped in a helper block or function, the sensor can be used much like a native gyro sensor that supports continuous headings.

---

# 11. Key Code Snippets (Quick Reference)

For quick reference, the following snippets highlight the most important structural elements of the firmware.

The core hardware and object definitions are:

```cpp
#define SLAVE_ADDRESS 0x04
#define EV3_SDA 12
#define EV3_SCL 11

TwoWire EV3Wire = TwoWire(1);
BNO08x imu;
uint8_t currentAddr = 0x4A;
```

The EV3 command reception callback is:

```cpp
void receiveData(int b) {
  while (EV3Wire.available()) {
    lastCmd = EV3Wire.read();
    cmdReady = true;
  }
}
```

The yaw transmission callback is:

```cpp
void sendData() {
  int32_t centi = (int32_t)roundf(accumYaw * 100);
  uint8_t b[4] = {
    (uint8_t)(centi & 0xFF),
    (uint8_t)((centi >> 8) & 0xFF),
    (uint8_t)((centi >> 16) & 0xFF),
    (uint8_t)((centi >> 24) & 0xFF)
  };
  EV3Wire.write(b, 4);
}
```

And the quaternion-to-yaw conversion helper is:

```cpp
static float quatToYawDeg(float w, float x, float y, float z) {
  float s = 2 * (w * z + x * y);
  float c = 1 - 2 * (y * y + z * z);
  return rad2deg(atan2f(s, c));
}
```

These pieces together define how the hardware is wired, how the EV3 talks to the sensor, and how the IMU’s quaternion output is turned into a usable heading.

---

# 12. Notes for Future WRO Engineers

This firmware is designed with competition conditions in mind. It is built to tolerate robot crashes, shaking cables, slightly loose connectors, and electrical noise from high-current motors, all while maintaining a valid yaw output. The automatic recovery system continuously monitors the IMU and attempts to restart it whenever communication is lost, which reduces the risk of losing a match because the gyro stopped responding.

During development or troubleshooting, you can use the serial monitor to print IMU event rates, inspect yaw values and deltas, detect unexpected IMU resets, and monitor how frequently the recovery system is activated. Observing the I²C bus usage on the EV3 side can also help prevent situations where too many sensors compete for bandwidth.

The firmware is compatible with common EV3 programming environments, including EV3-G, EV3 Python, EV3Dev, and Pybricks, and is well suited for WRO Open robots that need robust and precise heading control. If you want to extend the EV3 command protocol, you can easily add new command bytes such as `0x02` for a soft calibration, `0x03` for changing the update rate, `0x04` for freezing or unfreezing the output, or `0x05` for directly setting the yaw. Implementing these features typically requires only small additions to the command handling logic and corresponding changes in the main loop.

---

# END

