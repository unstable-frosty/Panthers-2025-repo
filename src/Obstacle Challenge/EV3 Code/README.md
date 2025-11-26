# 📘 Clever – WRO Future Engineers  


# ⚙️ 1. System Overview

The Clever robot uses a hybrid sensing and control architecture to remain stable and responsive throughout the competition course.  
The EV3 brick acts as the main processing unit, while an Arduino board provides high-frequency gyro readings and camera-based object detection.  
Together, they form a system where:

- The **Arduino** captures:
  - High-resolution yaw data (converted to degrees)
  - Object center position and bounding box
  - Visual obstacle detection signal (`sig`)
- **Two ultrasonic sensors** measure distances to the left and right walls. These help with:
  - Maintaining central alignment inside the corridor  
  - Triggering emergency avoidance if too close  
- A **color sensor** recognizes colored floor segments. These markers signal:
  - When the robot must turn  
  - Whether to turn left or right  
- The **steering system** uses filtered gyro and ultrasonic data to:
  - Correct heading  
  - Keep the robot centered  
  - Execute smooth turns  

This multi-sensor fusion allows the robot to drive reliably while adapting to obstacles and route changes.

---

# 🧭 2. Program Structure

The software is organized into logical layers that continuously interact during execution.

### These layers include:

- **Initialization:**  
  Sets up all variables, target states, sensor offsets, and controller constants.

- **Multithreaded sensing:**  
  - Ultrasonic thread  
  - Gyro + camera thread  
  These threads run in parallel to provide real-time data updates.

- **Startup logic:**  
  The robot calibrates steering, centers itself, and detects which direction (CW or CCW) to follow based on wall distance.

- **Corridor entry:**  
  A controlled gyro-guided 90° turn aligns the robot with the main corridor.

- **Main navigation loop:**  
  Executes continuous adjustments, obstacle avoidance, and color-based turn handling.

- **Final sprint:**  
  Uses pure gyro control and combined control to complete the final straight path.

This modular structure allows the robot to transition smoothly between driving states.

---

# 🟦 3. Variable Initialization

Before driving begins, the robot defines a wide set of global variables. These variables carry data such as sensor readings, controller parameters, and navigation state.

### Key groups of variables:

- **Navigation direction and heading:**
  - `cw`: determines course direction (CW = 1, CCW = 0)
  - `gyro`: filtered current heading  
  - `target`: desired target heading for the gyro controller  

- **Distance measurements:**
  - `Ldistance` and `Rdistance`: low-pass filtered ultrasonic distances  

- **Obstacle and camera data:**
  - `sig`: object signal (6 = no obstacle, others = object detected)  
  - `camError`: lateral offset of detected object  
  - `mcx`, `mcy`, `mw`, `mh`: bounding box of detected object  

- **Obstacle avoidance controller:**
  - `ob_kp`, `ob_kd`: proportional & derivative gains  
  - `ob_corr`, `ob_old_err`: used for steering correction around obstacles  

Example snippet:

cw = 3  
gyro = 0  
Ldistance = 0  
Rdistance = 0  
ob_kp = 1.23  
ob_kd = 8.1  

These parameters configure all later behaviors.

---

# 📡 4. Ultrasonic Sensor Thread

The ultrasonic thread continuously reads the left and right ultrasonic sensors and applies smoothing to produce stable measurements.

### How it works:

- The thread repeatedly reads raw data from both ultrasonic sensors.
- A **low-pass filter** blends old and new readings:
  - Smooths noise  
  - Prevents sudden spikes  
- Every few iterations, the filter resets to the raw value to avoid lag.
- Updated `Ldistance` and `Rdistance` values are sent to:
  - Wall-following controller  
  - Emergency avoidance logic  
  - Direction selection logic (CW/CCW)

This thread ensures the robot always has reliable and responsive corridor distance measurements.

---

# 🎥 5. Gyro + Camera Thread

This thread reads all sensor information from the Arduino, covering both gyro orientation and camera-based object detection.

### Gyro processing:

- The Arduino outputs yaw in centi-degrees (e.g., 1523 = 15.23°).
- The code:
  - Converts this value into degrees  
  - Applies low-pass filtering  
  - Periodically resets the filter to avoid drift  

### Camera processing:

- The Arduino also sends:
  - Object lateral error (`camError`)
  - Detected object type (`sig`)
  - Object center and size (`mcx`, `mcy`, `mw`, `mh`)

These are essential for:

- Determining if an obstacle is blocking the path  
- Steering around objects  
- Triggering bypass maneuvers  

This thread forms the robot’s “eyes” and “inner ear.”

---

# 🔊 6. Optional Beep Thread

The beep thread is used for debugging or auditory feedback during test runs.

### Behavior:

- When certain conditions occur (e.g., `con = 1`), the robot emits periodic tones.
- These tones help indicate:
  - Special modes  
  - Debug information  
  - Safety states  

This thread is optional but helpful during testing.

---

# 🚗 7. Startup Sequence

Before entering the corridor, the robot performs several critical calibration steps:

### Steps include:

- **Gyro reset:**  
  Ensures orientation readings begin from a known reference.

- **Steering centering:**  
  Moves the steering motor to its neutral center position.

- **Initial straight drive:**  
  Ensures wheels align under load, helping improve driving consistency.

- **Thread startup:**  
  Begins the ultrasonic and gyro/camera threads so real-time data is available.

- **Direction selection:**  
  The robot compares left and right distances:  
  - If the left side has more space, it chooses CCW (`cw = 0`)  
  - Otherwise it chooses CW (`cw = 1`)  

This ensures the robot starts driving with correct alignment and strategy.

---

# ↩️ 8. Entry Turn into the Corridor

After selecting a direction, the robot must align itself with the corridor by performing a precise 90-degree turn.

### How it works:

- The robot starts moving forward slightly.
- Then, using gyro feedback, it rotates to:
  - **+90°** if driving CCW  
  - **−90°** if driving CW  
- This is controlled using:

CAR.update_gyro_control(±90, ...)

By the end of this phase, the robot is perfectly aligned with the route ahead.

---

# 🛣️ 9. Main Corridor Loop

The main navigation loop repeats until the robot has executed **12 color-based turns**.

### Inside this loop, the robot performs:

- **Color sensing** to detect when to turn  
- **Obstacle avoidance** using camera data  
- **Wall-following** using ultrasonic sensors  
- **Heading correction** using gyro feedback  
- **Turn counting** to track progress  

This loop is active for most of the challenge.

---

## 9.1 Obstacle Avoidance Using Camera

When an obstacle is detected (`sig ≠ 6`), the robot switches from wall-following to visual-based bypass.

### This involves:

- Using the object’s bounding box to determine size and position  
- Computing lateral correction using a PD controller:

CAR.OB_AV(centerPos, camError, sig, ob_count, mcx, mcy, mw, mh, ob_kp, ob_kd, ob_corr, ob_old_err)

- Steering away from the obstacle smoothly  
- Repositioning itself after bypass completion  

This enables dynamic, intelligent movement around obstacles.

---

## 9.2 Emergency Ultrasonic Quick Pull

If the robot gets dangerously close to the wall (<5 cm), it performs an immediate escape maneuver.

### This maneuver includes:

- A brief acceleration to create space  
- A sharp steering adjustment of ±55°  
- A safety beep  
- Resetting back to normal power afterward  

This protects the robot from collisions with corridor walls.

---

## 9.3 Normal Corridor Keeping

When no obstacles or emergencies are present, the robot relies on combined gyro and distance control.

### This controller:

- Maintains the robot at a specific distance (~40 cm) from the chosen wall  
- Holds the correct target heading using gyro data  
- Smooths motion using weighted error correction  

This mode is responsible for consistent and stable driving.

Example controller call:

CAR.update_gyro_dis(...)

---

## 9.4 Color-Based Turning

The robot uses color markers printed on the floor to determine turn points.

### Turning rules:

- **Orange marker (2)** → turn right if driving CW  
- **Blue marker (3)** → turn left if driving CCW  

Example logic:

If clr = 2 And cw = 1 Then  
  target = target - 90  
  t_count++  

The robot performs exactly **12 such turns** before exiting the loop.

---

# 🏁 10. Final Straight Run

The final stretch of the course requires stability and precision.

### Final phase steps:

- **Gyro-only stabilization:**  
  Maintains a fixed heading while accelerating.

- **Combined control refinement:**  
  Uses gyro + distance control to stay aligned and smooth.

- **Motion completion conditions:**  
  Ends when:
  - A specific tacho count is reached  
  - Or a time limit expires  

- **Final angle display:**  
  The robot prints its final orientation on the screen:

LCD.Text("Angle: " + gyro)

This gives a visual confirmation of stability at the end.

---

# 📄 11. Summary

The Clever robot uses a well-structured control system that combines multiple sensors and algorithms.  
In summary, the robot:

- Filters and processes gyro, ultrasonic, and camera inputs in real time  
- Makes intelligent driving decisions using sensor fusion  
- Avoids obstacles smoothly and dynamically  
- Follows walls and maintains central position  
- Executes 12 precise color-based turns  
- Finishes with a stable, controlled final sprint  


