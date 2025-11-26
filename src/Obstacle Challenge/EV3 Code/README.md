# WRO Future Engineers – EV3 Navigation + Obstacle Avoidance
## Complete Technical README (with Full Source Code)

This README documents the **full navigation and obstacle-avoidance program** written in Clev3r for EV3, used in the WRO Future Engineers Open Challenge.

It is intended for:
- Future team members learning how the robot works  
- Judges or mentors reviewing the implementation  
- Anyone who wants to extend, debug, or port this logic to another platform  

The document is split into:
1. Project structure and imports  
2. Global variables and system state  
3. Ultrasonic distance thread  
4. Gyro + camera thread (Arduino bridge)  
5. Initialization and steering alignment  
6. Direction selection and initial rotation  
7. Main navigation loop (with obstacle avoidance)  
8. Color-based turning logic  
9. End-phase sequence  
10. Full source code  

---

## 1. Project Structure and Module Imports

The program begins by declaring the project folder and importing the custom modules that provide sensor and motion utilities:

```vb
folder "prjs""Clever"
import "Mods\HTColorV2"
import "Mods\arduino"
import "Mods\CAR"
import "Mods\Ultrasonic"
import "Mods\Tool"
```

### Module overview

- **HTColorV2**  
  Provides robust RGB readings and color classification. It is used to detect the orange and blue markers on the field that trigger 90° turns.

- **arduino**  
  Handles communication with an Arduino that sends:
  - Gyro yaw (`yawC`)
  - Camera error (`camError`)
  - Object signature (`sig`)
  - Bounding box information (`mcx`, `mcy`, `mw`, `mh`)  

  This allows the EV3 to use an external IMU + camera as if they were native sensors.

- **CAR**  
  A high-level motion library that encapsulates:
  - Steering control and centering
  - Gyro-based straight driving (`update_gyro_control`)
  - Combined gyro + distance wall following (`update_gyro_dis`)
  - Low-pass filters for noisy signals
  - Camera-based obstacle avoidance helper (`OB_AV`)

- **Ultrasonic**  
  Provides distance readings in centimeters from ultrasonic sensors on ports 2 and 4. These are used both for wall following and emergency “side yank” escape maneuvers.

- **Tool**  
  A collection of helper utilities (timing, math, etc.) that support the rest of the system.

By isolating hardware-specific logic into modules, the main script can focus on strategy instead of raw motor commands.

---

## 2. Global Variables and System State

Next, the program declares a set of global variables that are shared between the main code and the threads. They store the current navigation state, filtered sensor values, and controller memory.

```vb
cw = 3
gyro = 0
Ldistance = 0
Rdistance = 0
t_count = 0
target = 0
DIR = 0
t_distance = 40
repeat_straight = 0
con = 0
' loop counter for obstacle low-pass reset
ob_count = 0
```

### Navigation and distance state

- **`cw`** – direction mode  
  - `3` = not yet decided  
  - `0` or `1` = fixed direction (CCW vs CW) chosen based on walls

- **`gyro`** – filtered gyro heading used by the PID controllers.

- **`target`** – desired heading in degrees. Every time the robot performs a 90° turn, this value is increased or decreased by 90.

- **`t_count`** – number of 90° turns that have already been executed. The main loop runs until `t_count = 12`.

- **`Ldistance`, `Rdistance`** – current filtered distances from the ultrasonic sensors on the left and right.

- **`t_distance`** – target wall distance for the combined gyro + distance controller.

- **`DIR`** – auxiliary direction flag used inside the CAR library for wall-following logic.

- **`repeat_straight`** – used during initialization to repeatedly drive straight and settle mechanical slack in the steering linkage.

- **`con`** – a flag originally used for beeping logic, still available for diagnostics.

- **`ob_count`** – a loop counter passed into the obstacle avoidance function to allow periodic internal resets.

```vb
' ===== camera / obstacle variables =====
yawC = 0
camError = 0
sig = 6         ' 6 = no obstacle, anything else = obstacle / object
mcx = 0
mcy = 0
mw = 0
mh = 0

' gyro raw values
gyroRaw = 0
rawGyro = 0

' ===== obstacle avoidance controller state =====
ob_kp = 1.23
ob_kd = 8.1
ob_corr = 0
ob_old_err = 0
```

### Camera and obstacle variables

These are all updated by the Arduino thread:

- **`yawC`** – raw yaw angle from the external IMU (as an integer scaled by 100).
- **`camError`** – horizontal camera error of the tracked object relative to the center of the image (used for steering).
- **`sig`** – signature or mode from the camera system.  
  - `6` = *no obstacle present*  
  - any other value = *object detected* in front of the robot.
- **`mcx`, `mcy`** – center of the obstacle bounding box.
- **`mw`, `mh`** – width and height of the obstacle bounding box.

### Gyro raw values

- **`gyroRaw`**, **`rawGyro`** – intermediate variables used in filtering the gyro signal.

### Obstacle avoidance controller

- **`ob_kp`, `ob_kd`** – proportional and derivative gains for the camera-based obstacle avoidance PID.
- **`ob_corr`** – current correction output.
- **`ob_old_err`** – previous error value, used to calculate the derivative term.

By organizing the system state in this way, the robot can combine gyro, ultrasonic, and camera information to produce smooth and robust behavior.

---

## 3. Ultrasonic Sensor Thread

The ultrasonic sensors are read continuously in a dedicated thread. This ensures that distance data is always fresh, even when the main loop is busy with other tasks like turning or obstacle avoidance.

```vb
Sub ReadUltrasonic
  alphaDist = 0.9
  counter = 0

  ' Initialize once
  Ultrasonic.getCm(2, Ldistance)
  Ultrasonic.getCm(4, Rdistance)

  While 1=1
    Ultrasonic.getCm(2, rawLdistance)
    Ultrasonic.getCm(4, rawRdistance)

    CAR.lowpass(rawLdistance, Ldistance, alphaDist, Ldistance)
    CAR.lowpass(rawRdistance, Rdistance, alphaDist, Rdistance)

    ' Reset every 5 loops
    counter = counter + 1
    If counter >= 5 Then
      Ldistance = rawLdistance
      Rdistance = rawRdistance
      counter = 0
    EndIf

  EndWhile
EndSub
```

### How it works

1. **Initialization**  
   The first distance readings from ports 2 and 4 are stored in `Ldistance` and `Rdistance`.

2. **Continuous loop**  
   Inside the `While 1=1` loop, the thread:
   - Reads raw distances into `rawLdistance` and `rawRdistance`.
   - Passes them through a low-pass filter using `CAR.lowpass`:
     - This smooths out short spikes and noise.
   - Increments `counter`, and every 5 iterations it *resets* the filtered values to the raw readings.

3. **Why reset the filter periodically?**  
   Over time, a low-pass filter can lag behind sudden real changes. By forcing a reset every few cycles, the robot:
   - Retains the benefits of smoothing.
   - Avoids large delays when an obstacle suddenly appears or disappears.

This thread feeds stable distance data to:
- the wall-following controller, and
- the emergency side-escape logic.

---

## 4. Gyro + Camera Thread (Arduino Bridge)

The second background thread reads both gyro and camera data from the Arduino using a single call, `arduino.ReadArduinoFull`. This keeps orientation and visual information synchronized and up to date.

```vb
Sub ReadGyro
  alphaGyro = 0.6
  counter = 0

  ' Initialize once using Arduino full data (gyro + camera)
  arduino.ReadArduinoFull(1, yawC, camError, sig, mcx, mcy, mw, mh)
  gyroRaw = yawC
  gyro = (gyroRaw / 100) - offset

  While 1=1
    ' Read full data: yaw + camera information
    arduino.ReadArduinoFull(1, yawC, camError, sig, mcx, mcy, mw, mh)
    rawGyro = (yawC / 100) - offset

    ' Apply low-pass filter (same logic as before)
    CAR.lowpass2(rawGyro, gyro, alphaGyro, gyro)

    ' Reset filter every 10 loops (same logic as before)
    counter = counter + 1
    If counter >= 10 Then
      gyro = rawGyro        ' reset to current sensor reading
      counter = 0
    EndIf

  EndWhile
EndSub
```

### Step-by-step behavior

1. **Initial read**  
   - The thread calls `arduino.ReadArduinoFull` once to populate:
     - `yawC` (gyro yaw)
     - `camError`, `sig`, `mcx`, `mcy`, `mw`, `mh` (camera data)
   - It converts `yawC` (which is scaled by 100) into degrees and subtracts `offset` to define the current heading as zero.

2. **Continuous loop**  
   Inside the infinite loop:
   - `ReadArduinoFull` is called repeatedly to update all gyro + camera values.
   - The raw yaw (`rawGyro`) is converted to degrees and filtered using `CAR.lowpass2`.
   - Every 10 iterations, the filtered `gyro` is reset to the raw value to remove accumulated filtering bias.

3. **Why combine gyro and camera in one thread?**  
   - Both signals are time-critical.
   - Using a single read call keeps them synchronized and lowers communication overhead.
   - The main loop always has access to recent gyro and obstacle data without blocking.

This thread is critical for:
- Heading control (straight driving and turning)
- Camera-based obstacle avoidance
- Final end-phase diagnostics (printing the final angle)

---

## 5. Beep Thread (Optional Diagnostic)

There is an optional `beep` thread that can be used for acoustic feedback when certain conditions are met.

```vb
Sub beep
  LCD.Clear()
  While 1=1
    If con = 1 Then
      Speaker.Tone(100, 500, 100)
    EndIf
  EndWhile
EndSub
```

Currently, the line that starts this thread is commented out, so it is not active in the main behavior. However, it is useful for debugging:
- Setting `con = 1` in specific situations can make the robot beep continuously, signaling a particular mode or condition.

---

## 6. Initialization and Steering Alignment

Before starting complex navigation, the robot performs several initialization steps:

```vb
LCD.Clear()
''''''''''''''''''''''centering steering position'''''''''''''''''''''''
CAR.reset_gyro(offset)
Program.Delay(300)
CAR.center_steer(centerPos)
Program.Delay(50)

While repeat_straight < 1000
  CAR.move_steering(centerPos,0,lasterror)
  repeat_straight = repeat_straight+1
EndWhile
```

### What happens here

1. **Clear display**  
   The LCD is cleared so that any upcoming debug messages can be seen clearly.

2. **Gyro reset**  
   `CAR.reset_gyro(offset)` calibrates and stores a gyro offset so that the current physical orientation becomes the zero-degree reference.

3. **Center steering**  
   `CAR.center_steer(centerPos)` mechanically centers the steering motor, defining the neutral steering position (`centerPos`).

4. **Steering “settling”**  
   The robot repeatedly calls `CAR.move_steering(centerPos, 0, lasterror)` 1000 times:
   - This moves the steering into the centered position under closed-loop control.
   - It helps to remove mechanical slack or backlash.

This initialization step improves:
- Straight-line accuracy  
- Turn repeatability  
- Overall consistency between runs  

---

## 7. Starting the Sensor Threads

Once initialization is complete, the program starts both sensor threads:

```vb
LCD.Clear()
Program.Delay(100)
Thread.Run = ReadUltrasonic
Thread.Run = ReadGyro
'Thread.Run = beep
Program.Delay(100)
```

- `ReadUltrasonic` continuously updates `Ldistance` and `Rdistance`.
- `ReadGyro` continuously updates `gyro`, `camError`, `sig`, and bounding box data.
- The `beep` thread can be enabled by uncommenting its line if needed.

The short delays (`Program.Delay(100)`) give the threads time to start and begin populating global variables.

---

## 8. Direction Selection and Initial Rotation

Before the main navigation loop, the robot chooses whether it will navigate in a clockwise or counter-clockwise pattern, based on the initial distances to the walls:

```vb
'''''''''''''''''''''''''''''''''determine direction''''''''''''''''''
If Ldistance > Rdistance Then
  cw = 0
Else 
  cw = 1
EndIf
```

- If the left wall is farther than the right (`Ldistance > Rdistance`), then `cw = 0`.
- Otherwise, `cw = 1`.

This provides an automatic way to decide which side of the corridor to follow.

Next, the robot performs a short backward motion and a 90° rotation using gyro control:

```vb
Motor.ResetCount("C")
While Math.Abs(Motor.GetCount("C"))<400
  CAR.move_steering(centerPos,0,lasterror)
  Motor.StartPower("C",-50)
EndWhile
Motor.StartPower("C",0)
Speaker.Tone(100, 4000, 100)
Motor.ResetCount("C")
Motor.StartPower("C",50)
While Math.Abs(Motor.GetCount("C"))<550
  If cw = 0 Then
    CAR.update_gyro_control(90, 3, 8, 0,centerPos, gyro,filteredError,  gyroLastError)
  Else
    CAR.update_gyro_control(-90, 3, 8, 0,centerPos, gyro,filteredError,  gyroLastError)
  EndIf
EndWhile
Motor.StartPower("C",0)

Program.Delay(100)
Motor.StartPower("C",70)
```

### Breakdown

1. **Short backward movement**  
   - The robot drives backwards while keeping the steering centered.
   - This likely positions it correctly at the start line.

2. **Beep notification**  
   - After stopping, it plays a tone to indicate that the initial backing-up is complete.

3. **Initial 90° rotation**  
   - The robot drives forward while using `CAR.update_gyro_control` to rotate to:
     - +90° if `cw = 0`
     - −90° if `cw = 1`
   - This aligns the robot with the first corridor it will follow.

4. **Transition to normal speed**  
   - The drive motor is set to power 70, and the robot is ready to start the main navigation loop.

---

## 9. Main Navigation Loop (with Obstacle Avoidance)

The main loop runs until the robot has executed 12 turns:

```vb
While t_count<>12
  CAR.color_RGB(3,clr)

  ' ---------------------------------------
  ' Obstacle LPF loop counter
  ' Reset every 10 main-loop iterations
  ' ---------------------------------------
  ob_count = ob_count + 1
  If ob_count >= 10 Then
    ob_count = 0
  EndIf
```

At the start of each iteration:
- The color sensor on port 3 is read into `clr`.
- `ob_count` is incremented and reset every 10 loops (this value is passed into the obstacle avoidance routine for its own internal timing).

### 9.1 Camera-Based Obstacle Avoidance

If the Arduino reports that an obstacle is present (i.e., `sig <> 6`), the robot hands control to a dedicated obstacle avoidance routine in the CAR module:

```vb
  If sig <> 6 Then
    ' Use camera-based obstacle avoidance from CAR library
    ' NOTE: OB_AV signature (in CAR):
    ' OB_AV(centerPos, error, sig, count, mcx, mcy, mw, mh, kp, kd, corr, old_err)
    CAR.OB_AV(centerPos, camError, sig, ob_count, mcx, mcy, mw, mh, ob_kp, ob_kd, ob_corr, ob_old_err)
```

- `camError` tells the controller how far left or right the obstacle is.
- `mcx`, `mcy`, `mw`, `mh` give extra shape and position information.
- `ob_kp` and `ob_kd` define how aggressively the robot should respond.
- `ob_corr` and `ob_old_err` store internal state for the controller.

Within this branch, **the standard wall-following logic is temporarily suspended**, and the robot focuses purely on safe avoidance.

### 9.2 Ultrasonic “Side Yank” Escape Maneuvers

If there is **no camera obstacle** (`sig = 6`), the robot first checks for dangerous proximity to the side walls using the ultrasonic sensors:

```vb
  Else
    ' =====================================================
    ' Side ultrasonic "quick pull" logic with speed boost
    '   - If left < 5 cm  -> pull right (steering = 55)
    '   - If right < 5 cm -> pull left  (steering = -55)
    '   - During yank: speed = 100
    '   - After yank : speed = 70
    ' =====================================================

    If Ldistance < 5 Then
      Motor.StartPower("C",100)
      Motor.ResetCount("C")
      While Math.Abs(Motor.GetCount("C")) < 15
        CAR.move_steering(centerPos, -55, lastCombinedError)
      EndWhile
      Speaker.Tone(100, 4000, 100)
      Motor.StartPower("C",70)
    EndIf

    If Rdistance < 5 Then
      Motor.StartPower("C",100)
      Motor.ResetCount("C")
      While Math.Abs(Motor.GetCount("C")) < 15
        CAR.move_steering(centerPos, 55, lastCombinedError)

      EndWhile
      Speaker.Tone(100, 4000, 100)
      Motor.StartPower("C",70)
    EndIf
```

- If the **left** distance drops below 5 cm:
  - The robot briefly speeds up (`power = 100`) and steers strongly to the **right** (steering = −55).
- If the **right** distance drops below 5 cm:
  - The robot does the mirrored maneuver to the **left**.
- In both cases, a high-pitched beep is played as feedback, and the speed returns to 70 afterwards.

This logic acts as a last-second safety net against crashing into walls.

### 9.3 Normal Course-Keeping: Gyro + Distance Control

If there is no camera obstacle and no emergency side yank in progress, the robot uses its standard navigation logic:

```vb
    ' ================= ORIGINAL COURSE-KEEPING LOGIC =================
    If cw = 3 Then
      CAR.update_gyro_control(target, 0.5, 8, 0,centerPos, gyro,filteredError,  gyroLastError)
      con = 1
    Else
      con = 0
      CAR.select(cw,Rdistance,Ldistance,distance)
      If cw = 0 And target-gyro>30 Then
        'Motor.StartPower("C", 100)
        CAR.update_gyro_control(target, 1.2, 12 , 0,centerPos, gyro,filteredError,  gyroLastError)
      ElseIf cw = 1 And target - gyro < -30 Then
        'Motor.StartPower("C", 100)
        CAR.update_gyro_control(target, 1.2, 12 , 0,centerPos, gyro,filteredError,  gyroLastError)
      Else
        'Motor.StartPower("C", 50)
        CAR.update_gyro_dis(cw,target,gyro, t_distance, 1.2, 6, 1.5,6,centerPos,distance,gyroError,distError,lastCombinedError,DIR)
      EndIf
    EndIf
    ' ================= END ORIGINAL COURSE-KEEPING LOGIC =================
  EndIf
```

#### Behavior in detail

- If `cw = 3`, the direction hasn’t been fixed yet:
  - The robot simply uses `update_gyro_control` to stay near the `target` heading with gentle parameters.

- If `cw` is 0 or 1 (direction chosen):
  - `CAR.select(cw, Rdistance, Ldistance, distance)` decides which wall to use for wall-following and stores it in `distance`.
  - If the heading error is large (more than 30° in the direction that matters for `cw`), the robot uses a strong pure gyro controller:
    - `CAR.update_gyro_control(target, 1.2, 12, ...)`
  - Otherwise, it uses combined gyro + distance control:
    - `CAR.update_gyro_dis(...)`
    - This keeps the robot at a stable distance from the chosen wall **and** aligned with the overall desired heading.

This layered control gives:
- Strong corrections when badly misaligned.
- Smooth, balanced tracking of both heading and wall distance in normal conditions.

---

## 10. Color-Based Turning Logic

Within the same main loop, color markers trigger 90° turns. The robot only reacts to a color if it matches the chosen direction mode `cw`:

```vb
  If clr = 2 And cw =1 Then '''''''''orange(CW)'''''''''
    Speaker.Tone(100, 2000, 100)
    'Program.Delay(250)
    Motor.ResetCount("C")
    While Math.Abs(Motor.GetCount("C"))<50
      CAR.move_steering(centerPos,0,lastCombinedError)
    EndWhile
    target = target - 90
    t_count = t_count+1

  ElseIf clr = 3 And cw = 0 Then'''''''''''BLUE(CCW)'''''''''''''''
    Speaker.Tone(100, 2000, 100)
    'Program.Delay(250)
    Motor.ResetCount("C")
    While Math.Abs(Motor.GetCount("C"))<50
      CAR.move_steering(centerPos,0,lastCombinedError)
    EndWhile
    target = target + 90
    t_count = t_count+1
  EndIf
```

### Color roles

- **Orange (`clr = 2`) + `cw = 1`**  
  → Robot commits to a **clockwise** 90° turn by setting `target = target - 90`.

- **Blue (`clr = 3`) + `cw = 0`**  
  → Robot commits to a **counter-clockwise** 90° turn by setting `target = target + 90`.

In both cases:
1. The robot drives straight a short encoder distance (50 counts) to pass the color marker.
2. Then adjusts `target` by ±90 degrees.
3. Increments `t_count`, moving to the next navigation segment.

The actual physical rotation toward the new `target` is handled by the gyro-based control in the normal course-keeping logic.

Once `t_count` reaches 12, the main loop ends and the robot enters its end-phase sequence.

---

## 11. End-Phase Sequence

After all 12 turns are completed, the robot performs a carefully structured end-phase:

```vb
''''''''''''''''''''''''''''''END PHASE'''''''''''''''''''''''
Motor.ResetCount("C")
While Math.Abs(Motor.GetCount("C"))<255
  CAR.move_steering(centerPos,0,lastCombinedError)
EndWhile

Motor.ResetCount("C")
Time.Reset2()
While Math.abs(MotorC.GetTacho())<1000 Or Time.Get2()<4
  CAR.update_gyro_control(target, 1.2, 12 , 0,centerPos, gyro,filteredError,  gyroLastError)
EndWhile

While Math.abs(MotorC.GetTacho())<3100 Or Time.Get2()<4
  CAR.select(cw,Rdistance,Ldistance,distance)
  CAR.update_gyro_dis(cw,target,gyro, t_distance, 1.2, 6, 1.5,6,centerPos,distance,gyroError,distError,lastCombinedError,DIR)
EndWhile

LCD.Clear()
LCD.Text(1, 0, 0, 2, "Angle: " + gyro)
Motor.StartPower("C", 0)
Program.Delay(200000)
```

### Phases

1. **Short straight segment**  
   - The robot drives straight for 255 encoder counts with neutral steering.
   - This stabilizes the motion after the last turn.

2. **Gyro-only correction segment**  
   - The robot drives while `update_gyro_control` tries to match the final `target`.
   - This continues until either:
     - the tacho count reaches about 1000, or
     - 4 seconds have passed.
   - This step maximizes heading accuracy.

3. **Gyro + distance final approach**  
   - The robot then uses `update_gyro_dis` again, combining gyro and wall distance.
   - This continues until either:
     - the tacho count reaches about 3100, or
     - 4 seconds have passed.
   - This keeps the robot aligned and at the correct lateral position near the finish.

4. **Diagnostic display and stop**  
   - The LCD is cleared and the final gyro angle is printed (`"Angle: " + gyro`).
   - Motor C is stopped.
   - `Program.Delay(200000)` keeps the program alive so the result can be inspected.

This end-phase is designed to deliver a clean, stable final pose with a readable final heading.

---

## 12. Full Source Code

Below is the complete program exactly as used for the robot.  
You can copy-paste this into Clev3r to reproduce the behavior.

```vb
folder "prjs""Clever"
import "Mods\HTColorV2"
import "Mods\arduino"
import "Mods\CAR"
import "Mods\Ultrasonic"
import "Mods\Tool"

''''''''''''''''''''''variable declaration'''''''''''''''''''
cw = 3
gyro = 0
Ldistance = 0
Rdistance = 0
t_count = 0
target = 0
DIR = 0
t_distance = 40
repeat_straight = 0
con = 0
' loop counter for obstacle low-pass reset
ob_count = 0

' ===== camera / obstacle variables =====
yawC = 0
camError = 0
sig = 6         ' 6 = no obstacle, anything else = obstacle / object
mcx = 0
mcy = 0
mw = 0
mh = 0

' gyro raw values
gyroRaw = 0
rawGyro = 0

' ===== obstacle avoidance controller state =====
ob_kp = 1.23
ob_kd = 8.1
ob_corr = 0
ob_old_err = 0

''''''''''''''''''''''sub to read ultrasonic'''''''''''''''''''''''
Sub ReadUltrasonic
  alphaDist = 0.9
  counter = 0

  ' Initialize once
  Ultrasonic.getCm(2, Ldistance)
  Ultrasonic.getCm(4, Rdistance)

  While 1=1
    Ultrasonic.getCm(2, rawLdistance)
    Ultrasonic.getCm(4, rawRdistance)

    CAR.lowpass(rawLdistance, Ldistance, alphaDist, Ldistance)
    CAR.lowpass(rawRdistance, Rdistance, alphaDist, Rdistance)

    ' Reset every 5 loops
    counter = counter + 1
    If counter >= 5 Then
      Ldistance = rawLdistance
      Rdistance = rawRdistance
      counter = 0
    EndIf

  EndWhile
EndSub

''''''''''''''''''''''sub to read gyro (using ReadArduinoFull)'''''''''''''''''''''''
Sub ReadGyro
  alphaGyro = 0.6
  counter = 0

  ' Initialize once using Arduino full data (gyro + camera)
  arduino.ReadArduinoFull(1, yawC, camError, sig, mcx, mcy, mw, mh)
  gyroRaw = yawC
  gyro = (gyroRaw / 100) - offset

  While 1=1
    ' Read full data: yaw + camera information
    arduino.ReadArduinoFull(1, yawC, camError, sig, mcx, mcy, mw, mh)
    rawGyro = (yawC / 100) - offset

    ' Apply low-pass filter (same logic as before)
    CAR.lowpass2(rawGyro, gyro, alphaGyro, gyro)

    ' Reset filter every 10 loops (same logic as before)
    counter = counter + 1
    If counter >= 10 Then
      gyro = rawGyro        ' reset to current sensor reading
      counter = 0
    EndIf

  EndWhile
EndSub

Sub beep
  LCD.Clear()
  While 1=1
    If con = 1 Then
      Speaker.Tone(100, 500, 100)
    EndIf
  EndWhile
EndSub


LCD.Clear()
''''''''''''''''''''''centering steering position'''''''''''''''''''''''
CAR.reset_gyro(offset)
Program.Delay(300)
CAR.center_steer(centerPos)
Program.Delay(50)

While repeat_straight < 1000
  CAR.move_steering(centerPos,0,lasterror)
  repeat_straight = repeat_straight+1
EndWhile

''''''''''''''''''''''start sensor threads''''''''''''''''

LCD.Clear()
Program.Delay(100)
Thread.Run = ReadUltrasonic
Thread.Run = ReadGyro
'Thread.Run = beep
Program.Delay(100)

'''''''''''''''''''''''''''''''''determine direction''''''''''''''''''
If Ldistance > Rdistance Then
  cw = 0
Else 
  cw = 1
EndIf
Motor.ResetCount("C")
While Math.Abs(Motor.GetCount("C"))<400
  CAR.move_steering(centerPos,0,lasterror)
  Motor.StartPower("C",-50)
EndWhile
Motor.StartPower("C",0)
Speaker.Tone(100, 4000, 100)
Motor.ResetCount("C")
Motor.StartPower("C",50)
While Math.Abs(Motor.GetCount("C"))<550
  If cw = 0 Then
    CAR.update_gyro_control(90, 3, 8, 0,centerPos, gyro,filteredError,  gyroLastError)
  Else
    CAR.update_gyro_control(-90, 3, 8, 0,centerPos, gyro,filteredError,  gyroLastError)
  EndIf
EndWhile
Motor.StartPower("C",0)

Program.Delay(100)
Motor.StartPower("C",70)

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
While t_count<>12
  CAR.color_RGB(3,clr)

  ' ---------------------------------------
  ' Obstacle LPF loop counter
  ' Reset every 10 main-loop iterations
  ' ---------------------------------------
  ob_count = ob_count + 1
  If ob_count >= 10 Then
    ob_count = 0
  EndIf

  ' =====================================================
  ' Obstacle avoidance:
  '   sig = 6  -> no obstacle   (follow original logic)
  '   sig <> 6 -> obstacle      (use CAR.OB_AV)
  ' =====================================================
  If sig <> 6 Then
    ' Use camera-based obstacle avoidance from CAR library
    ' NOTE: OB_AV signature (in CAR):
    ' OB_AV(centerPos, error, sig, count, mcx, mcy, mw, mh, kp, kd, corr, old_err)
    CAR.OB_AV(centerPos, camError, sig, ob_count, mcx, mcy, mw, mh, ob_kp, ob_kd, ob_corr, ob_old_err)
  Else
    ' =====================================================
    ' Side ultrasonic "quick pull" logic with speed boost
    '   - If left < 5 cm  -> pull right (steering = 55)
    '   - If right < 5 cm -> pull left  (steering = -55)
    '   - During yank: speed = 100
    '   - After yank : speed = 70
    ' =====================================================

    If Ldistance < 5 Then
      Motor.StartPower("C",100)
      Motor.ResetCount("C")
      While Math.Abs(Motor.GetCount("C")) < 15
        CAR.move_steering(centerPos, -55, lastCombinedError)
      EndWhile
      Speaker.Tone(100, 4000, 100)
      Motor.StartPower("C",70)
    EndIf

    If Rdistance < 5 Then
      Motor.StartPower("C",100)
      Motor.ResetCount("C")
      While Math.Abs(Motor.GetCount("C")) < 15
        CAR.move_steering(centerPos, 55, lastCombinedError)

      EndWhile
      Speaker.Tone(100, 4000, 100)
      Motor.StartPower("C",70)
    EndIf

    ' ================= ORIGINAL COURSE-KEEPING LOGIC =================
    If cw = 3 Then
      CAR.update_gyro_control(target, 0.5, 8, 0,centerPos, gyro,filteredError,  gyroLastError)
      con = 1
    Else
      con = 0
      CAR.select(cw,Rdistance,Ldistance,distance)
      If cw = 0 And target-gyro>30 Then
        'Motor.StartPower("C", 100)
        CAR.update_gyro_control(target, 1.2, 12 , 0,centerPos, gyro,filteredError,  gyroLastError)
      ElseIf cw = 1 And target - gyro < -30 Then
        'Motor.StartPower("C", 100)
        CAR.update_gyro_control(target, 1.2, 12 , 0,centerPos, gyro,filteredError,  gyroLastError)
      Else
        'Motor.StartPower("C", 50)
        CAR.update_gyro_dis(cw,target,gyro, t_distance, 1.2, 6, 1.5,6,centerPos,distance,gyroError,distError,lastCombinedError,DIR)
        'CAR.update_gyro_control(target, 1.2, 12 , 0,centerPos, gyro,filteredError,  gyroLastError)
      EndIf
    EndIf
    ' ================= END ORIGINAL COURSE-KEEPING LOGIC =================
  EndIf



  If clr = 2 And cw =1 Then '''''''''orange(CW)'''''''''
    Speaker.Tone(100, 2000, 100)
    'Program.Delay(250)
    Motor.ResetCount("C")
    While Math.Abs(Motor.GetCount("C"))<50
      CAR.move_steering(centerPos,0,lastCombinedError)
    EndWhile
    target = target - 90
    t_count = t_count+1

  ElseIf clr = 3 And cw = 0 Then'''''''''''BLUE(CCW)'''''''''''''''
    Speaker.Tone(100, 2000, 100)
    'Program.Delay(250)
    Motor.ResetCount("C")
    While Math.Abs(Motor.GetCount("C"))<50
      CAR.move_steering(centerPos,0,lastCombinedError)
    EndWhile
    target = target + 90
    t_count = t_count+1
  EndIf

EndWhile

''''''''''''''''''''''''''''''END PHASE'''''''''''''''''''''''
Motor.ResetCount("C")
While Math.Abs(Motor.GetCount("C"))<255
  CAR.move_steering(centerPos,0,lastCombinedError)
EndWhile

Motor.ResetCount("C")
Time.Reset2()
While Math.abs(MotorC.GetTacho())<1000 Or Time.Get2()<4
  CAR.update_gyro_control(target, 1.2, 12 , 0,centerPos, gyro,filteredError,  gyroLastError)
EndWhile

While Math.abs(MotorC.GetTacho())<3100 Or Time.Get2()<4
  CAR.select(cw,Rdistance,Ldistance,distance)
  CAR.update_gyro_dis(cw,target,gyro, t_distance, 1.2, 6, 1.5,6,centerPos,distance,gyroError,distError,lastCombinedError,DIR)
EndWhile

LCD.Clear()
LCD.Text(1, 0, 0, 2, "Angle: " + gyro)
Motor.StartPower("C", 0)
Program.Delay(200000)
```

---

You can now place this `README.md` directly in your GitHub repository alongside the program file.  
It should provide enough detail for Future Engineers team members (and judges) to understand exactly how the robot navigates and avoids obstacles.
