# EV3 Open Challenge Code Explanation


This document explains the navigation program for the WRO Future Engineers Open Challenge.

---

## 1. Project Structure and Module Imports

The program begins by declaring the project folder and importing several specialized modules. These modules collectively provide the robot with the ability to sense its environment, perform stable motion, communicate with an external gyro interface, and apply advanced control techniques. Instead of writing raw motor logic repeatedly throughout the code, these imported modules encapsulate reusable behaviors such as wall-following, low-pass filtering, steering alignment, and color recognition. Loading them at the beginning enables the main script to operate at a high level, focusing on navigation strategy rather than low-level commands.

```vb
folder "prjs""Clever"
  import "Mods\HTColorV2"
  import "Mods\arduino"
  import "Mods\CAR"
  import "Mods\Ultrasonic"
  import "Mods\Tool"
```

### Key capabilities added by these imports:
- More stable RGB detection for colored markers  
- Precise gyro readings from an Arduino bridge  
- Steering and PID utilities for straight driving and turning  
- Real-time ultrasonic handling  
- General helper functions used across multiple subsystems  

These imports define the environment in which the robot will operate during the challenge.

---

## 2. Global Variables and System State

After loading the required modules, the program initializes a set of global variables. These variables act as a shared memory space that both the main logic and background threads can access. They store sensor readings, operational modes, counters, and target heading values. Because the robot must move through twelve navigation segments with precise turns and dynamic behaviors, these variables allow different parts of the program to coordinate and maintain consistent behavior across time.

```vb
cw = 3
gyro = 0
Ldistance = 0
Rdistance = 0
t_count = 0
target = 0
DIR = 0
t_distance = 30
repeat_straight = 0
```

### These variables represent:
- **Navigation mode** (neutral, clockwise, counter‑clockwise)  
- **Heading** (filtered gyro angle)  
- **Distance to walls** on left and right  
- **Turn counter** which ensures exactly 12 turns  
- **Desired heading target** for gyro control  
- **Preferred wall distance** for wall‑following  
- **Steering alignment buffer** during initialization  

This shared state gives the robot a consistent reference throughout the entire run.

---

## 3. Ultrasonic Sensor Thread

The ultrasonic thread continuously reads the left and right distance sensors in the background. This design ensures the robot always has fresh distance data even when the main loop is busy performing steering updates or evaluating turn logic. Because ultrasonic sensors can produce noisy readings, the thread smooths the values using a low‑pass filter. This prevents sudden changes from destabilizing the robot during wall‑following.

```vb
Sub ReadUltrasonic
  alphaDist = 0.9
  counter = 0
  Ultrasonic.getCm(2, Ldistance)
  Ultrasonic.getCm(4, Rdistance)
  While 1=1
    Ultrasonic.getCm(2, rawLdistance)
    Ultrasonic.getCm(4, rawRdistance)
    CAR.lowpass(rawLdistance, Ldistance, alphaDist, Ldistance)
    CAR.lowpass(rawRdistance, Rdistance, alphaDist, Rdistance)
    counter = counter + 1
    If counter >= 5 Then
      Ldistance = rawLdistance
      Rdistance = rawRdistance
      counter = 0
    EndIf
  EndWhile
EndSub
```

### Why this is important:
- Ultrasonic noise is reduced before being used for steering  
- Filtering prevents “twitchy” corrections  
- Periodic resets ensure the sensor doesn’t accumilate error
- Continuous looping ensures high update frequency  

This thread allows the robot to rely confidently on wall distance measurements.

---

## 4. Gyro Thread Through Arduino

The gyro thread performs a similar role but focuses on angular heading. It gathers angle information from the Arduino gyro bridge, applies smoothing, and periodically resets the filter. Since the gyro reading is essential for straight driving and accurate 90° turns, this continuous background process ensures minimal drift and maximum responsiveness.

```vb
Sub ReadGyro
  alphaGyro = 0.6
  counter = 0
  arduino.GyroArduino(1, gyroRaw)
  gyro = (gyroRaw / 100) - offset
  While 1=1
    arduino.GyroArduino(1, gyroRaw)
    rawGyro = (gyroRaw / 100) - offset
    CAR.lowpass2(rawGyro, gyro, alphaGyro, gyro)
    counter = counter + 1
    If counter >= 10 Then
      gyro = rawGyro
      counter = 0
    EndIf
  EndWhile
EndSub
```

### Key benefits of the gyro thread:
- Smooth heading changes  
- Stable readings for PID control  
- Prevents drift buildup  
- Maintains constant orientation awareness during all segments  

Accurate heading data is fundamental to the robot’s navigation performance.

---

## 5. Initialization and Steering Alignment

Before entering the main navigation loop, the robot performs several important setup routines. It first resets and calibrates the gyro so that the initial heading becomes the zero-degree reference. The robot then centers the steering mechanism to ensure the wheels are aligned for straight driving. Because mechanical linkages can introduce slack or misalignment, the robot performs a repeated straight‑drive routine that physically settles the steering system into its true center. This helps ensure that the first long straight segment is accurate and reduces error accumulation later.

### The steps achieve:
- A reliable gyro baseline  
- Properly centered wheels  
- Removal of mechanical slack  
- A consistent starting movement pattern  

This preparation dramatically improves the repeatability of the robot’s navigation performance.

---

## 6. Main Navigation Loop

The main loop runs until the robot completes twelve navigation segments. Each cycle begins with reading the color sensor, followed by choosing between different steering behaviors. Initially, the robot drives straight using soft gyro control. Once a color marker is detected, the robot switches into its long‑term direction mode. From there, it uses either pure gyro correction or combined gyro‑distance correction depending on how far it has drifted from the target heading. Wall‑following uses both distance and heading error simultaneously, producing a balanced steering output that keeps the robot aligned to the wall while maintaining the correct overall direction.

### The main loop blends:
- Color detection  
- Gyro‑only correction  
- Wall‑following control  
- Large‑error heading correction  
- Turn detection and execution  

By switching intelligently between these behaviors, the robot adapts to environmental conditions while maintaining a predictable path.

---

## 7. Color‑Based Direction Selection

When the robot first sees orange or blue, it chooses whether to follow a clockwise or counter‑clockwise pattern for the rest of the run. This choice determines which wall the robot uses for guidance and in which direction each 90° turn will rotate. The logic only triggers once because the robot must commit to its direction for consistency.

### Color meaning:
- **Orange → Clockwise**  
- **Blue → Counter‑Clockwise**

This decision defines the robot’s navigation style for all remaining segments.

---

## 8. Performing 90° Turns

Each turn is triggered when the robot detects the appropriate color while already in its corresponding direction mode. Before turning, the robot drives forward a short encoder distance to clear the marker. It then adjusts the target heading by ±90°, depending on the direction. The gyro PID controller handles the rotation automatically, steering the robot to align itself with the new heading. After completing the turn, the robot increments the `t_count` variable to progress to the next navigation segment.

### Turn elements include:
- Color confirmation  
- Encoder‑based forward buffer  
- Target heading change  
- PID‑stabilized rotation  
- Segment count update  

This structured approach ensures predictable and repeatable turns.

---

## 9. End‑Phase Sequence

After completing all twelve turns, the robot enters a final sequence of movements. It first drives a short straight distance, then transitions to a longer gyro‑corrected segment(turn), and finally performs a wall‑following stretch. These layers help compensate for any drift accumulated earlier in the run and place the robot near the final area with consistent alignment. At the end, the robot displays its final gyro angle and stops the drive motor while leaving the program running so that we can inspect the result.

### This final section provides:
- A stabilizing straight movement  
- A long accurate gyro‑guided segment  
- A final wall‑corrected approach  
- A diagnostic display of heading  

This ensures the robot finishes cleanly and predictably.




