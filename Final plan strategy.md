# Engineering Insights & Key Takeaways

Throughout the development of our robot, we collected a series of technical insights and patterns  
that guided our decision-making. These notes summarize what consistently worked, what failed,  
and what we found to be the most reliable engineering approaches for this challenge.

---

## Core Development Mindset

- **Keep the architecture predictable.** A robot with fewer surprises is easier to tune.
- **Stability beats complexity.** A simple mechanism that behaves consistently will always outperform a “clever” mechanism that behaves unpredictably.
- **Every component must justify its placement.** If it doesn’t add accuracy, stability, or reliability — remove or reposition it.
- **Sensors are only as good as their mounting.** A perfect sensor in a bad position becomes a bad sensor.
- **Test early, test small, test often.** The fastest progress came from validating tiny pieces before joining them into a full system.

---

## Mechanical Observations

- **Wheel size defines everything.** Turning radius, weight distribution, and sensor visibility all change dramatically with wheel size; smaller wheels gave us far more control.
- **Steering sensitivity is nonlinear.** A direct motor-to-axle connection was too aggressive; introducing intentional backlash through gear reduction created smoother, more controllable steering.
- **Center of gravity matters more than expected.** Reducing height improved reactions during tight turns and minimized oscillations.
- **Rigid structures cause vibration.** Allowing micro-flexibility in the front assembly dampened overshoot and helped with PID stability.
- **Cable routing affects performance.** Avoiding EMI by separating camera, motor, and ultrasonic cables improved consistency and reduced random sensor spikes.

---

## Sensor Behavior & Data Lessons

- **Ultrasonic “look-ahead” is crucial.** Placing the sensor farther forward allowed the robot to react earlier and stopped oscillation in narrow corridors.
- **Lighting changes everything.** Even small shifts in brightness can affect camera processing; designing for worst-case lighting pays off.
- **Default states are not failures.** When the camera misread a sign, using a robust fallback mode prevented crashes and allowed recovery.
- **Sampling rate beats raw accuracy.** Fast, lower-noise readings performed better in practice than slow, “perfect” readings.

---

## Software & Algorithm Insights

- **PID must match the hardware, not the ideal math.** Our best PID tuning came after acknowledging the real mechanical imperfections of the steering system.
- **Modular code accelerates debugging.** Testing each module independently (steering, distance control, sign detection) made integration smoother.
- **Safety conditions matter.** Handling rare edge cases early prevented unexpected robot behavior deep into testing.
- **Reset logic is underrated.** A quick calibration reset helped keep readings consistent across long test sessions.

---

## Practical Workflow Notes

- **Never trust the first successful test.** Repeated success under different lighting, orientations, and speeds ensured we had real stability.
- **Video feedback is incredibly useful.** Reviewing slow-motion recordings helped catch issues invisible to the naked eye.
- **Iterate toward reliability, not luck.** If a solution only works “sometimes,” it is not a solution.
- **Mechanical fixes often solve software problems.** Many of our biggest bugs disappeared after adjusting hardware, not code.

---

## Final Reflection

These insights shaped every stage of our development — from planning, to prototyping, to tuning the final robot.  
They capture the real engineering mindset that guided our decisions and helped us turn early designs into a  
stable, competition-ready machine.


