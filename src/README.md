# Coding documentation
### Starting with open challenge logic

- ### steering recentering "calibration"
As soon as the robot starts, it goes into a steering recentering mode where it makes sure the steering angle is 0 in the start to ensure reliable control in the run. The robot turns fully to the left and then fully to the right twice and records the max angle position in both directions and calculates the middle point. This center position is saved and used in the code as a reference for all steering corrections throughout the run. Without this step, the robot may drift due to misalignment. Doing this in the start ensures symmetry in steering.

- ### Straight movement using the Gyro 
Once the center point is determined, the robot begins moving forward using only the gyro control. it keeps comparing the current heading angle from gyro to a target angle which is set to 0 from the start.
A PD algorithm (proportional-derivative) is used to calculate and readjust the steering. All sensor reading (ultrasonic, gyro) in exception to the colour sensor are passed through a low-pass filter to reduce abrupt fluctuations to smoothen the car movement. 

error = target - currentAngle
filteredError = α × previousFiltered + (1 - α) × error
steeringPower = (Kp × filteredError) + (Kd × change in error)

The robot is then locked in to a safe range to prevent the robot from oversteering (between -55 and 55 degrees). This is activated as soon as the robot starts before any detections, to ensure a straight path.

- ### Color detection for turning
As the robot moves forward, it keeps scanning the ground using its color sensor. when it detects orange the robot moves in a clockwise direction and blue chooses a counterclockwise direction. this detection is set in a variable called first_color, which is used to determine which side's ultrasonic sensor to follow when correcting based on the distance. 

- ### Turn Execution
Each time the robot detects the same color during the run it interprets this as a cue to turn +-91 degrees depending on the color detected
-  target = target ± 91°

This change in the target angle makes the correction in the cotnrol loop making the robot turn in a curve accordingly.

(we chose 91 not 90 because we have a gear backlash from the steering so we accounted for it in the code)
The algorithm goes on loop until 12 color detections are made indicating the 3 laps have finished.

- ### Hybrid steering system using Gyro + Distance sensor Fusion
After the first color is detected, the robot upgrades the control by switching into a hybrid mode which combines data like the angular error from the gyro sensor; The wall distance error from the left or right ultrsonic, depending on direction of turning.
Both of the readings are filtered seperately using low-pass filtering and then it is combined to calculate the final steering output.

gyroError = target - currentAngle
filteredGyro = α × previousGyro + (1 - α) × gyroError

distanceError = desiredWallDistance - measuredDistance
filteredDist = α × previousDist + (1 - α) × distanceError

combinedError = (filteredGyro + filteredDist) / 2

steeringPower = (Kp1 × filteredGyro + Kp2 × filteredDist) + Kd × ΔcombinedError

The steering value is limited again to avoid oversteering and wheel drifting.

- ### Final lap and stopping
Once the robot detects 12 times it transitions to the final stage. it drives forward with predefined distances using the encoder while still using the hybrid steering logic. This is to ensure that any accumelated gyro drift values (if any) is avoided and ensures a straight path.


## Obstacle challenge

### Initialization Process

The initialization process is exactly the same as the Open Challenge; The robot begins by centering the steering system, going fully left and right and calculating the middle point. The gyro sensor resets to ensure that the car starts at angle 0. These are extremely critical steps in order to have an accurate PD control in throughout the round. we also initialize 3 threads, 1 for color detection, another one which reads from the multiplexer by Reading 2 bytes of data from register 84, which is where the SMUX stores sensor readings. Combines the two bytes into a single 16-bit number. Returns this number as the sensor value, and a thread for the pixy cam to grab the closest signiture.(we also used low pass filtering to filter our noisy data for the size and coordinates of the signiture).

### 




















#### Sensor setup and its role:
The robot heavily relies on the gyro sensor to maintain its direction throughout the section navigation. The gyro is first initialized in angle mode and then reseted at the beginning to ensure consistent non incremented readings. it is then continuously read during the car's movement to determine the direction and orientation relative to the "target" angle. The angle gets updated whenever the robot turns. The steering correction is then applied based on the difference between the current angle and the target angle using what is called a "PD algorithm" AKA Proportional-Derivative controller.

Two ultrasonic sensors are mounted on both sides of the car above the steering to measure the distance from both walls. these readings allow for the robot to realign itself in the center between both walls. A sensor fusion technique is also applied to achieve stable movement by combining the filtered error from both gyro and ultrasonics to create a good steering angle. The readings are filtered by using a low pass filter to smoothen sudden changes in values and improve stability control. we also have an ultrasonic infront of the robot incase the robot gets too close the the obstacle it backs up and re-adjusts

A colour sensor is used to determine the direction the car will be steering too depending on the first line colour it detects from the corner sections and uses ±90 degree controlled turn.

## Steering and motion execution

The robots main navigation control is the gyro based PD algorithm. It keeps comparing the robots current angle to the target angle and calculates the error, then applies the corrections. The Proportional term (P) reacts to the size of error, while the derivative (D) reacts to how fast the error changes to ensure a smooth steering without under or overshooting.

## Turn Detection

The colour sensor detects the orange and blue lines. Orange triggeres the right turn while blue is left. When either detected first the robot updates the target by going either ±90°, then it moves forward using the gyro to complete the turn and it keeps repeating until 12 detections are completed.

## Final part

Once the last turn is counted, the robot aligns with the left or right wall using the left or right ultrasonic sensor depending on the driving direction, backs up and the robot then alights it self with the wall perpendicular to the one infront of it. then it keeps avoiding the obstacles until the robot reaches a certain distance measured by encoders.

## Issues faced

due to the limitted type of motors we can use on ev3 we had to use a dc motor with encoder instead of a servo, the difference between the 2 is the servo motor measures it angle using an internal potentiometer meaning that the servo always knows its position even after turning te servo on and off.However a dc motor with encoder does not know its initial position so we had to run a steer-centering program.Another issue with the our robot is our multiplexer, due it it not being official lego component intigrating it was not the easiest task, somtimes the ultrasonic sensors attached to the multiplexer randomly turn off which causes the robot to have false readings which we attempted to filter out. Additionally we had to place a delay between switching wit the different chanels so the data does not get corrupted.


