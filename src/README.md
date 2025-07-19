# Coding documentation
### Starting with open challenge logic
#### Sensor setup and its role:
The robot heavily relies on the gyro sensor to maintain its direction throughout the section navigation. The gyro is first initialized in angle mode and then reseted at the beginning to ensure consistent non incremented readings. it is then continuously read during the car's movement to determine the direction and orientation relative to the "target" angle. The angle gets updated whenever the robot turns. The steering correction is then applied based on the difference between the current angle and the target angle using what is called a "PD algorithm" AKA Proportional-Derivative controller.

Two ultrasonic sensors are mounted on both sides of the car above the steering to measure the distance from both walls. these readings allow for the robot to realign itself in the center between both walls. A cencor fusion technique is also applied to achieve stable movement by combining the filtered error from both gyro and ultrasonics to create a good steering angle. The readings are filtered by using a low pass filter to smoothen sudden changes in values and improve stability control.

A colour sensor is used to determine the direction the car will be steering too depending on the first line colour it detects from the corner sections and uses ±90 degree controlled turn while controlling the distance using the motor encoder.

## Steering and motion execution

The robots main navigation control is the gyro based PD algorithm. It keeps comparing the robots current angle to the target angle and calculates the error, then applies the corrections. The Proportional term (P) reacts to the size of error, while the derivative (D) reacts to how fast the error changes to ensure a smooth steering without under or overshooting.

## Turn Detection

The colour sensor detects the orange and blue lines. Orange triggeres the right turn while blue is left. When either detected firs the robot updates the target by going either ±90°, then it moves forward using the gyro to complete the turn and it keeps repeating until 12 detections are completed.

## Final part

Once the last turn is counted, the robot enters the last part of the code. it drives straight using the gyro for direction and motor encoder for distance ensuring a precise alignment towards the last parking section.


