# Coding documentation
### Starting with open challenge logic
#### Sensor setup and its role:
The robot heavily relies on the gyro sensor to maintain its direction throughout the section navigation. The gyro is first initialized in angle mode and then reseted at the beginning to ensure consistent non incremented readings. it is then continuously read during the car's movement to determine the direction and orientation relative to the "target" angle. The angle gets updated whenever the robot turns. The steering correction is then applied based on the difference between the current angle and the target angle using what is called a "PD algorithm" AKA Proportional-Derivative controller.

Two ultrasonic sensors are mounted on both sides of the car above the steering to measure the distance from both walls. these readings allow for the robot to realign itself in the center between both walls. A cencor fusion technique is also applied to achieve stable movement by combining the filtered error from both gyro and ultrasonics to create a good steering angle. The readings are filtered by using a low pass filter to smoothen sudden changes in values and improve stability control.

A colour sensor is used to determine the direction the car will be steering too depending on the first line colour it detects from the corner sections and uses ±90 degree controlled turn while controlling the distance using the motor encoder.
