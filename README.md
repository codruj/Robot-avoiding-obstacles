# Line Follower Robot Overview

This project involves the implementation of a line-following robot that uses a combination of sensors, motors, and visual feedback to autonomously follow a line and avoid obstacles.

## Technical Details

### Hardware
- **Microcontroller**: Arduino Uno for processing sensor data and controlling outputs.
- **Sensors**: 
  - **XLine 16 Sensor Array** for line detection.
  - **HC-SR04 Ultrasonic Sensor** for obstacle detection.
- **Motors**: DC motors controlled by an H-Bridge to drive the robot.
- **Control**: Potentiometer for dynamic speed adjustment.
- **Visual Feedback**: NeoPixel LED Strip to indicate movement direction.

### Software
- **Line Sensor Multiplexer**: Code to read from the 16-sensor array by selecting each sensor using digital pins (S0-S3).
- **Motor Control**: Functions to control the motor speeds based on sensor inputs and potentiometer readings.
- **Obstacle Avoidance**: Logic to stop and perform a 180-degree turn when an obstacle is detected within 10 cm.
- **LED Feedback**: NeoPixel LEDs indicate the robot’s direction (left, right, forward).

### Performance
- **Line Following**: 99% accuracy in straight-line tracking.
- **Obstacle Detection**: 99% success rate in detecting obstacles under 10 cm.
- **NeoPixel LED Feedback**: 100% accurate state indication.

### Testing and Validation
- **Challenges**: Sensor calibration, motor control adjustments, and speed tuning were necessary for optimal performance.
- **Testing Limits**:
  - The robot struggles to follow the line if the starting angle is more than 60 degrees.
  - Exceeding 10% of the motor’s max speed causes the robot to become unreliable, especially at turns and obstacle detection.

### Phases of Development
1. **Basic Movement**: Initial tests with motors and speed adjustment.
2. **Sensor Integration**: Addition of line and ultrasonic sensors.
3. **Visual Feedback**: Integration of NeoPixel LEDs for direction indication.

[Further Details](https://github.com/codruj/Robot-avoiding-obstacles/blob/main/robot-documentation.pdf)

