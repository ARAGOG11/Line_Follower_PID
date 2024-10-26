# IR Line Following Robot

This project involves creating an IR line-following robot using an Arduino. The robot utilizes infrared (IR) sensors to detect the line on the ground and control the motors accordingly to follow the line.

## Features

- Uses three IR sensors to detect the line: left, center, and right.
- Implements a PID control algorithm for smooth line following.
- Configures PWM for motor control to achieve precise speed control.
- Handles various scenarios based on sensor input for effective navigation.

## Components Required

- Arduino (e.g., Uno, Mega)
- IR sensors (3 units)
- DC motors with motor driver (e.g., L298N)
- Power supply for motors
- Jumper wires
- Chassis for the robot

## Pin Configuration

| Pin | Function                |
|-----|-------------------------|
| 5   | Enable Right Motor      |
| 6   | Enable Left Motor       |
| 7   | Left Motor Direction 1  |
| 8   | Left Motor Direction 2  |
| 9   | Right Motor Direction 1  |
| 10  | Right Motor Direction 2  |
| 11  | IR Sensor Right (Input) |
| 12  | IR Sensor Left (Input)  |
| 4   | IR Sensor Center (Input) |

## Code Overview

### Setup Function

- Initializes the PWM frequency for motor control.
- Sets up timers for controlling the timing of operations.
- Configures motor and sensor pins.

### Loop Function

- Continuously reads sensor values to determine the position of the line.
- Calculates error and applies PID control to adjust motor speed.
- Implements decision-making based on sensor inputs to control the robot's movement (e.g., turn left, turn right, go straight).

### Motor Control

The `rotateMotor` function controls the direction and speed of the motors based on the calculated values from the sensors.

## PID Control Parameters

- **Kp (Proportional Gain)**: 50
- **Ki (Integral Gain)**: 0.5
- **Kd (Derivative Gain)**: 10

Adjust these parameters for tuning the responsiveness of the robot.

## Usage

1. Connect the components as per the pin configuration.
2. Upload the code to the Arduino.
3. Place the robot on a surface with a clearly defined line.
4. Power on the robot and observe its line-following behavior.

## Troubleshooting

- If the robot does not follow the line correctly, adjust the PID parameters.
- Ensure that the IR sensors are positioned correctly and are functioning.
- Check motor connections and ensure they are receiving power.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

Feel free to modify and improve the code and hardware setup as needed to enhance the performance of your line-following robot!
