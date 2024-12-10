# PS5-Controlled Robot with MPU6050

**Description**

This project implements a robot controlled via a PlayStation 5 (PS5) controller and an MPU6050 sensor for orientation detection. The robot uses the PS5's analog sticks and buttons for movement and direction control, while the MPU6050 provides feedback on the robot's angle for balance and potentially more advanced maneuvers.

**for Libraries **
refer https://github.com/rodneybakiskan/ps5-esp32

**Features**

* PS5 controller integration for intuitive control (Forward, backward, left, right, speed adjustment)
* MPU6050 sensor for angle detection (Improves balance, potential for more features)
* Clear serial communication for debugging and monitoring (Optional, based on your needs)
* PID control algorithm for smooth and responsive movement (Optional, based on your implementation)

**Getting Started**

1. **Hardware Requirements:**
   * Arduino-compatible board (e.g., Uno, Mega)
   * PS5 controller
   * MPU6050 sensor module
   * Jumper wires
   * Motor driver and motors (if applicable)

2. **Software Requirements:**
   * Arduino IDE
   * Libraries (if necessary):
     * PS5 controller library (e.g., Sixaxis library)
     * MPU6050 library
     * PID library (optional)

3. **Instructions:**
   * Install the Arduino IDE.
   * Download and install the required libraries (if necessary) according to their instructions.
   * Clone or download this repository.
   * Open the `*.ino` file (e.g., `PS5_Robot_MPU6050.ino`) in the Arduino IDE.
   * Connect your Arduino board, PS5 controller, and MPU6050 sensor according to the hardware schematics (not included in this repository, create based on your specific hardware).
   * Upload the code to your Arduino board.

**Customization**

* You can adjust motor control parameters (pins, directions, speed limits) in the `motorApwm`, `motorAdir`, etc. sections of the code.
* The PID control parameters (Kp, Ki, Kd) can be fine-tuned for optimal robot behavior, potentially requiring additional libraries.
* Consider adding comments and explanations to the code for better understanding.

**Future Enhancements**

* Implement more advanced maneuvers based on PS5 controller inputs (e.g., diagonal movement, turning using analog sticks)
* Integrate sensor data from the MPU6050 for more sophisticated control (e.g., automatic balance adjustment)
* Explore additional libraries or custom functions for more complex functionalities

**License**

(Apache License 2.0)

**Author**

(Ganesh Solapure)
