#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ps5Controller.h>

// Motor Pin Definitions
const int motorApwm = 13; // Left
const int motorAdir = 12;
const int motorBpwm = 27; // Left
const int motorBdir = 26;
const int motorCpwm = 33; // Right
const int motorCdir = 32;
const int motorDpwm = 18; // Right
const int motorDdir = 19;

const int pwmFrequency = 5000;
const int pwmResolution = 8; // 8-bit resolution
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmChannelC = 2;
const int pwmChannelD = 3;

float Speed = 0.0;
float leftMotorSpeed = 0.0;
float rightMotorSpeed = 0.0;

// PID Constants
const float Kp = 2.0;
const float Ki = 0.05;
const float Kd = 0.1;
const float baseSpeed = 0;
float maxSpeed = 0;

float error = 0.0;
float prevError = 0.0;
long timer = 0;
float pidOutput = 0.0;

// MPU6050 Sensor
MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(115200);

  // Initialize PS5 Controller
  ps5.begin("D0:BC:C1:98:2E:F3");
  Serial.println("Ready.");

  // Initialize MPU6050
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Initialize Motor Pins and PWM
  pinMode(motorAdir, OUTPUT);
  pinMode(motorBdir, OUTPUT);
  pinMode(motorCdir, OUTPUT);
  pinMode(motorDdir, OUTPUT);

  ledcSetup(pwmChannelA, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannelB, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannelC, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannelD, pwmFrequency, pwmResolution);

  ledcAttachPin(motorApwm, pwmChannelA);
  ledcAttachPin(motorBpwm, pwmChannelB);
  ledcAttachPin(motorCpwm, pwmChannelC);
  ledcAttachPin(motorDpwm, pwmChannelD);

  stop();
}

void loop() {
  if (!ps5.isConnected()) {
    Serial.println("PS5 controller not found");
    delay(300);
    return;
  }

  while (ps5.isConnected()) {
    mpu6050.update();
    float angle = mpu6050.getAngleZ();

    if (millis() - timer > 300) {
      handleAngleCorrection(angle);
      handlePS5Inputs();
      timer = millis();
    }
  }
}

// Handles Angle Correction using PID
void handleAngleCorrection(float angle) {
  error = computeAngleError(angle);
  pid();
}

// Computes the angle error
float computeAngleError(float angle) {
  if (abs(angle) < 1) return 0;
  if (abs(angle - 90) < 1) return 90;
  if (abs(angle + 90) < 1) return -90;
  return -angle;
}

// PID Function
void pid() {
  float derivative = error - prevError;
  prevError = error;

  pidOutput = Kp * error + Kd * derivative;
  motorspeed(pidOutput);

  Serial.print("\tError: ");
  Serial.print(error);
  Serial.print("\tPID Output: ");
  Serial.println(pidOutput);
}

// Sets motor speeds based on PID output
void motorspeed(float pidOutput) {
  leftMotorSpeed = constrain(baseSpeed + Speed + 3 * pidOutput, 0, 255);
  rightMotorSpeed = constrain(baseSpeed + Speed - 3 * pidOutput, 0, 255);

  ledcWrite(pwmChannelA, leftMotorSpeed);
  ledcWrite(pwmChannelB, leftMotorSpeed);
  ledcWrite(pwmChannelC, rightMotorSpeed);
  ledcWrite(pwmChannelD, rightMotorSpeed);
}

// Handles PS5 Controller Inputs
void handlePS5Inputs() {
  if (ps5.Right()) buttonRight();
  if (ps5.Down()) buttonDown();
  if (ps5.Up()) buttonUp();
  if (ps5.Left()) buttonLeft();
  if (ps5.Square()) squareButtonAction();
  if (ps5.R2()) handleR2Trigger();
  if (ps5.LStickX()) handleLeftStickX();
  if (ps5.LStickY()) handleLeftStickY();
  if (ps5.L1()) stop();
}

// PS5 Button Actions
void buttonRight() { setMotorDirection(HIGH, LOW, LOW, HIGH, "Right"); }
void buttonLeft() { setMotorDirection(LOW, HIGH, HIGH, LOW, "Left"); }
void buttonUp() { setMotorDirection(HIGH, LOW, HIGH, LOW, "Forward"); }
void buttonDown() { setMotorDirection(LOW, HIGH, LOW, HIGH, "Backward"); }
void squareButtonAction() {
  Speed = 80;
  error = 90;
  pid();
  timer = millis();
}

// R2 Trigger Handling
void handleR2Trigger() {
  Speed = ps5.R2() ? ps5.R2Value() : 0;
  Serial.printf("R2 button at %d\n", ps5.R2Value());
}

// Left Joystick Handling
void handleLeftStickX() {
  if (ps5.LStickX() > 30) right();
  if (ps5.LStickX() < -30) left();
  Serial.printf("Left Stick x at %d\n", ps5.LStickX());
}

void handleLeftStickY() {
  if (ps5.LStickY() > 30) forward();
  if (ps5.LStickY() < -30) backward();
  Serial.printf("Left Stick y at %d\n", ps5.LStickY());
}

// Motor Direction Functions
void forward() { setMotorDirection(HIGH, LOW, HIGH, LOW, "Forward"); }
void backward() { setMotorDirection(LOW, HIGH, LOW, HIGH, "Backward"); }
void left() { setMotorDirection(HIGH, LOW, LOW, HIGH, "Left"); }
void right() { setMotorDirection(LOW, HIGH, HIGH, LOW, "Right"); }
void stop() { setMotorDirection(LOW, LOW, LOW, LOW, "STOP"); }

// Sets motor directions
void setMotorDirection(int aDir, int bDir, int cDir, int dDir, const char* action) {
  digitalWrite(motorAdir, aDir);
  digitalWrite(motorBdir, bDir);
  digitalWrite(motorCdir, cDir);
  digitalWrite(motorDdir, dDir);

  ledcWrite(pwmChannelA, leftMotorSpeed);
  ledcWrite(pwmChannelB, leftMotorSpeed);
  ledcWrite(pwmChannelC, rightMotorSpeed);
  ledcWrite(pwmChannelD, rightMotorSpeed);

  Serial.println(action);
}
