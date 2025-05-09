#include "MotorControl.h"

// Internal helper function to apply motor direction and speed
void MotorControl::setMotorStates(int leftSpeed, int rightSpeed,
                                  int leftForward, int leftBackward,
                                  int rightForward, int rightBackward) {
  // Set PWM speeds for both motors
  analogWrite(LEFT_MOTOR_SPEED_PIN, leftSpeed);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, rightSpeed);

  // Set direction for left motor
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, leftForward);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, leftBackward);

  // Set direction for right motor
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, rightForward);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, rightBackward);
}

// Move robot straight forward at given speed
void MotorControl::moveForward(int speed) {
  setMotorStates(speed, speed, HIGH, LOW, HIGH, LOW);
}

// Move robot straight backward at given speed
void MotorControl::moveBackward(int speed) {
  setMotorStates(speed, speed, LOW, HIGH, LOW, HIGH);
}

// Rotate robot to the left (left wheel backward, right wheel forward)
void MotorControl::turnLeft(int speed) {
  setMotorStates(speed, speed, LOW, HIGH, HIGH, LOW);
}

// Rotate robot to the right (left wheel forward, right wheel backward)
void MotorControl::turnRight(int speed) {
  setMotorStates(speed, speed, HIGH, LOW, LOW, HIGH);
}

// Stop both motors by setting speed to 0
void MotorControl::stopMotors() {
  analogWrite(LEFT_MOTOR_SPEED_PIN, 0);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 0);
}
