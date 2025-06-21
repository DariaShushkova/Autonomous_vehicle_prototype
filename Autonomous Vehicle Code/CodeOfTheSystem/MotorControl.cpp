#include "MotorControl.h"

// Move robot straight forward at given speed
void MotorControl::moveForward(int speed) {
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}

// Move robot straight backward at given speed
void MotorControl::moveBackward(int speed) {
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

// Turn robot to the left
void MotorControl::turnLeft(int speed) {
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

// Turn robot to the right
void MotorControl::turnRight(int speed) {
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}

// Stop robot
void MotorControl::stopMotors() {
  analogWrite(LEFT_MOTOR_SPEED_PIN, 255);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 255);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}

// Rotate robot to the right
void MotorControl::rotateRight(int speed) {
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}

// Rotate robot to the left
void MotorControl::rotateLeft(int speed) {
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}
