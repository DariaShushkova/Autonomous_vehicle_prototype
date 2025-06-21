#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

// Motor control pin definitions
#define LEFT_MOTOR_SPEED_PIN 10       // PWM pin controlling left motor speed
#define RIGHT_MOTOR_SPEED_PIN 11      // PWM pin controlling right motor speed

#define LEFT_MOTOR_FORWARD_PIN 2      // Digital pin to move left motor forward
#define LEFT_MOTOR_BACKWARD_PIN 3     // Digital pin to move left motor backward
#define RIGHT_MOTOR_FORWARD_PIN 4     // Digital pin to move right motor forward
#define RIGHT_MOTOR_BACKWARD_PIN 5    // Digital pin to move right motor backward

class MotorControl {
  public:
    void moveForward(int speed);       // Move robot forward
    void moveBackward(int speed);      // Move robot backward
    void turnLeft(int speed);          // Turn robot left
    void turnRight(int speed);         // Turn robot right
    void rotateLeft(int speed);        // Rotate robot left
    void rotateRight(int speed);       // Rotate robot right
    void stopMotors();                 // Stop both motors
};

#endif
