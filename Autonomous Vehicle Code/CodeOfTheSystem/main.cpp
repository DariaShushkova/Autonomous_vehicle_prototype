#include <Arduino.h>
#include "MotorControl.h"
#include "UltrasonicSensor.h"
#include "RobotNavigation.h"

// Create hardware instances
MotorControl motor;
UltrasonicSensor rightUltrasonic(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
UltrasonicSensor leftUltrasonic(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
RobotNavigation robotNav(&rightUltrasonic, &leftUltrasonic, &motor);

// Arduino initialization
void setup() {
  // Setup motor pins as outputs
  pinMode(LEFT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);

  // Initialize ultrasonic sensors
  rightUltrasonic.begin();
  leftUltrasonic.begin();

  // Start navigation system
  robotNav.begin();
}

// Arduino main loop
void loop() {
  unsigned long currentTime = millis();     // Get current time
  robotNav.updateNavigation(currentTime);   // Execute robot logic
}
