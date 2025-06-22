#ifndef ROBOTNAVIGATION_H
#define ROBOTNAVIGATION_H

#include <Arduino.h>
#include "MotorControl.h"
#include "UltrasonicSensor.h"

// Line sensor digital pins
#define LINE_SENSOR_LEFT 13    // Left line sensor analog input
#define LINE_SENSOR_RIGHT 12   // Right line sensor analog input

// Ultrasonic Sensors configurations
#define minDistance 3
#define maxDistance 20

// Speed configurations
#define DefaultTurnSpeed 140
#define DefaultRotateSpeed 120
#define DefaultForwardSpeed 120
#define DefaultBackwardSpeed 80

// Time constants
#define searchTimeout 100000

// Robot states
#define OBSTACLE_AVOID 0
#define LEFT 1
#define RIGHT 2
#define LINE_SEARCH 3


class RobotNavigation {
  private:
    int currentState;                    // Current robot behavior state

    // Obstacle avoidance internal state
    int obstacleStep = 0;                // Indicates a step in obstacle routine

    // Pointers to hardware control classes
    UltrasonicSensor* rightSensor;
    UltrasonicSensor* leftSensor;
    MotorControl* motor;
    ColorSensor* red;

  public:
    // Constructor to initialize with motor and sensor objects
    RobotNavigation(UltrasonicSensor* right, UltrasonicSensor* left, MotorControl* motorControl, ColorSensor* red);

    void begin();                        // Initialize robot and LCD
    void updateNavigation(); // Run navigation loop
    void setRobotState(int state);       // Set a new robot state
    void handleObstacleAvoidance(); // Obstacle avoidance handler
    void handleLineSearch();
};

#endif
