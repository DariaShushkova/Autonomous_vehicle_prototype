#ifndef ROBOTNAVIGATION_H
#define ROBOTNAVIGATION_H

#include <Arduino.h>
#include "MotorControl.h"
#include "UltrasonicSensor.h"
#include <Adafruit_LiquidCrystal.h>

// Line sensor digital pins
#define LINE_SENSOR_LEFT 13    // Left line sensor analog input
#define LINE_SENSOR_RIGHT 12   // Right line sensor analog input


#define DEBUG_USE_LCD          // To disable LCD for real experiment comment this out

#ifdef DEBUG_USE_LCD
Adafruit_LiquidCrystal lcd(0); // LCD with I2C address 0 (used in simulation)
#endif

// Robot states
#define FORWARD 0
#define LEFT 1
#define RIGHT 2
#define STOPPED 3
#define OBSTACLE_AVOID 4

class RobotNavigation {
  private:
    int currentState;                    // Current robot behavior state
    unsigned long previousTime;          // Last time state was updated
    int stateChangeDelay = 100;          // Minimum delay between state transitions

    // Obstacle avoidance internal state
    bool avoidingObstacle = false;       // Flag to set OBSTACLE_AVOID state
    int obstacleStep = 0;                // Indicates a step in obstacle routine
    unsigned long obstacleStartTime;     // Timestamp of current step in obstacle routine

    // Pointers to hardware control classes
    UltrasonicSensor* rightSensor;
    UltrasonicSensor* leftSensor;
    MotorControl* motor;

  public:
    // Constructor to initialize with motor and sensor objects
    RobotNavigation(UltrasonicSensor* right, UltrasonicSensor* left, MotorControl* motorControl);

    void begin();                        // Initialize robot and LCD
    void updateNavigation(unsigned long currentTime); // Run navigation loop
    void setRobotState(int state);       // Set a new robot state
    void handleObstacleAvoidance(unsigned long currentTime); // Obstacle avoidance handler
};

#endif
