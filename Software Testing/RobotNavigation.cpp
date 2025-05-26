#include "RobotNavigation.h"

// Constructor: Assign passed sensor and motor objects
RobotNavigation::RobotNavigation(UltrasonicSensor* right, UltrasonicSensor* left, MotorControl* motorControl) {
  rightSensor = right;
  leftSensor = left;
  motor = motorControl;
  currentState = STOPPED; // Default state on boot
}

// Initialize LCD and stop motors
void RobotNavigation::begin() {
#ifdef DEBUG_USE_LCD
  lcd.begin(16, 2);    // Initialize 16x2 LCD
  lcd.clear();
  lcd.print("INIT");   // Display initialization message
#endif

  currentState = STOPPED;
  motor->stopMotors();           // Ensure robot is not moving
  previousTime = millis();       // Record current time for state delay
}

// Change robot's state and act accordingly
void RobotNavigation::setRobotState(int state) {
  currentState = state;

#ifdef DEBUG_USE_LCD
  lcd.clear();                   // Clear LCD screen for new state
#endif

  // Execute behavior for each state
  switch (currentState) {
    case FORWARD:
#ifdef DEBUG_USE_LCD
      lcd.print("Moving Forward");
#endif
      motor->moveForward(200);   // Drive forward at speed 200
      break;

    case LEFT:
#ifdef DEBUG_USE_LCD
      lcd.print("Turning Left");
#endif
      motor->turnLeft(90);       // Slow turn left
      break;

    case RIGHT:
#ifdef DEBUG_USE_LCD
      lcd.print("Turning Right");
#endif
      motor->turnRight(90);      // Slow turn right
      break;

    case OBSTACLE_AVOID:
#ifdef DEBUG_USE_LCD
      lcd.print("Avoiding Obstacle");
#endif
      avoidingObstacle = true;
      obstacleStep = 0;                  // Reset obstacle avoidance sequence
      obstacleStartTime = millis();      // Start timing
      break;

    case STOPPED:
#ifdef DEBUG_USE_LCD
      lcd.print("Stopped");
#endif
      motor->stopMotors();              // Cut power to motors
      break;
  }
}

// Main logic loop: process line and obstacle sensors
void RobotNavigation::updateNavigation(unsigned long currentTime) {
  // If in obstacle avoidance, delegate to obstacle handler
  if (avoidingObstacle) {
    handleObstacleAvoidance(currentTime);
    return;
  }

  // Small delay between state changes (millis() instead of delay(100))
  if (currentTime - previousTime < stateChangeDelay) {
    return;
  }

  // Read line sensors
  int leftLine = digitalRead(LINE_SENSOR_LEFT);
  int rightLine = digitalRead(LINE_SENSOR_RIGHT);

  // Read obstacle sensors
  float rightDistance = rightSensor->getDistance();
  float leftDistance = leftSensor->getDistance();

  int newState;

  // Choose action based on sensor readings
  if (leftLine == HIGH && rightLine == HIGH && (rightDistance >= 20 || rightDistance == 0) && (leftDistance >= 20 || leftDistance == 0)) {
    newState = FORWARD;  // Path clear
  }
  else if (leftLine == LOW && rightLine == HIGH) {
    newState = LEFT;     // Left line lost: turn left
  }
  else if (leftLine == HIGH && rightLine == LOW) {
    newState = RIGHT;    // Right line lost: turn right
  }
  else if ((rightDistance < 20 && rightDistance != 0) || (leftDistance < 20 && leftDistance != 0)) {
    newState = OBSTACLE_AVOID;  // Obstacle detected
  }
  else {
    newState = STOPPED;  // Unknown condition: stop
  }

  // Apply new state if changed
  if (newState != currentState) {
    setRobotState(newState);
    previousTime = currentTime;  // Update timestamp
  }
}

// Behavior for obstacle avoidance sequence
void RobotNavigation::handleObstacleAvoidance(unsigned long currentTime) {
  float rightDistance = rightSensor->getDistance();
  float leftDistance = leftSensor->getDistance();

  // Update LCD only if step changed
  static int lastStepDisplayed = -1;
  if (obstacleStep != lastStepDisplayed) {
    lastStepDisplayed = obstacleStep;

#ifdef DEBUG_USE_LCD
    lcd.clear();
    switch (obstacleStep) {
      case 0: lcd.print("Back up"); break;
      case 1: lcd.print("Turn left"); break;
      case 2: lcd.print("Bypass1"); break;
      case 3: lcd.print("Turn right 1"); break;
      case 4: lcd.print("Bypass2"); break;
      case 5: lcd.print("Turn right 2"); break;
      case 6: lcd.print("Searching line"); break;
    }
#endif
  }

  // Step-by-step routine
  switch (obstacleStep) {
    case 0:
      // Step 0: Move backward until obstacle cleared
      motor->moveBackward(90);
      if ((rightDistance >= 20 || rightDistance == 0) && (leftDistance >= 20 || leftDistance == 0)) {
        obstacleStep++;
        obstacleStartTime = currentTime;
      }
      break;

    case 1:
      // Step 1: Turn left to start bypass
      motor->turnLeft(100);
      if (currentTime - obstacleStartTime >= 1000) {
        obstacleStep++;
        obstacleStartTime = currentTime;
      }
      break;

    case 2:
      // Step 2: Move forward around obstacle
      motor->moveForward(100);
      if (currentTime - obstacleStartTime >= 3000) {
        obstacleStep++;
        obstacleStartTime = currentTime;
      }
      break;

    case 3:
      // Step 3: Turn right to realign
      motor->turnRight(100);
      if (currentTime - obstacleStartTime >= 1000) {
        obstacleStep++;
        obstacleStartTime = currentTime;
      }
      break;

    case 4:
      // Step 4: Move forward again
      motor->moveForward(100);
      if (currentTime - obstacleStartTime >= 3000) {
        obstacleStep++;
        obstacleStartTime = currentTime;
      }
      break;

    case 5:
      // Step 5: Turn right again to face line
      motor->turnRight(100);
      if (currentTime - obstacleStartTime >= 1000) {
        obstacleStep++;
        obstacleStartTime = currentTime;
      }
      break;

    case 6:
      // Step 6: Search for line or timeout
      if (digitalRead(LINE_SENSOR_LEFT) == LOW &&
          digitalRead(LINE_SENSOR_RIGHT) == LOW) {
        motor->moveForward(90);

        // Timeout failsafe (5 seconds)
        if (currentTime - obstacleStartTime >= 5000) {
#ifdef DEBUG_USE_LCD
          lcd.clear();
          lcd.print("Timed out");
#endif
          motor->stopMotors();
          avoidingObstacle = false;
          currentState = STOPPED;
        }

      } else {
        // Line found again
#ifdef DEBUG_USE_LCD
        lcd.clear();
        lcd.print("Line found");
#endif
        avoidingObstacle = false;
        currentState = STOPPED;
      }
      break;
  }
}
