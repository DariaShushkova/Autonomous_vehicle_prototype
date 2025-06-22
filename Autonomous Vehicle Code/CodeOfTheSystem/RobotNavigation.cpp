#include "RobotNavigation.h"

// Constructor: Assign passed sensor and motor objects
RobotNavigation::RobotNavigation(UltrasonicSensor* right, UltrasonicSensor* left, MotorControl* motorControl, ColorSensor* red) {
  rightSensor = right;
  leftSensor = left;
  motor = motorControl;
  red = red;
}

void RobotNavigation::begin() {
  pinMode(LINE_SENSOR_LEFT, INPUT);
  pinMode(LINE_SENSOR_RIGHT, INPUT);
}

// Change robot's state and act accordingly
void RobotNavigation::setRobotState(int state) {
  currentState = state;

  // Execute behavior for each state
  switch (currentState) {

    case OBSTACLE_AVOID:
      obstacleStep = 0;
      handleObstacleAvoidance(); // Begin obstacle avoidance
      break;

    case LEFT:
      while (digitalRead(LINE_SENSOR_RIGHT) == LOW)
          motor->turnLeft(DefaultTurnSpeed); // Turn left until the right IR won't lose the line
      motor->stopMotors();
      break;

    case RIGHT:
      while (digitalRead(LINE_SENSOR_LEFT) == LOW)
          motor->turnRight(DefaultTurnSpeed); // Turn right until the left IR won't lose the line
      motor->stopMotors();
      break;

    case LINE_SEARCH:
      handleLineSearch(); // Begin line search
      break;
  }
}

// Main logic loop: process line and obstacle sensors
void RobotNavigation::updateNavigation() {

  // Read line sensors
  int leftLine = digitalRead(LINE_SENSOR_LEFT);
  int rightLine = digitalRead(LINE_SENSOR_RIGHT);

  // Read obstacle sensors
  float rightDistance = rightSensor->getDistance();
  float leftDistance = leftSensor->getDistance();

  // Choose action based on sensor readings
  if (((rightDistance < maxDistance && rightDistance > minDistance) ||
      (leftDistance < maxDistance && leftDistance > minDistance)) &&
      !red->ColorSensorObserve()) {
    motor->stopMotors();
    delay(300);
    setRobotState(OBSTACLE_AVOID);  // Obstacle detected
  }
  if (leftLine == HIGH && rightLine == LOW) {
    motor->stopMotors();
    setRobotState(LEFT);  // Left line lost: turn left
  } else if (leftLine == LOW && rightLine == HIGH) {
    motor->stopMotors();
    setRobotState(RIGHT);  // Right line lost: turn right
  } else if (leftLine == LOW && rightLine == LOW) {
    if (currentState == LEFT) {
      motor->stopMotors();
      setRobotState(RIGHT);
    } else if (currentState == RIGHT) {
      motor->stopMotors();
      setRobotState(LEFT);
    }
  }
}

// Behavior for obstacle avoidance sequence
void RobotNavigation::handleObstacleAvoidance() {
  switch (obstacleStep) {
    // Step 0: Move backward to clear way for maneuver
    case 0:
      motor->stopMotors();
      delay(100);
      motor->moveBackward(DefaultBackwardSpeed);
      delay(200);
      obstacleStep++;
      break;
    // Step 1: Rotate left before bypass
    case 1:
      motor->rotateLeft(80);
      delay(600);
      motor->stopMotors();
      delay(1000);
      obstacleStep++;
      break;
  }
  // Check obstacle
  float rightDistance = rightSensor->getDistance();
  float leftDistance = leftSensor->getDistance();

  // Obstacle detected
  if ((rightDistance < 100 && rightDistance > 3) ||
      (leftDistance < 100 && leftDistance > 3)) {

    // Rotate back on the line to switch the direction on bypass
    motor->rotateRight(68);
    while (true) {
      bool leftLine = digitalRead(LINE_SENSOR_LEFT);
      bool rightLine = digitalRead(LINE_SENSOR_RIGHT);
      if (rightLine == HIGH) {
        motor->stopMotors();
        delay(500);
        break;
      }
    }
    // Bypass the obstacle
    motor->rotateRight(75);
    delay(500);
    motor->stopMotors();
    delay(700);
    motor->moveForward(75);
    delay(750);
    motor->stopMotors();
    delay(600);
    motor->rotateLeft(75);
    delay(650);
    motor->stopMotors();
    delay(700);
    motor->moveForward(75);
    delay(800);
    motor->stopMotors();
    delay(600);
    motor->turnLeft(75);
    delay(600);
    motor->stopMotors();
    delay(1000);
    currentState = LINE_SEARCH;
    setRobotState(LINE_SEARCH);

  } else {
    motor->rotateRight(75);
    delay(350);
    motor->stopMotors();
    delay(500);
    motor->moveForward(70);
    delay(900);
    motor->stopMotors();
    delay(1000);
    motor->rotateRight(70);
    delay(500);
    motor->stopMotors();
    delay(500);
    currentState = LINE_SEARCH;
    setRobotState(LINE_SEARCH);
  }
}

void RobotNavigation::handleLineSearch() {
  bool foundLine = false;
  unsigned long startTime = millis();
  while (millis() - startTime < searchTimeout) {
    motor->turnLeft(120);
    delay(110);
    motor->stopMotors();
    delay(100);
    bool leftLine = digitalRead(LINE_SENSOR_LEFT);
    bool rightLine = digitalRead(LINE_SENSOR_RIGHT);
    if (leftLine == HIGH || rightLine == HIGH) {
      motor->stopMotors();
      delay(100);
      leftLine = digitalRead(LINE_SENSOR_LEFT);
      rightLine = digitalRead(LINE_SENSOR_RIGHT);
      if (leftLine == HIGH)
        while (rightLine == HIGH)
            motor->turnLeft(DefaultTurnSpeed);
      if (rightLine == HIGH)
        while (leftLine == HIGH)
            motor->turnRight(DefaultTurnSpeed);
      motor->stopMotors();
      delay(100);
      motor->moveForward(DefaultForwardSpeed);
      delay(100);
      foundLine = true;
      break;
      return;
    }

    motor->turnRight(DefaultTurnSpeed);
    delay(100);
    motor->stopMotors();
    delay(100);
    leftLine = digitalRead(LINE_SENSOR_LEFT);
    rightLine = digitalRead(LINE_SENSOR_RIGHT);
    if (leftLine == HIGH || rightLine == HIGH) {
      motor->stopMotors();
      delay(100);
      if (leftLine == HIGH)
        while (leftLine == HIGH) motor->turnRight(DefaultTurnSpeed);
      if (rightLine == HIGH)
        while (rightLine == HIGH) motor->turnLeft(DefaultTurnSpeed);
      motor->stopMotors();
      delay(100);
      motor->moveForward(DefaultForwardSpeed);
      delay(100);
      foundLine = true;
      break;
    }
  }
}
