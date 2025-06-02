#include <Adafruit_LiquidCrystal.h>

// Define pins for motors
#define LEFT_MOTOR_SPEED_PIN 10       // PWM pin controlling left motor speed
#define RIGHT_MOTOR_SPEED_PIN 11      // PWM pin controlling right motor speed
#define LEFT_MOTOR_FORWARD_PIN 2      // Digital pin to move left motor forward
#define LEFT_MOTOR_BACKWARD_PIN 3     // Digital pin to move left motor backward
#define RIGHT_MOTOR_FORWARD_PIN 4     // Digital pin to move right motor forward
#define RIGHT_MOTOR_BACKWARD_PIN 5    // Digital pin to move right motor backward

// Define pins for ultrasonic sensors
#define RIGHT_TRIG_PIN 6   // Trigger pin for right ultrasonic sensor
#define RIGHT_ECHO_PIN 7   // Echo pin for right ultrasonic sensor
#define LEFT_TRIG_PIN 8    // Trigger pin for left ultrasonic sensor
#define LEFT_ECHO_PIN 9    // Echo pin for left ultrasonic sensor

// Define pins for line sensors
#define LINE_SENSOR_LEFT 12    // Left line sensor digital input
#define LINE_SENSOR_RIGHT 13   // Right line sensor digital input

#define DEBUG_USE_LCD          // To disable LCD, comment this out

// Robot states
#define FORWARD 0
#define LEFT 1
#define RIGHT 2
#define STOPPED 3
#define OBSTACLE_AVOID 4

// Speed configurations
#define speedOnForward 150
#define speedOnLeft 90
#define speedOnRight 90
#define speedOnBackward 90

// LCD initialization
#ifdef DEBUG_USE_LCD
Adafruit_LiquidCrystal lcd(0); // LCD with I2C address 0 (used in simulation)
#endif

class UltrasonicSensor {
  private:
    int trigPin;   // Trigger pin used to send ultrasonic pulse
    int echoPin;   // Echo pin used to receive reflection

  public:
    // Constructor: store trigger and echo pins
    UltrasonicSensor(int trig, int echo) {
      trigPin = trig;
      echoPin = echo;
    }

    // Set trigger as output and echo as input
    void begin() {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }

    // Measure distance using ultrasonic sensor and return value in cm
    float getDistance() {
      // Ensure clean LOW pulse before sending trigger
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);

      // Send 10µs HIGH pulse to start measurement
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Measure duration of echo signal (time between trigger and echo return)
      long duration = pulseIn(echoPin, HIGH, 20000); // Timeout after 20ms

      // Convert duration to distance in centimeters
      return duration * 0.01723;  // Speed of sound formula
    }
};

class MotorControl {
  private:
    // Internal helper function to apply motor direction and speed
    void setMotorStates(int leftSpeed, int rightSpeed,
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

  public:
    // Move robot straight forward at given speed
    void moveForward(int speed) {
      setMotorStates(speed, speed, HIGH, LOW, HIGH, LOW);
    }

    // Move robot straight backward at given speed
    void moveBackward(int speed) {
      setMotorStates(speed, speed, LOW, HIGH, LOW, HIGH);
    }

    // Rotate robot to the left (left wheel backward, right wheel forward)
    void turnLeft(int speed) {
      setMotorStates(speed, speed, LOW, HIGH, HIGH, LOW);
    }

    // Rotate robot to the right (left wheel forward, right wheel backward)
    void turnRight(int speed) {
      setMotorStates(speed, speed, HIGH, LOW, LOW, HIGH);
    }

    // Stop both motors by setting speed to 0
    void stopMotors() {
      analogWrite(LEFT_MOTOR_SPEED_PIN, 0);
      analogWrite(RIGHT_MOTOR_SPEED_PIN, 0);
    }
};

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
    // Constructor: Assign passed sensor and motor objects
    RobotNavigation(UltrasonicSensor* right, UltrasonicSensor* left, MotorControl* motorControl) {
      rightSensor = right;
      leftSensor = left;
      motor = motorControl;
      currentState = STOPPED; // Default state on boot
    }

    // Initialize LCD and stop motors
    void begin() {
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
    void setRobotState(int state) {
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
          motor->moveForward(speedOnForward);   // Drive forward at speed 200
          break;

        case LEFT:
    #ifdef DEBUG_USE_LCD
          lcd.print("Turning Left");
    #endif
          motor->turnLeft(speedOnLeft);       // Slow turn left
          break;

        case RIGHT:
    #ifdef DEBUG_USE_LCD
          lcd.print("Turning Right");
    #endif
          motor->turnRight(speedOnRight);      // Slow turn right
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
    void updateNavigation(unsigned long currentTime) {
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
    void handleObstacleAvoidance(unsigned long currentTime) {
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
          motor->moveBackward(speedOnBackward);
          if ((rightDistance >= 20 || rightDistance == 0) && (leftDistance >= 20 || leftDistance == 0)) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;

        case 1:
          // Step 1: Turn left to start bypass
          motor->turnLeft(speedOnLeft);
          if (currentTime - obstacleStartTime >= 1000) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;
        case 2:
          // Step 2: Move forward around obstacle
          motor->moveForward(speedOnForward - 50);
          if (currentTime - obstacleStartTime >= 3000) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;

        case 3:
          // Step 3: Turn right to realign
          motor->turnRight(speedOnRight);
          if (currentTime - obstacleStartTime >= 1000) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;

        case 4:
          // Step 4: Move forward again
          motor->moveForward(speedOnForward - 50);
          if (currentTime - obstacleStartTime >= 3000) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;

        case 5:
          // Step 5: Turn right again to face line
          motor->turnRight(speedOnRight);
          if (currentTime - obstacleStartTime >= 1000) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;

        case 6:
          // Step 6: Search for line or timeout
          if (digitalRead(LINE_SENSOR_LEFT) == LOW &&
              digitalRead(LINE_SENSOR_RIGHT) == LOW) {
            motor->moveForward(speedOnForward - 50);

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
};

// ============================= GLOBAL OBJECTS =============================
// Create hardware instances
MotorControl motor;
UltrasonicSensor rightUltrasonic(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
UltrasonicSensor leftUltrasonic(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
RobotNavigation robotNav(&rightUltrasonic, &leftUltrasonic, &motor);

// ============================= ARDUINO FUNCTIONS =============================
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

void loop() {
  unsigned long currentTime = millis();     // Get current time
  robotNav.updateNavigation(currentTime);   // Execute robot logic
}
