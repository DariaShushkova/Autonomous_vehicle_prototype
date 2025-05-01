#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>

// Motor pins
#define ENA 10
#define ENB 11
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

// Ultrasonic sensors
#define TRIG_PIN_1 6
#define ECHO_PIN_1 7
#define TRIG_PIN_2 8
#define ECHO_PIN_2 9

// Line sensors
#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1
#define COLOR_THRESHOLD 100

// Robot states
#define FORWARD 0
#define LEFT 1
#define RIGHT 2
#define STOPPED 3
#define OBSTACLE_AVOID 4

#define DEBUG_USE_LCD  // Comment this line for real experiment (no LCD)

#ifdef DEBUG_USE_LCD
Adafruit_LiquidCrystal lcd_1(0);
#endif

class UltrasonicSensor {
  private:
    int trig, echo;
    
  public:
    UltrasonicSensor(int trigPin, int echoPin) : trig(trigPin), echo(echoPin) {}
    
    void begin() {
      pinMode(trig, OUTPUT);
      pinMode(echo, INPUT);
    }
    
    float getDistance() {
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      long duration = pulseIn(echo, HIGH, 20000);
      return duration * 0.01723;
    }
};

class MotorControl {
  private:
    void setMotors(int speedA, int speedB, int in1, int in2, int in3, int in4) {
      analogWrite(ENA, speedA);
      analogWrite(ENB, speedB);
      digitalWrite(IN1, in1);
      digitalWrite(IN2, in2);
      digitalWrite(IN3, in3);
      digitalWrite(IN4, in4);
    }
    
  public:
      // Adjusct speed depending on the state (for balancing)
    void moveForward(int speed) {
      setMotors(speed, speed, HIGH, LOW, HIGH, LOW);
    }
    
    void moveBackward(int speed) {
      setMotors(speed, speed, LOW, HIGH, LOW, HIGH);
    }
    
    void turnLeft(int speed) {
      setMotors(speed, speed, LOW, HIGH, HIGH, LOW);
    }
    
    void turnRight(int speed) {
      setMotors(speed, speed, HIGH, LOW, LOW, HIGH);
    }
    
    void stopMotors() {
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
    }
};

class RobotNavigation {
  private:
    int currentState;
    unsigned long previousTime = 0;
    int stateDelay = 100; // Minimum time between state changes (millis() instead of delay(100))
      int dynamicSpeed;
    
    UltrasonicSensor* rightSensor;
    UltrasonicSensor* leftSensor;
    MotorControl* motor;
    
    // For obstacle avoidance state machine
    unsigned long obstacleStartTime = 0;
    int obstacleStep = 0;
    bool inObstacleAvoidance = false;
    
  public:
    RobotNavigation(UltrasonicSensor* right, UltrasonicSensor* left, MotorControl* motorControl) {
      rightSensor = right;
      leftSensor = left;
      motor = motorControl;
      currentState = STOPPED;
    }
    
    void begin() {
      #ifdef DEBUG_USE_LCD
        lcd_1.clear();
        lcd_1.print("INIT");
      #endif
      
      currentState = STOPPED;
      motor->stopMotors();
      previousTime = millis();
    }
    
    void setRobotState(int state) {
      currentState = state;
      
      #ifdef DEBUG_USE_LCD
        lcd_1.clear();
      #endif
      
      switch (currentState) {
        case FORWARD:
          #ifdef DEBUG_USE_LCD
            lcd_1.print("forward");
          #endif
          motor->moveForward(200);
          break;
          
        case LEFT:
          #ifdef DEBUG_USE_LCD
            lcd_1.print("turn left");
          #endif
          motor->turnLeft(90);
          break;
          
        case RIGHT:
          #ifdef DEBUG_USE_LCD
            lcd_1.print("turn right");
          #endif
          motor->turnRight(90);
          break;
          
        case OBSTACLE_AVOID:
          #ifdef DEBUG_USE_LCD
            lcd_1.print("obstacle");
          #endif
          inObstacleAvoidance = true;
          obstacleStep = 0;
          obstacleStartTime = millis();
          break;
          
        case STOPPED:
          #ifdef DEBUG_USE_LCD
            lcd_1.print("stop");
          #endif
          motor->stopMotors();
          break;
      }
    }
    
    void updateNavigation(unsigned long currentTime) {
      
      if (inObstacleAvoidance) {
        handleObstacleAvoidance(currentTime);
        return;
      }
      
      // Small delay between state changes (millis() instead of delay(100))
      if (currentTime - previousTime < stateDelay) {
        return;
      }
      
      // Read sensor values
      int leftValue = analogRead(LEFT_SENSOR);
      int rightValue = analogRead(RIGHT_SENSOR);
      float rightDistance = rightSensor->getDistance();
      float leftDistance = leftSensor->getDistance();
      
      // Determine new state conditions
      int newState;
      
      if (leftValue >= COLOR_THRESHOLD && rightValue >= COLOR_THRESHOLD && rightDistance >= 20 && leftDistance >= 20) {
        newState = FORWARD;
      }
      else if (leftValue < COLOR_THRESHOLD && rightValue >= COLOR_THRESHOLD) {
        newState = LEFT;
      }
      else if (leftValue >= COLOR_THRESHOLD && rightValue < COLOR_THRESHOLD) {
        newState = RIGHT;
      }
      else if (rightDistance < 20 || leftDistance < 20) {
        newState = OBSTACLE_AVOID;
      }
      else {
        newState = STOPPED;
      }
      
      // If state changed, update it
      if (newState != currentState) {
        setRobotState(newState);
        previousTime = currentTime;
      }
    }
  
    void handleObstacleAvoidance(unsigned long currentTime) {
      float rightDistance = rightSensor->getDistance();
      float leftDistance = leftSensor->getDistance();
      
      // Only update LCD if step changes
      static int lastObstacleStep = -1;
      if (obstacleStep != lastObstacleStep) {
        lastObstacleStep = obstacleStep;
        
        #ifdef DEBUG_USE_LCD
          lcd_1.clear();
          // Print appropriate message based on obstacle step
          switch (obstacleStep) {
            case 0: lcd_1.print("obs:backward"); break;
            case 1: lcd_1.print("obs:left"); break;
            case 2: lcd_1.print("obs:forward1"); break;
            case 3: lcd_1.print("obs:right1"); break;
            case 4: lcd_1.print("obs:forward2"); break;
            case 5: lcd_1.print("obs:right2"); break;
            case 6: lcd_1.print("obs:forward3"); break;
          }
        #endif
      }
      
      switch (obstacleStep) {
        case 0: // Move backward
          motor->moveBackward(90);
          if (rightDistance >= 20 && leftDistance >= 20) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;
    
        case 1: // Turn left
          motor->turnLeft(100);
          if ((currentTime - obstacleStartTime >= 1000)) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;
    
        case 2: // Drive forward to bypass
          motor->moveForward(100);
          if ((currentTime - obstacleStartTime >= 3000)) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;
    
        case 3: // Turn right
          motor->turnRight(100);
          if ((currentTime - obstacleStartTime >= 1000)) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;
    
        case 4: // Drive forward to get around obstacle
          motor->moveForward(100);
          if ((currentTime - obstacleStartTime >= 3000)) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;
    
        case 5: // Final turn right
          motor->turnRight(100);
          if ((currentTime - obstacleStartTime >= 1000)) {
            obstacleStep++;
            obstacleStartTime = currentTime;
          }
          break;
    
        case 6: // Move forward until line is found or timeout
          if (analogRead(LEFT_SENSOR) < COLOR_THRESHOLD && analogRead(RIGHT_SENSOR) < COLOR_THRESHOLD) {
            motor->moveForward(90);
            
            // Check timeout
            if (currentTime - obstacleStartTime >= 5000) {  // 5-seconds timeout
              #ifdef DEBUG_USE_LCD
                lcd_1.clear();
                lcd_1.print("stopped");
              #endif
              motor->stopMotors();
              inObstacleAvoidance = false;
              currentState = STOPPED;
            }
            
          } else {
            #ifdef DEBUG_USE_LCD
              lcd_1.clear();
              lcd_1.print("obs:out");
            #endif
            inObstacleAvoidance = false;
            currentState = STOPPED;
          }
          break;
      }
    }
};

MotorControl motor;
UltrasonicSensor rightSensor(TRIG_PIN_1, ECHO_PIN_1);
UltrasonicSensor leftSensor(TRIG_PIN_2, ECHO_PIN_2);
RobotNavigation robotNav(&rightSensor, &leftSensor, &motor);

void setup() {
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize sensors
  rightSensor.begin();
  leftSensor.begin();
  
  // Initialize LCD for simulation
  #ifdef DEBUG_USE_LCD
    lcd_1.begin(16, 2);
  #endif
  
  // Initialize navigation
  robotNav.begin();
}

void loop() {
  unsigned long currentTime = millis();
  robotNav.updateNavigation(currentTime);
}
