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
#define BACKWARD 4

#define DEBUG_USE_LCD  // Comment this line for real experiment (no LCD, no Serial)

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
          
        case BACKWARD:
          #ifdef DEBUG_USE_LCD
            lcd_1.print("backward");
          #endif
          motor->moveBackward(90);
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
      else if ((leftValue >= COLOR_THRESHOLD && rightValue >= COLOR_THRESHOLD) && (rightDistance < 20 || leftDistance < 20)) {
        newState = BACKWARD;
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
