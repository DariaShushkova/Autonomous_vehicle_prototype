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

#define DEBUG_USE_LCD  // Comment this line for real experiment (no LCD, no Serial)

#ifdef DEBUG_USE_LCD
Adafruit_LiquidCrystal lcd_1(0);
#endif

enum RobotState { FORWARD, LEFT, RIGHT, STOPPED, BACKWARD };
RobotState currentState = STOPPED;

class MotorControl {
public:
  void moveForward() {
    setMotors(200, 200, HIGH, LOW, HIGH, LOW);
  }
  void moveBackward() {
    setMotors(200, 200, LOW, HIGH, LOW, HIGH);
  }
  void turnLeft() {
    setMotors(200, 200, LOW, HIGH, HIGH, LOW);
  }
  void turnRight() {
    setMotors(200, 200, HIGH, LOW, LOW, HIGH);
  }
  void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  }

private:
  void setMotors(int speedA, int speedB, int in1, int in2, int in3, int in4) {
    analogWrite(ENA, speedA);
    analogWrite(ENB, speedB);
    digitalWrite(IN1, in1);
    digitalWrite(IN2, in2);
    digitalWrite(IN3, in3);
    digitalWrite(IN4, in4);
  }
};

class UltrasonicSensor {
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
private:
  int trig, echo;
};

MotorControl motor;
UltrasonicSensor rightSensor(TRIG_PIN_1, ECHO_PIN_1);
UltrasonicSensor leftSensor(TRIG_PIN_2, ECHO_PIN_2);

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  rightSensor.begin();
  leftSensor.begin();

#ifdef DEBUG_USE_LCD
  lcd_1.begin(16, 2);
  lcd_1.clear();
  lcd_1.print("INIT");
#endif
}

void loop() {
  int leftValue = analogRead(LEFT_SENSOR);
  int rightValue = analogRead(RIGHT_SENSOR);
  float rightDistance = rightSensor.getDistance();
  float leftDistance = leftSensor.getDistance();

  RobotState newState;

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

  if (newState != currentState) {
    currentState = newState;

#ifdef DEBUG_USE_LCD
    lcd_1.clear();
#endif

    switch (currentState) {
      case FORWARD:
#ifdef DEBUG_USE_LCD
        lcd_1.print("forward");
#endif
        motor.moveForward();
        break;

      case LEFT:
#ifdef DEBUG_USE_LCD
        lcd_1.print("turn left");
#endif
        motor.turnLeft();
        break;

      case RIGHT:
#ifdef DEBUG_USE_LCD
        lcd_1.print("turn right");
#endif
        motor.turnRight();
        break;

      case BACKWARD:
#ifdef DEBUG_USE_LCD
        lcd_1.print("backward");
#endif
        motor.moveBackward();
        break;

      case STOPPED:
#ifdef DEBUG_USE_LCD
        lcd_1.print("stop");
#endif
        motor.stopMotors();
        break;
    }
  }

  delay(100);
}
