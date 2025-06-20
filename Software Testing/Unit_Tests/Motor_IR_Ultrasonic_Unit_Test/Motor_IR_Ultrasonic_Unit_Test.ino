#define LEFT_MOTOR_SPEED_PIN 10
#define RIGHT_MOTOR_SPEED_PIN 11
#define LEFT_MOTOR_FORWARD_PIN 2
#define LEFT_MOTOR_BACKWARD_PIN 3
#define RIGHT_MOTOR_FORWARD_PIN 4
#define RIGHT_MOTOR_BACKWARD_PIN 5
#define leftIR 13
#define rightIR 12
#define LEFT_SPEED 70
#define RIGHT_SPEED 70
#define Left_echoPin 9
#define Left_trigPin 8
#define Right_echoPin 7
#define Right_trigPin 6

void stopMotors();
void turnLeft();
void turnRight();
float getLeftDistance();
float getRightDistance();
void moveBackward();
void moveForward();

long left_duration, right_duration;
float left_distance, right_distance;
bool obstacle = 0;
bool left_IR;
bool right_IR;
int state = 0;

void setup() {
  pinMode(LEFT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(Left_trigPin, OUTPUT);
  pinMode(Left_echoPin, INPUT);
  pinMode(Right_trigPin, OUTPUT);
  pinMode(Right_echoPin, INPUT);
}

void loop() {
  left_IR = digitalRead(leftIR);
  right_IR = digitalRead(rightIR);

  if (left_IR == HIGH && right_IR == LOW) {
    stopMotors();
    delay(20);
    state = 1;
    while (digitalRead(rightIR) == LOW) {
      turnLeft();
    }
    stopMotors();
  } else if (left_IR == LOW && right_IR == HIGH) {
    stopMotors();
    delay(20);
    state = 2;
    while (digitalRead(leftIR) == LOW) {
      turnRight();
    }
    stopMotors();

    float leftDist = getLeftDistance();
    float rightDist = getRightDistance();

    if ((leftDist > 2 && rightDist > 2) && (leftDist < 20 || rightDist < 20)) {
      obstacle = 1;

      stopMotors();
      delay(100);
      moveBackward();
      delay(1000);
      stopMotors();
      delay(100);
      turnRight();
      delay(600);
      stopMotors();
      delay(100);
      moveForward();
      delay(1350);
      stopMotors();
      delay(100);
      turnLeft();
      delay(800);
      stopMotors();
      delay(100);

      stopMotors();
      delay(200);

      bool foundLine = false;
      unsigned long startTime = millis();

      while (millis() - startTime < 4000) {
        turnLeft();
        delay(360);

        left_IR = digitalRead(leftIR);
        right_IR = digitalRead(rightIR);
        if (left_IR == HIGH || right_IR == HIGH) {
          stopMotors();
          delay(150);
          obstacle = 0;
          foundLine = true;
          break;
        }

        turnRight();
        delay(320);

        left_IR = digitalRead(leftIR);
        right_IR = digitalRead(rightIR);
        if (left_IR == HIGH || right_IR == HIGH) {
          stopMotors();
          delay(150);
          obstacle = 0;
          foundLine = true;
          break;
        }
      }

      while (!foundLine) {
        turnLeft();
        delay(800);
        stopMotors();
        delay(200);
        left_IR = digitalRead(leftIR);
        right_IR = digitalRead(rightIR);
        if (left_IR == HIGH || right_IR == HIGH) {
          stopMotors();
          delay(150);
          obstacle = 0;
          foundLine = true;
          break;
        }
        startTime = millis();

        while (millis() - startTime < 4000) {
          turnLeft();
          delay(360);

          left_IR = digitalRead(leftIR);
          right_IR = digitalRead(rightIR);
          if (left_IR == HIGH || right_IR == HIGH) {
            stopMotors();
            delay(150);
            obstacle = 0;
            foundLine = true;
            break;
          }

          turnRight();
          delay(320);

          left_IR = digitalRead(leftIR);
          right_IR = digitalRead(rightIR);
          if (left_IR == HIGH || right_IR == HIGH) {
            stopMotors();
            delay(150);
            obstacle = 0;
            foundLine = true;
            break;
          }
        }
      }
    } else {
      if (obstacle == 1) {
        stopMotors();
      }
    }
  } else if (left_IR == LOW && right_IR == LOW) {
    // Zigzag to search for line based on last known state
    if (state == 1) {
      turnLeft();
      delay(100);
      stopMotors();
      delay(10);
    } else if (state == 2) {
      turnRight();
      delay(100);
      stopMotors();
      delay(10);
    } else {
      // Unknown last direction, zigzag alternately
      turnLeft();
      delay(150);
      stopMotors();
      delay(10);
      turnRight();
      delay(150);
      stopMotors();
      delay(10);
    }
  } else {
    moveForward();
  }
}

float getLeftDistance() {
  digitalWrite(Left_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Left_trigPin, LOW);
  left_duration = pulseIn(Left_echoPin, HIGH, 22000);
  return left_duration * 0.034 / 2;
}

float getRightDistance() {
  digitalWrite(Right_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Right_trigPin, LOW);
  right_duration = pulseIn(Right_echoPin, HIGH, 22000);
  return right_duration * 0.034 / 2;
}

void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
  analogWrite(LEFT_MOTOR_SPEED_PIN, LEFT_SPEED);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, RIGHT_SPEED);
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(LEFT_MOTOR_SPEED_PIN, LEFT_SPEED);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, RIGHT_SPEED);
}

void turnRight() {
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(LEFT_MOTOR_SPEED_PIN, 120);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 0);
}

void turnLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 120);
  analogWrite(LEFT_MOTOR_SPEED_PIN, 0);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
  analogWrite(LEFT_MOTOR_SPEED_PIN, 0);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 0);
}