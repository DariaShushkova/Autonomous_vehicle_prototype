// Motor pins - same as original
#define LEFT_MOTOR_SPEED_PIN 10
#define RIGHT_MOTOR_SPEED_PIN 11
#define LEFT_MOTOR_FORWARD_PIN 2
#define LEFT_MOTOR_BACKWARD_PIN 3
#define RIGHT_MOTOR_FORWARD_PIN 4
#define RIGHT_MOTOR_BACKWARD_PIN 5
#define leftIR 13
#define rightIR 12

// Minimal effective speeds (L298N needs at least ~60 PWM)
#define LEFT_SPEED 65  // Slightly higher for weaker left motor
#define RIGHT_SPEED 65

#define Left_echoPin 9
#define Left_trigPin 8
#define Right_echoPin 7
#define Right_trigPin 6

void stopMotors();
void turnLeft(int speed = 140);
void turnRight(int speed = 140);
float getLeftDistance();
float getRightDistance();
void moveBackward(int speed = 120);
void moveForward(int speed = 120);
void rotateLeft(int speed=120);
void rotateRight(int speed=120);
long left_duration, right_duration;
float left_distance, right_distance;
bool obstacle = 0;
bool left_IR = digitalRead(leftIR);
bool right_IR = digitalRead(rightIR);
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
  delay(400);
}

void loop() {
  left_IR = digitalRead(leftIR);
  right_IR = digitalRead(rightIR);
  float checkLeft = getLeftDistance();
  float checkRight = getRightDistance();
  if ((checkLeft < 20 && checkLeft > 3) || (checkRight < 20 && checkRight > 3)) {
    state = 0;
    stopMotors();
    delay(300);
    avoidObstacle();
  }
  if (left_IR == HIGH && right_IR == LOW) {
    stopMotors();
    state = 1;

    while (digitalRead(rightIR) == LOW) turnLeft();
    stopMotors();
  } else if (left_IR == LOW && right_IR == HIGH) {
    stopMotors();
    state = 2;

    while (digitalRead(leftIR) == LOW) turnRight();
    stopMotors();
  } else if (left_IR == LOW && right_IR == LOW) {
    if (state == 1) {
      stopMotors();
      while (digitalRead(rightIR) == LOW) turnRight();
      stopMotors();
      state = 0;
    } else if (state == 2) {
      stopMotors();
      while (digitalRead(leftIR) == LOW) turnLeft();
      stopMotors();
      state = 0;
    }
  }
}


void avoidObstacle() {
  stopMotors();
  delay(100);
  moveBackward(80);
  delay(200);
  rotateLeft(80);
  delay(600);
  stopMotors();
  delay(1000);
  float leftUltrasonic= getLeftDistance();
  float rightUltrasonic = getRightDistance();
  if ((leftUltrasonic <100 && leftUltrasonic >3) ||(rightUltrasonic <100 && rightUltrasonic >3) ){
    rotateRight(68);
    while(true){
      left_IR=digitalRead(leftIR);
      right_IR=digitalRead(rightIR);
      if(right_IR==HIGH){
        stopMotors();
        delay(500);
        break;
      }
    }
    rotateRight(75);
    delay(500);
    stopMotors();
    delay(700);
    moveForward(75);
    delay(750);
    stopMotors();
    delay(600);
    rotateLeft(75);
    delay(650);
  stopMotors();
    delay(700);




    moveForward(75);
    delay(800);
    stopMotors();
    delay(600);
    turnLeft(75);
    delay(600);
    stopMotors();
    delay(1000);
    bool foundLine = false;
    unsigned long startTime = millis();

    while (millis() - startTime < 100000) {
      turnLeft(120);
      delay(110);
      stopMotors();
      delay(100);
      left_IR = digitalRead(leftIR);
      right_IR = digitalRead(rightIR);
      if (left_IR == HIGH || right_IR == HIGH) {
        stopMotors();
        delay(100);
        left_IR = digitalRead(leftIR);
      right_IR = digitalRead(rightIR);
        if (left_IR == HIGH)
          while (digitalRead(rightIR) == HIGH) turnLeft();
        if (right_IR == HIGH)
          while (digitalRead(leftIR) == HIGH) turnRight();
        stopMotors();
        delay(100);
        moveForward();
        delay(100);
        foundLine = true;
        break;
        return;
      }


      turnRight();
      delay(100);
      stopMotors();
      delay(100);
      left_IR = digitalRead(leftIR);
      right_IR = digitalRead(rightIR);
      if (left_IR == HIGH || right_IR == HIGH) {
        stopMotors();
        delay(100);
        if (left_IR == HIGH)
          while (digitalRead(leftIR) == HIGH) turnRight();
        if (right_IR == HIGH)
          while (digitalRead(rightIR) == HIGH) turnLeft();
        stopMotors();
        delay(100);
        moveForward();
        delay(100);
        foundLine = true;
        break;
      }

  }}else{
    rotateRight(75);
    delay(350);
    stopMotors();
    delay(500);
    moveForward(70);
    delay(900);
    stopMotors();
    delay(1000);
    rotateRight(70);
    delay(500);
    stopMotors();
    delay(500);


    bool foundLine = false;
    unsigned long startTime = millis();

    while (millis() - startTime < 100000) {
      turnLeft(120);
      delay(110);
      stopMotors();
      delay(100);
      left_IR = digitalRead(leftIR);
      right_IR = digitalRead(rightIR);
      if (left_IR == HIGH || right_IR == HIGH) {
        stopMotors();
        delay(100);
        left_IR = digitalRead(leftIR);
      right_IR = digitalRead(rightIR);
        if (left_IR == HIGH)
          while (digitalRead(rightIR) == HIGH) turnLeft();
        if (right_IR == HIGH)
          while (digitalRead(leftIR) == HIGH) turnRight();
        stopMotors();
        delay(100);
        moveForward();
        delay(100);
        foundLine = true;
        break;
        return;
      }
      turnRight();
      delay(100);
      stopMotors();
      delay(100);
      left_IR = digitalRead(leftIR);
      right_IR = digitalRead(rightIR);
      if (left_IR == HIGH || right_IR == HIGH) {
        stopMotors();
        delay(100);
        if (left_IR == HIGH)
          while (digitalRead(leftIR) == HIGH) turnRight();
        if (right_IR == HIGH)
          while (digitalRead(rightIR) == HIGH) turnLeft();
        stopMotors();
        delay(100);
        moveForward();
        delay(100);
        foundLine = true;
        break;
      }

   

  }
  }
  
}

float getLeftDistance() {
  digitalWrite(Left_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Left_trigPin, LOW);
  left_duration = pulseIn(Left_echoPin, HIGH, 22000);
  if (left_duration == 0) return 1000;
  return left_duration * 0.034 / 2;
}

float getRightDistance() {
  digitalWrite(Right_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Right_trigPin, LOW);
  right_duration = pulseIn(Right_echoPin, HIGH, 22000);
  if (right_duration == 0) return 1000;
  return right_duration * 0.034 / 2;
}

void moveForward(int speed) {
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}

void moveBackward(int speed) {
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

void turnRight(int speed) {
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}

void turnLeft(int speed) {
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_SPEED_PIN, 255);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 255);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}
void rotateRight(int speed) {
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);

  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}
void rotateLeft(int speed) {
  analogWrite(RIGHT_MOTOR_SPEED_PIN, speed);
  analogWrite(LEFT_MOTOR_SPEED_PIN, speed);

  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}
