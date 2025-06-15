// Motor pins
#define LEFT_MOTOR_SPEED_PIN 10
#define RIGHT_MOTOR_SPEED_PIN 11
#define LEFT_MOTOR_FORWARD_PIN 2
#define LEFT_MOTOR_BACKWARD_PIN 3
#define RIGHT_MOTOR_FORWARD_PIN 4
#define RIGHT_MOTOR_BACKWARD_PIN 5

// IR Line Following Sensors
#define leftIR 13
#define rightIR 12

// Speed
#define LEFT_SPEED 70
#define RIGHT_SPEED 70

// Ultrasonic Distance Sensors
#define Left_echoPin 9
#define Left_trigPin 8
#define Right_echoPin 7
#define Right_trigPin 6

// --- NEW COMPONENT: COLOR SENSOR (TCS3200) ---
// Using available Analog pins for the new sensor
#define S0_PIN A0
#define S1_PIN A1
#define S2_PIN A2
#define S3_PIN A3
#define SENSOR_OUT_PIN A5  // Renamed 'OUT' to avoid confusion

long left_duration, right_duration;
float left_distance, right_distance;
bool obstacle = 0;
bool left_IR;
bool right_IR;

// NEW VARIABLES FOR COLOR SENSOR
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup() {
  // Initialize motors, IR and Ultrasonic
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

  // Initialize Color Sensor pins
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  pinMode(SENSOR_OUT_PIN, INPUT);

  // Set frequency for Color Sensor
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, HIGH);

  // Serial for debugging
  Serial.begin(9600);
}

void loop() {
  // Test color detection
  readColorValues();  // Update the color readings

  // Check red color. If red is detected, stop.
  // The frequency is inversely proportional to the light intensity, so a lower frequency means more red.
  if (redFrequency < greenFrequency && redFrequency < blueFrequency && redFrequency < 80) {
    Serial.println("RED DETECTED! STOPPING...");
    Serial.print("R: ");
    Serial.print(redFrequency);
    Serial.print(" G: ");
    Serial.print(greenFrequency);
    Serial.print(" B: ");
    Serial.println(blueFrequency);

    stopMotors();
    delay(1000);

  } else {
    // If no red is detected, execute the standart behavior
    left_IR = digitalRead(leftIR);
    right_IR = digitalRead(rightIR);

    // Line Following Logic
    if (left_IR == HIGH && right_IR == LOW) {
      stopMotors();
      turnLeft();
    } else if (left_IR == LOW && right_IR == HIGH) {
      stopMotors();
      turnRight();
    } else {
      // No line detected, check for obstacle
      float leftDist = getLeftDistance();
      float rightDist = getRightDistance();
      stopMotors();

      if ((leftDist > 2 && rightDist > 2) && (leftDist < 20 || rightDist < 20)) {
        obstacle = 1;

        // Maneuver around obstacle
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
        delay(200);

        // Try to find the line again using zigzag pattern
        bool foundLine = false;
        unsigned long startTime = millis();

        while (millis() - startTime < 4000) {  // Search for 4 seconds
          // Small left curve
          turnLeft();
          delay(360);
          left_IR = digitalRead(leftIR);
          right_IR = digitalRead(rightIR);
          if (left_IR == HIGH || right_IR == HIGH) {
            stopMotors();
            delay(150);
            obstacle = 0;  // Clear flag
            foundLine = true;
            break;
          }

          // Small right curve
          turnRight();
          delay(320);
          left_IR = digitalRead(leftIR);
          right_IR = digitalRead(rightIR);
          if (left_IR == HIGH || right_IR == HIGH) {
            stopMotors();
            delay(150);
            obstacle = 0;  // Clear flag
            foundLine = true;
            break;
          }
        }

        // Failsafe if line not found
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
          startTime = millis();  // restart timeout

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
        // No obstacle, act based on current flag
        if (obstacle == 1) {
          stopMotors();
        } else if (obstacle == 0) {
          moveForward();
        }
      }
    }
  }

  // NEW FUNCTION: READ COLOR SENSOR 
  void readColorValues() {
    // Set filter to Red
    digitalWrite(S2_PIN, LOW);
    digitalWrite(S3_PIN, LOW);
    // Read the output frequency
    redFrequency = pulseIn(SENSOR_OUT_PIN, LOW);
    delay(50);  // Short delay for stability

    // Set filter to Green
    digitalWrite(S2_PIN, HIGH);
    digitalWrite(S3_PIN, HIGH);
    // Read the output frequency
    greenFrequency = pulseIn(SENSOR_OUT_PIN, LOW);
    delay(50);

    // Set filter to Blue
    digitalWrite(S2_PIN, LOW);
    digitalWrite(S3_PIN, HIGH);
    // Read the output frequency
    blueFrequency = pulseIn(SENSOR_OUT_PIN, LOW);
    delay(50);
  }

  // EXISTING FUNCTIONS (UNCHANGED)
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
    analogWrite(LEFT_MOTOR_SPEED_PIN, LEFT_SPEED);
    analogWrite(RIGHT_MOTOR_SPEED_PIN, RIGHT_SPEED);
    digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
  }

  void moveBackward() {
    analogWrite(LEFT_MOTOR_SPEED_PIN, LEFT_SPEED);
    analogWrite(RIGHT_MOTOR_SPEED_PIN, RIGHT_SPEED);
    digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
  }

  void turnRight() {
    analogWrite(RIGHT_MOTOR_SPEED_PIN, 85);
    digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
  }

  void turnLeft() {
    analogWrite(LEFT_MOTOR_SPEED_PIN, 85);
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
