// Motor pins - same as original
#define LEFT_MOTOR_SPEED_PIN 10
#define RIGHT_MOTOR_SPEED_PIN 11
#define LEFT_MOTOR_FORWARD_PIN 2
#define LEFT_MOTOR_BACKWARD_PIN 3
#define RIGHT_MOTOR_FORWARD_PIN 4
#define RIGHT_MOTOR_BACKWARD_PIN 5

// Minimal effective speeds (L298N needs at least ~60 PWM)
#define LEFT_SPEED 65  // Slightly higher for weaker left motor
#define RIGHT_SPEED 60
#define TEST_DURATION 10000

void setup() {
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("FORWARD MOVEMENT TEST ONLY");
}

void loop() {
  // Test forward movement with minimal calibrated speeds
  Serial.println("Testing forward...");
  unsigned long startTime = millis();
  moveForward();
  while(millis() - startTime < TEST_DURATION) {
    Serial.print("L:");
    Serial.print(LEFT_SPEED);
    Serial.print(" R:");
    Serial.println(RIGHT_SPEED);
    delay(200); // Report every 200ms
  }
  
  // Stop and wait
  stopMotors();
  Serial.println("Stopped");
  delay(10000);
}

void moveForward() {
  analogWrite(LEFT_MOTOR_SPEED_PIN, LEFT_SPEED);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, RIGHT_SPEED);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_SPEED_PIN, 0);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 0);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
  
  // Report stop
  Serial.println("L:0 R:0");
}
