#define Left_echoPin 9
#define Left_trigPin 8
#define Right_echoPin 7
#define Right_trigPin 6
#define IR_LEFT 13
#define IR_RIGHT 12

#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 10
#define ENB 11

long left_duration, right_duration;
float left_distance, right_distance;

void setup() {
  // Distance sensors
  pinMode(Left_trigPin, OUTPUT);
  pinMode(Left_echoPin, INPUT);
  pinMode(Right_trigPin, OUTPUT);
  pinMode(Right_echoPin, INPUT);

  // IR-sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Left Ultra sensor
  digitalWrite(Left_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Left_trigPin, LOW);
  left_duration = pulseIn(Left_echoPin, HIGH,12000);
  left_distance = left_duration * 0.034 / 2;

  // IR-sensors
  int left_state = digitalRead(IR_LEFT);
  int right_state = digitalRead(IR_RIGHT);

  // Right Utra sensor
  delay(50);
  digitalWrite(Right_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(Right_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Right_trigPin, LOW);
  right_duration = pulseIn(Right_echoPin, HIGH,12000);
  right_distance = right_duration * 0.034 / 2;

  // Print of values
  Serial.print("Left: ");
  Serial.print(left_distance);
  Serial.print(" cm | Right: ");
  Serial.print(right_distance);
  Serial.println(" cm");

  Serial.print("Left IR: ");
  Serial.print(left_state);
  Serial.print(" | Right IR: ");
  Serial.println(right_state);

  // If both IR = 1 → movie, otherwise stop
  if (left_state == 0 && right_state == 0) {
    // Movie foward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  }

  delay(100);
}
