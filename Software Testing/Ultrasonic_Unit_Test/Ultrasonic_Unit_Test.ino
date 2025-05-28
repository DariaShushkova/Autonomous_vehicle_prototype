
#define Left_echoPin 9
#define Left_trigPin 8
#define Right_echoPin 7
#define Right_trigPin 6
long left_duration, right_duration;
float left_distance, right_distance;
void setup() {
  // put your setup code here, to run once:
  pinMode(Left_trigPin, OUTPUT);
  pinMode(Left_echoPin, INPUT);
  pinMode(Right_trigPin, OUTPUT);
  pinMode(Right_echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  //left ultrasonic sensor
digitalWrite(Left_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(Left_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Left_trigPin, LOW);
  left_duration = pulseIn(Left_echoPin, HIGH,12000);
  left_distance = left_duration * 0.034 / 2;
//Right ultrasonic sensor
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
  delay(100);

}
