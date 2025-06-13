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
#define LEFT_SPEED 70  // Slightly higher for weaker left motor
#define RIGHT_SPEED 70
#define Left_echoPin 9
#define Left_trigPin 8
#define Right_echoPin 7
#define Right_trigPin 6
long left_duration, right_duration;
float left_distance, right_distance;


void setup() {
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD_PIN, OUTPUT);
  pinMode(leftIR,INPUT);
  pinMode(rightIR,INPUT);
  Serial.begin(9600);
  
}

void loop() {
  bool left_IR=digitalRead(leftIR);
  bool right_IR=digitalRead(rightIR);
  if(left_IR==HIGH && right_IR==LOW){
    stopMotors();
    
    turnLeft();
    
    
   

  }
  else if(left_IR==LOW && right_IR==HIGH){
    stopMotors();
    
    turnRight();
    
    
    
  }
  else if(left_IR==HIGH && right_IR==HIGH){
    stopMotors();
   
  }
  else{
    moveForward();
    
  }
 


  
}

void moveForward() {
  analogWrite(LEFT_MOTOR_SPEED_PIN, LEFT_SPEED);
  analogWrite(RIGHT_MOTOR_SPEED_PIN, RIGHT_SPEED);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}
void turnRight(){
   
  analogWrite(RIGHT_MOTOR_SPEED_PIN, 85);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);
}
void turnLeft(){
  analogWrite(LEFT_MOTOR_SPEED_PIN, 85);

  digitalWrite(LEFT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, LOW);
}
void stopMotors(){
   analogWrite(LEFT_MOTOR_SPEED_PIN, 255);
  analogWrite(RIGHT_MOTOR_SPEED_PIN,255);
  digitalWrite(LEFT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD_PIN, HIGH);

}
