
#define IR_LEFT 13
#define IR_RIGHT 12
int left_state,right_state;

void setup() {
 
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  Serial.begin(9600);
}

void loop() {
  left_state = digitalRead(IR_LEFT);
  right_state = digitalRead(IR_RIGHT);
  Serial.print("Left IR: ");
  Serial.print(left_state);
  Serial.print(" | Right IR: ");
  Serial.println(right_state);
    delay(100);
}
