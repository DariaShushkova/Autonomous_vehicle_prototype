#include "UltrasonicSensor.h"

// Constructor: store trigger and echo pins
UltrasonicSensor::UltrasonicSensor(int trig, int echo) {
  trigPin = trig;
  echoPin = echo;
}

// Set trigger as output and echo as input
void UltrasonicSensor::begin() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

// Measure distance using ultrasonic sensor and return value in cm
float UltrasonicSensor::getDistance() {
  // Ensure clean LOW pulse before sending trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send 10µs HIGH pulse to start measurement
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure duration of echo signal (time between trigger and echo return)
  long duration = pulseIn(echoPin, HIGH, 20000); // Timeout after 20ms

  // Convert duration to distance in centimeters
  return duration * 0.01723;  // Speed of sound formula
}
