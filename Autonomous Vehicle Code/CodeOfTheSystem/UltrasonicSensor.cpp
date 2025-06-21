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
  long duration = pulseIn(echoPin, HIGH, 22000); // Timeout after 22ms
  if (duration == 0) return 1000;

  // Convert duration to distance in centimeters
  return duration * 0.034 / 2;
}
