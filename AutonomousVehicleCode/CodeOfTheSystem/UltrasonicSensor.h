#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>

// Ultrasonic sensor pin definitions
#define RIGHT_TRIG_PIN 6   // Trigger pin for right ultrasonic sensor
#define RIGHT_ECHO_PIN 7   // Echo pin for right ultrasonic sensor
#define LEFT_TRIG_PIN 8    // Trigger pin for left ultrasonic sensor
#define LEFT_ECHO_PIN 9    // Echo pin for left ultrasonic sensor

class UltrasonicSensor {
  private:
    int trigPin;   // Trigger pin used to send ultrasonic pulse
    int echoPin;   // Echo pin used to receive reflection

  public:
    UltrasonicSensor(int trig, int echo);  // Constructor with pin setup
    void begin();                          // Initialize sensor pins
    float getDistance();                   // Return distance in centimeters
};

#endif
