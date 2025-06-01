#include "ColorSensor.h"

void ColorSensor::ColorSensorInit() {
    // Set the pins for the color sensor
    pinMode(COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S0, OUTPUT);
    pinMode(COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S1, OUTPUT);
    pinMode(COLOR_SENSOR_PHOTODIODE_INPUT_S2, INPUT);
    pinMode(COLOR_SENSOR_PHOTODIODE_INPUT_S3, INPUT);
    pinMode(COLOR_SENSOR_OUTPUT_PIN, INPUT);

    // Initialize the color sensor
    digitalWrite(COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S0, HIGH);
    digitalWrite(COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S1, LOW);
}