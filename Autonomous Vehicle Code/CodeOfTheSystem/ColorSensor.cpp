#include "ColorSensor.h"

void ColorSensor::ColorSensorInit() {
    pinMode(COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S0, OUTPUT);
    pinMode(COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S1, OUTPUT);
    pinMode(COLOR_SENSOR_PHOTODIODE_INPUT_S2, OUTPUT);
    pinMode(COLOR_SENSOR_PHOTODIODE_INPUT_S3, OUTPUT);
    pinMode(COLOR_SENSOR_OUTPUT_PIN, INPUT);

    digitalWrite(COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S0, HIGH); // Full scale
    digitalWrite(COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S1, LOW);  // Frequency scaling = 20%
}

void ColorSensor::ColorSensorObserve() {
    // RED
    digitalWrite(COLOR_SENSOR_PHOTODIODE_INPUT_S2, LOW);
    digitalWrite(COLOR_SENSOR_PHOTODIODE_INPUT_S3, LOW);
    colorRedFrequency = pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW);

    // GREEN
    digitalWrite(COLOR_SENSOR_PHOTODIODE_INPUT_S2, HIGH);
    digitalWrite(COLOR_SENSOR_PHOTODIODE_INPUT_S3, HIGH);
    colorGreenFrequency = pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW);

    // BLUE
    digitalWrite(COLOR_SENSOR_PHOTODIODE_INPUT_S2, LOW);
    digitalWrite(COLOR_SENSOR_PHOTODIODE_INPUT_S3, HIGH);
    colorBlueFrequency = pulseIn(COLOR_SENSOR_OUTPUT_PIN, LOW);

    // Log
    Serial.print("R: "); Serial.print(colorRedFrequency);
    Serial.print(" G: "); Serial.print(colorGreenFrequency);
    Serial.print(" B: "); Serial.println(colorBlueFrequency);

    // Detect
    if (colorRedFrequency < colorGreenFrequency && colorRedFrequency < colorBlueFrequency)
        Serial.println("Detected Color: RED");
    else if (colorGreenFrequency < colorRedFrequency && colorGreenFrequency < colorBlueFrequency)
        Serial.println("Detected Color: GREEN");
    else if (colorBlueFrequency < colorRedFrequency && colorBlueFrequency < colorGreenFrequency)
        Serial.println("Detected Color: BLUE");
    else
        Serial.println("Detected Color: UNKNOWN");
}
