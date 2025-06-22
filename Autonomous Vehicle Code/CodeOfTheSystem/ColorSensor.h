#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>

// Pin definitions
#define COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S0 A0
#define COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S1 A1
#define COLOR_SENSOR_PHOTODIODE_INPUT_S2 A2
#define COLOR_SENSOR_PHOTODIODE_INPUT_S3 A3
#define COLOR_SENSOR_OUTPUT_PIN A4

class ColorSensor {
    private:
        int colorRedFrequency;
        int colorGreenFrequency;
        int colorBlueFrequency;
        bool isRed = false;

    public:
        void ColorSensorInit();       // Setup pins
        bool ColorSensorObserve();    // Detect and print color
};

#endif
