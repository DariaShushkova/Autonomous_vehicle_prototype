#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>

// Pin definitions
#define COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S0 4
#define COLOR_SENSOR_OUTPUT_FREQUENCY_INPUT_S1 5
#define COLOR_SENSOR_PHOTODIODE_INPUT_S2 6
#define COLOR_SENSOR_PHOTODIODE_INPUT_S3 7
#define COLOR_SENSOR_OUTPUT_PIN 8

class ColorSensor {
    private:
        int colorRedFrequency;
        int colorGreenFrequency;
        int colorBlueFrequency;

    public:
        void ColorSensorInit();       // Setup pins
        void ColorSensorObserve();    // Detect and print color
};

#endif
