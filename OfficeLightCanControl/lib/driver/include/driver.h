#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>
#include <luxmeter.h>

class Driver {
private:
    int _ledPin;
    int _dacRange;
    int _stepSize;
    int _interval;

    float _dutyCycle;
    unsigned long _previousMillis;

public:
    // Constructor
    Driver(int ledPin, int dacRange, int stepSize, int interval);

    // Calibration method for PWM control
    int calibrate_bm(unsigned long currentMillis);
};

#endif