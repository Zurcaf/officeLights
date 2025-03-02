#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>
#include <luxmeter.h>

class Driver {
private:
    int _ledPin;
    int _dacRange;
    int _interval;

    float _stepSize;
    float _dutyCycle;
    unsigned long _previousMillis;

    float _G;
    float _d;

public:
    // Constructor
    Driver(int ledPin, int dacRange, float stepSize, int interval);

    // Control of LED method for calibration values b and m
    // Returns the duty cycle
    float calibrate_bm(unsigned long currentMillis);

    // Set the duty cycle of the LED
    void setDutyCycle(float dutyCycle);

    // Set Gain and offset d
    void setGainOffset(float G, float d);

};

#endif