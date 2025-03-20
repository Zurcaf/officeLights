#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>

// Driver class for controlling the LED
// This class is responsible for controlling the LED brightness using PWM
// and for calibrating the system using the b and m parameters.
// Arguments:
// ledPin: Pin number for the LED
// dacRange: Range of the DAC (0-4095 for 12-bit resolution)
// stepSize: Step size for calibration (0.1%)
// interval: Time interval for calibration in milliseconds
class Driver {
private:
    int _ledPin;
    int _dacRange;
    int _interval;

    float _stepSize;
    float _dutyCycle;
    unsigned long _previousMillis;

    bool manualDutyMode = false; // Flag for manual mode


public:
    // Constructor
    Driver(int ledPin, int dacRange, float stepSize, int interval);

    // Control of LED method for calibration values b and m
    // Returns the duty cycle
    float calibrate_bm(unsigned long currentMillis);

    float getDutyCycle();

    // Set the duty cycle of the LED
    void setDutyCycle(float dutyCycle);

    // Set the manual mode for the driver
    void setManualMode(bool manualMode);

    // Set Gain and offset d
    void setGainOffset(float _G, float _d);

    float G;
    float d;

};

#endif