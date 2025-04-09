// luxmeter.h
#ifndef LUXMETER_H
#define LUXMETER_H

#include <Arduino.h>
#include <math.h>
#include <tuple>

// Luxmeter class definition
// This class is responsible for reading the LDR sensor and calculating the lux value
// It uses a moving average filter to smooth out the readings and applies calibration coefficients
// based on the unique ID of the device
// Arguments:
// - ldrPin: Analog pin connected to the LDR sensor
// - vcc: Supply voltage (default is 3.3V)
// - rFixed: Fixed resistor value in the voltage divider (default is 10k ohm)
// - adcRange: ADC range (default is 4096 for 12-bit ADC)
// - dacRange: DAC range (default is 4096 for 12-bit DAC)
// methods :
// - setCalibration: Set calibration coefficients based on the unique ID of the device
// - calculateAllValues: Calculate and return the filtered ADC value, voltage, resistance, and lux value
// - getLuxValue: Get the lux value directly
// - updateMovingAverage: Update the moving average of the ADC readings
class LuxMeter
{
public:
    // Constructor
    LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange, int dacRange);

    // Set calibration based on the ID of the device
    void setCalibration(float m, float b);

    // Calculate all values and return as a tuple
    std::tuple<float, float, float, float> calculateAllValues();

    // Get lux value directly
    float getLuxValue();

    // Get LDR voltage value
    float getLdrVoltage();

    // Helper method to update history and running sum
    void updateMovingAverage();

    void calibrate_bm(unsigned long currentMillis, float dutuCycle);
    
    float filteredAdcValue;


private:
    // Original Lux calculation parameters
    int _ldrPin = A0;
    float _vcc = 3.3;
    float _rFixed = 10000;
    int _adcRange = 4096;
    int _dacRange = 4096;

    // Lux calculation parameters
    float VOLT_PER_UNIT = _vcc / _adcRange;
    static constexpr float LN10 = 2.302585092994046f; // ln(10)

    // ID of the device
    uint8_t *_id;

    // Calibration parameters
    float _m;
    float _b;

    // Moving average parameters
    static constexpr int WINDOW_SIZE = 8;
    float adcHistory[WINDOW_SIZE];
    int historyIndex;
    float runningSum;


    // Outlier rejection parameters
    static constexpr float OUTLIER_THRESHOLD = 2.0f; // Multiplier for deviation from average
    float runningVariance;                    // For calculating standard deviation

    // Lux value range
    static constexpr float MAX_LUX = 100000.0f;
    static constexpr float MIN_LUX = 0.001f;

    void updateHistory(int adcValue);
};

#endif
