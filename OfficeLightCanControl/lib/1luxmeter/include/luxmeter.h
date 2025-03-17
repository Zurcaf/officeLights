// luxmeter.h
#ifndef LUXMETER_H
#define LUXMETER_H

#include <Arduino.h>
#include <math.h>
#include <tuple>

class LuxMeter
{
public:
    // Constructor
    LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange, int dacRange);

    // Set calibration based on the ID of the device
    bool setCalibration(uint8_t *id);

    // Calculate all values and return as a tuple
    std::tuple<float, float, float, float> calculateAllValues();

    // Get lux value directly
    float getLuxValue();

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
