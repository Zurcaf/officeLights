// luxmeter.h
#ifndef LUXMETER_H
#define LUXMETER_H

#include <Arduino.h>
#include <math.h>
#include <tuple>

class LuxMeter {
public:
    LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange);
    void setCalibration(uint8_t* id);
    std::tuple<float, float, float> calculateAllValues(int adc_value);
;

private:
    int _ldrPin;
    float _vcc = 3.3;
    float _rFixed;
    int _adcRange;

    uint8_t* _id;

    float _m = 0;
    float _b = 0;
};

#endif
