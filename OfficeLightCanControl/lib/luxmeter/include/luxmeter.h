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
    std::tuple<float, float, float, float> calculateAllValues();
    float getLuxValue();
;

private:
    float _ldrPin = A0;
    float _vcc = 3.3;
    float _rFixed =10000;
    int _adcRange = 4096;
    float VOLT_PER_UNIT = _vcc / _adcRange;
    static constexpr float LN10 = 2.302585092994046f;  // ln(10)


    uint8_t* _id;

    float _m = 0;
    float _b = 0;
};

#endif
