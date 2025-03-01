// luxmeter.h
#ifndef LUXMETER_H
#define LUXMETER_H

#include <Arduino.h>

class LuxMeter {
public:
    LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange);
    float readLux();
    float readVoltage();
    float readResistance();
    int readRawADC();
    void setCalibration(uint8_t* id);
    float _m = 0;
    float _b = 0;

private:
    int _ldrPin;
    float _vcc = 3.3;
    float _rFixed;

    uint8_t* _id;
    int _adcRange;

    float adcToVoltage(int adcValue);
    float voltageToResistance(float vAdc);
    float voltageToLux(float vAdc);
};

#endif
