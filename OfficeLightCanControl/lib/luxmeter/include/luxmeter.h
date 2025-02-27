// luxmeter.h
#ifndef LUXMETER_H
#define LUXMETER_H

#include <Arduino.h>

class LuxMeter {
public:
    LuxMeter(int ldrPin, float vcc, float rFixed, float m, float b, int adcRange);
    float readLux();
    float readVoltage();
    float readResistance();
    int readRawADC();

private:
    int _ldrPin;
    float _vcc;
    float _rFixed;
    float _m;
    float _b;
    int _adcRange;

    float adcToVoltage(int adcValue);
    float voltageToResistance(float vAdc);
    float voltageToLux(float vAdc);
};

#endif
