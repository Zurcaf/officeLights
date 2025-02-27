// luxmeter.cpp
#include <luxmeter.h>
#include <math.h>

LuxMeter::LuxMeter(int ldrPin, float vcc, float rFixed, float m, float b, int adcRange)
    : _ldrPin(ldrPin), _vcc(vcc), _rFixed(rFixed), _m(m), _b(b), _adcRange(adcRange) {}

int LuxMeter::readRawADC() {
    return analogRead(_ldrPin);
}

float LuxMeter::adcToVoltage(int adcValue) {
    return (adcValue * _vcc) / _adcRange;
}

float LuxMeter::readVoltage() {
    return adcToVoltage(readRawADC());
}

float LuxMeter::voltageToResistance(float vAdc) {
    if (vAdc <= 0) return -1; // Avoid division by zero
    return _rFixed * ((_vcc / vAdc) - 1);
}

float LuxMeter::readResistance() {
    return voltageToResistance(readVoltage());
}

float LuxMeter::voltageToLux(float vAdc) {
    float rLdr = voltageToResistance(vAdc);
    if (rLdr <= 0) return 0;
    float logLux = (log10(rLdr) - _b) / _m;
    return pow(10, logLux);
}

float LuxMeter::readLux() {
    return voltageToLux(readVoltage());
}