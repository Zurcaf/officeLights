// luxmeter.cpp
#include <luxmeter.h>
#include <math.h>

LuxMeter::LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange)
    : _ldrPin(ldrPin), _vcc(vcc), _rFixed(rFixed), _adcRange(adcRange) {}

void LuxMeter::setCalibration(uint8_t* id) {
    // Seting b and m according to the ID of the device (unique to each LDR sensor)
    // The specific ID checking for in byte array form
    uint8_t targetId1[8] = {0xE6, 0x60, 0xC0, 0xD1, 0xC7, 0x6F, 0x22, 0x2F};
    uint8_t targetId2[8] = {0xE6, 0x61, 0x18, 0x60, 0x4B, 0x84, 0x3A, 0x21};

    // Compare the id byte-by-byte
    if (id != nullptr && memcmp(id, targetId1, 8) == 0) {
        // If IDs match, set the m and b coefficients
        _m = -0.8;
        _b = 5.976;
    }else if (id != nullptr && memcmp(id, targetId2, 8) == 0) {
        // If IDs match, set the m and b coefficients
        _m = -0.9;
        _b = 6.976;
    }  
}


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