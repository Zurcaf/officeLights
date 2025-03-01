// luxmeter.cpp
#include <luxmeter.h>

LuxMeter::LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange)
    : _ldrPin(ldrPin), _vcc(vcc), _rFixed(rFixed), _adcRange(adcRange) {}

void LuxMeter::setCalibration(uint8_t* id) {
    // Seting b and m according to the ID of the device (unique to each LDR sensor)
    // The specific ID checking for in byte array form

    //ID of rapsberry pi B
    uint8_t targetId1[8] = {0xE6, 0x60, 0xC0, 0xD1, 0xC7, 0x6F, 0x22, 0x2F};

    //ID of rapsberry pi A
    uint8_t targetId2[8] = {0xE6, 0x61, 0x18, 0x60, 0x4B, 0x84, 0x3A, 0x21};

    // Compare the id byte-by-byte
    if (id != nullptr && memcmp(id, targetId1, 8) == 0) {
        // If IDs match, set the m and b coefficients for B
        _m = -1.189;
        _b = 5.976;
    }else if (id != nullptr && memcmp(id, targetId2, 8) == 0) {
        // If IDs match, set the m and b coefficients
        _m = -1.189;
        _b = 5.976;
    }  
}

std::tuple<float, float, float> LuxMeter::calculateAllValues(int adc_value) {
    float voltage = (adc_value * _vcc) / _adcRange;
    float resistance = (voltage > 0) ? _rFixed * ((_vcc / voltage) - 1) : -1;
    float lux = (resistance > 0) ? pow(10, (log10(resistance) - _b) / _m) : 0;
    return std::make_tuple(voltage, resistance, lux);
}