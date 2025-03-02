// luxmeter.cpp
#include <luxmeter.h>

LuxMeter::LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange)
    : _ldrPin(ldrPin), _vcc(vcc), _rFixed(rFixed), _adcRange(adcRange) 
    {
        VOLT_PER_UNIT = _vcc / _adcRange;
    }

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

std::tuple<float, float, float, float> LuxMeter::calculateAllValues() {
    // 1. Take multiple samples for noise reduction
    constexpr int SAMPLE_COUNT = 5;
    constexpr int SAMPLE_DELAY_MS = 2;
    uint32_t adcSum = 0;
    
    // Accumulate samples
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        adcSum += analogRead(_ldrPin);
        if (i < SAMPLE_COUNT - 1) {
            delayMicroseconds(SAMPLE_DELAY_MS * 1000);
        }
    }
    
    // 2. Calculate average ADC value
    float avgAdc = static_cast<float>(adcSum) / SAMPLE_COUNT;
    
    // 3. Convert to voltage with precalculated multiplier
    float voltage = avgAdc * VOLT_PER_UNIT;
    
    // 4. Handle invalid cases
    if (voltage <= 0.001f) {  // Small threshold instead of 0
        return std::make_tuple(avgAdc, voltage, -1.0f, 0.0f);
    }
    
    // 5. Optimize resistance calculation
    float vRatio = _vcc / voltage;
    float resistance = _rFixed * (vRatio - 1.0f);
    
    // 6. Calculate lux if resistance is valid
    float lux = 0.0f;
    if (resistance > 0.0f) {
        float logRes = log(resistance) / LN10;         // Convert to base 10
        lux = pow(10.0f, (logRes - _b) / _m);
    }
    
    return std::make_tuple(avgAdc, voltage, resistance, lux);
}


float LuxMeter::getLuxValue() {
    // 1. Take multiple samples for noise reduction
    constexpr int SAMPLE_COUNT = 5;
    constexpr int SAMPLE_DELAY_MS = 2;
    uint32_t adcSum = 0;
    
    // Accumulate samples
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        adcSum += analogRead(_ldrPin);
        if (i < SAMPLE_COUNT - 1) {
            delayMicroseconds(SAMPLE_DELAY_MS * 1000);
        }
    }
    
    // 2. Calculate average ADC value
    float avgAdc = static_cast<float>(adcSum) / SAMPLE_COUNT;
    
    // 3. Convert to voltage with precalculated multiplier
    float voltage = avgAdc * VOLT_PER_UNIT;
    
    // 4. Early return for invalid voltage
    if (voltage <= 0.001f) {  // Small threshold instead of 0
        return 0.0f;
    }
    
    // 5. Optimize resistance calculation
    float vRatio = _vcc / voltage;
    float resistance = _rFixed * (vRatio - 1.0f);
    
    if (resistance <= 0.0f) {
        return 0.0f;
    }
    
    // 6. Optimize lux calculation using precomputed coefficients
    float logRes = log(resistance) / LN10;         // Convert to base 10
    float lux = pow(10.0f, (logRes - _b) / _m);
    
    return lux;
}