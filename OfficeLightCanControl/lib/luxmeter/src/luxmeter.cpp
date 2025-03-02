// luxmeter.cpp
#include <luxmeter.h>

LuxMeter::LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange)
    : _ldrPin(ldrPin), _vcc(vcc), _rFixed(rFixed), _adcRange(adcRange) 
    {
        VOLT_PER_UNIT = _vcc / _adcRange;
    }

bool LuxMeter::setCalibration(uint8_t* id) {
    // Seting b and m according to the ID of the device (unique to each LDR sensor)
    // The specific ID checking for in byte array form

    //ID of rapsberry pi A
    uint8_t targetId1[8] = {0xE6, 0x61, 0x18, 0x60, 0x4B, 0x84, 0x3A, 0x21};

    //ID of rapsberry pi B
    uint8_t targetId2[8] = {0xE6, 0x60, 0xC0, 0xD1, 0xC7, 0x6F, 0x22, 0x2F};

    // Compare the id byte-by-byte
    if (id != nullptr && memcmp(id, targetId1, 8) == 0) {
        // If IDs match, set the m and b coefficients for A
        _m = -0.8;
        _b = 5.976;
        return false;
    }else if (id != nullptr && memcmp(id, targetId2, 8) == 0) {
        // If IDs match, set the m and b coefficients for B
        _m = -0.8;
        _b = 5.976;
        return false;
    }else {
        return true;
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
    
    float voltage = filteredAdcValue * VOLT_PER_UNIT;
    if (voltage <= 0.001f)
        return 0.0f;

    float resistance = _rFixed * ((_vcc / voltage) - 1.0f);
    if (resistance <= 0.0f)
        return 0.0f;

    float logRes = log(resistance) / LN10;
    float lux = pow(10.0f, (logRes - _b) / _m);

    // Clamp the lux value to a valid range
    if (lux > MAX_LUX)
        return MAX_LUX;
    if (lux < MIN_LUX)
        return MIN_LUX;

    return lux;
}

void LuxMeter::updateMovingAverage(unsigned long currentMillis)
{
    if (currentMillis - lastUpdateMillis >= UPDATE_INTERVAL_MS)
    {
        int newAdcValue = analogRead(_ldrPin);
        float currentAvg = (runningSum / WINDOW_SIZE);

        // Calculate deviation from current average
        float deviation = abs(newAdcValue - currentAvg);

        // Simple initial case: accept first WINDOW_SIZE readings
        if (historyIndex < WINDOW_SIZE && runningSum == 0)
        {
            updateHistory(newAdcValue);
        }
        // Check if the new value is within acceptable range
        else
        {
            // Use a simpler threshold-based approach initially
            float threshold = currentAvg * OUTLIER_THRESHOLD; // e.g., 2x the average
            if (deviation <= threshold || threshold == 0)
            {
                updateHistory(newAdcValue);
            } // Else, skip this outlier
        }
        
        filteredAdcValue = runningSum / WINDOW_SIZE;
        lastUpdateMillis = currentMillis;
    }
}

void LuxMeter::updateHistory(int adcValue)
{
    runningSum -= adcHistory[historyIndex];
    runningSum += adcValue;
    adcHistory[historyIndex] = adcValue;

    // Update the history index for the next value (%) makes it a circular buffer
    historyIndex = (historyIndex + 1) % WINDOW_SIZE;
}
