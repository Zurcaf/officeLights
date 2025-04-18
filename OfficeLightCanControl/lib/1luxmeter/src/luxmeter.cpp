// luxmeter.cpp
#include <luxmeter.h>

LuxMeter::LuxMeter(int ldrPin, float vcc, float rFixed, int adcRange, int dacRange)
    : _ldrPin(ldrPin), _vcc(vcc), _rFixed(rFixed), _adcRange(adcRange), _dacRange(dacRange)
    {
        VOLT_PER_UNIT = _vcc / _adcRange;
        _m = 0;
        _b = 0;

        filteredAdcValue = 0;
        for (int i = 0; i < WINDOW_SIZE; i++)
            adcHistory[i] = 0;
        historyIndex = 0;
        runningSum = 0.0f;
        runningVariance = 0.0f; 
    }

void LuxMeter::setCalibration(float m, float b) 
{
    // Set calibration parameters
    _m = m;
    _b = b;
}


std::tuple<float, float, float, float> LuxMeter::calculateAllValues() {
   
    // 1. Convert to voltage with precalculated multiplier
    float voltage = filteredAdcValue * VOLT_PER_UNIT;
    
    // 2. Handle invalid cases
    if (voltage <= 0.001f) {  // Small threshold instead of 0
        return std::make_tuple(filteredAdcValue, voltage, -1.0f, 0.0f);
    }
    
    // 3. Optimized resistance calculation
    float vRatio = _vcc / voltage;
    float resistance = _rFixed * (vRatio - 1.0f);
    
    // 4. Calculate lux if resistance is valid
    float lux = 0.0f;
    if (resistance > 0.0f) {
        float logRes = log(resistance) / LN10;         // Convert to base 10
        lux = pow(10.0f, (logRes - _b) / _m);
    }
    
    return std::make_tuple(filteredAdcValue, voltage, resistance, lux);
}

float LuxMeter::getLuxValue() 
{
    // 1. Convert to voltage with precalculated multiplier    
    float voltage = filteredAdcValue * VOLT_PER_UNIT;
    if (voltage <= 0.001f)
        return 0.0f;
        
    // 2. Optimized resistance calculation
    float resistance = _rFixed * ((_vcc / voltage) - 1.0f);
    if (resistance <= 0.0f)
        return 0.0f;

    // 3. Calculate lux
    float logRes = log(resistance) / LN10;
    float lux = pow(10.0f, (logRes - _b) / _m);

    // 4. Clamp lux value to a valid range
    if (lux > MAX_LUX)
        return MAX_LUX;
    if (lux < MIN_LUX)
        return MIN_LUX;

    return lux;
}

float LuxMeter::getLdrVoltage() {
    // 1. Convert to voltage with precalculated multiplier
    float voltage = filteredAdcValue * VOLT_PER_UNIT;
    
    // 2. Handle invalid cases
    if (voltage <= 0.001f) {
        return 0.0f;
    }
    
    return voltage;
}

void LuxMeter::updateMovingAverage() {
    int newAdcValue = analogRead(_ldrPin);
    float currentAvg = (runningSum / WINDOW_SIZE);

    if (historyIndex < WINDOW_SIZE) {  // Filling the window
        updateHistory(newAdcValue);
    } else {  // Window is full, apply outlier rejection
        float deviation = abs(newAdcValue - currentAvg);
        float threshold = currentAvg * OUTLIER_THRESHOLD;
        if (deviation <= threshold || threshold == 0) {
            updateHistory(newAdcValue);
        }
    }

    filteredAdcValue = runningSum / WINDOW_SIZE;
}

void LuxMeter::updateHistory(int adcValue)
{
    runningSum -= adcHistory[historyIndex];
    runningSum += adcValue;
    adcHistory[historyIndex] = adcValue;

    // Update the history index for the next value (%) makes it a circular buffer
    historyIndex = (historyIndex + 1) % WINDOW_SIZE;
}

void LuxMeter::calibrate_bm(unsigned long currentMillis, float dutyCycle)
{
      // Call calculateAllValues and get the results
      auto [adcValue,voltage, resistance, lux] = calculateAllValues();

      Serial.printf("%lu, %.1f, %.2f, %.2fV, %.2f, %.2f\n",
                    currentMillis, dutyCycle, adcValue, voltage, resistance, lux);
      delay(50);
}

