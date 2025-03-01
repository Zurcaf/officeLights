#include <Arduino.h>
#include <0hardware_config.h>
#include "hardware/flash.h"
#include <luxmeter.h>

// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE);

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);

    flash_get_unique_id(id);
    luxMeter.setCalibration(id);
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        dutyCycle += STEP_SIZE;
        if (dutyCycle > DAC_RANGE) {
            dutyCycle = 0;
            analogWrite(LED_PIN, dutyCycle);
            delay (3000);
        }
        analogWrite(LED_PIN, dutyCycle);
    }

    // Read the ADC value
    int adcValue = analogRead(LDR_PIN);

    // Call calculateAllValues and get the results
    auto [voltage, resistance, lux] = luxMeter.calculateAllValues(adcValue);


    Serial.printf("%lu, %.1f, %d, %.2fV, %.2f, %.2f\n",
                  currentMillis, dutyCycle / (float)DAC_RANGE, adcValue, voltage, resistance, lux);

    delay(50);
}
