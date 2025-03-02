#include <Arduino.h>
#include <0configs.h>
#include "hardware/flash.h"
#include <luxmeter.h>

// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE);
bool unowned_rasp = true;
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);

    flash_get_unique_id(id);
    unowned_rasp = luxMeter.setCalibration(id);

    // Print the ID of the Raspberry Pi to avoid running the code on the wrong device
    while (unowned_rasp) {
        Serial.printf("ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
                      id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
        Serial.println("Unowned Raspberry Pi.");
        delay(5000);
    }
    // Launch moving average task on Core 0
    multicore_launch_core1(core1_task);
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

    // Call calculateAllValues and get the results
    auto [adcValue,voltage, resistance, lux] = luxMeter.calculateAllValues();


    Serial.printf("%lu, %.1f, %.2f, %.2fV, %.2f, %.2f\n",
                  currentMillis, dutyCycle / (float)DAC_RANGE, adcValue, voltage, resistance, lux);

    delay(50);
}

// Function to run on Core 0: Continuously update moving average
void core1_task() {
    while (true) {
        unsigned long currentMillis = millis();
        luxMeter.updateMovingAverage(currentMillis);
        delay(1);  // Small delay to prevent core from hogging resources
    }
}

