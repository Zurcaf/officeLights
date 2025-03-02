#include <Arduino.h>
#include "hardware/flash.h"
#include <0configs.h>
#include <luxmeter.h>
#include <driver.h>

// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE, DAC_RANGE);

Driver driver(LED_PIN, DAC_RANGE, STEP_SIZE, interval);

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

    // Call calibrate_bm and get the results
    int dutyCycle = driver.calibrate_bm(currentMillis);

    // Call calculateAllValues and get the results
    luxMeter.calibrate_bm(currentMillis, dutyCycle);
}

// Function to run on Core 0: Continuously update moving average
void core1_task() {
    while (true) {
        unsigned long currentMillis = millis();
        luxMeter.updateMovingAverage(currentMillis);
        delay(1);  // Small delay to prevent core from hogging resources
    }
}

