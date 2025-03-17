#include <Arduino.h>
#include "hardware/flash.h"

#include <0configs.h>
#include <luxmeter.h>
#include <driver.h>
#include <localController.h>
#include <pcInterface.h>
#include <dataStorageMetrics.h>
#include <canRoutines.h>

// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE, DAC_RANGE);

// Initialize Driver
Driver driver(LED_PIN, DAC_RANGE, STEP_SIZE, interval);

// PID controller instance
localController pidController(h, K, b, c, Ti, Td, Tt, officeLightsMode, N);
bool unowned_rasp = true;

float setpoint = 3.0f; // Setpoint for PID control

void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);

    flash_get_unique_id(id);
    unowned_rasp = luxMeter.setCalibration(id);

    // Print the ID of the Raspberry Pi to avoid running the code on the wrong device
    while (unowned_rasp)
    {
        Serial.printf("ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
                      id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
        Serial.println("Unowned Raspberry Pi.");
        delay(5000);
    }

    calibrate_Gd();

    Serial.printf("G: %f, d: %f\n", driver.G, driver.d);

    // After calibration, start PID control to maintain a setpoint (e.g., 3 lux)
    pidController.update_reference(setpoint);
}

void loop()
{
    unsigned long currentMillis = millis();
    
    if (currentMillis - LastUpdate_1000Hz >= FREQ_1000Hz)
    {
        LastUpdate_1000Hz = currentMillis;

        // Get current lux value
        luxMeter.updateMovingAverage();
    }

    if (currentMillis - LastUpdate_100Hz >= FREQ_100Hz)
    {
        LastUpdate_100Hz = currentMillis;

        // Get the current lux value
        float measuredLux = luxMeter.getLuxValue();

        // Update PID controller
        float dutyCycle = pidController.compute_control(); // Compute control output based on reference (r) and measured output (y)

        // Set the duty cycle (0-100%) using the Driver
        driver.setDutyCycle(dutyCycle); // Convert percentage to 0-1 for Driver

        pidController.housekeep(measuredLux); // Update internal state (housekeeping) for the PID controller

        // debug output
        // auto [adcValue, voltage, resistance, lux] = luxMeter.calculateAllValues();
        // Serial.printf("ADC: %.2f, Voltage: %.2f, Resistance: %.2f, Lux: %.2f\n", adcValue, voltage, resistance, lux);

        // Debug output
        Serial.printf("Setpoint: %.1f lux, Measured: %.1f lux, Duty Cycle: %.4f%%\n",
                      setpoint, measuredLux, dutyCycle);
    }
}

void calibrate_Mb()
{
    unsigned long currentMillis = millis();
    float dutyCycle = driver.calibrate_bm(currentMillis);
    luxMeter.calibrate_bm(currentMillis, dutyCycle);
}

void calibrate_Gd()
{
    // // Get the 0% duty cycle lux value
    // driver.setDutyCycle(0);
    // delay(5000);
    // float lux = luxMeter.getLuxValue();
    // // Serial.printf("Lux: %f\n", lux);

    // // Set the duty cycle to 70% and get the lux value
    // driver.setDutyCycle(0.7);
    // delay(5000);
    // float lux_70 = luxMeter.getLuxValue();
    // // Serial.printf("Lux 70: %f\n", lux_70);

    // // Calculate the gain and offset
    // float G = (lux_70 - lux) / 0.7;
    // float d = lux;

    // // Set the gain and offset
    // driver.setGainOffset(G, d);
    // // Serial.printf("G: %f, d: %f\n", G, d);

    delay(1000);
    driver.setGainOffset(4.986404, 0.702457);
    
}
