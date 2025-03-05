#include <Arduino.h>
#include "hardware/flash.h"
#include <0configs.h>
#include <luxmeter.h>
#include <driver.h>

// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE, DAC_RANGE);

Driver driver(LED_PIN, DAC_RANGE, STEP_SIZE, interval);

// PID controller instance
//                 h,    K,    b,   Ti,    Td,      N
pid pidController(1.0f, 0.1f, 1.0f, 10.0f, 0.0f, 10.0f); // Default tuning (adjust as needed)

bool unowned_rasp = true;
bool uncalibrated = true;

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

    // Launch moving average task on Core 0
    multicore_launch_core1(core1_task);
}

void loop()
{
    if (uncalibrated)
    {
        calibrate_Gd();

        Serial.printf("G: %f, d: %f\n", driver.G, driver.d);
        delay(5000);

        // After calibration, start PID control to maintain a setpoint (e.g., 3 lux)

        // Update PID reference value
        pidController.update_reference(setpoint);
    }

    float measuredLux = luxMeter.getLuxValue(); // Get current illuminance from LuxMeter

    // Update PID controller
    float dutyCycle = pidController.compute_control(measuredLux);

    // Set the duty cycle (0-100%) using the Driver
    driver.setDutyCycle(dutyCycle); // Convert percentage to 0-1 for Driver

    // Debug output
    Serial.printf("Setpoint: %.1f lux, Measured: %.1f lux, Duty Cycle: %.1f%%\n",
                  setpoint, measuredLux, dutyCycle);

    delay(100); // Update every 100ms (adjust as needed for stability)
}

// Function to run on Core 0: Continuously update moving average
void core1_task()
{
    while (true)
    {
        unsigned long currentMillis = millis();
        luxMeter.updateMovingAverage(currentMillis);
        delay(1); // Small delay to prevent core from hogging resources

        
        
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
    // Get the 0% duty cycle lux value
    driver.setDutyCycle(0);
    delay(5000);
    float lux = luxMeter.getLuxValue();
    // Serial.printf("Lux: %f\n", lux);

    // Set the duty cycle to 70% and get the lux value
    driver.setDutyCycle(0.7);
    delay(5000);
    float lux_70 = luxMeter.getLuxValue();
    // Serial.printf("Lux 70: %f\n", lux_70);

    // Calculate the gain and offset
    float G = (lux_70 - lux) / 0.7;
    float d = lux;

    // Set the gain and offset
    driver.setGainOffset(G, d);
    // Serial.printf("G: %f, d: %f\n", G, d);
}
