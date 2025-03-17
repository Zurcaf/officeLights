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

Driver driver(LED_PIN, DAC_RANGE, STEP_SIZE, interval);

// PID controller instance
//                             h,    K,    b,   Ti,    Td,      N
localController pidController(1.0f, 0.1f, 1.0f, 10.0f, 0.0f, 10.0f); // Default tuning (adjust as needed)

// Data storage metrics instance~
dataStorageMetrics metrics;

bool unowned_rasp = true;
bool uncalibrated = true;

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
        
    }
    delay(3000);
}

void loop()
{
    int current = millis();

    if (current - previousMillis >= 2000)
    {
        previousMillis = current;

        // insert debug values for testing
        metrics.insertValues(0.5, 100, 100, current);

        // Get buffer contents
        float uData[6000], yData[6000];
        int timestamps[6000];
        uint16_t count = metrics.getBuffer(uData, yData, timestamps);

        uint16_t elements = metrics.getBuffer(uData, yData, timestamps);

        // print the buffer contents
        for (uint16_t i = 0; i < elements; i++)
        {
            Serial.printf("Duty cycle: %f, Lux: %f, Timestamp: %d\n", uData[i], yData[i], timestamps[i]);
        }

        // Calculate energy consumption
        float energy = metrics.getEnergy();
        Serial.printf("Energy: %f\n", energy);

        // Calculate average visibility error
        float visibilityError = metrics.getVisibilityError();
        Serial.printf("Visibility error: %f\n", visibilityError);

        // Calculate average flicker
        float flicker = metrics.getFlicker();
        Serial.printf("Flicker: %f\n", flicker);
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
