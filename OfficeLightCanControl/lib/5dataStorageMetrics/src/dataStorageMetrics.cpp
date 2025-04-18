#include "dataStorageMetrics.h"
#include <stdlib.h> // For abs()
#include <Arduino.h>

dataStorageMetrics::dataStorageMetrics() : 
    head(0), 
    count(0), 
    isFull(false),
    energySum(0.0f),
    visibilityError(0.0f),
    flickerSum(0.0f) {
    // Initialize buffers with zeros
    for (uint16_t i = 0; i < STORAGE_BUFFER_SIZE; i++) {
        uBuffer[i] = 0.0f;
        yBuffer[i] = 0.0f;
        rBuffer[i] = 0.0f;
        timestampBuffer[i] = 0;
    }
}

dataStorageMetrics::~dataStorageMetrics() {}

void dataStorageMetrics::insertValues(float dutyCycle, float luxMeasured, float luxReference, int timestamp) {
    // Update buffer count
    if (count < STORAGE_BUFFER_SIZE) {
        count++;
    } else {
        isFull = true;
    }
    
    // Store values in buffers
    uBuffer[head] = dutyCycle;
    yBuffer[head] = luxMeasured;
    rBuffer[head] = luxReference;
    timestampBuffer[head] = timestamp;

    // Update metrics incrementally
    updateMetrics(dutyCycle, luxMeasured, luxReference, timestamp);

    // Update head index
    head = incrementIndex(head);
}

uint16_t dataStorageMetrics::getBuffer(float* dutyCycleOut, float* luxOut, int* timestampsOut) {
    uint16_t elements = isFull ? STORAGE_BUFFER_SIZE : count;
    uint16_t current = isFull ? head : 0;
    
    for (uint16_t i = 0; i < elements; i++) {
        dutyCycleOut[i] = uBuffer[current];
        luxOut[i] = yBuffer[current];
        timestampsOut[i] = timestampBuffer[current];
        current = incrementIndex(current);
    }
    return elements;
}

uint16_t dataStorageMetrics::getUbuffer(float* uOut) {
    uint16_t elements = isFull ? STORAGE_BUFFER_SIZE : count;
    uint16_t current = isFull ? head : 0;
    
    for (uint16_t i = 0; i < elements; i++) {
        uOut[i] = uBuffer[current];
        current = incrementIndex(current);
    }
    return elements;
}

uint16_t dataStorageMetrics::getYbuffer(float* yOut) {
    uint16_t elements = isFull ? STORAGE_BUFFER_SIZE : count;
    uint16_t current = isFull ? head : 0;
    
    for (uint16_t i = 0; i < elements; i++) {
        yOut[i] = yBuffer[current];
        current = incrementIndex(current);
    }
    return elements;
}

float dataStorageMetrics::getPowerConsumption() 
{
    float instantPower = 0.0f;
    instantPower = uBuffer[head] * LED_MAX_POWER; // Convert duty cycle to percentage
    Serial.printf("Duty cycle: %.2f, Instant power: %.2f\n", uBuffer[head], instantPower);
    return instantPower;
}
    

float dataStorageMetrics::getEnergy() {
    return energySum * LED_MAX_POWER;
}

float dataStorageMetrics::getVisibilityError() {
    if (count == 0) return 0.0f;
    return visibilityError / count;
}

float dataStorageMetrics::getFlicker() {
    if (count < 2) return 0.0f; // Flicker calculation requires at least 2 samples
    return flickerSum / count;
}

uint16_t dataStorageMetrics::incrementIndex(uint16_t idx) {
    return (idx + 1) % STORAGE_BUFFER_SIZE;
}

void dataStorageMetrics::updateMetrics(float dutyCycle, float luxMeasured, float luxReference, int timestamp) {
    // Energy calculation requires previous sample
    if (count > 1) {
        uint16_t prevIdx = (head - 1 + STORAGE_BUFFER_SIZE) % STORAGE_BUFFER_SIZE;
        float timeDiff = (timestamp - timestampBuffer[prevIdx]) / 1000.0f;  // ms to s
        energySum += uBuffer[prevIdx] * timeDiff;                                  // Will be multiplied by LED_MAX_POWER later
    }

    // Visibility error
    float error = luxReference - luxMeasured;
    visibilityError += (error > 0) ? error : 0.0f;

    // Flicker calculation (requires at least 2 previous samples)
    if (count >= 2) {
        uint16_t prev1 = (head - 1 + STORAGE_BUFFER_SIZE) % STORAGE_BUFFER_SIZE;        // Previous sample index (t-1)
        uint16_t prev2 = (head - 2 + STORAGE_BUFFER_SIZE) % STORAGE_BUFFER_SIZE;        // Sample before previous (t-2)

        // Serial.printf("Flicker: prev1 = %d, prev2 = %d\n", prev1, prev2);

        if (luxReference != rBuffer[prev1])
        {
            // Serial.println("Flicker calculation error: Reference lux changed.");
            return;  // Skip if reference has lux changed
        }
        if (luxReference != rBuffer[prev2]) 
        {
            // Serial.println("Flicker calculation error: Reference lux changed.");
            return;  // Skip if reference has lux changed
        }

        float diff1 = uBuffer[head] - uBuffer[prev1];
        float diff2 = uBuffer[prev1] - uBuffer[prev2];

        // Serial.printf("Flicker: diff1 = %f, diff2 = %f\n", diff1, diff2);
        
        if ((diff1 * diff2) < 0) {  // Sign change detected
            // Serial.println("Flicker: Sign change detected.");
            flickerSum += (abs(diff1) + abs(diff2));
        }
    }
}