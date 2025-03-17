#include "dataStorageMetrics.h"
#include <stdlib.h> // For abs()


dataStorageMetrics::dataStorageMetrics() : 
    head(0), 
    count(0), 
    isFull(false),
    energySum(0.0f),
    visibilityError(0.0f),
    flickerSum(0.0f) {
    // Initialize buffers with zeros
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        uBuffer[i] = 0.0f;
        yBuffer[i] = 0.0f;
        rBuffer[i] = 0.0f;
        timestampBuffer[i] = 0;
    }
}

dataStorageMetrics::~dataStorageMetrics() {}

void dataStorageMetrics::insertValues(float dutyCycle, float luxMeasured, float luxReference, int timestamp) {
    // Store values in buffers
    uBuffer[head] = dutyCycle;
    yBuffer[head] = luxMeasured;
    rBuffer[head] = luxReference;
    timestampBuffer[head] = timestamp;

    // Update metrics incrementally
    updateMetrics(dutyCycle, luxMeasured, luxReference, timestamp);

    // Update buffer state
    head = incrementIndex(head);
    if (count < BUFFER_SIZE) {
        count++;
    } else {
        isFull = true;
    }
}

uint16_t dataStorageMetrics::getBuffer(float* dutyCycleOut, float* luxOut, int* timestampsOut) {
    uint16_t elements = isFull ? BUFFER_SIZE : count;
    uint16_t current = isFull ? head : 0;
    
    for (uint16_t i = 0; i < elements; i++) {
        dutyCycleOut[i] = uBuffer[current];
        luxOut[i] = yBuffer[current];
        timestampsOut[i] = timestampBuffer[current];
        current = incrementIndex(current);
    }
    return elements;
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
    return (idx + 1) % BUFFER_SIZE;
}

void dataStorageMetrics::updateMetrics(float dutyCycle, float luxMeasured, float luxReference, int timestamp) {
    // Energy calculation requires previous sample
    if (count > 1) {
        uint16_t prevIdx = (head - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        float timeDiff = (timestamp - timestampBuffer[prevIdx]) / 1000.0f;  // ms to s
        energySum += dutyCycle * timeDiff;                                  // Will be multiplied by LED_MAX_POWER later
    }

    // Visibility error
    float error = luxReference - luxMeasured;
    visibilityError += (error > 0) ? error : 0.0f;

    // Flicker calculation (requires at least 2 previous samples)
    if (count >= 2) {
        uint16_t prev1 = (head - 1 + BUFFER_SIZE) % BUFFER_SIZE;        // Previous sample index (t-1)
        uint16_t prev2 = (head - 2 + BUFFER_SIZE) % BUFFER_SIZE;        // Sample before previous (t-2)
        
        float diff1 = uBuffer[head] - uBuffer[prev1];
        float diff2 = uBuffer[prev1] - uBuffer[prev2];
        
        if ((diff1 * diff2) < 0) {  // Sign change detected
            flickerSum += (abs(diff1) + abs(diff2));
        }
    }
}