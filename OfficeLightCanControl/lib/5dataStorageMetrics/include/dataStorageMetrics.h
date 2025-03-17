#ifndef DATA_STORAGE_METRICS_H
#define DATA_STORAGE_METRICS_H

#include <stdint.h>
#include <cstdlib>


class dataStorageMetrics {
public:
    // Constructor
    explicit dataStorageMetrics();

    // Destructor
    ~dataStorageMetrics();

    // Insert new values into the circular buffers
    void insertValues(float dutyCycle, float luxMeasured, float luxReference, int timestamp);

    // Get buffer contents (returns number of valid elements)
    uint16_t getBuffer(float* dutyCycleOut, float* luxOut, int* timestampsOut);

    // Calculate energy consumption (in Joules)
    float getEnergy();

    // Calculate average visibility error (in LUX)
    float getVisibilityError();

    // Calculate average flicker (in s^-1)
    float getFlicker();

private:
    static const uint16_t STORAGE_BUFFER_SIZE = 6000; // 100 Hz * 60 seconds = 6000 samples
    static const uint16_t SAMPLING_FREQ = 100; // 100 Hz
    static constexpr float LED_MAX_POWER = 0.099f; // Maximum power consumption in Watts Pmax = V_F × I_F = 3,3V × 30mA = 99mW
    
    // Circular buffers for u (duty cycle) and y (lux values)
    float uBuffer[STORAGE_BUFFER_SIZE];  // Duty cycle values (0 to 1)
    float yBuffer[STORAGE_BUFFER_SIZE];  // Measured lux values
    float rBuffer[STORAGE_BUFFER_SIZE];  // Reference lux values
    int timestampBuffer[STORAGE_BUFFER_SIZE]; // Timestamps in milliseconds
    
    uint16_t head;  // Points to next insertion position
    uint16_t count; // Number of valid elements in buffer
    bool isFull;    // Indicates if buffer is full
    
    // Metrics accumulators
    float energySum;        // For energy calculation
    float visibilityError;  // For visibility error calculation
    float flickerSum;      // For flicker calculation

    // Helper method to increment circular buffer index
    uint16_t incrementIndex(uint16_t idx);

    // Update metrics incrementally with each new sample
    void updateMetrics(float dutyCycle, float luxMeasured, float luxReference, int timestamp);
};

#endif // DATA_STORAGE_METRICS_H