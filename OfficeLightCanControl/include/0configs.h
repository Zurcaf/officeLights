#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>

// Pin Definitions
#define LED_PIN 1
#define LDR_PIN A0

// ID of the device
uint8_t id[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Time configurations
unsigned long previousMillis = 0;

// ADC and DAC configurations
constexpr float Vcc = 3.3;
constexpr float R_fixed = 10000;
constexpr int ADC_RANGE = 4096;
constexpr int DAC_RANGE = 4096;
constexpr int STEP_SIZE = DAC_RANGE / 10;
constexpr int interval = 6000;

// Function to run on Core 1: Continuously update moving average
void core1_task();

#endif // HARDWARE_CONFIG_H
