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
constexpr int interval = 6000;
constexpr float PID_PERIOD = 10; // PID control period in miliseconds
constexpr float STEP_SIZE = 0.1;

// Local controller configurations
constexpr float h = 0.01;
constexpr float K = 10;
constexpr float b = 1;
constexpr float c = 0;
constexpr float Ti = 0.000001;
constexpr float Td = 0;
constexpr float Tt = 100;
constexpr bool officeLightsMode = false;
constexpr float N = 10;


// Function to run on Core 1: Continuously update moving average
void core1_task();
void calibrate_Mb ();
void calibrate_Gd ();

#endif // HARDWARE_CONFIG_H
