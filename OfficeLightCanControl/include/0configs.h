#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>
#include "hardware/flash.h"

#include <luxmeter.h>
#include <driver.h>
#include <localController.h>
#include <pcInterface.h>
#include <dataStorageMetrics.h>
#include <canRoutines.h>

// Pin Definitions
#define LED_PIN 1
#define LDR_PIN A0



constexpr int FREQ_1000Hz = 2; // Sample period in miliseconds
constexpr int FREQ_100Hz = 10; // PID control period in miliseconds

// ADC and DAC configurations
constexpr float Vcc = 3.3;
constexpr float R_fixed = 10000;
constexpr int ADC_RANGE = 4096;
constexpr int DAC_RANGE = 4096;
constexpr int interval = 6000;

constexpr float STEP_SIZE = 0.1;

// Function to run on Core 1: Continuously update moving average
void core1_task();
void calibrate_Mb ();
void calibrate_Gd ();

#endif // HARDWARE_CONFIG_H
