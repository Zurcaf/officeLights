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

// ID of the device
uint8_t id[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Time configurations
unsigned long LastUpdate_1000Hz = 0;
unsigned long LastUpdate_100Hz = 0;

constexpr int FREQ_1000Hz = 2; // Sample period in miliseconds
constexpr int FREQ_100Hz = 10; // PID control period in miliseconds

// ADC and DAC configurations
constexpr float Vcc = 3.3;
constexpr float R_fixed = 10000;
constexpr int ADC_RANGE = 4096;
constexpr int DAC_RANGE = 4096;
constexpr int interval = 6000;

constexpr float STEP_SIZE = 0.1;

// Local controller configurations
constexpr float h = 0.01;           // Sampling period: 10 ms (100 Hz). Reasonable for many systems (e.g., heaters, motors) and manageable on most microcontrollers.
constexpr float K = 1.0;            // Proportional gain: Start low to ensure stability, increase if response is too slow.
constexpr float b = 1.0;            // Setpoint weight (proportional): Full setpoint contribution, standard choice.
constexpr float c = 0.0;            // Setpoint weight (derivative): No setpoint in derivative, avoids amplifying setpoint steps.
constexpr float Ti = 2;           // Integral time: 1 second, moderate integral action for steady-state error correction (adjust based on system time constant).
constexpr float Td = 0.5;           // Derivative time: Start with 0 to avoid noise issues, add later (e.g., 0.1-0.25) if overshoot occurs.
constexpr float Tt = 1.0;           // Anti-windup time: 2 seconds, faster than 100 s, balances windup recovery with stability.
constexpr bool officeLightsMode = true; // Standard PID mode for general control.
constexpr float N = 10.0;           // Derivative filter: Typical value, effective if Td > 0 is added later.

// Function to run on Core 1: Continuously update moving average
void core1_task();
void calibrate_Mb ();
void calibrate_Gd ();

#endif // HARDWARE_CONFIG_H
