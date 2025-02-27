#ifndef LUXMETER_MANAGER_H
#define LUXMETER_MANAGER_H

#include <Arduino.h>
#include <luxmeter.h>

// LUX conversion coefficients
constexpr float m = -0.8;
constexpr float b = 5.976;

// Global Variables
extern unsigned long previousMillis;
extern int dutyCycle;
extern LuxMeter luxMeter;

#endif // LUXMETER_MANAGER_H
