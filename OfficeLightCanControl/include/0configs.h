#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>
#include "hardware/flash.h"
#include <SPI.h>
#include <mcp2515.h>


#include <luxmeter.h>
#include <driver.h>
#include <localController.h>
#include <pcInterface.h>
#include <dataStorageMetrics.h> 
#include "CANHandler.h"
#include "networkboot.h"
#include "calibration_manager.h"

// Pin Definitions
#define LED_PIN 15
#define LDR_PIN A0


// SPI pins for Raspberry Pi Pico
#define SPI_MISO 3
#define SPI_MOSI 4
#define SPI_SCK 2
#define CAN_CS 5


constexpr int FREQ_500Hz = 2; // Sample period in miliseconds
constexpr int FREQ_100Hz = 10; // PID control period in miliseconds

// ADC and DAC configurations
constexpr float Vcc = 3.3;
constexpr float R_fixed = 10000;
constexpr int ADC_RANGE = 4096;
constexpr int DAC_RANGE = 4096;
constexpr int interval = 6000;

constexpr float STEP_SIZE = 0.1;

void raspConfig();
void calibrate_Mb ();
void calibrate_Gd ();

#endif // HARDWARE_CONFIG_H