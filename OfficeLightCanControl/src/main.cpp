#include <Arduino.h>
#include <0hardware_config.h>
#include <1luxmeter_manager.h>
#include "hardware/flash.h"
#include <luxmeter.h>



unsigned long previousMillis = 0;
int dutyCycle = 0;


uint8_t id[8];


// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE);

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);


    flash_get_unique_id(id);
    luxMeter.setCalibration(id);
    


}

void loop() {
    // unsigned long currentMillis = millis();

    // if (currentMillis - previousMillis >= interval) {
    //     previousMillis = currentMillis;

    //     analogWrite(LED_PIN, dutyCycle);
    //     dutyCycle += STEP_SIZE;
    //     if (dutyCycle > DAC_RANGE) {
    //         dutyCycle = 0;
    //     }
    // }

    // float lux = luxMeter.readLux();
    // float voltage = luxMeter.readVoltage();
    // float resistance = luxMeter.readResistance();
    // int adcValue = luxMeter.readRawADC();

    // Serial.printf("Millis:%lu, PWM:%.1f, ADC:%d, Voltage:%.2fV, Resistance:%.2fohm, LUX:%.2f\n",
    //               currentMillis, dutyCycle / (float)DAC_RANGE, adcValue, voltage, resistance, lux);
}
