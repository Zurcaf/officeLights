#include <Arduino.h>
#include "hardware/flash.h"
#include <0configs.h>
#include <luxmeter.h>
#include <driver.h>

// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE, DAC_RANGE);

Driver driver(LED_PIN, DAC_RANGE, STEP_SIZE, interval);

bool unowned_rasp = true;
bool uncalibrated = true;

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);

    flash_get_unique_id(id);
    unowned_rasp = luxMeter.setCalibration(id);

    // Print the ID of the Raspberry Pi to avoid running the code on the wrong device
    while (unowned_rasp) {
        Serial.printf("ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
                      id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
        Serial.println("Unowned Raspberry Pi.");
        delay(5000);
    }

    // Launch moving average task on Core 0
    multicore_launch_core1(core1_task);
}

void loop()
{
    calibrate_Gd();

    Serial.printf("G: %f, d: %f\n", driver.G, driver.d);
    delay(5000);

}

// Function to run on Core 0: Continuously update moving average
void core1_task() {
    while (true) {
        unsigned long currentMillis = millis();
        luxMeter.updateMovingAverage(currentMillis);
        delay(1);  // Small delay to prevent core from hogging resources
    }
}

void calibrate_Mb ()
{
    unsigned long currentMillis = millis();
    float dutyCycle = driver.calibrate_bm(currentMillis);
    luxMeter.calibrate_bm(currentMillis, dutyCycle);
}

void calibrate_Gd()
{
    if (uncalibrated)
    {
        // Get the 0% duty cycle lux value
        driver.setDutyCycle(0);
        delay(5000);
        float lux = luxMeter.getLuxValue();
        // Serial.printf("Lux: %f\n", lux);

        // Set the duty cycle to 70% and get the lux value
        driver.setDutyCycle(0.7);
        delay(5000);
        float lux_70 = luxMeter.getLuxValue();
        // Serial.printf("Lux 70: %f\n", lux_70);

        // Calculate the gain and offset
        float G = (lux_70 - lux) / 0.7;
        float d = lux;

        // Set the gain and offset
        driver.setGainOffset(G, d);
        // Serial.printf("G: %f, d: %f\n", G, d);
    }
}




// // Example usage
// int main() {
//     // Create PID controller with initial gains
//     // kp=0.1, ki=0.01, kd=0.05, kf=0.5
//     LuminairePID controller(0.1, 0.01, 0.05, 0.5);
    
//     // Set desired illuminance to 500 lux
//     controller.setTargetIlluminance(500.0);
    
//     // Simulate control loop
//     double currentIlluminance = 0.0;
//     for (int i = 0; i < 100; i++) {
//         // Get control signal (0-100%)
//         double controlSignal = controller.update(currentIlluminance);
        
//         // Simulate luminaire response
//         currentIlluminance = controller.simulateLuminaire(controlSignal);
        
//         // Print results
//         printf("Step %d: Setpoint=%.1f, Current=%.1f, Control=%.1f%%\n",
//                i, controller.getSetPoint(), currentIlluminance, controlSignal);
//     }
    
//     return 0;
// }