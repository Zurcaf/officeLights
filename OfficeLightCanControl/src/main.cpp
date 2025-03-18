#include <0configs.h>

bool unowned_rasp = true;
float reference = 3.0f; // Reference value for PID control
float currentMillis; // Current time in milliseconds

// ID of the device
uint8_t id[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Time configurations
unsigned long LastUpdate_500Hz = 0;
unsigned long LastUpdate_100Hz = 0;

// Local controller configurations
float h = 0.01; // Sampling period: 10 ms (100 Hz). Reasonable for many systems (e.g., heaters, motors) and manageable on most microcontrollers.
float K = 1.0;  // Proportional gain: Start low to ensure stability, increase if response is too slow.
float b = 1.0;  // Setpoint weight (proportional): Full setpoint contribution, standard choice.
float c = 0.0;  // Setpoint weight (derivative): No setpoint in derivative, avoids amplifying setpoint steps.
float Ti = 2;   // Integral time: 1 second, moderate integral action for steady-state error correction (adjust based on system time constant).
float Td = 0.5; // Derivative time: Start with 0 to avoid noise issues, add later (e.g., 0.1-0.25) if overshoot occurs.
float Tt = 1.0; // Anti-windup time: 2 seconds, faster than 100 s, balances windup recovery with stability.
float N = 10.0; // Derivative filter: Typical value, effective if Td > 0 is added later.

bool integratorOnly = true; // Standard PID mode for general control.
bool bumpLess = true;       // Bump-less mode
bool occupancy = false;     // Occupancy control mode
bool feedback = true;       // Feedback control mode
bool antiWindup = true;     // Anti-windup control mode

// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE, DAC_RANGE);

// Initialize Driver
Driver driver(LED_PIN, DAC_RANGE, STEP_SIZE, interval);

// PID controller instance
localController pidController;

// Data storage metrics instance~
dataStorageMetrics metrics;

// Serial Interface to comunicate with PC
pcInterface interface(1); // Assign this Pico as desk ID 1

void setup()
{
    interface.begin(115200); // Start serial communication with PC
    analogReadResolution(12);
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);

    flash_get_unique_id(id);
    unowned_rasp = luxMeter.setCalibration(id);

    // Print the ID of the Raspberry Pi to avoid running the code on the wrong device
    while (unowned_rasp)
    {
        Serial.printf("ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
                      id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
        Serial.println("Unowned Raspberry Pi.");
    }

    calibrate_Gd();

    Serial.printf("G: %f, d: %f\n", driver.G, driver.d);
}

void loop()
{
    if (currentMillis - LastUpdate_500Hz >= FREQ_500Hz)
    {
        LastUpdate_500Hz = currentMillis;

        // Get current lux value
        luxMeter.updateMovingAverage();
    }

    if (currentMillis - LastUpdate_100Hz >= FREQ_100Hz)
    {
        LastUpdate_100Hz = currentMillis;

        // Get the current lux value
        float measuredLux = luxMeter.getLuxValue();

        // Update PID controller
        float dutyCycle = pidController.compute_control(); // Compute control output based on reference (r) and measured output (y)

        // Set the duty cycle (0-100%) using the Driver
        driver.setDutyCycle(dutyCycle);

        // Update internal state (housekeeping) for the PID controller
        pidController.housekeep(measuredLux); 

         // Insert values into the metrics buffer
        metrics.insertValues(dutyCycle, measuredLux, reference, currentMillis);

        // Debug output
        Serial.printf("Reference: %.1f lux, Measured: %.1f lux, Duty Cycle: %.4f%%\n",
                      reference, measuredLux, dutyCycle);

        // Check interface for incoming messages
        interface.processSerial();
    }
}

void calibrate_Mb()
{
    unsigned long currentMillis = millis();
    float dutyCycle = driver.calibrate_bm(currentMillis);
    luxMeter.calibrate_bm(currentMillis, dutyCycle);
}

void calibrate_Gd()
{
    // // Get the 0% duty cycle lux value
    // driver.setDutyCycle(0);
    // delay(5000);
    // float lux = luxMeter.getLuxValue();
    // // Serial.printf("Lux: %f\n", lux);

    // // Set the duty cycle to 70% and get the lux value
    // driver.setDutyCycle(0.7);
    // delay(5000);
    // float lux_70 = luxMeter.getLuxValue();
    // // Serial.printf("Lux 70: %f\n", lux_70);

    // // Calculate the gain and offset
    // float G = (lux_70 - lux) / 0.7;
    // float d = lux;

    // // Set the gain and offset
    // driver.setGainOffset(G, d);
    // // Serial.printf("G: %f, d: %f\n", G, d);

    delay(1000);
    driver.setGainOffset(4.986404, 0.702457);
}

// SAVED FROM OLD MAIN.CPP FILE for use buffer functions and calculations!!!
// if (current - previousMillis >= 2000)
// {
//     previousMillis = current;
//     // Test Flicker
//     if (uncalibrated)
//     {
//         // insert debug values for testing
//         
//         uncalibrated = false;
//     }else
//     {
//         // insert debug values for testing
//         metrics.insertValues(200, 100, reference, current);
//         uncalibrated = true;
//     }

//     // Get buffer contents
//     float uData[6000], yData[6000];
//     int timestamps[6000];
//     uint16_t count = metrics.getBuffer(uData, yData, timestamps);

//     uint16_t elements = metrics.getBuffer(uData, yData, timestamps);
//     // print the buffer contents
//     for (uint16_t i = 0; i < elements; i++)
//     {
//         Serial.printf("Duty cycle: %f, Lux: %f, Timestamp: %d\n", uData[i], yData[i], timestamps[i]);
//     }
//     // Calculate energy consumption
//     float energy = metrics.getEnergy();
//     Serial.printf("Energy: %f\n", energy);
//     // Calculate average visibility error
//     float visibilityError = metrics.getVisibilityError();
//     Serial.printf("Visibility error: %f\n", visibilityError);
//     // Calculate average flicker
//     float flicker = metrics.getFlicker();
//     Serial.printf("Flicker: %f\n", flicker);
