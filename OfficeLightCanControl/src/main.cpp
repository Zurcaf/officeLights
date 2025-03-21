#include <0configs.h>

bool unowned_rasp = true;
float reference;     // Reference value for PID control
float currentMillis; // Current time in milliseconds
bool manualDuty = false; // Flag for manual mode

// ID of the device
uint8_t id[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Time configurations
unsigned long LastUpdate_500Hz = 0;
unsigned long LastUpdate_100Hz = 0;

// Initialize LuxMeter
LuxMeter luxMeter(LDR_PIN, Vcc, R_fixed, ADC_RANGE, DAC_RANGE);

// Initialize Driver
Driver driver(LED_PIN, DAC_RANGE, STEP_SIZE, interval);

// PID controller instance
localController pidController;

// Data storage metrics instance~
dataStorageMetrics metrics;

// Serial Interface to comunicate with PC
pcInterface interface(1, luxMeter, driver, pidController, metrics);

void setup()
{
    interface.begin(115200); // Start serial communication with PC
    analogReadResolution(12);
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);

    pinMode(LED_PIN, OUTPUT_12MA);

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
    currentMillis = millis(); // Get the current time in milliseconds

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

        reference = pidController.getReference(); // Set the duty cycle in the PID controller

        // Insert values into the metrics buffer
        metrics.insertValues(dutyCycle, measuredLux, reference, currentMillis);

        // Check interface for incoming messages
        interface.processSerial();

        // Stream Serial data to the PC
        interface.streamSerialData(dutyCycle, measuredLux, reference, currentMillis);
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
//     float lux_0 = 0.0f; // Initialize lux value
//     float lux_1 = 0.0f; // Initialize lux value
//     float G = 0.0f;     // Gain value
//     float d = 0.0f;     // Offset value

//     bool calibrated = false; // Flag to indicate if the lux meter is ready

//     bool duty0calibrated = false; // Flag to indicate if the duty cycle is set to 0%
//     bool duty1calibrated = false; // Flag to indicate if the duty cycle is set to 100%

//     int prevCalibrationTryCounter = 0; // Previous calibration attempt counter
//     int calibrationTryCounter = 0; // Counter for calibration attempts
//     int calibrationTryMax = 5;     // Maximum number of calibration attempts

//     int startTime = 0; // Start time for calibration

//     int calibrateInterval = 2500; // Calibration time in milliseconds

//     // Get the 0% duty cycle lux value
//     driver.setDutyCycle(0);

//     while (!calibrated)
//     {
//         currentMillis = millis(); // Get the current time in milliseconds

//         // Wait for the lux meter to be ready
//         if (currentMillis - LastUpdate_500Hz >= FREQ_500Hz)
//         {
//             LastUpdate_500Hz = currentMillis;

//             // Get current lux value
//             luxMeter.updateMovingAverage();
//         }

//         if (currentMillis >= (3*calibrateInterval*calibrationTryCounter) + calibrateInterval && !duty0calibrated)
//         {
//             // Get the lux value at 0% duty cycle
//             lux_0 = luxMeter.getLuxValue();
//             Serial.printf("Lux_0: %f\n", lux_0);

//             // Set the duty cycle to 100%
//             driver.setDutyCycle(1);

//             duty0calibrated = true; // Set the flag to true to exit the loop
//         }

//         if (currentMillis > (3*calibrateInterval*calibrationTryCounter) + 2 * calibrateInterval && !duty1calibrated)
//         {
//             // Get the lux value at 100% duty cycle
//             lux_1 = luxMeter.getLuxValue();
//             Serial.printf("Lux_1: %f\n", lux_1);

//             // Calculate the gain and offset
//             G = lux_1 - lux_0;
//             d = lux_0;

//             driver.setDutyCycle(0); // Set the duty cycle to 0%

//             if (G <= 0.0f)
//             {
//                 calibrationTryCounter++; // Increment the calibration attempt counter
//                 Serial.println("Calibration failed. Retrying...");

//                 duty0calibrated = false; // Reset the flag for the next attempt
//                 duty1calibrated = false; // Reset the flag for the next attempt
//             }
//             else
//             {
//                 duty1calibrated = true; // Set the flag to true to exit the loop
//             }
//         }

//         if (currentMillis >= (3 * calibrateInterval * calibrationTryCounter) + 3 * calibrateInterval)
//         {
//             if (duty1calibrated)
//             {
//                 // Set the gain and offset
//                 driver.setGainOffset(G, d);
//                 pidController.setGainAndExternal(G, d); // Set the gain and external illuminance in the controller

//                 calibrated = true; // Set the flag to true to exit the loop
//             }
//         }
//     }

    // Overwrite the gain and offset in the driver for debugging purposes (real values already precalibrated to not waist time)
    driver.setGainOffset(17.970798, 2.081886);
    pidController.setGainAndExternal(17.970798, 2.081886); // Set the gain and external illuminance in the controller
}
