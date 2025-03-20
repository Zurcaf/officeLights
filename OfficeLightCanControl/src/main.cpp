#include <0configs.h>

bool unowned_rasp = true;
float reference = 3.0f; // Reference value for PID control
float currentMillis; // Current time in milliseconds

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
pcInterface interface(1, pidController, metrics);

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
    float lux_0 = 0.0f; // Initialize lux value
    float lux_1 = 0.0f; // Initialize lux value
    bool luxMeterReady = false; // Flag to indicate if the lux meter is ready

    // Get the 0% duty cycle lux value
    driver.setDutyCycle(0);

    while (!luxMeterReady)
    {
        currentMillis = millis(); // Get the current time in milliseconds

        // Wait for the lux meter to be ready
        if (currentMillis - LastUpdate_500Hz >= FREQ_500Hz)
        {
            LastUpdate_500Hz = currentMillis;

            // Get current lux value
            luxMeter.updateMovingAverage();
        }

        if(currentMillis > 2000)
        {
            lux_0 = luxMeter.getLuxValue();
            Serial.printf("Lux_0: %f\n", lux_0);

            luxMeterReady = true; // Set the flag to true to exit the loop
        }
    }

    // Set the duty cycle to 70% and get the lux value
    driver.setDutyCycle(1);

    luxMeterReady = false; // Reset the flag for the next measurement

    while (!luxMeterReady)
    {
        currentMillis = millis(); // Get the current time in milliseconds

        // Wait for the lux meter to be ready
        if (currentMillis - LastUpdate_500Hz >= FREQ_500Hz)
        {
            LastUpdate_500Hz = currentMillis;

            // Get current lux value
            luxMeter.updateMovingAverage();
        }

        if(currentMillis > 4000)
        {
            lux_1 = luxMeter.getLuxValue();
            Serial.printf("Lux_1: %f\n", lux_1);

            luxMeterReady = true; // Set the flag to true to exit the loop
        }
    }

    // Calculate the gain and offset
    float G = lux_1 - lux_0;
    float d = lux_0;

    // Set the gain and offset
    driver.setGainOffset(G, d);
    pidController.setGainAndExternal(G, d); // Set the gain and external illuminance in the controller

    // Serial.printf("G: %f, d: %f\n", G, d);

    driver.setDutyCycle(0); // Set the duty cycle to 0%


    // delay(1000);
    // driver.setGainOffset(4.986404, 0.702457);
}

// SAVED FROM OLD MAIN.CPP FILE for use buffer functions and calculations!!!

