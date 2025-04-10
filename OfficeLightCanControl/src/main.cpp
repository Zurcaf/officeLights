#include <0configs.h>

#define MY_NODE_ID 2
#define MAX_NODES 3

unsigned long lastTestMessageTime = 0;
const unsigned long testMessageInterval = 1000; // Send every 1 second
uint8_t testCounter = 0;

float reference;     // Reference value for PID control
float gain;
float gains[MAX_NODES];
float currentMillis; // Current time in milliseconds
bool manualDuty = false; // Flag for manual mode
bool gains_stashed = false;

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

// Create CAN handler instance with your SPI configuration
CANHandler canHandler(spi0, 5, 3, 4, 2, 10000000);

// Serial Interface to comunicate with PC
pcInterface interface(luxMeter, driver, pidController, metrics, canHandler);

// Instantiate the NetworkBoot class.
NetworkBoot networkBoot;

CalibrationManager* calibrator = nullptr;
bool calibration_ready = false;

void setup()
{
    interface.begin(115200); // Start serial communication with PC

    analogReadResolution(12);
    analogWriteFreq(60000);
    analogWriteRange(DAC_RANGE);
    pinMode(LED_PIN, OUTPUT_12MA);

    if (!canHandler.begin(CAN_1000KBPS)) {
        Serial.println("CAN initialization failed!");
        while(1); // halt if CAN initialization fails
    }
    canHandler.setTransmitInterval(1000); // Set transmit interval to 1000ms

    raspConfig(); // Configure the Raspberry Pi based on its unique ID
    networkBoot.begin(); // Start the network boot process
}

void loop()
{

    if (!networkBoot.isBootComplete()){
        calibration_ready = false;
        networkBoot.update(); // keep booting
        return;
    }

    // --- Step 2: Initialize Calibration ---
    if (networkBoot.isBootComplete() && !calibration_ready){
        receive_nodes();  // sets calibration_ready = true
        return;
    }

    // --- Step 3: Run Calibration Once ---
    if (calibrator && !calibrator->isCalibrationComplete()) {
        calibrator->startCalibration();
        return;
    }

    if (calibrator->isCalibrationComplete() && !gains_stashed) {
        const float* receivedGains = calibrator->getAllGains();
        for (int i = 0; i < MAX_NODES; ++i) {
            gains[i] = receivedGains[i];
        }
        gain = calibrator->getOwnGain();
        float offset = calibrator->getOffset();
        driver.setGainOffset(gain, offset);
        pidController.setGainAndExternal(gain, offset); // Set the gain and external illuminance in the controller
        gains_stashed = true;
        return;
    }

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
        dutyCycle = driver.setDutyCycle(dutyCycle);

        // Update internal state (housekeeping) for the PID controller
        pidController.housekeep(measuredLux);

        reference = pidController.getReference(); // Set the duty cycle in the PID controller

        // Insert values into the metrics buffer
        metrics.insertValues(dutyCycle, measuredLux, reference, currentMillis);

        // Check interface for incoming messages
        interface.processSerial();

        float voltage = luxMeter.getLdrVoltage(); // Get the LDR voltage value

        // Stream Serial data to the PC
        interface.streamSerialData(dutyCycle, measuredLux, reference, voltage, currentMillis);

        // Process incoming CAN messages
        interface.processIncomingCANMessages();
    }

    // can_checker(); // Check for incoming CAN messages
}

void can_checker()
{
    if (currentMillis - lastTestMessageTime >= testMessageInterval) {
        lastTestMessageTime = currentMillis;

        // Prepare test message
        uint8_t testData[8] = {0};
        testData[0] = testCounter++;
        testData[1] = interface.getMyId(); // Include our own ID in the message

        bool sent = canHandler.sendMessage(1, 2, testData, sizeof(testData));
        if (sent)
        {
            Serial.printf("Sent test message to desk 2, counter: %d\n", testCounter);
        }
        else
        {
            Serial.println("Failed to send test message");
        }
    }

        // Check for received CAN messages
    uint8_t receivedMessageId, receivedDeskId;
    uint8_t receivedData[8];
    uint8_t receivedLength;

    if (canHandler.readMessage(&receivedMessageId, &receivedDeskId, receivedData, &receivedLength)) {
        Serial.printf("Received message from desk %d, type %d: ", receivedDeskId, receivedMessageId);
        for (int i = 0; i < receivedLength; i++) {
            Serial.printf("%02X ", receivedData[i]);
        }
        Serial.println();

        // For test messages (type 1)
        if (receivedMessageId == 1) {
            uint8_t senderCounter = receivedData[0];
            uint8_t senderId = receivedData[1];
            Serial.printf("Test message from desk %d: counter=%d\n", senderId, senderCounter);
        }
    }
}

void receive_nodes() {
    const uint8_t* discovered_ids = networkBoot.getDiscoveredNodeIDs();
    int count = networkBoot.getNodeCount();

    if (count > 0) {
        if (calibrator != nullptr) delete calibrator; // cleanup if already exists
        calibrator = new CalibrationManager(networkBoot.myNodeId, discovered_ids, count, 2000);
        calibration_ready = true;
        Serial.println("Calibration manager initialized.");
    }
}    

// Seting b and m according to the ID of the device (unique to each LDR sensor)
// The specific ID checking for in byte array form
void raspConfig() 
{
    // ID of the device
    uint8_t id[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // Get the unique ID of the Raspberry Pi
    flash_get_unique_id(id);

    //ID of rapsberry pi A
    uint8_t targetId1[8] = {0xE6, 0x61, 0x18, 0x60, 0x4B, 0x84, 0x3A, 0x21};

    //ID of rapsberry pi B
    uint8_t targetId2[8] = {0xE6, 0x60, 0xC0, 0xD1, 0xC7, 0x6F, 0x22, 0x2F};

    //ID of rapsberry pi C E6:61:18:60:4B:4E:B6:27
    uint8_t targetId3[8] = {0xE6, 0x61, 0x18, 0x60, 0x4B, 0x4E, 0xB6, 0x27};

    // Compare the id byte-by-byte
    if (id != nullptr && memcmp(id, targetId1, 8) == 0) 
    {
        //COM10
        luxMeter.setCalibration(-0.8, 5.976); // Set the calibration coefficients for A if the ID matches

        // AQUI ENTRA O CODIOGO DE boot DOS IDS
        interface.myIdInit(1); // Set the ID for the interface
        networkBoot.myNodeId = 1; // Set the ID for the network boot
        interface.addDeskId(2); // Add the ID to the list of desks
        interface.addDeskId(3); // Add the ID to the list of desks
    }
    else if (id != nullptr && memcmp(id, targetId2, 8) == 0)
    {
        //COM8
        luxMeter.setCalibration(-0.8, 5.976); // Set the calibration coefficients for B if the ID matches

        interface.myIdInit(2); // Set the ID for the interface
        networkBoot.myNodeId = 2; // Set the ID for the network boot
        interface.addDeskId(1); // Add the ID to the list of desks
        interface.addDeskId(3); // Add the ID to the list of desks
    }else if (id != nullptr && memcmp(id, targetId3, 8) == 0)
    {
        // COM24
        luxMeter.setCalibration(-0.8, 5.976); // Set the calibration coefficients for C if the ID matches

        interface.myIdInit(3); // Set the ID for the interface
        networkBoot.myNodeId = 3; // Set the ID for the network boot
        interface.addDeskId(1); // Add the ID to the list of desks
        interface.addDeskId(2); // Add the ID to the list of desks
    }
    else
    {
        // Print the ID of the Raspberry Pi to avoid running the code on the wrong device
        while (true)
        {
            Serial.printf("ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
                          id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
            Serial.println("Unowned Raspberry Pi.");
        }
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
    
    Serial.printf("G: %f, d: %f\n", driver.G, driver.d);
}
