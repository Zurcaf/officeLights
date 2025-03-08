#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include <Arduino.h>

#define BUFFER_SIZE 64

class pcInterface {
public:
    pcInterface(uint8_t deskId);  // Constructor takes desk ID
    void begin(uint32_t baudRate);
    void processSerial();

private:
    // Desk state structure for a single desk
    struct DeskState {
        float dutyCycle;           // u
        float illuminanceRef;      // r
        float measuredIlluminance; // y
        float ldrVoltage;          // v
        bool occupancy;            // o
        bool antiWindup;          // a
        bool feedbackControl;     // f
        float externalIlluminance; // d
        float powerConsumption;    // p
        float elapsedTime;         // t
        float avgEnergy;          // E
        float avgVisibilityError; // V
        float avgFlickerError;    // F
        float occupiedLowerBound; // O
        float unoccupiedLowerBound;// U
        float currentLowerBound;  // L
        float energyCost;         // C
    };

    uint8_t myDeskId;  // The desk ID assigned to this Pico
    DeskState desk;    // Single desk state
    char commandBuffer[BUFFER_SIZE];
    uint8_t bufferIndex;

    // Command parsing and execution
    void parseCommand(const char* cmd);
    void executeGetCommand(const char* cmd);
    void executeSetCommand(const char* cmd);
    void sendResponse(const char* format, ...);
    
    // Helper functions
    uint8_t extractDeskId(const char* cmd);
    float extractValue(const char* cmd);
    void resetSystem();
};

#endif