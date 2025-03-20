#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include <Arduino.h>
#include <stdarg.h>
#include <string>    // For std::string
#include <sstream>   // For std::stringstream
#include <vector>    // For std::vector
#include <localController.h>  // Include localController header
#include <dataStorageMetrics.h> // Include 

#define BUFFER_SIZE 64

class pcInterface {
public:
    pcInterface(uint8_t deskId, localController& ctrl, dataStorageMetrics& storage); // Constructor
    void begin(uint32_t baudRate);
    void processSerial();

private:
    // Desk state structure for a single desk
    struct DeskState {
        bool streaming_u;           // streaming duty cycle
        bool streaming_y;           // streaming illuminance

        float dutyCycle;            // u (duty cycle)
        float illuminanceRef;       // r (reference illuminance)

        // meeasurements
        float measuredIlluminance;  // y (measured illuminance)
        float ldrVoltage;           // v (LDR voltage)

        bool occupancy;             // o (occupancy)
        bool antiWindup;            // a (anti-windup)
        bool feedbackControl;       // f (feedback control)
        
        float externalIlluminance;  // d (external illuminance) - constant

        // metrics
        float powerConsumption;     // p (power consumption)
        float elapsedTime;          // t (elapsed time)
        float avgEnergy;            // E (average energy)
        float avgVisibilityError;   // V (average visibility error)
        float avgFlickerError;      // F (average flicker error)

        // Second part of lab
        float occupiedLowerBound;   // O (occupied lower bound)
        float unoccupiedLowerBound; // U (unoccupied lower bound)
        float currentLowerBound;    // L (current lower bound)
        float energyCost;           // C (energy cost)
    };

    int myDeskId;                   // The desk ID assigned to this Pico
    int numDesks;                   // Number of desks in the system
    std::vector<int> listIds;       // List of all desk IDs in the system

    DeskState desk;                 // Single desk state
    localController& controller;  // Store reference to existing localController    
    dataStorageMetrics& dataSt; // Store reference to existing dataStorageMetrics

    char commandBuffer[BUFFER_SIZE];
    uint8_t bufferIndex;

    // Command parsing and execution
    void parseCommand(const char* cmd);
    bool isNotValidID(int id);
    void executeGetCommand(std::vector<std::string> tokens);
    void executeStreamCommand(std::vector<std::string> tokens);
    void executeSetCommand(std::vector<std::string> tokens);
    
    // Helper functions
    void sendResponse(const char* format, ...);
    uint8_t extractDeskId(const char* cmd);
    float extractValue(const char* cmd);
    void resetSystem();
};

#endif