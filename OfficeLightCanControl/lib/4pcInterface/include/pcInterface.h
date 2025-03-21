#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include <Arduino.h>
#include <stdarg.h>
#include <string>    // For std::string
#include <sstream>   // For std::stringstream
#include <vector>    // For std::vector

#include <luxmeter.h> // Include luxmeter header
#include <driver.h>     // Include driver header
#include <localController.h>  // Include localController header
#include <dataStorageMetrics.h> // Include 

#define BUFFER_SIZE 64

class pcInterface {
public:
    pcInterface(uint8_t deskId, LuxMeter& luxM, Driver& driv, localController& ctrl, dataStorageMetrics& storage); // Constructor
    void begin(uint32_t baudRate);
    void processSerial();
    void streamSerialData(float u, float y, float r, unsigned long time);


private:

    int myDeskId;                   // The desk ID assigned to this Pico
    int numDesks;                   // Number of desks in the system
    std::vector<int> listIds;       // List of all desk IDs in the system

    LuxMeter& luxMeter;           // Lux meter instance
    Driver& driver;             // Store reference to existing driver
    localController& controller;  // Store reference to existing localController    
    dataStorageMetrics& dataSt; // Store reference to existing dataStorageMetrics

    char commandBuffer[BUFFER_SIZE];
    uint8_t bufferIndex;

    bool streaming_y = false; // Flag for streaming y data
    bool streaming_u = false; // Flag for streaming u data
    bool streaming_r = false; // Flag for streaming r data

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