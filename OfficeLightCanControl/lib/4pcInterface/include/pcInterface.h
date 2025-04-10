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
#include <CANHandler.h> // Include CANHandler header

#define BUFFER_SIZE 64

// Command message IDs
enum MessageType
{
    MSG_RESET = 1,

    // Get commands
    MSG_GET_DUTY_CYCLE,             // g u
    MSG_GET_REFERENCE,              // g r
    MSG_GET_ILLUMINANCE,            // g y
    MSG_GET_VOLTAGE,                // g v
    MSG_GET_OCCUPANCY,              // g o
    MSG_GET_ANTI_WINDUP,            // g a
    MSG_GET_FEEDBACK,               // g f
    MSG_GET_EXTERNAL,               // g d
    MSG_GET_POWER,                  // g p
    MSG_GET_TIME,                   // g t
    MSG_GET_ENERGY,                 // g E
    MSG_GET_VISIBILITY_ERROR,       // g V
    MSG_GET_FLICKER,                // g F
    MSG_GET_LOWER_BOUND_OCCUPIED,   // g O
    MSG_GET_LOWER_BOUND_UNOCCUPIED, // g U
    MSG_GET_CURRENT_LOWER_BOUND,    // g L
    MSG_GET_ENERGY_COST,            // g C

    // Buffer get commands
    MSG_GET_BUFFER_U, // g b u
    MSG_GET_BUFFER_Y, // g b y

    // Set commands
    MSG_SET_DUTY_CYCLE,             // u
    MSG_SET_REFERENCE,              // r
    MSG_SET_OCCUPANCY,              // o
    MSG_SET_ANTI_WINDUP,            // a
    MSG_SET_FEEDBACK,               // f
    MSG_SET_LOWER_BOUND_OCCUPIED,   // O
    MSG_SET_LOWER_BOUND_UNOCCUPIED, // U
    MSG_SET_ENERGY_COST,            // C

    // Stream commands
    MSG_STREAM_START_U,   // s u
    MSG_STREAM_START_Y,   // s y
    MSG_STREAM_START_R,   // s r
    MSG_STREAM_START_V,   // s v
    MSG_STREAM_START_ALL, // s all

    MSG_STREAM_STOP_U,   // S u
    MSG_STREAM_STOP_Y,   // S y
    MSG_STREAM_STOP_R,   // S r
    MSG_STREAM_STOP_V,   // S v
    MSG_STREAM_STOP_ALL, // S all

    // Response types
    MSG_ACK,
    MSG_ERROR
};

class pcInterface {
public:
    pcInterface(LuxMeter &luxM, Driver &driv, localController &ctrl,
                dataStorageMetrics &storage, CANHandler &canHandler);

    void begin(uint32_t baudRate);
    void processSerial();
    void streamSerialData(float u, float y, float r, float v, unsigned long time);

    void processIncomingCANMessages();

    // ID management
    int myDeskId;
    void myIdInit(int id);
    void addDeskId(int id);
    int getMyId() const { return myDeskId; } // Getter for myDeskId

private:
    int numDesks;
    std::vector<int> listIds;

    LuxMeter& luxMeter;
    Driver& driver;
    localController& controller;    
    dataStorageMetrics& dataSt;
    CANHandler& canHandler;

    char commandBuffer[BUFFER_SIZE];
    uint8_t bufferIndex;

    bool streaming_y = false;
    bool streaming_u = false;
    bool streaming_r = false;
    bool streaming_v = false;

    void parseCommand(const char* cmd);

    void handleCommand(MessageType msgType, std::vector<std::string> tokens);
    void sendResponse(MessageType msgType, const char* format, ...);

    void handleRemoteCommand(MessageType msgType, uint8_t targetDeskId, std::vector<std::string> tokens);
    void sendDataResponse(MessageType msgType, int deskId, float value);
    void sendDataResponse(MessageType msgType, int deskId, int value);
    

    bool sendCanCommand(MessageType msgType, uint8_t targetDeskId, float value = 0.0f, int intValue = 0);
    bool waitForCanResponse(uint8_t expectedDeskId);


    bool isNotValidID(int id);
    uint8_t extractDeskId(const char* cmd);
    float extractValue(const char* cmd);
    void resetSystem();

};

#endif