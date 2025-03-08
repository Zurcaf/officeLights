#include "pcInterface.h"
#include <stdarg.h>

pcInterface::pcInterface(uint8_t deskId) : myDeskId(deskId), bufferIndex(0) {
    // Initialize single desk state
    desk = {0.0f, 0.0f, 0.0f, 0.0f, false, false, false, 
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 0.0f};
}

void pcInterface::begin(uint32_t baudRate) {
    Serial.begin(baudRate);
}

void pcInterface::processSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            commandBuffer[bufferIndex] = '\0';
            if (bufferIndex > 0) {
                parseCommand(commandBuffer);
            }
            bufferIndex = 0;
        } else if (bufferIndex < BUFFER_SIZE - 1) {
            commandBuffer[bufferIndex++] = c;
        }
    }
}

void pcInterface::parseCommand(const char* cmd) {
    uint8_t deskId = extractDeskId(cmd + (cmd[0] == 'g' ? 2 : 1));
    if (deskId != myDeskId) {  // Check if command is for this Pico's desk ID
        sendResponse("err");   // Ignore commands not matching this desk ID
        return;
    }

    if (cmd[0] == 'g') {
        executeGetCommand(cmd);
    } else if (cmd[0] == 'R') {
        resetSystem();
        sendResponse("ack");
    } else {
        executeSetCommand(cmd);
    }
}

void pcInterface::executeGetCommand(const char* cmd) {
    switch (cmd[2]) {
        case 'u': sendResponse("u %d %.2f", myDeskId, desk.dutyCycle); break;
        case 'r': sendResponse("r %d %.2f", myDeskId, desk.illuminanceRef); break;
        case 'y': sendResponse("y %d %.2f", myDeskId, desk.measuredIlluminance); break;
        case 'v': sendResponse("v %d %.2f", myDeskId, desk.ldrVoltage); break;
        case 'o': sendResponse("o %d %d", myDeskId, desk.occupancy); break;
        case 'a': sendResponse("a %d %d", myDeskId, desk.antiWindup); break;
        case 'f': sendResponse("f %d %d", myDeskId, desk.feedbackControl); break;
        case 'd': sendResponse("d %d %.2f", myDeskId, desk.externalIlluminance); break;
        case 'p': sendResponse("p %d %.2f", myDeskId, desk.powerConsumption); break;
        case 't': sendResponse("t %d %.2f", myDeskId, desk.elapsedTime); break;
        case 'E': sendResponse("E %d %.2f", myDeskId, desk.avgEnergy); break;
        case 'V': sendResponse("V %d %.2f", myDeskId, desk.avgVisibilityError); break;
        case 'F': sendResponse("F %d %.2f", myDeskId, desk.avgFlickerError); break;
        case 'O': sendResponse("O %d %.2f", myDeskId, desk.occupiedLowerBound); break;
        case 'U': sendResponse("U %d %.2f", myDeskId, desk.unoccupiedLowerBound); break;
        case 'L': sendResponse("L %d %.2f", myDeskId, desk.currentLowerBound); break;
        case 'C': sendResponse("C %d %.2f", myDeskId, desk.energyCost); break;
        default: sendResponse("err"); break;
    }
}

void pcInterface::executeSetCommand(const char* cmd) {
    float value = extractValue(cmd + 3);
    switch (cmd[0]) {
        case 'u': desk.dutyCycle = value; sendResponse("ack"); break;
        case 'r': desk.illuminanceRef = value; sendResponse("ack"); break;
        case 'o': desk.occupancy = (value > 0); sendResponse("ack"); break;
        case 'a': desk.antiWindup = (value > 0); sendResponse("ack"); break;
        case 'f': desk.feedbackControl = (value > 0); sendResponse("ack"); break;
        case 'O': desk.occupiedLowerBound = value; sendResponse("ack"); break;
        case 'U': desk.unoccupiedLowerBound = value; sendResponse("ack"); break;
        case 'C': desk.energyCost = value; sendResponse("ack"); break;
        default: sendResponse("err"); break;
    }
}

void pcInterface::sendResponse(const char* format, ...) {
    char buffer[BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, BUFFER_SIZE, format, args);
    va_end(args);
    Serial.println(buffer);
}

uint8_t pcInterface::extractDeskId(const char* cmd) {
    return atoi(cmd);
}

float pcInterface::extractValue(const char* cmd) {
    return atof(cmd);
}

void pcInterface::resetSystem() {
    desk = {0.0f, 0.0f, 0.0f, 0.0f, false, false, false, 
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 0.0f};
}