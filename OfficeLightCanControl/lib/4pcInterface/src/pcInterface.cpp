#include "pcInterface.h"

pcInterface::pcInterface(LuxMeter &luxM, Driver &driv,
                         localController &ctrl, dataStorageMetrics &storage,
                         CANHandler &canHandler)
    : luxMeter(luxM), driver(driv),
      controller(ctrl), dataSt(storage), canHandler(canHandler)
{
}

void pcInterface::begin(uint32_t baudRate)
{
    Serial.begin(baudRate);
}

void pcInterface::processSerial()
{
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            commandBuffer[bufferIndex] = '\0';
            if (bufferIndex > 0)
            {
                parseCommand(commandBuffer);
                memset(commandBuffer, 0, BUFFER_SIZE);
            }
            bufferIndex = 0;
        }
        else if (bufferIndex < BUFFER_SIZE - 1)
        {
            commandBuffer[bufferIndex++] = c;
        }
    }
}

void pcInterface::parseCommand(const char *cmd)
{
    std::string command(cmd);
    std::stringstream ss(command);
    std::string token;
    std::vector<std::string> tokens;

    while (tokens.size() < 5 && std::getline(ss, token, ' '))
    {
        tokens.push_back(token);
    }

    if (tokens.empty())
    {
        sendResponse(MSG_ERROR, "empty command");
        return;
    }

    // Determine message type
    MessageType msgType = MSG_ERROR;
    uint8_t targetDeskId = myDeskId; // Default to this desk

    if (tokens[0] == "R")
    {
        msgType = MSG_RESET;
    }
    else if (tokens[0] == "g")
    {
        if (tokens.size() < 2)
        {
            sendResponse(MSG_ERROR, "invalid get command");
            return;
        }

        // Extract target desk ID (last token for get commands)
        targetDeskId = extractDeskId(tokens.back().c_str());

        if (tokens[1] == "u")
            msgType = MSG_GET_DUTY_CYCLE;
        else if (tokens[1] == "r")
            msgType = MSG_GET_REFERENCE;
        else if (tokens[1] == "y")
            msgType = MSG_GET_ILLUMINANCE;
        else if (tokens[1] == "v")
            msgType = MSG_GET_VOLTAGE;
        else if (tokens[1] == "o")
            msgType = MSG_GET_OCCUPANCY;
        else if (tokens[1] == "a")
            msgType = MSG_GET_ANTI_WINDUP;
        else if (tokens[1] == "f")
            msgType = MSG_GET_FEEDBACK;
        else if (tokens[1] == "d")
            msgType = MSG_GET_EXTERNAL;
        else if (tokens[1] == "p")
            msgType = MSG_GET_POWER;
        else if (tokens[1] == "t")
            msgType = MSG_GET_TIME;
        else if (tokens[1] == "E")
            msgType = MSG_GET_ENERGY;
        else if (tokens[1] == "V")
            msgType = MSG_GET_VISIBILITY_ERROR;
        else if (tokens[1] == "F")
            msgType = MSG_GET_FLICKER;
        else if (tokens[1] == "O")
            msgType = MSG_GET_LOWER_BOUND_OCCUPIED;
        else if (tokens[1] == "U")
            msgType = MSG_GET_LOWER_BOUND_UNOCCUPIED;
        else if (tokens[1] == "L")
            msgType = MSG_GET_CURRENT_LOWER_BOUND;
        else if (tokens[1] == "C")
            msgType = MSG_GET_ENERGY_COST;
        else if (tokens[1] == "b")
        {
            if (tokens.size() < 3)
            {
                sendResponse(MSG_ERROR, "invalid get buffer command");
                return;
            }
            if (tokens[2] == "u")
                msgType = MSG_GET_BUFFER_U;
            else if (tokens[2] == "y")
                msgType = MSG_GET_BUFFER_Y;
        }
    }
    else if (tokens[0] == "s")
    {
        if (tokens.size() < 2)
        {
            sendResponse(MSG_ERROR, "invalid stream start command");
            return;
        }
        if (tokens[1] == "u")
            msgType = MSG_STREAM_START_U;
        else if (tokens[1] == "y")
            msgType = MSG_STREAM_START_Y;
        else if (tokens[1] == "r")
            msgType = MSG_STREAM_START_R;
        else if (tokens[1] == "v")
            msgType = MSG_STREAM_START_V;
        else if (tokens[1] == "all")
            msgType = MSG_STREAM_START_ALL;

        // Extract target desk ID (last token for start stream commands)
        targetDeskId = extractDeskId(tokens.back().c_str());
    }
    else if (tokens[0] == "S")
    {
        if (tokens.size() < 2)
        {
            sendResponse(MSG_ERROR, "invalid stream stop command");
            return;
        }
        if (tokens[1] == "u")
            msgType = MSG_STREAM_STOP_U;
        else if (tokens[1] == "y")
            msgType = MSG_STREAM_STOP_Y;
        else if (tokens[1] == "r")
            msgType = MSG_STREAM_STOP_R;
        else if (tokens[1] == "v")
            msgType = MSG_STREAM_STOP_V;
        else if (tokens[1] == "all")
            msgType = MSG_STREAM_STOP_ALL;

        // Extract target desk ID (last token for get commands)
        targetDeskId = extractDeskId(tokens.back().c_str());
    }
    else if (tokens[0] == "u")
    {
        msgType = MSG_SET_DUTY_CYCLE;
        targetDeskId = extractDeskId(tokens[1].c_str());
    }
    else if (tokens[0] == "r")
    {
        msgType = MSG_SET_REFERENCE;
        targetDeskId = extractDeskId(tokens[1].c_str());
    }
    else if (tokens[0] == "o")
    {
        msgType = MSG_SET_OCCUPANCY;
        targetDeskId = extractDeskId(tokens[1].c_str());
    }
    else if (tokens[0] == "a")
    {
        msgType = MSG_SET_ANTI_WINDUP;
        targetDeskId = extractDeskId(tokens[1].c_str());
    }
    else if (tokens[0] == "f")
    {
        msgType = MSG_SET_FEEDBACK;
        targetDeskId = extractDeskId(tokens[1].c_str());
    }
    else if (tokens[0] == "O")
    {
        msgType = MSG_SET_LOWER_BOUND_OCCUPIED;
        targetDeskId = extractDeskId(tokens[1].c_str());
    }
    else if (tokens[0] == "U")
    {
        msgType = MSG_SET_LOWER_BOUND_UNOCCUPIED;
        targetDeskId = extractDeskId(tokens[1].c_str());
    }
    else if (tokens[0] == "C")
    {
        msgType = MSG_SET_ENERGY_COST;
        targetDeskId = extractDeskId(tokens[1].c_str());
    }
    if (msgType == MSG_ERROR)
    {
        sendResponse(MSG_ERROR, "unknown command %s", tokens[0].c_str());
        return;
    }

    // Check if the target desk ID is valid
    if (isNotValidID(targetDeskId))
    {
        sendResponse(MSG_ERROR, "invalid desk ID %d", targetDeskId);
        return;
    }

    // Check if the target desk ID is the same as this desk ID
    if (targetDeskId != myDeskId)
    {
        handleRemoteCommand(msgType, targetDeskId, tokens);
        return;
    }

    handleCommand(msgType, tokens);
}

void pcInterface::handleRemoteCommand(MessageType msgType, uint8_t targetDeskId, std::vector<std::string> tokens)
{
    switch (msgType)
    {
    // Get commands
    case MSG_GET_DUTY_CYCLE:
    case MSG_GET_REFERENCE:
    case MSG_GET_ILLUMINANCE:
    case MSG_GET_VOLTAGE:
    case MSG_GET_OCCUPANCY:
    case MSG_GET_ANTI_WINDUP:
    case MSG_GET_FEEDBACK:
    case MSG_GET_EXTERNAL:
    case MSG_GET_POWER:
    case MSG_GET_TIME:
    case MSG_GET_ENERGY:
    case MSG_GET_VISIBILITY_ERROR:
    case MSG_GET_FLICKER:
    case MSG_GET_LOWER_BOUND_OCCUPIED:
    case MSG_GET_LOWER_BOUND_UNOCCUPIED:
    case MSG_GET_CURRENT_LOWER_BOUND:
    case MSG_GET_ENERGY_COST:
    {
        if (sendCanCommand(msgType, targetDeskId))
        {
            float floatValue;
            int intValue;

            if (waitForCanResponse(msgType, targetDeskId, &floatValue, &intValue))
            {
                // Determine if the response should be float or int
                bool isFloatResponse = (msgType != MSG_GET_OCCUPANCY &&
                                        msgType != MSG_GET_ANTI_WINDUP &&
                                        msgType != MSG_GET_FEEDBACK);

                if (isFloatResponse)
                {
                    sendDataResponse(msgType, targetDeskId, floatValue);
                }
                else
                {
                    sendDataResponse(msgType, targetDeskId, intValue);
                }
            }
            else
            {
                sendResponse(MSG_ERROR, "no response from desk %d", targetDeskId);
            }
        }
        else
        {
            sendResponse(MSG_ERROR, "failed to send command to desk %d", targetDeskId);
        }
        break;
    }

    // Set commands
    case MSG_SET_DUTY_CYCLE:
    case MSG_SET_REFERENCE:
    case MSG_SET_OCCUPANCY:
    case MSG_SET_ANTI_WINDUP:
    case MSG_SET_FEEDBACK:
    case MSG_SET_LOWER_BOUND_OCCUPIED:
    case MSG_SET_LOWER_BOUND_UNOCCUPIED:
    case MSG_SET_ENERGY_COST:
    {
        if (tokens.size() < 3)
        {
            sendResponse(MSG_ERROR, "invalid set command");
            return;
        }

        float floatValue = 0.0f;
        int intValue = 0;
        bool isFloatCommand = (msgType != MSG_SET_OCCUPANCY &&
                               msgType != MSG_SET_ANTI_WINDUP &&
                               msgType != MSG_SET_FEEDBACK);

        if (isFloatCommand)
        {
            floatValue = extractValue(tokens[2].c_str());
        }
        else
        {
            intValue = atoi(tokens[2].c_str());
        }

        if (sendCanCommand(msgType, targetDeskId, floatValue, intValue))
        {
            if (waitForCanResponse(MSG_ACK, targetDeskId))
            {
                sendResponse(MSG_ACK, "");
            }
            else
            {
                sendResponse(MSG_ERROR, "no ack from desk %d", targetDeskId);
            }
        }
        else
        {
            sendResponse(MSG_ERROR, "failed to send command to desk %d", targetDeskId);
        }
        break;
    }

    // Stream commands (not supported for remote desks)
    case MSG_STREAM_START_U:
    case MSG_STREAM_START_Y:
    case MSG_STREAM_START_R:
    case MSG_STREAM_START_V:
    case MSG_STREAM_START_ALL:
    case MSG_STREAM_STOP_U:
    case MSG_STREAM_STOP_Y:
    case MSG_STREAM_STOP_R:
    case MSG_STREAM_STOP_V:
    case MSG_STREAM_STOP_ALL:
        sendResponse(MSG_ERROR, "streaming not supported for remote desks");
        break;

    default:
        sendResponse(MSG_ERROR, "remote command not supported");
        break;
    }
}

void pcInterface::handleCommand(MessageType msgType, std::vector<std::string> tokens)
{
    switch (msgType)
    {
    case MSG_RESET:
    {
        resetSystem();
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_GET_DUTY_CYCLE:
    {
        sendDataResponse(MSG_GET_DUTY_CYCLE, myDeskId, driver.getDutyCycle());
        break;
    }
    case MSG_GET_REFERENCE:
    {
        sendDataResponse(MSG_GET_REFERENCE, myDeskId, controller.getReference());
        break;
    }
    case MSG_GET_ILLUMINANCE:
    {
        sendDataResponse(MSG_GET_ILLUMINANCE, myDeskId, luxMeter.getLuxValue());
        break;
    }
    case MSG_GET_VOLTAGE:
    {
        sendDataResponse(MSG_GET_VOLTAGE, myDeskId, luxMeter.getLdrVoltage());
        break;
    }
    case MSG_GET_OCCUPANCY:
    {
        sendDataResponse(MSG_GET_OCCUPANCY, myDeskId, controller.getOccupancy());
        break;
    }
    case MSG_GET_ANTI_WINDUP:
    {
        sendDataResponse(MSG_GET_ANTI_WINDUP, myDeskId, controller.getAntiWindup());
        break;
    }
    case MSG_GET_FEEDBACK:
    {
        sendDataResponse(MSG_GET_FEEDBACK, myDeskId, controller.getFeedback());
        break;
    }
    case MSG_GET_EXTERNAL:
    {
        sendDataResponse(MSG_GET_EXTERNAL, myDeskId, controller.getExternal());
        break;
    }
    case MSG_GET_POWER:
    {
        sendDataResponse(MSG_GET_POWER, myDeskId, dataSt.getPowerConsumption());
        break;
    }
    case MSG_GET_TIME:
    {
        unsigned long elapsedTime = millis();
        sendDataResponse(MSG_GET_TIME, myDeskId, elapsedTime / 1000.0f);
        break;
    }
    case MSG_GET_ENERGY:
    {
        sendDataResponse(MSG_GET_ENERGY, myDeskId, dataSt.getEnergy());
        break;
    }
    case MSG_GET_VISIBILITY_ERROR:
    {
        sendDataResponse(MSG_GET_VISIBILITY_ERROR, myDeskId, dataSt.getVisibilityError());
        break;
    }
    case MSG_GET_FLICKER:
    {
        sendDataResponse(MSG_GET_FLICKER, myDeskId, dataSt.getFlicker());
        break;
    }
    case MSG_GET_LOWER_BOUND_OCCUPIED:
    {
        sendDataResponse(MSG_GET_LOWER_BOUND_OCCUPIED, myDeskId, controller.getLowerBoundOccupied());
        break;
    }
    case MSG_GET_LOWER_BOUND_UNOCCUPIED:
    {
        sendDataResponse(MSG_GET_LOWER_BOUND_UNOCCUPIED, myDeskId, controller.getLowerBoundUnoccupied());
        break;
    }
    case MSG_GET_CURRENT_LOWER_BOUND:
    {
        if (controller.getOccupancy())
        {
            sendDataResponse(MSG_GET_LOWER_BOUND_OCCUPIED, myDeskId, controller.getLowerBoundOccupied());
        }
        else
        {
            sendDataResponse(MSG_GET_LOWER_BOUND_UNOCCUPIED, myDeskId, controller.getLowerBoundUnoccupied());
        }
        break;
    }
    case MSG_GET_BUFFER_U:
    {
        int ID = atoi(tokens[3].c_str());
        if (isNotValidID(ID))
        {
            sendResponse(MSG_ERROR, "invalid desk ID %d", ID);
            return;
        }
        float uData[6000];
        uint16_t elements = dataSt.getUbuffer(uData);
        Serial.printf("b u %d\n", ID);
        for (uint16_t i = 0; i < elements - 1; i++)
        {
            Serial.printf("%0.2f, ", uData[i]);
        }
        Serial.printf("%0.2f\n", uData[elements - 1]);
        break;
    }
    case MSG_GET_BUFFER_Y:
    {
        int ID = atoi(tokens[3].c_str());
        if (isNotValidID(ID))
        {
            sendResponse(MSG_ERROR, "invalid desk ID %d", ID);
            return;
        }
        float yData[6000];
        uint16_t elements = dataSt.getYbuffer(yData);
        Serial.printf("b y %d\n", ID);
        for (uint16_t i = 0; i < elements - 1; i++)
        {
            Serial.printf("%0.2f ,", yData[i]);
        }
        Serial.printf("%0.2f\n", yData[elements - 1]);
        break;
    }
    case MSG_SET_DUTY_CYCLE:
    {
        if (tokens.size() < 3)
        {
            sendResponse(MSG_ERROR, "invalid set command");
            return;
        }

        float value = extractValue(tokens[2].c_str());
        if (value == -1.0f)
        {
            driver.setManualMode(false);
            sendResponse(MSG_ACK, "manual mode disabled");
            return;
        }

        if (value < 0.0f || value > 1.0f)
        {
            sendResponse(MSG_ERROR, "invalid duty cycle value %f", value);
            return;
        }

        driver.setManualMode(false);
        driver.setDutyCycle(value);
        driver.setManualMode(true);
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_SET_REFERENCE:
    {
        if (tokens.size() < 3)
        {
            sendResponse(MSG_ERROR, "invalid set command");
            return;
        }

        float value = extractValue(tokens[2].c_str());
        controller.setReference(value);
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_SET_OCCUPANCY:
    {
        if (tokens.size() < 3)
        {
            sendResponse(MSG_ERROR, "invalid set command");
            return;
        }

        int value = atoi(tokens[2].c_str());
        if (value == 0)
            controller.setOccupancy(false);
        else if (value == 1)
            controller.setOccupancy(true);
        else
        {
            sendResponse(MSG_ERROR, "invalid occupancy value %d", value);
            return;
        }
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_SET_ANTI_WINDUP:
    {
        if (tokens.size() < 3)
        {
            sendResponse(MSG_ERROR, "invalid set command");
            return;
        }

        int value = atoi(tokens[2].c_str());
        if (value == 0)
            controller.setAntiWindup(false);
        else if (value == 1)
            controller.setAntiWindup(true);
        else
        {
            sendResponse(MSG_ERROR, "invalid anti-windup value %d", value);
            return;
        }
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_SET_FEEDBACK:
    {
        if (tokens.size() < 3)
        {
            sendResponse(MSG_ERROR, "invalid set command");
            return;
        }

        int value = atoi(tokens[2].c_str());
        if (value == 0)
            controller.setFeedback(false);
        else if (value == 1)
            controller.setFeedback(true);
        else
        {
            sendResponse(MSG_ERROR, "invalid feedback value %d", value);
            return;
        }
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_SET_LOWER_BOUND_OCCUPIED:
    {
        if (tokens.size() < 3)
        {
            sendResponse(MSG_ERROR, "invalid set command");
            return;
        }

        float value = extractValue(tokens[2].c_str());
        if (value <= 0.0f)
        {
            sendResponse(MSG_ERROR, "invalid lower bound occupied value %f", value);
            return;
        }
        controller.setLowerBoundOccupied(value);
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_SET_LOWER_BOUND_UNOCCUPIED:
    {
        if (tokens.size() < 3)
        {
            sendResponse(MSG_ERROR, "invalid set command");
            return;
        }

        float value = extractValue(tokens[2].c_str());
        if (value <= 0.0f)
        {
            sendResponse(MSG_ERROR, "invalid lower bound unoccupied value %f", value);
            return;
        }
        controller.setLowerBoundUnoccupied(value);
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_STREAM_START_U:
    {
        streaming_u = true;
        break;
    }
    case MSG_STREAM_START_Y:
    {
        streaming_y = true;
        break;
    }
    case MSG_STREAM_START_R:
    {
        streaming_r = true;
        break;
    }
    case MSG_STREAM_START_V:
    {
        streaming_v = true;
        break;
    }
    case MSG_STREAM_START_ALL:
    {
        streaming_u = true;
        streaming_y = true;
        streaming_r = true;
        streaming_v = true;
        break;
    }
    case MSG_STREAM_STOP_U:
    {
        streaming_u = false;
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_STREAM_STOP_Y:
    {
        streaming_y = false;
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_STREAM_STOP_R:
    {
        streaming_r = false;
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_STREAM_STOP_V:
    {
        streaming_v = false;
        sendResponse(MSG_ACK, "");
        break;
    }
    case MSG_STREAM_STOP_ALL:
    {
        streaming_u = false;
        streaming_y = false;
        streaming_r = false;
        streaming_v = false;
        sendResponse(MSG_ACK, "");
        break;
    }
    default:
    {
        sendResponse(MSG_ERROR, "unknown command");
        break;
    }
    }
}

void pcInterface::streamSerialData(float u, float y, float r, float v, unsigned long time)
{
    if (streaming_u && !streaming_y && !streaming_r && !streaming_v)
    {
        sendResponse(MSG_GET_DUTY_CYCLE, "s %d %.2f %lu\n", myDeskId, u, time);
    }
    if (!streaming_u && streaming_y && !streaming_r && !streaming_v)
    {
        sendResponse(MSG_GET_ILLUMINANCE, "s %d %.2f %lu\n", myDeskId, y, time);
    }
    if (!streaming_u && !streaming_y && streaming_r && !streaming_v)
    {
        sendResponse(MSG_GET_REFERENCE, "s %d %.2f %lu\n", myDeskId, r, time);
    }
    if (!streaming_u && !streaming_y && !streaming_r && streaming_v)
    {
        sendResponse(MSG_GET_VOLTAGE, "s %d %.2f %lu\n", myDeskId, v, time);
    }

    if (streaming_u && streaming_y && streaming_r && streaming_v)
    {
        Serial.printf("s %d %.2f %.3f %.2f %.3f %lu\n", myDeskId, u, y, r, v, time);
    }
}

void pcInterface::sendResponse(MessageType msgType, const char *format, ...)
{
    char buffer[BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, BUFFER_SIZE, format, args);
    va_end(args);

    // Prefix with message type if needed
    Serial.print(buffer);
}

void pcInterface::sendDataResponse(MessageType msgType, int deskId, float value)
{
    switch (msgType)
    {
    case MSG_GET_DUTY_CYCLE:
        Serial.printf("u %d %.2f\n", deskId, value);
        break;
    case MSG_GET_REFERENCE:
        Serial.printf("r %d %.2f\n", deskId, value);
        break;
    case MSG_GET_ILLUMINANCE:
        Serial.printf("y %d %.2f\n", deskId, value);
        break;
    case MSG_GET_VOLTAGE:
        Serial.printf("v %d %.2f\n", deskId, value);
        break;
    case MSG_GET_EXTERNAL:
        Serial.printf("d %d %.2f\n", deskId, value);
        break;
    case MSG_GET_POWER:
        Serial.printf("p %d %.2f\n", deskId, value);
        break;
    case MSG_GET_TIME:
        Serial.printf("t %d %.2f\n", deskId, value);
        break;
    case MSG_GET_ENERGY:
        Serial.printf("E %d %.3f\n", deskId, value);
        break;
    case MSG_GET_VISIBILITY_ERROR:
        Serial.printf("V %d %.3f\n", deskId, value);
        break;
    case MSG_GET_FLICKER:
        Serial.printf("F %d %.6f\n", deskId, value);
        break;
    case MSG_GET_LOWER_BOUND_OCCUPIED:
        Serial.printf("O %d %.2f\n", deskId, value);
        break;
    case MSG_GET_LOWER_BOUND_UNOCCUPIED:
        Serial.printf("U %d %.2f\n", deskId, value);
        break;
    default:
        break;
    }
}

void pcInterface::sendDataResponse(MessageType msgType, int deskId, int value)
{
    switch (msgType)
    {
    case MSG_GET_OCCUPANCY:
        Serial.printf("o %d %d\n", deskId, value);
        break;
    case MSG_GET_ANTI_WINDUP:
        Serial.printf("a %d %d\n", deskId, value);
        break;
    case MSG_GET_FEEDBACK:
        Serial.printf("f %d %d\n", deskId, value);
        break;
    default:
        break;
    }
}

bool pcInterface::isNotValidID(int id)
{
    // check every id in the listIds
    for (int i = 0; i < numDesks; i++)
    {
        if (id == listIds[i])
        {
            return false;
        }
    }

    // if the id is not in the list, return true
    return true;
}

uint8_t pcInterface::extractDeskId(const char *cmd)
{
    return atoi(cmd);
}

float pcInterface::extractValue(const char *cmd)
{
    return atof(cmd);
}

void pcInterface::resetSystem()
{
    Serial.println("System reset initiated. - NOT DONE YET");
}

bool pcInterface::sendCanCommand(MessageType msgType, uint8_t targetDeskId, float value, int intValue)
{
    uint8_t data[8];
    uint8_t length = 8;

    // inicialization of data at zeros
    for (int i = 0; i < 8; i++)
    {
        data[i] = 0;
    }

    // First byte is always the target desk ID
    data[0] = targetDeskId;

    // For set commands, include the value in the message
    if (msgType >= MSG_SET_DUTY_CYCLE && msgType <= MSG_SET_ENERGY_COST)
    {
        if (msgType == MSG_SET_OCCUPANCY || msgType == MSG_SET_ANTI_WINDUP || msgType == MSG_SET_FEEDBACK)
        {
            // Integer commands (1 byte value)
            data[1] = intValue;
            length = 2;
        }
        else
        {
            // Float commands (4 byte value)
            memcpy(data + 1, &value, sizeof(float));
        }
    }

    // Debug output
    Serial.printf("[CAN TX] Sending message type %d from desk %d to desk %d, data: ",
                  msgType, myDeskId, targetDeskId);
    for (int i = 0; i < length; i++)
    {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();

    uint8_t msgTypeId = static_cast<uint8_t>(msgType);

    Serial.println(msgTypeId);

    // Send using our own deskId in the CAN ID
    return canHandler.sendMessage(msgTypeId, myDeskId, data, length);
}

bool pcInterface::waitForCanResponse(MessageType expectedMsgType, uint8_t expectedDeskId, float *outValue, int *outIntValue)
{
    unsigned long startTime = millis();
    const unsigned long timeout = 100; // 100ms timeout

    while (millis() - startTime < timeout)
    {
        // Check if there are any incoming messages

        uint8_t receivedMsgType;
        uint8_t senderDeskId;
        uint8_t data[8];
        uint8_t length;

        // debug in loop
        // Serial.println("[CAN] Processing incoming messages in waitForCanResponse()");

        if (canHandler.readMessage(&receivedMsgType, &senderDeskId, data, &length))
        {
            // Debug received message
            Serial.printf("[CAN RX] Received message type %d from desk %d, data: ",
                          receivedMsgType, senderDeskId);
            for (int i = 0; i < length; i++)
            {
                Serial.printf("%02X ", data[i]);
            }
            Serial.println();

            if (receivedMsgType == static_cast<uint8_t>(expectedMsgType) &&
                senderDeskId == expectedDeskId)
            {
                // Handle the response data
                if (outValue != nullptr && length >= sizeof(float))
                {
                    memcpy(outValue, data, sizeof(float));
                }
                if (outIntValue != nullptr && length >= 1)
                {
                    *outIntValue = data[0];
                }
                return true;
            }
            else if (receivedMsgType == static_cast<uint8_t>(MSG_ERROR) &&
                     senderDeskId == expectedDeskId)
            {
                Serial.println("[CAN] Received error response");
                return false;
            }
        }
    }

    Serial.println("[CAN] Response timeout");
    return false;
}

void pcInterface::processIncomingCANMessages()
{
    // Check if there are any incoming messages
    uint8_t messageId = 0, senderDeskId = 0, data[8], length = 0;
    const char *valueName = "";
    float floatValue = 0.0f;
    int intValue = 0;
    bool isFloatResponse = true;
    MessageType msgType;

    // Serial.printf("[CAN] Processing message type: %d from desk %d\n", msgType, senderDeskId);

    bool success = false;
    const char *settingName = "";

    if (canHandler.readMessage(&messageId, &senderDeskId, data, &length))
    {
        Serial.printf("[CAN RX] Message ID: %d, From Desk: %d, Length: %d, Data: ",
                      messageId, senderDeskId, length);
        for (int i = 0; i < length; i++)
        {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();

        // Serial.printf("[CAN] My Desk ID: %d\n", myDeskId); // Confirm myDeskId
        if (length < 1 || data[0] != myDeskId)
        {
            Serial.printf("[CAN] Ignoring message not for me (target desk %d, I'm desk %d)\n",
                          length > 0 ? data[0] : 0xFF, myDeskId);
        }

        msgType = static_cast<MessageType>(messageId);
        switch (msgType)
        {
        // Handle get commands
        case MSG_GET_DUTY_CYCLE:

            floatValue = driver.getDutyCycle();
            valueName = "Duty Cycle";
            break;
        case MSG_GET_REFERENCE:
            floatValue = controller.getReference();
            valueName = "Reference";
            break;
        case MSG_GET_ILLUMINANCE:
            floatValue = luxMeter.getLuxValue();
            valueName = "Illuminance";
            break;
        case MSG_GET_VOLTAGE:
            floatValue = luxMeter.getLdrVoltage();
            valueName = "Voltage";
            break;
        case MSG_GET_OCCUPANCY:
            intValue = controller.getOccupancy();
            valueName = "Occupancy";
            isFloatResponse = false;
            break;
        case MSG_GET_ANTI_WINDUP:
            intValue = controller.getAntiWindup();
            valueName = "Anti-windup";
            isFloatResponse = false;
            break;
        case MSG_GET_FEEDBACK:
            intValue = controller.getFeedback();
            valueName = "Feedback";
            isFloatResponse = false;
            break;
        case MSG_GET_EXTERNAL:
            floatValue = controller.getExternal();
            valueName = "External";
            break;
        case MSG_GET_POWER:
            floatValue = dataSt.getPowerConsumption();
            valueName = "Power";
            break;
        case MSG_GET_TIME:
            floatValue = millis() / 1000.0f;
            valueName = "Time";
            break;
        case MSG_GET_ENERGY:
            floatValue = dataSt.getEnergy();
            valueName = "Energy";
            break;
        case MSG_GET_VISIBILITY_ERROR:
            floatValue = dataSt.getVisibilityError();
            valueName = "Visibility Error";
            break;
        case MSG_GET_FLICKER:
            floatValue = dataSt.getFlicker();
            valueName = "Flicker";
            break;
        case MSG_GET_LOWER_BOUND_OCCUPIED:
            floatValue = controller.getLowerBoundOccupied();
            valueName = "Lower Bound (Occupied)";
            break;
        case MSG_GET_LOWER_BOUND_UNOCCUPIED:
            floatValue = controller.getLowerBoundUnoccupied();
            valueName = "Lower Bound (Unoccupied)";
            break;
        case MSG_GET_CURRENT_LOWER_BOUND:
            floatValue = controller.getOccupancy() ? controller.getLowerBoundOccupied() : controller.getLowerBoundUnoccupied();
            valueName = "Current Lower Bound";
            break;
        case MSG_GET_ENERGY_COST:
            // floatValue = dataSt.getEnergyCost();
            valueName = "Energy Cost";
            break;
        case MSG_SET_DUTY_CYCLE:
        {
            settingName = "Duty Cycle";
            if (length >= sizeof(float))
            {
                float value;
                memcpy(&value, data + 1, sizeof(float));
                Serial.printf("[CAN] Setting Duty Cycle to: %.2f\n", value);

                if (value >= 0.0f && value <= 1.0f)
                {
                    driver.setManualMode(false);
                    driver.setDutyCycle(value);
                    driver.setManualMode(true);
                    success = true;
                }
                else
                {
                    Serial.println("[CAN] Error: Duty cycle out of range");
                }
            }
            break;
        }
        case MSG_SET_REFERENCE:
        {
            settingName = "Reference";
            if (length >= sizeof(float))
            {
                float value;
                memcpy(&value, data, sizeof(float));
                Serial.printf("[CAN] Setting Reference to: %.2f\n", value);
                controller.setReference(value);
                success = true;
            }
            break;
        }
        case MSG_SET_OCCUPANCY:
        {
            settingName = "Occupancy";
            if (length >= 1)
            {
                bool value = data[0] != 0;
                Serial.printf("[CAN] Setting Occupancy to: %s\n", value ? "true" : "false");
                controller.setOccupancy(value);
                success = true;
            }
            break;
        }
        case MSG_SET_ANTI_WINDUP:
        {
            settingName = "Anti-windup";
            if (length >= 1)
            {
                bool value = data[0] != 0;
                Serial.printf("[CAN] Setting Anti-windup to: %s\n", value ? "true" : "false");
                controller.setAntiWindup(value);
                success = true;
            }
            break;
        }
        case MSG_SET_FEEDBACK:
        {
            settingName = "Feedback";
            if (length >= 1)
            {
                bool value = data[0] != 0;
                Serial.printf("[CAN] Setting Feedback to: %s\n", value ? "true" : "false");
                controller.setFeedback(value);
                success = true;
            }
            break;
        }
        case MSG_SET_LOWER_BOUND_OCCUPIED:
        {
            settingName = "Lower Bound (Occupied)";
            if (length >= sizeof(float))
            {
                float value;
                memcpy(&value, data, sizeof(float));
                Serial.printf("[CAN] Setting Lower Bound (Occupied) to: %.2f\n", value);
                if (value > 0.0f)
                {
                    controller.setLowerBoundOccupied(value);
                    success = true;
                }
                else
                {
                    Serial.println("[CAN] Error: Value must be positive");
                }
            }
            break;
        }
        case MSG_SET_LOWER_BOUND_UNOCCUPIED:
        {
            settingName = "Lower Bound (Unoccupied)";
            if (length >= sizeof(float))
            {
                float value;
                memcpy(&value, data, sizeof(float));
                Serial.printf("[CAN] Setting Lower Bound (Unoccupied) to: %.2f\n", value);
                if (value > 0.0f)
                {
                    controller.setLowerBoundUnoccupied(value);
                    success = true;
                }
                else
                {
                    Serial.println("[CAN] Error: Value must be positive");
                }
            }
            break;
        }
        case MSG_SET_ENERGY_COST:
        {
            settingName = "Energy Cost";
            if (length >= sizeof(float))
            {
                float value;
                memcpy(&value, data, sizeof(float));
                Serial.printf("[CAN] Setting Energy Cost to: %.2f\n", value);
                // dataSt.setEnergyCost(value);
                success = true;
            }
            break;
        }
        case MSG_ACK:
            // Acknowledge message received
            Serial.printf("[CAN] Acknowledgment from desk %d\n", senderDeskId);
            break;
        case MSG_ERROR:
            // Error message received
            Serial.printf("[CAN] Error from desk %d\n", senderDeskId);
            break;
        default:
            // Unknown command - send error with our deskId and sender's deskId
            uint8_t responseData[1] = {senderDeskId};
            Serial.printf("[CAN] Unknown command type: %d\n", msgType);
            break;
        }

        // If is get message
        if (msgType >= MSG_GET_DUTY_CYCLE && msgType <= MSG_GET_ENERGY_COST)
        {
            // Debug output
            if (isFloatResponse)
            {
                Serial.printf("[CAN] Sending %s: %.2f\n", valueName, floatValue);
            }
            else
            {
                Serial.printf("[CAN] Sending %s: %d\n", valueName, intValue);
            }

            // Send the response
            if (isFloatResponse)
            {
                memcpy(data, &floatValue, sizeof(float));

                // Serial.printf("[CAN] Sending %s: %.2f\n", valueName, floatValue);
                canHandler.sendMessage(messageId, myDeskId, data, sizeof(float));
            }
            else
            {
                data[0] = intValue;
                canHandler.sendMessage(messageId, myDeskId, data, 1);
            }
        }

        // if is set message
        if (msgType >= MSG_SET_DUTY_CYCLE && msgType <= MSG_SET_ENERGY_COST)
        {
            bool success = false;

            // Data format: [0] = target deskId, [1..] = value

            // Send ACK/ERROR with our deskId in CAN ID and sender's deskId in data[0]
            uint8_t responseData[1] = {senderDeskId};
            if (success)
            {
                canHandler.sendMessage(static_cast<uint8_t>(MSG_ACK), myDeskId, responseData, 1);
            }
            else
            {
                canHandler.sendMessage(static_cast<uint8_t>(MSG_ERROR), myDeskId, responseData, 1);
            }

            // Send acknowledgment or error
            if (success)
            {
                Serial.printf("[CAN] %s set successfully\n", settingName);
                canHandler.sendMessage(static_cast<uint8_t>(MSG_ACK), myDeskId, nullptr, 0);
            }
            else
            {
                Serial.printf("[CAN] Failed to set %s\n", settingName);
                canHandler.sendMessage(static_cast<uint8_t>(MSG_ERROR), myDeskId, nullptr, 0);
            }
        }
    }
}

void pcInterface::myIdInit(int id)
{
    myDeskId = id;
    listIds.push_back(myDeskId);
    numDesks = 1;
    Serial.printf("Desk ID initialized to %d\n", myDeskId);
}

void pcInterface::addDeskId(int id)
{
    if (isNotValidID(id))
    {
        listIds.push_back(id);
        numDesks++;
        Serial.printf("Desk ID %d added\n", id);
    }
    else
    {
        Serial.printf("Desk ID %d already exists\n", id);
    }
}