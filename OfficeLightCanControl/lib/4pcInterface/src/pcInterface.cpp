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

void pcInterface::processIncomingCANMessages()
{
    uint8_t messageId = 0, senderDeskId = 0, data[8], length = 0;
    float floatValue = 0.0f;
    int intValue = 0;
    bool success = false;
    MessageType msgType;

    if (canHandler.readMessage(&messageId, &senderDeskId, data, &length))
    {
        if (length < 1 || data[0] != myDeskId)
            return;

        msgType = static_cast<MessageType>(messageId);
        switch (msgType)
        {
        case MSG_GET_DUTY_CYCLE:
            floatValue = driver.getDutyCycle();
            break;
        case MSG_GET_REFERENCE:
            floatValue = controller.getReference();
            break;
        case MSG_GET_ILLUMINANCE:
            floatValue = luxMeter.getLuxValue();
            break;
        case MSG_GET_VOLTAGE:
            floatValue = luxMeter.getLdrVoltage();
            break;
        case MSG_GET_OCCUPANCY:
            intValue = controller.getOccupancy();
            floatValue = static_cast<float>(intValue);
            break;
        case MSG_GET_ANTI_WINDUP:
            intValue = controller.getAntiWindup();
            floatValue = static_cast<float>(intValue);
            break;
        case MSG_GET_FEEDBACK:
            intValue = controller.getFeedback();
            floatValue = static_cast<float>(intValue);
            break;
        case MSG_GET_EXTERNAL:
            floatValue = controller.getExternal();
            break;
        case MSG_GET_POWER:
            floatValue = dataSt.getPowerConsumption();
            break;
        case MSG_GET_TIME:
            floatValue = millis() / 1000.0f;
            break;
        case MSG_GET_ENERGY:
            floatValue = dataSt.getEnergy();
            break;
        case MSG_GET_VISIBILITY_ERROR:
            floatValue = dataSt.getVisibilityError();
            break;
        case MSG_GET_FLICKER:
            floatValue = dataSt.getFlicker();
            break;
        case MSG_GET_LOWER_BOUND_OCCUPIED:
            floatValue = controller.getLowerBoundOccupied();
            break;
        case MSG_GET_LOWER_BOUND_UNOCCUPIED:
            floatValue = controller.getLowerBoundUnoccupied();
            break;
        case MSG_GET_CURRENT_LOWER_BOUND:
            floatValue = controller.getOccupancy() ? controller.getLowerBoundOccupied() : controller.getLowerBoundUnoccupied();
            break;
        case MSG_GET_ENERGY_COST:
            // floatValue = dataSt.getEnergyCost();
            break;

            
        case MSG_SET_DUTY_CYCLE:
            if (length >= sizeof(float) + 1)
            {
                float value;
                memcpy(&value, data + 1, sizeof(float));
                if (value >= 0.0f && value <= 1.0f)
                {
                    driver.setManualMode(false);
                    driver.setDutyCycle(value);
                    driver.setManualMode(true);
                    success = true;
                }
            }
            break;
        case MSG_SET_REFERENCE:
            if (length >= sizeof(float) + 1)
            {
                float value;
                memcpy(&value, data + 1, sizeof(float));
                controller.setReference(value);
                success = true;
            }
            break;
        case MSG_SET_OCCUPANCY:
            if (length >= 2)
            {
                controller.setOccupancy(data[1] != 0);
                success = true;
            }
            break;
        case MSG_SET_ANTI_WINDUP:
            if (length >= 2)
            {
                controller.setAntiWindup(data[1] != 0);
                success = true;
            }
            break;
        case MSG_SET_FEEDBACK:
            if (length >= 2)
            {
                controller.setFeedback(data[1] != 0);
                success = true;
            }
            break;
        case MSG_SET_LOWER_BOUND_OCCUPIED:
            if (length >= sizeof(float) + 1)
            {
                float value;
                memcpy(&value, data + 1, sizeof(float));
                if (value > 0.0f)
                {
                    controller.setLowerBoundOccupied(value);
                    success = true;
                }
            }
            break;
        case MSG_SET_LOWER_BOUND_UNOCCUPIED:
            if (length >= sizeof(float) + 1)
            {
                float value;
                memcpy(&value, data + 1, sizeof(float));
                if (value > 0.0f)
                {
                    controller.setLowerBoundUnoccupied(value);
                    success = true;
                }
            }
            break;
        case MSG_SET_ENERGY_COST:
            if (length >= sizeof(float) + 1)
            {
                float value;
                memcpy(&value, data + 1, sizeof(float));
                // dataSt.setEnergyCost(value);
                success = true;
            }
            break;
        case MSG_ACK:
            // Handle ACK messages if needed
            if (length == 1)
            {
                Serial.printf("ack from %d\n", senderDeskId);
            }
            if (length == sizeof(float) + 2)
            {
                memcpy(&floatValue, data + 2, sizeof(float));
                msgType = static_cast<MessageType>(data[1]);
                Serial.printf(" %d %.3f\n", senderDeskId, floatValue);
            }
            break;
        case MSG_ERROR:
        default:
            return;
        }

        if (msgType >= MSG_GET_DUTY_CYCLE && msgType <= MSG_GET_ENERGY_COST)
        {
            sendCanCommand(MSG_ACK, senderDeskId, floatValue, 0);
        }
        else if (msgType >= MSG_SET_DUTY_CYCLE && msgType <= MSG_SET_ENERGY_COST)
        {
            sendCanCommand(MSG_ACK, senderDeskId, 0.0f, 0);
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


// Private Functions

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




void pcInterface::handleRemoteCommand(MessageType msgType, uint8_t targetDeskId, std::vector<std::string> tokens)
{
    float value = 0.0f;
    // if Get commands
    if (msgType >= MSG_GET_DUTY_CYCLE && msgType <= MSG_GET_ENERGY_COST)
    {
        if (!sendCanCommand(msgType, targetDeskId))
        {
            sendResponse(MSG_ERROR, "failed to send command to desk %d", targetDeskId);
        }
        value = waitForCanResponse(targetDeskId);
        if (value == 0.0f)
        {
            sendResponse(MSG_ERROR, "no ack from desk %d", targetDeskId);
        }
        else if (msgType == MSG_GET_DUTY_CYCLE)
        {
            sendResponse(MSG_ERROR, "u %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_REFERENCE)
        {
            sendResponse(MSG_ERROR, "r %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_ILLUMINANCE)
        {
            sendResponse(MSG_ERROR, "y %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_VOLTAGE)
        {
            sendResponse(MSG_ERROR, "v %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_OCCUPANCY)
        {
            sendResponse(MSG_ERROR, "o %d %d\n", targetDeskId, static_cast<int>(value));
        }
        else if (msgType == MSG_GET_ANTI_WINDUP)
        {
            sendResponse(MSG_ERROR, "a %d %d\n", targetDeskId, static_cast<int>(value));
        }
        else if (msgType == MSG_GET_FEEDBACK)
        {
            sendResponse(MSG_ERROR, "f %d %d\n", targetDeskId, static_cast<int>(value));
        }
        else if (msgType == MSG_GET_EXTERNAL)
        {
            sendResponse(MSG_ERROR, "d %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_POWER)
        {
            sendResponse(MSG_ERROR, "p %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_TIME)
        {
            sendResponse(MSG_ERROR, "t %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_ENERGY)
        {
            sendResponse(MSG_ERROR, "E %d %.3f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_VISIBILITY_ERROR)
        {
            sendResponse(MSG_ERROR, "V %d %.3f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_FLICKER)
        {
            sendResponse(MSG_ERROR, "F %d %.6f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_LOWER_BOUND_OCCUPIED)
        {
            sendResponse(MSG_ERROR, "O %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_LOWER_BOUND_UNOCCUPIED)
        {
            sendResponse(MSG_ERROR, "U %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_CURRENT_LOWER_BOUND)
        {
            sendResponse(MSG_ERROR, "L %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_ENERGY_COST)
        {
            // sendResponse(MSG_ERROR, "C %d %.2f\n", targetDeskId, value);
        }
        else if (msgType == MSG_GET_BUFFER_U)
        {
            // Handle buffer request here
        }
        else if (msgType == MSG_GET_BUFFER_Y)
        {
            // Handle buffer request here
        }
        else
        {
            sendResponse(MSG_ERROR, "no ack from desk %d", targetDeskId);
        }
        return;
    }
    else
    {
        sendResponse(MSG_ERROR, "remote command not supported");
    }

    return;

    // // Set commands
    // case MSG_SET_DUTY_CYCLE:
    // case MSG_SET_REFERENCE:
    // case MSG_SET_OCCUPANCY:
    // case MSG_SET_ANTI_WINDUP:
    // case MSG_SET_FEEDBACK:
    // case MSG_SET_LOWER_BOUND_OCCUPIED:
    // case MSG_SET_LOWER_BOUND_UNOCCUPIED:
    // case MSG_SET_ENERGY_COST:
    // {
    //     if (tokens.size() < 3)
    //     {
    //         sendResponse(MSG_ERROR, "invalid set command");
    //         return;
    //     }

    //     float floatValue = 0.0f;
    //     int intValue = 0;
    //     bool isFloatCommand = (msgType != MSG_SET_OCCUPANCY &&
    //                            msgType != MSG_SET_ANTI_WINDUP &&
    //                            msgType != MSG_SET_FEEDBACK);

    //     if (isFloatCommand)
    //     {
    //         floatValue = extractValue(tokens[2].c_str());
    //     }
    //     else
    //     {
    //         intValue = atoi(tokens[2].c_str());
    //     }

    //     if (sendCanCommand(msgType, targetDeskId, floatValue, intValue))
    //     {
    //         if (waitForCanResponse(MSG_ACK, targetDeskId))
    //         {
    //             sendResponse(MSG_ACK, "");
    //         }
    //         else
    //         {
    //             sendResponse(MSG_ERROR, "no ack from desk %d", targetDeskId);
    //         }
    //     }
    //     else
    //     {
    //         sendResponse(MSG_ERROR, "failed to send command to desk %d", targetDeskId);
    //     }
    //     break;
    // }

    // // Stream commands (not supported for remote desks)
    // case MSG_STREAM_START_U:
    // case MSG_STREAM_START_Y:
    // case MSG_STREAM_START_R:
    // case MSG_STREAM_START_V:
    // case MSG_STREAM_START_ALL:
    // case MSG_STREAM_STOP_U:
    // case MSG_STREAM_STOP_Y:
    // case MSG_STREAM_STOP_R:
    // case MSG_STREAM_STOP_V:
    // case MSG_STREAM_STOP_ALL:
    //     sendResponse(MSG_ERROR, "streaming not supported for remote desks");
    //     break;

}

bool pcInterface::sendCanCommand(MessageType msgType, uint8_t targetDeskId, float value, int intValue)
{
    uint8_t data[8];
    uint8_t length = 1; // Default length is 1 byte (target desk ID)
    
    // First byte is always the target desk ID
    data[0] = targetDeskId;

    // For set commands, include the value in the message
    if (msgType >= MSG_SET_DUTY_CYCLE && msgType <= MSG_SET_ENERGY_COST)
    {
        if (msgType == MSG_SET_OCCUPANCY || msgType == MSG_SET_ANTI_WINDUP || msgType == MSG_SET_FEEDBACK)
        {
            // Integer commands (1 byte value)
            data[1] = intValue;
            length = 2; // 1 byte for target desk ID + 1 byte for int value
        }
        else
        {
            // Float commands (4 byte value)
            memcpy(data + 1, &value, sizeof(float));
            length = sizeof(float) + 1; // 1 byte for target desk ID + 4 bytes for float value
        }
    }

    if (msgType == MSG_ACK && value != 0.0f)
    {
        // For ACK messages, we include the value in the message
        memcpy(data + 1, &value, sizeof(float));
        length = sizeof(float) + 1; // 1 byte for target desk ID + 4 bytes for float value
    }
    else if (msgType == MSG_ACK && value == 0.0f)
    {
        // For ACK messages, we only need the target desk ID
        length = 1; // 1 byte for target desk ID
    }
    // // Debug output
    // Serial.printf("[CAN TX] Sending message type %d from desk %d to desk %d, data: ",
    //               msgType, myDeskId, targetDeskId);
    // for (int i = 0; i < length; i++)
    // {
    //     Serial.printf("%02X ", data[i]);
    // }
    // Serial.println();

    uint8_t msgTypeId = static_cast<uint8_t>(msgType);

    // Send using our own deskId in the CAN ID
    return canHandler.sendMessage(msgTypeId, myDeskId, data, length);
}

float pcInterface::waitForCanResponse(uint8_t expectedDeskId)
{
    uint8_t messageId = 0, senderDeskId = 0, data[8], length = 0;
    float floatValue = 0.0f;
    int intValue = 0;
    bool success = false;
    MessageType msgType;

    if (canHandler.readMessage(&messageId, &senderDeskId, data, &length))
    {
        if (length < 1 || data[0] != myDeskId)
            return;

        msgType = static_cast<MessageType>(messageId);
    
        if (msgType == MSG_ACK && senderDeskId == expectedDeskId)
        {
            if (length == sizeof(float) + 2)
            {
                memcpy(&floatValue, data + 2, sizeof(float));
                msgType = static_cast<MessageType>(data[1]);
                Serial.printf(" %d %.3f\n", senderDeskId, floatValue);
            }
            return floatValue;
        }
    }
    return 0.0f; // No response received
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
