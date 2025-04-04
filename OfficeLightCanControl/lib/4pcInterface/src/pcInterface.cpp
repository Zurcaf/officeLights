#include "pcInterface.h"

pcInterface::pcInterface(uint8_t deskId, LuxMeter& luxM, Driver& driv, localController& ctrl, dataStorageMetrics& storage)
    : myDeskId(deskId),luxMeter(luxM), driver(driv), controller(ctrl), dataSt(storage)
{
    // Initialize single desk state
    listIds.push_back(myDeskId); // Add the current desk ID to the list
    numDesks = 1;                // Initialize with 1 desk
}

void pcInterface::begin(uint32_t baudRate)
{
    Serial.begin(baudRate);
    while (!Serial)
    { // Wait for Serial to be ready
        delay(10);
    }

    // sendResponse("Desk %d initialized", myDeskId); // Debug: Confirm initialization
}

void pcInterface::processSerial()
{
    if (Serial.available())
    {
        // sendResponse("Serial data available"); // Debug: Confirm data is received
    }
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            commandBuffer[bufferIndex] = '\0';
            if (bufferIndex > 0)
            {
                // sendResponse("Received: %s\n", commandBuffer); // Debug: Show received command
                parseCommand(commandBuffer);
                // Clear the buffer
                for (uint8_t i = 0; i < BUFFER_SIZE; i++)
                {
                    commandBuffer[i] = '\0';
                }
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
    std::vector<std::string> tokens; // Vector to store up to 4 tokens

    // Tokenize the command, limiting to 4 tokens
    while (tokens.size() < 5 && std::getline(ss, token, ' '))
    {
        tokens.push_back(token);
    }

    // Example: Basic validation and debug output
    if (tokens.empty())
    {
        sendResponse("err - empty command");
        return;
    }
    if (tokens.size() > 4)
    {
        sendResponse("err - too many tokens");
        return;
    }

    // Check if the command is a reset command
    if (tokens.size() == 1)
    {
        if (tokens[0] == "R")
        {
            resetSystem();
            sendResponse("ack");
            return; // Exit early
        }
    }

    // Check if the command is starting stream command

    if (tokens[0] == "s" || tokens[0] == "S")
    {
        executeStreamCommand(tokens);
        return;
    }

    // Check if the command is a get or set command
    if (tokens[0] == "g")
    {
        executeGetCommand(tokens);
        return;
    }

    executeSetCommand(tokens);

    // Serial.printf("Command: %s\n", cmd);
    // Serial.printf("Token 1: %s\n", tokens[0].c_str());
    // Serial.printf("Token 2: %s\n", tokens[1].c_str());
    // Serial.printf("Token 3: %s\n", tokens[2].c_str());
    // Serial.printf("Token 4: %s\n", tokens[3].c_str());
    // Serial.printf("Token 5: %s\n", tokens[4].c_str());

    return;
}

bool pcInterface::isNotValidID(int id)
{
    return false;
    // if (std::find(listIds.begin(), listIds.end(), id) == listIds.end())
    // {
    //     return true;
    // }
    // return false;
}

void pcInterface::executeGetCommand(std::vector<std::string> tokens)
{
    int ID = 0;

    // Check if the command is a get buffer command
    // g b u id
    // g b y id
    if (tokens[1] == "b")
    {
        if (tokens.size() != 4)
        {
            sendResponse("err - invalid get buffer command size");
            return;
        }

        ID = atoi(tokens[3].c_str());

        if (isNotValidID(ID))
        {
            sendResponse("err - invalid desk ID %d", ID);
            return;
        }
        if (tokens[2] == "u")
        {
            float uData[6000];
            uint16_t elements = dataSt.getUbuffer(uData);
            // print the buffer contents
            Serial.printf("b u %d\n", ID);

            for (uint16_t i = 0; i < elements-1; i++)
            {
                Serial.printf("%0.2f, ", uData[i]);
            }
            Serial.printf("%0.2f\n", uData[elements-1]);
        }
        else if (tokens[2] == "y")
        {
            float yData[6000];
            uint16_t elements = dataSt.getYbuffer(yData);
            // print the buffer contents
            Serial.printf("b y %d\n", ID);

            for (uint16_t i = 0; i < elements-1; i++)
            {
                Serial.printf("%0.2f ,", yData[i]);
            }
            Serial.printf("%0.2f\n", yData[elements-1]);
        }
        else
        {
            sendResponse("err - unknown get buffer command", tokens[3]);
        }
        return;
    }

    if (tokens.size() != 3)
    {
        sendResponse("err - invalid get command size");
        return;
    }

    ID = atoi(tokens[2].c_str());
    if (isNotValidID(ID))
    {
        sendResponse("err - invalid desk ID %d", ID);
        return;
    }

    if (tokens[1] == "u")
    {
        sendResponse("u %d %.2f", myDeskId, driver.getDutyCycle());
    }
    else if (tokens[1] == "r")
    {
        sendResponse("r %d %.2f", myDeskId, controller.getReference());
    }
    else if (tokens[1] == "y")
    {
        // Get the measured illuminance value
        sendResponse("y %d %.2f", myDeskId, luxMeter.getLuxValue());
    }
    else if (tokens[1] == "v")
    {
        // Get the LDR voltage value
        sendResponse("v %d %.2f", myDeskId, luxMeter.getLdrVoltage());
    }
    else if (tokens[1] == "o")
    {
        // Get the occupancy status
        sendResponse("o %d %d", myDeskId, controller.getOccupancy());
    }
    else if (tokens[1] == "a")
    {
        // Get the anti-windup status
        sendResponse("a %d %d", myDeskId, controller.getAntiWindup());
    }
    else if (tokens[1] == "f")
    {
        // Get the feedback control status
        sendResponse("f %d %d", myDeskId, controller.getFeedback());
    }
    else if (tokens[1] == "d")
    {
        // Get the external illuminance value
        sendResponse("d %d %.2f", myDeskId, controller.getExternal());
    }
    else if (tokens[1] == "p")
    {
        // Get the power consumption value
        sendResponse("p %d %.2f", myDeskId, dataSt.getPowerConsumption());
    }
    else if (tokens[1] == "t")
    {
        // Get the elapsed time value
        unsigned long elapsedTime = millis();
        sendResponse("t %d %.2f", myDeskId, elapsedTime / 1000.0f); // Convert to seconds
    }
    else if (tokens[1] == "E")
    {
        // Get the energy consumption value
        sendResponse("E %d %.3f", myDeskId, dataSt.getEnergy());
    }
    else if (tokens[1] == "V")
    {
        // Get the visibility error value
        sendResponse("V %d %.3f", myDeskId, dataSt.getVisibilityError());
    }
    else if (tokens[1] == "F")
    {
        // Get the flicker error value
        sendResponse("F %d %.6f", myDeskId, dataSt.getFlicker());
    }
    else if (tokens[1] == "O")
    {
        // Get the occupied lower bound value
        sendResponse("O %d %.2f", myDeskId, controller.getLowerBoundOccupied());
    }
    else if (tokens[1] == "U")
    {
        // Get the unoccupied lower bound value
        sendResponse("U %d %.2f", myDeskId, controller.getLowerBoundUnoccupied());
    }
    else if (tokens[1] == "L")
    {
        // Get the current lower bound value
        if (controller.getOccupancy())
        {
            controller.getLowerBoundOccupied();
            sendResponse("O %d %.2f", myDeskId, controller.getLowerBoundOccupied());
        }
        else
        {
            controller.getLowerBoundUnoccupied();
            sendResponse("U %d %.2f", myDeskId, controller.getLowerBoundUnoccupied());
        }
    }
    else if (tokens[1] == "C")
    {
        // sendResponse("C %d %.2f", myDeskId, desk.energyCost);
    }
    else
    {
        sendResponse("err - unknown get command argument %s", tokens[1]);
    }
}

void pcInterface::executeStreamCommand(std::vector<std::string> tokens)
{
    // Example: Implement the stream command from id and variable x
    // x = y, u; y = measured illuminance, u = duty cycle
    // s <x> <id>       // Start stream command
    // S <x> <id>       // Stop stream command
    if (tokens.size() != 3)
    {
        sendResponse("err - invalid stream command size");
        return;
    }

    int ID = atoi(tokens[2].c_str());
    if (isNotValidID(ID))
    {
        sendResponse("err - invalid desk ID %d", ID);
        return;
    }

    if (tokens[0] == "s")
    {
        if (tokens[1] == "y")
        {
            streaming_y = true;
        }
        else if (tokens[1] == "u")
        {
            streaming_u = true;
        }
        else if (tokens[1] == "r")
        {
            streaming_r = true;
        }
        else if (tokens[1] == "all")
        {
            streaming_y = true;
            streaming_u = true;
            streaming_r = true;
            streaming_v = true;
        }
        else
        {
            sendResponse("err - unknown stream command argument %s", tokens[1]);
        }
    }
    else if (tokens[0] == "S")
    {
        if (tokens[1] == "y")
        {
            streaming_y = false;
            sendResponse("ack");
        }
        else if (tokens[1] == "u")
        {
            streaming_u = false;
            sendResponse("ack");
        }
        else if (tokens[1] == "r")
        {
            streaming_r = false;
            sendResponse("ack");
        }
        else if (tokens[1] == "all")
        {
            streaming_y = false;
            streaming_u = false;
            streaming_r = false;
            streaming_v = false;
            sendResponse("ack");
        }
        else
        {
            sendResponse("err");
        }
    }
}

void pcInterface::streamSerialData(float u, float y, float r, float v, unsigned long time)
{
    if (streaming_u && !streaming_y && !streaming_r && !streaming_v)
    {
        sendResponse("s u %d %.2f %lu\n", myDeskId, u, time);
    }
    if (!streaming_u && streaming_y && !streaming_r && !streaming_v)
    {
        sendResponse("s y %d %.2f %lu\n", myDeskId, y, time);
    }
    if (!streaming_u && !streaming_y && streaming_r && !streaming_v)
    {
        sendResponse("s r %d %.2f %lu\n", myDeskId, r, time);
    }
    if (!streaming_u && !streaming_y && !streaming_r && streaming_v)
    {
        sendResponse("s v %d %.2f %lu\n", myDeskId, v, time);
    }

    if (streaming_u && streaming_y && streaming_r && streaming_v)
    {
        sendResponse("s %d %.2f %.3f %.2f %.3f %lu\n", myDeskId, u, y, r, v, time);
    }
}

void pcInterface::executeSetCommand(std::vector<std::string> tokens)
{
    if (tokens.size() != 3)
    {
        sendResponse("err - invalid set dommand size");
        return;
    }

    int ID = atoi(tokens[1].c_str());
    if (isNotValidID(ID))
    {
        sendResponse("err - invalid desk ID %d", ID);
        return;
    }

    if (tokens[0] == "u")
    {
        float value = extractValue(tokens[2].c_str());

        // Check if it's a special case for deactivating manual mode
        if (value == -1.0f)
        {
            driver.setManualMode(false); // Set manual mode to false
            sendResponse("ack - manual mode disabled");
            return;
        }
        
        // Check if the value is within the valid range
        // Assuming the valid range is between 0.0 and 1.0 for duty cycle
        if (value < 0.0f || value > 1.0f)
        {
            sendResponse("err - invalid duty cycle value %f", value);
            return;
        }

        driver.setManualMode(false); // Set manual mode to false
        driver.setDutyCycle(value);
        driver.setManualMode(true); // Set manual mode to true
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "r")
    {
        float value = extractValue(tokens[2].c_str());
        controller.setReference(value);
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "o")
    {
        int value = atoi(tokens[2].c_str());
        if (value == 0)
        {
            controller.setOccupancy(false);
        }
        else if (value == 1)
        {
            controller.setOccupancy(true);
        }
        else
        {
            sendResponse("err - invalid occupancy value %d", value);
            return;
        }
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "a")
    {
        int value = atoi(tokens[2].c_str());
        if (value == 0)
        {
            controller.setAntiWindup(false);
        }
        else if (value == 1)
        {
            controller.setAntiWindup(true);
        }
        else
        {
            sendResponse("err - invalid anti-windup value %d", value);
            return;
        }
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "f")
    {
        int value = atoi(tokens[2].c_str());
        if (value == 0)
        {
            controller.setFeedback(false);
        }
        else if (value == 1)
        {
            controller.setFeedback(true);
        }
        else
        {
            sendResponse("err - invalid feedback value %d", value);
            return;
        }
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "O")
    {
        float value = extractValue(tokens[2].c_str());

        // Check if the value is within the valid range
        // Assuming the valid range is above 0.0 for lower bound occupied
        if (value <= 0.0f)
        {
            sendResponse("err - invalid lower bound occupied value %f", value);
            return;
        }
        controller.setLowerBoundOccupied(value);
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "U")
    {
        float value = extractValue(tokens[2].c_str());
        // Check if the value is within the valid range
        // Assuming the valid range is above 0.0 for lower bound unoccupied
        if (value <= 0.0f)
        {
            sendResponse("err - invalid lower bound unoccupied value %f", value);
            return;
        }
        controller.setLowerBoundUnoccupied(value);
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "C")
    {
        // float value = extractValue(tokens[2].c_str());
        // desk.energyCost = value;
        sendResponse("ack- NOT DONE YET");
        return;
    }

    sendResponse("err");
    return;
}

void pcInterface::sendResponse(const char *format, ...)
{
    char buffer[BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, BUFFER_SIZE, format, args);
    va_end(args);
    Serial.print(buffer);
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
