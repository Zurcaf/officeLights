#include "pcInterface.h"

pcInterface::pcInterface(uint8_t deskId) : myDeskId(deskId), bufferIndex(0)
{
    // Initialize single desk state
    desk = {false, false, 0.0f, 0.0f, 0.0f, 0.0f, false, false, false,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f};

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

    sendResponse("Desk %d initialized", myDeskId); // Debug: Confirm initialization
}

void pcInterface::processSerial()
{
    if (Serial.available())
    {
        sendResponse("Serial data available"); // Debug: Confirm data is received
    }
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            commandBuffer[bufferIndex] = '\0';
            if (bufferIndex > 0)
            {
                sendResponse("Received: %s", commandBuffer); // Debug: Show received command
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

    Serial.printf("Command: %s\n", cmd);
    Serial.printf("Token 1: %s\n", tokens[0].c_str());
    Serial.printf("Token 2: %s\n", tokens[1].c_str());
    Serial.printf("Token 3: %s\n", tokens[2].c_str());
    Serial.printf("Token 4: %s\n", tokens[3].c_str());
    Serial.printf("Token 5: %s\n", tokens[4].c_str());

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
            sendResponse("u %d %.2f", ID, desk.dutyCycle);
        }
        else if (tokens[2] == "y")
        {
            sendResponse("y %d %.2f", ID, desk.measuredIlluminance);
        }
        else
        {
            sendResponse("err - unknown get buffer command", tokens[3]);
        }
    }

    if (tokens.size() != 3)
    {
        sendResponse("err - invalid get command size");
        return;
    }

    //
    ID = atoi(tokens[2].c_str());
    if (isNotValidID(ID))
    {
        sendResponse("err - invalid desk ID %d", ID);
        return;
    }

    if (tokens[1] == "u")
    {
        sendResponse("u %d %.2f", myDeskId, desk.dutyCycle);
    }
    else if (tokens[1] == "r")
    {
        sendResponse("r %d %.2f", myDeskId, desk.illuminanceRef);
    }
    else if (tokens[1] == "y")
    {
        sendResponse("y %d %.2f", myDeskId, desk.measuredIlluminance);
    }
    else if (tokens[1] == "v")
    {
        sendResponse("v %d %.2f", myDeskId, desk.ldrVoltage);
    }
    else if (tokens[1] == "o")
    {
        sendResponse("o %d %d", myDeskId, desk.occupancy);
    }
    else if (tokens[1] == "a")
    {
        sendResponse("a %d %d", myDeskId, desk.antiWindup);
    }
    else if (tokens[1] == "f")
    {
        sendResponse("f %d %d", myDeskId, desk.feedbackControl);
    }
    else if (tokens[1] == "d")
    {
        sendResponse("d %d %.2f", myDeskId, desk.externalIlluminance);
    }
    else if (tokens[1] == "p")
    {
        sendResponse("p %d %.2f", myDeskId, desk.powerConsumption);
    }
    else if (tokens[1] == "t")
    {
        sendResponse("t %d %.2f", myDeskId, desk.elapsedTime);
    }
    else if (tokens[1] == "E")
    {
        sendResponse("E %d %.2f", myDeskId, desk.avgEnergy);
    }
    else if (tokens[1] == "V")
    {
        sendResponse("V %d %.2f", myDeskId, desk.avgVisibilityError);
    }
    else if (tokens[1] == "F")
    {
        sendResponse("F %d %.2f", myDeskId, desk.avgFlickerError);
    }
    else if (tokens[1] == "O")
    {
        sendResponse("O %d %.2f", myDeskId, desk.occupiedLowerBound);
    }
    else if (tokens[1] == "U")
    {
        sendResponse("U %d %.2f", myDeskId, desk.unoccupiedLowerBound);
    }
    else if (tokens[1] == "L")
    {
        sendResponse("L %d %.2f", myDeskId, desk.currentLowerBound);
    }
    else if (tokens[1] == "C")
    {
        sendResponse("C %d %.2f", myDeskId, desk.energyCost);
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
            desk.streaming_y = true;
            sendResponse("s %d y %.2f time_in_millis!!", ID, desk.measuredIlluminance); //should be every 10 millis
        }
        else if (tokens[1] == "u")
        {
            desk.streaming_u = true;
            sendResponse("s %d u %.2f time_in_millis!!", ID, desk.dutyCycle);
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
            desk.streaming_y = false;
            sendResponse("ack");
        }
        else if (tokens[1] == "u")
        {
            desk.streaming_u = false;
            sendResponse("ack");
        }
        else
        {
            sendResponse("err");
        }
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
        desk.dutyCycle = value;
        sendResponse("ack");
        return;
    }

    if (tokens[0] == "r")
    {
        float value = extractValue(tokens[2].c_str());
        desk.illuminanceRef = value;
        sendResponse("ack");
        return;
    }

    if (tokens[0] == "o")
    {
        int value = atoi(tokens[2].c_str());
        desk.occupancy = value;
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "a")
    {
        int value = atoi(tokens[2].c_str());
        desk.antiWindup = value;
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "f")
    {
        int value = atoi(tokens[2].c_str());
        desk.feedbackControl = value;
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "O")
    {
        float value = extractValue(tokens[2].c_str());
        desk.occupiedLowerBound = value;
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "U")
    {
        float value = extractValue(tokens[2].c_str());
        desk.unoccupiedLowerBound = value;
        sendResponse("ack");
        return;
    }
    if (tokens[0] == "C")
    {
        float value = extractValue(tokens[2].c_str());
        desk.energyCost = value;
        sendResponse("ack");
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
    Serial.println(buffer);
    Serial.flush(); // Ensure the response is sent immediately
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
    desk = {false, false, 0.0f, 0.0f, 0.0f, 0.0f, false, false, false,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f};
}