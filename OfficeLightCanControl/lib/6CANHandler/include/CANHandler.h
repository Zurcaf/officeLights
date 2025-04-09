#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "mcp2515.h"
#include "pico/unique_id.h"
#include <cstring>
#include <cstdio>

class CANHandler {
public:
    // Constructor
    CANHandler(spi_inst_t *spi, uint8_t cs_pin, uint8_t tx_pin, uint8_t rx_pin, uint8_t sck_pin, uint32_t spi_clock_speed);
    
    // Initialize CAN controller
    bool begin(uint32_t bitrate = CAN_1000KBPS);
    
    // Send a message with composite ID
    bool sendMessage(uint8_t messageId, uint8_t deskId, const uint8_t *data, uint8_t length);
    
    // Check if a message is available
    bool available();
    
    // Read a received message and extract composite IDs
    bool readMessage(uint8_t *messageId, uint8_t *deskId, uint8_t *data, uint8_t *length);
    
    // Get the node address (short id)
    uint8_t getNodeAddress() const;
    
    // Set the transmission interval
    void setTransmitInterval(unsigned long interval);
    
    // Main processing function
    // void process();
    void printToSerial(const char *message);

    
private:
    MCP2515 m_can;
    pico_unique_board_id_t m_board_id;
    uint8_t m_node_address;
    unsigned long m_transmit_interval;
    unsigned long m_last_transmit_time;
    unsigned long m_counter;
    char m_print_buffer[100];
    
    // Combine message and desk IDs into 11-bit CAN ID
    uint16_t composeCanId(uint8_t messageId, uint8_t deskId) const;
    
    // Extract message and desk IDs from 11-bit CAN ID
    void decomposeCanId(uint16_t canId, uint8_t *messageId, uint8_t *deskId) const;
    
};

#endif // CAN_HANDLER_H