#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "mcp2515.h"
#include "pico/unique_id.h"
#include <cstring>  // Added for memcpy
#include <cstdio>   // For snprintf

class CANHandler {
public:
    // Constructor
    CANHandler(spi_inst_t *spi, uint8_t cs_pin, uint8_t tx_pin, uint8_t rx_pin, uint8_t sck_pin, uint32_t spi_clock_speed);
    
    // Initialize CAN controller
    bool begin(uint32_t bitrate = CAN_1000KBPS);
    
    // Send a message
    bool sendMessage(uint32_t id, const uint8_t *data, uint8_t length);
    
    // Check if a message is available
    bool available();
    
    // Read a received message
    bool readMessage(uint32_t *id, uint8_t *data, uint8_t *length);
    
    // Get the node address (short ID)
    uint8_t getNodeAddress() const;
    
    // Set the transmission interval
    void setTransmitInterval(unsigned long interval);
    
    // Main processing function (replaces loop functionality)
    void process();
    
private:
    MCP2515 m_can;
    pico_unique_board_id_t m_board_id;
    uint8_t m_node_address;
    unsigned long m_transmit_interval;
    unsigned long m_last_transmit_time;
    unsigned long m_counter;
    char m_print_buffer[100];
    
    void printToSerial(const char *message);
};

#endif // CAN_HANDLER_H