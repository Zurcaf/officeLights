#include "CANHandler.h"
#include "hardware/spi.h"
#include <Arduino.h>
#include <cstring>

// Bit masks and shifts for ID composition
constexpr uint8_t MESSAGE_ID_BITS = 6;
constexpr uint8_t DESK_ID_BITS = 5;
constexpr uint16_t MESSAGE_ID_MASK = 0x07C0; // 6 bits shifted left by 5
constexpr uint16_t DESK_ID_MASK = 0x001F;    // 5 bits

CANHandler::CANHandler(spi_inst_t *spi, uint8_t cs_pin, uint8_t tx_pin, 
                     uint8_t rx_pin, uint8_t sck_pin, uint32_t spi_clock_speed)
    : m_can(spi, cs_pin, tx_pin, rx_pin, sck_pin, spi_clock_speed),
      m_last_transmit_time(0),
      m_counter(0)
{
    pico_get_unique_board_id(&m_board_id);
    m_node_address = m_board_id.id[7];
}


bool CANHandler::begin(uint32_t bitrate) {
    m_can.reset();
    if (m_can.setBitrate(static_cast<CAN_SPEED>(bitrate)) != MCP2515::ERROR_OK) {
        printToSerial("Failed to set CAN bitrate\n");
        return false;
    }
    
    if (m_can.setNormalMode() != MCP2515::ERROR_OK) {
        printToSerial("Failed to set CAN normal mode\n");
        return false;
    }
    
    m_last_transmit_time = millis();
    return true;
}

uint16_t CANHandler::composeCanId(uint8_t messageId, uint8_t deskId) const {
    // Ensure IDs fit in their bit allocations
    messageId &= 0x3F; // Keep only 6 bits
    deskId &= 0x1F;    // Keep only 5 bits
    
    // Combine IDs (messageId in higher bits, deskId in lower)
    return (static_cast<uint16_t>(messageId) << DESK_ID_BITS| deskId);
}

void CANHandler::decomposeCanId(uint16_t canId, uint8_t *messageId, uint8_t *deskId) const {
    *messageId = (canId >> DESK_ID_BITS) & 0x3F;  // Shift right 5, then mask 6 bits
    *deskId = canId & DESK_ID_MASK;               // Mask lower 5 bits
}

bool CANHandler::sendMessage(uint8_t messageId, uint8_t deskId, const uint8_t *data, uint8_t length) {
    struct can_frame frame;
    frame.can_id = composeCanId(messageId, deskId);
    frame.can_dlc = length;
    memcpy(frame.data, data, length);
    
    MCP2515::ERROR err = m_can.sendMessage(&frame);
    if (err != MCP2515::ERROR_OK) {
        snprintf(m_print_buffer, sizeof(m_print_buffer), 
               "Failed to send message, error: %d\n", err);
        printToSerial(m_print_buffer);
        return false;
    }
    return true;
}

bool CANHandler::readMessage(uint8_t *messageId, uint8_t *deskId, uint8_t *data, uint8_t *length) {
    struct can_frame frame;
    MCP2515::ERROR err = m_can.readMessage(&frame);
    
    if (err == MCP2515::ERROR_OK)
    {
        decomposeCanId(frame.can_id, messageId, deskId);
        *length = frame.can_dlc;
        memcpy(data, frame.data, frame.can_dlc);
        return true;
    }
    
    return false;
}

bool CANHandler::available() {
    struct can_frame frame;
    return m_can.readMessage(&frame) == MCP2515::ERROR_OK;
}

uint8_t CANHandler::getNodeAddress() const {
    return m_node_address;
}



// void CANHandler::process() {
//     unsigned long current_time = millis();
    // int m_transmit_interval = 1000; // Example interval in milliseconds
    
//     // Handle transmission
//     if (current_time - m_last_transmit_time >= m_transmit_interval) {
//         uint8_t data[8];
//         unsigned long div = m_counter;
        
//         for (int i = 0; i < 8; i++) {
//             data[i] = '0' + (int)(div % 10);
//             div = div / 10;
//         }
        
//         if (sendMessage(m_node_address, data, 8)) {
//             snprintf(m_print_buffer, sizeof(m_print_buffer), 
//                     "Sending message %ld from node %x\n", 
//                     m_counter++, m_node_address);
//             printToSerial(m_print_buffer);
//         }
        
//         m_last_transmit_time = current_time;
//     }
    
//     // Handle reception
//     uint32_t received_id;
//     uint8_t received_data[8];
//     uint8_t received_length;
    
//     while (readMessage(&received_id, received_data, &received_length)) {
//         unsigned long rx_msg = 0;
//         unsigned long mult = 1;
        
//         for (int i = 0; i < received_length; i++) {
//             rx_msg += mult * (received_data[i] - '0');
//             mult *= 10;
//         }
        
//         snprintf(m_print_buffer, sizeof(m_print_buffer),
//                 "\t\t\t\tReceived message %ld from node %x\n", 
//                 rx_msg, received_id);
//         printToSerial(m_print_buffer);
//     }
// }

void CANHandler::printToSerial(const char *message) {
    // This needs to be implemented based on your serial library
    // For Pico SDK, you might use:
    // printf("%s", message);
    // Or if using Arduino framework:
    Serial.print(message);
}