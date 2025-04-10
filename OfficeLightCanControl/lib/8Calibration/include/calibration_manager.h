#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include <Arduino.h>
#include "mcp2515.h"
#include "luxmeter.h"

#define MAX_NODES 3  // Adjust as needed

// === Message Types ===
enum MsgType {
    REQUEST_TURN = 1,
    ACK,
    LOW_LIGHT,
    HIGH_LIGHT,
    DONE_LIGHT,
    CALIBRATION_DONE
};

// === Calibration States ===
enum CalibrationState {
    IDLE,
    WAIT_ACK,
    FIRST_LIGHT,
    WAIT_HIGH,
    SECOND_LIGHT,
    WAIT_DONE
};

class CalibrationManager {
public:
    // Constructor
    CalibrationManager(int id, const uint8_t* ids, int count, unsigned long measure_time);
    const float* getAllGains() const;  // Get pointer to the entire gain row
    float getOwnGain() const;          // Get this node's self gain (K_ii)
    float getOffset() const;

    // API
    void startCalibration();
    bool isCalibrationComplete() const;
    void printGains() const;

private:
    // Internal helpers
    void receiveCANMessages();
    void sendCANMessage(int type);
    void sendAck(uint8_t target_id);
    void requestTurn();
    void notifyLow();
    void notifyHigh();
    void notifyDone();
    void moveToNextNode();

    // Hardware-specific functions you must define elsewhere
    float readLux();
    void setPWM(float duty);

    // Member variables
    uint8_t node_ids[MAX_NODES];
    float K_row[MAX_NODES];

    uint8_t node_id;
    int my_index = -1;
    int total_nodes;
    int current_turn_index = 0;
    int current_turn;
    int acks_received = 0;
    int current_light_id = -1;
    float offset = 0;

    float ml = 0.0;  // measured lux at low PWM
    float mh = 0.0;  // measured lux at high PWM

    unsigned long measure_start_time = 0;
    unsigned long measurement_time = 500;
    unsigned long lastRequestSentTime = 0;
    const unsigned long REQUEST_RESEND_INTERVAL = 500; // ms


    CalibrationState calibrationState = IDLE;
    bool calibration_complete = false;
};

#endif // CALIBRATION_MANAGER_H
