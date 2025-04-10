#include "calibration_manager.h"

// The MCP2515 CAN controller is assumed to be defined globally.
extern MCP2515 canHandler;

CalibrationManager::CalibrationManager(int id, const uint8_t* ids, int count, unsigned long measure_time)
    : node_id(id), total_nodes(count), measurement_time(measure_time) {

    Serial.println("[CalibrationManager] Constructor called.");

    for (int i = 0; i < total_nodes; ++i) {
        node_ids[i] = ids[i];
        K_row[i] = 0.0f;
        if (node_ids[i] == node_id) {
            my_index = i;
        }
    }

    current_turn = node_ids[current_turn_index];
    Serial.print("[CalibrationManager] Node ID: ");
    Serial.println(node_id);
}

void CalibrationManager::sendCANMessage(int type) {
    struct can_frame canMsgTx;
    canMsgTx.can_dlc = 0;
    canMsgTx.can_id = (type << 4) | (node_id & 0x0F);

    MCP2515::ERROR err = canHandler.sendMessage(&canMsgTx);

    if (err != MCP2515::ERROR_OK) {
        Serial.println("[CAN] Transmission failed!");
    } else {
        Serial.print("[CAN] Sent message: type=");
        Serial.print(type);
        Serial.print(", from node=");
        Serial.println(node_id);
    }
}

void CalibrationManager::requestTurn() {
    Serial.println("[Calibration] Requesting turn...");
    sendCANMessage(REQUEST_TURN);
    acks_received = 0;
}

void CalibrationManager::sendAck(uint8_t target_id) {
    struct can_frame canMsgTx;
    canMsgTx.can_dlc = 1;
    canMsgTx.data[0] = target_id;
    canMsgTx.can_id = (ACK << 4) | (node_id & 0x0F);

    MCP2515::ERROR err = canHandler.sendMessage(&canMsgTx);

    if (err != MCP2515::ERROR_OK) {
        Serial.println("[CAN] ACK transmission failed!");
    } else {
        Serial.print("[CAN] Sent ACK to ");
        Serial.println(target_id);
    }
}

void CalibrationManager::notifyLow() {
    Serial.println("[Calibration] Notifying LOW_LIGHT");
    sendCANMessage(LOW_LIGHT);
}

void CalibrationManager::notifyHigh() {
    Serial.println("[Calibration] Notifying HIGH_LIGHT");
    sendCANMessage(HIGH_LIGHT);
}

void CalibrationManager::notifyDone() {
    Serial.println("[Calibration] Notifying DONE_LIGHT");
    sendCANMessage(DONE_LIGHT);
}

void CalibrationManager::moveToNextNode() {
    current_turn_index++;
    if (current_turn_index < total_nodes) {
        current_turn = node_ids[current_turn_index];
        Serial.print("[Calibration] Moving to next node: ");
        Serial.println(current_turn);
    } else {
        sendCANMessage(CALIBRATION_DONE);
        calibration_complete = true;
        Serial.println("[Calibration] Calibration complete (I was the last node).");
    }
}

float CalibrationManager::readLux() {
    int read_adc = analogRead(A0);
    float v = (read_adc * 3.3) / 4096;
    float Rl = ((10000 * 3.3) / v) - 10000;
    float lux = pow(10, (log10(Rl) - 6.2) / (-0.9));
    Serial.print("[Sensor] Read lux: ");
    Serial.println(lux);
    return lux;
}

void CalibrationManager::setPWM(float duty) {
    float u = duty * 4096;
    int pwm = static_cast<int>(u);
    analogWrite(15, pwm);
    Serial.print("[PWM] Set duty: ");
    Serial.print(duty * 100);
    Serial.println(" %");
}

void CalibrationManager::receiveCANMessages() {
    MCP2515::ERROR err;
    struct can_frame canMsgRx;

    while ((err = canHandler.readMessage(&canMsgRx)) == MCP2515::ERROR_OK) {
        uint8_t type = (canMsgRx.can_id >> 4) & 0x0F;
        uint8_t sender_id = canMsgRx.can_id & 0x0F;

        Serial.print("[CAN] Received type=");
        Serial.print(type);
        Serial.print(" from sender=");
        Serial.println(sender_id);

        switch (type) {
            case REQUEST_TURN:
                sendAck(sender_id);
                current_light_id = sender_id;
                break;

            case ACK:
                if (canMsgRx.data[0] == node_id) {
                    acks_received++;
                    Serial.print("[ACK] Received ACK. Total now: ");
                    Serial.println(acks_received);
                }
                break;

            case LOW_LIGHT:
                ml = readLux();
                sendAck(sender_id);
                break;

            case HIGH_LIGHT:
                mh = readLux();
                sendAck(sender_id);
                break;

            case DONE_LIGHT:
                K_row[current_turn_index] = (mh - ml) / 0.6;
                Serial.print("[GAIN] K[");
                Serial.print(node_id);
                Serial.print("][");
                Serial.print(current_turn);
                Serial.print("] = ");
                Serial.println(K_row[current_turn_index], 3);

                ml = 0;
                mh = 0;
                current_light_id = -1;
                moveToNextNode();
                calibrationState = IDLE;
                break;

            case CALIBRATION_DONE:
                calibration_complete = true;
                Serial.println("[Calibration] Calibration complete (received from another node).");
                break;

            default:
                Serial.println("[CAN] Unknown message type.");
                break;
        }
    }
}

void CalibrationManager::startCalibration() {
    receiveCANMessages();

    switch (calibrationState) {
        case IDLE:
            if (node_id == current_turn) {
                Serial.println("[FSM] State: IDLE → Requesting turn");
                requestTurn();
                calibrationState = WAIT_ACK;
            }
            break;

        case WAIT_ACK:
            // Resend request every interval if ACKs not received
            if (millis() - lastRequestSentTime >= REQUEST_RESEND_INTERVAL) {
                Serial.println("[FSM] Resending REQUEST_TURN");
                requestTurn();
                lastRequestSentTime = millis();
            }

            if (acks_received == total_nodes - 1) {
                Serial.println("[FSM] All ACKs received. Starting FIRST_LIGHT");
                setPWM(0.2);
                measure_start_time = millis();
                acks_received = 0;
                calibrationState = FIRST_LIGHT;
            }
            break;

        case FIRST_LIGHT:
            if (millis() - measure_start_time >= measurement_time) {
                Serial.println("[FSM] Finished FIRST_LIGHT");
                ml = readLux();
                notifyLow();
                calibrationState = WAIT_HIGH;
            }
            break;

        case WAIT_HIGH:
            if (acks_received == total_nodes - 1) {
                Serial.println("[FSM] All ACKs received. Starting SECOND_LIGHT");
                setPWM(0.8);
                measure_start_time = millis();
                acks_received = 0;
                calibrationState = SECOND_LIGHT;
            }
            break;

        case SECOND_LIGHT:
            if (millis() - measure_start_time >= measurement_time) {
                Serial.println("[FSM] Finished SECOND_LIGHT");
                mh = readLux();
                notifyHigh();
                calibrationState = WAIT_DONE;
            }
            break;

        case WAIT_DONE:
            if (acks_received == total_nodes - 1) {
                Serial.println("[FSM] WAIT_DONE complete. Finalizing calibration.");
                K_row[current_turn_index] = (mh - ml) / 0.6;
                float g = (mh - ml) / 0.6;
                offset = mh - g;
                notifyDone();
                ml = 0;
                mh = 0;
                setPWM(0);
                moveToNextNode();
                calibrationState = IDLE;
            }
            break;
    }
}

bool CalibrationManager::isCalibrationComplete() const {
    return calibration_complete;
}

void CalibrationManager::printGains() const {
    Serial.print("[Calibration] Final Gains for node ");
    Serial.print(node_id);
    Serial.print(": ");
    for (int i = 0; i < total_nodes; ++i) {
        Serial.print(K_row[i], 3);
        Serial.print("\t");
    }
    Serial.println();
}


const float* CalibrationManager::getAllGains() const {
    return K_row;
}

float CalibrationManager::getOwnGain() const {
    return K_row[my_index];
}

float CalibrationManager::getOffset() const {
    return offset;
}
