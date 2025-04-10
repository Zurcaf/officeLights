#include "distributed_control.h"

DistributedLuminaire::DistributedLuminaire(uint8_t id, uint8_t ledPin, uint8_t ldrPin, int numNodes, MCP2515& canDev)
    : nodeID(id), ledPin(ledPin), ldrPin(ldrPin), numNodes(numNodes), can(canDev), iter(0) {
    pinMode(ledPin, OUTPUT);
    analogWrite(ledPin, 0);
    for (int i = 0; i < MAX_NODES; ++i) {
        u[i] = 0.0;
    }

    nodeIndex = getNodeIndex(nodeID)
    my_turn = nodeIndex;
}

void DistributedLuminaire::setCouplingVector(float vector[MAX_NODES]) {
    memcpy(k, vector, sizeof(k));
}

void DistributedLuminaire::setReference(float ref) {
    r = ref;
}

void DistributedLuminaire::setExternalIllum() {
    float l = readLUX();
    float sum=0;
    for(int i; i<numNodes; i++){
        sum += k[i]*u[i];
    }
    d = l - sum;
    if(d < 0){d=0;}
    return;
}

int CalibrationManager::getNodeIndex(uint8_t sender_id) {
    for (int i = 0; i < numNodes; ++i) {
        if (node_ids[i] == sender_id) {
            return i;
        }
    }
    return -1; // Not found
}

void DistributedLuminaire::updateControl() {
    float sum_k_neighbors = 0.0;
    for (int j = 0; j < numNodes; ++j) {
        if (j != nodeIndex) {
            sum_k_neighbors += k[j] * u[j];
        }
    }

    float u_new = (r - d - sum_k_neighbors) / k[nodeIndex];
    u[nodeIndex] = constrain(u_new, 0.0, 100.0);
    //setLED(u[nodeIndex]);
}

uint16_t DistributedLuminaire::calcChecksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i)
        sum += data[i];
    return sum;
}


void DistributedLuminaire::sendIntent() {
    DimmingIntentMsg msg;
    msg.dimming = u[nodeIndex];
    msg.checksum = calcChecksum(reinterpret_cast<uint8_t*>(&msg), sizeof(float)); // just over dimming

    canHandler.sendMessage(
        CONTROL_INTENT_MSG,       // your defined message type
        nodeID,                   // sender ID
        reinterpret_cast<uint8_t*>(&msg),
        sizeof(DimmingIntentMsg)
    );
}


void DistributedLuminaire::receiveIntents() {
    uint8_t messageId, senderId, length;
    uint8_t buffer[sizeof(DimmingIntentMsg)];

    while (canHandler.readMessage(&messageId, &senderId, buffer, &length)) {
        if (messageId == CONTROL_INTENT_MSG && length == sizeof(DimmingIntentMsg)) {
            DimmingIntentMsg msg;
            memcpy(&msg, buffer, sizeof(msg));

            uint16_t expected = calcChecksum(reinterpret_cast<uint8_t*>(&msg), sizeof(float));

            if (msg.checksum != expected) {
                Serial.println("Checksum mismatch — dropping message.");
                continue;
            }

            if (!isfinite(msg.dimming)) {
                Serial.println("Invalid dimming value (NaN/Inf) — ignoring.");
                continue;
            }

            float safeValue = constrain(msg.dimming, 0.0f, 100.0f);
            int idx = getNodeIndex(senderId);
            if (idx >= 0 && idx < numNodes) {
                u[idx] = safeValue;
            }
        }
    }
}



float DistributedLuminaire::getLUX() {
    int read_adc = analogRead(ldrPin);
    float v = (read_adc * 3.3) / 4096;
    float Rl = ((10000 * 3.3) / v) - 10000;
    float lux = pow(10, (log10(Rl) - 6.2) / (-0.9));
    return lux;
}

void DistributedLuminaire::setLED(float duty) {
    float u = duty * 4096;
    int pwm = static_cast<int>(u);
    analogWrite(ledPin, pwm);
}

int DistributedLuminaire::getIter(){
    return iter;
}

bool checkTurn(){
    if(my_turn<0 || my_turn > numNodes-1){
        Serial.print("Something went wrong");
        return false;
    }

    if(my_turn == 0){
        my_turn = numNodes-1;
        iter++;
        return true;
    }   
    else{
        my_turn = my_turn -1;
        iter++;
        return false;
    }
}

void startControl(){
    while(iter <= MAX_ITER){
        if(checkTurn()){
            updateControl();
            sendIntent();
        }
        else{
            receiveIntents();
        }
    }
    return;
}
