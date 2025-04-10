#include "distributed_control.h"

extern LuxMeter luxMeter;
extern CANHandler canHandler;

DistributedLuminaire::DistributedLuminaire(uint8_t id, uint8_t ledPin, uint8_t ldrPin, int numNodes, const uint8_t nodeIDs[], MCP2515& canDev)
    : nodeID(id), ledPin(ledPin), ldrPin(ldrPin), numNodes(numNodes), can(canDev), iter(0) {
    pinMode(ledPin, OUTPUT);
    analogWrite(ledPin, 0);
    for (int i = 0; i < MAX_NODES; ++i) {
        u[i] = 0.0;
    }
    memcpy(node_ids, nodeIDs, numNodes * sizeof(uint8_t));

    nodeIndex = getNodeIndex(nodeID);
    my_turn = nodeIndex;
}

void DistributedLuminaire::setCouplingVector(float vector[MAX_NODES]) {
    memcpy(k, vector, sizeof(k));
}

void DistributedLuminaire::setReference(float ref) {
    r = ref;
}

void DistributedLuminaire::setExternalIllum() {
    float l = luxMeter.getLuxValue();
    float sum=0;
    for(int i=0; i<numNodes; i++){
        sum += k[i]*u[i];
    }
    d = l - sum;
    if(d < 0){d=0;}
    return;
}

int DistributedLuminaire::getNodeIndex(uint8_t sender_id) {
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
    u[nodeIndex] = constrain(u_new, 0.0f, 1.0f);
    //setLED(u[nodeIndex]);
}


void DistributedLuminaire::sendIntent() {
    float dimming = u[nodeIndex];

    canHandler.sendMessage(
        CONTROL_INTENT_MSG,
        nodeID,
        reinterpret_cast<uint8_t*>(&dimming),
        sizeof(float)
    );
}



void DistributedLuminaire::receiveIntents() {
    uint8_t messageId, senderId, length;
    uint8_t buffer[sizeof(float)];

    while (canHandler.readMessage(&messageId, &senderId, buffer, &length)) {
        if (messageId == CONTROL_INTENT_MSG && length == sizeof(float)) {
            float dimming;
            memcpy(&dimming, buffer, sizeof(float));

            if (!isfinite(dimming)) {
                Serial.println("Invalid dimming value (NaN/Inf) — ignoring.");
                continue;
            }

            float safeValue = constrain(dimming, 0.0f, 1.0f);
            int idx = getNodeIndex(senderId);
            if (idx >= 0 && idx < numNodes) {
                u[idx] = safeValue;
            }
        }
    }
}


void DistributedLuminaire::setLED(float duty) {
    float u = duty * 4095.0f;
    int pwm = static_cast<int>(u);
    pwm = constrain(pwm, 0, 4095);
    analogWrite(ledPin, pwm);
}

int DistributedLuminaire::getIter(){
    return iter;
}

bool DistributedLuminaire::checkTurn(){
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

void DistributedLuminaire::startControl(){
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
