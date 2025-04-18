#ifndef DISTRIBUTED_CONTROL_H
#define DISTRIBUTED_CONTROL_H

#include <Arduino.h>
#include "mcp2515.h"
#include "CANHandler.h"
#include "luxmeter.h"
#include <pcInterface.h>

#define MAX_NODES 3
#define PWM_RESOLUTION 4096
#define MAX_ITER 50


class DistributedLuminaire {
public:
    DistributedLuminaire(uint8_t id, uint8_t ledPin, uint8_t ldrPin, int numNodes, const uint8_t nodeIDs[], MCP2515& canDev);

    void setCouplingVector(float matrix[MAX_NODES]);
    void setReference();
    void setExternalIllum();

    void updateControl();
    void sendIntent();
    void receiveIntents();
    int getNodeIndex(uint8_t sender_id);
    int getIter();

    bool checkTurn();
    void startControl();

private:

    uint8_t nodeID;
    uint8_t ledPin;
    uint8_t ldrPin;
    int nodeIndex;
    int numNodes;
    uint8_t node_ids[MAX_NODES];
    int iter;
    int my_turn;
    MCP2515& can;

    float u[MAX_NODES];
    float k[MAX_NODES];
    float r;
    float d;

    void setLED(float duty);
};

#endif
