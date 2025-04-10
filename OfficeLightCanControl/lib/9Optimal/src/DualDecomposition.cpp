#include "DualDecomposition.h"
#include "CANHandler.h"
#include <SPI.h>
#include <pcInterface.h>
#include <NetworkBoot.h>


int nodeId = 0;  // To be set from user input
// Global Node variable. The Node class (from DualDecomposition.h) must include fields such as
// id, u_own, u_ant, l_own, u_other1, price_other1, u_other2, price_other2, etc.
Node localNode(0, 0, 0, 0, 0, 0, 0, 0, 0);

const int MAX_ITER = 50;
int current_iter = 0;
const float CONVERGENCE_THRESHOLD = 0.01;

// Check convergence: compares current u_own and dual variable l_own with a recomputed l value
bool checkConvergence(Node &node) {
  float delta_u = fabs(node.u_own - node.u_ant);
  float delta_l = fabs(node.l_own - node.compute_l());
  return (delta_u < CONVERGENCE_THRESHOLD && delta_l < CONVERGENCE_THRESHOLD);
}

// Packs u_own and l_own into a 5-byte payload.
// Message structure:
//   Bytes [0-1]: u_own (uint16_t, big-endian) scaled by 100
//   Bytes [2-3]: l_own (uint16_t, big-endian) scaled by 100
//   Byte 4: spare (set to 0)
void sendOptimizationMessage(Node &node) {
  uint8_t data[4];
  uint16_t u_val = (uint16_t)(node.u_own * 100);
  uint16_t l_val = (uint16_t)(node.l_own * 100);
  data[0] = (uint8_t)(u_val >> 8);
  data[1] = (uint8_t)(u_val & 0xFF);
  data[2] = (uint8_t)(l_val >> 8);
  data[3] = (uint8_t)(l_val & 0xFF);
  
  // Use a fixed messageId (e.g., 0x01) for optimization messages,
  // and set deskId as the node's ID.
  canHandler.sendMessage(MSG_OPTIMIZATION_VARIABLES, node.id, data, 4);
}

// Reads any available optimization messages from the CAN bus.
// Unpacks the message and updates the Node's received values accordingly.
void receiveOptimizationMessages(Node &node, int nodeCount) {
  uint8_t msgId, deskId, data[8], length;
  int message_count;
  while () {
    if (canHandler.readMessage(&msgId, &deskId, data, &length)) {
      // Process messages only from other nodes (deskId different from node.id)
      if (deskId != node.id) {
        uint16_t u_val = ((uint16_t)data[0] << 8) | data[1];
        uint16_t l_val = ((uint16_t)data[2] << 8) | data[3];
        float received_u = u_val / 100.0;
        float received_l = l_val / 100.0;
        // Update the corresponding fields based on sender deskId.
        if (deskId == 1 && node.id != 1) {
          node.u_other1 = received_u;
          node.price_other1 = received_l;
        } else if (deskId == 2 && node.id != 2) {
          node.u_other2 = received_u;
          node.price_other2 = received_l;
        } else if (deskId == 3 && node.id != 3) {
          node.u_other3 = received_u;
          node.price_other3 = received_l;
        }
        // Increment message_count for each received message.
        message_count++
        if(message_count == nodeCount) {
            // All messages received, reset message_count for next iteration
            message_count= 0;
            break;
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { } // Wait for serial connection
  delay(2000);
  
  Serial.println("Enter Node ID (1, 2, or 3):");
  while (Serial.available() == 0) { }
  String input = Serial.readStringUntil('\n');
  nodeId = input.toInt();
  
  // Initialize the Node based on the user input.
  if (nodeId == 1) {
    localNode = Node(1, 1, 2, 1, 1, 150, 30, 0.01, 0.005);
  } else if (nodeId == 2) {
    localNode = Node(2, 1, 2, 1, 1, 80, 0, 0.01, 0.005);
  } else if (nodeId == 3) {
    localNode = Node(3, 1, 2, 1, 1, 120, 20, 0.01, 0.005);
  } else {
    Serial.println("Invalid Node ID! Halting execution.");
    while (1);
  }
  Serial.print("Node ");
  Serial.print(localNode.id);
  Serial.println(" Initialized Successfully!");
  
  // Initialize the CAN bus via CANHandler.
  if (!canHandler.begin(CAN_1000KBPS)) {
    Serial.println("Failed to initialize CAN communication.");
    while (1);
  }
}

void loop() {
  if (current_iter < MAX_ITER) {
    // Compute new control variable u_own.
    localNode.u_own = localNode.compute_u();
    localNode.u_ant = localNode.u_own;
    
    // Send this node's current optimization message.
    sendOptimizationMessage(localNode);
    // Process any messages received from the CAN bus.
    receiveOptimizationMessages(localNode, NetworkBoot.nodeCount);
    
    // Update the dual variable l_own.
    localNode.l_own = localNode.compute_l();
    
    Serial.print("Iteration ");
    Serial.println(current_iter + 1);
    Serial.print("Node ");
    Serial.print(localNode.id);
    Serial.print(" u_own: ");
    Serial.println(localNode.u_own);
    Serial.print("l_own: ");
    Serial.println(localNode.l_own);
    
    if (checkConvergence(localNode)) {
      Serial.println("Convergence Reached!");
      while (1); // Halt execution upon convergence.
    }
    
    current_iter++;
    delay(100); // Brief delay between iterations.
  } else {
    Serial.println("Algorithm Complete.");
    while (1);
  }
}
