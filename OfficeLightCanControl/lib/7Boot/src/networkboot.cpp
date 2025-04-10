#include "networkboot.h"

// The MCP2515 CAN controller is assumed to be defined globally.
extern MCP2515 canHandler;

NetworkBoot::NetworkBoot()
  : bootState(BOOT_INIT),
    bootStartTime(0),
    lastBootSentTime(0),
    nodeCount(0)
    
{bootedNodes[nodeCount++] = myNodeId;}

void NetworkBoot::begin() {
  // Start the boot procedure.
  bootState = BOOT_BROADCAST;
  bootStartTime = millis();
  lastBootSentTime = 0;
}

void NetworkBoot::update() {
  unsigned long currentTime = millis();

  switch (bootState) {
    case BOOT_BROADCAST:
      // Send a boot message immediately, then transition to listen state.
      if (currentTime - lastBootSentTime >= BOOT_SEND_INTERVAL) {
        sendBootMessage();
        lastBootSentTime = currentTime;
      }
      bootState = BOOT_LISTEN;
      break;

    case BOOT_LISTEN:
      // Continue sending boot messages periodically.
      if (currentTime - lastBootSentTime >= BOOT_SEND_INTERVAL) {
        sendBootMessage();
        lastBootSentTime = currentTime;
      }
      // Listen for incoming boot messages.
      checkForBootMessages();
      // If the boot timeout has elapsed, finish booting.
      if (currentTime - bootStartTime >= BOOT_TIMEOUT) {
        bootState = BOOT_COMPLETE;
        printBootResults();
      }
      break;

    case BOOT_COMPLETE:
      bootComplete = true; 
      break;

    default:
      break;
  }
}
bool NetworkBoot::isBootComplete(){
  return bootComplete;
}

uint8_t* NetworkBoot::getDiscoveredNodeIDs() {
    std::sort(bootedNodes, bootedNodes + nodeCount);
    return bootedNodes;
}

int NetworkBoot::getNodeCount() {
    return nodeCount;
}

void NetworkBoot::sendBootMessage() {
  struct can_frame msg;
  msg.can_id = 0x100;  // Identifier for boot messages.
  msg.can_dlc = 1;     // One-byte payload.
  msg.data[0] = myNodeId;
  if (canHandler.sendMessage(&msg) == MCP2515::ERROR_OK) {
    Serial.println("Boot message sent.");
  } else {
    Serial.println("Error sending boot message.");
  }
}

void NetworkBoot::checkForBootMessages() {
  struct can_frame msg;
  if (canHandler.readMessage(&msg) == MCP2515::ERROR_OK) {
    // Check if the message is a boot message (identifier 0x100).
    if ((msg.can_id & 0x7FF) == 0x100 && msg.can_dlc >= 1) {
      uint8_t receivedID = msg.data[0];
      Serial.print("Received boot message from node: ");
      Serial.println(receivedID);
      // Ignore our own boot message.
      if (receivedID != myNodeId) {
        bool alreadyPresent = false;
        for (int i = 0; i < nodeCount; i++) {
          if (bootedNodes[i] == receivedID) {
            alreadyPresent = true;
            break;
          }
        }
        if (!alreadyPresent && nodeCount < MAX_NODES) {
          bootedNodes[nodeCount++] = receivedID;
          Serial.print("Added node ");
          Serial.println(receivedID);
        }
      }
    }
  }
}

void NetworkBoot::printBootResults() {
  Serial.println("Boot complete. Active nodes:");
  Serial.print("My Node ID: ");
  Serial.println(myNodeId);
  Serial.print("Other nodes detected: ");
  for (int i = 0; i < nodeCount; i++) {
    Serial.print(bootedNodes[i]);
    Serial.print(" ");
  }
  Serial.println();
}