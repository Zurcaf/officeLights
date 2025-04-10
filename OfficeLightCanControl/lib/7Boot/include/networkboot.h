#ifndef NETWORKBOOT_H
#define NETWORKBOOT_H

#include <Arduino.h>
#include <Algorithm>
#include "mcp2515.h"

// The NetworkBoot class encapsulates the boot-up procedure.
class NetworkBoot {
public:
  // Enumeration of boot states.
  enum BootState {
    BOOT_INIT,
    BOOT_BROADCAST,
    BOOT_LISTEN,
    BOOT_COMPLETE
  };

  // Constructor: accepts the unique node ID.
  NetworkBoot();

  // Initializes Serial, the CAN-BUS, and starts the boot procedure.
  void begin();

  // Call this method repeatedly (e.g. in loop()) to run the boot sequence.
  void update();

  uint8_t myNodeId;
  uint8_t* getDiscoveredNodeIDs();
  int getNodeCount();
  bool isBootComplete();

private:
  BootState bootState;
  unsigned long bootStartTime;
  unsigned long lastBootSentTime;

  static const unsigned long BOOT_TIMEOUT = 15000;     // Boot listen period in milliseconds.
  static const unsigned long BOOT_SEND_INTERVAL = 5000;  // Interval between boot messages (ms).
  static const int MAX_NODES = 3;

  uint8_t bootedNodes[MAX_NODES];  // Array to store the IDs of other nodes.
  int nodeCount; // Number of nodes detected.

  bool bootComplete = false;

  // Private helper methods.
  void sendBootMessage();
  void checkForBootMessages();
  void printBootResults();
};

#endif  // NETWORKBOOT_H