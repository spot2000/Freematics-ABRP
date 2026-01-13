#pragma once

#include <Arduino.h>

struct UdsFrame {
  uint32_t id = 0;
  bool extended = false;
  uint8_t len = 0;
  uint8_t data[8] = {0};
};

class UdsClient {
public:
  bool begin(uint32_t baud = 500000);
  void end();

  bool request(uint32_t txId, bool txExtended,
               uint32_t rxId, bool rxExtended,
               const uint8_t* payload, uint8_t payloadLen,
               uint8_t* response, uint16_t* responseLen,
               uint32_t timeoutMs = 200);

  bool writeDataByIdentifier(uint32_t txId, bool txExtended,
                             uint32_t rxId, bool rxExtended,
                             uint16_t did, const uint8_t* data,
                             uint8_t dataLen,
                             uint8_t* response, uint16_t* responseLen,
                             uint32_t timeoutMs = 200);

private:
  bool sendFrame(uint32_t id, bool extended, const uint8_t* data, uint8_t len, uint32_t timeoutMs = 50);
  bool readFrame(UdsFrame& frame, uint32_t timeoutMs);

  bool sendIsoTp(uint32_t txId, bool txExtended, uint32_t rxId, bool rxExtended,
                 const uint8_t* payload, uint8_t payloadLen, uint32_t timeoutMs);
  bool readIsoTp(uint32_t txId, bool txExtended, uint32_t rxId, bool rxExtended,
                 uint8_t* response, uint16_t* responseLen, uint32_t timeoutMs);

  bool m_started = false;
};
