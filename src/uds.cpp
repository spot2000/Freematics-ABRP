#include "uds.h"
#include "config.h"
#include "driver/twai.h"

namespace {
constexpr uint32_t kDefaultTimeoutMs = 200;
constexpr uint8_t kIsoTpSingleFrame = 0x0;
constexpr uint8_t kIsoTpFirstFrame = 0x1;
constexpr uint8_t kIsoTpConsecutiveFrame = 0x2;
constexpr uint8_t kIsoTpFlowControl = 0x3;

constexpr uint8_t kIsoTpPciMask = 0xF0;
constexpr uint8_t kIsoTpLenMask = 0x0F;
}

bool UdsClient::begin(uint32_t baud)
{
  if (m_started) {
    return true;
  }

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (baud == 250000) {
    t_config = TWAI_TIMING_CONFIG_250KBITS();
  } else if (baud == 1000000) {
    t_config = TWAI_TIMING_CONFIG_1MBITS();
  }

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }

  m_started = true;
  return true;
}

void UdsClient::end()
{
  if (!m_started) {
    return;
  }
  twai_stop();
  twai_driver_uninstall();
  m_started = false;
}

bool UdsClient::sendFrame(uint32_t id, bool extended, const uint8_t* data, uint8_t len, uint32_t timeoutMs)
{
  if (!m_started) {
    return false;
  }

  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = extended ? 1 : 0;
  msg.data_length_code = len;
  if (len > 0) {
    memcpy(msg.data, data, len);
  }

  return twai_transmit(&msg, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

bool UdsClient::readFrame(UdsFrame& frame, uint32_t timeoutMs)
{
  if (!m_started) {
    return false;
  }

  twai_message_t msg = {};
  if (twai_receive(&msg, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) {
    return false;
  }

  frame.id = msg.identifier;
  frame.extended = msg.extd != 0;
  frame.len = msg.data_length_code;
  if (frame.len > 0) {
    memcpy(frame.data, msg.data, frame.len);
  }
  return true;
}

bool UdsClient::sendIsoTp(uint32_t txId, bool txExtended, uint32_t rxId, bool rxExtended,
                          const uint8_t* payload, uint8_t payloadLen, uint32_t timeoutMs)
{
  if (payloadLen <= 7) {
    uint8_t frame[8] = {0};
    frame[0] = (kIsoTpSingleFrame << 4) | (payloadLen & kIsoTpLenMask);
    if (payloadLen > 0) {
      memcpy(frame + 1, payload, payloadLen);
    }
    return sendFrame(txId, txExtended, frame, 1 + payloadLen, timeoutMs);
  }

  uint8_t frame[8] = {0};
  frame[0] = (kIsoTpFirstFrame << 4) | ((payloadLen >> 8) & kIsoTpLenMask);
  frame[1] = payloadLen & 0xFF;
  memcpy(frame + 2, payload, 6);
  if (!sendFrame(txId, txExtended, frame, 8, timeoutMs)) {
    return false;
  }

  UdsFrame fc = {};
  if (!readFrame(fc, timeoutMs)) {
    return false;
  }
  if (fc.id != rxId || fc.extended != rxExtended || fc.len < 3) {
    return false;
  }
  if ((fc.data[0] >> 4) != kIsoTpFlowControl) {
    return false;
  }

  uint8_t blockSize = fc.data[1];
  uint8_t stMin = fc.data[2];
  uint8_t seq = 1;
  uint16_t offset = 6;
  uint8_t sentInBlock = 0;

  while (offset < payloadLen) {
    frame[0] = (kIsoTpConsecutiveFrame << 4) | (seq & kIsoTpLenMask);
    uint8_t copyLen = payloadLen - offset;
    if (copyLen > 7) {
      copyLen = 7;
    }
    memcpy(frame + 1, payload + offset, copyLen);
    if (!sendFrame(txId, txExtended, frame, copyLen + 1, timeoutMs)) {
      return false;
    }
    offset += copyLen;
    seq = (seq + 1) & 0x0F;

    if (stMin) {
      delay(stMin);
    }

    if (blockSize > 0) {
      sentInBlock++;
      if (sentInBlock >= blockSize && offset < payloadLen) {
        if (!readFrame(fc, timeoutMs)) {
          return false;
        }
        if (fc.id != rxId || fc.extended != rxExtended || (fc.data[0] >> 4) != kIsoTpFlowControl) {
          return false;
        }
        blockSize = fc.data[1];
        stMin = fc.data[2];
        sentInBlock = 0;
      }
    }
  }

  return true;
}

bool UdsClient::readIsoTp(uint32_t txId, bool txExtended, uint32_t rxId, bool rxExtended,
                          uint8_t* response, uint16_t* responseLen, uint32_t timeoutMs)
{
  UdsFrame frame = {};
  if (!readFrame(frame, timeoutMs)) {
    return false;
  }
  if (frame.id != rxId || frame.extended != rxExtended || frame.len == 0) {
    return false;
  }

  uint8_t frameType = frame.data[0] >> 4;
  if (frameType == kIsoTpSingleFrame) {
    uint8_t len = frame.data[0] & kIsoTpLenMask;
    if (len > 7 || len > frame.len - 1) {
      return false;
    }
    if (response && responseLen && *responseLen >= len) {
      memcpy(response, frame.data + 1, len);
      *responseLen = len;
      return true;
    }
    return false;
  }

  if (frameType != kIsoTpFirstFrame || frame.len < 8) {
    return false;
  }

  uint16_t totalLen = ((frame.data[0] & kIsoTpLenMask) << 8) | frame.data[1];
  if (totalLen == 0 || !response || !responseLen || *responseLen < totalLen) {
    return false;
  }

  uint16_t copied = 0;
  memcpy(response, frame.data + 2, 6);
  copied += 6;

  uint8_t fc[3] = {static_cast<uint8_t>((kIsoTpFlowControl << 4) | 0x0), 0, 0};
  if (!sendFrame(txId, txExtended, fc, sizeof(fc), timeoutMs)) {
    return false;
  }

  uint8_t seq = 1;
  while (copied < totalLen) {
    UdsFrame cf = {};
    if (!readFrame(cf, timeoutMs)) {
      return false;
    }
    if (cf.id != rxId || cf.extended != rxExtended || cf.len == 0) {
      continue;
    }
    if ((cf.data[0] >> 4) != kIsoTpConsecutiveFrame) {
      continue;
    }
    if ((cf.data[0] & kIsoTpLenMask) != (seq & kIsoTpLenMask)) {
      return false;
    }
    uint8_t copyLen = cf.len - 1;
    if (copied + copyLen > totalLen) {
      copyLen = totalLen - copied;
    }
    memcpy(response + copied, cf.data + 1, copyLen);
    copied += copyLen;
    seq = (seq + 1) & 0x0F;
  }

  *responseLen = totalLen;
  return true;
}

bool UdsClient::request(uint32_t txId, bool txExtended,
                        uint32_t rxId, bool rxExtended,
                        const uint8_t* payload, uint8_t payloadLen,
                        uint8_t* response, uint16_t* responseLen,
                        uint32_t timeoutMs)
{
  if (!m_started && !begin()) {
    return false;
  }

  if (!sendIsoTp(txId, txExtended, rxId, rxExtended, payload, payloadLen, timeoutMs)) {
    return false;
  }

  uint16_t len = responseLen ? *responseLen : 0;
  if (!readIsoTp(txId, txExtended, rxId, rxExtended, response, &len, timeoutMs)) {
    return false;
  }
  if (responseLen) {
    *responseLen = len;
  }
  return true;
}

bool UdsClient::writeDataByIdentifier(uint32_t txId, bool txExtended,
                                      uint32_t rxId, bool rxExtended,
                                      uint16_t did, const uint8_t* data,
                                      uint8_t dataLen,
                                      uint8_t* response, uint16_t* responseLen,
                                      uint32_t timeoutMs)
{
  uint8_t payload[32] = {0};
  if (dataLen + 3 > sizeof(payload)) {
    return false;
  }
  payload[0] = 0x2E;
  payload[1] = static_cast<uint8_t>(did >> 8);
  payload[2] = static_cast<uint8_t>(did & 0xFF);
  if (dataLen > 0) {
    memcpy(payload + 3, data, dataLen);
  }
  return request(txId, txExtended, rxId, rxExtended,
                 payload, static_cast<uint8_t>(dataLen + 3),
                 response, responseLen, timeoutMs);
}
