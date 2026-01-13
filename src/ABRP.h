#pragma once

#include <Arduino.h>
#include <FS.h>
#include <FreematicsPlus.h>
#include "uds.h"

constexpr size_t ABRP_MAX_SIGNALS = 32;
constexpr size_t ABRP_MAX_REQUEST_BYTES = 24;

enum AbrpField : uint8_t {
  ABRP_FIELD_UTC = 0,
  ABRP_FIELD_SOC,
  ABRP_FIELD_POWER,
  ABRP_FIELD_SPEED,
  ABRP_FIELD_LAT,
  ABRP_FIELD_LON,
  ABRP_FIELD_IS_CHARGING,
  ABRP_FIELD_IS_DCFC,
  ABRP_FIELD_IS_PARKED,
  ABRP_FIELD_CAPACITY,
  ABRP_FIELD_KWH_CHARGED,
  ABRP_FIELD_SOH,
  ABRP_FIELD_HEADING,
  ABRP_FIELD_ELEVATION,
  ABRP_FIELD_EXT_TEMP,
  ABRP_FIELD_BATT_TEMP,
  ABRP_FIELD_VOLTAGE,
  ABRP_FIELD_CURRENT,
  ABRP_FIELD_ODOMETER,
  ABRP_FIELD_EST_BATTERY_RANGE,
  ABRP_FIELD_COUNT
};

struct AbrpSignalConfig {
  AbrpField field = ABRP_FIELD_COUNT;
  char name[24] = {0};
  char unit[8] = {0};
  uint32_t txId = 0;
  uint32_t rxId = 0;
  bool txExtended = false;
  bool rxExtended = false;
  uint8_t request[ABRP_MAX_REQUEST_BYTES] = {0};
  uint8_t requestLength = 0;
  uint8_t startByte = 0;
  uint8_t length = 0;
  int8_t bit = -1;
  float scale = 1.0f;
  float offset = 0.0f;
};

struct AbrpConfig {
  bool saveJsonLog = true;
  uint16_t sendIntervalSec = 1;
  char userToken[96] = {0};
  size_t signalCount = 0;
  AbrpSignalConfig signals[ABRP_MAX_SIGNALS];
};

class AbrpJsonLogger {
public:
  bool begin(uint32_t fileId);
  void end();
  void writeLine(const char* line);
  void flush();
  bool isOpen() const { return m_file; }

private:
  File m_file;
};

class AbrpManager {
public:
  void begin(const AbrpConfig& config);
  void setStorageReady(uint32_t fileId);
  void updateGps(const GPS_DATA* gps);
  void updateUtc();
  void pollUds(uint32_t nowMs);
  void logJson(uint32_t nowMs);
  void setEnabled(bool enabled) { m_enabled = enabled; }

private:
  void applyDerivedValues();
  bool decodeSignal(const AbrpSignalConfig& signal, float& outValue);
  void setField(AbrpField field, float value);
  bool isFieldValid(AbrpField field) const;
  float getField(AbrpField field) const;
  const char* fieldName(AbrpField field) const;

  bool m_enabled = true;
  AbrpConfig m_config = {};
  UdsClient m_uds;
  AbrpJsonLogger m_logger;
  uint32_t m_lastPollMs = 0;
  uint32_t m_lastLogMs = 0;

  bool m_valid[ABRP_FIELD_COUNT] = {false};
  float m_values[ABRP_FIELD_COUNT] = {0.0f};
};
