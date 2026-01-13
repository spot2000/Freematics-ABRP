/*
Functions to link data to ABRP
*/

#include <FreematicsPlus.h>
#include "ABRP.h"
#include "config.h"
#include <SD.h>
#include <SPIFFS.h>
#include <time.h>

namespace {
constexpr uint32_t kJsonFlushIntervalMs = 5000;

struct FieldMeta {
  AbrpField field;
  const char* name;
};

constexpr FieldMeta kFieldMeta[] = {
  {ABRP_FIELD_UTC, "utc"},
  {ABRP_FIELD_SOC, "soc"},
  {ABRP_FIELD_POWER, "power"},
  {ABRP_FIELD_SPEED, "speed"},
  {ABRP_FIELD_LAT, "lat"},
  {ABRP_FIELD_LON, "lon"},
  {ABRP_FIELD_IS_CHARGING, "is_charging"},
  {ABRP_FIELD_IS_DCFC, "is_dcfc"},
  {ABRP_FIELD_IS_PARKED, "is_parked"},
  {ABRP_FIELD_CAPACITY, "capacity"},
  {ABRP_FIELD_KWH_CHARGED, "kwh_charged"},
  {ABRP_FIELD_SOH, "soh"},
  {ABRP_FIELD_HEADING, "heading"},
  {ABRP_FIELD_ELEVATION, "elevation"},
  {ABRP_FIELD_EXT_TEMP, "ext_temp"},
  {ABRP_FIELD_BATT_TEMP, "batt_temp"},
  {ABRP_FIELD_VOLTAGE, "voltage"},
  {ABRP_FIELD_CURRENT, "current"},
  {ABRP_FIELD_ODOMETER, "odometer"},
  {ABRP_FIELD_EST_BATTERY_RANGE, "est_battery_range"},
};

const char* fieldName(AbrpField field)
{
  for (const auto& meta : kFieldMeta) {
    if (meta.field == field) {
      return meta.name;
    }
  }
  return "";
}

bool appendJsonField(char* buffer, size_t bufferSize, size_t& offset, const char* key, float value, bool isInteger)
{
  if (offset >= bufferSize) {
    return false;
  }
  int written = 0;
  if (offset > 1) {
    written = snprintf(buffer + offset, bufferSize - offset, ",\"%s\":", key);
  } else {
    written = snprintf(buffer + offset, bufferSize - offset, "\"%s\":", key);
  }
  if (written < 0 || static_cast<size_t>(written) >= bufferSize - offset) {
    return false;
  }
  offset += static_cast<size_t>(written);

  if (isInteger) {
    written = snprintf(buffer + offset, bufferSize - offset, "%d", static_cast<int>(value));
  } else {
    written = snprintf(buffer + offset, bufferSize - offset, "%.3f", value);
  }
  if (written < 0 || static_cast<size_t>(written) >= bufferSize - offset) {
    return false;
  }
  offset += static_cast<size_t>(written);
  return true;
}

bool isBoolField(AbrpField field)
{
  return field == ABRP_FIELD_IS_CHARGING || field == ABRP_FIELD_IS_DCFC || field == ABRP_FIELD_IS_PARKED;
}
}

bool AbrpJsonLogger::begin(uint32_t fileId)
{
#if STORAGE == STORAGE_NONE
  (void)fileId;
  return false;
#else
  char path[32] = {0};
#if STORAGE == STORAGE_SPIFFS
  snprintf(path, sizeof(path), "/ABRP-%u.json", fileId);
  m_file = SPIFFS.open(path, FILE_APPEND);
#else
  snprintf(path, sizeof(path), "/DATA/ABRP-%u.json", fileId);
  m_file = SD.open(path, FILE_APPEND);
#endif
  return m_file;
#endif
}

void AbrpJsonLogger::end()
{
  if (m_file) {
    m_file.flush();
    m_file.close();
  }
}

void AbrpJsonLogger::writeLine(const char* line)
{
  if (!m_file) {
    return;
  }
  m_file.println(line);
}

void AbrpJsonLogger::flush()
{
  if (m_file) {
    m_file.flush();
  }
}

void AbrpManager::begin(const AbrpConfig& config)
{
  m_config = config;
  m_lastPollMs = 0;
  m_lastLogMs = 0;
  memset(m_valid, 0, sizeof(m_valid));
  memset(m_values, 0, sizeof(m_values));
  m_uds.begin();
}

void AbrpManager::setStorageReady(uint32_t fileId)
{
  if (!m_config.saveJsonLog) {
    return;
  }
  m_logger.end();
  m_logger.begin(fileId);
}

void AbrpManager::updateGps(const GPS_DATA* gps)
{
  if (!gps) {
    return;
  }
  if (gps->lat || gps->lng) {
    setField(ABRP_FIELD_LAT, gps->lat);
    setField(ABRP_FIELD_LON, gps->lng);
  }
  if (gps->speed >= 0) {
    float kph = gps->speed * 1.852f;
    setField(ABRP_FIELD_SPEED, kph);
  }
  if (gps->heading) {
    setField(ABRP_FIELD_HEADING, gps->heading);
  }
  if (gps->alt) {
    setField(ABRP_FIELD_ELEVATION, gps->alt);
  }
}

void AbrpManager::updateUtc()
{
  time_t utc;
  time(&utc);
  if (utc > 0) {
    setField(ABRP_FIELD_UTC, static_cast<float>(utc));
  }
}

void AbrpManager::pollUds(uint32_t nowMs)
{
  if (!m_enabled || m_config.signalCount == 0) {
    return;
  }

  uint32_t intervalMs = static_cast<uint32_t>(m_config.sendIntervalSec) * 1000;
  if (intervalMs == 0) {
    intervalMs = 1000;
  }
  if (nowMs - m_lastPollMs < intervalMs) {
    return;
  }
  m_lastPollMs = nowMs;

  for (size_t i = 0; i < m_config.signalCount; i++) {
    float value = 0.0f;
    if (decodeSignal(m_config.signals[i], value)) {
      setField(m_config.signals[i].field, value);
    }
  }

  applyDerivedValues();
}

void AbrpManager::logJson(uint32_t nowMs)
{
  if (!m_enabled || !m_config.saveJsonLog || !m_logger.isOpen()) {
    return;
  }

  if (nowMs - m_lastLogMs < 1000) {
    return;
  }
  m_lastLogMs = nowMs;

  char line[512] = {0};
  size_t offset = 0;
  line[offset++] = '{';

  for (uint8_t field = 0; field < ABRP_FIELD_COUNT; field++) {
    AbrpField f = static_cast<AbrpField>(field);
    if (!isFieldValid(f)) {
      continue;
    }
    const char* name = fieldName(f);
    if (!name || !*name) {
      continue;
    }
    if (!appendJsonField(line, sizeof(line), offset, name, getField(f), isBoolField(f) || f == ABRP_FIELD_UTC)) {
      break;
    }
  }

  if (offset < sizeof(line) - 2) {
    line[offset++] = '}';
    line[offset] = 0;
    m_logger.writeLine(line);
  }

  static uint32_t lastFlush = 0;
  if (nowMs - lastFlush > kJsonFlushIntervalMs) {
    m_logger.flush();
    lastFlush = nowMs;
  }
}

void AbrpManager::applyDerivedValues()
{
  if (!isFieldValid(ABRP_FIELD_POWER) && isFieldValid(ABRP_FIELD_VOLTAGE) && isFieldValid(ABRP_FIELD_CURRENT)) {
    float power = getField(ABRP_FIELD_VOLTAGE) * getField(ABRP_FIELD_CURRENT) / 1000.0f;
    setField(ABRP_FIELD_POWER, power);
  }

  if (!isFieldValid(ABRP_FIELD_IS_CHARGING) && isFieldValid(ABRP_FIELD_POWER)) {
    setField(ABRP_FIELD_IS_CHARGING, getField(ABRP_FIELD_POWER) < 0.0f ? 1.0f : 0.0f);
  }

  if (!isFieldValid(ABRP_FIELD_IS_DCFC) && isFieldValid(ABRP_FIELD_POWER)) {
    float power = getField(ABRP_FIELD_POWER);
    setField(ABRP_FIELD_IS_DCFC, power < -20.0f ? 1.0f : 0.0f);
  }

  if (!isFieldValid(ABRP_FIELD_IS_PARKED) && isFieldValid(ABRP_FIELD_SPEED)) {
    float speed = getField(ABRP_FIELD_SPEED);
    setField(ABRP_FIELD_IS_PARKED, speed < 1.0f ? 1.0f : 0.0f);
  }
}

bool AbrpManager::decodeSignal(const AbrpSignalConfig& signal, float& outValue)
{
  if (signal.requestLength == 0 || signal.length == 0) {
    return false;
  }

  uint8_t response[64] = {0};
  uint16_t responseLen = sizeof(response);
  if (!m_uds.request(signal.txId, signal.txExtended,
                     signal.rxId, signal.rxExtended,
                     signal.request, signal.requestLength,
                     response, &responseLen)) {
    return false;
  }

  uint16_t payloadStart = 0;
  if (responseLen >= 3 && response[0] == 0x62) {
    payloadStart = 3;
  }

  uint16_t start = payloadStart + signal.startByte;
  if (start + signal.length > responseLen) {
    return false;
  }

  uint32_t raw = 0;
  for (uint8_t i = 0; i < signal.length; i++) {
    raw = (raw << 8) | response[start + i];
  }

  if (signal.bit >= 0 && signal.bit < 32) {
    raw = (raw >> signal.bit) & 0x1;
  }

  outValue = static_cast<float>(raw) * signal.scale + signal.offset;
  return true;
}

void AbrpManager::setField(AbrpField field, float value)
{
  if (field >= ABRP_FIELD_COUNT) {
    return;
  }
  m_values[field] = value;
  m_valid[field] = true;
}

bool AbrpManager::isFieldValid(AbrpField field) const
{
  if (field >= ABRP_FIELD_COUNT) {
    return false;
  }
  return m_valid[field];
}

float AbrpManager::getField(AbrpField field) const
{
  if (field >= ABRP_FIELD_COUNT) {
    return 0.0f;
  }
  return m_values[field];
}

const char* AbrpManager::fieldName(AbrpField field) const
{
  return ::fieldName(field);
}
