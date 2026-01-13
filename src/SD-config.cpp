/*
Functions to handle writing and reading of settings from SD-card
*/

#include "SD-config.h"
#include "config.h"
#include <Arduino.h>
#include <SD.h>
#include <SPIFFS.h>

namespace {
constexpr const char* kConfigPaths[] = {"/config/config.cfg", "/config.cfg"};
constexpr const char* kObdPaths[] = {"/config/obd.cfg", "/obd.cfg"};

bool openConfigFile(const char* const paths[], size_t pathCount, File& file)
{
#if STORAGE == STORAGE_SPIFFS
  for (size_t i = 0; i < pathCount; i++) {
    file = SPIFFS.open(paths[i], FILE_READ);
    if (file) {
      return true;
    }
  }
#else
  for (size_t i = 0; i < pathCount; i++) {
    file = SD.open(paths[i], FILE_READ);
    if (file) {
      return true;
    }
  }
#endif
  return false;
}

void trim(String& value)
{
  value.trim();
  if (value.startsWith("\"")) {
    value.remove(0, 1);
  }
  if (value.endsWith("\"")) {
    value.remove(value.length() - 1);
  }
  value.trim();
}

bool parseBool(const String& value)
{
  return value.equalsIgnoreCase("on") || value.equalsIgnoreCase("true") || value.equalsIgnoreCase("1");
}

bool parseCanId(String token, uint32_t& id, bool& extended)
{
  trim(token);
  if (token.length() == 0) {
    return false;
  }

  int colon = token.indexOf(':');
  if (colon > 0) {
    String prefix = token.substring(0, colon);
    token = token.substring(colon + 1);
    if (prefix == "29") {
      extended = true;
    } else if (prefix == "11") {
      extended = false;
    }
  }

  id = strtoul(token.c_str(), nullptr, 16);
  if (id > 0x7FF) {
    extended = true;
  }
  return id > 0;
}

uint8_t parseHexBytes(const String& hexString, uint8_t* out, size_t outSize)
{
  String cleaned = hexString;
  cleaned.replace(" ", "");
  if (cleaned.startsWith("0x") || cleaned.startsWith("0X")) {
    cleaned.remove(0, 2);
  }
  if (cleaned.length() % 2 != 0) {
    return 0;
  }
  uint8_t count = 0;
  for (size_t i = 0; i + 1 < cleaned.length() && count < outSize; i += 2) {
    char buf[3] = {cleaned[i], cleaned[i + 1], 0};
    out[count++] = static_cast<uint8_t>(strtoul(buf, nullptr, 16));
  }
  return count;
}

AbrpField fieldFromName(const String& name)
{
  if (name == "utc") return ABRP_FIELD_UTC;
  if (name == "soc") return ABRP_FIELD_SOC;
  if (name == "power") return ABRP_FIELD_POWER;
  if (name == "speed") return ABRP_FIELD_SPEED;
  if (name == "lat") return ABRP_FIELD_LAT;
  if (name == "lon") return ABRP_FIELD_LON;
  if (name == "is_charging") return ABRP_FIELD_IS_CHARGING;
  if (name == "is_dcfc") return ABRP_FIELD_IS_DCFC;
  if (name == "is_parked") return ABRP_FIELD_IS_PARKED;
  if (name == "capacity") return ABRP_FIELD_CAPACITY;
  if (name == "kwh_charged") return ABRP_FIELD_KWH_CHARGED;
  if (name == "soh") return ABRP_FIELD_SOH;
  if (name == "heading") return ABRP_FIELD_HEADING;
  if (name == "elevation") return ABRP_FIELD_ELEVATION;
  if (name == "ext_temp") return ABRP_FIELD_EXT_TEMP;
  if (name == "batt_temp") return ABRP_FIELD_BATT_TEMP;
  if (name == "voltage") return ABRP_FIELD_VOLTAGE;
  if (name == "current") return ABRP_FIELD_CURRENT;
  if (name == "odometer") return ABRP_FIELD_ODOMETER;
  if (name == "est_battery_range") return ABRP_FIELD_EST_BATTERY_RANGE;
  return ABRP_FIELD_COUNT;
}

void parseAbrpSignal(const String& key, const String& value, AbrpConfig& config)
{
  if (config.signalCount >= ABRP_MAX_SIGNALS) {
    return;
  }

  String name = key.substring(String("OBD-ABRP-").length());
  name.trim();

  AbrpField field = fieldFromName(name);
  if (field == ABRP_FIELD_COUNT) {
    return;
  }

  AbrpSignalConfig signal = {};
  signal.field = field;
  name.toCharArray(signal.name, sizeof(signal.name));

  int start = 0;
  int end = 0;
  int length = 0;
  int bit = -1;
  float scale = 1.0f;
  float offset = 0.0f;

  int tokenIndex = 0;
  int last = 0;
  String token;
  for (int i = 0; i <= value.length(); i++) {
    if (i == value.length() || value[i] == ',') {
      token = value.substring(last, i);
      trim(token);
      switch (tokenIndex) {
        case 0:
          token.toCharArray(signal.unit, sizeof(signal.unit));
          break;
        case 1:
          parseCanId(token, signal.txId, signal.txExtended);
          break;
        case 2:
          signal.requestLength = parseHexBytes(token, signal.request, sizeof(signal.request));
          break;
        case 3:
          parseCanId(token, signal.rxId, signal.rxExtended);
          break;
        case 4:
          start = token.toInt();
          break;
        case 5:
          end = token.toInt();
          break;
        case 6:
          length = token.toInt();
          break;
        case 7:
          bit = token.toInt();
          break;
        case 8:
          scale = token.toFloat();
          break;
        case 9:
          offset = token.toFloat();
          break;
        default:
          break;
      }
      tokenIndex++;
      last = i + 1;
    }
  }

  if (length <= 0) {
    if (end > start) {
      length = end - start + 1;
    } else {
      length = 1;
    }
  }

  if (start > 0) {
    signal.startByte = static_cast<uint8_t>(start - 1);
  }
  signal.length = static_cast<uint8_t>(length);
  signal.bit = static_cast<int8_t>(bit);
  signal.scale = scale == 0.0f ? 1.0f : scale;
  signal.offset = offset;

  if (signal.requestLength == 0 || signal.txId == 0 || signal.rxId == 0) {
    return;
  }

  config.signals[config.signalCount++] = signal;
}

void parseConfigFile(File& file, AbrpConfig& config)
{
  String section;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty() || line.startsWith(";") || line.startsWith("#")) {
      continue;
    }
    if (line.startsWith("[")) {
      int end = line.indexOf(']');
      if (end > 0) {
        section = line.substring(1, end);
        section.trim();
      }
      continue;
    }

    int eq = line.indexOf('=');
    if (eq < 0) {
      continue;
    }
    String key = line.substring(0, eq);
    String value = line.substring(eq + 1);
    key.trim();
    trim(value);

    if (section.equalsIgnoreCase("common") && key.equalsIgnoreCase("save-json-log")) {
      config.saveJsonLog = parseBool(value);
    } else if (section.equalsIgnoreCase("ABRP")) {
      if (key.equalsIgnoreCase("ABRP-user-token")) {
        value.toCharArray(config.userToken, sizeof(config.userToken));
      } else if (key.equalsIgnoreCase("ABRP-send-data-interval")) {
        config.sendIntervalSec = static_cast<uint16_t>(value.toInt());
      }
    }
  }
}

void parseObdFile(File& file, AbrpConfig& config)
{
  String section;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.isEmpty() || line.startsWith(";") || line.startsWith("#")) {
      continue;
    }
    if (line.startsWith("[")) {
      int end = line.indexOf(']');
      if (end > 0) {
        section = line.substring(1, end);
        section.trim();
      }
      continue;
    }

    if (!section.equalsIgnoreCase("ABRP")) {
      continue;
    }

    int eq = line.indexOf('=');
    if (eq < 0) {
      continue;
    }
    String key = line.substring(0, eq);
    String value = line.substring(eq + 1);
    key.trim();
    value.trim();

    if (key.startsWith("OBD-ABRP-")) {
      parseAbrpSignal(key, value, config);
    }
  }
}
} // namespace

bool loadAbrpConfig(AbrpConfig& config)
{
  config = {};
  config.saveJsonLog = true;
  config.sendIntervalSec = 1;

  File file;
  if (openConfigFile(kConfigPaths, sizeof(kConfigPaths) / sizeof(kConfigPaths[0]), file)) {
    parseConfigFile(file, config);
    file.close();
  }

  if (openConfigFile(kObdPaths, sizeof(kObdPaths) / sizeof(kObdPaths[0]), file)) {
    parseObdFile(file, config);
    file.close();
  }

  return true;
}
