/*
 * bEEmapiX-C3 v0.2 — Температурен мониторинг на пчелен кошер (AUTO-LEARNING)
 * MoniX JSON Protocol v1
 *
 * Описание:
 *   Firmware за ESP32-C3 (Seeed XIAO), управляващ решетка от 3×7 (21 броя) DS18B20
 *   температурни сензора на обща OneWire шина (GPIO8).
 *   При първо включване системата влиза в LEARNING MODE — потребителят загрява
 *   сензорите един по един с ръка, firmware-ът записва позицията им в NVS.
 *   При следващо захранване директно минава в работен режим.
 *
 * Протокол:
 *   MoniX JSON Protocol v1 — model="mapix", изпраща temperature_grid 3×7 и
 *   статистики. Поддържа OTA, set_cfg от сървър, light/deep sleep.
 *
 * Хардуер:
 *   - Seeed XIAO ESP32-C3
 *   - 21× DS18B20 на обща OneWire шина, GPIO8, с 4.7kΩ pull-up
 *
 * Зависимости (Arduino библиотеки):
 *   - OneWire, DallasTemperature, ArduinoJson (v6)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>

// ============ ПРОТОКОЛ И ВЕРСИЯ ============

static const uint8_t  PROTO_VER  = 1;
static const char*    FW_VERSION = "0.2.0";
static const char*    HW_VERSION = "c3-v1";
static const char*    MODEL      = "mapix";
static const char*    POST_PATH  = "/api/telemetry";

static const unsigned long LIGHT_SLEEP_THRESHOLD_MS = 60UL  * 60 * 1000; // 60 мин
static const unsigned long DEEP_SLEEP_THRESHOLD_MS  = 120UL * 60 * 1000; // 120 мин

// ============ ХАРДУЕР ============

#define ONE_WIRE_BUS   8
#define ROWS           3
#define COLS           7
#define TOTAL_SENSORS  21

// ============ SETTINGS STRUCT ============

struct Settings {
  String        ssid;
  String        password;
  String        post_host;      // само хост[:порт], без https:// и без /path
  uint16_t      post_port;
  unsigned long sendIntervalMs;

  // MoniX Protocol v1 cfg (управлявани от сървъра)
  uint32_t  cfg_ver;
  bool      enableGrid;         // включи temperature_grid в tele
  bool      enableStats;        // включи statistics в tele
  String    lastCmdId;          // за идемпотентност
};

Settings cfg;

static void applyDefaults() {
  cfg.ssid          = "Hotspot_10";
  cfg.password      = "dedaznam";
  cfg.post_host     = "koshx-hub-production.up.railway.app";
  cfg.post_port     = 443;
  cfg.sendIntervalMs = 10000UL;
  cfg.cfg_ver       = 0;
  cfg.enableGrid    = true;
  cfg.enableStats   = true;
  cfg.lastCmdId     = "";
}

// ============ RTC ПАМЕТ (запазва се при deep sleep) ============

RTC_DATA_ATTR static uint32_t rtcUplinkSeq  = 0;
RTC_DATA_ATTR static char     rtcBootId[7]  = "";
RTC_DATA_ATTR static bool     rtcNtpSynced  = false;

// ============ ГЛОБАЛНИ ПРОМЕНЛИВИ ============

OneWire         oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

Preferences preferences;

DeviceAddress sensorMap[ROWS][COLS];      // позиция → адрес на сензор
bool          mappingComplete = false;

float temperatureGrid[ROWS][COLS];        // последно измерени °C, -127.0 = невалидна
float calibrationOffset[ROWS][COLS];      // offset по сензор (avg - raw)
bool  calibrationLoaded = false;

unsigned long lastMeasurementTime = 0;
unsigned long lastSendTime        = 0;

char serialNumBuffer[17] = "";            // уникален device ID (hex)

// --- Learning mode ---
bool          learningMode      = false;
int           learnedSensors    = 0;
float         baselineTemps[TOTAL_SENSORS];
DeviceAddress discoveredAddresses[TOTAL_SENSORS];
int           discoveredCount   = 0;

// --- ACK ---
String pendingAckCmdId   = "";
bool   pendingAckApplied = false;
String pendingAckErr     = "";

// --- OTA ---
static String otaPendingUrl = "";

// ============ FORWARD DECLARATIONS ============

void sendPayload(const char* event);
void handleServerConfig(const String& responseBody);
void performOTA(const String& url);
void checkAndRunOTA();
void enterSleepIfNeeded();
void pollSerialCLI();
void printHelp();
void printShow();
void runCalibration();
void measureTemperatures();
bool isAddressEmpty(DeviceAddress addr);
bool addressesMatch(DeviceAddress addr1, DeviceAddress addr2);
void printAddress(DeviceAddress deviceAddress);

// ============ DEVICE ID ============

void initSecretSerial() {
  uint64_t chipid   = ESP.getEfuseMac();
  uint64_t secretId = chipid * 13;
  uint32_t high = (uint32_t)(secretId >> 32);
  uint32_t low  = (uint32_t)(secretId);
  snprintf(serialNumBuffer, sizeof(serialNumBuffer), "%X%08X", high, low);
}

// ============ NVS — SETTINGS (namespace "beemapix") ============

void loadFromNVS() {
  applyDefaults();
  Preferences p;
  p.begin("beemapix", true);   // read-only
  if (p.isKey("ssid"))       cfg.ssid           = p.getString("ssid",      cfg.ssid.c_str());
  if (p.isKey("pass"))       cfg.password       = p.getString("pass",      cfg.password.c_str());
  if (p.isKey("post_host"))  cfg.post_host      = p.getString("post_host", cfg.post_host.c_str());
  if (p.isKey("post_port"))  cfg.post_port      = p.getUShort("post_port", cfg.post_port);
  if (p.isKey("send_ms"))    cfg.sendIntervalMs = p.getULong("send_ms",    cfg.sendIntervalMs);
  if (p.isKey("cfg_ver"))    cfg.cfg_ver        = p.getULong("cfg_ver",    cfg.cfg_ver);
  if (p.isKey("en_grid"))    cfg.enableGrid     = p.getBool("en_grid",     cfg.enableGrid);
  if (p.isKey("en_stats"))   cfg.enableStats    = p.getBool("en_stats",    cfg.enableStats);
  if (p.isKey("last_cmd"))   cfg.lastCmdId      = p.getString("last_cmd",  cfg.lastCmdId.c_str());
  p.end();
}

void saveToNVS() {
  Preferences p;
  p.begin("beemapix", false);
  p.putString("ssid",      cfg.ssid.c_str());
  p.putString("pass",      cfg.password.c_str());
  p.putString("post_host", cfg.post_host.c_str());
  p.putUShort("post_port", cfg.post_port);
  p.putULong("send_ms",    cfg.sendIntervalMs);
  p.putULong("cfg_ver",    cfg.cfg_ver);
  p.putBool("en_grid",     cfg.enableGrid);
  p.putBool("en_stats",    cfg.enableStats);
  p.putString("last_cmd",  cfg.lastCmdId.c_str());
  p.end();
  Serial.println(">>> Настройките са записани в NVS.");
}

// ============ WiFi ============

void setupWiFi() {
  Serial.printf(">>> WiFi: свързване към %s\n", cfg.ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n>>> WiFi OK   IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n>>> WiFi FAIL (watchdog ще опитва на всеки 30s)");
  }
}

// ============ СЕНЗОРНИ ФУНКЦИИ ============

void setupSensors() {
  Serial.println(">>> DS18B20: инициализация...");
  sensors.begin();
  discoveredCount = sensors.getDeviceCount();
  Serial.printf(">>> DS18B20: открити %d сензора\n", discoveredCount);

  if (discoveredCount == 0) {
    Serial.println(">>> ГРЕШКА: Не са открити сензори! Проверете GPIO8 и pull-up (4.7kΩ).");
    return;
  }

  for (int i = 0; i < discoveredCount && i < TOTAL_SENSORS; i++) {
    if (sensors.getAddress(discoveredAddresses[i], i)) {
      sensors.setResolution(discoveredAddresses[i], 12);
    }
  }

  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      temperatureGrid[row][col] = -127.0;
      for (int b = 0; b < 8; b++) sensorMap[row][col][b] = 0;
    }
  }
}

// Проверява NVS за запазен сензорен мапинг.
// Ако има — зарежда го. Ако няма — стартира Learning Mode.
void loadOrLearnMapping() {
  Serial.println(">>> Зареждане на сензорен мапинг...");
  preferences.begin("sensor-map", false);
  mappingComplete = preferences.getBool("complete", false);

  if (mappingComplete) {
    for (int row = 0; row < ROWS; row++) {
      for (int col = 0; col < COLS; col++) {
        char key[6];
        snprintf(key, sizeof(key), "%d_%d", row, col);
        preferences.getBytes(key, sensorMap[row][col], 8);
      }
    }
    Serial.println(">>> Мапинг зареден от NVS.");
  } else {
    Serial.println("\n>>> Няма запазен мапинг. Стартиране на LEARNING MODE!");
    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║              LEARNING MODE АКТИВИРАН                  ║");
    Serial.println("╠════════════════════════════════════════════════════════╣");
    Serial.println("║  Загрейте сензорите един по един с ръка:              ║");
    Serial.println("║  [0,0] -> [0,1] -> ... -> [0,6]                       ║");
    Serial.println("║  [1,0] -> [1,1] -> ... -> [1,6]                       ║");
    Serial.println("║  [2,0] -> [2,1] -> ... -> [2,6]                       ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    startLearningMode();
  }

  preferences.end();
}

// Записва базовите температури и влиза в режим учене.
void startLearningMode() {
  learningMode    = true;
  learnedSensors  = 0;

  sensors.requestTemperatures();
  delay(1000);
  for (int i = 0; i < discoveredCount; i++) {
    baselineTemps[i] = sensors.getTempC(discoveredAddresses[i]);
  }

  Serial.println(">>> Базови температури записани.");
  Serial.printf(">>> Загрейте сензор [%d,%d]...\n",
                learnedSensors / COLS, learnedSensors % COLS);
}

// Извиква се в loop() при learningMode == true — търси загрят сензор.
void checkForHeatSignal() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 1000) return;
  lastCheck = millis();

  sensors.requestTemperatures();

  for (int i = 0; i < discoveredCount; i++) {
    float currentTemp = sensors.getTempC(discoveredAddresses[i]);
    float diff        = currentTemp - baselineTemps[i];

    if (diff > 1.5) {
      int row = learnedSensors / COLS;
      int col = learnedSensors % COLS;

      bool alreadyLearned = false;
      for (int r = 0; r < ROWS && !alreadyLearned; r++) {
        for (int c = 0; c < COLS && !alreadyLearned; c++) {
          if (addressesMatch(sensorMap[r][c], discoveredAddresses[i])) {
            alreadyLearned = true;
            Serial.println(">>> Този сензор вече е научен! Загрейте друг.");
          }
        }
      }

      if (!alreadyLearned) {
        memcpy(sensorMap[row][col], discoveredAddresses[i], 8);

        Serial.printf(">>> НАУЧЕН #%d  позиция [%d,%d]  +%.2f°C  (%d/%d)\n",
                      learnedSensors + 1, row, col, diff,
                      learnedSensors + 1, TOTAL_SENSORS);

        learnedSensors++;
        baselineTemps[i] = currentTemp;

        if (learnedSensors < TOTAL_SENSORS) {
          Serial.printf(">>> Загрейте следващия сензор [%d,%d]...\n",
                        learnedSensors / COLS, learnedSensors % COLS);
        } else {
          finishLearning();
        }
      }

      break;
    }
  }
}

// Записва целия научен мапинг в NVS и преминава в работен режим.
void finishLearning() {
  Serial.println("\n>>> LEARNING ЗАВЪРШЕН! Запазване в NVS...");

  preferences.begin("sensor-map", false);
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      char key[6];
      snprintf(key, sizeof(key), "%d_%d", row, col);
      preferences.putBytes(key, sensorMap[row][col], 8);
    }
  }
  preferences.putBool("complete", true);
  preferences.end();

  Serial.println(">>> Мапингът е запазен.");
  Serial.println(">>> За изтриване: изпратете 'reset' в Serial Monitor.");

  learningMode    = false;
  mappingComplete = true;
}

// ============ КАЛИБРАЦИЯ ============

void loadCalibration() {
  for (int row = 0; row < ROWS; row++)
    for (int col = 0; col < COLS; col++)
      calibrationOffset[row][col] = 0.0;

  Preferences calPrefs;
  calPrefs.begin("sensor-cal", true);
  calibrationLoaded = calPrefs.getBool("done", false);

  if (calibrationLoaded) {
    for (int row = 0; row < ROWS; row++) {
      for (int col = 0; col < COLS; col++) {
        char key[6];
        snprintf(key, sizeof(key), "c%d_%d", row, col);
        calibrationOffset[row][col] = calPrefs.getFloat(key, 0.0);
      }
    }
    Serial.println(">>> Калибрация заредена.");
  } else {
    Serial.println(">>> Няма калибрация (offsets = 0).");
  }

  calPrefs.end();
}

void runCalibration() {
  if (!mappingComplete) {
    Serial.println(">>> Първо завършете Learning Mode.");
    return;
  }

  Serial.println(">>> Калибрация — измерване...");
  sensors.requestTemperatures();

  float rawTemps[ROWS][COLS];
  float sum   = 0.0;
  int   count = 0;

  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      if (!isAddressEmpty(sensorMap[row][col])) {
        float temp = sensors.getTempC(sensorMap[row][col]);
        if (temp <= DEVICE_DISCONNECTED_C) {
          rawTemps[row][col] = -127.0;
        } else {
          rawTemps[row][col] = temp;
          sum += temp;
          count++;
        }
      } else {
        rawTemps[row][col] = -127.0;
      }
    }
  }

  if (count < 2) {
    Serial.println(">>> Твърде малко валидни сензори за калибрация.");
    return;
  }

  float avg = sum / count;
  Serial.printf(">>> Средна: %.2f°C  (%d сензора)\n", avg, count);

  Preferences calPrefs;
  calPrefs.begin("sensor-cal", false);

  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      if (rawTemps[row][col] > DEVICE_DISCONNECTED_C) {
        calibrationOffset[row][col] = avg - rawTemps[row][col];
        char key[6];
        snprintf(key, sizeof(key), "c%d_%d", row, col);
        calPrefs.putFloat(key, calibrationOffset[row][col]);
        Serial.printf("  [%d,%d]: %.2f°C  offset %+.2f°C\n",
                      row, col, rawTemps[row][col], calibrationOffset[row][col]);
      } else {
        calibrationOffset[row][col] = 0.0;
      }
    }
  }

  calPrefs.putBool("done", true);
  calPrefs.end();

  calibrationLoaded = true;
  Serial.println(">>> Калибрацията е записана в NVS.");
}

// ============ ИЗМЕРВАНЕ НА ТЕМПЕРАТУРИ ============

void measureTemperatures() {
  sensors.requestTemperatures();

  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      if (!isAddressEmpty(sensorMap[row][col])) {
        float temp = sensors.getTempC(sensorMap[row][col]);
        if (temp <= DEVICE_DISCONNECTED_C) temp = -127.0;
        else                               temp += calibrationOffset[row][col];
        temperatureGrid[row][col] = temp;
      } else {
        temperatureGrid[row][col] = -127.0;
      }
    }
  }
}

// ============ SEND PAYLOAD — MoniX Protocol v1 ============

void sendPayload(const char* event) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(">>> WiFi не е свързан — пропускане на uplink");
    return;
  }

  rtcUplinkSeq++;

  // Timestamp
  time_t now = 0;
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 200)) now = mktime(&timeinfo);

  // URL
  char urlBuffer[160];
  snprintf(urlBuffer, sizeof(urlBuffer), "https://%s%s",
           cfg.post_host.c_str(), POST_PATH);

  // Статистики
  float minTemp      = 999.0;
  float maxTemp      = -999.0;
  float sumTemp      = 0.0;
  int   validReadings = 0;

  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      float t = temperatureGrid[row][col];
      if (t > DEVICE_DISCONNECTED_C) {
        if (t < minTemp) minTemp = t;
        if (t > maxTemp) maxTemp = t;
        sumTemp += t;
        validReadings++;
      }
    }
  }

  // JSON build  (capacity for 3×7 grid + all fields)
  StaticJsonDocument<3072> doc;

  doc["v"]        = PROTO_VER;
  doc["id"]       = serialNumBuffer;
  doc["model"]    = MODEL;
  doc["hw"]       = HW_VERSION;
  doc["fw"]       = FW_VERSION;
  doc["ts"]       = (uint32_t)now;
  doc["uptime_s"] = (uint32_t)(millis() / 1000);
  doc["boot_id"]  = rtcBootId;
  doc["seq"]      = rtcUplinkSeq;
  doc["cfg_ver"]  = cfg.cfg_ver;
  doc["event"]    = event;

  JsonObject status = doc.createNestedObject("status");
  status["wifi"]    = (WiFi.status() == WL_CONNECTED) ? "ok" : "err";
  status["ds18b20"] = (discoveredCount > 0)            ? "ok" : "err";

  JsonObject data = doc.createNestedObject("data");
  JsonObject tele = data.createNestedObject("tele");

  if (cfg.enableGrid) {
    JsonArray grid = tele.createNestedArray("temperature_grid");
    for (int row = 0; row < ROWS; row++) {
      JsonArray rowArr = grid.createNestedArray();
      for (int col = 0; col < COLS; col++) {
        float t = temperatureGrid[row][col];
        if (t > DEVICE_DISCONNECTED_C) rowArr.add(serialized(String(t, 2)));
        else                           rowArr.add((const char*)nullptr);
      }
    }
    tele["rows"] = ROWS;
    tele["cols"] = COLS;
  }

  if (cfg.enableStats && validReadings > 0) {
    JsonObject stats   = tele.createNestedObject("statistics");
    stats["min_temp"]       = serialized(String(minTemp, 2));
    stats["max_temp"]       = serialized(String(maxTemp, 2));
    stats["avg_temp"]       = serialized(String(sumTemp / validReadings, 2));
    stats["valid_readings"] = validReadings;
  }

  JsonObject ack = doc.createNestedObject("ack");
  if (pendingAckCmdId.length() > 0) {
    ack["cmd_id"]  = pendingAckCmdId;
    ack["applied"] = pendingAckApplied;
    ack["err"]     = pendingAckErr.length() > 0 ? pendingAckErr.c_str()
                                                 : (const char*)nullptr;
    pendingAckCmdId   = "";
    pendingAckApplied = false;
    pendingAckErr     = "";
  } else {
    ack["cmd_id"]  = (const char*)nullptr;
    ack["applied"] = false;
    ack["err"]     = (const char*)nullptr;
  }

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.printf(">>> TX [%s]  seq=%u  %u bytes\n",
                event, rtcUplinkSeq, jsonString.length());

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15000);

  HTTPClient http;
  http.begin(client, urlBuffer);
  http.setTimeout(15000);
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(jsonString);

  if (code > 0) {
    Serial.printf(">>> HTTP %d\n", code);
    String resp = http.getString();
    if (resp.length() > 2) handleServerConfig(resp);
  } else {
    Serial.printf(">>> HTTP ERR: %s\n", http.errorToString(code).c_str());
  }

  http.end();
}

// ============ HANDLE SERVER CONFIG ============

void handleServerConfig(const String& responseBody) {
  StaticJsonDocument<1024> respDoc;
  if (deserializeJson(respDoc, responseBody) != DeserializationError::Ok) return;
  if (!respDoc.containsKey("cmd")) return;

  JsonObject cmd = respDoc["cmd"];
  if (!cmd.containsKey("ops")) return;

  String cmdId     = cmd.containsKey("cmd_id") ? cmd["cmd_id"].as<String>() : "";
  bool   idempotent = (cmdId.length() > 0 && cmdId == cfg.lastCmdId);

  JsonArray ops  = cmd["ops"];
  bool      allOk  = true;
  String    errCode = "";

  for (JsonObject op : ops) {
    if (!op.containsKey("op")) continue;
    String opType = op["op"].as<String>();

    if (opType == "set_cfg") {
      if (idempotent) continue;  // вече приложена команда

      uint32_t newCfgVer = op.containsKey("cfg_ver") ? op["cfg_ver"].as<uint32_t>() : 0;
      if (newCfgVer <= cfg.cfg_ver) {
        Serial.printf(">>> set_cfg: cfg_ver %u <= текуща %u — пропускане\n",
                      newCfgVer, cfg.cfg_ver);
        continue;
      }

      bool changed = false;
      if (op.containsKey("cfg")) {
        JsonObject c = op["cfg"];

        if (c.containsKey("interval_min")) {
          unsigned long newMs = c["interval_min"].as<unsigned long>() * 60UL * 1000UL;
          if (newMs >= 60000UL) {
            cfg.sendIntervalMs = newMs;
            changed = true;
          } else {
            errCode = "invalid_cfg";
            allOk   = false;
            continue;
          }
        }

        if (c.containsKey("enable")) {
          JsonObject en = c["enable"];
          if (en.containsKey("temperature_grid")) {
            cfg.enableGrid  = en["temperature_grid"].as<bool>();
            changed = true;
          }
          if (en.containsKey("statistics")) {
            cfg.enableStats = en["statistics"].as<bool>();
            changed = true;
          }
        }
      }

      if (changed) {
        cfg.cfg_ver = newCfgVer;
        Serial.printf(">>> set_cfg OK  cfg_ver=%u  interval=%lums\n",
                      cfg.cfg_ver, cfg.sendIntervalMs);
      }

    } else if (opType == "ota_update") {
      if (!op.containsKey("url")) {
        errCode = "invalid_cfg";
        allOk   = false;
        continue;
      }

      String otaUrl = op["url"].as<String>();

      if (op.containsKey("fw_ver")) {
        if (op["fw_ver"].as<String>() == String(FW_VERSION)) {
          Serial.printf(">>> OTA: вече на fw %s, пропускане\n", FW_VERSION);
          continue;
        }
      }

      Serial.println(">>> OTA: scheduled");
      otaPendingUrl = otaUrl;

    } else {
      Serial.printf(">>> Непозната операция: %s\n", opType.c_str());
    }
  }

  if (cmdId.length() > 0) {
    pendingAckCmdId   = cmdId;
    pendingAckApplied = allOk;
    pendingAckErr     = allOk ? "" : errCode;

    if (cmdId != cfg.lastCmdId) {
      cfg.lastCmdId = cmdId;
      saveToNVS();
    }
  }
}

// ============ OTA ============

void performOTA(const String& url) {
  Serial.println(">>> OTA: стартиране от: " + url);
  Serial.flush();

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(30000);

  httpUpdate.onProgress([](int cur, int total) {
    Serial.printf(">>> OTA: %d / %d bytes (%.0f%%)\n",
                  cur, total, total > 0 ? 100.0f * cur / total : 0.0f);
  });

  t_httpUpdate_return ret = httpUpdate.update(client, url);

  switch (ret) {
    case HTTP_UPDATE_OK:
      Serial.println(">>> OTA: успех — рестарт");
      Serial.flush();
      delay(300);
      ESP.restart();
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(">>> OTA: сървърът не изпрати ъпдейт");
      break;
    case HTTP_UPDATE_FAILED: default:
      Serial.printf(">>> OTA FAILED (%d): %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      break;
  }
}

static void checkAndRunOTA() {
  if (otaPendingUrl.length() == 0) return;
  String url = otaPendingUrl;
  otaPendingUrl = "";
  sendPayload("ota_start");  // ACK uplink преди рестарта
  performOTA(url);
}

// ============ SLEEP ============

void enterSleepIfNeeded() {
  if (cfg.sendIntervalMs < LIGHT_SLEEP_THRESHOLD_MS) return;

  if (cfg.sendIntervalMs >= DEEP_SLEEP_THRESHOLD_MS) {
    uint64_t sleepUs = (uint64_t)(cfg.sendIntervalMs - 5000) * 1000ULL;
    Serial.printf(">>> Deep sleep %lu мин...\n", cfg.sendIntervalMs / 60000);
    Serial.flush();
    esp_sleep_enable_timer_wakeup(sleepUs);
    esp_deep_sleep_start();
  } else {
    uint64_t sleepUs = (uint64_t)(cfg.sendIntervalMs - 5000) * 1000ULL;
    Serial.printf(">>> Light sleep %lu мин...\n", cfg.sendIntervalMs / 60000);
    Serial.flush();
    esp_sleep_enable_timer_wakeup(sleepUs);
    esp_light_sleep_start();
    Serial.println(">>> Събуждане от light sleep");
  }
}

// ============ SERIAL CLI ============

void printHelp() {
  Serial.println("\n─── bEEmapiX CLI ────────────────────────────────────────");
  Serial.println("help                    - тази помощ");
  Serial.println("show                    - текущи настройки и статус");
  Serial.println("set ssid <v>            - WiFi мрежа");
  Serial.println("set password <v>        - WiFi парола");
  Serial.println("set post_host <v>       - хост на сървъра (без https://)");
  Serial.println("set post_port <v>       - TCP порт (по подразбиране: 443)");
  Serial.println("set sendintervalms <v>  - интервал (ms, мин. 10000)");
  Serial.println("save                    - записва настройките в NVS");
  Serial.println("load                    - зарежда от NVS");
  Serial.println("defaults                - фабрични стойности (не записва)");
  Serial.println("apply                   - WiFi reconnect с текущите настройки");
  Serial.println("reboot                  - рестарт");
  Serial.println("ota <url>               - OTA firmware update от URL");
  Serial.println("calibrate               - калибрация на сензорите");
  Serial.println("reset cal               - изтрива само калибрацията");
  Serial.println("reset                   - изтрива sensor-map + sensor-cal + reboot");
  Serial.println("─────────────────────────────────────────────────────────");
}

void printShow() {
  Serial.println("\n─── Текущи настройки ────────────────────────────────────");
  Serial.printf("Device ID       : %s\n", serialNumBuffer);
  Serial.printf("FW version      : %s\n", FW_VERSION);
  Serial.printf("boot_id         : %s\n", rtcBootId);
  Serial.printf("uplink seq      : %u\n",  rtcUplinkSeq);
  Serial.println("---");
  Serial.printf("ssid            : %s\n", cfg.ssid.c_str());
  Serial.printf("post_host       : %s\n", cfg.post_host.c_str());
  Serial.printf("post_port       : %u\n",  cfg.post_port);
  Serial.printf("sendIntervalMs  : %lu ms  (%lu сек)\n",
                cfg.sendIntervalMs, cfg.sendIntervalMs / 1000);
  Serial.printf("cfg_ver         : %u\n",  cfg.cfg_ver);
  Serial.printf("enableGrid      : %s\n", cfg.enableGrid  ? "true" : "false");
  Serial.printf("enableStats     : %s\n", cfg.enableStats ? "true" : "false");
  Serial.println("---");
  Serial.printf("WiFi            : %s\n",
                WiFi.status() == WL_CONNECTED ? "OK" : "FAIL");
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("IP              : %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Sensors found   : %d\n", discoveredCount);
  Serial.printf("Mapping         : %s\n", mappingComplete ? "complete" : "incomplete");
  Serial.printf("Calibration     : %s\n", calibrationLoaded ? "loaded" : "none");
  Serial.printf("Learning mode   : %s\n", learningMode ? "ACTIVE" : "off");
  Serial.printf("OTA pending     : %s\n",
                otaPendingUrl.length() > 0 ? otaPendingUrl.c_str() : "none");
  Serial.printf("Free heap       : %u bytes\n", ESP.getFreeHeap());
  Serial.println("─────────────────────────────────────────────────────────");
}

void handleCommand(const String& raw) {
  String cmd = raw;
  cmd.trim();
  if (cmd.length() == 0) return;

  String lo = cmd;
  lo.toLowerCase();

  if (lo == "help") {
    printHelp();

  } else if (lo == "show") {
    printShow();

  } else if (lo == "save") {
    saveToNVS();

  } else if (lo == "load") {
    loadFromNVS();
    Serial.println(">>> Настройките са заредени от NVS.");

  } else if (lo == "defaults") {
    applyDefaults();
    Serial.println(">>> Фабрични стойности приложени (не са записани — използвай 'save').");

  } else if (lo == "apply") {
    Serial.println(">>> WiFi reconnect...");
    WiFi.disconnect();
    delay(500);
    WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());

  } else if (lo == "reboot") {
    Serial.println(">>> Рестарт...");
    Serial.flush();
    delay(200);
    ESP.restart();

  } else if (lo == "calibrate") {
    runCalibration();

  } else if (lo == "reset cal") {
    Serial.println(">>> Изтриване на калибрацията...");
    Preferences calPrefs;
    calPrefs.begin("sensor-cal", false);
    calPrefs.clear();
    calPrefs.end();
    calibrationLoaded = false;
    for (int r = 0; r < ROWS; r++)
      for (int c = 0; c < COLS; c++)
        calibrationOffset[r][c] = 0.0;
    Serial.println(">>> Калибрацията е изтрита. Offsets = 0.");

  } else if (lo == "reset") {
    Serial.println(">>> Изтриване на sensor-map и sensor-cal...");
    preferences.begin("sensor-map", false);
    preferences.clear();
    preferences.end();
    Preferences calPrefs;
    calPrefs.begin("sensor-cal", false);
    calPrefs.clear();
    calPrefs.end();
    Serial.println(">>> Изтрито. Рестарт...");
    Serial.flush();
    delay(500);
    ESP.restart();

  } else if (lo.startsWith("set ")) {
    // Формат: "set key value" — key е case-insensitive, value е case-sensitive
    String rest   = cmd.substring(4);  // оригинален case
    rest.trim();
    String loRest = rest;
    loRest.toLowerCase();

    int spaceIdx = loRest.indexOf(' ');
    if (spaceIdx < 1) {
      Serial.println(">>> Формат: set <ключ> <стойност>");
      return;
    }

    String key = loRest.substring(0, spaceIdx);
    String val = rest.substring(spaceIdx + 1);  // оригинален case за стойностите

    if (key == "ssid") {
      cfg.ssid = val;
      Serial.printf(">>> ssid = %s\n", cfg.ssid.c_str());

    } else if (key == "password") {
      cfg.password = val;
      Serial.println(">>> password = ***");

    } else if (key == "post_host") {
      cfg.post_host = val;
      Serial.printf(">>> post_host = %s\n", cfg.post_host.c_str());

    } else if (key == "post_port") {
      cfg.post_port = (uint16_t)val.toInt();
      Serial.printf(">>> post_port = %u\n", cfg.post_port);

    } else if (key == "sendintervalms") {
      unsigned long ms = (unsigned long)val.toInt();
      if (ms >= 10000UL) {
        cfg.sendIntervalMs = ms;
        Serial.printf(">>> sendIntervalMs = %lu ms\n", cfg.sendIntervalMs);
      } else {
        Serial.println(">>> Минимален интервал: 10000 ms (10 секунди)");
      }

    } else {
      Serial.printf(">>> Непознат ключ: %s\n", key.c_str());
    }

  } else if (lo.startsWith("ota ")) {
    String url = cmd.substring(4);
    url.trim();
    if (url.length() > 0) {
      Serial.println(">>> OTA стартиране...");
      performOTA(url);
    } else {
      Serial.println(">>> Формат: ota <url>");
    }

  } else {
    Serial.printf(">>> Непозната команда: '%s'  (help за помощ)\n", cmd.c_str());
  }
}

void pollSerialCLI() {
  static String inputBuf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuf.length() > 0) {
        handleCommand(inputBuf);
        inputBuf = "";
      }
    } else {
      inputBuf += c;
      if (inputBuf.length() > 256) inputBuf = "";  // overflow guard
    }
  }
}

// ============ HELPER ФУНКЦИИ ============

bool isAddressEmpty(DeviceAddress addr) {
  for (int i = 0; i < 8; i++) if (addr[i] != 0) return false;
  return true;
}

bool addressesMatch(DeviceAddress addr1, DeviceAddress addr2) {
  for (int i = 0; i < 8; i++) if (addr1[i] != addr2[i]) return false;
  return true;
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(":");
  }
}

// ============ SETUP ============

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Detect deep sleep wake
  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  bool fromDeepSleep = (wakeupCause == ESP_SLEEP_WAKEUP_TIMER);

  if (!fromDeepSleep) {
    // Нов boot — генерира нов boot_id и нулира seq
    uint32_t rnd = esp_random();
    snprintf(rtcBootId, sizeof(rtcBootId), "%06X", rnd & 0xFFFFFF);
    rtcUplinkSeq = 0;
    rtcNtpSynced = false;
  }

  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.printf( "║  bEEmapiX-C3 v%s  MoniX Protocol v1                 ║\n",
                 FW_VERSION);
  Serial.printf( "║  boot_id: %-6s   %s\n",
                 rtcBootId, fromDeepSleep
                 ? "                [deep sleep wake]  ║\n"
                 : "                [fresh boot]        ║\n");
  Serial.println("╚════════════════════════════════════════════════════════╝\n");

  initSecretSerial();
  Serial.printf(">>> Device ID: %s\n\n", serialNumBuffer);

  loadFromNVS();
  setupWiFi();
  setupSensors();

  if (!fromDeepSleep) {
    // Нормален boot — зареди или стартирай Learning Mode
    loadOrLearnMapping();
  } else {
    // Deep sleep wake — зареди мапинга директно без Learning Mode
    preferences.begin("sensor-map", true);
    mappingComplete = preferences.getBool("complete", false);
    if (mappingComplete) {
      for (int row = 0; row < ROWS; row++) {
        for (int col = 0; col < COLS; col++) {
          char key[6];
          snprintf(key, sizeof(key), "%d_%d", row, col);
          preferences.getBytes(key, sensorMap[row][col], 8);
        }
      }
      Serial.println(">>> Мапинг зареден (wake from deep sleep).");
    }
    preferences.end();
  }

  loadCalibration();

  // NTP синхронизация (пропуска се при deep sleep wake ако вече е синхронизирано)
  if (!rtcNtpSynced) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print(">>> NTP sync");
    for (int i = 0; i < 10; i++) {
      delay(500);
      Serial.print(".");
      struct tm tm;
      if (getLocalTime(&tm, 100)) { rtcNtpSynced = true; break; }
    }
    Serial.println(rtcNtpSynced ? " OK" : " FAIL (без timestamp)");
  }

  // Начално измерване
  if (mappingComplete && !learningMode) {
    measureTemperatures();
  }

  // Начален uplink
  const char* ev = fromDeepSleep ? "interval" : "boot";
  sendPayload(ev);
  lastSendTime        = millis();
  lastMeasurementTime = millis();

  checkAndRunOTA();
  enterSleepIfNeeded();
}

// ============ LOOP ============

void loop() {
  pollSerialCLI();

  unsigned long now = millis();

  // WiFi watchdog
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiTry = 0;
    if (now - lastWifiTry > 30000) {
      lastWifiTry = now;
      Serial.println(">>> WiFi загубен — reconnect...");
      WiFi.reconnect();
    }
  }

  if (learningMode) {
    checkForHeatSignal();
  } else {
    // Периодично измерване
    if (now - lastMeasurementTime >= cfg.sendIntervalMs) {
      lastMeasurementTime = now;
      measureTemperatures();
    }

    // Периодично изпращане
    if (now - lastSendTime >= cfg.sendIntervalMs) {
      lastSendTime = now;
      sendPayload("interval");
      checkAndRunOTA();
      enterSleepIfNeeded();
    }
  }

  delay(10);
}
