/*
 * Температурен мониторинг ESP32-C3 v3.0 - AUTO-LEARNING
 * 
 * Функции:
 * - Автоматично учене на позициите чрез загряване
 * - Запазване в енергонезависима памет (Preferences)
 * - WiFi watchdog
 * - Hardware device ID (serialNumBuffer)
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ============ КОНФИГУРАЦИЯ ============

#include "config.h"

char WIFI_SSID[33] = DEFAULT_WIFI_SSID;
char WIFI_PASSWORD[65] = DEFAULT_WIFI_PASSWORD;
char SERVER_URL[129] = DEFAULT_SERVER_URL;

#define ONE_WIRE_BUS 8  // GPIO8
#define ROWS 3
#define COLS 7
#define TOTAL_SENSORS 21

unsigned long SEND_PERIOD = 60000;         // 60 секунди
unsigned long MEASUREMENT_PERIOD = SEND_PERIOD;


// ============ ГЛОБАЛНИ ПРОМЕНЛИВИ ============

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Preferences за енергонезависима памет
Preferences preferences;

// Сензорен мапинг [ред][колона] = адрес
DeviceAddress sensorMap[ROWS][COLS];
bool mappingComplete = false;

// Температурна матрица
float temperatureGrid[ROWS][COLS];

// Таймери
unsigned long lastMeasurementTime = 0;
unsigned long lastSendTime = 0;

// Уникален device ID
char serialNumBuffer[17] = "";

// Learning mode
bool learningMode = false;
int learnedSensors = 0;
float baselineTemps[TOTAL_SENSORS];
DeviceAddress discoveredAddresses[TOTAL_SENSORS];
int discoveredCount = 0;

// ============ DEVICE ID ФУНКЦИЯ ============

void initSecretSerial() {
  uint64_t chipid = ESP.getEfuseMac();
  uint64_t secretId = chipid * 13;
  uint32_t high = (uint32_t)(secretId >> 32);
  uint32_t low  = (uint32_t)(secretId);
  snprintf(serialNumBuffer, sizeof(serialNumBuffer), "%X%08X", high, low);
}

// ============ SETUP ============

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║  		bEEmapiX-C3 v0.1 AUTO-LEARNING 		 ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  
  // Инициализация на device ID
  initSecretSerial();
  Serial.printf("\n📟 Device ID: %s\n", serialNumBuffer);
  
  Serial.printf("📊 Конфигурация: %dx%d решетка (%d сензора)\n", ROWS, COLS, TOTAL_SENSORS);
  Serial.printf("⏱  Период измерване: %lu ms\n", MEASUREMENT_PERIOD);
  Serial.printf("📤 Период изпращане: %lu ms\n\n", SEND_PERIOD);
  
  // WiFi
  setupWiFi();
  
  // Сензори
  setupSensors();
  
  // Зареждане или учене на мапинг
  loadOrLearnMapping();
  
  Serial.println("\n✓ Системата е готова!\n");
}

// ============ LOOP ============

void loop() {
  unsigned long currentTime = millis();
  
  // WiFi Watchdog
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiTry = 0;
    if (currentTime - lastWifiTry > 30000) {
      lastWifiTry = currentTime;
      Serial.println("⚠ WiFi връзка загубена. Опит за преконектване...");
      WiFi.reconnect();
    }
  }
  
  // Learning mode
  if (learningMode) {
    checkForHeatSignal();
  } else {
    // Нормална работа
    if (currentTime - lastMeasurementTime >= MEASUREMENT_PERIOD) {
      lastMeasurementTime = currentTime;
      measureTemperatures();
    }
    
    if (currentTime - lastSendTime >= SEND_PERIOD) {
      lastSendTime = currentTime;
      sendData();
    }
  }
  
  serialEvent();
  delay(10);
}

// ============ WiFi ФУНКЦИИ ============

void setupWiFi() {
  Serial.printf("📡 Свързване към WiFi: %s\n", WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi свързан!");
    Serial.printf("  IP адрес: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n✗ WiFi неуспешен!");
  }
}

// ============ СЕНЗОРНИ ФУНКЦИИ ============

void setupSensors() {
  Serial.println("\n🔍 Инициализация на DS18B20 сензори...");
  
  sensors.begin();
  discoveredCount = sensors.getDeviceCount();
  
  Serial.printf("✓ Открити сензори: %d\n", discoveredCount);
  
  if (discoveredCount == 0) {
    Serial.println("❌ ГРЕШКА: Не са открити сензори!");
    Serial.println("   Проверете свързването и pull-up резистора (4.7kΩ)");
    return;
  }
  
  // Запазване на адресите
  for (int i = 0; i < discoveredCount && i < TOTAL_SENSORS; i++) {
    if (sensors.getAddress(discoveredAddresses[i], i)) {
      sensors.setResolution(discoveredAddresses[i], 12);
      Serial.printf("  Сензор %d: ", i);
      printAddress(discoveredAddresses[i]);
      Serial.println();
    }
  }
  
  // Инициализация на температурната матрица
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      temperatureGrid[row][col] = -127.0;
      // Инициализация на празен адрес
      for (int b = 0; b < 8; b++) {
        sensorMap[row][col][b] = 0;
      }
    }
  }
}

void loadOrLearnMapping() {
  Serial.println("\n📚 Зареждане на сензорен мапинг...");
  
  preferences.begin("sensor-map", false);
  
  // Проверка дали има запазен мапинг
  mappingComplete = preferences.getBool("complete", false);
  
  if (mappingComplete) {
    Serial.println("✓ Намерен запазен мапинг в паметта!");
    
    // Зареждане на мапинга
    for (int row = 0; row < ROWS; row++) {
      for (int col = 0; col < COLS; col++) {
        String key = String(row) + "_" + String(col);
        preferences.getBytes(key.c_str(), sensorMap[row][col], 8);
        
        // Показване на заредения адрес
        if (!isAddressEmpty(sensorMap[row][col])) {
          Serial.printf("  [%d,%d]: ", row, col);
          printAddress(sensorMap[row][col]);
          Serial.println();
        }
      }
    }
    
    Serial.println("\n✅ Мапингът е зареден. Нормална работа.");
    
  } else {
    Serial.println("⚠ Няма запазен мапинг. Стартиране на LEARNING MODE!");
    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║              🔥 LEARNING MODE АКТИВИРАН 🔥            ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("📋 ИНСТРУКЦИИ:");
    Serial.println("───────────────────────────────────────────────────────");
    Serial.println("1. ЗАГРЕЙТЕ ПЪРВИЯ СЕНЗОР (позиция [0,0])");
    Serial.println("   → Хванете го с ръка или феш за 5-10 секунди");
    Serial.println("2. Изчакайте да се открие и запише");
    Serial.println("3. ЗАГРЕЙТЕ ВТОРИЯ СЕНЗОР (позиция [0,1])");
    Serial.println("4. Продължете в реда:");
    Serial.println("   [0,0] → [0,1] → [0,2] → ... → [0,6]");
    Serial.println("   [1,0] → [1,1] → [1,2] → ... → [1,6]");
    Serial.println("   [2,0] → [2,1] → [2,2] → ... → [2,6]");
    Serial.println("5. Общо трябва да загреете 21 сензора последователно");
    Serial.println("───────────────────────────────────────────────────────");
    Serial.println();
    
    startLearningMode();
  }
  
  preferences.end();
}

void startLearningMode() {
  learningMode = true;
  learnedSensors = 0;
  
  // Четене на базовите температури
  sensors.requestTemperatures();
  delay(1000);
  
  for (int i = 0; i < discoveredCount; i++) {
    baselineTemps[i] = sensors.getTempC(discoveredAddresses[i]);
  }
  
  Serial.println("📊 Базови температури записани.");
  Serial.printf("\n🔥 Загрейте сензор за позиция [%d,%d]...\n", 
                learnedSensors / COLS, learnedSensors % COLS);
}

void checkForHeatSignal() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck < 1000) return; // Проверка на всяка секунда
  lastCheck = millis();
  
  sensors.requestTemperatures();
  
  // Търсене на сензор с промяна > 1.5°C
  for (int i = 0; i < discoveredCount; i++) {
    float currentTemp = sensors.getTempC(discoveredAddresses[i]);
    float diff = currentTemp - baselineTemps[i];
    
    if (diff > 1.5) {
      // Намерен загрят сензор!
      int row = learnedSensors / COLS;
      int col = learnedSensors % COLS;
      
      // Проверка дали този адрес вече е научен
      bool alreadyLearned = false;
      for (int r = 0; r < ROWS && !alreadyLearned; r++) {
        for (int c = 0; c < COLS && !alreadyLearned; c++) {
          if (addressesMatch(sensorMap[r][c], discoveredAddresses[i])) {
            alreadyLearned = true;
            Serial.println("⚠ Този сензор вече е научен! Загрейте друг.");
          }
        }
      }
      
      if (!alreadyLearned) {
        // Запазване на адреса
        memcpy(sensorMap[row][col], discoveredAddresses[i], 8);
        
        Serial.println("\n╔════════════════════════════════════════════════════════╗");
        Serial.printf("║  ✅ СЕНЗОР #%d НАУЧЕН - Позиция [%d,%d]              ║\n", 
                      learnedSensors + 1, row, col);
        Serial.println("╚════════════════════════════════════════════════════════╝");
        Serial.printf("   Адрес: ");
        printAddress(discoveredAddresses[i]);
        Serial.printf("\n   Промяна: +%.2f°C\n", diff);
        Serial.printf("   Прогрес: %d/%d сензора\n", learnedSensors + 1, TOTAL_SENSORS);
        
        learnedSensors++;
        
        // Обновяване на базовата температура
        baselineTemps[i] = currentTemp;
        
        if (learnedSensors < TOTAL_SENSORS) {
          Serial.printf("\n🔥 Загрейте следващия сензор за позиция [%d,%d]...\n", 
                        learnedSensors / COLS, learnedSensors % COLS);
        } else {
          // Всички сензори са научени!
          finishLearning();
        }
      }
      
      break; // Само един сензор на итерация
    }
  }
}

void finishLearning() {
  Serial.println("\n");
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║         🎉 LEARNING ЗАВЪРШЕН УСПЕШНО! 🎉             ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
  
  // Запазване на мапинга
  preferences.begin("sensor-map", false);
  
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      String key = String(row) + "_" + String(col);
      preferences.putBytes(key.c_str(), sensorMap[row][col], 8);
    }
  }
  
  preferences.putBool("complete", true);
  preferences.end();
  
  Serial.println("💾 Мапингът е запазен в енергонезависимата памет!");
  Serial.println();
  Serial.println("📋 НАУЧЕН МАПИНГ:");
  Serial.println("───────────────────────────────────────────────────────");
  
  for (int row = 0; row < ROWS; row++) {
    Serial.printf("Ред %d: ", row);
    for (int col = 0; col < COLS; col++) {
      Serial.print("[");
      printAddress(sensorMap[row][col]);
      Serial.print("] ");
    }
    Serial.println();
  }
  
  Serial.println("───────────────────────────────────────────────────────");
  Serial.println();
  Serial.println("✅ Системата преминава в нормална работа!");
  Serial.println("   За изтриване на мапинга: Изпратете 'RESET' в Serial Monitor");
  Serial.println();
  
  learningMode = false;
  mappingComplete = true;
}

void measureTemperatures() {
  Serial.println("\n─── Измерване на температури ───");
  
  sensors.requestTemperatures();
  
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      if (!isAddressEmpty(sensorMap[row][col])) {
        float temp = sensors.getTempC(sensorMap[row][col]);
        temperatureGrid[row][col] = temp;
        Serial.printf("[%d,%d]: %.2f°C\n", row, col, temp);
      } else {
        temperatureGrid[row][col] = -127.0;
      }
    }
  }
}

void sendData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("✗ WiFi не е свързан");
    return;
  }
  
  Serial.println("\n─── Изпращане на данни ───");
  
  StaticJsonDocument<2048> doc;
  
  doc["dev_num"] = serialNumBuffer;
  doc["timestamp"] = millis();
  doc["interval"] = SEND_PERIOD / 1000;
  doc["rows"] = ROWS;
  doc["cols"] = COLS;
  
  JsonArray grid = doc.createNestedArray("temperature_grid");
  
  for (int row = 0; row < ROWS; row++) {
    JsonArray rowArray = grid.createNestedArray();
    for (int col = 0; col < COLS; col++) {
      rowArray.add(temperatureGrid[row][col]);
    }
  }
  
  // Статистики
  float minTemp = 999.0, maxTemp = -999.0, sumTemp = 0.0;
  int validReadings = 0;
  
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      float temp = temperatureGrid[row][col];
      if (temp > -127.0) {
        if (temp < minTemp) minTemp = temp;
        if (temp > maxTemp) maxTemp = temp;
        sumTemp += temp;
        validReadings++;
      }
    }
  }
  
  if (validReadings > 0) {
    doc["statistics"]["min_temp"] = minTemp;
    doc["statistics"]["max_temp"] = maxTemp;
    doc["statistics"]["avg_temp"] = sumTemp / validReadings;
    doc["statistics"]["valid_readings"] = validReadings;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // HTTPS заявка
  WiFiClientSecure client;
  client.setInsecure(); // За тестване - в production използвайте сертификат
  
  HTTPClient http;
  http.begin(client, SERVER_URL);
  http.setTimeout(10000);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    Serial.printf("✓ HTTP: %d\n", httpResponseCode);
    String response = http.getString();
    Serial.println("Отговор: " + response);

    StaticJsonDocument<256> respDoc;
    if (deserializeJson(respDoc, response) == DeserializationOk) {
      if (respDoc["edit"] == 1 && respDoc.containsKey("interval")) {
        unsigned long newInterval = respDoc["interval"].as<unsigned long>() * 1000;
        if (newInterval < 10000) newInterval = 10000;
        SEND_PERIOD = newInterval;
        MEASUREMENT_PERIOD = SEND_PERIOD;
        Serial.printf("⚙ Нов интервал: %lu ms\n", SEND_PERIOD);
      }
    }
  } else {
    Serial.printf("✗ HTTP грешка: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  
  http.end();
}

// ============ HELPER ФУНКЦИИ ============

bool isAddressEmpty(DeviceAddress addr) {
  for (int i = 0; i < 8; i++) {
    if (addr[i] != 0) return false;
  }
  return true;
}

bool addressesMatch(DeviceAddress addr1, DeviceAddress addr2) {
  for (int i = 0; i < 8; i++) {
    if (addr1[i] != addr2[i]) return false;
  }
  return true;
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(":");
  }
}

// ============ SERIAL COMMANDS ============

void serialEvent() {
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "HELP") {
      Serial.println("\n─── Команди ───");
      Serial.println("SHOW                  - текущи настройки");
      Serial.println("SET WIFI ssid pass    - смяна на WiFi мрежа");
      Serial.println("SET SERVER url        - смяна на сървър");
      Serial.println("SET INTERVAL секунди  - смяна на интервал (мин 10)");
      Serial.println("RESET                 - изтриване на сензорен мапинг");
      Serial.println("HELP                  - тази помощ");
      Serial.println("───────────────────────");

    } else if (command == "RESET") {
      Serial.println("\n⚠ Изтриване на запазения мапинг...");
      preferences.begin("sensor-map", false);
      preferences.clear();
      preferences.end();
      Serial.println("✓ Мапингът е изтрит. Рестартирайте устройството.");

    } else if (command == "SHOW") {
      Serial.println("\n─── Текущи настройки ───");
      Serial.printf("📟 Device ID: %s\n", serialNumBuffer);
      Serial.printf("📡 WiFi SSID: %s\n", WIFI_SSID);
      Serial.printf("📡 WiFi статус: %s\n", WiFi.status() == WL_CONNECTED ? "свързан" : "не е свързан");
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("   IP адрес: %s\n", WiFi.localIP().toString().c_str());
      }
      Serial.printf("🌐 Сървър: %s\n", SERVER_URL);
      Serial.printf("📤 Интервал изпращане: %lu сек\n", SEND_PERIOD / 1000);
      Serial.printf("📊 Решетка: %dx%d (%d сензора)\n", ROWS, COLS, TOTAL_SENSORS);
      Serial.printf("🔍 Открити сензори: %d\n", discoveredCount);
      Serial.printf("📚 Мапинг: %s\n", mappingComplete ? "зареден" : "не е зареден");
      Serial.printf("🔥 Learning mode: %s\n", learningMode ? "активен" : "неактивен");
      Serial.println("────────────────────────");

    } else if (command.startsWith("SET WIFI ")) {
      String args = command.substring(9);
      int spaceIdx = args.indexOf(' ');
      if (spaceIdx > 0) {
        String ssid = args.substring(0, spaceIdx);
        String pass = args.substring(spaceIdx + 1);
        strncpy(WIFI_SSID, ssid.c_str(), sizeof(WIFI_SSID) - 1);
        strncpy(WIFI_PASSWORD, pass.c_str(), sizeof(WIFI_PASSWORD) - 1);
        Serial.printf("✓ WiFi: %s\n", WIFI_SSID);
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.println("📡 Преконектване...");
      } else {
        Serial.println("✗ Формат: SET WIFI ssid password");
      }

    } else if (command.startsWith("SET SERVER ")) {
      String url = command.substring(11);
      strncpy(SERVER_URL, url.c_str(), sizeof(SERVER_URL) - 1);
      Serial.printf("✓ Сървър: %s\n", SERVER_URL);

    } else if (command.startsWith("SET INTERVAL ")) {
      unsigned long sec = command.substring(13).toInt();
      if (sec >= 10) {
        SEND_PERIOD = sec * 1000;
        MEASUREMENT_PERIOD = SEND_PERIOD;
        Serial.printf("✓ Интервал: %lu сек\n", sec);
      } else {
        Serial.println("✗ Минимален интервал: 10 секунди");
      }

    } else {
      Serial.println("✗ Непозната команда. Въведете HELP");
    }
  }
}
