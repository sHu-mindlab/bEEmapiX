/*
 * bEEmapiX-C3 v0.1 — Температурен мониторинг с ESP32-C3 (AUTO-LEARNING)
 *
 * Описание:
 *   Firmware за ESP32-C3, който управлява решетка от 3x7 (21 броя) DS18B20
 *   температурни сензора, свързани на обща OneWire шина (GPIO8).
 *   При първо включване системата влиза в режим на учене (Learning Mode) —
 *   потребителят загрява сензорите един по един с ръка, а firmware-ът
 *   автоматично разпознава кой сензор къде се намира в решетката.
 *   Научената карта се запазва в енергонезависимата памет (NVS/Preferences),
 *   така че при следващо захранване устройството директно минава в работен режим.
 *
 * Работен режим:
 *   - Измерва температурите от всички 21 сензора на зададен интервал
 *   - Изпраща JSON POST към отдалечен HTTPS сървър с цялата температурна
 *     матрица и статистики (min/max/avg)
 *   - Сървърът може да промени интервала на изпращане чрез отговор
 *     с JSON {"edit":1, "interval":120}  (стойност в секунди, мин. 10)
 *
 * WiFi:
 *   - Свързва се при стартиране, ако загуби връзка — автоматично
 *     се преконектва на всеки 30 секунди (WiFi Watchdog)
 *
 * Device ID:
 *   - Генерира уникален хексадецимален идентификатор от вградения
 *     eFuse MAC адрес на чипа (обфускиран с множител)
 *
 * Serial команди (Serial Monitor, 115200 baud):
 *   HELP                  — списък на всички команди
 *   SHOW                  — показва текущите настройки и статус
 *   SET WIFI ssid pass    — сменя WiFi мрежата и се преконектва
 *   SET SERVER url        — сменя URL адреса на сървъра
 *   SET INTERVAL секунди  — сменя интервала на изпращане (мин. 10 сек)
 *   RESET                 — изтрива научения сензорен мапинг от паметта
 *
 * Хардуер:
 *   - ESP32-C3 (напр. ESP32-C3-DevKitM-1)
 *   - 21x DS18B20 на обща OneWire шина, GPIO8, с 4.7kΩ pull-up
 *
 * Зависимости (Arduino библиотеки):
 *   - OneWire
 *   - DallasTemperature
 *   - ArduinoJson (v6)
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
// WiFi и сървър идват от config.h (не се качва в git — виж config.h.example)

#include "config.h"

char WIFI_SSID[33] = "Hotspot_10";          // Текущо SSID (може да се смени с SET WIFI)
char WIFI_PASSWORD[65] = "dedaznam";   // Текуща парола
char SERVER_URL[129] = "https://koshx-hub-production.up.railway.app/api/telemetry";        // /URL за POST заявките

#define ONE_WIRE_BUS 8      // GPIO пин за OneWire шината
#define ROWS 3              // Брой редове в сензорната решетка
#define COLS 7              // Брой колони в сензорната решетка
#define TOTAL_SENSORS 21    // ROWS * COLS — общ брой сензори

unsigned long SEND_PERIOD = 10000;              // Интервал за изпращане към сървъра (ms)
unsigned long MEASUREMENT_PERIOD = SEND_PERIOD; // Интервал за измерване (синхронизиран със SEND_PERIOD)


// ============ ГЛОБАЛНИ ПРОМЕНЛИВИ ============

OneWire oneWire(ONE_WIRE_BUS);              // OneWire шина на зададения GPIO
DallasTemperature sensors(&oneWire);        // DallasTemperature библиотека върху шината

Preferences preferences;                    // NVS хранилище за запазване на мапинга между рестарти

DeviceAddress sensorMap[ROWS][COLS];        // Научен мапинг: позиция [ред][колона] → 8-байтов адрес на сензор
bool mappingComplete = false;               // true когато всички 21 позиции са научени и запазени

float temperatureGrid[ROWS][COLS];          // Последно измерените температури (°C), -127.0 = невалидна

// --- Калибрация ---
float calibrationOffset[ROWS][COLS];        // Offset за всеки сензор (avg - raw), 0.0 = некалибриран
bool calibrationLoaded = false;             // true ако калибрацията е заредена от NVS

unsigned long lastMeasurementTime = 0;      // millis() на последното измерване
unsigned long lastSendTime = 0;             // millis() на последното изпращане

char serialNumBuffer[17] = "";              // Уникален device ID (hex стринг от eFuse MAC)

// --- Learning mode ---
bool learningMode = false;                  // true докато системата учи позициите
int learnedSensors = 0;                     // Колко сензора са научени до момента
float baselineTemps[TOTAL_SENSORS];         // Базови температури преди загряване (за сравнение)
DeviceAddress discoveredAddresses[TOTAL_SENSORS]; // Всички открити адреси на OneWire шината
int discoveredCount = 0;                    // Брой реално открити сензори на шината

// ============ DEVICE ID ФУНКЦИЯ ============
// Генерира уникален hex идентификатор от вградения eFuse MAC адрес.
// MAC се умножава по 13 за обфускация, резултатът се форматира като hex стринг.

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
  Serial.setTimeout(100);  // readStringUntil timeout: 100ms вместо 1000ms по подразбиране
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

  // Зареждане на калибрация от паметта
  loadCalibration();

  Serial.println("\n✓ Системата е готова!\n");
}

// ============ LOOP ============

void loop() {
  unsigned long currentTime = millis();
  
  // WiFi Watchdog — ако връзката е паднала, опитва преконектване на всеки 30 сек
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiTry = 0;
    if (currentTime - lastWifiTry > 30000) {
      lastWifiTry = currentTime;
      Serial.println("⚠ WiFi връзка загубена. Опит за преконектване...");
      WiFi.reconnect();
    }
  }

  if (learningMode) {
    // В режим учене — проверява за загрят сензор всяка секунда
    checkForHeatSignal();
  } else {
    // Нормална работа — периодично измерване и изпращане
    if (currentTime - lastMeasurementTime >= MEASUREMENT_PERIOD) {
      lastMeasurementTime = currentTime;
      measureTemperatures();
    }

    if (currentTime - lastSendTime >= SEND_PERIOD) {
      lastSendTime = currentTime;
      sendData();
    }
  }

  // Обработка на Serial команди (ESP32 не извиква serialEvent автоматично)
  serialEvent();
  delay(10);
}

// ============ WiFi ФУНКЦИИ ============

// Свързване към WiFi мрежата. Опитва до 30 пъти (15 сек) и продължава дори без WiFi.
void setupWiFi() {
  Serial.printf("📡 Свързване към WiFi: %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);  // ESP32 ще опитва reconnect автоматично на ниско ниво
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
    Serial.println("\n✗ WiFi неуспешен! (Watchdog ще опитва на всеки 30 сек)");
  }
}

// ============ СЕНЗОРНИ ФУНКЦИИ ============

// Открива всички DS18B20 сензори на OneWire шината, задава им резолюция 12-bit
// и нулира температурната матрица и сензорната карта.
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

  // Запазване на всички открити адреси и задаване на 12-bit резолюция (0.0625°C)
  for (int i = 0; i < discoveredCount && i < TOTAL_SENSORS; i++) {
    if (sensors.getAddress(discoveredAddresses[i], i)) {
      sensors.setResolution(discoveredAddresses[i], 12);
      Serial.printf("  Сензор %d: ", i);
      printAddress(discoveredAddresses[i]);
      Serial.println();
    }
  }

  // Нулиране на температурната матрица и сензорната карта
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      temperatureGrid[row][col] = -127.0;   // -127 = няма данни
      for (int b = 0; b < 8; b++) {
        sensorMap[row][col][b] = 0;          // Празен адрес = позицията не е научена
      }
    }
  }
}

// Проверява NVS паметта за запазен сензорен мапинг.
// Ако има — зарежда го и преминава в работен режим.
// Ако няма — стартира Learning Mode и чака потребителят да загрява сензорите.
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
        char key[6];
        snprintf(key, sizeof(key), "%d_%d", row, col);
        preferences.getBytes(key, sensorMap[row][col], 8);
        
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

// Стартира режим на учене: записва текущите температури като базова линия,
// след което чака потребителят да загрява сензорите един по един.
void startLearningMode() {
  learningMode = true;
  learnedSensors = 0;

  // Измерване на базовите температури — спрямо тях ще се засича загряване
  sensors.requestTemperatures();
  delay(1000);
  
  for (int i = 0; i < discoveredCount; i++) {
    baselineTemps[i] = sensors.getTempC(discoveredAddresses[i]);
  }
  
  Serial.println("📊 Базови температури записани.");
  Serial.printf("\n🔥 Загрейте сензор за позиция [%d,%d]...\n", 
                learnedSensors / COLS, learnedSensors % COLS);
}

// Извиква се в loop() докато learningMode == true.
// На всяка секунда чете всички сензори и търси такъв с покачване > 1.5°C
// спрямо базовата линия. Ако намери — записва адреса му на следващата позиция.
void checkForHeatSignal() {
  static unsigned long lastCheck = 0;

  if (millis() - lastCheck < 1000) return;
  lastCheck = millis();

  sensors.requestTemperatures();
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

// Извиква се когато всички 21 сензора са научени.
// Записва целия мапинг в NVS паметта и преминава в работен режим.
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
      char key[6];
      snprintf(key, sizeof(key), "%d_%d", row, col);
      preferences.putBytes(key, sensorMap[row][col], 8);
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

// ============ КАЛИБРАЦИЯ ============

// Зарежда калибрационните offsets от NVS. Ако няма запазени — остават 0.0.
void loadCalibration() {
  for (int row = 0; row < ROWS; row++)
    for (int col = 0; col < COLS; col++)
      calibrationOffset[row][col] = 0.0;

  Preferences calPrefs;
  calPrefs.begin("sensor-cal", true);  // read-only
  calibrationLoaded = calPrefs.getBool("done", false);

  if (calibrationLoaded) {
    for (int row = 0; row < ROWS; row++) {
      for (int col = 0; col < COLS; col++) {
        char key[6];
        snprintf(key, sizeof(key), "c%d_%d", row, col);
        calibrationOffset[row][col] = calPrefs.getFloat(key, 0.0);
      }
    }
    Serial.println("✓ Калибрация заредена от паметта.");
  } else {
    Serial.println("⚠ Няма запазена калибрация (offsets = 0).");
  }

  calPrefs.end();
}

// Чете всички сензори, изчислява средна температура и записва offset за всеки.
// offset = средна - сурова стойност. Запазва в NVS.
void runCalibration() {
  if (!mappingComplete) {
    Serial.println("✗ Мапингът не е завършен. Първо завършете Learning Mode.");
    return;
  }

  Serial.println("\n─── Калибрация ───");
  Serial.println("Измерване...");

  sensors.requestTemperatures();

  float rawTemps[ROWS][COLS];
  float sum = 0.0;
  int count = 0;

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
    Serial.println("✗ Твърде малко валидни сензори за калибрация.");
    return;
  }

  float avg = sum / count;
  Serial.printf("📊 Средна температура: %.2f°C (%d сензора)\n\n", avg, count);

  // Изчисляване и запазване на offsets
  Preferences calPrefs;
  calPrefs.begin("sensor-cal", false);

  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      if (rawTemps[row][col] > DEVICE_DISCONNECTED_C) {
        calibrationOffset[row][col] = avg - rawTemps[row][col];
        char key[6];
        snprintf(key, sizeof(key), "c%d_%d", row, col);
        calPrefs.putFloat(key, calibrationOffset[row][col]);
        Serial.printf("  [%d,%d]: %.2f°C → offset %+.2f°C\n",
                      row, col, rawTemps[row][col], calibrationOffset[row][col]);
      } else {
        calibrationOffset[row][col] = 0.0;
      }
    }
  }

  calPrefs.putBool("done", true);
  calPrefs.end();

  calibrationLoaded = true;
  Serial.println("\n✅ Калибрацията е запазена в паметта!");
}

// Чете температурите от всички научени позиции и ги записва в temperatureGrid.
// Позиции без научен сензор получават -127.0 (невалидно).
void measureTemperatures() {
  Serial.println("\n─── Измерване на температури ───");
  
  sensors.requestTemperatures();
  
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      if (!isAddressEmpty(sensorMap[row][col])) {
        float temp = sensors.getTempC(sensorMap[row][col]);
        if (temp <= DEVICE_DISCONNECTED_C) temp = -127.0;  // Невалидно четене
        else temp += calibrationOffset[row][col];           // Прилагане на калибрация
        temperatureGrid[row][col] = temp;
        Serial.printf("[%d,%d]: %.2f°C\n", row, col, temp);
      } else {
        temperatureGrid[row][col] = -127.0;
      }
    }
  }
}

// Изгражда JSON с температурната матрица и статистики, изпраща го като HTTPS POST.
// Формат: { dev_num, timestamp, interval, rows, cols, temperature_grid[][], statistics{} }
// След успешен отговор проверява дали сървърът иска промяна на интервала:
//   отговор {"edit":1, "interval":120} → SEND_PERIOD = 120 сек (мин. 10 сек защита)
void sendData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("✗ WiFi не е свързан");
    return;
  }

  Serial.println("\n─── Изпращане на данни ───");

  StaticJsonDocument<2048> doc;

  doc["dev_num"] = serialNumBuffer;       // Уникален ID на устройството
  doc["device_type"] = "heatmap";
  doc["version"] = "0.1";
  doc["timestamp"] = millis();            // Време от стартирането (ms)
  doc["interval"] = SEND_PERIOD / 1000;   // Текущ интервал на изпращане (сек)
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
      if (temp > DEVICE_DISCONNECTED_C) {
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
  
  // HTTPS заявка (setInsecure пропуска проверка на сертификата — OK за тестване)
  WiFiClientSecure client;
  client.setInsecure();
  
  HTTPClient http;
  http.begin(client, SERVER_URL);
  http.setTimeout(10000);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    Serial.printf("✓ HTTP: %d\n", httpResponseCode);
    String response = http.getString();
    Serial.println("Отговор: " + response);

    // Парсване на отговора — сървърът може да промени интервала дистанционно
    StaticJsonDocument<256> respDoc;
    if (deserializeJson(respDoc, response) == DeserializationError::Ok) {
      if (respDoc["edit"] == 1 && respDoc.containsKey("interval")) {
        unsigned long newInterval = respDoc["interval"].as<unsigned long>() * 1000;
        if (newInterval < 10000) newInterval = 10000;  // Защита: минимум 10 секунди
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

// Проверява дали 8-байтов DS18B20 адрес е празен (всички нули)
bool isAddressEmpty(DeviceAddress addr) {
  for (int i = 0; i < 8; i++) {
    if (addr[i] != 0) return false;
  }
  return true;
}

// Сравнява два 8-байтови DS18B20 адреса
bool addressesMatch(DeviceAddress addr1, DeviceAddress addr2) {
  for (int i = 0; i < 8; i++) {
    if (addr1[i] != addr2[i]) return false;
  }
  return true;
}

// Отпечатва 8-байтов адрес като HEX (напр. 28:FF:A1:32:00:00:00:5C)
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(":");
  }
}

// ============ SERIAL COMMANDS ============
// Обработва команди от Serial Monitor. Извиква се ръчно от loop(),
// защото ESP32 не поддържа автоматично извикване на serialEvent().

void serialEvent() {
  while (Serial.available()) {
    String raw = Serial.readStringUntil('\n');
    raw.trim();
    if (raw.length() == 0) continue;       // Игнориране на празни редове
    String command = raw;
    command.toUpperCase();                  // За сравнение (case-insensitive)

    if (command == "HELP") {
      Serial.println("\n─── Команди ───");
      Serial.println("SHOW                  - текущи настройки");
      Serial.println("CALIBRATE             - калибрация на сензорите");
      Serial.println("RESET CAL             - изтриване само на калибрацията");
      Serial.println("SET WIFI ssid pass    - смяна на WiFi мрежа");
      Serial.println("SET SERVER url        - смяна на сървър");
      Serial.println("SET INTERVAL секунди  - смяна на интервал (мин 10)");
      Serial.println("RESET                 - изтриване на мапинг и калибрация");
      Serial.println("HELP                  - тази помощ");
      Serial.println("───────────────────────");

    } else if (command == "CALIBRATE") {
      runCalibration();

    } else if (command == "RESET CAL") {
      Serial.println("\n⚠ Изтриване на калибрацията...");
      Preferences calPrefs;
      calPrefs.begin("sensor-cal", false);
      calPrefs.clear();
      calPrefs.end();
      calibrationLoaded = false;
      for (int row = 0; row < ROWS; row++)
        for (int col = 0; col < COLS; col++)
          calibrationOffset[row][col] = 0.0;
      Serial.println("✓ Калибрацията е изтрита. Offsets = 0.");

    } else if (command == "RESET") {
      Serial.println("\n⚠ Изтриване на мапинг и калибрация...");
      preferences.begin("sensor-map", false);
      preferences.clear();
      preferences.end();
      Preferences calPrefs;
      calPrefs.begin("sensor-cal", false);
      calPrefs.clear();
      calPrefs.end();
      calibrationLoaded = false;
      Serial.println("✓ Всичко е изтрито. Рестартирайте устройството.");

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
      Serial.printf("🎯 Калибрация: %s\n", calibrationLoaded ? "заредена" : "няма");
      Serial.printf("🔥 Learning mode: %s\n", learningMode ? "активен" : "неактивен");
      Serial.printf("💾 Free heap: %u bytes\n", ESP.getFreeHeap());
      Serial.println("────────────────────────");

    } else if (command.startsWith("SET WIFI ")) {
      String args = raw.substring(9);      // Оригинал — пароли са case-sensitive
      int spaceIdx = args.indexOf(' ');
      if (spaceIdx > 0) {
        String ssid = args.substring(0, spaceIdx);
        String pass = args.substring(spaceIdx + 1);
        strncpy(WIFI_SSID, ssid.c_str(), sizeof(WIFI_SSID) - 1);
        WIFI_SSID[sizeof(WIFI_SSID) - 1] = '\0';
        strncpy(WIFI_PASSWORD, pass.c_str(), sizeof(WIFI_PASSWORD) - 1);
        WIFI_PASSWORD[sizeof(WIFI_PASSWORD) - 1] = '\0';
        Serial.printf("✓ WiFi: %s\n", WIFI_SSID);
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.println("📡 Преконектване...");
      } else {
        Serial.println("✗ Формат: SET WIFI ssid password");
      }

    } else if (command.startsWith("SET SERVER ")) {
      String url = raw.substring(11);      // Оригинал — URL е case-sensitive
      strncpy(SERVER_URL, url.c_str(), sizeof(SERVER_URL) - 1);
      SERVER_URL[sizeof(SERVER_URL) - 1] = '\0';
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
