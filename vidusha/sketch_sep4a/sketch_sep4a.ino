#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include "secrets.h"
#include "esp_task_wdt.h"

// ===== Debug helper =====
#define DEBUG 1
#define DBG_PRINT(x)   do { if (DEBUG) Serial.print(x); } while(0)
#define DBG_PRINTLN(x) do { if (DEBUG) Serial.println(x); } while(0)
#define DBG_PRINTF(...) do { if (DEBUG) Serial.printf(__VA_ARGS__); } while(0)

// ===== Mode definitions =====
#define MODE_TRANSIT 0
#define MODE_NORMAL  1
#define MODE_HIGH    2

// ===== Configuration Constants =====
const unsigned long WIFI_TIMEOUT_MS = 15000;
const unsigned long MQTT_TIMEOUT_MS = 30000;
const unsigned long MQTT_PUBLISH_INTERVAL_MS = 10000;
const unsigned long WATCHDOG_TIMEOUT_S = 10000;
const unsigned long WIFI_RETRY_INTERVAL_MS = 30000;
const size_t SERIAL_BUFFER_SIZE = 64;
const unsigned long GPS_TIMEOUT_MS = 5000;

// ===== Pins =====
const int BUZ_PIN = 25;
const int LED_PIN = 13;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int GPS_RX2 = 16;
const int GPS_TX2 = 17;

// ===== System State Variables =====
volatile bool protectionEnabled = false;
volatile bool motionProtectionEnabled = false;
volatile bool distanceAlertsEnabled = false;
volatile int maxDistanceThreshold = 10;
volatile bool deviceConnected = false;

String currentSensitivity = "NORMAL";
SemaphoreHandle_t settingsMutex;

// ===== WiFi/MQTT State =====
enum WiFiState { WIFI_DISCONNECTED, WIFI_CONNECTING, WIFI_CONNECTED };

// Custom MQTT state enum to avoid PubSubClient macro conflicts
enum MQTTStateCustom { 
    MQTT_STATE_DISCONNECTED, 
    MQTT_STATE_CONNECTING, 
    MQTT_STATE_CONNECTED 
};

volatile MQTTStateCustom mqttState = MQTT_STATE_DISCONNECTED;

volatile WiFiState wifiState = WIFI_DISCONNECTED;

unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;

// ===== Tether alarm (BLE disconnect) =====
unsigned long lastDisconnectedAt = 0;
unsigned long GRACE_MS = 2500;
bool alarmTriggered = false;

// ===== Siren =====
int SIREN_F1 = 900;
int SIREN_F2 = 1400;
unsigned long SIREN_STEP_MS = 300;
bool alarmOn = false;
bool prevAlarmOn = false;
bool sirenAtF1 = true;
unsigned long lastSirenStep = 0;

// ===== Motion detection =====
uint8_t  MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const float   ACC_LSB_PER_G    = 16384.0f;

volatile float     JERK_THR_G_PER_S = 1.00f;
volatile float     STD_THR_G        = 0.065f;
volatile float     TILT_THR_DEG     = 40.0f;
volatile uint16_t  HOLD_MS          = 220;
volatile uint16_t  WINDOW_MS        = 700;
volatile uint16_t  READ_PERIOD_MS   = 15;

struct Sample { float dyn; float pitch; float roll; uint32_t t; };
const int MAX_SAMPLES = 128;
Sample ringBuf[MAX_SAMPLES];
volatile int rbHead = 0, rbCount = 0;

float    lastMag = 1.0f;
uint32_t lastReadMs = 0;
uint32_t motionOverSince = 0;
uint32_t lastMotionNotify = 0;
const uint32_t REFRACTORY_MS = 4000;
bool motionInProgress = false;

// ===== GPS =====
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);
double lastLat = 0.0, lastLng = 0.0;
unsigned long lastGpsUpdate = 0;

// ===== BLE globals =====
BLEServer*          gServer   = nullptr;
BLECharacteristic*  gCharEvt  = nullptr;
BLECharacteristic*  gCharGps  = nullptr;
class MyServerCallbacks;
class MyCharacteristicCallbacks;
MyServerCallbacks* serverCallbacks = nullptr;
MyCharacteristicCallbacks* charCallbacks = nullptr;

// ===== Preferences =====
Preferences preferences;

// ===== WiFi / MQTT =====
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// ===== Serial Command Buffer =====
char serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialBufferPos = 0;

// ===== Power Management =====
void enterLightSleep(uint32_t sleepTimeMs) {
  if (sleepTimeMs < 10) return;
  esp_sleep_enable_timer_wakeup(sleepTimeMs * 1000);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  esp_light_sleep_start();
}

// ===== Helpers =====
float toDeg(float r){ return r * 57.29578f; }

// ===== Buzzer =====
void buzzerInit() { 
  pinMode(BUZ_PIN, OUTPUT); 
  digitalWrite(BUZ_PIN, LOW); 
}

inline void buzzerTone(uint16_t freq) { tone(BUZ_PIN, freq); }
inline void buzzerOff() { noTone(BUZ_PIN); digitalWrite(BUZ_PIN, LOW); }

// ===== Forward declarations =====
void setProfile(uint8_t mode);

// ===== Error Recovery =====
void handleCriticalError(const char* error) {
  DBG_PRINTF("[ERROR] Critical error: %s\n", error);
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
  ESP.restart();
}

// ===== WiFi Management =====
void setupWiFi() {
  if (wifiState != WIFI_DISCONNECTED) return;
  unsigned long startTime = millis();
  wifiState = WIFI_CONNECTING;
  lastWifiAttempt = startTime;
  
  DBG_PRINTLN("[WIFI] Attempting connection...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
    esp_task_wdt_reset();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiState = WIFI_CONNECTED;
    DBG_PRINTLN("\n[WIFI] Connected, IP=" + WiFi.localIP().toString());
  } else {
    wifiState = WIFI_DISCONNECTED;
    DBG_PRINTLN("\n[WIFI] Connection timeout, will retry later");
    WiFi.disconnect();
  }
}

void checkWiFiConnection() {
  unsigned long now = millis();
  switch (wifiState) {
    case WIFI_DISCONNECTED:
      if (now - lastWifiAttempt > WIFI_RETRY_INTERVAL_MS) setupWiFi();
      break;
    case WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) wifiState = WIFI_CONNECTED;
      else if (now - lastWifiAttempt > WIFI_TIMEOUT_MS) {
        wifiState = WIFI_DISCONNECTED;
        DBG_PRINTLN("[WIFI] Connection attempt timed out");
        WiFi.disconnect();
      }
      break;
    case WIFI_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        wifiState = WIFI_DISCONNECTED;
        mqttState = MQTT_STATE_DISCONNECTED;
        DBG_PRINTLN("[WIFI] Connection lost");
      }
      break;
  }
}

// ===== MQTT Management =====
bool testTcpConnect() {
  if (wifiState != WIFI_CONNECTED) return false;
  DBG_PRINTF("[TCP] Testing connection to %s:%d\n", MQTT_SERVER, MQTT_PORT);
  espClient.stop();
  bool connected = espClient.connect(MQTT_SERVER, MQTT_PORT);
  espClient.stop();
  if (connected) DBG_PRINTLN("[TCP] Connection test successful");
  else DBG_PRINTLN("[TCP] Connection test failed");
  return connected;
}

void connectMQTT() {
  if (wifiState != WIFI_CONNECTED || mqttState != MQTT_STATE_DISCONNECTED) return;
  unsigned long startTime = millis();
  mqttState = MQTT_STATE_CONNECTING;
  lastMqttAttempt = startTime;
  if (!testTcpConnect()) { mqttState = MQTT_STATE_DISCONNECTED; return; }
  
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  DBG_PRINTLN("[MQTT] Attempting connection...");
  if (mqttClient.connect("SmartBagClient", HIVEMQ_USER, HIVEMQ_PASS)) {
    mqttState = MQTT_STATE_CONNECTED;
    DBG_PRINTLN("[MQTT] Connected to HiveMQ");
  } else {
    mqttState = MQTT_STATE_DISCONNECTED;
    DBG_PRINTF("[MQTT] Connection failed, rc=%d\n", mqttClient.state());
  }
}

void checkMQTTConnection() {
  unsigned long now = millis();
  switch (mqttState) {
    case MQTT_STATE_DISCONNECTED:
      if (wifiState == WIFI_CONNECTED && now - lastMqttAttempt > WIFI_RETRY_INTERVAL_MS) connectMQTT();
      break;
    case MQTT_STATE_CONNECTING:
      if (mqttClient.connected()) mqttState = MQTT_STATE_CONNECTED;
      else if (now - lastMqttAttempt > MQTT_TIMEOUT_MS) {
        mqttState = MQTT_STATE_DISCONNECTED;
        DBG_PRINTLN("[MQTT] Connection attempt timed out");
      }
      break;
    case MQTT_STATE_CONNECTED:
      if (!mqttClient.connected()) {
        mqttState = MQTT_STATE_DISCONNECTED;
        DBG_PRINTLN("[MQTT] Connection lost");
      }
      break;
  }
}

void publishLocation() {
  if (mqttState != MQTT_STATE_CONNECTED) { DBG_PRINTLN("[MQTT] Not connected - cannot publish"); return; }
  mqttClient.loop();
  char msg[128];
  if (gps.location.isValid()) snprintf(msg, sizeof(msg),
           "{\"lat\":%.6f,\"lon\":%.6f,\"protection\":true,\"timestamp\":%lu}",
           gps.location.lat(), gps.location.lng(), millis());
  else snprintf(msg, sizeof(msg),
           "{\"fix\":false,\"protection\":true,\"timestamp\":%lu}", millis());

  if (mqttClient.publish("smartbag/location", msg)) DBG_PRINTLN("[MQTT] Published: " + String(msg));
  else { DBG_PRINTLN("[MQTT] Publish failed"); mqttState = MQTT_STATE_DISCONNECTED; }
}

// ===== Settings Storage with Error Handling =====
bool saveSettings() {
  if (xSemaphoreTake(settingsMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    DBG_PRINTLN("[SETTINGS] Failed to acquire mutex for save");
    return false;
  }
  
  bool success = true;
  if (!preferences.begin("securecase", false)) {
    DBG_PRINTLN("[SETTINGS] Failed to open preferences for writing");
    success = false;
  } else {
    success &= preferences.putBool("protection", protectionEnabled);
    success &= preferences.putBool("motion", motionProtectionEnabled);
    success &= preferences.putBool("distance", distanceAlertsEnabled);
    success &= preferences.putInt("maxDist", maxDistanceThreshold);
    success &= preferences.putString("sensitivity", currentSensitivity);
    preferences.end();
    
    if (success) {
      DBG_PRINTLN("[SETTINGS] Saved successfully");
    } else {
      DBG_PRINTLN("[SETTINGS] Save operation failed");
    }
  }
  
  xSemaphoreGive(settingsMutex);
  return success;
}

bool loadSettings() {
  if (xSemaphoreTake(settingsMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    DBG_PRINTLN("[SETTINGS] Failed to acquire mutex for load");
    return false;
  }
  
  bool success = true;
  if (!preferences.begin("securecase", true)) {
    DBG_PRINTLN("[SETTINGS] Failed to open preferences for reading, using defaults");
    success = false;
  } else {
    protectionEnabled = preferences.getBool("protection", false);
    motionProtectionEnabled = preferences.getBool("motion", false);
    distanceAlertsEnabled = preferences.getBool("distance", false);
    maxDistanceThreshold = preferences.getInt("maxDist", 10);
    currentSensitivity = preferences.getString("sensitivity", "NORMAL");
    preferences.end();
    
    DBG_PRINTLN("[SETTINGS] Loaded:");
    DBG_PRINTF("  Protection=%d Motion=%d Distance=%d MaxDist=%d Sensitivity=%s\n",
               protectionEnabled, motionProtectionEnabled, distanceAlertsEnabled,
               maxDistanceThreshold, currentSensitivity.c_str());
  }
  
  xSemaphoreGive(settingsMutex);
  return success;
}

// ===== Command Processing =====
void processCommand(String command) {
  DBG_PRINTLN("[CMD] Received: " + command);

  if (command == "PROTECTION_ON") {
    protectionEnabled = true;
    alarmTriggered = false; // Reset alarm trigger state
    DBG_PRINTLN("[CMD] Protection ENABLED");
  }
  else if (command == "PROTECTION_OFF") {
    protectionEnabled = false;
    motionProtectionEnabled = false;
    alarmOn = false;
    alarmTriggered = false;
    buzzerOff();
    digitalWrite(LED_PIN, LOW);
    DBG_PRINTLN("[CMD] Protection DISABLED");
  }
  else if (command == "MOTION_ON") {
    if (protectionEnabled) {
      motionProtectionEnabled = true;
      DBG_PRINTLN("[CMD] Motion detection ENABLED");
    } else {
      DBG_PRINTLN("[CMD] Ignored MOTION_ON (protection disabled)");
    }
  }
  else if (command == "MOTION_OFF") {
    motionProtectionEnabled = false;
    DBG_PRINTLN("[CMD] Motion detection DISABLED");
  }
  else if (command == "SENSITIVITY_LOW") {
    currentSensitivity = "LOW";
    setProfile(MODE_TRANSIT);
  }
  else if (command == "SENSITIVITY_NORMAL") {
    currentSensitivity = "NORMAL";
    setProfile(MODE_NORMAL);
  }
  else if (command == "SENSITIVITY_HIGH") {
    currentSensitivity = "HIGH";
    setProfile(MODE_HIGH);
  }
  else if (command.startsWith("DISTANCE_ON_")) {
    String distStr = command.substring(12);
    int newThreshold = distStr.toInt();
    if (newThreshold > 0 && newThreshold <= 1000) { // Sanity check
      maxDistanceThreshold = newThreshold;
      distanceAlertsEnabled = true;
      DBG_PRINTF("[CMD] Distance alerts ENABLED, max=%dm\n", maxDistanceThreshold);
    } else {
      DBG_PRINTLN("[CMD] Invalid distance threshold");
      return;
    }
  }
  else if (command == "DISTANCE_OFF") {
    distanceAlertsEnabled = false;
    DBG_PRINTLN("[CMD] Distance alerts DISABLED");
  }
  else if (command == "STATUS") {
    // Send status update
    if (deviceConnected && gCharEvt) {
      char status[256];
      snprintf(status, sizeof(status),
               "STATUS:{\"protection\":%s,\"motion\":%s,\"distance\":%s,\"sensitivity\":\"%s\",\"wifi\":%s,\"mqtt\":%s}",
               protectionEnabled ? "true" : "false",
               motionProtectionEnabled ? "true" : "false", 
               distanceAlertsEnabled ? "true" : "false",
               currentSensitivity.c_str(),
               wifiState == WIFI_CONNECTED ? "true" : "false",
               mqttState == MQTT_STATE_CONNECTED ? "true" : "false");
      gCharEvt->setValue(status);
      gCharEvt->notify();
    }
    return; // Don't save settings for status request
  }
  else {
    DBG_PRINTLN("[CMD] Unknown command: " + command);
    return;
  }

  if (!saveSettings()) {
    DBG_PRINTLN("[CMD] Warning: Failed to save settings");
  }
}

// ===== BLE callbacks with proper cleanup =====
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    alarmOn = false;
    alarmTriggered = false;
    buzzerOff();
    digitalWrite(LED_PIN, LOW);
    DBG_PRINTF("[BLE] Connected at %lu ms\n", millis());
  }
  
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    lastDisconnectedAt = millis();
    DBG_PRINTF("[BLE] Disconnected at %lu ms\n", lastDisconnectedAt);
    
    // Restart advertising with error handling
    BLEAdvertising* advertising = pServer->getAdvertising();
    if (advertising) {
      advertising->start();
    } else {
      DBG_PRINTLN("[BLE] Warning: Could not restart advertising");
    }
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();   // Arduino String
    std::string stdValue = value.c_str();         // convert to std::string
    
    if (value.length() > 0 && value.length() < 64) { // Sanity check
      processCommand(value);
    } else {
      DBG_PRINTLN("[BLE] Invalid command length");
    }
  }
};

// ===== I2C / MPU with Error Handling =====
bool mpuWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    DBG_PRINTF("[MPU] Write error %d to reg 0x%02X\n", error, reg);
    return false;
  }
  return true;
}

bool mpuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission(false);
  if (error != 0) {
    DBG_PRINTF("[MPU] Read setup error %d for reg 0x%02X\n", error, reg);
    return false;
  }
  
  uint8_t received = Wire.requestFrom((int)MPU_ADDR, (int)len);
  if (received != len) {
    DBG_PRINTF("[MPU] Read error: requested %d, got %d\n", len, received);
    return false;
  }
  
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool readAccelG(float &ax, float &ay, float &az) {
  uint8_t raw[6];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, raw, 6)) {
    return false;
  }
  
  int16_t rx = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ry = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t rz = (int16_t)((raw[4] << 8) | raw[5]);
  
  ax = (float)rx / ACC_LSB_PER_G;
  ay = (float)ry / ACC_LSB_PER_G;
  az = (float)rz / ACC_LSB_PER_G;
  
  return true;
}

// ===== Siren =====
void runSiren() {
  unsigned long now = millis();

  if (alarmOn && !prevAlarmOn) {
    buzzerTone(SIREN_F1);
    digitalWrite(LED_PIN, HIGH);
    lastSirenStep = now;
    sirenAtF1 = true;
    DBG_PRINTF("[SIREN] START at %lu ms\n", now);
  }
  
  if (!alarmOn && prevAlarmOn) {
    buzzerOff();
    digitalWrite(LED_PIN, LOW);
    DBG_PRINTF("[SIREN] STOP at %lu ms\n", now);
  }
  
  if (alarmOn && now - lastSirenStep >= SIREN_STEP_MS) {
    lastSirenStep = now;
    sirenAtF1 = !sirenAtF1;
    buzzerTone(sirenAtF1 ? SIREN_F1 : SIREN_F2);
    digitalWrite(LED_PIN, sirenAtF1 ? HIGH : LOW);
  }
  
  prevAlarmOn = alarmOn;
}

// ===== Motion Detection with Improved Logic =====
void processMotion(float ax, float ay, float az) {
  uint32_t now = millis();
  float mag = sqrtf(ax*ax + ay*ay + az*az);
  float dyn = fabsf(mag - 1.0f);

  float dt = (lastReadMs == 0) ? 0.01f : (now - lastReadMs) / 1000.0f;
  if (dt <= 0 || dt > 1.0f) dt = 0.01f; // Sanity check
  lastReadMs = now;

  float jerk = fabsf(mag - lastMag) / dt;
  lastMag = mag;

  float pitch = toDeg(atan2f(ax, sqrtf(ay*ay + az*az)));
  float roll  = toDeg(atan2f(ay, sqrtf(ax*ax + az*az)));

  // Thread-safe ring buffer update
  ringBuf[rbHead] = { dyn, pitch, roll, now };
  rbHead = (rbHead + 1) % MAX_SAMPLES;
  if (rbCount < MAX_SAMPLES) rbCount++;

  // Calculate statistics over window
  float mean = 0, m2 = 0;
  int n = 0;
  float minP = +1e9, maxP = -1e9, minR = +1e9, maxR = -1e9;
  
  for (int i = 0; i < rbCount; i++) {
    int idx = (rbHead - 1 - i + MAX_SAMPLES) % MAX_SAMPLES;
    if (now - ringBuf[idx].t > WINDOW_MS) break;
    
    float x = ringBuf[idx].dyn;
    n++;
    float d = x - mean;
    mean += d / n;
    m2 += d * (x - mean);
    
    minP = min(minP, ringBuf[idx].pitch);
    maxP = max(maxP, ringBuf[idx].pitch);
    minR = min(minR, ringBuf[idx].roll);
    maxR = max(maxR, ringBuf[idx].roll);
  }
  
  if (n < 5) return; // Not enough samples

  float stdDyn = sqrtf(m2 / (n - 1));
  float tiltSpan = max(maxP - minP, maxR - minR);

  bool conditionMet = (jerk > JERK_THR_G_PER_S) && 
                     (stdDyn > STD_THR_G) && 
                     (tiltSpan > TILT_THR_DEG);

  if (conditionMet) {
    if (!motionInProgress) {
      motionInProgress = true;
      motionOverSince = now;
    }
  } else {
    motionInProgress = false;
    motionOverSince = 0;
  }

  bool persistent = motionInProgress && (now - motionOverSince >= HOLD_MS);
  bool cooldown = (now - lastMotionNotify) < REFRACTORY_MS;

  if (persistent && !cooldown) {
    lastMotionNotify = now;
    
    // Determine motion type
    String motionType = "MOTION_DETECTED";
    if (tiltSpan > TILT_THR_DEG * 1.5) {
      motionType = "MOTION_TILT";
    } else if (jerk > JERK_THR_G_PER_S * 2.0) {
      motionType = "MOTION_IMPACT";
    } else if (stdDyn > STD_THR_G * 3.0) {
      motionType = "MOTION_SHAKE";
    }

    DBG_PRINTF("[MOTION] Event: %s (j=%.3f, s=%.3f, t=%.1f)\n", 
               motionType.c_str(), jerk, stdDyn, tiltSpan);

    // Notify via BLE if connected
    if (deviceConnected && gCharEvt) {
      gCharEvt->setValue(motionType.c_str());
      gCharEvt->notify();
    }
    
    // Brief feedback
    buzzerTone(1400);
    digitalWrite(LED_PIN, HIGH);
    delay(50); // Reduced delay to prevent blocking
    buzzerOff();
    digitalWrite(LED_PIN, LOW);
    
    motionInProgress = false; // Reset after notification
  }
}

// ===== Profiles =====
void setProfile(uint8_t mode) {
  switch (mode) {
    case MODE_TRANSIT:
      JERK_THR_G_PER_S = 1.40f;
      STD_THR_G        = 0.10f;
      TILT_THR_DEG     = 60.0f;
      HOLD_MS          = 300;
      WINDOW_MS        = 700;
      READ_PERIOD_MS   = 20;
      DBG_PRINTLN("[MODE] TRANSIT (LOW sensitivity)");
      break;
      
    case MODE_HIGH:
      JERK_THR_G_PER_S = 0.70f;
      STD_THR_G        = 0.045f;
      TILT_THR_DEG     = 25.0f;
      HOLD_MS          = 150;
      WINDOW_MS        = 800;
      READ_PERIOD_MS   = 12;
      DBG_PRINTLN("[MODE] HIGH sensitivity");
      break;
      
    case MODE_NORMAL:
    default:
      JERK_THR_G_PER_S = 1.00f;
      STD_THR_G        = 0.065f;
      TILT_THR_DEG     = 40.0f;
      HOLD_MS          = 220;
      WINDOW_MS        = 700;
      READ_PERIOD_MS   = 15;
      DBG_PRINTLN("[MODE] NORMAL sensitivity");
      break;
  }
}

// ===== Serial Command Processing =====
void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serialBufferPos > 0) {
        serialBuffer[serialBufferPos] = '\0';
        String command = String(serialBuffer);
        command.trim();
        
        // Process full commands or single character shortcuts
        if (command.length() == 1) {
          char shortcut = command.charAt(0);
          switch (shortcut) {
            case '1': processCommand("PROTECTION_ON"); break;
            case '0': processCommand("PROTECTION_OFF"); break;
            case 'm': processCommand("MOTION_ON"); break;
            case 'n': processCommand("MOTION_OFF"); break;
            case 'l': processCommand("SENSITIVITY_LOW"); break;
            case 'k': processCommand("SENSITIVITY_NORMAL"); break;
            case 'h': processCommand("SENSITIVITY_HIGH"); break;
            case 'd': processCommand("DISTANCE_OFF"); break;
            case 's': processCommand("STATUS"); break;
            default:
              DBG_PRINTLN("[SERIAL] Unknown shortcut: " + String(shortcut));
              break;
          }
        } else {
          processCommand(command);
        }
        
        serialBufferPos = 0;
      }
    } else if (c >= 32 && c <= 126) { // Printable ASCII
      if (serialBufferPos < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[serialBufferPos++] = c;
      } else {
        DBG_PRINTLN("[SERIAL] Buffer overflow, resetting");
        serialBufferPos = 0;
      }
    }
  }
}

// ===== GPS Processing with Timeout Handling =====
void processGPS() {
  unsigned long now = millis();
  bool dataProcessed = false;
  
  // Process available GPS data
  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      dataProcessed = true;
      lastGpsUpdate = now;
    }
  }
  
  // Handle GPS updates
  if (gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    
    if (deviceConnected && gCharGps) {
      char buf[128];
      snprintf(buf, sizeof(buf),
               "{\"fix\":true,\"lat\":%.6f,\"lng\":%.6f,\"satellites\":%d,\"hdop\":%.2f}",
               lastLat, lastLng, gps.satellites.value(), gps.hdop.hdop());
      gCharGps->setValue(buf);
      gCharGps->notify();
    }
    
    DBG_PRINTF("[GPS] Location updated: %.6f,%.6f (sats=%d)\n", 
               lastLat, lastLng, gps.satellites.value());
  }
  
  // Handle loss of GPS fix
  static bool lastFixValid = false;
  bool currentFixValid = gps.location.isValid();
  
  if (!currentFixValid && (lastFixValid || (now - lastGpsUpdate > GPS_TIMEOUT_MS))) {
    if (deviceConnected && gCharGps) {
      char buf[64];
      snprintf(buf, sizeof(buf), "{\"fix\":false,\"satellites\":%d}", gps.satellites.value());
      gCharGps->setValue(buf);
      gCharGps->notify();
    }
    
    if (lastFixValid) {
      DBG_PRINTLN("[GPS] Fix lost");
    }
  }
  
  lastFixValid = currentFixValid;
}

// ===== BLE Initialization with Error Handling =====
bool initializeBLE() {
  try {
    BLEDevice::init("SmartBag-Tracker");
    
    gServer = BLEDevice::createServer();
    if (!gServer) {
      DBG_PRINTLN("[BLE] Failed to create server");
      return false;
    }
    
    // Create and store callback objects for proper cleanup
    serverCallbacks = new MyServerCallbacks();
    gServer->setCallbacks(serverCallbacks);

    BLEService* svc = gServer->createService(BLEUUID(SERVICE_UUID));
    if (!svc) {
      DBG_PRINTLN("[BLE] Failed to create service");
      return false;
    }

    // Event characteristic
    gCharEvt = svc->createCharacteristic(
                 BLEUUID(CHARACTERISTIC_UUID_EVT),
                 BLECharacteristic::PROPERTY_READ |
                 BLECharacteristic::PROPERTY_WRITE |
                 BLECharacteristic::PROPERTY_NOTIFY
               );
    if (!gCharEvt) {
      DBG_PRINTLN("[BLE] Failed to create event characteristic");
      return false;
    }
    
    gCharEvt->setValue("ready");
    gCharEvt->addDescriptor(new BLE2902());
    
    charCallbacks = new MyCharacteristicCallbacks();
    gCharEvt->setCallbacks(charCallbacks);

    // GPS characteristic
    gCharGps = svc->createCharacteristic(
                 BLEUUID(CHARACTERISTIC_UUID_GPS),
                 BLECharacteristic::PROPERTY_READ | 
                 BLECharacteristic::PROPERTY_NOTIFY
               );
    if (!gCharGps) {
      DBG_PRINTLN("[BLE] Failed to create GPS characteristic");
      return false;
    }
    
    gCharGps->setValue("{\"fix\":false}");
    gCharGps->addDescriptor(new BLE2902());

    svc->start();

    BLEAdvertising* adv = gServer->getAdvertising();
    if (!adv) {
      DBG_PRINTLN("[BLE] Failed to get advertising");
      return false;
    }
    
    adv->addServiceUUID(svc->getUUID());
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    adv->setMinPreferred(0x12);
    adv->start();

    DBG_PRINTLN("[BLE] Initialization successful");
    return true;
    
  } catch (const std::exception& e) {
    DBG_PRINTF("[BLE] Exception during initialization: %s\n", e.what());
    return false;
  } catch (...) {
    DBG_PRINTLN("[BLE] Unknown exception during initialization");
    return false;
  }
}

// ===== MPU Initialization with Error Handling =====
bool initializeMPU() {
  DBG_PRINTLN("[MPU] Initializing...");
  
  // Try primary address first
  uint8_t whoAmI = 0;
  MPU_ADDR = 0x68;
  
  if (!mpuReadBytes(0x75, &whoAmI, 1) || whoAmI == 0x00) {
    // Try alternate address
    MPU_ADDR = 0x69;
    if (!mpuReadBytes(0x75, &whoAmI, 1) || whoAmI == 0x00) {
      DBG_PRINTLN("[MPU] Not found at either address");
      return false;
    }
  }
  
  DBG_PRINTF("[MPU] Found device 0x%02X at address 0x%02X\n", whoAmI, MPU_ADDR);
  
  // Reset device
  if (!mpuWrite8(REG_PWR_MGMT_1, 0x80)) {
    DBG_PRINTLN("[MPU] Failed to reset device");
    return false;
  }
  delay(100);
  
  // Wake up and set clock source
  if (!mpuWrite8(REG_PWR_MGMT_1, 0x01)) {
    DBG_PRINTLN("[MPU] Failed to wake up device");
    return false;
  }
  delay(10);
  
  // Set accelerometer range to ±2g
  if (!mpuWrite8(REG_ACCEL_CONFIG, 0x00)) {
    DBG_PRINTLN("[MPU] Failed to configure accelerometer");
    return false;
  }
  delay(10);
  
  // Test read
  float ax, ay, az;
  if (!readAccelG(ax, ay, az)) {
    DBG_PRINTLN("[MPU] Failed to read test data");
    return false;
  }
  
  DBG_PRINTF("[MPU] Test read: ax=%.3f, ay=%.3f, az=%.3f\n", ax, ay, az);
  DBG_PRINTLN("[MPU] Initialization successful");
  return true;
}

// ===== Main Setup =====
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  buzzerInit();

  Serial.begin(115200);
  delay(500); // Allow serial to stabilize
  
  DBG_PRINTLN("\n=== SmartBag Enhanced Tracker Starting ===");
  
  // Initialize watchdog
  esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_S,
        .trigger_panic = true
  };

  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  DBG_PRINTLN("[WDT] Watchdog initialized");

  // Create mutex for settings
  settingsMutex = xSemaphoreCreateMutex();
  if (!settingsMutex) {
    handleCriticalError("Failed to create settings mutex");
  }

  // Load settings
  if (!loadSettings()) {
    DBG_PRINTLN("[SETUP] Using default settings due to load failure");
  }

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(50);

  // Initialize MPU
  if (!initializeMPU()) {
    DBG_PRINTLN("[SETUP] Warning: MPU initialization failed, motion detection disabled");
    motionProtectionEnabled = false;
  }

  // Initialize GPS
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX2, GPS_TX2);
  DBG_PRINTLN("[GPS] Initialized");

  // Set motion profile
  if (currentSensitivity == "LOW") {
    setProfile(MODE_TRANSIT);
  } else if (currentSensitivity == "HIGH") {
    setProfile(MODE_HIGH);
  } else {
    setProfile(MODE_NORMAL);
  }

  // Initialize BLE
  if (!initializeBLE()) {
    handleCriticalError("BLE initialization failed");
  }

  // Initialize WiFi (non-blocking)
  setupWiFi();

  // Initialize MQTT client
  espClient.setInsecure(); // Skip certificate validation
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(5);

  // Reset state variables
  lastDisconnectedAt = millis();
  alarmTriggered = false;
  motionInProgress = false;
  
  DBG_PRINTLN("[SETUP] Initialization complete");
  DBG_PRINTLN("=== Available Commands ===");
  DBG_PRINTLN("Serial shortcuts: 1=protection on, 0=off, m=motion on, n=off");
  DBG_PRINTLN("                 l=low sens, k=normal, h=high, d=distance off, s=status");
  DBG_PRINTLN("========================\n");
}

// ===== Main Loop =====
void loop() {
  unsigned long now = millis();
  static unsigned long lastMqttPublish = 0;
  static unsigned long lastStatusReport = 0;
  
  // Reset watchdog
  esp_task_wdt_reset();
  
  // Process serial input
  processSerialInput();
  
  // Process GPS data
  processGPS();
  
  // Check network connections
  checkWiFiConnection();
  checkMQTTConnection();
  
  // Handle BLE disconnect alarm logic
  if (!deviceConnected && protectionEnabled) {
    if (!alarmTriggered && (now - lastDisconnectedAt > GRACE_MS)) {
      alarmTriggered = true;
      alarmOn = true;
      DBG_PRINTLN("[ALARM] Triggered by BLE disconnect");
      
      // Publish location immediately when alarm first triggers
      if (mqttState == MQTT_STATE_CONNECTED) {
        publishLocation();
        lastMqttPublish = now;
      }
    }
    
    // Continue alarm state
    if (alarmTriggered) {
      alarmOn = true;
    }
  } else {
    // Device connected or protection disabled
    if (alarmOn) {
      DBG_PRINTLN("[ALARM] Clearing alarm");
    }
    alarmOn = false;
    alarmTriggered = false;
  }

  // Run siren
  runSiren();

  // Motion detection
  if (motionProtectionEnabled && now - lastReadMs >= READ_PERIOD_MS) {
    float ax, ay, az;
    if (readAccelG(ax, ay, az)) {
      processMotion(ax, ay, az);
    } else {
      // Handle MPU read failure
      static unsigned long lastMpuError = 0;
      if (now - lastMpuError > 5000) { // Log error max once per 5 seconds
        DBG_PRINTLN("[MPU] Read error, motion detection may be unreliable");
        lastMpuError = now;
      }
    }
  }

  // Periodic MQTT publishing during alarm
  if (alarmTriggered && mqttState == MQTT_STATE_CONNECTED) {
    if (now - lastMqttPublish >= MQTT_PUBLISH_INTERVAL_MS) {
        publishLocation();
        lastMqttPublish = now;
    }
}
  
  // Periodic status report
  if (now - lastStatusReport > 60000) { // Every minute
    lastStatusReport = now;
    DBG_PRINTF("[STATUS] Uptime: %lu min, WiFi: %s, MQTT: %s, Protection: %s\n",
               now / 60000,
               wifiState == WIFI_CONNECTED ? "OK" : "FAIL",
               mqttState == MQTT_STATE_CONNECTED ? "OK" : "FAIL", 
               protectionEnabled ? "ON" : "OFF");
    
    if (gps.location.isValid()) {
      DBG_PRINTF("[STATUS] GPS: %.6f,%.6f (%d sats)\n", 
                 gps.location.lat(), gps.location.lng(), gps.satellites.value());
    }
  }

  // Keep MQTT connection alive
  if (mqttState == MQTT_STATE_CONNECTED) {
    mqttClient.loop();
  }
  
  // Light sleep for power saving when not in alarm state
  if (!alarmOn && !motionProtectionEnabled) {
    enterLightSleep(10); // 10ms sleep
  } else {
    delay(1); // Minimal delay to prevent tight loop
  }
}
