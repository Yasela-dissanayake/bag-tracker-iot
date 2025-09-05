#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <TinyGPSPlus.h>

// ===== Pins =====
const int BUZ_PIN = 25;   // Buzzer +
const int LED_PIN = 13;    // LED (through 220Ω to GND)
const int SDA_PIN = 21;   // I2C SDA (MPU)
const int SCL_PIN = 22;   // I2C SCL (MPU)
const int GPS_RX2 = 16;   // ESP32 RX2  <- GPS TX
const int GPS_TX2 = 17;   // ESP32 TX2  -> GPS RX (optional)

// ===== BLE UUIDs =====
#define SERVICE_UUID              "00001234-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_EVT   "00005678-0000-1000-8000-00805f9b34fb" // motion/tether events
#define CHARACTERISTIC_UUID_GPS   "0000A001-0000-1000-8000-00805f9b34fb" // GPS JSON

// ===== Tether alarm (BLE disconnect) =====
volatile bool deviceConnected = false;
unsigned long lastDisconnectedAt = 0;
unsigned long GRACE_MS = 2500;  // delay after disconnect before alarm

// ===== Siren (two-tone, non-blocking, using tone()) =====
int SIREN_F1 = 900;              // Hz
int SIREN_F2 = 1400;             // Hz
unsigned long SIREN_STEP_MS = 300;
bool alarmOn = false;
bool prevAlarmOn = false;
bool sirenAtF1 = true;
unsigned long lastSirenStep = 0;

// ===== Motion detection (MPU-9250/6500) =====
uint8_t  MPU_ADDR = 0x68; // 0x68 if AD0=GND, 0x69 if AD0=VCC
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_ACCEL_CONFIG = 0x1C; // ±2g when 0x00
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const float   ACC_LSB_PER_G    = 16384.0f;

// ---- Tunable thresholds (changed via profiles/dev mode) ----
float     JERK_THR_G_PER_S = 1.00f;
float     STD_THR_G        = 0.065f;
float     TILT_THR_DEG     = 40.0f;
uint16_t  HOLD_MS          = 220;
uint16_t  WINDOW_MS        = 700;
uint16_t  READ_PERIOD_MS   = 15;    // ~66 Hz

// ---- Motion state / buffers ----
struct Sample { float dyn; float pitch; float roll; uint32_t t; };
const int MAX_SAMPLES = 128;
Sample ringBuf[MAX_SAMPLES];
int rbHead = 0, rbCount = 0;

float    lastMag = 1.0f;
uint32_t lastReadMs = 0;
uint32_t motionOverSince = 0;
uint32_t lastMotionNotify = 0;
const uint32_t REFRACTORY_MS = 4000; // min gap between motion notifies

// ===== GPS =====
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);
double lastLat = 0.0, lastLng = 0.0;

// ===== BLE globals =====
BLEServer*          gServer   = nullptr;
BLECharacteristic*  gCharEvt  = nullptr; // "MOTION" etc.
BLECharacteristic*  gCharGps  = nullptr; // JSON GPS

// ===== Helpers =====
float toDeg(float r){ return r * 57.29578f; }

// ===== Buzzer helpers (portable) =====
void buzzerInit() { pinMode(BUZ_PIN, OUTPUT); digitalWrite(BUZ_PIN, LOW); }
inline void buzzerTone(uint16_t freq) { tone(BUZ_PIN, freq); } // start/retune tone
inline void buzzerOff() { noTone(BUZ_PIN); digitalWrite(BUZ_PIN, LOW); }

// ===== BLE callbacks =====
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    alarmOn = false;
    buzzerOff();
    digitalWrite(LED_PIN, LOW);
    Serial.println("[BLE] Connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    lastDisconnectedAt = millis();
    Serial.println("[BLE] Disconnected");
    pServer->getAdvertising()->start();
  }
};

// ===== I2C / MPU helpers =====
bool mpuWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool mpuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len) != len) return false;
  for (uint8_t i=0;i<len;i++) buf[i]=Wire.read();
  return true;
}
bool readAccelG(float &ax, float &ay, float &az) {
  uint8_t raw[6];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, raw, 6)) return false;
  int16_t rx = (int16_t)((raw[0]<<8)|raw[1]);
  int16_t ry = (int16_t)((raw[2]<<8)|raw[3]);
  int16_t rz = (int16_t)((raw[4]<<8)|raw[5]);
  ax=(float)rx/ACC_LSB_PER_G; ay=(float)ry/ACC_LSB_PER_G; az=(float)rz/ACC_LSB_PER_G;
  return true;
}

// ===== Siren (non-blocking) with instant start on rising edge =====
void runSiren() {
  unsigned long now = millis();

  // Rising edge: start immediately
  if (alarmOn && !prevAlarmOn) {
    buzzerTone(SIREN_F1);
    digitalWrite(LED_PIN, HIGH);
    lastSirenStep = now;
    sirenAtF1 = true;
  }

  if (!alarmOn) {
    buzzerOff();
    digitalWrite(LED_PIN, LOW);
  } else {
    if (now - lastSirenStep >= SIREN_STEP_MS) {
      lastSirenStep = now;
      sirenAtF1 = !sirenAtF1;
      buzzerTone(sirenAtF1 ? SIREN_F1 : SIREN_F2);
      digitalWrite(LED_PIN, sirenAtF1 ? HIGH : LOW);
    }
  }

  prevAlarmOn = alarmOn;
}

// ===== Motion features / decision =====
void processMotion(float ax,float ay,float az) {
  uint32_t now = millis();
  float mag = sqrtf(ax*ax+ay*ay+az*az);
  float dyn = fabsf(mag - 1.0f);

  float dt = (lastReadMs==0) ? 0.01f : (now - lastReadMs) / 1000.0f;
  if (dt <= 0) dt = 0.01f;
  lastReadMs = now;

  float jerk = fabsf(mag - lastMag) / dt;
  lastMag = mag;

  float pitch = toDeg(atan2f(ax, sqrtf(ay*ay + az*az)));
  float roll  = toDeg(atan2f(ay, sqrtf(ax*ax + az*az)));

  // push to ring buffer
  ringBuf[rbHead] = { dyn, pitch, roll, now };
  rbHead = (rbHead + 1) % MAX_SAMPLES;
  if (rbCount < MAX_SAMPLES) rbCount++;

  // window stats (last WINDOW_MS)
  float mean=0, m2=0; int n=0;
  float minP=+1e9, maxP=-1e9, minR=+1e9, maxR=-1e9;
  for (int i=0;i<rbCount;i++) {
    int idx = (rbHead - 1 - i + MAX_SAMPLES) % MAX_SAMPLES;
    if (now - ringBuf[idx].t > WINDOW_MS) break;
    float x = ringBuf[idx].dyn;
    n++; float d = x - mean; mean += d / n; m2 += d * (x - mean);
    if (ringBuf[idx].pitch < minP) minP = ringBuf[idx].pitch;
    if (ringBuf[idx].pitch > maxP) maxP = ringBuf[idx].pitch;
    if (ringBuf[idx].roll  < minR) minR = ringBuf[idx].roll;
    if (ringBuf[idx].roll  > maxR) maxR = ringBuf[idx].roll;
  }
  if (n < 5) return;

  float stdDyn   = sqrtf(m2 / (n - 1));
  float tiltSpan = max(maxP - minP, maxR - minR);

  bool cond = (jerk > JERK_THR_G_PER_S) && (stdDyn > STD_THR_G) && (tiltSpan > TILT_THR_DEG);

  // must persist for HOLD_MS
  if (cond) {
    if (motionOverSince == 0) motionOverSince = now;
  } else {
    motionOverSince = 0;
  }

  bool persistent = motionOverSince && (now - motionOverSince >= HOLD_MS);
  bool cooldown   = (now - lastMotionNotify) < REFRACTORY_MS;

  if (persistent && !cooldown) {
    lastMotionNotify = now;

    // Notify phone (if connected)
    if (deviceConnected && gCharEvt) {
      gCharEvt->setValue("MOTION");
      gCharEvt->notify();
    }
    // local short chirp (not the big tether siren)
    buzzerTone(1400);
    digitalWrite(LED_PIN, HIGH);
    delay(70);
    buzzerOff();
    digitalWrite(LED_PIN, LOW);

    motionOverSince = 0; // reset latch
  }
}

// ===== Profiles (numbers, no enums) =====
#define MODE_TRANSIT 0
#define MODE_NORMAL  1
#define MODE_HIGH    2

void setProfile(uint8_t mode) {
  switch (mode) {
    case MODE_TRANSIT:  // least sensitive (bus/train)
      JERK_THR_G_PER_S = 1.40f;
      STD_THR_G        = 0.10f;
      TILT_THR_DEG     = 60.0f;
      HOLD_MS          = 300;
      WINDOW_MS        = 700;
      READ_PERIOD_MS   = 20;
      Serial.println("[MODE] TRANSIT");
      break;
    case MODE_HIGH:     // most sensitive (café/library)
      JERK_THR_G_PER_S = 0.70f;
      STD_THR_G        = 0.045f;
      TILT_THR_DEG     = 25.0f;
      HOLD_MS          = 150;
      WINDOW_MS        = 800;
      READ_PERIOD_MS   = 12;
      Serial.println("[MODE] HIGH");
      break;
    case MODE_NORMAL:   // default campus walking
    default:
      JERK_THR_G_PER_S = 1.00f;
      STD_THR_G        = 0.065f;
      TILT_THR_DEG     = 40.0f;
      HOLD_MS          = 220;
      WINDOW_MS        = 700;
      READ_PERIOD_MS   = 15;
      Serial.println("[MODE] NORMAL");
      break;
  }
}

// ===== Auto-calibration =====
void autoCalibrateStd() {
  Serial.println("[CAL] capturing baseline 1.5 s...");
  uint32_t t0 = millis();
  float mean=0, m2=0; int n=0;
  while (millis() - t0 < 1500) {
    float ax, ay, az;
    if (readAccelG(ax, ay, az)) {
      float mag = sqrtf(ax*ax + ay*ay + az*az);
      float dyn = fabsf(mag - 1.0f);
      n++; float d = dyn - mean; mean += d / n; m2 += d * (dyn - mean);
    }
    delay(10);
  }
  float noiseStd = (n>1) ? sqrtf(m2/(n-1)) : 0.04f;
  STD_THR_G = max(0.045f, noiseStd * 2.2f);
  Serial.printf("[CAL] noiseStd=%.4f -> STD_THR_G=%.4f\n", noiseStd, STD_THR_G);
}

// ===== DEV / PROD quick toggles =====
void setDevMode(bool on) {
  if (on) {
    GRACE_MS = 0;                // instant siren on disconnect
    SIREN_STEP_MS = 120;         // snappier
    setProfile(MODE_HIGH);       // start very sensitive for testing
    Serial.println("[DEV] Instant-response mode ON");
  } else {
    GRACE_MS = 2500;
    SIREN_STEP_MS = 300;
    setProfile(MODE_NORMAL);
    Serial.println("[DEV] Instant-response mode OFF (production)");
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  buzzerInit();

  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] SmartBag: BLE + Motion + GPS");

  // I2C init (MPU)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(50);

  // WHO_AM_I & address fallback to 0x69 if needed
  uint8_t who=0; mpuReadBytes(0x75, &who, 1);
  if (who == 0x00) { MPU_ADDR = 0x69; mpuReadBytes(0x75, &who, 1); }
  Serial.printf("[MPU] WHO_AM_I=0x%02X at addr 0x%02X\n", who, MPU_ADDR);

  // Wake + set accel ±2g
  mpuWrite8(REG_PWR_MGMT_1, 0x01);   // PLL
  delay(10);
  mpuWrite8(REG_ACCEL_CONFIG, 0x00); // ±2g
  delay(10);

  // GPS UART
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX2, GPS_TX2); // NEO-6M default baud

  // Sensitivity profile
  setProfile(MODE_NORMAL);
  // For development, you can enable instant mode:
  // setDevMode(true);

  // BLE init
  BLEDevice::init("SmartBag-Tracker");
  gServer = BLEDevice::createServer();
  gServer->setCallbacks(new MyServerCallbacks());

  BLEService* svc = gServer->createService(BLEUUID(SERVICE_UUID));

  // Event characteristic (MOTION etc.)
  gCharEvt = svc->createCharacteristic(
               BLEUUID(CHARACTERISTIC_UUID_EVT),
               BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
             );
  gCharEvt->setValue("ready");
  gCharEvt->addDescriptor(new BLE2902());

  // GPS characteristic (JSON)
  gCharGps = svc->createCharacteristic(
               BLEUUID(CHARACTERISTIC_UUID_GPS),
               BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
             );
  gCharGps->setValue("{\"fix\":false}");
  gCharGps->addDescriptor(new BLE2902());

  svc->start();

  BLEAdvertising* adv = gServer->getAdvertising();
  adv->addServiceUUID(svc->getUUID());
  adv->setScanResponse(true);
  adv->start();

  Serial.println("[BLE] Advertising as SmartBag-Tracker. Connect & enable notifications.");
  Serial.println("[HINT] Serial: 't'=Transit, 'n'=Normal, 'h'=High, 'c'=Cal, 'd'=Dev, 'p'=Prod");
}

void loop() {
  unsigned long now = millis();

  // ----------- Tether alarm state -----------
  if (deviceConnected) {
    alarmOn = false;
  } else if (lastDisconnectedAt && (now - lastDisconnectedAt) > GRACE_MS) {
    alarmOn = true;
  } else {
    alarmOn = false;
  }
  runSiren();

  // ----------- Motion detection -----------
  static unsigned long lastRead = 0;
  if (deviceConnected && (now - lastRead) >= READ_PERIOD_MS) {
    lastRead = now;
    float ax, ay, az;
    if (readAccelG(ax, ay, az)) {
      processMotion(ax, ay, az);
    }
  }

  // ----------- GPS feeding & publish -----------
  while (GPS_Serial.available()) gps.encode(GPS_Serial.read());

  static unsigned long lastGpsMs = 0;
  if (deviceConnected && (now - lastGpsMs >= 1000)) {
    lastGpsMs = now;

    if (gps.location.isValid() && gps.hdop.isValid()) {
      double lat = gps.location.lat();
      double lng = gps.location.lng();
      uint32_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
      double hdop = gps.hdop.hdop();

      bool movedEnough = (fabs(lat - lastLat) > 0.0001) || (fabs(lng - lastLng) > 0.0001);

      char buf[100];
      snprintf(buf, sizeof(buf),
               "{\"lat\":%.6f,\"lon\":%.6f,\"sats\":%lu,\"hdop\":%.1f}",
               lat, lng, (unsigned long)sats, hdop);
      gCharGps->setValue((uint8_t*)buf, strlen(buf));

      if (movedEnough || (lastLat == 0.0 && lastLng == 0.0)) {
        gCharGps->notify();
        lastLat = lat; lastLng = lng;
      }
      Serial.print("[GPS] "); Serial.println(buf);
    } else {
      gCharGps->setValue("{\"fix\":false}");
      Serial.println("[GPS] no fix");
    }
  }

  // ----------- Serial live controls -----------
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='t') setProfile(MODE_TRANSIT);
    if (c=='n') setProfile(MODE_NORMAL);
    if (c=='h') setProfile(MODE_HIGH);
    if (c=='c') autoCalibrateStd();
    if (c=='d') setDevMode(true);
    if (c=='p') setDevMode(false);
  }

  // periodic state log
  static unsigned long lastLog=0;
  if (now - lastLog >= 2000) {
    lastLog = now;
    Serial.print("[STATE] connected=");
    Serial.print(deviceConnected ? "true" : "false");
    Serial.print(" alarm=");
    Serial.println(alarmOn ? "on" : "off");
  }
}
