#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ===== Pins =====
const int BUZ_PIN = 25;   // Buzzer +
const int LED_PIN = 5;    // LED (through 220Ω to GND)
const int SDA_PIN = 21;   // I2C SDA
const int SCL_PIN = 22;   // I2C SCL

// ===== BLE UUIDs =====
#define SERVICE_UUID        "00001234-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00005678-0000-1000-8000-00805f9b34fb"

// ===== Tether alarm (BLE disconnect) =====
volatile bool deviceConnected = false;
unsigned long lastDisconnectedAt = 0;
unsigned long GRACE_MS = 2500;  // ms delay after disconnect before alarm

// ===== Siren (two-tone, non-blocking, using tone()) =====
int SIREN_F1 = 900;              // Hz
int SIREN_F2 = 1400;             // Hz
unsigned long SIREN_STEP_MS = 300;
bool alarmOn = false;
bool sirenAtF1 = true;
unsigned long lastSirenStep = 0;

// ===== Motion detection (MPU-9250/6500) =====
uint8_t  MPU_ADDR = 0x68; // 0x68 if AD0=GND, 0x69 if AD0=VCC
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_ACCEL_CONFIG = 0x1C; // ±2g when 0x00
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const float   ACC_LSB_PER_G    = 16384.0f;

// ---- Tunable thresholds (changed via profiles) ----
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

// BLE globals
BLEServer*         gServer = nullptr;
BLECharacteristic* gChar   = nullptr;

float toDeg(float r){ return r * 57.29578f; }

// ===== Buzzer helpers (portable) =====
void buzzerInit() {
  pinMode(BUZ_PIN, OUTPUT);
  digitalWrite(BUZ_PIN, LOW);
}
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

// ===== I2C helpers =====
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

// ===== Siren (two-tone, non-blocking) =====
void runSiren() {
  if (!alarmOn) { buzzerOff(); digitalWrite(LED_PIN,LOW); return; }
  unsigned long now = millis();
  if (now - lastSirenStep >= SIREN_STEP_MS) {
    lastSirenStep = now;
    sirenAtF1 = !sirenAtF1;
    buzzerTone(sirenAtF1 ? SIREN_F1 : SIREN_F2);
    digitalWrite(LED_PIN, sirenAtF1 ? HIGH : LOW); // blink with tone
  }
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
    if (deviceConnected && gChar) {
      gChar->setValue("MOTION"); gChar->notify();
    }
    // local short chirp (not the big tether siren)
    buzzerTone(1400);
    digitalWrite(LED_PIN, HIGH);
    delay(70);
    buzzerOff();
    digitalWrite(LED_PIN, LOW);

    motionOverSince = 0; // reset latch
  }

  // Optional debug every ~600 ms
  static uint32_t lastDbg=0;
  if (now - lastDbg > 600) {
    lastDbg = now;
    Serial.printf("[FEAT] jerk=%.2f g/s  std=%.3f g  tilt=%.1f deg  cond=%d\n",
                  jerk, stdDyn, tiltSpan, cond?1:0);
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

// ===== Auto-calibration (press 'c' in Serial) =====
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
  // set threshold somewhat above measured noise floor
  STD_THR_G = max(0.045f, noiseStd * 2.2f);
  Serial.printf("[CAL] noiseStd=%.4f -> STD_THR_G=%.4f\n", noiseStd, STD_THR_G);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  buzzerInit();

  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] SmartBag: BLE tether + robust motion (tone-based siren)");

  // I2C init
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

  // Apply default sensitivity profile
  setProfile(MODE_NORMAL);

  // BLE init (classic)
  BLEDevice::init("SmartBag-Tracker");
  gServer = BLEDevice::createServer();
  gServer->setCallbacks(new MyServerCallbacks());

  BLEService* svc = gServer->createService(BLEUUID(SERVICE_UUID));
  gChar = svc->createCharacteristic(
            BLEUUID(CHARACTERISTIC_UUID),
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
         );
  gChar->setValue("ready");
  gChar->addDescriptor(new BLE2902());
  svc->start();

  BLEAdvertising* adv = gServer->getAdvertising();
  adv->addServiceUUID(svc->getUUID());
  adv->setScanResponse(true);
  adv->start();

  Serial.println("[BLE] Advertising as SmartBag-Tracker. Connect & enable notifications.");
  Serial.println("[HINT] Press 't'=Transit, 'n'=Normal, 'h'=High, 'c'=Calibrate");
}

void loop() {
  unsigned long now = millis();

  // Tether alarm state
  if (deviceConnected) {
    alarmOn = false;
  } else if (lastDisconnectedAt && (now - lastDisconnectedAt) > GRACE_MS) {
    alarmOn = true;
  } else {
    alarmOn = false;
  }

  // Run siren while alarmOn (non-blocking)
  runSiren();

  // Motion detection while connected (sample at READ_PERIOD_MS)
  static unsigned long lastRead = 0;
  if (deviceConnected && (now - lastRead) >= READ_PERIOD_MS) {
    lastRead = now;
    float ax, ay, az;
    if (readAccelG(ax, ay, az)) {
      processMotion(ax, ay, az);
    }
  }

  // Serial commands for live tuning
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='t') setProfile(MODE_TRANSIT);
    if (c=='n') setProfile(MODE_NORMAL);
    if (c=='h') setProfile(MODE_HIGH);
    if (c=='c') autoCalibrateStd();
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
