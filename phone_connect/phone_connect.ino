#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// ===== Pins (change if needed) =====
const int BUZ_PIN = 25;   // Buzzer +
const int LED_PIN = 5;    // Optional LED (with 220Ω to GND)

// ===== Behavior tuning =====
const unsigned long GRACE_MS    = 2500; // wait after disconnect before alarming
const unsigned long BUZZ_ON_MS  = 120;  // buzzer on time (ms)
const unsigned long BUZZ_OFF_MS = 220;  // buzzer off time (ms)

// ===== State =====
volatile bool deviceConnected = false;
unsigned long lastDisconnectedAt = 0;
bool alarmOn = false;
bool buzzState = false;
unsigned long buzzTick = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    alarmOn = false;
    digitalWrite(BUZ_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    Serial.println("[BLE] Connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    lastDisconnectedAt = millis();
    Serial.println("[BLE] Disconnected");
    pServer->getAdvertising()->start(); // allow phone to auto-reconnect
  }
};

void setup() {
  // IO
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZ_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Serial
  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] SmartBag BLE Tether Alarm");

  // ===== BLE init (classic ESP32 BLE lib) =====
  BLEDevice::init("SmartBag-Tracker");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Minimal service/characteristic (we mostly care about link state)
  BLEService* pSvc = pServer->createService(BLEUUID("12345678-1234-1234-1234-1234567890ab"));
  pSvc->createCharacteristic(BLEUUID("abcd"), BLECharacteristic::PROPERTY_READ)->setValue("hello");
  pSvc->start();

  BLEAdvertising* pAdv = pServer->getAdvertising();
  pAdv->addServiceUUID(pSvc->getUUID());
  pAdv->setScanResponse(true); // okay in classic BLE lib
  pAdv->start();

  Serial.println("[BLE] Advertising as SmartBag-Tracker. Connect using nRF Connect.");
}

void loop() {
  const unsigned long now = millis();

  // Connected → silent; Disconnected → start alarm after grace period
  if (deviceConnected) {
    alarmOn = false;
  } else {
    alarmOn = (lastDisconnectedAt != 0 && (now - lastDisconnectedAt) > GRACE_MS);
  }

  // Pulse buzzer/LED if alarm is active (less annoying than steady tone)
  if (alarmOn) {
    unsigned long dt = now - buzzTick;
    if (!buzzState && dt >= BUZZ_OFF_MS) {
      buzzState = true;
      digitalWrite(BUZ_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
      buzzTick = now;
    } else if (buzzState && dt >= BUZZ_ON_MS) {
      buzzState = false;
      digitalWrite(BUZ_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      buzzTick = now;
    }
  } else {
    digitalWrite(BUZ_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    buzzState = false;
    buzzTick = now; // reset phase so pulses start cleanly next time
  }

  // Periodic console state (handy while testing)
  static unsigned long lastLog = 0;
  if (now - lastLog >= 2000) {
    lastLog = now;
    Serial.print("[STATE] connected=");
    Serial.print(deviceConnected ? "true" : "false");
    Serial.print(" alarm=");
    Serial.println(alarmOn ? "on" : "off");
  }
}
