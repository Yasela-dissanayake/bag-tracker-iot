#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "00001234-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00005678-0000-1000-8000-00805f9b34fb"

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);
  
  // Initialize BLE
  BLEDevice::init("AntiTheftBag");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  
  Serial.println("BLE Server is running and advertising...");
}

void loop() {
  if(deviceConnected) {
    int randomEvent = random(0, 10); // Random number 0-9
    if(randomEvent > 7) {  // 20% chance to trigger alarm
      String alarmMsg = "ALARM";
      pCharacteristic->setValue(alarmMsg.c_str());
      pCharacteristic->notify();  // Send notification
      Serial.println("ALARM sent over BLE!");
    }
  }
  delay(3000); // 3 seconds interval
}
