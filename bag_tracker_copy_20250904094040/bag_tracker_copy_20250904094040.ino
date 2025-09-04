// 02_serial_json_sanity.ino
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("{\"event\":\"boot\",\"msg\":\"esp32 up\"}");
}

void loop() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 1000) {
    last = now;
    // fake “heartbeat” you’ll keep for future tests
    Serial.print("{\"ts\":");
    Serial.print((unsigned long)(millis()));
    Serial.println(",\"event\":\"heartbeat\"}");
  }
}
