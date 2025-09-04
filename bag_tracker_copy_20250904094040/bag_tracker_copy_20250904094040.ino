/*
  ESP32 Ultrasonic Proximity Alarm
  - System always active (no button)
  - Ultrasonic triggers buzzer + LED if object < 50 cm
  - JSON logs over Serial @115200
*/

#include <Arduino.h>

// ==== Pins ====
const int TRIG_PIN = 4;
const int ECHO_PIN = 15;   // ⚠️ use resistor divider for 5V → 3.3V
const int LED_PIN  = 5;
const int BUZ_PIN  = 25;

// ==== Settings ====
const unsigned long MEAS_PERIOD_MS = 200;    // check every 200 ms
const float TRIGGER_DISTANCE_CM = 50.0;      // alarm if closer than this

// ---- Distance helper ----
float measureDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 25000); // 25 ms timeout
  if (dur == 0) return NAN;
  return dur / 58.0f;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("{\"event\":\"boot\",\"msg\":\"ultrasonic proximity alarm ready\"}");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZ_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZ_PIN, LOW);
}

void loop() {
  static unsigned long lastMs = 0;
  unsigned long now = millis();

  if (now - lastMs >= MEAS_PERIOD_MS) {
    lastMs = now;
    float d = measureDistanceCm();
    if (!isnan(d)) {
      Serial.print("{\"ts\":"); Serial.print(now);
      Serial.print(",\"ultra\":{\"cm\":"); Serial.print(d, 1);
      Serial.println("}}");

      if (d <= TRIGGER_DISTANCE_CM) {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(BUZ_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
        digitalWrite(BUZ_PIN, LOW);
      }
    }
  }
}
