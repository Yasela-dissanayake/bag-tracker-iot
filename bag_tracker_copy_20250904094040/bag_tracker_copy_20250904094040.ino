// 03_button_event.ino
const int BTN = 18;

void setup() {
  Serial.begin(115200);
  pinMode(BTN, INPUT_PULLUP); // pressed = LOW
}

void loop() {
  static int last = HIGH;
  int v = digitalRead(BTN);
  if (v != last) {
    last = v;
    if (v == LOW) Serial.println("{\"event\":\"button\",\"state\":\"pressed\"}");
    else          Serial.println("{\"event\":\"button\",\"state\":\"released\"}");
  }
}
