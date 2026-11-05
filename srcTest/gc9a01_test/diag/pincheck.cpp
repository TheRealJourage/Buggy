#include <Arduino.h>

struct Line { uint8_t gpio; const char *name; };
const Line lines[] = {{17, "SCL"}, {18, "SDA"}, {19, "DC"}, {16, "CS"}, {20, "RST"}};

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  for (const Line &l : lines) { pinMode(l.gpio, OUTPUT); digitalWrite(l.gpio, LOW); }
}

void loop() {
  for (const Line &l : lines) {
    for (const Line &o : lines) digitalWrite(o.gpio, LOW);
    digitalWrite(l.gpio, HIGH);
    Serial.printf("HIGH: %s (GP%u)\n", l.name, l.gpio);
    for (int i = 0; i < 10; i++) { digitalWrite(LED_BUILTIN, i & 1); delay(500); }
  }
}
