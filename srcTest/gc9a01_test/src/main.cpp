#include <Arduino.h>
#include <Arduino_GFX_Library.h>

// Pins as soldered 2026-11-05; GP17 is no SCK pin, so bit-banged SPI.
constexpr int8_t PIN_SCK  = 17;
constexpr int8_t PIN_MOSI = 18;
constexpr int8_t PIN_CS   = 16;
constexpr int8_t PIN_DC   = 19;
constexpr int8_t PIN_RST  = 20;

Arduino_DataBus *bus = new Arduino_SWSPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, PIN_RST, 0, true);

void drawTestScreen(uint32_t round) {
  gfx->fillScreen(RGB565_BLACK);
  gfx->drawCircle(120, 120, 119, RGB565_WHITE);
  gfx->drawCircle(120, 120, 100, RGB565_YELLOW);
  gfx->drawLine(0, 120, 239, 120, RGB565_DARKGREY);
  gfx->drawLine(120, 0, 120, 239, RGB565_DARKGREY);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(50, 80);
  gfx->print("GC9A01");
  gfx->setTextSize(4);
  gfx->setCursor(85, 140);
  gfx->print(round);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  if (!gfx->begin(4000000)) {
    Serial.println("gfx->begin() failed");
  }
  Serial.println("GC9A01 test start (Arduino_GFX)");
}

void loop() {
  static uint32_t round = 0;
  const uint16_t colors[] = {RGB565_RED, RGB565_GREEN, RGB565_BLUE, RGB565_WHITE};
  for (uint16_t c : colors) {
    gfx->fillScreen(c);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(700);
  }
  drawTestScreen(round);
  Serial.printf("round %lu ok\n", (unsigned long)round);
  round++;
  delay(3000);
}
