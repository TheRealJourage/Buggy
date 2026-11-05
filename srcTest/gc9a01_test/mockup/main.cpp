// Layout mockup of the three round status displays (viewer/displays.html), shown
// on three displays at once with simulated values.
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <U8g2lib.h>

// Wiring 2026-11-05: SCL/SDA/DC/RST shared, one CS per display, hardware SPI0.
constexpr int8_t PIN_SCK  = 18;
constexpr int8_t PIN_MOSI = 19;
constexpr int8_t PIN_DC   = 17;
constexpr int8_t PIN_RST  = 20;
constexpr int8_t PIN_CS[3] = {16, 21, 22};
// Arduino_RPiPicoSPI always claims a MISO pin; GP4 is SPI0 RX and unconnected.
constexpr int8_t PIN_MISO_UNUSED = 4;
constexpr int32_t SPI_HZ = 40000000;

Arduino_GFX *panel[3];
// One shared framebuffer (115 KB); three would not fit into the RP2040 RAM.
Arduino_Canvas *g;

constexpr uint16_t C_OK = 0x3E6D, C_WARN = 0xF606, C_BAD = 0xFA69;
constexpr uint16_t C_GREY = 0x8C51, C_DIM = 0x7BAF, C_TRACK = 0x2124, C_SEP = 0x3186;
const uint16_t STATUS_COL[] = {C_OK, C_WARN, C_BAD};

struct Small { const char *val; const char *label; uint8_t st; };
struct Screen {
  const char *title;
  const char *big; const char *bigLabel; uint8_t bigSt;
  Small s[2];
  const char *alarm;  // red badge text or nullptr
  float bar;          // 0..1 for bar ring, <0 = status ring
  uint8_t page;
};

void centered(const char *txt, int16_t x, int16_t y, const uint8_t *font, uint16_t col) {
  int16_t x1, y1; uint16_t w, h;
  g->setFont(font);
  g->getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
  g->setTextColor(col);
  g->setCursor(x - w / 2 - x1, y);
  g->print(txt);
}

void arc(float from, float to, uint16_t col) {
  // 0° = 3 o'clock, clockwise; the gauge runs 135°..405° with the gap at the bottom.
  if (to <= 360) { g->fillArc(120, 120, 118, 106, from, to, col); return; }
  if (from < 360) g->fillArc(120, 120, 118, 106, from, 360, col);
  g->fillArc(120, 120, 118, 106, from > 360 ? from - 360 : 0, to - 360, col);
}

void render(const Screen &sc) {
  uint8_t worst = sc.bigSt;
  for (auto &s : sc.s) worst = max(worst, s.st);
  if (sc.alarm) worst = 2;

  g->fillScreen(RGB565_BLACK);
  if (sc.bar < 0) {
    arc(135, 405, STATUS_COL[worst]);
  } else {
    arc(135, 405, C_TRACK);
    arc(135, 135 + 270 * max(0.01f, sc.bar), STATUS_COL[worst]);
  }

  centered(sc.title, 120, 58, u8g2_font_helvB12_tf, C_GREY);
  if (sc.alarm) {
    g->fillRect(120 - 34, 66, 68, 18, C_BAD);
    centered(sc.alarm, 120, 80, u8g2_font_helvB10_tf, RGB565_BLACK);
  }
  centered(sc.big, 120, 132, u8g2_font_logisoso42_tf, sc.bigSt ? STATUS_COL[sc.bigSt] : RGB565_WHITE);
  centered(sc.bigLabel, 120, 152, u8g2_font_helvB10_tf, C_GREY);

  g->drawFastVLine(120, 160, 40, C_SEP);
  for (int j = 0; j < 2; j++) {
    int16_t x = j ? 162 : 78;
    const Small &s = sc.s[j];
    centered(s.val, x, 184, u8g2_font_helvB18_tf, s.st ? STATUS_COL[s.st] : 0xEF7D);
    centered(s.label, x, 199, u8g2_font_helvR08_tf, C_DIM);
  }
  for (int p = 0; p < 2; p++) g->fillCircle(112 + p * 16, 222, 4, p == sc.page ? 0xDEFB : 0x4208);
}

void show(int disp, const Screen &sc) {
  render(sc);
  panel[disp]->draw16bitRGBBitmap(0, 0, g->getFramebuffer(), 240, 240);
}

// Page 1 of all three displays, then the alarm scenarios from the viewer.
const Screen SCREENS[] = {
  {"MOTOR",    "0,98", "Lambda",       0, {{"32", "Luft \xb0""C", 0}, {"28", "Duty %", 0}},          nullptr, -1, 0},
  {"ANTRIEB",  "1",    "Gang",         0, {{"9,4", "Druck bar", 0}, {"aus", "Kompr.", 0}},          nullptr, -1, 0},
  {"FAHRZEUG", "64",   "Tank %",       0, {{"35", "L\xfc""fter %", 0}, {"AUTO", "Modus", 0}},       nullptr, -1, 0},
  {"MOTOR",    "1,19", "Lambda",       2, {{"34", "Luft \xb0""C", 0}, {"72", "Duty %", 0}},          nullptr, -1, 0},
  {"ANTRIEB",  "3",    "Gang",         0, {{"7,6", "Druck bar", 1}, {"AN", "Kompr.", 0}},           nullptr, -1, 0},
  {"ANTRIEB",  "3",    "Gang",         0, {{"6,4", "Druck bar", 2}, {"AN", "Kompr.", 0}},           nullptr, -1, 0},
  {"ANTRIEB",  "2",    "Gang",         0, {{"9,1", "Druck bar", 0}, {"aus", "Kompr.", 0}},          "SCHALT",  -1, 0},
  {"MOTOR",    "--",   "Lambda",       0, {{"33", "Luft \xb0""C", 0}, {"0", "Duty %", 0}},           "SYNC",    -1, 0},
  {"FAHRZEUG", "8",    "Tank %",       2, {{"35", "L\xfc""fter %", 0}, {"AUTO", "Modus", 0}},       nullptr, 0.08f, 0},
  {"LAST",     "38",   "MAP kPa",      0, {{"6", "Drossel %", 0}, {"14", "Z\xfc""nd. \xb0", 0}},     nullptr, 0.06f, 1},
};

void setup() {
  Serial.begin(115200);
  for (int8_t cs : PIN_CS) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
  for (int i = 0; i < 3; i++) {
    auto *bus = new Arduino_RPiPicoSPI(PIN_DC, PIN_CS[i], PIN_SCK, PIN_MOSI, PIN_MISO_UNUSED, spi0);
    // RST is shared: only display 1 pulses it (resets all three before any init).
    panel[i] = new Arduino_GC9A01(bus, i == 0 ? PIN_RST : GFX_NOT_DEFINED, 0, true);
    if (!panel[i]->begin(SPI_HZ)) Serial.printf("display %d begin failed\n", i + 1);
  }
  g = new Arduino_Canvas(240, 240, panel[0]);
  g->begin(GFX_SKIP_OUTPUT_BEGIN);
}

// Normal view on all three, then one alarm scenario at a time on the matching display.
void loop() {
  for (int d = 0; d < 3; d++) show(d, SCREENS[d]);
  delay(5000);
  const int target[] = {0, 1, 1, 1, 0, 2, 0};
  for (size_t i = 3; i < sizeof(SCREENS) / sizeof(SCREENS[0]); i++) {
    uint32_t t = millis();
    show(target[i - 3], SCREENS[i]);
    Serial.printf("screen %u on display %d in %lu ms\n", (unsigned)i, target[i - 3] + 1, (unsigned long)(millis() - t));
    delay(3000);
    for (int d = 0; d < 3; d++) show(d, SCREENS[d]);
  }
}
