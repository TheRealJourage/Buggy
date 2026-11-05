// Layout mockup of the three round status displays (viewer/displays.html), shown
// on three displays at once with simulated values.
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <U8g2lib.h>

// Wiring 2026-11-05/06: SCL/SDA/DC/RST shared, one CS per display, hardware SPI0.
constexpr int8_t PIN_SCK  = 18;
constexpr int8_t PIN_MOSI = 19;
constexpr int8_t PIN_DC   = 17;
constexpr int8_t PIN_RST  = 20;
constexpr int NUM_DISPLAYS = 4;
constexpr int8_t PIN_CS[NUM_DISPLAYS] = {16, 21, 22, 26};
// Arduino_RPiPicoSPI always claims a MISO pin; GP4 is SPI0 RX and unconnected.
constexpr int8_t PIN_MISO_UNUSED = 4;
constexpr int32_t SPI_HZ = 40000000;

Arduino_GFX *panel[NUM_DISPLAYS];
// One shared framebuffer (115 KB); three would not fit into the RP2040 RAM.
Arduino_Canvas *g;

constexpr uint16_t C_OK = 0x3E6D, C_WARN = 0xF606, C_BAD = 0xFA69;
constexpr uint16_t C_GREY = 0x8C51, C_DIM = 0x7BAF, C_TRACK = 0x2124, C_SEP = 0x3186;
const uint16_t STATUS_COL[] = {C_OK, C_WARN, C_BAD};

constexpr int PAGES = 3;

struct Small { const char *val; const char *unit; const char *label; uint8_t st; };
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

// Unit sits small behind the number, so the label below needs no unit and stays short.
void valueWithUnit(const char *val, const char *unit, int16_t x, int16_t y, uint16_t col) {
  int16_t x1, y1; uint16_t wv, wu = 0, h;
  g->setFont(u8g2_font_helvB18_tf);
  g->getTextBounds(val, 0, 0, &x1, &y1, &wv, &h);
  int16_t xv = x1;
  if (*unit) {
    g->setFont(u8g2_font_helvB10_tf);
    g->getTextBounds(unit, 0, 0, &x1, &y1, &wu, &h);
    wu += 3;
  }
  int16_t left = x - (wv + wu) / 2;
  g->setFont(u8g2_font_helvB18_tf);
  g->setTextColor(col);
  g->setCursor(left - xv, y);
  g->print(val);
  if (*unit) {
    g->setFont(u8g2_font_helvB10_tf);
    g->setTextColor(C_GREY);
    g->setCursor(left + wv + 3, y);
    g->print(unit);
  }
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
  centered(sc.big, 120, 128, u8g2_font_logisoso42_tf, sc.bigSt ? STATUS_COL[sc.bigSt] : RGB565_WHITE);
  centered(sc.bigLabel, 120, 150, u8g2_font_helvB14_tf, C_GREY);

  g->drawFastVLine(120, 158, 40, C_SEP);
  for (int j = 0; j < 2; j++) {
    int16_t x = j ? 158 : 82;
    const Small &s = sc.s[j];
    valueWithUnit(s.val, s.unit, x, 177, s.st ? STATUS_COL[s.st] : 0xEF7D);
    centered(s.label, x, 195, u8g2_font_helvB12_tf, C_GREY);
  }
  for (int p = 0; p < PAGES; p++) g->fillCircle(104 + p * 16, 222, 4, p == sc.page ? 0xDEFB : 0x4208);
}

void show(int disp, const Screen &sc) {
  render(sc);
  panel[disp]->draw16bitRGBBitmap(0, 0, g->getFramebuffer(), 240, 240);
}

#define DEG "\xb0"
#define UE "\xfc"

// Pages 1-3 for displays 1-3 (viewer/displays.html), then alarm scenarios on page 1.
const Screen SCREENS[] = {
  {"MOTOR",     "0,98", "Lambda",       0, {{"32", DEG "C", "Luft", 0}, {"28", "%", "Duty", 0}},          nullptr, -1, 0},
  {"ANTRIEB",   "1",    "Gang",         0, {{"9,4", "bar", "Druck", 0}, {"aus", "", "Kompr.", 0}},        nullptr, -1, 0},
  {"FAHRZEUG",  "64",   "Tank %",       0, {{"35", "%", "L" UE "fter", 0}, {"AUTO", "", "Modus", 0}},     nullptr, -1, 0},
  {"LAST",      "38",   "MAP kPa",      0, {{"6", "%", "Drossel", 0}, {"14", DEG, "Z" UE "ndung", 0}},    nullptr, -1, 1},
  {"DRUCKLUFT", "9,4",  "Tank bar",     0, {{"aus", "", "Kompr.", 0}, {"OK", "", "Schaltung", 0}},        nullptr, -1, 1},
  {"L" UE "FTER", "35", "PWM %",        0, {{"AUTO", "", "Modus", 0}, {"aus", "", "Anford.", 0}},         nullptr, -1, 1},
  {"GEMISCH",   "+3",   "Korrektur %",  0, {{"1,00", "", "Soll", 0}, {"3,2", "ms", "Puls", 0}},           nullptr, -1, 2},
  {"SCHALTUNG", "OK",   "letzte",       0, {{"180", "ms", "Dauer", 0}, {"1", "", "Gang", 0}},             nullptr, -1, 2},
  {"FAHRT",     "42",   "km/h",         0, {{"64", "%", "Tank", 0}, {"35", "%", "L" UE "fter", 0}},       nullptr, -1, 2},
  {"MOTOR",     "1,19", "Lambda",       2, {{"34", DEG "C", "Luft", 0}, {"72", "%", "Duty", 0}},          nullptr, -1, 0},
  {"ANTRIEB",   "3",    "Gang",         0, {{"7,6", "bar", "Druck", 1}, {"AN", "", "Kompr.", 0}},         nullptr, -1, 0},
  {"ANTRIEB",   "3",    "Gang",         0, {{"6,4", "bar", "Druck", 2}, {"AN", "", "Kompr.", 0}},         nullptr, -1, 0},
  {"ANTRIEB",   "2",    "Gang",         0, {{"9,1", "bar", "Druck", 0}, {"aus", "", "Kompr.", 0}},        "SCHALT",  -1, 0},
  {"MOTOR",     "--",   "Lambda",       0, {{"33", DEG "C", "Luft", 0}, {"0", "%", "Duty", 0}},           "SYNC",    -1, 0},
  {"FAHRZEUG",  "8",    "Tank %",       2, {{"35", "%", "L" UE "fter", 0}, {"AUTO", "", "Modus", 0}},     nullptr, 0.08f, 0},
};
constexpr int SCENARIO_DISPLAY[] = {0, 1, 1, 1, 0, 2};
// Display 4 has no assigned content yet (only 3 pages x 3 displays decided).
const Screen PLACEHOLDER = {"DISPLAY 4", "--", "noch frei", 0, {{"--", "", "Wert", 0}, {"--", "", "Wert", 0}}, nullptr, -1, 0};

void setup() {
  Serial.begin(115200);
  for (int8_t cs : PIN_CS) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
  for (int i = 0; i < NUM_DISPLAYS; i++) {
    auto *bus = new Arduino_RPiPicoSPI(PIN_DC, PIN_CS[i], PIN_SCK, PIN_MOSI, PIN_MISO_UNUSED, spi0);
    // RST is shared: only display 1 pulses it (resets all three before any init).
    panel[i] = new Arduino_GC9A01(bus, i == 0 ? PIN_RST : GFX_NOT_DEFINED, 0, true);
    if (!panel[i]->begin(SPI_HZ)) Serial.printf("display %d begin failed\n", i + 1);
  }
  g = new Arduino_Canvas(240, 240, panel[0]);
  g->begin(GFX_SKIP_OUTPUT_BEGIN);
}

void showPage(int page) {
  for (int d = 0; d < 3; d++) show(d, SCREENS[page * 3 + d]);
  show(3, PLACEHOLDER);
}

void loop() {
  for (int p = 0; p < PAGES; p++) { showPage(p); delay(5000); }
  for (int i = 0; i < 6; i++) {
    uint32_t t = millis();
    show(SCENARIO_DISPLAY[i], SCREENS[9 + i]);
    Serial.printf("scenario %d on display %d in %lu ms\n", i, SCENARIO_DISPLAY[i] + 1, (unsigned long)(millis() - t));
    delay(3000);
    showPage(0);
  }
}
