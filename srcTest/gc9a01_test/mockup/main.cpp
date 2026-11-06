// Mockup of the three round displays as decided 2026-11-06 (vault Buggy/Werteliste.md,
// "Belegung Rund-Displays nach dem Fahrdisplay" and "Display SCHALTUNG/DRUCKLUFT"):
//   1 SCHALTUNG/DRUCKLUFT, 2 GEMISCH, 3 MOTOR; normal, page 2 (button) and emergency
//   mode (no Pi heartbeat). Values are simulated; a script plays through the scenarios.
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <U8g2lib.h>

// Wiring 2026-11-05/06: SCL/SDA/DC/RST shared, one CS per display, hardware SPI0.
constexpr int8_t PIN_SCK  = 18;
constexpr int8_t PIN_MOSI = 19;
constexpr int8_t PIN_DC   = 17;
constexpr int8_t PIN_RST  = 20;
// Display 4 (GP26) is still wired on the bench but not part of the decided set:
// it is initialised and left black, otherwise CS floats low (R8) and it mirrors the others.
constexpr int NUM_WIRED = 4;
constexpr int NUM_DISPLAYS = 3;
constexpr int8_t PIN_CS[NUM_WIRED] = {16, 21, 22, 26};
// Arduino_RPiPicoSPI always claims a MISO pin; GP4 is SPI0 RX and unconnected.
constexpr int8_t PIN_MISO_UNUSED = 4;
constexpr int32_t SPI_HZ = 40000000;

Arduino_GFX *panel[NUM_WIRED];
// One shared framebuffer (115 KB); three would not fit into the RP2040 RAM.
Arduino_Canvas *g;

constexpr uint16_t C_OK = 0x3E6D, C_WARN = 0xF606, C_BAD = 0xFA69;
constexpr uint16_t C_GREY = 0x8C51, C_TRACK = 0x2124, C_SEP = 0x3186;
constexpr uint16_t C_TEXT = 0xEF7D, C_TARGET = 0x5DDF, C_RPM = 0xBDF7;
constexpr uint16_t C_FLASH_RED = 0xFAEB, C_FLASH_GREEN = 0x7F2F, C_FAULT_BG = 0xB800;
const uint16_t STATUS_COL[] = {C_OK, C_WARN, C_BAD};

// Placeholders like viewer/pi-dash.html (not from the tune).
constexpr int RPM_MAX = 8000, RPM_SHIFT = 6500;
// Ring full = pressure switch cut-out 10.3 bar (Druckluftanlage.md).
constexpr float PRESS_FULL = 10.3f;

// ---------- simulated vehicle state ----------
enum ShiftState : uint8_t { SH_IDLE, SH_TARGET, SH_FAULT };
struct Vehicle {
  float lambda = 0.98f, lsoll = 1.00f; int ego = 3; float pw = 3.2f; int adv = 14;
  int map = 38, tps = 6, duty = 28, iat = 32, fan = 35; bool fanManual = false, d47 = false;
  int kmh = 0, rpm = 900; float oil = 3.1f; int clt = 82, tank = 64;
  float press = 9.4f; bool comp = false;
  int gear = 1, target = 0; ShiftState shift = SH_IDLE;
  bool sync = true;
} v;

bool emergency = true;          // start in emergency mode until the Pi heartbeat arrives
uint8_t page[NUM_DISPLAYS] = {0, 0, 0};

// ---------- limits (Teile-Bestand "Grenzwerte der Ringfarbe", start values) ----------
uint8_t stPress(float p) { return p >= 8.3f ? 0 : p >= 7.0f ? 1 : 2; }
uint8_t stDuty(int d)    { return d < 75 ? 0 : d < 85 ? 1 : 2; }
uint8_t stLambda(float dev) { dev = fabsf(dev); return dev <= 0.05f ? 0 : dev <= 0.10f ? 1 : 2; }
// Red should be the EGO limit of the tune; 15 % stands in until it is read out.
uint8_t stEgo(int e)     { e = abs(e); return e <= 10 ? 0 : e <= 15 ? 1 : 2; }
uint8_t stIat(int t)     { return t < 50 ? 0 : t <= 65 ? 1 : 2; }
uint8_t stTank(int t)    { return t > 20 ? 0 : t >= 10 ? 1 : 2; }

// ---------- screen description ----------
enum RingMode : uint8_t { RING_STATUS, RING_BAR, RING_DEV };
struct Small { char val[10]; const char *unit; const char *label; uint8_t st; };
struct Screen {
  const char *title;
  char big[10]; char bigLabel[20]; uint8_t bigSt; uint16_t bigCol; bool bigLarge;
  Small s[2];
  const char *alarm;
  RingMode ring; float ringVal; uint16_t ringCol;  // ringCol 0 = worst status colour
  int8_t pages;                                     // page dots, 0 = none
  uint8_t page;
};

void fmt(char *buf, size_t n, float val, int dec) {
  snprintf(buf, n, "%.*f", dec, val);
  for (char *c = buf; *c; c++) if (*c == '.') *c = ',';
}

const char *gearText(int gear) {
  static const char *names[] = {"R", "N", "1", "2", "3", "4", "5"};
  return names[gear + 1];
}

#define DEG "\xb0"
#define UE "\xfc"

Screen screenShift() {
  Screen sc{};
  sc.title = "SCHALTUNG";
  if (v.shift == SH_TARGET) {
    strcpy(sc.big, gearText(v.target));
    snprintf(sc.bigLabel, sizeof sc.bigLabel, "neu (war %s)", gearText(v.gear));
    sc.bigCol = C_TARGET;
  } else {
    strcpy(sc.big, gearText(v.gear));
    strcpy(sc.bigLabel, "Gang");
  }
  sc.bigLarge = true;
  sc.s[0] ={"", "bar", "Druck", stPress(v.press)};
  fmt(sc.s[0].val, sizeof sc.s[0].val, v.press, 1);
  sc.s[1] = {"", "", "Kompr.", 0};
  strcpy(sc.s[1].val, v.comp ? "AN" : "aus");
  sc.ring = RING_BAR; sc.ringVal = v.press / PRESS_FULL;
  sc.ringCol = STATUS_COL[stPress(v.press)];
  return sc;
}

Screen screenMixture() {
  Screen sc{};
  if (emergency) {
    sc.title = "FAHRT";
    snprintf(sc.big, sizeof sc.big, "%d", v.kmh);
    strcpy(sc.bigLabel, "km/h");
    sc.bigLarge = true;
    sc.s[0] = {"", "", "Drehzahl", 0};
    snprintf(sc.s[0].val, sizeof sc.s[0].val, "%d", v.rpm);
    sc.s[1] = {"", "", "Gang", 0};
    strcpy(sc.s[1].val, gearText(v.gear));
    sc.ring = RING_BAR; sc.ringVal = (float)v.rpm / RPM_MAX;
    sc.ringCol = v.rpm >= RPM_SHIFT ? C_BAD : C_RPM;
    sc.alarm = v.sync ? nullptr : "SYNC";
    return sc;
  }
  sc.pages = 2; sc.page = page[1];
  sc.alarm = v.sync ? nullptr : "SYNC";
  if (page[1] == 0) {
    sc.title = "GEMISCH";
    float dev = v.lambda - v.lsoll;
    fmt(sc.big, sizeof sc.big, v.lambda, 2);
    strcpy(sc.bigLabel, "Lambda");
    sc.bigSt = stLambda(dev);
    sc.s[0] = {"", "", "Soll", 0};
    fmt(sc.s[0].val, sizeof sc.s[0].val, v.lsoll, 2);
    sc.s[1] = {"", "%", "Korrektur", stEgo(v.ego)};
    snprintf(sc.s[1].val, sizeof sc.s[1].val, "%+d", v.ego);
    sc.ring = RING_DEV; sc.ringVal = dev / 0.15f;
  } else {
    sc.title = "Z" UE "NDUNG";
    fmt(sc.big, sizeof sc.big, v.pw, 1);
    strcpy(sc.bigLabel, "Puls ms");
    sc.s[0] = {"", DEG, "Z" UE "ndung", 0};
    snprintf(sc.s[0].val, sizeof sc.s[0].val, "%d", v.adv);
    sc.s[1] = {"", "kPa", "MAP", 0};
    snprintf(sc.s[1].val, sizeof sc.s[1].val, "%d", v.map);
    sc.ring = RING_STATUS;
  }
  return sc;
}

Screen screenEngine() {
  Screen sc{};
  if (emergency) {
    // Oil and coolant limits are still placeholders -> no colour yet.
    sc.title = "MOTOR";
    fmt(sc.big, sizeof sc.big, v.oil, 1);
    strcpy(sc.bigLabel, "\xd6l bar");
    sc.s[0] = {"", DEG "C", "K" UE "hlm.", 0};
    snprintf(sc.s[0].val, sizeof sc.s[0].val, "%d", v.clt);
    sc.s[1] = {"", "%", "Tank", stTank(v.tank)};
    snprintf(sc.s[1].val, sizeof sc.s[1].val, "%d", v.tank);
    sc.ring = RING_STATUS;
    return sc;
  }
  sc.pages = 2; sc.page = page[2];
  if (page[2] == 0) {
    sc.title = "MOTOR";
    snprintf(sc.big, sizeof sc.big, "%d", v.duty);
    strcpy(sc.bigLabel, "Duty %");
    sc.bigSt = stDuty(v.duty);
    sc.s[0] = {"", DEG "C", "Luft", stIat(v.iat)};
    snprintf(sc.s[0].val, sizeof sc.s[0].val, "%d", v.iat);
    sc.s[1] = {"", "%", v.fanManual ? "L" UE "fter M" : "L" UE "fter", (uint8_t)(v.fanManual ? 1 : 0)};
    snprintf(sc.s[1].val, sizeof sc.s[1].val, "%d", v.fan);
    sc.ring = RING_BAR; sc.ringVal = v.duty / 100.0f;
  } else {
    sc.title = "LAST";
    snprintf(sc.big, sizeof sc.big, "%d", v.tps);
    strcpy(sc.bigLabel, "Drossel %");
    sc.s[0] = {"", "kPa", "MAP", 0};
    snprintf(sc.s[0].val, sizeof sc.s[0].val, "%d", v.map);
    sc.s[1] = {"", "", "Anf. D47", 0};
    strcpy(sc.s[1].val, v.d47 ? "AN" : "aus");
    sc.ring = RING_STATUS;
  }
  return sc;
}

// ---------- drawing ----------
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
  uint16_t ringCol = sc.ringCol ? sc.ringCol : STATUS_COL[worst];

  g->fillScreen(RGB565_BLACK);
  switch (sc.ring) {
    case RING_STATUS:
      arc(135, 405, ringCol);
      break;
    case RING_BAR:
      arc(135, 405, C_TRACK);
      arc(135, 135 + 270 * constrain(sc.ringVal, 0.01f, 1.0f), ringCol);
      break;
    case RING_DEV: {
      // Deviation from target, centred at the top: right = lean, left = rich; full side = 0.15.
      arc(135, 405, C_TRACK);
      float d = 135 * constrain(sc.ringVal, -1.0f, 1.0f);
      if (fabsf(d) < 3) d = d < 0 ? -3 : 3;
      if (d > 0) arc(270, 270 + d, ringCol); else arc(270 + d, 270, ringCol);
      g->fillRect(119, 1, 3, 16, C_TEXT);
      break;
    }
  }

  int16_t bigY = 128, labelY = 150, smallY = 177, smallLY = 195, sepY = 158;
  const uint8_t *bigFont = u8g2_font_logisoso42_tf;
  if (sc.bigLarge) {
    bigY = 142; labelY = 161; smallY = 184; smallLY = 201; sepY = 167;
    bigFont = u8g2_font_logisoso58_tr;
  }

  centered(sc.title, 120, 58, u8g2_font_helvB12_tf, C_GREY);
  if (sc.alarm) {
    g->fillRect(120 - 34, 66, 68, 18, C_BAD);
    centered(sc.alarm, 120, 80, u8g2_font_helvB10_tf, RGB565_BLACK);
  }
  uint16_t bigCol = sc.bigCol ? sc.bigCol : sc.bigSt ? STATUS_COL[sc.bigSt] : RGB565_WHITE;
  centered(sc.big, 120, bigY, bigFont, bigCol);
  centered(sc.bigLabel, 120, labelY, u8g2_font_helvB14_tf, C_GREY);

  g->drawFastVLine(120, sepY, 38, C_SEP);
  for (int j = 0; j < 2; j++) {
    int16_t x = j ? 158 : 82;
    const Small &s = sc.s[j];
    valueWithUnit(s.val, s.unit, x, smallY, s.st ? STATUS_COL[s.st] : C_TEXT);
    centered(s.label, x, smallLY, u8g2_font_helvB12_tf, C_GREY);
  }
  if (emergency) {
    centered("NOTBETRIEB", 120, 228, u8g2_font_helvB10_tf, C_WARN);
  } else {
    for (int p = 0; p < sc.pages; p++)
      g->fillCircle(120 - (sc.pages - 1) * 8 + p * 16, 222, 4, p == sc.page ? 0xDEFB : 0x4208);
  }
}

// FAULT: display stays red with "F" until the next shift command.
void renderFault() {
  g->fillScreen(C_FAULT_BG);
  centered("SCHALTUNG", 120, 58, u8g2_font_helvB12_tf, RGB565_WHITE);
  centered("F", 120, 150, u8g2_font_logisoso58_tr, RGB565_WHITE);
  centered("neu schalten", 120, 182, u8g2_font_helvB14_tf, RGB565_WHITE);
}

void push(int d) { panel[d]->draw16bitRGBBitmap(0, 0, g->getFramebuffer(), 240, 240); }

void drawDisplay(int d) {
  if (d == 0) {
    if (v.shift == SH_FAULT) renderFault(); else render(screenShift());
  } else {
    render(d == 1 ? screenMixture() : screenEngine());
  }
  push(d);
}

void drawAll() { for (int d = 0; d < NUM_DISPLAYS; d++) drawDisplay(d); }

// Whole display flashes once (shift aborted = light red, gear in = light green).
void flash(int d, uint16_t col) {
  g->fillScreen(col);
  push(d);
  delay(350);
  drawDisplay(d);
}

// Limit pop-up: a value leaving its range pulls its display back to the page showing it.
void popUp() {
  if (emergency) return;
  if (page[1] && (stLambda(v.lambda - v.lsoll) || stEgo(v.ego) || !v.sync)) page[1] = 0;
  if (page[2] && (stDuty(v.duty) || stIat(v.iat) || v.fanManual)) page[2] = 0;
}

void hold(uint32_t ms) {
  popUp();
  uint32_t t = millis();
  drawAll();
  uint32_t used = millis() - t;
  if (used < ms) delay(ms - used);
}

void step(const char *name) { Serial.printf("-- %s\n", name); }

void setup() {
  Serial.begin(115200);
  for (int8_t cs : PIN_CS) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
  for (int i = 0; i < NUM_WIRED; i++) {
    auto *bus = new Arduino_RPiPicoSPI(PIN_DC, PIN_CS[i], PIN_SCK, PIN_MOSI, PIN_MISO_UNUSED, spi0);
    // RST is shared: only display 1 pulses it (resets all before any init).
    panel[i] = new Arduino_GC9A01(bus, i == 0 ? PIN_RST : GFX_NOT_DEFINED, 0, true);
    if (!panel[i]->begin(SPI_HZ)) Serial.printf("display %d begin failed\n", i + 1);
  }
  for (int i = NUM_DISPLAYS; i < NUM_WIRED; i++) panel[i]->fillScreen(RGB565_BLACK);
  g = new Arduino_Canvas(240, 240, panel[0]);
  g->begin(GFX_SKIP_OUTPUT_BEGIN);
}

void loop() {
  v = Vehicle{};
  emergency = true;
  page[1] = page[2] = 0;

  step("power on: emergency mode until the Pi heartbeat arrives");
  hold(3000);
  step("driving off, RPM ring, shift range red");
  for (int i = 0; i <= 16; i++) {
    v.rpm = 900 + i * 380; v.kmh = i * 3;
    if (i == 9) v.gear = 2;
    hold(0);
  }
  v.rpm = 3400;
  hold(2000);

  step("Pi heartbeat present: normal mode");
  emergency = false;
  hold(5000);

  step("button: page 2 on GEMISCH and MOTOR");
  page[1] = page[2] = 1;
  hold(5000);
  page[1] = page[2] = 0;
  hold(1500);

  step("shift up: target gear shown, then gear in (green)");
  v.target = 3; v.shift = SH_TARGET;
  hold(1500);
  v.gear = 3; v.target = 0; v.shift = SH_IDLE;
  flash(0, C_FLASH_GREEN);
  hold(2000);

  step("shift aborted: clutch not fully pressed for 3 s (red)");
  v.target = 4; v.shift = SH_TARGET;
  hold(2000);
  v.target = 0; v.shift = SH_IDLE;
  flash(0, C_FLASH_RED);
  hold(2000);

  step("FAULT: stays red with F until the next command");
  v.shift = SH_FAULT;
  hold(3500);
  v.shift = SH_IDLE;
  hold(1500);

  step("tank pressure dropping, compressor on");
  v.comp = true; v.press = 7.6f;
  hold(2500);
  v.press = 6.4f;
  hold(2500);
  v.press = 9.4f; v.comp = false;
  hold(1500);

  step("lambda lean against target");
  v.lambda = 1.12f; v.ego = 12;
  hold(3000);
  v.lambda = 0.98f; v.ego = 3;

  step("fan switched to manual (yellow reminder)");
  v.fanManual = true; v.fan = 80;
  hold(3000);
  v.fanManual = false; v.fan = 35;

  step("pop-up: MOTOR on page 2, duty runs high -> back to page 1");
  page[2] = 1;
  hold(2500);
  v.duty = 88;
  hold(3500);
  v.duty = 28;

  step("sync lost");
  v.sync = false;
  hold(3000);
  v.sync = true;

  step("Pi heartbeat lost: emergency mode, tank low");
  emergency = true; v.kmh = 46; v.rpm = 3800; v.tank = 8;
  hold(5000);
}
