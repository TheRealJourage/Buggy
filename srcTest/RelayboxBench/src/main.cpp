// Relay box controller bench test (Nucleo-F446RE).
// Pin map: Vault Buggy/Relaisbox.md "Pinbelegung Nucleo-F446RE", plan viewer/nucleo-heck.html.
// All outputs start LOW. Commands over the ST-Link virtual COM port (115200 baud), see TESTPLAN.md.
#include <Arduino.h>

// ---------------- pins ----------------
struct Relay { const char *name; uint32_t pin; };
// driver chips A/B/C, IN1..IN7 (wire colours lila, gruen, orange, weiss, gelb, blau, grau)
static const Relay RELAYS[] = {
  {"A1", PA9},  {"A2", PA8},  {"A3", PB10}, {"A4", PB4},  {"A5", PB5},  {"A6", PB3},  {"A7", PA10},
  {"B1", PB12}, {"B2", PB2},  {"B3", PB1},  {"B4", PB15}, {"B5", PB14}, {"B6", PB13}, {"B7", PC4},
  {"C1", PB8},  {"C2", PB9},  {"C3", PC5},  {"C4", PA6},  {"C5", PA7},  {"C6", PB6},  {"C7", PA5},
};
static const int NREL = sizeof(RELAYS) / sizeof(RELAYS[0]);
static const char *COLOURS[] = {"lila", "gruen", "orange", "weiss", "gelb", "blau", "grau"};

static const uint32_t PIN_PWM = PA15;   // fan PWM, BC547B pulls the J293 line low when the pin is HIGH
static const uint32_t PIN_ZL = PB7;     // ZL180 coil low side (IRLZ44N gate)
static const uint32_t PIN_D47 = PA0;    // Speeduino D47, switches to GND (pull-up R15, diode D7)
static const uint32_t PIN_ZS = PA1;     // ignition switch state, to GND (R16, D6)
static const uint32_t PIN_WB = PA4;     // hazard switch at the box, to GND (R17, D5)
static const uint32_t PIN_K15 = PB0;    // Kl. 15 feedback, divider R13/R14
static const uint32_t PIN_BAT = PC1;    // battery voltage, divider R11/R12
static const uint32_t PIN_TANK = PC0;   // fuel sender, 1k pull-up + 1k series

Uart ser3(PC11, PC10);                  // UART4: RX = PC11 (via R8 from D14), TX = PC10 (level shifter ch. 4 -> D15)

static float divider = 6.0f;            // (R_top + R_bottom) / R_bottom: 10k/2k = 6, 10k/1k = 11
static const float VREF = 3.3f;

// ---------------- helpers ----------------
static String line;

static bool keyPressed() {
  bool hit = false;
  while (Serial.available()) { char ch = Serial.read(); if (ch != '\r' && ch != '\n') hit = true; }
  return hit;
}

static float pinVolts(uint32_t pin) { return analogRead(pin) * VREF / 4095.0f; }

static void allOff() {
  for (int i = 0; i < NREL; i++) digitalWrite(RELAYS[i].pin, LOW);
  digitalWrite(PIN_ZL, LOW);
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_PWM, LOW);
}

static void help() {
  Serial.println(F(
    "\n=== Relaisbox Bench-Test (Nucleo-F446RE) ===\n"
    "h            Hilfe\n"
    "i            Eingaenge einmal lesen\n"
    "w            Eingaenge alle 0,5 s (beliebige Taste = Stopp)\n"
    "k <faktor>   Teilerfaktor Kl.15/Batterie (6 = 10k/2k, 11 = 10k/1k)\n"
    "r <A1..C7> <0|1>   ein Relaiskanal aus/an,  r all 0  = alle aus\n"
    "rs           alle 21 Kanaele nacheinander je 1 s an (Taste = Stopp)\n"
    "p <0..100>   Luefter-PWM 25 Hz, Tastgrad der LEITUNG (Low-Anteil), p off = Pin aus\n"
    "z <0|1>      ZL180-Treiber (PB7) aus/an\n"
    "s            Serial3-Schleifentest (Bruecke D14<->D15 an J3, 5 V an E5V noetig)\n"
    "sm           Serial3 10 s mithoeren (Hex)\n"
    "c l          CAN Loopback (intern, ohne Bus)\n"
    "c n          CAN 10 Frames senden, Fehlerzaehler anzeigen\n"
    "c r          CAN 10 s mithoeren\n"
    "c b <250|500> CAN-Bitrate in kbit/s (Standard 250)\n"
    "a            alles aus"));
}

// ---------------- inputs ----------------
static void printInputs() {
  float vk = pinVolts(PIN_K15), vb = pinVolts(PIN_BAT), vt = pinVolts(PIN_TANK);
  float rt = vt < VREF - 0.02f ? 1000.0f * vt / (VREF - vt) : -1.0f;
  Serial.print("D47="); Serial.print(digitalRead(PIN_D47) ? "HIGH(offen)" : "LOW(Masse)");
  Serial.print("  ZS="); Serial.print(digitalRead(PIN_ZS) ? "HIGH(offen)" : "LOW(Masse)");
  Serial.print("  WB="); Serial.print(digitalRead(PIN_WB) ? "HIGH(offen)" : "LOW(Masse)");
  Serial.print("  | K15 pin "); Serial.print(vk, 3); Serial.print(" V -> "); Serial.print(vk * divider, 2); Serial.print(" V");
  Serial.print("  BAT pin "); Serial.print(vb, 3); Serial.print(" V -> "); Serial.print(vb * divider, 2); Serial.print(" V");
  Serial.print("  TANK pin "); Serial.print(vt, 3); Serial.print(" V -> ");
  if (rt < 0) Serial.println("offen"); else { Serial.print(rt, 0); Serial.println(" Ohm"); }
}

// ---------------- relays ----------------
static int relayIndex(const String &n) {
  for (int i = 0; i < NREL; i++) if (n.equalsIgnoreCase(RELAYS[i].name)) return i;
  return -1;
}

static void relaySequence() {
  Serial.println("Sequenz: jeder Kanal 1 s an. Taste = Stopp.");
  for (int i = 0; i < NREL; i++) {
    digitalWrite(RELAYS[i].pin, HIGH);
    Serial.print(RELAYS[i].name); Serial.print(" ("); Serial.print(COLOURS[i % 7]); Serial.println(") AN");
    uint32_t t = millis();
    while (millis() - t < 1000) if (keyPressed()) { allOff(); Serial.println("abgebrochen, alles aus"); return; }
    digitalWrite(RELAYS[i].pin, LOW);
  }
  Serial.println("Sequenz fertig");
}

// ---------------- fan PWM ----------------
static HardwareTimer *pwmTim = nullptr;
static uint32_t pwmCh = 0;

static void setLineDuty(int lineDuty) {
  // pin HIGH = line LOW, so pin duty = line low share
  if (!pwmTim) {
    TIM_TypeDef *inst = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PIN_PWM), PinMap_PWM);
    pwmCh = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PIN_PWM), PinMap_PWM));
    pwmTim = new HardwareTimer(inst);
  }
  pwmTim->setPWM(pwmCh, PIN_PWM, 25, constrain(lineDuty, 0, 100));
  Serial.print("PWM 25 Hz, Leitung "); Serial.print(lineDuty); Serial.println(" % Low-Anteil (Pin-Tastgrad gleich)");
}

static void pwmOff() {
  if (pwmTim) { pwmTim->pause(); delete pwmTim; pwmTim = nullptr; }
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_PWM, LOW);
  Serial.println("PWM aus: Pin LOW, Leitung ueber Pull-up auf Kl. 15");
}

// ---------------- Serial3 ----------------
static int loopbackAt(uint32_t baud) {
  ser3.begin(baud);
  delay(20);
  while (ser3.available()) ser3.read();
  int ok = 0;
  for (int b = 0; b < 256; b++) {
    ser3.write((uint8_t)b);
    uint32_t t = millis();
    while (!ser3.available() && millis() - t < 20) {}
    if (ser3.available() && ser3.read() == b) ok++;
  }
  return ok;
}

static void serialLoopback() {
  Serial.println("Serial3-Schleife: J3 D14 <-> D15 gebrueckt? 5 V an E5V?");
  uint32_t bauds[] = {9600, 115200};
  for (uint32_t bd : bauds) {
    int ok = loopbackAt(bd);
    Serial.print(bd); Serial.print(" Baud: "); Serial.print(ok); Serial.print("/256 Bytes korrekt");
    Serial.println(ok == 256 ? "  -> OK" : "  -> FEHLER");
  }
}

static void serialMonitor() {
  ser3.begin(115200);
  Serial.println("Serial3 mithoeren 10 s (115200 Baud)...");
  uint32_t t = millis(); int n = 0;
  while (millis() - t < 10000) {
    while (ser3.available()) {
      uint8_t c = ser3.read();
      if (c < 16) Serial.print('0');
      Serial.print(c, HEX); Serial.print(++n % 16 ? ' ' : '\n');
    }
    if (keyPressed()) break;
  }
  Serial.print("\n"); Serial.print(n); Serial.println(" Bytes empfangen");
}

// ---------------- CAN (HAL, CAN1 on PA11/PA12) ----------------
static CAN_HandleTypeDef hcan;
static bool canUp = false;
static uint32_t canKbit = 250;

static bool canStart(uint32_t mode) {
  if (canUp) { HAL_CAN_Stop(&hcan); HAL_CAN_DeInit(&hcan); canUp = false; }
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_CAN1_CLK_ENABLE();
  GPIO_InitTypeDef g = {};
  g.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOA, &g);
  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  uint32_t presc = pclk / (canKbit * 1000UL * 18UL);   // 18 tq: 1 + 13 + 4, sample point 78 %
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = presc;
  hcan.Init.Mode = mode;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) { Serial.println("CAN Init fehlgeschlagen"); return false; }
  CAN_FilterTypeDef f = {};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &f);
  if (HAL_CAN_Start(&hcan) != HAL_OK) { Serial.println("CAN Start fehlgeschlagen"); return false; }
  canUp = true;
  Serial.print("CAN "); Serial.print(canKbit); Serial.print(" kbit/s, PCLK1 "); Serial.print(pclk / 1000000UL);
  Serial.print(" MHz, Prescaler "); Serial.println(presc);
  return true;
}

static void canStatus() {
  uint32_t esr = CAN1->ESR;
  Serial.print("TEC="); Serial.print((esr >> 16) & 0xFF);
  Serial.print(" REC="); Serial.print((esr >> 24) & 0xFF);
  Serial.print(" LEC="); Serial.print((esr >> 4) & 0x7);
  Serial.print(" (0 ok, 3 ACK-Fehler = kein anderer Knoten, 4/5 Bitfehler)");
  Serial.print(" BOFF="); Serial.println((esr >> 2) & 1);
}

static bool canSend(uint32_t id, uint8_t n) {
  CAN_TxHeaderTypeDef h = {};
  h.StdId = id; h.IDE = CAN_ID_STD; h.RTR = CAN_RTR_DATA; h.DLC = 8; h.TransmitGlobalTime = DISABLE;
  uint8_t d[8] = {0xB0, 0x66, n, 1, 2, 3, 4, 5};
  uint32_t mb;
  if (HAL_CAN_AddTxMessage(&hcan, &h, d, &mb) != HAL_OK) return false;
  uint32_t t = millis();
  while (HAL_CAN_IsTxMessagePending(&hcan, mb) && millis() - t < 50) {}
  return true;
}

static void canDump(uint32_t ms) {
  uint32_t t = millis(); int n = 0;
  while (millis() - t < ms) {
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
      CAN_RxHeaderTypeDef h; uint8_t d[8];
      HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &h, d);
      n++;
      Serial.print("RX id 0x"); Serial.print(h.IDE == CAN_ID_STD ? h.StdId : h.ExtId, HEX);
      Serial.print(" dlc "); Serial.print(h.DLC); Serial.print(":");
      for (uint32_t i = 0; i < h.DLC; i++) { Serial.print(' '); if (d[i] < 16) Serial.print('0'); Serial.print(d[i], HEX); }
      Serial.println();
    }
    if (keyPressed()) break;
  }
  Serial.print(n); Serial.println(" Frames empfangen");
}

static void canLoopback() {
  if (!canStart(CAN_MODE_LOOPBACK)) return;
  for (uint8_t i = 0; i < 5; i++) canSend(0x123, i);
  canDump(200);
  canStatus();
  Serial.println("Erwartet: 5 Frames empfangen, TEC=0. Prueft nur den Controller im Chip, nicht Modul/Bus.");
}

static void canNormalSend() {
  if (!canStart(CAN_MODE_NORMAL)) return;
  for (uint8_t i = 0; i < 10; i++) { canSend(0x123, i); delay(20); }
  canStatus();
  Serial.println("Ohne zweiten Knoten: TEC steigt, LEC=3 (ACK fehlt) -> Modul sendet. LEC=4/5 oder BOFF -> Verdrahtung/Modul pruefen.");
}

// ---------------- command parser ----------------
static void handle(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  String a = cmd, b = "", c = "";
  int s1 = cmd.indexOf(' ');
  if (s1 > 0) { a = cmd.substring(0, s1); b = cmd.substring(s1 + 1); b.trim(); int s2 = b.indexOf(' '); if (s2 > 0) { c = b.substring(s2 + 1); c.trim(); b = b.substring(0, s2); } }
  if (a == "h") help();
  else if (a == "i") printInputs();
  else if (a == "w") { Serial.println("Taste = Stopp"); while (!keyPressed()) { printInputs(); delay(500); } }
  else if (a == "k") { float f = b.toFloat(); if (f > 1) { divider = f; Serial.print("Teilerfaktor "); Serial.println(divider); } else Serial.println("k <faktor>, z. B. k 6"); }
  else if (a == "r" && b.equalsIgnoreCase("all")) { for (int i = 0; i < NREL; i++) digitalWrite(RELAYS[i].pin, c == "1" ? HIGH : LOW); Serial.println(c == "1" ? "alle 21 AN" : "alle aus"); }
  else if (a == "r") {
    int i = relayIndex(b);
    if (i < 0 || (c != "0" && c != "1")) { Serial.println("r <A1..C7> <0|1>"); return; }
    digitalWrite(RELAYS[i].pin, c == "1" ? HIGH : LOW);
    Serial.print(RELAYS[i].name); Serial.print(" ("); Serial.print(COLOURS[i % 7]); Serial.println(c == "1" ? ") AN" : ") aus");
  }
  else if (a == "rs") relaySequence();
  else if (a == "p") { if (b == "off") pwmOff(); else setLineDuty(b.toInt()); }
  else if (a == "z") { digitalWrite(PIN_ZL, b == "1" ? HIGH : LOW); Serial.println(b == "1" ? "ZL180-Treiber AN (PB7 high)" : "ZL180-Treiber aus"); }
  else if (a == "s") serialLoopback();
  else if (a == "sm") serialMonitor();
  else if (a == "c" && b == "l") canLoopback();
  else if (a == "c" && b == "n") canNormalSend();
  else if (a == "c" && b == "r") { if (canStart(CAN_MODE_NORMAL)) canDump(10000); }
  else if (a == "c" && b == "b") { uint32_t k = c.toInt(); if (k == 250 || k == 500) { canKbit = k; Serial.print("CAN-Bitrate "); Serial.println(k); } else Serial.println("c b <250|500>"); }
  else if (a == "a") { allOff(); pwmOff(); Serial.println("alles aus"); }
  else { Serial.print("unbekannt: "); Serial.println(cmd); help(); }
}

void setup() {
  for (int i = 0; i < NREL; i++) { pinMode(RELAYS[i].pin, OUTPUT); digitalWrite(RELAYS[i].pin, LOW); }
  pinMode(PIN_ZL, OUTPUT); digitalWrite(PIN_ZL, LOW);
  pinMode(PIN_PWM, OUTPUT); digitalWrite(PIN_PWM, LOW);
  pinMode(PIN_D47, INPUT); pinMode(PIN_ZS, INPUT); pinMode(PIN_WB, INPUT);
  analogReadResolution(12);
  Serial.begin(115200);
  delay(300);
  help();
  Serial.print("> ");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') { if (line.length()) { Serial.println(); handle(line); line = ""; Serial.print("> "); } }
    else line += ch;
  }
}
