/*******************************************************************************
 * LIN BUS TEST - RP2040-TINY (slave side)
 *
 * Wiring test without the LIN protocol. Controlled over USB serial (115200):
 *
 *   q  QUIET (default): RP2040 does not send, only counts incoming bytes.
 *      -> Uno in SEND mode: expect ~1900 x 0x55 per second here
 *   s  SEND: RP2040 sends 0xA5 back-to-back and checks its own echo.
 *      -> expect ~1900 echo 0xA5/s; the Uno (QUIET) toggles its green LED
 *   e  single echo test: sends one 0xA5 and reports whether it came back
 *
 * Status line once per second:
 *   mode | rx/s total | 0x55 (from Uno) | 0xA5 (own echo) | other | CS
 *
 * Status LED: GPIO14 (220R) blinks on received bytes.
 ******************************************************************************/

#define LIN_TX_PIN   0
#define LIN_RX_PIN   1
#define LIN_CS_PIN   2
#define LED_PIN      14

#define TEST_BYTE_UNO    0x55
#define TEST_BYTE_RP2040 0xA5

bool sendMode = false;
uint32_t cntTotal = 0, cntUno = 0, cntEcho = 0, cntOther = 0;
uint8_t lastOther = 0;
unsigned long lastReport = 0, lastRx = 0;

void singleEchoTest() {
  while (Serial1.available()) Serial1.read();
  Serial1.write(TEST_BYTE_RP2040);
  Serial1.flush();
  unsigned long t = millis();
  while (!Serial1.available() && millis() - t < 5) {}
  if (!Serial1.available()) {
    Serial.println(F("ECHO: nothing came back -> TXD/CS/VBB/VSS of MCP2003 #1 or RXD path"));
  } else {
    uint8_t b = Serial1.read();
    if (b == TEST_BYTE_RP2040) Serial.println(F("ECHO: OK (0xA5)"));
    else { Serial.print(F("ECHO: wrong byte 0x")); Serial.println(b, HEX); }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // UART first so TXD idles HIGH, then CS -> MCP2003 enters Operation mode
  Serial1.setTX(LIN_TX_PIN);
  Serial1.setRX(LIN_RX_PIN);
  Serial1.begin(19200);
  pinMode(LIN_CS_PIN, OUTPUT);
  digitalWrite(LIN_CS_PIN, HIGH);

  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) {}
  Serial.println(F("RP2040 LIN bus test. Commands: q = quiet, s = send 0xA5, e = single echo test"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') { sendMode = true;  Serial.println(F("-> SEND 0xA5")); }
    if (c == 'q') { sendMode = false; Serial.println(F("-> QUIET")); }
    if (c == 'e') { sendMode = false; singleEchoTest(); }
  }

  if (sendMode) {
    while (Serial1.availableForWrite() > 0) Serial1.write(TEST_BYTE_RP2040);
  }

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    cntTotal++;
    if (b == TEST_BYTE_UNO) cntUno++;
    else if (b == TEST_BYTE_RP2040) cntEcho++;
    else { cntOther++; lastOther = b; }
    lastRx = millis();
  }
  digitalWrite(LED_PIN, millis() - lastRx < 50 && (millis() / 50) % 2);

  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    Serial.print(sendMode ? F("SEND ") : F("QUIET"));
    Serial.print(F(" | rx/s ")); Serial.print(cntTotal);
    Serial.print(F(" | 0x55 from Uno ")); Serial.print(cntUno);
    Serial.print(F(" | 0xA5 echo ")); Serial.print(cntEcho);
    Serial.print(F(" | other ")); Serial.print(cntOther);
    if (cntOther) { Serial.print(F(" (last 0x")); Serial.print(lastOther, HEX); Serial.print(')'); }
    Serial.print(F(" | CS ")); Serial.println(digitalRead(LIN_CS_PIN) ? F("HIGH") : F("LOW"));
    cntTotal = cntUno = cntEcho = cntOther = 0;
  }
}
