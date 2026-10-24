/*******************************************************************************
 * LIN BUS TEST - ARDUINO UNO (master side)
 *
 * Wiring test without the LIN protocol. Two modes, selected by a jumper:
 *
 *   D7 open (default)  QUIET: Uno does not send. Every byte arriving from the
 *                      RP2040 (0xA5) toggles the green LED (D12).
 *                      -> tests RP2040 -> MCP2003 #1 -> bus -> MCP2003 #2 -> Uno
 *
 *   D7 to GND          SEND: Uno sends 0x55 back-to-back and checks the echo
 *                      coming back through its own MCP2003.
 *                      Green LED on  = echo correct (own TX -> bus -> RX path OK)
 *                      Red LED blink = wrong bytes / no echo
 *                      -> LBUS reads ~6V on a multimeter (50% dominant)
 *
 * LEDs: D12 green (220R), D13 red (220R + onboard)
 * Disconnect MCP2003 from D0/D1 while uploading.
 ******************************************************************************/

#define LIN_CS_PIN     2
#define MODE_JUMPER    7
#define LED_OK         12
#define LED_ERR        13

#define TEST_BYTE_UNO    0x55
#define TEST_BYTE_RP2040 0xA5

unsigned long lastGood = 0;
unsigned long lastBad = 0;
unsigned long lastErrBlink = 0;
bool okToggle = false;

void setup() {
  pinMode(LED_OK, OUTPUT);
  pinMode(LED_ERR, OUTPUT);
  pinMode(MODE_JUMPER, INPUT_PULLUP);

  // UART first so TXD idles HIGH, then CS -> MCP2003 enters Operation mode (not Transmitter-Off)
  Serial.begin(19200);
  pinMode(LIN_CS_PIN, OUTPUT);
  digitalWrite(LIN_CS_PIN, HIGH);

  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_OK, HIGH); digitalWrite(LED_ERR, HIGH); delay(150);
    digitalWrite(LED_OK, LOW);  digitalWrite(LED_ERR, LOW);  delay(150);
  }
}

void loop() {
  bool sendMode = digitalRead(MODE_JUMPER) == LOW;
  unsigned long now = millis();

  if (sendMode) {
    while (Serial.availableForWrite() > 0) Serial.write(TEST_BYTE_UNO);
  }

  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (sendMode) {
      if (b == TEST_BYTE_UNO) lastGood = now; else lastBad = now;
    } else {
      if (b == TEST_BYTE_RP2040) { okToggle = !okToggle; lastGood = now; } else lastBad = now;
    }
  }

  if (sendMode) {
    digitalWrite(LED_OK, now - lastGood < 200 ? HIGH : LOW);
  } else {
    digitalWrite(LED_OK, now - lastGood < 200 ? okToggle : LOW);
  }

  // Red: blink while errors occur, or while sending without any echo
  bool error = (now - lastBad < 300) || (sendMode && now - lastGood > 200);
  if (error && now - lastErrBlink > 100) {
    lastErrBlink = now;
    digitalWrite(LED_ERR, !digitalRead(LED_ERR));
  } else if (!error) {
    digitalWrite(LED_ERR, LOW);
  }
}
