/*******************************************************************************
 * STEERING WHEEL CONTROLLER - RP2040 GPIO/UART DIAGNOSTIC
 *
 * Board: Waveshare RP2040-Tiny
 * Core: Arduino-Pico (Earle Philhower)
 *
 * DIAGNOSTIC TEST - Two phases:
 *
 * PHASE 1 (first 10 seconds) - UART LOOPBACK TEST:
 *   Connect GPIO0 directly to GPIO1 with a jumper wire (disconnect MCP2003).
 *   The RP2040 sends bytes on TX and checks if they arrive on RX.
 *   LED blinks FAST (5 Hz) = UART loopback works, GPIO0/1 are accessible
 *   LED stays OFF          = GPIO0/1 are NOT connected (need soldering?)
 *   LED pattern: 5 slow blinks at startup, then test begins.
 *
 * PHASE 2 (after 10 seconds) - MCP2003 LISTEN TEST:
 *   Reconnect MCP2003. LED blinks on ANY received byte.
 *   LED pattern changes to 1Hz heartbeat to show Phase 2 is active.
 *
 * STATUS LED: GPIO14
 ******************************************************************************/

#define LIN_TX_PIN        0
#define LIN_RX_PIN        1
#define LIN_CS_PIN        2
#define STATUS_LED_PIN    14

#define LIN_BAUD_RATE     19200

bool ledState = false;
unsigned long startTime = 0;
bool phase1Done = false;
int loopbackSuccess = 0;
int loopbackFail = 0;

void setup() {
  Serial1.setTX(LIN_TX_PIN);
  Serial1.setRX(LIN_RX_PIN);
  Serial1.begin(LIN_BAUD_RATE, SERIAL_8N1);

  pinMode(LIN_CS_PIN, OUTPUT);
  digitalWrite(LIN_CS_PIN, HIGH);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Startup: 5 slow blinks to indicate "Phase 1 starting"
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
  }

  delay(500);
  startTime = millis();
}

void loop() {
  unsigned long elapsed = millis() - startTime;

  if (elapsed < 10000) {
    // === PHASE 1: UART LOOPBACK TEST ===
    // Disconnect MCP2003! Short GPIO0 to GPIO1 with jumper wire.

    // Flush RX buffer
    while (Serial1.available()) Serial1.read();

    // Send a test byte
    Serial1.write(0xAA);
    Serial1.flush();

    // Wait briefly for loopback
    delay(5);

    if (Serial1.available()) {
      uint8_t received = Serial1.read();
      if (received == 0xAA) {
        loopbackSuccess++;
        // Fast blink = success!
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
      } else {
        loopbackFail++;
        delay(200);
      }
    } else {
      // Nothing received - GPIO0/1 not connected?
      loopbackFail++;
      delay(200);
    }

  } else {
    // === PHASE 2: MCP2003 LISTEN TEST ===
    // Reconnect MCP2003, remove GPIO0-GPIO1 jumper.

    if (!phase1Done) {
      phase1Done = true;

      // Signal Phase 2 start: LED result summary
      // Long ON = loopback worked, Long OFF = loopback failed
      if (loopbackSuccess > 0) {
        // GPIO0/1 work! 3 long blinks
        for (int i = 0; i < 3; i++) {
          digitalWrite(STATUS_LED_PIN, HIGH);
          delay(500);
          digitalWrite(STATUS_LED_PIN, LOW);
          delay(200);
        }
      } else {
        // GPIO0/1 FAILED! 10 very fast blinks
        for (int i = 0; i < 10; i++) {
          digitalWrite(STATUS_LED_PIN, HIGH);
          delay(50);
          digitalWrite(STATUS_LED_PIN, LOW);
          delay(50);
        }
      }
      delay(1000);
    }

    // Heartbeat: 1Hz blink to show Phase 2 is active
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat >= 500) {
      lastHeartbeat = millis();
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
    }

    // Check for ANY byte from MCP2003
    if (Serial1.available()) {
      Serial1.read();
      // Override heartbeat: rapid blink to show data received!
      for (int i = 0; i < 5; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(30);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(30);
      }
    }
  }
}
