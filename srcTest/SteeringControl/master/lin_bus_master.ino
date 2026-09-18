/*******************************************************************************
 * LIN BUS MASTER - ARDUINO UNO R3 - INPUT TEST VERSION
 *
 * Board: Arduino Uno R3 (ATmega328P)
 * Power: 5V Laboratory Supply
 *
 * TEST MODE: Visual feedback for all inputs via LEDs
 *
 * LED WIRING:
 *   Pin 3  ──[ 220Ω ]──[LED]── GND   → Button 1 (ON when pressed)
 *   Pin 4  ──[ 220Ω ]──[LED]── GND   → Button 2 (ON when pressed)
 *   Pin 5  ──[ 220Ω ]──[LED]── GND   → Button 3 (ON when pressed)
 *   Pin 6  ──[ 220Ω ]──[LED]── GND   → POT1 (blink speed = pot position)
 *   Pin 7  ──[ 220Ω ]──[LED]── GND   → POT2 (blink speed = pot position)
 *   Pin 12 ──[ 220Ω ]──[LED]── GND   → Status (toggles on valid frame)
 *   Pin 13 (built-in)                 → Error (blinks on timeout)
 *
 * POT LED BEHAVIOR:
 *   Pot at 0%   → LED blinks slowly  (1000ms interval)
 *   Pot at 50%  → LED blinks medium  (~500ms interval)
 *   Pot at 100% → LED blinks fast    (50ms interval)
 *
 * Author: Claude Code
 * Date: 2026-02-13
 ******************************************************************************/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LIN_CS_PIN        2     // MCP2003 Chip Select

// Test LEDs
#define LED_BTN1          3     // Button 1 indicator
#define LED_BTN2          4     // Button 2 indicator
#define LED_BTN3          5     // Button 3 indicator
#define LED_POT1          6     // Potentiometer 1 blink
#define LED_POT2          7     // Potentiometer 2 blink

#define STATUS_LED_PIN    12    // Valid frame toggle
#define ERROR_LED_PIN     13    // Error blink

// ============================================================================
// LIN PROTOCOL CONSTANTS
// ============================================================================
#define LIN_BAUD_RATE     19200
#define LIN_FRAME_ID      0x3C
#define LIN_SYNC_BYTE     0x55
#define POLL_INTERVAL     100   // 100ms polling
#define RESPONSE_TIMEOUT  20    // 20ms timeout

// ============================================================================
// DATA
// ============================================================================
struct SteeringData {
  uint8_t buttons;
  uint16_t pot1;
  uint16_t pot2;
  bool valid;
};

SteeringData currentData = {0, 0, 0, false};

unsigned long lastPollTime = 0;
bool statusLedState = false;

// Non-blocking blink state for pot LEDs
unsigned long lastBlinkPot1 = 0;
unsigned long lastBlinkPot2 = 0;
bool blinkStatePot1 = false;
bool blinkStatePot2 = false;

/*******************************************************************************
 * SETUP
 ******************************************************************************/
void setup() {
  Serial.begin(LIN_BAUD_RATE);

  // MCP2003 CS pin - keep HIGH
  pinMode(LIN_CS_PIN, OUTPUT);
  digitalWrite(LIN_CS_PIN, HIGH);

  // Configure all LED pins
  pinMode(LED_BTN1, OUTPUT);
  pinMode(LED_BTN2, OUTPUT);
  pinMode(LED_BTN3, OUTPUT);
  pinMode(LED_POT1, OUTPUT);
  pinMode(LED_POT2, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  // Startup: blink all LEDs
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BTN1, HIGH);
    digitalWrite(LED_BTN2, HIGH);
    digitalWrite(LED_BTN3, HIGH);
    digitalWrite(LED_POT1, HIGH);
    digitalWrite(LED_POT2, HIGH);
    digitalWrite(STATUS_LED_PIN, HIGH);
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_BTN1, LOW);
    digitalWrite(LED_BTN2, LOW);
    digitalWrite(LED_BTN3, LOW);
    digitalWrite(LED_POT1, LOW);
    digitalWrite(LED_POT2, LOW);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(100);
  }

  delay(500);
}

/*******************************************************************************
 * MAIN LOOP
 ******************************************************************************/
void loop() {
  // Poll slave
  if (millis() - lastPollTime >= POLL_INTERVAL) {
    lastPollTime = millis();

    if (requestSlaveData()) {
      statusLedState = !statusLedState;
      digitalWrite(STATUS_LED_PIN, statusLedState);

      // Update button LEDs: ON when pressed, OFF when released
      digitalWrite(LED_BTN1, (currentData.buttons & 0x01) ? HIGH : LOW);
      digitalWrite(LED_BTN2, (currentData.buttons & 0x02) ? HIGH : LOW);
      digitalWrite(LED_BTN3, (currentData.buttons & 0x04) ? HIGH : LOW);

      // Clear error LED on success
      digitalWrite(ERROR_LED_PIN, LOW);
    } else {
      blinkErrorLED();
    }
  }

  // Non-blocking pot LED binking (runs every loop iteration)
  updatePotLED(LED_POT1, currentData.pot1, lastBlinkPot1, blinkStatePot1);
  updatePotLED(LED_POT2, currentData.pot2, lastBlinkPot2, blinkStatePot2);
}

/*******************************************************************************
 * UPDATE POT LED - Blink speed controlled by potentiometer value
 *
 * ADC 0    → 1000ms interval (slow blink)
 * ADC 4095 → 50ms interval   (fast blink)
 ******************************************************************************/
void updatePotLED(uint8_t pin, uint16_t adcValue,
                  unsigned long &lastBlink, bool &state) {
  if (!currentData.valid) return;

  // Map ADC (0-4095) to blink interval (1000ms-50ms)
  // Higher ADC = faster blink
  unsigned long interval = map(adcValue, 0, 4095, 1000, 50);

  if (millis() - lastBlink >= interval) {
    lastBlink = millis();
    state = !state;
    digitalWrite(pin, state);
  }
}

/*******************************************************************************
 * LIN COMMUNICATION FUNCTIONS
 ******************************************************************************/
bool requestSlaveData() {
  while (Serial.available()) Serial.read();

  sendBreakField();
  Serial.write(LIN_SYNC_BYTE);
  Serial.write(LIN_FRAME_ID);

  uint8_t response[6];
  unsigned long timeout = millis();

  for (int i = 0; i < 6; i++) {
    while (!Serial.available()) {
      if (millis() - timeout > RESPONSE_TIMEOUT) {
        currentData.valid = false;
        return false;
      }
    }
    response[i] = Serial.read();
  }

  uint8_t calculated = calculateChecksum(response, 5);
  if (response[5] != calculated) {
    currentData.valid = false;
    return false;
  }

  parseResponse(response);
  currentData.valid = true;
  return true;
}

void sendBreakField() {
  Serial.end();
  Serial.begin(9600);
  Serial.write(0x00);
  Serial.flush();
  Serial.end();
  Serial.begin(LIN_BAUD_RATE);
  delayMicroseconds(200);
}

void parseResponse(uint8_t* data) {
  currentData.buttons = data[0] & 0x7F;
  currentData.pot1 = ((uint16_t)data[1] << 4) | ((data[2] >> 4) & 0x0F);
  currentData.pot2 = ((uint16_t)data[3] << 4) | ((data[4] >> 4) & 0x0F);
}

uint8_t calculateChecksum(uint8_t* data, uint8_t length) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < length; i++) sum += data[i];
  return sum & 0xFF;
}

void blinkErrorLED() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(50);
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(50);
  }
}
