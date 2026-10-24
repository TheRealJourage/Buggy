/*******************************************************************************
 * LIN BUS MASTER - ARDUINO UNO R3
 *
 * Board: Arduino Uno R3 (ATmega328P)
 * Power: Uno via USB, MCP2003 VBB = 12V lab supply
 *
 * DESCRIPTION:
 * LIN bus master/commander that polls the RP2040 slave every 100ms for
 * steering wheel buttons, potentiometers and Hall paddles. Validates received data
 * and displays decoded values via Serial Monitor and status LEDs.
 *
 * LIN PROTOCOL BASICS:
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ Break Field  │ Sync │ PID  │ Data Bytes (1-8)        │ Checksum    │
 * │ 13+ bits LOW │ 0x55 │ 0x3C │ Button + ADC0..ADC3     │ Modular Sum │
 * └─────────────────────────────────────────────────────────────────────┘
 *
 * TIMING DIAGRAM:
 * Master:  |<---- BREAK ---->|SYNC|PID |
 *          |_________________|0x55|0x3C|
 *          └─ 13 bits = 676µs min @ 19200 baud
 *
 * Slave:                              |BTN| ADC0+ADC1 (3) | ADC2+ADC3 (3) |CHK|
 *
 * MCP2003 LIN TRANSCEIVER CONNECTIONS (see README, rev. 3.0):
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ MCP2003 Pin │ Function │ Arduino Connection                         │
 * ├─────────────┼──────────┼────────────────────────────────────────────┤
 * │ Pin 1 (RXD) │ Receive  │ Pin 0 (RX) + 4.7kΩ pull-up to 5V           │
 * │ Pin 2 (CS)  │ Chip Sel │ Pin 2, driven HIGH by firmware, no pull-up │
 * │ Pin 3 (WAKE)│ Wake-up  │ Not connected                              │
 * │ Pin 4 (TXD) │ Transmit │ Pin 1 (TX)                                 │
 * │ Pin 5 (VSS) │ Ground   │ GND (common with lab supply and RP2040)    │
 * │ Pin 6 (LBUS)│ LIN Bus  │ To RP2040 MCP2003 LBUS                     │
 * │             │          │ + master pull-up: 12V → 1N4148 → 1kΩ       │
 * │             │          │ + 220pF to GND                             │
 * │ Pin 7 (VBB) │ Battery  │ 12V lab supply (MCP2003 needs > 5.5V)      │
 * │ Pin 8 (VREN)│ VReg En  │ Output - not connected                     │
 * └─────────────┴──────────┴────────────────────────────────────────────┘
 *
 * ECHO: the transceiver mirrors the bus on RXD, so the master receives its
 * own SYNC and PID before the slave response. They are read and checked
 * before the 8 response bytes.
 *
 * KEY (challenge-response, lib/steering_key):
 *   While locked, every KEY_RETRY_INTERVAL one poll slot is used for:
 *   PID 0x50: we send 8 random bytes + checksum (challenge)
 *   PID 0x11: the wheel answers HMAC-SHA256(secret, challenge)[0..7] + checksum
 *   Match -> unlocked, ECU_ENABLE_PIN high. No valid input frame for
 *   KEY_LOST_TIMEOUT (wheel pulled off) -> locked again.
 *   Test master only: random() is seeded from floating analog pins, the STM32
 *   hub uses its hardware RNG. Relocking while driving is still an open decision.
 *
 * HALL HOLD: a pull beyond HALL_ACTIVE_PERCENT starts a new setpoint, the
 *   highest travel of that pull is kept after the paddle is released.
 *
 * STATUS INDICATORS:
 * - Pin 12 (Status LED): Toggles on each valid frame received
 * - Pin 13 (Error LED): Blinks on LIN errors (timeout, checksum fail)
 *
 * SERIAL OUTPUT (115200 baud when USB connected):
 * - Button states: names of the pressed buttons, e.g. "BTN: hupe gelb"
 * - ADC0: Raw value (0-4095) and percentage (0-100%)
 * - ADC1: Raw value (0-4095) and percentage (0-100%)
 * - HALL1/HALL2: Raw value and percentage of the calibrated travel
 *
 * NOTE: Serial debugging conflicts with LIN on pins 0/1!
 * - For initial testing WITH USB: Serial output works but LIN may be unreliable
 * - For production WITHOUT USB: LIN works perfectly, use LED patterns for status
 *
 * Author: Claude Code
 * Date: 2026-09-19
 * Version: 3.4 (button bits as built, shared table in steering_buttons.h)
 ******************************************************************************/

#include <steering_key.h>
#include <steering_buttons.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LIN_CS_PIN        2     // Chip Select → MCP2003 Pin 2 (CS)
#define STATUS_LED_PIN    12    // Status LED (toggles on valid frames)
#define ERROR_LED_PIN     13    // Error LED (blinks on errors)
#define ECU_ENABLE_PIN    3     // HIGH while unlocked (placeholder for the ECU supply)

// Note: Pins 0 and 1 are used by hardware Serial for LIN communication
// Pin 0 (RX) ← MCP2003 Pin 1 (RXD)
// Pin 1 (TX) → MCP2003 Pin 4 (TXD)

// ============================================================================
// LIN PROTOCOL CONSTANTS
// ============================================================================
#define LIN_BAUD_RATE     19200           // Standard LIN baud rate
#define LIN_FRAME_ID      0x3C            // Protected ID for input request
#define LIN_SYNC_BYTE     0x55            // Standard LIN sync byte
#define LIN_PID_KEY_CHALLENGE 0x50        // ID 0x10, master -> wheel
#define LIN_PID_KEY_RESPONSE  0x11        // ID 0x11, wheel -> master
#define LIN_BREAK_BITS    13              // Break field duration (bits)

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
#define POLL_INTERVAL     100             // Poll slave every 100ms
#define RESPONSE_TIMEOUT  20              // Wait up to 20ms for slave response
#define ECHO_TIMEOUT      5               // Own SYNC+PID echo must be back within 5ms
#define ERROR_BLINK_MS    50              // Error LED: 3 blinks of 50ms on / 50ms off
#define BREAK_DURATION    700             // Break field: 700µs (13 bits @ 19200 = 677µs)
#define RESPONSE_LEN      8               // Buttons + 4x 12 bit packed (6) + checksum
#define CHALLENGE_ECHO_TIMEOUT 15         // SYNC+PID+9 bytes echo (~5.7ms at 19200)
#define KEY_RETRY_INTERVAL 500            // While locked: new challenge every 500ms
#define KEY_LOST_TIMEOUT  1000            // No valid input frame this long -> locked

// Hall clutch paddles (Simube, measured 2026-09-24 on 3.3V): released ~876,
// fully pulled ~3310. The deadband makes both end positions read exactly 0/100%.
#define HALL_RELEASED     876
#define HALL_FULL         3310
#define HALL_DEADBAND     30
#define HALL_ACTIVE_PERCENT 5             // Above this a pull starts a new setpoint

// ============================================================================
// DATA STRUCTURE
// ============================================================================
struct SteeringData {
  uint8_t buttons;                        // Bit = SteeringButton (steering_buttons.h)
  uint16_t pot1;                          // 12-bit ADC value (0-4095)
  uint16_t pot2;                          // 12-bit ADC value (0-4095)
  uint16_t hall1;                         // 12-bit ADC value (0-4095)
  uint16_t hall2;                         // 12-bit ADC value (0-4095)
  bool valid;                             // Data validity flag
};

SteeringData currentData = {0, 0, 0, 0, 0, false};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
unsigned long lastPollTime = 0;
bool statusLedState = false;
unsigned long errorBlinkStart = 0;
bool errorBlinkActive = false;

bool unlocked = false;
unsigned long lastKeyTry = 0;
unsigned long lastValidFrame = 0;

uint8_t hallHold[2] = {0, 0};             // Held setpoint per paddle (0-100%)
bool hallPulling[2] = {false, false};

/*******************************************************************************
 * SETUP - Initialize hardware and peripherals
 ******************************************************************************/
void setup() {
  // Initialize hardware Serial for LIN communication
  Serial.begin(LIN_BAUD_RATE);

  // Configure LIN CS pin - keep HIGH to enable MCP2003
  pinMode(LIN_CS_PIN, OUTPUT);
  digitalWrite(LIN_CS_PIN, HIGH);

  // Configure status LEDs
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  pinMode(ECU_ENABLE_PIN, OUTPUT);
  digitalWrite(ECU_ENABLE_PIN, LOW);

  seedRandom();

  // Startup sequence: Blink both LEDs 3 times
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(100);
  }

  delay(500);  // Allow MCP2003 to stabilize

  // NOTE: Serial.println() will interfere with LIN on pins 0/1!
  // Uncomment for initial testing with USB, comment out for production
  // Serial.end();
  // Serial.begin(115200);  // Switch to debug baud rate
  // Serial.println(F("=== LIN BUS MASTER ==="));
  // Serial.println(F("Arduino Uno @ 5V Lab Supply"));
  // Serial.println(F("Polling RP2040 slave every 100ms..."));

#ifdef LIN_DEBUG_PRINT
  Serial.println(steeringKeySelfTest() ? F("SELFTEST OK") : F("SELFTEST FAIL"));
#endif
}

/*******************************************************************************
 * MAIN LOOP
 ******************************************************************************/
void loop() {
  // Poll slave at regular intervals
  if (millis() - lastPollTime >= POLL_INTERVAL) {
    lastPollTime = millis();

    if (!unlocked && millis() - lastKeyTry >= KEY_RETRY_INTERVAL) {
      lastKeyTry = millis();
      unlocked = checkKey();
      digitalWrite(ECU_ENABLE_PIN, unlocked);
      printKeyResult();
      if (unlocked) lastValidFrame = millis();
    } else if (requestSlaveData()) {
      lastValidFrame = millis();
      updateHallHold();

      // Data received successfully - toggle status LED
      statusLedState = !statusLedState;
      digitalWrite(STATUS_LED_PIN, statusLedState);

      // Print data to Serial Monitor (if USB connected)
      printSteeringData();
    } else {
      // Error occurred - blink error LED
      startErrorBlink();
    }
  }

  if (unlocked && millis() - lastValidFrame > KEY_LOST_TIMEOUT) {
    unlocked = false;
    digitalWrite(ECU_ENABLE_PIN, LOW);
  }

  updateErrorLED();
}

/*******************************************************************************
 * REQUEST SLAVE DATA - Send LIN header and receive slave response
 *
 * LIN Frame Sequence:
 * 1. Send BREAK field (13 bits LOW = 676µs minimum)
 * 2. Send SYNC byte (0x55)
 * 3. Send PID (0x3C)
 * 4. Wait for slave response (8 bytes)
 * 5. Validate checksum
 * 6. Parse data
 *
 * Returns: true if valid data received, false on error
 ******************************************************************************/
bool requestSlaveData() {
  // Steps 1-3: BREAK, SYNC, PID
  if (!sendHeader(LIN_FRAME_ID)) {
    currentData.valid = false;
    return false;  // Own header not seen on the bus: transceiver/wiring or bus collision
  }

  // Step 4: Wait for slave response
  uint8_t response[RESPONSE_LEN];
  if (!readBytes(response, RESPONSE_LEN, RESPONSE_TIMEOUT)) {
    currentData.valid = false;
    return false;  // Timeout error
  }

  // Step 5: Validate checksum
  uint8_t calculatedChecksum = calculateChecksum(response, RESPONSE_LEN - 1);
  if (response[RESPONSE_LEN - 1] != calculatedChecksum) {
    currentData.valid = false;
    return false;  // Checksum error
  }

  // Step 6: Parse response data
  parseResponse(response);
  currentData.valid = true;

  return true;
}

/*******************************************************************************
 * SEND HEADER - BREAK + SYNC + PID, true if our own header echoed back
 ******************************************************************************/
bool sendHeader(uint8_t pid) {
  sendBreakField();  // its echo is discarded by Serial.end()

  // Clear anything that arrived around the baud rate switch
  while (Serial.available()) {
    Serial.read();
  }

  Serial.write(LIN_SYNC_BYTE);
  Serial.write(pid);

  uint8_t echo[2];
  return readBytes(echo, 2, ECHO_TIMEOUT) && echo[0] == LIN_SYNC_BYTE && echo[1] == pid;
}

/*******************************************************************************
 * CHECK KEY - Challenge the wheel, true if its answer matches our own HMAC
 ******************************************************************************/
bool checkKey() {
  uint8_t frame[STEERING_KEY_LEN + 1];
  for (uint8_t i = 0; i < STEERING_KEY_LEN; i++) {
    frame[i] = random(256);
  }
  frame[STEERING_KEY_LEN] = calculateChecksum(frame, STEERING_KEY_LEN);

  // Challenge frame: we publish the data ourselves
  if (!sendHeader(LIN_PID_KEY_CHALLENGE)) return false;
  Serial.write(frame, sizeof frame);
  uint8_t echo[sizeof frame];
  if (!readBytes(echo, sizeof echo, CHALLENGE_ECHO_TIMEOUT) || memcmp(echo, frame, sizeof frame) != 0) {
    return false;  // Collision or wiring fault
  }

  // Computed before the response header, the wheel has its answer ready by then
  uint8_t expected[STEERING_KEY_LEN];
  steeringKeyResponse(frame, expected);

  if (!sendHeader(LIN_PID_KEY_RESPONSE)) return false;
  uint8_t answer[STEERING_KEY_LEN + 1];
  if (!readBytes(answer, sizeof answer, RESPONSE_TIMEOUT)) return false;
  if (answer[STEERING_KEY_LEN] != calculateChecksum(answer, STEERING_KEY_LEN)) return false;
  return memcmp(answer, expected, STEERING_KEY_LEN) == 0;
}

// Test-grade entropy: LSBs of the floating analog inputs A0-A5
void seedRandom() {
  uint32_t seed = 0;
  for (uint8_t i = 0; i < 32; i++) {
    seed = (seed << 1) ^ analogRead(A0 + i % 6) ^ micros();
  }
  randomSeed(seed);
}

/*******************************************************************************
 * HALL HOLD - Keep the highest travel of the last pull as setpoint
 ******************************************************************************/
void updateHallHold() {
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t percent = getHallPercent(i + 1);
    if (percent > HALL_ACTIVE_PERCENT) {
      hallHold[i] = hallPulling[i] ? max(hallHold[i], percent) : percent;
      hallPulling[i] = true;
    } else {
      hallPulling[i] = false;
    }
  }
}

/*******************************************************************************
 * READ BYTES - Read `count` bytes, give up after `timeoutMs`
 ******************************************************************************/
bool readBytes(uint8_t* buffer, uint8_t count, unsigned long timeoutMs) {
  unsigned long start = millis();
  for (uint8_t i = 0; i < count; i++) {
    while (!Serial.available()) {
      if (millis() - start > timeoutMs) return false;
    }
    buffer[i] = Serial.read();
  }
  return true;
}

/*******************************************************************************
 * SEND BREAK FIELD - Generate LIN break (13 bits LOW)
 *
 * LIN Break Field Requirements:
 * - Duration: 13+ bit times LOW
 * - At 19200 baud: 1 bit = 52.08µs, so 13 bits = 676.04µs minimum
 * - We use 700µs for safety margin
 *
 * Implementation:
 * - Switch to 9600 baud (half speed)
 * - Send 0x00 (10 bits: start + 8 data + stop)
 * - At 9600 baud, 10 bits = 1.04ms ≈ 20 bits @ 19200 baud
 * - This creates a sufficiently long break field
 * - Restore 19200 baud for normal communication
 ******************************************************************************/
void sendBreakField() {
  Serial.end();                 // Close serial
  Serial.begin(9600);           // Switch to half speed
  Serial.write(0x00);           // Send null byte (creates extended LOW)
  Serial.flush();               // Wait for transmission complete
  Serial.end();                 // Close serial
  Serial.begin(LIN_BAUD_RATE);  // Restore normal baud rate
  delayMicroseconds(200);       // Small delay for stability
}

/*******************************************************************************
 * PARSE RESPONSE - Extract button states and ADC values
 *
 * Frame format (8 bytes):
 * ┌─────────┬───────────────────────┬───────────────────────┬──────────┐
 * │ Byte 0  │ Byte 1-3              │ Byte 4-6              │ Byte 7   │
 * │ Buttons │ POT1 + POT2 (2x12 bit)│ HALL1 + HALL2         │ Checksum │
 * └─────────┴───────────────────────┴───────────────────────┴──────────┘
 *
 * Button byte: bit = SteeringButton (steering_buttons.h), 1 = pressed
 * Two 12-bit values A, B packed into 3 bytes:
 *   b0 = A[11:4], b1 = A[3:0] << 4 | B[11:8], b2 = B[7:0]
 ******************************************************************************/
void unpackPair(uint8_t* in, uint16_t* a, uint16_t* b) {
  *a = ((uint16_t)in[0] << 4) | (in[1] >> 4);
  *b = ((uint16_t)(in[1] & 0x0F) << 8) | in[2];
}

void parseResponse(uint8_t* data) {
  currentData.buttons = data[0];
  unpackPair(&data[1], &currentData.pot1, &currentData.pot2);
  unpackPair(&data[4], &currentData.hall1, &currentData.hall2);
}

/*******************************************************************************
 * CALCULATE CHECKSUM - Simple modular sum
 *
 * Checksum = (Sum of all data bytes) % 256
 * Matches RP2040 slave checksum algorithm
 ******************************************************************************/
uint8_t calculateChecksum(uint8_t* data, uint8_t length) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum & 0xFF;  // Keep lower 8 bits
}

/*******************************************************************************
 * PRINT STEERING DATA - Output decoded values to Serial Monitor
 *
 * Only built with -DLIN_DEBUG_PRINT (env uno_master_debug). The text also goes
 * onto the LIN bus via D1; it contains no 'U' (0x55 = SYNC), so the slave
 * never sees a header in it. Keep it that way when changing the labels.
 ******************************************************************************/
void printSteeringData() {
#ifdef LIN_DEBUG_PRINT
  // Print button states
  Serial.print(F("BTN:"));
  if (currentData.buttons == 0) Serial.print(F(" -"));
  for (uint8_t i = 0; i < STEERING_BUTTON_COUNT; i++) {
    if (isButtonPressed((SteeringButton)i)) {
      Serial.print(' ');
      Serial.print(STEERING_BUTTON_NAME[i]);
    }
  }

  // Print ADC0 (raw value and percentage)
  Serial.print(F(" | POT1: "));
  Serial.print(currentData.pot1);
  Serial.print(F(" ("));
  Serial.print(map(currentData.pot1, 0, 4095, 0, 100));
  Serial.print(F("%)"));

  // Print ADC1 (raw value and percentage)
  Serial.print(F(" | POT2: "));
  Serial.print(currentData.pot2);
  Serial.print(F(" ("));
  Serial.print(map(currentData.pot2, 0, 4095, 0, 100));
  Serial.print(F("%)"));

  Serial.print(F(" | H1: "));
  Serial.print(currentData.hall1);
  Serial.print(F(" ("));
  Serial.print(getHallPercent(1));
  Serial.print(F("%) | H2: "));
  Serial.print(currentData.hall2);
  Serial.print(F(" ("));
  Serial.print(getHallPercent(2));
  Serial.print(F("%) | HOLD: "));
  Serial.print(hallHold[0]);
  Serial.print(F("% "));
  Serial.print(hallHold[1]);
  Serial.println(unlocked ? F("% | KEY OK") : F("% | KEY --"));
#endif
}

void printKeyResult() {
#ifdef LIN_DEBUG_PRINT
  Serial.println(unlocked ? F("KEY: OK, ECU on") : F("KEY: no valid answer"));
#endif
}

/*******************************************************************************
 * ERROR LED - Indicate LIN communication error without blocking the poll loop
 *
 * Error types:
 * - Echo missing: own header not seen on the bus
 * - Timeout: Slave not responding
 * - Checksum mismatch: Data corruption
 *
 * LED pattern: 3 quick blinks (restarted by every new error)
 ******************************************************************************/
void startErrorBlink() {
  errorBlinkStart = millis();
  errorBlinkActive = true;
}

void updateErrorLED() {
  if (!errorBlinkActive) return;
  unsigned long elapsed = millis() - errorBlinkStart;
  if (elapsed >= 6 * ERROR_BLINK_MS) {
    errorBlinkActive = false;
    digitalWrite(ERROR_LED_PIN, LOW);
    return;
  }
  digitalWrite(ERROR_LED_PIN, (elapsed / ERROR_BLINK_MS) % 2 == 0 ? HIGH : LOW);
}

/*******************************************************************************
 * UTILITY FUNCTIONS - Access current data
 ******************************************************************************/

bool isButtonPressed(SteeringButton button) {
  return (currentData.buttons & (1 << button)) != 0;
}

bool isHornPressed() {
  return isButtonPressed(BTN_HORN);
}

// Get potentiometer value (0-4095)
uint16_t getPotValue(uint8_t potNum) {
  if (potNum == 1) return currentData.pot1;
  if (potNum == 2) return currentData.pot2;
  return 0;
}

// Get potentiometer percentage (0-100%)
uint8_t getPotPercent(uint8_t potNum) {
  uint16_t value = getPotValue(potNum);
  return map(value, 0, 4095, 0, 100);
}

// Get Hall paddle travel (0-100%), clamped with a deadband at both ends
uint8_t getHallPercent(uint8_t hallNum) {
  uint16_t value = (hallNum == 1) ? currentData.hall1 : currentData.hall2;
  if (value <= HALL_RELEASED + HALL_DEADBAND) return 0;
  if (value >= HALL_FULL - HALL_DEADBAND) return 100;
  return map(value, HALL_RELEASED + HALL_DEADBAND, HALL_FULL - HALL_DEADBAND, 0, 100);
}

// Check if data is valid
bool isDataValid() {
  return currentData.valid;
}
