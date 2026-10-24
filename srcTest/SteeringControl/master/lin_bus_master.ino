/*******************************************************************************
 * LIN BUS MASTER - ARDUINO UNO R3
 *
 * Board: Arduino Uno R3 (ATmega328P)
 * Power: Uno via USB, MCP2003 VBB = 12V lab supply
 *
 * DESCRIPTION:
 * LIN bus master/commander that polls the RP2040 slave every 100ms for
 * steering wheel button and potentiometer data. Validates received data
 * and displays decoded values via Serial Monitor and status LEDs.
 *
 * LIN PROTOCOL BASICS:
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ Break Field  │ Sync │ PID  │ Data Bytes (1-8)        │ Checksum    │
 * │ 13+ bits LOW │ 0x55 │ 0x3C │ Button + ADC0 + ADC1    │ Modular Sum │
 * └─────────────────────────────────────────────────────────────────────┘
 *
 * TIMING DIAGRAM:
 * Master:  |<---- BREAK ---->|SYNC|PID |
 *          |_________________|0x55|0x3C|
 *          └─ 13 bits = 676µs min @ 19200 baud
 *
 * Slave:                              |BTN|ADC0_H|ADC0_L|ADC1_H|ADC1_L|CHK|
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
 * before the 6 response bytes.
 *
 * STATUS INDICATORS:
 * - Pin 12 (Status LED): Toggles on each valid frame received
 * - Pin 13 (Error LED): Blinks on LIN errors (timeout, checksum fail)
 *
 * SERIAL OUTPUT (115200 baud when USB connected):
 * - Button states: "BTN: [1][2][3][4][5][6][7]"
 * - ADC0: Raw value (0-4095) and percentage (0-100%)
 * - ADC1: Raw value (0-4095) and percentage (0-100%)
 *
 * NOTE: Serial debugging conflicts with LIN on pins 0/1!
 * - For initial testing WITH USB: Serial output works but LIN may be unreliable
 * - For production WITHOUT USB: LIN works perfectly, use LED patterns for status
 *
 * Author: Claude Code
 * Date: 2026-09-19
 * Version: 3.0 (echo handling, non-blocking error LED, 12V wiring)
 ******************************************************************************/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LIN_CS_PIN        2     // Chip Select → MCP2003 Pin 2 (CS)
#define STATUS_LED_PIN    12    // Status LED (toggles on valid frames)
#define ERROR_LED_PIN     13    // Error LED (blinks on errors)

// Note: Pins 0 and 1 are used by hardware Serial for LIN communication
// Pin 0 (RX) ← MCP2003 Pin 1 (RXD)
// Pin 1 (TX) → MCP2003 Pin 4 (TXD)

// ============================================================================
// LIN PROTOCOL CONSTANTS
// ============================================================================
#define LIN_BAUD_RATE     19200           // Standard LIN baud rate
#define LIN_FRAME_ID      0x3C            // Protected ID for input request
#define LIN_SYNC_BYTE     0x55            // Standard LIN sync byte
#define LIN_BREAK_BITS    13              // Break field duration (bits)

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
#define POLL_INTERVAL     100             // Poll slave every 100ms
#define RESPONSE_TIMEOUT  20              // Wait up to 20ms for slave response
#define ECHO_TIMEOUT      5               // Own SYNC+PID echo must be back within 5ms
#define ERROR_BLINK_MS    50              // Error LED: 3 blinks of 50ms on / 50ms off
#define BREAK_DURATION    700             // Break field: 700µs (13 bits @ 19200 = 677µs)

// ============================================================================
// DATA STRUCTURE
// ============================================================================
struct SteeringData {
  uint8_t buttons;                        // 7 button states (bits 0-6)
  uint16_t pot1;                          // 12-bit ADC value (0-4095)
  uint16_t pot2;                          // 12-bit ADC value (0-4095)
  bool valid;                             // Data validity flag
};

SteeringData currentData = {0, 0, 0, false};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
unsigned long lastPollTime = 0;
bool statusLedState = false;
unsigned long errorBlinkStart = 0;
bool errorBlinkActive = false;

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
}

/*******************************************************************************
 * MAIN LOOP
 ******************************************************************************/
void loop() {
  // Poll slave at regular intervals
  if (millis() - lastPollTime >= POLL_INTERVAL) {
    lastPollTime = millis();

    // Request data from slave
    if (requestSlaveData()) {
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

  updateErrorLED();
}

/*******************************************************************************
 * REQUEST SLAVE DATA - Send LIN header and receive slave response
 *
 * LIN Frame Sequence:
 * 1. Send BREAK field (13 bits LOW = 676µs minimum)
 * 2. Send SYNC byte (0x55)
 * 3. Send PID (0x3C)
 * 4. Wait for slave response (6 bytes)
 * 5. Validate checksum
 * 6. Parse data
 *
 * Returns: true if valid data received, false on error
 ******************************************************************************/
bool requestSlaveData() {
  // Step 1: Send LIN BREAK field (its echo is discarded by Serial.end())
  sendBreakField();

  // Clear anything that arrived around the baud rate switch
  while (Serial.available()) {
    Serial.read();
  }

  // Step 2+3: Send SYNC byte and PID
  Serial.write(LIN_SYNC_BYTE);
  Serial.write(LIN_FRAME_ID);

  // Our own header comes back through the transceiver first
  uint8_t echo[2];
  if (!readBytes(echo, 2, ECHO_TIMEOUT) || echo[0] != LIN_SYNC_BYTE || echo[1] != LIN_FRAME_ID) {
    currentData.valid = false;
    return false;  // Own header not seen on the bus: transceiver/wiring or bus collision
  }

  // Step 4: Wait for slave response (6 bytes expected)
  uint8_t response[6];
  if (!readBytes(response, 6, RESPONSE_TIMEOUT)) {
    currentData.valid = false;
    return false;  // Timeout error
  }

  // Step 5: Validate checksum
  uint8_t calculatedChecksum = calculateChecksum(response, 5);
  if (response[5] != calculatedChecksum) {
    currentData.valid = false;
    return false;  // Checksum error
  }

  // Step 6: Parse response data
  parseResponse(response);
  currentData.valid = true;

  return true;
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
 * Frame format (6 bytes):
 * ┌─────────┬──────────┬──────────┬──────────┬──────────┬──────────┐
 * │ Byte 0  │ Byte 1   │ Byte 2   │ Byte 3   │ Byte 4   │ Byte 5   │
 * │ Buttons │ ADC0_MSB │ ADC0_LSB │ ADC1_MSB │ ADC1_LSB │ Checksum │
 * └─────────┴──────────┴──────────┴──────────┴──────────┴──────────┘
 *
 * Button byte: Bits 0-6 = Buttons 1-7 (1=pressed, 0=released)
 * ADC values: 12-bit (0-4095) split into MSB (8 bits) + LSB (4 bits)
 ******************************************************************************/
void parseResponse(uint8_t* data) {
  // Extract button states (7 bits)
  currentData.buttons = data[0] & 0x7F;

  // Reconstruct ADC0 (12-bit: MSB + LSB)
  currentData.pot1 = ((uint16_t)data[1] << 4) | ((data[2] >> 4) & 0x0F);

  // Reconstruct ADC1 (12-bit: MSB + LSB)
  currentData.pot2 = ((uint16_t)data[3] << 4) | ((data[4] >> 4) & 0x0F);
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
 * NOTE: This function uses Serial which conflicts with LIN on pins 0/1!
 * - Comment out this function for production (LIN-only operation)
 * - Uncomment for initial testing with USB connected
 ******************************************************************************/
void printSteeringData() {
  // Uncomment the following lines for Serial debugging:
  /*
  // Print button states
  Serial.print(F("BTN: "));
  for (int i = 0; i < 7; i++) {
    if (currentData.buttons & (1 << i)) {
      Serial.print(F("["));
      Serial.print(i + 1);
      Serial.print(F("]"));
    } else {
      Serial.print(F("[ ]"));
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
  Serial.println(F("%)"));
  */
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

// Check if specific button is pressed (1-7)
bool isButtonPressed(uint8_t buttonNum) {
  if (buttonNum < 1 || buttonNum > 7) return false;
  return (currentData.buttons & (1 << (buttonNum - 1))) != 0;
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

// Check if data is valid
bool isDataValid() {
  return currentData.valid;
}
