/*******************************************************************************
 * STEERING WHEEL CONTROLLER - RP2040 LIN BUS SLAVE
 *
 * Board: Waveshare RP2040-Tiny
 * Core: Arduino-Pico (Earle Philhower)
 * Power: RP2040 via USB, MCP2003 VBB = 12V lab supply
 *
 * DESCRIPTION:
 * LIN bus slave/responder that reads 7 buttons and 2 potentiometers, then
 * responds to master requests with current input states via LIN protocol.
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
 *          └─ min 676µs low
 *
 * Slave:                              |BTN|ADC0_H|ADC0_L|ADC1_H|ADC1_L|CHK|
 *                                     └─ Response frame (6 bytes)
 *
 * MCP2003 LIN TRANSCEIVER CONNECTIONS (see README, rev. 3.0):
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ MCP2003 Pin │ Function │ RP2040 Connection                          │
 * ├─────────────┼──────────┼────────────────────────────────────────────┤
 * │ Pin 1 (RXD) │ Receive  │ GPIO1 (UART0_RX) + 4.7kΩ pull-up to 3.3V   │
 * │ Pin 2 (CS)  │ Chip Sel │ GPIO2, driven HIGH by firmware, no pull-up │
 * │ Pin 3 (WAKE)│ Wake-up  │ Not connected                              │
 * │ Pin 4 (TXD) │ Transmit │ GPIO0 (UART0_TX)                           │
 * │ Pin 5 (VSS) │ Ground   │ GND (common with lab supply and Uno)       │
 * │ Pin 6 (LBUS)│ LIN Bus  │ To Arduino MCP2003 LBUS + 220pF to GND     │
 * │ Pin 7 (VBB) │ Battery  │ 12V lab supply (MCP2003 needs > 5.5V)      │
 * │ Pin 8 (VREN)│ VReg En  │ Output - not connected                     │
 * └─────────────┴──────────┴────────────────────────────────────────────┘
 *
 * INPUT HARDWARE:
 * - 7 buttons (GPIO6-12): Active LOW, internal pull-ups enabled in code
 * - 2x 10kΩ potentiometers:
 *   * POT1: Left→GND, Center→GPIO26 (ADC0), Right→3.3V, 100nF filter cap
 *   * POT2: Left→GND, Center→GPIO27 (ADC1), Right→3.3V, 100nF filter cap
 *   * Note: 10kΩ is ideal impedance for RP2040 ADC (better than 20kΩ)
 *
 * STATUS LED:
 * - GPIO14 (external LED + 220Ω): short flash per answered frame,
 *   1 Hz heartbeat while no master is polling.
 *   The RP2040-Tiny has no plain LED on GPIO25 (onboard is a WS2812 on GPIO16).
 *
 * POWER:
 * - RP2040 powered via USB (5V → 3.3V onboard regulator)
 * - MCP2003 VBB = 12V lab supply (the MCP2003 stays in POR below 5.5V)
 *
 * FRAME DETECTION:
 * The bus only carries master headers and our own responses. The response
 * echo is read back right after sending, so a SYNC (0x55) directly followed
 * by PID 0x3C is always a request. The break byte (0x00) is ignored.
 *
 * Author: Claude Code
 * Date: 2026-09-19
 * Version: 3.0 (echo handling, non-blocking LED, robust frame detection, 12V wiring)
 ******************************************************************************/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LIN_TX_PIN        0     // UART0 TX → MCP2003 Pin 4 (TXD)
#define LIN_RX_PIN        1     // UART0 RX ← MCP2003 Pin 1 (RXD)
#define LIN_CS_PIN        2     // Chip Select → MCP2003 Pin 2 (CS)
#define STATUS_LED_PIN    14    // External status LED

// Button pins (active LOW with internal pull-ups)
#define BUTTON_1_PIN      6
#define BUTTON_2_PIN      7
#define BUTTON_3_PIN      8
#define BUTTON_4_PIN      9
#define BUTTON_5_PIN      10
#define BUTTON_6_PIN      11
#define BUTTON_7_PIN      12

// Potentiometer pins (10kΩ, ideal for RP2040 ADC)
#define POT_1_PIN         26    // ADC0 (GPIO26)
#define POT_2_PIN         27    // ADC1 (GPIO27)

// ============================================================================
// LIN PROTOCOL CONSTANTS
// ============================================================================
#define LIN_BAUD_RATE     19200           // Standard LIN baud rate
#define LIN_FRAME_ID      0x3C            // Protected ID for input request
#define LIN_SYNC_BYTE     0x55            // Standard LIN sync byte
#define ECHO_TIMEOUT_US   2000            // Own response echo must be back within 2ms after flush

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
#define DEBOUNCE_DELAY    50              // Button debounce: 50ms
#define ADC_SAMPLES       4               // ADC averaging: 4 samples
#define LED_BLINK_TIME    20              // Status LED flash per answered frame
#define HEARTBEAT_AFTER   1000            // Heartbeat when no frame for this long

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
const uint8_t buttonPins[] = {BUTTON_1_PIN, BUTTON_2_PIN, BUTTON_3_PIN,
                               BUTTON_4_PIN, BUTTON_5_PIN, BUTTON_6_PIN, BUTTON_7_PIN};

// Button state tracking
uint8_t buttonStates = 0;                 // Packed button states (bit 0-6 = buttons 1-7)
uint8_t lastButtonReading[7] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTime[7] = {0, 0, 0, 0, 0, 0, 0};

// ADC values (12-bit: 0-4095)
uint16_t adc0Value = 0;
uint16_t adc1Value = 0;

// LIN frame detection
bool gotSync = false;
unsigned long lastFrameTime = 0;

/*******************************************************************************
 * SETUP - Initialize hardware and peripherals
 ******************************************************************************/
void setup() {
  // Initialize hardware Serial for LIN communication (UART0)
  Serial1.setTX(LIN_TX_PIN);
  Serial1.setRX(LIN_RX_PIN);
  Serial1.begin(LIN_BAUD_RATE, SERIAL_8N1);

  // Configure LIN CS pin - keep HIGH to enable MCP2003
  pinMode(LIN_CS_PIN, OUTPUT);
  digitalWrite(LIN_CS_PIN, HIGH);

  // Configure status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Configure button pins with internal pull-ups (active LOW)
  for (int i = 0; i < 7; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Configure ADC for 12-bit resolution (0-4095)
  analogReadResolution(12);
  pinMode(POT_1_PIN, INPUT);
  pinMode(POT_2_PIN, INPUT);

  // Startup LED sequence: 3 quick blinks
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }

  delay(200);  // Allow MCP2003 to stabilize
}

/*******************************************************************************
 * MAIN LOOP
 ******************************************************************************/
void loop() {
  // Continuously update input states
  updateButtons();
  updateADC();

  // Respond to master requests
  handleLINRequest();

  updateStatusLED();
}

/*******************************************************************************
 * STATUS LED - Flash per answered frame, heartbeat while idle (non-blocking)
 ******************************************************************************/
void updateStatusLED() {
  unsigned long now = millis();
  if (lastFrameTime != 0 && now - lastFrameTime < HEARTBEAT_AFTER) {
    digitalWrite(STATUS_LED_PIN, now - lastFrameTime < LED_BLINK_TIME ? HIGH : LOW);
  } else {
    digitalWrite(STATUS_LED_PIN, (now / 1000) % 2 ? HIGH : LOW);
  }
}

/*******************************************************************************
 * UPDATE BUTTONS - Read all 7 buttons with debouncing
 *
 * Button byte format:
 * Bit 7: Reserved (0)
 * Bit 6-0: Buttons 7-1 (1=pressed, 0=released)
 ******************************************************************************/
void updateButtons() {
  for (int i = 0; i < 7; i++) {
    uint8_t reading = digitalRead(buttonPins[i]);

    // Detect button state change
    if (reading != lastButtonReading[i]) {
      lastDebounceTime[i] = millis();
    }

    // Update button state after debounce period
    if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
      // Active LOW: pressed = LOW, so invert for logical representation
      if (reading == LOW) {
        buttonStates |= (1 << i);   // Set bit (button pressed)
      } else {
        buttonStates &= ~(1 << i);  // Clear bit (button released)
      }
    }

    lastButtonReading[i] = reading;
  }
}

/*******************************************************************************
 * UPDATE ADC - Read potentiometers with averaging
 *
 * 10kΩ potentiometers are ideal for RP2040 ADC:
 * - Low enough impedance for fast settling
 * - High enough to minimize current draw
 * - No special timing adjustments needed
 *
 * ADC averaging: 4 samples for stable readings with 100nF filter caps
 ******************************************************************************/
void updateADC() {
  uint32_t sum0 = 0;
  uint32_t sum1 = 0;

  // Take multiple samples and average
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum0 += analogRead(POT_1_PIN);
    sum1 += analogRead(POT_2_PIN);
    delayMicroseconds(100);  // Small delay between samples
  }

  adc0Value = sum0 / ADC_SAMPLES;
  adc1Value = sum1 / ADC_SAMPLES;
}

/*******************************************************************************
 * HANDLE LIN REQUEST - Find SYNC + PID in the byte stream and respond
 *
 * Processes every buffered byte, so nothing is lost while updateADC() runs.
 * The previous version flushed the RX buffer on a "break gap" measured when
 * new bytes had already arrived, which could throw away the SYNC byte.
 ******************************************************************************/
void handleLINRequest() {
  while (Serial1.available()) {
    uint8_t receivedByte = Serial1.read();

    if (gotSync && receivedByte == LIN_FRAME_ID) {
      gotSync = false;
      sendLINResponse();
      lastFrameTime = millis();
    } else {
      gotSync = (receivedByte == LIN_SYNC_BYTE);
    }
  }
}

/*******************************************************************************
 * SEND LIN RESPONSE - Transmit button states and ADC values
 *
 * Frame format (6 bytes):
 * ┌─────────┬──────────┬──────────┬──────────┬──────────┬──────────┐
 * │ Byte 0  │ Byte 1   │ Byte 2   │ Byte 3   │ Byte 4   │ Byte 5   │
 * │ Buttons │ ADC0_MSB │ ADC0_LSB │ ADC1_MSB │ ADC1_LSB │ Checksum │
 * └─────────┴──────────┴──────────┴──────────┴──────────┴──────────┘
 *
 * Button byte: Bits 0-6 = Buttons 1-7 (1=pressed)
 * ADC values: 12-bit (0-4095) split into MSB (8 bits) + LSB (4 bits)
 * Checksum: Simple modular sum of all data bytes % 256
 ******************************************************************************/
void sendLINResponse() {
  uint8_t dataFrame[6];

  // Byte 0: Button states (bits 0-6)
  dataFrame[0] = buttonStates & 0x7F;  // Mask to 7 bits

  // Bytes 1-2: ADC0 (12-bit → MSB + LSB)
  dataFrame[1] = (adc0Value >> 4) & 0xFF;      // Upper 8 bits
  dataFrame[2] = (adc0Value & 0x0F) << 4;      // Lower 4 bits in upper nibble

  // Bytes 3-4: ADC1 (12-bit → MSB + LSB)
  dataFrame[3] = (adc1Value >> 4) & 0xFF;      // Upper 8 bits
  dataFrame[4] = (adc1Value & 0x0F) << 4;      // Lower 4 bits in upper nibble

  // Byte 5: Checksum (modular sum)
  dataFrame[5] = calculateChecksum(dataFrame, 5);

  // Transmit frame
  Serial1.write(dataFrame, 6);
  Serial1.flush();  // Wait for transmission complete

  // The transceiver mirrors the bus on RXD: read our own 6 bytes back so they
  // are never mistaken for a new request (e.g. buttons 0x55 + ADC0_MSB 0x3C).
  uint8_t echoed = 0;
  unsigned long start = micros();
  while (echoed < 6 && micros() - start < ECHO_TIMEOUT_US) {
    if (Serial1.available()) {
      Serial1.read();
      echoed++;
    }
  }
}

/*******************************************************************************
 * CALCULATE CHECKSUM - Simple modular sum
 *
 * Checksum = (Sum of all data bytes) % 256
 * This is the classic LIN checksum method (non-inverted)
 ******************************************************************************/
uint8_t calculateChecksum(uint8_t* data, uint8_t length) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum & 0xFF;  // Keep lower 8 bits
}
