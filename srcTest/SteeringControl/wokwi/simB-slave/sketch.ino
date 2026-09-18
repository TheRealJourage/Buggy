/*******************************************************************************
 * STEERING WHEEL CONTROLLER - RP2040 LIN BUS SLAVE
 *
 * Board: Waveshare RP2040-Tiny
 * Core: Arduino-Pico (Earle Philhower)
 * Power: 5V Laboratory Supply (NOT 12V automotive)
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
 * MCP2003 LIN TRANSCEIVER CONNECTIONS (CORRECTED PINOUT):
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ MCP2003 Pin │ Function │ RP2040 Connection                          │
 * ├─────────────┼──────────┼────────────────────────────────────────────┤
 * │ Pin 1 (RXD) │ Receive  │ GPIO1 (UART0_RX) + 5kΩ pull-up to 3.3V    │
 * │ Pin 2 (CS)  │ Chip Sel │ GPIO2 + 5kΩ pull-up to 3.3V (keep HIGH)   │
 * │ Pin 3 (WAKE)│ Wake-up  │ Not connected (optional)                   │
 * │ Pin 4 (TXD) │ Transmit │ GPIO0 (UART0_TX)                           │
 * │ Pin 5 (VSS) │ Ground   │ GND                                        │
 * │ Pin 6 (LBUS)│ LIN Bus  │ To Arduino MCP2003 LBUS                    │
 * │ Pin 7 (VBB) │ Battery  │ 5V (laboratory power supply)               │
 * │ Pin 8 (VREN)│ VReg En  │ 5V or leave floating                       │
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
 * - GPIO25: Blinks 100ms when LIN frame is transmitted
 *
 * 5V LABORATORY POWER SETUP:
 * - RP2040 powered via USB (5V → 3.3V onboard regulator)
 * - MCP2003 VBB powered by 5V lab supply
 * - This setup is safe for prototyping
 * - Later upgrade to 12V: Add DC-DC converter (12V → 5V) before MCP2003 VBB
 *
 * Author: Claude Code
 * Date: 2026-02-12
 * Version: 2.0 (5V Lab Setup with 10kΩ Potentiometers)
 ******************************************************************************/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define LIN_TX_PIN        0     // UART0 TX → MCP2003 Pin 4 (TXD)
#define LIN_RX_PIN        1     // UART0 RX ← MCP2003 Pin 1 (RXD)
#define LIN_CS_PIN        2     // Chip Select → MCP2003 Pin 2 (CS)
#define STATUS_LED_PIN    25    // Onboard LED

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
#define BREAK_THRESHOLD   600             // Break detection: 600µs minimum (13 bits @ 19200 = 677µs)

// ============================================================================
// TIMING CONSTANTS
// ============================================================================
#define DEBOUNCE_DELAY    50              // Button debounce: 50ms
#define ADC_SAMPLES       4               // ADC averaging: 4 samples
#define LED_BLINK_TIME    100             // Status LED blink duration

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

// LIN communication timing
unsigned long lastRxTime = 0;
bool ledState = false;

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

  // Check for LIN break field and respond to master requests
  if (Serial1.available()) {
    handleLINRequest();
  }

  // Heartbeat LED: slow blink when idle
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);
    lastBlink = millis();
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
 * HANDLE LIN REQUEST - Detect break field and respond with data
 *
 * LIN Break Detection:
 * - Break field = 13+ bit times LOW (minimum 677µs at 19200 baud)
 * - Detected by measuring time since last received byte
 * - If gap > BREAK_THRESHOLD (600µs), it's a break
 ******************************************************************************/
void handleLINRequest() {
  unsigned long currentTime = micros();

  // Detect break field by checking for extended silence
  if (lastRxTime > 0 && (currentTime - lastRxTime) > BREAK_THRESHOLD) {
    // Break detected! Clear buffer and prepare for new frame
    while (Serial1.available()) Serial1.read();
    lastRxTime = 0;
  }

  // Read next byte
  if (Serial1.available()) {
    lastRxTime = micros();
    uint8_t receivedByte = Serial1.read();

    // Check for SYNC byte (0x55)
    if (receivedByte == LIN_SYNC_BYTE) {
      // Wait for PID (Protected ID)
      unsigned long timeout = millis();
      while (!Serial1.available() && (millis() - timeout < 10));

      if (Serial1.available()) {
        uint8_t pid = Serial1.read();

        // Check if this frame is for us (ID 0x3C)
        if (pid == LIN_FRAME_ID) {
          // Send response frame
          sendLINResponse();

          // Blink LED to indicate transmission
          digitalWrite(STATUS_LED_PIN, HIGH);
          delay(LED_BLINK_TIME);
          digitalWrite(STATUS_LED_PIN, LOW);
        }
      }
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
