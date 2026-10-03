/*******************************************************************************
 * IO TEST - RP2040-TINY
 *
 * Reads all steering-wheel inputs directly, no LIN needed. Prints 5x per
 * second over USB serial (115200):
 *
 *   BTN 1234567 H | POT1 POT2 | HALL1 HALL2   (raw 12-bit ADC, 0..4095)
 *
 * Buttons: '#' = pressed (pin pulled to GND), '-' = released.
 *
 * Send 's' for a short test: every pin with pull-up (LOW = path to GND), with
 * pull-down (HIGH = path to 3V3), and pin-to-pin bridges (one pin driven LOW,
 * all others read with pull-up). GP0-2 go to the MCP2003, GP1 has a 5.1k
 * pull-up, so HIGH on GP1 in the pull-down pass is expected.
 ******************************************************************************/

const uint8_t BUTTON_PINS[] = {6, 7, 8, 9, 10, 11, 12};
const uint8_t HORN_PIN = 5;
const uint8_t ADC_PINS[] = {26, 27, 28, 29};
const uint8_t ADC_SAMPLES = 8;

const uint8_t SHORT_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 26, 27, 28, 29};
const uint8_t N_SHORT = sizeof(SHORT_PINS);

void setAllShortPins(PinMode mode) {
  for (uint8_t p : SHORT_PINS) pinMode(p, mode);
  delay(20);
}

void shortTest() {
  Serial.println("=== SHORT TEST ===");
  setAllShortPins(INPUT_PULLUP);
  Serial.print("LOW with pull-up (path to GND):  ");
  for (uint8_t p : SHORT_PINS) if (!digitalRead(p)) Serial.printf("GP%u ", p);
  Serial.println();

  setAllShortPins(INPUT_PULLDOWN);
  Serial.print("HIGH with pull-down (path to 3V3): ");
  for (uint8_t p : SHORT_PINS) if (digitalRead(p)) Serial.printf("GP%u ", p);
  Serial.println();

  Serial.print("Bridges: ");
  uint8_t found = 0;
  setAllShortPins(INPUT_PULLUP);
  for (uint8_t i = 0; i < N_SHORT; i++) {
    pinMode(SHORT_PINS[i], OUTPUT);
    digitalWrite(SHORT_PINS[i], LOW);
    delayMicroseconds(200);
    for (uint8_t j = i + 1; j < N_SHORT; j++) {
      if (!digitalRead(SHORT_PINS[j])) {
        Serial.printf("GP%u-GP%u ", SHORT_PINS[i], SHORT_PINS[j]);
        found++;
      }
    }
    pinMode(SHORT_PINS[i], INPUT_PULLUP);
    delay(2);
  }
  Serial.println(found ? "" : "none");
  Serial.println("=== END ===");

  setAllShortPins(INPUT);
  for (uint8_t pin : BUTTON_PINS) pinMode(pin, INPUT_PULLUP);
  pinMode(HORN_PIN, INPUT_PULLUP);
}

uint16_t readAdc(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) sum += analogRead(pin);
  return sum / ADC_SAMPLES;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t pin : BUTTON_PINS) pinMode(pin, INPUT_PULLUP);
  pinMode(HORN_PIN, INPUT_PULLUP);
  analogReadResolution(12);
}

void loop() {
  if (Serial.available() && Serial.read() == 's') shortTest();

  char btn[8];
  for (uint8_t i = 0; i < 7; i++) btn[i] = digitalRead(BUTTON_PINS[i]) ? '-' : '#';
  btn[7] = 0;

  Serial.printf("BTN %s %c | POT %4u %4u | HALL %4u %4u\n", btn,
                digitalRead(HORN_PIN) ? '-' : 'H',
                readAdc(26), readAdc(27), readAdc(28), readAdc(29));
  delay(200);
}
