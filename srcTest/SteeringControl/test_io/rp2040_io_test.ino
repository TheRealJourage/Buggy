/*******************************************************************************
 * IO TEST - RP2040-TINY
 *
 * Reads all steering-wheel inputs directly, no LIN needed. Prints 5x per
 * second over USB serial (115200):
 *
 *   BTN 1234567 | POT1 POT2 | HALL1 HALL2   (raw 12-bit ADC, 0..4095)
 *
 * Buttons: '#' = pressed (pin pulled to GND), '-' = released.
 ******************************************************************************/

const uint8_t BUTTON_PINS[] = {6, 7, 8, 9, 10, 11, 12};
const uint8_t ADC_PINS[] = {26, 27, 28, 29};
const uint8_t ADC_SAMPLES = 8;

uint16_t readAdc(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) sum += analogRead(pin);
  return sum / ADC_SAMPLES;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t pin : BUTTON_PINS) pinMode(pin, INPUT_PULLUP);
  analogReadResolution(12);
}

void loop() {
  char btn[8];
  for (uint8_t i = 0; i < 7; i++) btn[i] = digitalRead(BUTTON_PINS[i]) ? '-' : '#';
  btn[7] = 0;

  Serial.printf("BTN %s | POT %4u %4u | HALL %4u %4u\n", btn,
                readAdc(26), readAdc(27), readAdc(28), readAdc(29));
  delay(200);
}
