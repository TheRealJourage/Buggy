// Readout of the Simube paddle module (one Hall clutch paddle + one shift
// button per side). Hall sensor: red = 3.3 V (never 5 V), black = GND,
// blue = signal. Button: green to the input, black = GND (internal pull-up).
// Prints raw value, voltage and min/max seen per side; send 'r' to reset min/max.
#include <Arduino.h>

static const uint8_t PIN_HALL_L = A4;   // PC1
static const uint8_t PIN_HALL_R = A5;   // PC0
static const uint8_t PIN_BTN_L  = D2;   // PA10
static const uint8_t PIN_BTN_R  = D11;  // PA7

static const float VREF = 3.3f;

struct Side {
    const char* name;
    uint8_t hallPin;
    uint8_t btnPin;
    int minRaw;
    int maxRaw;
};

static Side sides[] = {
    { "L", PIN_HALL_L, PIN_BTN_L, 4095, 0 },
    { "R", PIN_HALL_R, PIN_BTN_R, 4095, 0 },
};

static int readAvg(uint8_t pin) {
    long sum = 0;
    for (uint8_t i = 0; i < 16; i++) sum += analogRead(pin);
    return sum / 16;
}

static void printSide(Side& s) {
    int raw = readAvg(s.hallPin);
    if (raw < s.minRaw) s.minRaw = raw;
    if (raw > s.maxRaw) s.maxRaw = raw;
    Serial.print(s.name); Serial.print(": hall=");
    Serial.print(raw); Serial.print(" (");
    Serial.print(raw * VREF / 4095.0f, 2); Serial.print(" V) min=");
    Serial.print(s.minRaw); Serial.print(" max=");
    Serial.print(s.maxRaw); Serial.print(" btn=");
    Serial.print(digitalRead(s.btnPin) == LOW ? "PRESSED" : "-");
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    for (Side& s : sides) pinMode(s.btnPin, INPUT_PULLUP);
    delay(500);
    Serial.println(F("Hall test: L = A4 + D2, R = A5 + D11, red on 3V3. 'r' resets min/max"));
}

void loop() {
    if (Serial.available() && Serial.read() == 'r') {
        for (Side& s : sides) { s.minRaw = 4095; s.maxRaw = 0; }
    }
    printSide(sides[0]);
    Serial.print("   |   ");
    printSide(sides[1]);
    Serial.println();
    delay(100);
}
