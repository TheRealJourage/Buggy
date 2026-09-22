// Raw readout of the light stalk: common wire on GND, each signal line
// on its own ADC pin with a 220 R pull-up to 3.3 V.
// Prints raw value, voltage and the stalk resistance R = Rpu * V / (3.3 - V).
#include <Arduino.h>
#include "config.h"

static const float VREF = 3.3f;
static const float RPU  = 220.0f;

static int readAvg(uint8_t pin) {
    long sum = 0;
    for (uint8_t i = 0; i < 16; i++) sum += analogRead(pin);
    return sum / 16;
}

static void printChannel(const char* name, int raw) {
    float v = raw * VREF / 4095.0f;
    Serial.print(name); Serial.print('=');
    Serial.print(raw); Serial.print(" (");
    Serial.print(v, 2); Serial.print(" V, ");
    if (v > VREF - 0.02f) Serial.print("open");
    else { Serial.print((int)(RPU * v / (VREF - v))); Serial.print(" Ohm"); }
    Serial.print(')');
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    delay(500);
    Serial.println(F("ADC test: A0 = light stalk white-black, A2 = light stalk yellow-black"));
}

void loop() {
    printChannel("A0", readAvg(PIN_BLINKER));
    Serial.print("   ");
    printChannel("A2", readAvg(PIN_HIGHBEAM));
    Serial.println();
    delay(250);
}
