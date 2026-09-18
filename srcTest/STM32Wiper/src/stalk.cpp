#include "stalk.h"

int readAdcSmoothed(uint8_t pin, uint8_t samples) {
    long sum = 0;
    for (uint8_t i = 0; i < samples; i++) sum += analogRead(pin);
    return (int)(sum / samples);
}

uint8_t classifyAdc(int adcVal, const uint16_t* thresholds, uint8_t numThresholds) {
    for (uint8_t i = 0; i < numThresholds; i++)
        if (adcVal > (int)thresholds[i]) return i;
    return numThresholds;
}

BlinkerPos classifyBlinker(int adcVal, const uint16_t thresholds[2], const uint8_t remap[3]) {
    return (BlinkerPos)remap[classifyAdc(adcVal, thresholds, 2)];
}

WiperPos classifyWiper(int adcVal, const uint16_t thresholds[4], const uint8_t remap[5]) {
    return (WiperPos)remap[classifyAdc(adcVal, thresholds, 4)];
}

RearPos classifyRear(int adcVal, const uint16_t thresholds[2], const uint8_t remap[3]) {
    return (RearPos)remap[classifyAdc(adcVal, thresholds, 2)];
}
