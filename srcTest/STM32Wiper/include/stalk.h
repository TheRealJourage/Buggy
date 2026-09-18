#pragma once
#include <Arduino.h>

// ─── Blinker positions (3 states — direction only) ───────────────────────────
// LANE-CHANGE vs LATCH is detected by hold duration in main.cpp, not by ADC.
enum class BlinkerPos : uint8_t {
    OFF   = 0,
    RIGHT = 1,
    LEFT  = 2,
};

// ─── Wiper positions ─────────────────────────────────────────────────────────
enum class WiperPos : uint8_t {
    OFF      = 0,
    INTERVAL = 1,
    SPEED1   = 2,
    SPEED2   = 3,
    WASH_F   = 4,
};

// ─── Rear positions ───────────────────────────────────────────────────────────
enum class RearPos : uint8_t {
    OFF    = 0,
    WIPE_R = 1,
    WASH_R = 2,
};

// ─── Functions ────────────────────────────────────────────────────────────────

// Core classifier: returns bucket index 0..numThresholds.
// thresholds[] must be sorted descending, length = numThresholds.
uint8_t classifyAdc(int adcVal, const uint16_t* thresholds, uint8_t numThresholds);

// Convenience wrappers — apply classifyAdc then remap to enum value.
BlinkerPos classifyBlinker(int adcVal, const uint16_t thresholds[2], const uint8_t remap[3]);
WiperPos   classifyWiper  (int adcVal, const uint16_t thresholds[4], const uint8_t remap[5]);
RearPos    classifyRear   (int adcVal, const uint16_t thresholds[2], const uint8_t remap[3]);

// Averaged ADC read to suppress noise.
int readAdcSmoothed(uint8_t pin, uint8_t samples = 8);
