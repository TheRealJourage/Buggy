#pragma once
#include <Arduino.h>

// ─── ADC Inputs ───────────────────────────────────────────────────────────────
static const uint8_t PIN_BLINKER   = A0;   // PA0 – blinker SIG_A
static const uint8_t PIN_HIGHBEAM  = A2;   // PA4 – high-beam SIG_B (stub)
static const uint8_t PIN_WIPER     = A1;   // PA1 – front wiper stalk
static const uint8_t PIN_REAR      = A3;   // PB0 – rear wiper stalk

// ─── Calibration Button ───────────────────────────────────────────────────────
static const uint8_t PIN_CALIB_BTN = D7;   // PA8 – 3.3V → button → D7

// ─── Relay Outputs (active-LOW) ───────────────────────────────────────────────
static const uint8_t RELAY_BLINK_L  = D4;  // PB5  – IN1 left blinker
static const uint8_t RELAY_BLINK_R  = D5;  // PB4  – IN2 right blinker
static const uint8_t RELAY_WIPER1   = D6;  // PB10 – IN3 wiper speed 1
static const uint8_t RELAY_WIPER2   = D3;  // PB3  – IN4 wiper speed 2
static const uint8_t RELAY_WASH_F   = D8;  // PA9  – IN5 front washer
static const uint8_t RELAY_WIPER_R  = D9;  // PC7  – IN6 rear wiper
static const uint8_t RELAY_WASH_R   = D10; // PB6  – IN7 rear washer

// ─── Timing Constants (ms) ────────────────────────────────────────────────────
// Blinker: stalk held < TAP_TIMEOUT_MS then released = lane change (3 flashes).
// Stalk held > TAP_TIMEOUT_MS = latched continuous blink.
static const uint32_t TAP_TIMEOUT_MS     = 400;
static const uint32_t LANE_HALF_MS       = 400;  // flash half-period for lane-change
static const uint8_t  LANE_FLASHES       = 3;

static const uint32_t WIPER_INT_ON_MS    = 800;
static const uint32_t WIPER_INT_OFF_MS   = 4000;
static const uint32_t WASH_EXTRA_WIPE_MS = 800;
static const uint32_t DEBUG_PRINT_MS     = 250;

// ─── EEPROM ───────────────────────────────────────────────────────────────────
// Bump magic whenever EEData layout changes to force re-calibration.
static const uint16_t EEPROM_MAGIC = 0xAB14;
static const int      EEPROM_ADDR  = 0;

// ─── EEPROM Data Layout ───────────────────────────────────────────────────────
// Blinker uses 3 positions (OFF / RIGHT / LEFT) detected via SIG_A alone.
// LANE-CHANGE vs LATCH is determined by hold duration (TAP_TIMEOUT_MS).
//
// For wiper and rear, full multi-position cascade is retained.
//
// *Map arrays: classIdx(sorted high→low ADC) → enum value.
//  This handles any cascade order the stalk produces.
struct EEData {
    uint16_t magic;
    uint16_t blinkerThresh[2];   // 2 thresholds for 3 blinker positions
    uint8_t  blinkerMap[3];      // classIdx → BlinkerPos
    uint8_t  _pad0;
    uint16_t wiperThresh[4];     // 4 thresholds for 5 wiper positions
    uint8_t  wiperMap[5];        // classIdx → WiperPos
    uint8_t  _pad1[3];
    uint16_t rearThresh[2];      // 2 thresholds for 3 rear positions
    uint8_t  rearMap[3];         // classIdx → RearPos
    uint8_t  _pad2;
};
// Total: 2+4+3+1 + 8+5+3 + 4+3+1 = 34 bytes

// ─── Default ADC Thresholds ───────────────────────────────────────────────────
// Blinker: measured OFF≈290, LEFT≈152, RIGHT≈65 → midpoints:
static const uint16_t DEFAULT_BLINKER_THRESH[2] = { 221, 108 };
// Natural order high→low: OFF(290) > LEFT(152) > RIGHT(65)

static const uint16_t DEFAULT_WIPER_THRESH[4]   = { 2290, 375, 177, 72 };
static const uint16_t DEFAULT_REAR_THRESH[2]    = { 2290, 177 };

// ─── Default Position Maps ────────────────────────────────────────────────────
// Blinker natural order: classIdx 0=OFF(highest), 1=LEFT, 2=RIGHT
// BlinkerPos enum:       OFF=0, RIGHT=1, LEFT=2
// So map: {0, 2, 1}  (swap RIGHT and LEFT to match enum)
static const uint8_t DEFAULT_BLINKER_MAP[3] = { 0, 2, 1 };
static const uint8_t DEFAULT_WIPER_MAP[5]   = { 0, 1, 2, 3, 4 };
static const uint8_t DEFAULT_REAR_MAP[3]    = { 0, 1, 2 };
