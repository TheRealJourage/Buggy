#pragma once
#include <stdint.h>

// Steering wheel buttons as built, each one pressed and checked with
// rp2040_io_test on 2026-10-04 and 2026-10-05. Wiring (connector, pin, wire
// colour) is in Obsidian Buggy/SteeringControl/Lenkrad-Verdrahtung.md.
//
// The enum value is the bit in the LIN button byte. Slave and master both use
// this header, so a remap only ever happens here.
enum SteeringButton : uint8_t {
  BTN_RED    = 0,  // GP6
  BTN_PLUS   = 1,  // GP7
  BTN_HORN   = 2,  // GP8
  BTN_BLACK  = 3,  // GP9
  BTN_BLUE   = 4,  // GP10
  BTN_YELLOW = 5,  // GP11
  BTN_MINUS  = 6,  // GP12
  BTN_GREEN  = 7,  // GP5
};

#define STEERING_BUTTON_COUNT 8

// RP2040 GPIO per bit, active LOW with internal pull-up
const uint8_t STEERING_BUTTON_GPIO[STEERING_BUTTON_COUNT] = {6, 7, 8, 9, 10, 11, 12, 5};

// Debug labels. The Uno master's debug text also goes onto the LIN bus, so
// no label may contain 'U' (0x55 = SYNC).
const char* const STEERING_BUTTON_NAME[STEERING_BUTTON_COUNT] = {
  "rot", "plus", "hupe", "schwarz", "blau", "gelb", "minus", "gruen"
};
