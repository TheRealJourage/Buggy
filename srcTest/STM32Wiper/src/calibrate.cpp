// ═══════════════════════════════════════════════════════════════════════════════
// Corsa C Stalk Calibration Firmware
// Build:   pio run -e calibrate --target upload
// Purpose: Capture ADC values for all stalk positions, compute midpoint
//          thresholds + position remap table, write to emulated EEPROM.
//          Flash main firmware afterwards for normal operation.
// ═══════════════════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "stalk.h"

// ─── Position labels ──────────────────────────────────────────────────────────
static const char* BL_LABEL[3] = {
    "OFF   (center, stalk not touched)",
    "RIGHT (push UP — tap or hold, both detected by timing)",
    "LEFT  (push DOWN — tap or hold, both detected by timing)",
};
static const char* WP_LABEL[5] = {
    "OFF",
    "INTERVAL",
    "SPEED-1",
    "SPEED-2",
    "WASH-F   (push toward screen, momentary)",
};
static const char* RR_LABEL[3] = {
    "OFF",
    "REAR-WIPE",
    "REAR-WASH (push toward screen, momentary)",
};

// ─── Per-stalk calibration state ─────────────────────────────────────────────
struct StalkCal {
    const char*  name;
    uint8_t      pin;
    uint8_t      numPos;      // 5 for blinker/wiper, 3 for rear
    const char** labels;
    int          cap[5];      // captured ADC per position index
    bool         got[5];
};

static StalkCal sec[3] = {
    { "BLINKER A0", PIN_BLINKER, 3, BL_LABEL, {0,0,0,0,0}, {false,false,false,false,false} },
    { "WIPER   A1", PIN_WIPER,   5, WP_LABEL, {0,0,0,0,0}, {false,false,false,false,false} },
    { "REAR    A3", PIN_REAR,    3, RR_LABEL, {0,0,0,0,0}, {false,false,false,false,false} },
};
static uint8_t activeSec = 0;

static EEData ee;

// ─── Helpers ─────────────────────────────────────────────────────────────────
static void relaysOff() {
    const uint8_t pins[] = {
        RELAY_BLINK_L, RELAY_BLINK_R, RELAY_WIPER1,
        RELAY_WIPER2,  RELAY_WASH_F,  RELAY_WIPER_R, RELAY_WASH_R
    };
    for (uint8_t p : pins) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
}

static int captureADC(uint8_t pin) {
    long s = 0;
    for (uint8_t i = 0; i < 64; i++) { s += analogRead(pin); delay(3); }
    return (int)(s / 64);
}

static void printBar(int val) {
    int f = (long)val * 30 / 4095;
    for (int i = 0; i < 30; i++) Serial.print(i < f ? '=' : ' ');
}

// ─── Display ─────────────────────────────────────────────────────────────────
static void printStatus() {
    StalkCal& s = sec[activeSec];
    Serial.println(F("\r\n"));
    Serial.println(F("  ╔══════════════════════════════════════════════════════╗"));
    Serial.print  (F("  ║  CALIBRATION  —  "));
    Serial.print  (s.name);
    Serial.println(F("                   ║"));
    Serial.println(F("  ╚══════════════════════════════════════════════════════╝"));
    Serial.println(F("  Switch:  [b] Blinker   [w] Wiper   [r] Rear"));
    Serial.println(F("  Action:  [0-4] capture position   [x] reset   [s] save"));
    Serial.println(F("  ──────────────────────────────────────────────────────────"));

    for (uint8_t i = 0; i < s.numPos; i++) {
        Serial.print(F("  ["));
        Serial.print(i);
        Serial.print(F("] "));
        if (s.got[i]) {
            int v = s.cap[i];
            char buf[5];
            snprintf(buf, sizeof(buf), "%4d", v);
            Serial.print(buf);
            Serial.print(F("  ["));
            printBar(v);
            Serial.print(F("]  "));
        } else {
            Serial.print(F("----  [                              ]  "));
        }
        Serial.println(s.labels[i]);
    }

    int n = 0;
    for (uint8_t i = 0; i < s.numPos; i++) if (s.got[i]) n++;
    Serial.println(F("  ──────────────────────────────────────────────────────────"));
    Serial.print(F("  Captured: ")); Serial.print(n);
    Serial.print(F(" / ")); Serial.println(s.numPos);
    Serial.println();
}

// ─── Threshold + remap computation ───────────────────────────────────────────
// Sorts the numActive active positions (cap[1..numActive]) by ADC descending,
// then computes midpoint thresholds and builds the remap table so that
// classifyAdc() output index → correct enum value regardless of cascade order.
static void computeSection(StalkCal& s, uint8_t numActive,
                            uint16_t* thresh, uint8_t* remap) {
    // Copy active positions into sortable arrays
    int     sortVal[4];
    uint8_t sortPos[4];   // original position index = enum value
    for (uint8_t i = 0; i < numActive; i++) {
        sortVal[i] = s.got[i + 1] ? s.cap[i + 1] : 0;
        sortPos[i] = i + 1;
    }

    // Insertion sort descending by ADC value
    for (uint8_t i = 1; i < numActive; i++) {
        int     kv = sortVal[i];
        uint8_t ki = sortPos[i];
        int j = (int)i - 1;
        while (j >= 0 && sortVal[j] < kv) {
            sortVal[j + 1] = sortVal[j];
            sortPos[j + 1] = sortPos[j];
            j--;
        }
        sortVal[j + 1] = kv;
        sortPos[j + 1] = ki;
    }

    // Thresholds: midpoints between adjacent sorted ADC values
    int offADC = s.got[0] ? s.cap[0] : 4095;
    thresh[0] = (uint16_t)((offADC + sortVal[0]) / 2);
    for (uint8_t i = 0; i < numActive - 1; i++)
        thresh[i + 1] = (uint16_t)((sortVal[i] + sortVal[i + 1]) / 2);

    // Remap: classIdx 0 = OFF, classIdx 1..numActive = sorted positions
    remap[0] = 0;
    for (uint8_t i = 0; i < numActive; i++)
        remap[i + 1] = sortPos[i];
}

// ─── Save to EEPROM ───────────────────────────────────────────────────────────
static void saveAll() {
    Serial.println(F("\n  Computing & saving..."));

    // Blinker (3 positions → 2 thresholds: OFF + RIGHT + LEFT)
    if (sec[0].got[0]) {
        computeSection(sec[0], 2, ee.blinkerThresh, ee.blinkerMap);
    } else {
        memcpy(ee.blinkerThresh, DEFAULT_BLINKER_THRESH, sizeof(ee.blinkerThresh));
        memcpy(ee.blinkerMap,    DEFAULT_BLINKER_MAP,    sizeof(ee.blinkerMap));
    }

    // Wiper (5 positions → 4 thresholds)
    if (sec[1].got[0]) {
        computeSection(sec[1], 4, ee.wiperThresh, ee.wiperMap);
    } else {
        memcpy(ee.wiperThresh, DEFAULT_WIPER_THRESH, sizeof(ee.wiperThresh));
        memcpy(ee.wiperMap,    DEFAULT_WIPER_MAP,    sizeof(ee.wiperMap));
    }

    // Rear (3 positions → 2 thresholds)
    if (sec[2].got[0]) {
        computeSection(sec[2], 2, ee.rearThresh, ee.rearMap);
    } else {
        memcpy(ee.rearThresh, DEFAULT_REAR_THRESH, sizeof(ee.rearThresh));
        memcpy(ee.rearMap,    DEFAULT_REAR_MAP,    sizeof(ee.rearMap));
    }

    ee.magic = EEPROM_MAGIC;
    EEPROM.put(EEPROM_ADDR, ee);

    Serial.println(F("  ── Blinker ──────────────────────────────────────────"));
    Serial.print(F("  Thresh: "));
    for (int i = 0; i < 2; i++) { Serial.print(ee.blinkerThresh[i]); Serial.print(' '); }
    Serial.println();
    Serial.print(F("  Remap:  classIdx→pos  "));
    for (int i = 0; i < 3; i++) {
        Serial.print(i); Serial.print("→"); Serial.print(ee.blinkerMap[i]); Serial.print("  ");
    }
    Serial.println(F("\n"));
    Serial.println(F("  ✓ Saved to EEPROM."));
    Serial.println(F("  Now flash the main firmware:"));
    Serial.println(F("    pio run -e nucleo_f446re --target upload"));
    Serial.println();
    while (true) delay(1000);
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    analogReadResolution(12);
    relaysOff();
    EEPROM.begin();

    // Load existing EEPROM so uncalibrated stalks keep their values
    EEPROM.get(EEPROM_ADDR, ee);
    if (ee.magic != EEPROM_MAGIC) {
        memcpy(ee.blinkerThresh, DEFAULT_BLINKER_THRESH, sizeof(ee.blinkerThresh));
        memcpy(ee.wiperThresh,   DEFAULT_WIPER_THRESH,   sizeof(ee.wiperThresh));
        memcpy(ee.rearThresh,    DEFAULT_REAR_THRESH,    sizeof(ee.rearThresh));
        memcpy(ee.blinkerMap,    DEFAULT_BLINKER_MAP,    sizeof(ee.blinkerMap));
        memcpy(ee.wiperMap,      DEFAULT_WIPER_MAP,      sizeof(ee.wiperMap));
        memcpy(ee.rearMap,       DEFAULT_REAR_MAP,       sizeof(ee.rearMap));
    }

    printStatus();
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
    StalkCal& s = sec[activeSec];

    // Live ADC — refresh in place with \r
    static uint32_t lastLive  = 0;
    static int      stableRef = -999;
    static uint32_t stableT   = 0;
    static bool     stableMsg = false;

    if (millis() - lastLive > 80) {
        lastLive = millis();
        int raw = readAdcSmoothed(s.pin, 16);

        // Stability detection: if value holds ±20 for 900 ms, nudge user
        if (abs(raw - stableRef) > 20) {
            stableRef = raw; stableT = millis(); stableMsg = false;
        } else if (!stableMsg && millis() - stableT > 900) {
            stableMsg = true;
            Serial.println();
            Serial.print(F("  >> Stable at "));
            Serial.print(raw);
            Serial.println(F(" — press [0-4] to assign this position"));
        }

        const char* pinName =
            (s.pin == PIN_BLINKER) ? "A0" :
            (s.pin == PIN_WIPER)   ? "A1" : "A3";

        Serial.print(F("\r  LIVE "));
        Serial.print(pinName);
        Serial.print('=');
        char buf[5];
        snprintf(buf, sizeof(buf), "%4d", raw);
        Serial.print(buf);
        Serial.print(F("  ["));
        printBar(raw);
        Serial.print(F("]  "));
    }

    if (!Serial.available()) return;
    char c = Serial.read();
    while (Serial.available()) Serial.read();   // flush
    stableMsg = false;

    if (c >= '0' && c <= '4') {
        uint8_t idx = (uint8_t)(c - '0');
        if (idx >= s.numPos) {
            Serial.println(F("\n  [!] Index out of range for this stalk."));
            return;
        }
        Serial.print(F("\n  Capturing ["));
        Serial.print(idx);
        Serial.print(F("] "));
        Serial.print(s.labels[idx]);
        Serial.print(F(" ..."));
        s.cap[idx] = captureADC(s.pin);
        s.got[idx] = true;
        Serial.print(F("  ADC = "));
        Serial.println(s.cap[idx]);
        printStatus();

    } else if (c == 'b' || c == 'B') {
        activeSec = 0; printStatus();
    } else if (c == 'w' || c == 'W') {
        activeSec = 1; printStatus();
    } else if (c == 'r' || c == 'R') {
        activeSec = 2; printStatus();

    } else if (c == 'x' || c == 'X') {
        for (uint8_t i = 0; i < s.numPos; i++) s.got[i] = false;
        Serial.println(F("\n  Section reset."));
        printStatus();

    } else if (c == 's' || c == 'S') {
        bool anyDone = false;
        for (uint8_t si = 0; si < 3; si++)
            for (uint8_t i = 0; i < sec[si].numPos; i++)
                if (sec[si].got[i]) { anyDone = true; break; }
        if (!anyDone) {
            Serial.println(F("\n  [!] Nothing captured yet — nothing to save."));
        } else {
            saveAll();
        }

    } else if (c == '?') {
        Serial.println(F("\n  HELP"));
        Serial.println(F("  ────────────────────────────────────────────────────"));
        Serial.println(F("  1. Move stalk to the described position."));
        Serial.println(F("  2. Wait for STABLE message (or just press the key)."));
        Serial.println(F("  3. Press the matching number key — 64 samples averaged."));
        Serial.println(F("  4. You can recapture any position by pressing its key again."));
        Serial.println(F("  5. Switch stalks with [b/w/r].  [s] saves and exits."));
        Serial.println(F("  6. Uncalibrated stalks keep factory defaults."));
        Serial.println(F("  Note: OFF must be the highest ADC value (stalk not touched)."));
        Serial.println();
    }
}
