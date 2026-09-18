#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "stalk.h"

// EEData defined in config.h
static EEData cfg;

// ─── Blinker FSM ──────────────────────────────────────────────────────────────
// Stalk gives only 3 ADC states: OFF / RIGHT / LEFT.
// LANE-CHANGE vs LATCH is determined by how long the stalk is held:
//   < TAP_TIMEOUT_MS then released → 3 flashes (lane change)
//   ≥ TAP_TIMEOUT_MS still held    → continuous blink (latched)
enum class BlinkerFSM : uint8_t {
    OFF,
    PEND_R,    // stalk right, waiting for tap vs hold decision
    LANE_R,    // 3-flash right in progress (MCU controls relay)
    LATCH_R,   // continuous right (relay held, Hella flasher drives timing)
    PEND_L,
    LANE_L,
    LATCH_L,
};
static BlinkerFSM blinkerFSM    = BlinkerFSM::OFF;
static uint32_t   pendTimer     = 0;   // when PEND_x started
static uint32_t   laneTimer     = 0;   // half-cycle timer for lane-change flashing
static uint8_t    laneHalfCycle = 0;   // counts half-cycles (2 per flash)
static bool       laneRelayOn   = false;

// ─── Front wiper FSM ─────────────────────────────────────────────────────────
enum class FrontWiperFSM : uint8_t {
    OFF, INTERVAL_ON, INTERVAL_OFF, SPD1, SPD2, WASH_ACTIVE, WASH_EXTRA
};
static FrontWiperFSM frontFSM   = FrontWiperFSM::OFF;
static uint32_t      frontTimer = 0;

// ─── Rear wiper FSM ──────────────────────────────────────────────────────────
enum class RearFSM : uint8_t { OFF, WIPE, WASH_ACTIVE, WASH_EXTRA };
static RearFSM  rearFSM  = RearFSM::OFF;
static uint32_t rearTimer = 0;

static uint32_t lastDebug = 0;

// ─── Relay helpers ────────────────────────────────────────────────────────────
static inline void relayOn (uint8_t pin) { digitalWrite(pin, LOW);  }
static inline void relayOff(uint8_t pin) { digitalWrite(pin, HIGH); }

static void allRelaysOff() {
    relayOff(RELAY_BLINK_L); relayOff(RELAY_BLINK_R);
    relayOff(RELAY_WIPER1);  relayOff(RELAY_WIPER2);
    relayOff(RELAY_WASH_F);  relayOff(RELAY_WIPER_R); relayOff(RELAY_WASH_R);
}

// ─── EEPROM helpers ───────────────────────────────────────────────────────────
static void loadDefaults() {
    cfg.magic = EEPROM_MAGIC;
    memcpy(cfg.blinkerThresh, DEFAULT_BLINKER_THRESH, sizeof(cfg.blinkerThresh));
    memcpy(cfg.wiperThresh,   DEFAULT_WIPER_THRESH,   sizeof(cfg.wiperThresh));
    memcpy(cfg.rearThresh,    DEFAULT_REAR_THRESH,    sizeof(cfg.rearThresh));
    memcpy(cfg.blinkerMap,    DEFAULT_BLINKER_MAP,    sizeof(cfg.blinkerMap));
    memcpy(cfg.wiperMap,      DEFAULT_WIPER_MAP,      sizeof(cfg.wiperMap));
    memcpy(cfg.rearMap,       DEFAULT_REAR_MAP,       sizeof(cfg.rearMap));
}

static bool loadConfig() {
    EEPROM.get(EEPROM_ADDR, cfg);
    return (cfg.magic == EEPROM_MAGIC);
}

// ─── Inline lane-change flash helper ─────────────────────────────────────────
static void startLane(uint8_t relayPin) {
    laneHalfCycle = 0;
    laneRelayOn   = true;
    laneTimer     = millis();
    relayOn(relayPin);
}

static bool tickLane(uint8_t relayPin) {
    // Returns true when all flashes are done.
    if (millis() - laneTimer < LANE_HALF_MS) return false;
    laneTimer   = millis();
    laneRelayOn = !laneRelayOn;
    if (laneRelayOn) {
        relayOn(relayPin);
    } else {
        relayOff(relayPin);
        if (++laneHalfCycle >= LANE_FLASHES) return true;
    }
    return false;
}

// ─── Blinker update ───────────────────────────────────────────────────────────
static void updateBlinker(BlinkerPos pos) {
    uint32_t now = millis();

    switch (blinkerFSM) {

    case BlinkerFSM::OFF:
        if (pos == BlinkerPos::RIGHT) { blinkerFSM = BlinkerFSM::PEND_R; pendTimer = now; }
        else if (pos == BlinkerPos::LEFT) { blinkerFSM = BlinkerFSM::PEND_L; pendTimer = now; }
        break;

    // ── PEND: stalk is pushed, waiting to decide lane-change vs latch ─────────
    case BlinkerFSM::PEND_R:
        if (pos == BlinkerPos::LEFT) { blinkerFSM = BlinkerFSM::PEND_L; pendTimer = now; break; }
        if (pos == BlinkerPos::OFF) {
            // Released before timeout → lane change
            blinkerFSM = BlinkerFSM::LANE_R;
            startLane(RELAY_BLINK_R);
        } else if (now - pendTimer >= TAP_TIMEOUT_MS) {
            // Held long enough → latch
            blinkerFSM = BlinkerFSM::LATCH_R;
            relayOn(RELAY_BLINK_R);
        }
        break;

    case BlinkerFSM::PEND_L:
        if (pos == BlinkerPos::RIGHT) { blinkerFSM = BlinkerFSM::PEND_R; pendTimer = now; break; }
        if (pos == BlinkerPos::OFF) {
            blinkerFSM = BlinkerFSM::LANE_L;
            startLane(RELAY_BLINK_L);
        } else if (now - pendTimer >= TAP_TIMEOUT_MS) {
            blinkerFSM = BlinkerFSM::LATCH_L;
            relayOn(RELAY_BLINK_L);
        }
        break;

    // ── LANE: 3-flash sequence, MCU controls relay ───────────────────────────
    case BlinkerFSM::LANE_R:
        if (tickLane(RELAY_BLINK_R)) blinkerFSM = BlinkerFSM::OFF;
        // Upgrade to latch if user pushes stalk over again during flash
        if (pos == BlinkerPos::RIGHT) { blinkerFSM = BlinkerFSM::LATCH_R; relayOn(RELAY_BLINK_R); }
        break;

    case BlinkerFSM::LANE_L:
        if (tickLane(RELAY_BLINK_L)) blinkerFSM = BlinkerFSM::OFF;
        if (pos == BlinkerPos::LEFT) { blinkerFSM = BlinkerFSM::LATCH_L; relayOn(RELAY_BLINK_L); }
        break;

    // ── LATCH: relay held, Hella flasher drives indicator timing ─────────────
    case BlinkerFSM::LATCH_R:
        if (pos == BlinkerPos::OFF || pos == BlinkerPos::LEFT) {
            blinkerFSM = BlinkerFSM::OFF;
            relayOff(RELAY_BLINK_R);
        }
        break;

    case BlinkerFSM::LATCH_L:
        if (pos == BlinkerPos::OFF || pos == BlinkerPos::RIGHT) {
            blinkerFSM = BlinkerFSM::OFF;
            relayOff(RELAY_BLINK_L);
        }
        break;
    }
}

// ─── Front wiper update ───────────────────────────────────────────────────────
static void updateFrontWiper(WiperPos pos) {
    uint32_t now = millis();
    switch (frontFSM) {
    case FrontWiperFSM::OFF:
        switch (pos) {
        case WiperPos::INTERVAL: frontFSM = FrontWiperFSM::INTERVAL_ON; frontTimer = now; relayOn(RELAY_WIPER1); break;
        case WiperPos::SPEED1:   frontFSM = FrontWiperFSM::SPD1;        relayOn(RELAY_WIPER1); break;
        case WiperPos::SPEED2:   frontFSM = FrontWiperFSM::SPD2;        relayOn(RELAY_WIPER2); break;
        case WiperPos::WASH_F:   frontFSM = FrontWiperFSM::WASH_ACTIVE; relayOn(RELAY_WASH_F); relayOn(RELAY_WIPER1); break;
        default: break;
        }
        break;

    case FrontWiperFSM::INTERVAL_ON:
        if (pos != WiperPos::INTERVAL) { frontFSM = FrontWiperFSM::OFF; relayOff(RELAY_WIPER1); break; }
        if (now - frontTimer >= WIPER_INT_ON_MS) { frontTimer = now; frontFSM = FrontWiperFSM::INTERVAL_OFF; relayOff(RELAY_WIPER1); }
        break;

    case FrontWiperFSM::INTERVAL_OFF:
        if (pos != WiperPos::INTERVAL) { frontFSM = FrontWiperFSM::OFF; relayOff(RELAY_WIPER1); break; }
        if (now - frontTimer >= WIPER_INT_OFF_MS) { frontTimer = now; frontFSM = FrontWiperFSM::INTERVAL_ON; relayOn(RELAY_WIPER1); }
        break;

    case FrontWiperFSM::SPD1:
        if (pos != WiperPos::SPEED1) { frontFSM = FrontWiperFSM::OFF; relayOff(RELAY_WIPER1); }
        break;

    case FrontWiperFSM::SPD2:
        if (pos != WiperPos::SPEED2) { frontFSM = FrontWiperFSM::OFF; relayOff(RELAY_WIPER2); }
        break;

    case FrontWiperFSM::WASH_ACTIVE:
        if (pos != WiperPos::WASH_F) { relayOff(RELAY_WASH_F); frontFSM = FrontWiperFSM::WASH_EXTRA; frontTimer = now; }
        break;

    case FrontWiperFSM::WASH_EXTRA:
        if (now - frontTimer >= WASH_EXTRA_WIPE_MS) { frontFSM = FrontWiperFSM::OFF; relayOff(RELAY_WIPER1); }
        break;
    }
}

// ─── Rear wiper update ────────────────────────────────────────────────────────
static void updateRearWiper(RearPos pos) {
    uint32_t now = millis();
    switch (rearFSM) {
    case RearFSM::OFF:
        if (pos == RearPos::WIPE_R) { rearFSM = RearFSM::WIPE; relayOn(RELAY_WIPER_R); }
        else if (pos == RearPos::WASH_R) { rearFSM = RearFSM::WASH_ACTIVE; relayOn(RELAY_WIPER_R); relayOn(RELAY_WASH_R); }
        break;
    case RearFSM::WIPE:
        if (pos != RearPos::WIPE_R) { rearFSM = RearFSM::OFF; relayOff(RELAY_WIPER_R); }
        break;
    case RearFSM::WASH_ACTIVE:
        if (pos != RearPos::WASH_R) { relayOff(RELAY_WASH_R); rearFSM = RearFSM::WASH_EXTRA; rearTimer = now; }
        break;
    case RearFSM::WASH_EXTRA:
        if (now - rearTimer >= WASH_EXTRA_WIPE_MS) { rearFSM = RearFSM::OFF; relayOff(RELAY_WIPER_R); }
        break;
    }
}

// ─── Debug names ──────────────────────────────────────────────────────────────
static const char* blFSMName(BlinkerFSM s) {
    switch (s) {
    case BlinkerFSM::OFF:    return "OFF";
    case BlinkerFSM::PEND_R: return "PEND_R";
    case BlinkerFSM::LANE_R: return "LANE_R";
    case BlinkerFSM::LATCH_R:return "LATCH_R";
    case BlinkerFSM::PEND_L: return "PEND_L";
    case BlinkerFSM::LANE_L: return "LANE_L";
    case BlinkerFSM::LATCH_L:return "LATCH_L";
    default: return "?";
    }
}
static const char* wpFSMName(FrontWiperFSM s) {
    switch (s) {
    case FrontWiperFSM::OFF:           return "OFF";
    case FrontWiperFSM::INTERVAL_ON:   return "INT_ON";
    case FrontWiperFSM::INTERVAL_OFF:  return "INT_OFF";
    case FrontWiperFSM::SPD1:          return "SPD1";
    case FrontWiperFSM::SPD2:          return "SPD2";
    case FrontWiperFSM::WASH_ACTIVE:   return "WASH";
    case FrontWiperFSM::WASH_EXTRA:    return "WASH+";
    default: return "?";
    }
}
static const char* rrFSMName(RearFSM s) {
    switch (s) {
    case RearFSM::OFF:         return "OFF";
    case RearFSM::WIPE:        return "WIPE";
    case RearFSM::WASH_ACTIVE: return "WASH";
    case RearFSM::WASH_EXTRA:  return "WASH+";
    default: return "?";
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    analogReadResolution(12);

    // Read calibration button BEFORE relay init (shares D7/PA8)
    pinMode(PIN_CALIB_BTN, INPUT_PULLDOWN);
    bool calibHeld = (digitalRead(PIN_CALIB_BTN) == HIGH);

    const uint8_t relays[] = {
        RELAY_BLINK_L, RELAY_BLINK_R, RELAY_WIPER1,
        RELAY_WIPER2,  RELAY_WASH_F,  RELAY_WIPER_R, RELAY_WASH_R
    };
    for (uint8_t p : relays) { pinMode(p, OUTPUT); relayOff(p); }

    EEPROM.begin();

    if (calibHeld) {
        delay(200);
        if (digitalRead(PIN_CALIB_BTN) == HIGH) {
            Serial.println(F("[!] Hold detected — flash calibrate firmware to calibrate."));
            Serial.println(F("    pio run -e calibrate --target upload"));
        }
    }

    if (loadConfig()) {
        Serial.println(F("[OK] Calibration loaded from EEPROM."));
    } else {
        loadDefaults();
        Serial.println(F("[WARN] No calibration data — using defaults based on Corsa C measurements."));
        Serial.println(F("       Flash calibrate firmware to calibrate:"));
        Serial.println(F("       pio run -e calibrate --target upload"));
    }
    Serial.println(F("[OK] Ready."));
}

// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    int rawBlinker = readAdcSmoothed(PIN_BLINKER);
    int rawWiper   = readAdcSmoothed(PIN_WIPER);
    int rawRear    = readAdcSmoothed(PIN_REAR);

    BlinkerPos bPos = classifyBlinker(rawBlinker, cfg.blinkerThresh, cfg.blinkerMap);
    WiperPos   wPos = classifyWiper  (rawWiper,   cfg.wiperThresh,   cfg.wiperMap);
    RearPos    rPos = classifyRear   (rawRear,     cfg.rearThresh,    cfg.rearMap);

    updateBlinker   (bPos);
    updateFrontWiper(wPos);
    updateRearWiper (rPos);

    uint32_t now = millis();
    if (now - lastDebug >= DEBUG_PRINT_MS) {
        lastDebug = now;
        Serial.print(F("BL=")); Serial.print(rawBlinker);
        Serial.print(F(" WP=")); Serial.print(rawWiper);
        Serial.print(F(" RR=")); Serial.print(rawRear);
        Serial.print(F(" | BL=")); Serial.print(blFSMName(blinkerFSM));
        Serial.print(F(" WP=")); Serial.print(wpFSMName(frontFSM));
        Serial.print(F(" RR=")); Serial.println(rrFSMName(rearFSM));
    }
}
