# Steering Control - LIN Bus Communication System
## 5V Laboratory Test Setup

Complete LIN bus implementation for steering wheel control using **RP2040 (Slave)** and **Arduino Uno R3 (Master)** with **5V laboratory power supply** for safe prototyping.

---

## 📋 Table of Contents
- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [MCP2003 LIN Transceiver Pinout](#mcp2003-lin-transceiver-pinout)
- [Complete Wiring Diagram](#complete-wiring-diagram)
- [Software Installation](#software-installation)
- [Testing Procedure](#testing-procedure)
- [LED Status Indicators](#led-status-indicators)
- [Troubleshooting](#troubleshooting)
- [Upgrading to 12V Automotive Setup](#upgrading-to-12v-automotive-setup)

---

## 🔍 System Overview

### Architecture
```
┌─────────────────┐       LIN Bus        ┌─────────────────┐
│   RP2040 Slave  │◄─────────────────────►│ Arduino Master  │
│  + MCP2003 #1   │   (Single Wire)       │  + MCP2003 #2   │
└─────────────────┘                       └─────────────────┘
      ↓                                           ↓
  7 Buttons                                  Status LEDs
  2 Potis (10kΩ)                             Pin 12, 13
```

### Key Features
✅ **5V Laboratory Power** - Safe for prototyping (NOT 12V automotive)
✅ **10kΩ Potentiometers** - Ideal impedance for RP2040 ADC
✅ **5kΩ Pull-up Resistors** - MCP2003 RXD and CS pins
✅ **LIN Protocol** - Standard 19200 baud, break field, checksum validation
✅ **100ms Polling** - Real-time button and potentiometer data
✅ **LED Status** - Visual feedback for communication and errors

### Power Configuration
- **RP2040**: Powered via USB (5V → 3.3V onboard regulator)
- **Arduino Uno**: Powered via USB or 5V external supply
- **MCP2003 VBB**: Both connected to 5V lab supply
- **Logic Levels**: 3.3V (RP2040) and 5V (Arduino) - compatible via MCP2003

---

## 🛠️ Hardware Requirements

### RP2040 Slave Components
| Component | Specification | Quantity |
|-----------|--------------|----------|
| Waveshare RP2040-Tiny | Main controller | 1 |
| MCP2003 LIN Transceiver | PDIP-8 or SOIC-8 | 1 |
| Tactile Push Buttons | Active LOW | 7 |
| Linear Potentiometers | **10kΩ** (B10K) | 2 |
| Pull-up Resistors | **5kΩ** 1/4W | 2 |
| Filter Capacitors | 100nF ceramic | 2 |
| LED (optional) | External status LED | 1 |
| Resistor (for LED) | 220Ω 1/4W | 1 |

### Arduino Uno Master Components
| Component | Specification | Quantity |
|-----------|--------------|----------|
| Arduino Uno R3 | ATmega328P | 1 |
| MCP2003 LIN Transceiver | PDIP-8 or SOIC-8 | 1 |
| Pull-up Resistors | **5kΩ** 1/4W | 2 |
| Termination Resistor | **1kΩ** 1/4W | 1 |
| Termination Capacitor | 100nF ceramic | 1 |
| Status LED (optional) | External for Pin 12 | 1 |
| Resistor (for LED) | 220Ω 1/4W | 1 |

### Power Supply
- **5V Laboratory Power Supply** (stable, regulated)
- **Current capacity**: Minimum 500mA (both boards + transceivers)
- **Alternative**: Use USB power from computer (for testing)

### Wiring
- **LIN Bus Wire**: Twisted pair or single wire, max 10m for testing
- **Ground Wire**: Common ground for all components
- **Breadboard** or **prototype PCB** for connections

---

## 🔌 MCP2003 LIN Transceiver Pinout

### MCP2003A/B PDIP-8 / SOIC-8 Package (Corrected Pinout)

```
        MCP2003
      ┌─────────┐
RXD  1│●        │8  VREN (Voltage Regulator Enable)
CS   2│         │7  VBB  (Battery Voltage Input)
WAKE 3│         │6  LBUS (LIN Bus)
TXD  4│         │5  VSS  (Ground)
      └─────────┘
       (Top View)
```

### Pin Functions

| Pin | Name | Function | Connection Notes |
|-----|------|----------|------------------|
| 1 | RXD | Receive Data | To MCU RX pin + **5kΩ pull-up** |
| 2 | CS | Chip Select | To MCU GPIO + **5kΩ pull-up**, keep HIGH for active mode |
| 3 | WAKE | Wake-up | Optional, can be left floating for always-active mode |
| 4 | TXD | Transmit Data | To MCU TX pin (direct connection) |
| 5 | VSS | Ground | Connect to system GND |
| 6 | LBUS | LIN Bus | LIN bus wire (single wire communication) |
| 7 | VBB | Battery Voltage | **5V power supply** (NOT 12V for this setup) |
| 8 | VREN | Voltage Regulator Enable | Connect to 5V or leave floating |

### Important Notes
⚠️ **RXD (Pin 1)** and **CS (Pin 2)** require **5kΩ pull-up resistors**
⚠️ **VBB (Pin 7)** must be **5V** for this laboratory setup (NOT 12V)
⚠️ **LBUS (Pin 6)** on Arduino side needs **1kΩ termination to GND + 100nF cap**

---

## 📐 Complete Wiring Diagram

### RP2040 Slave + MCP2003 Connections

```
Waveshare RP2040-Tiny                 MCP2003 LIN Transceiver
┌───────────────────────────┐         ┌────────────────────────┐
│                           │         │                        │
│  GPIO0 (TX) ──────────────┼────────►│ Pin 4 (TXD)            │
│                           │         │                        │
│  GPIO1 (RX) ◄─────────────┼─────────│ Pin 1 (RXD)            │
│             │             │    ┌────┤         ↑              │
│             └─[ 5kΩ ]────┼────┘    │    (5kΩ to 3.3V)       │
│                      3.3V │         │                        │
│                           │         │                        │
│  GPIO2 ────────────────┬──┼────────►│ Pin 2 (CS)             │
│            [ 5kΩ ]─────┘  │    ┌────┤         ↑              │
│                      3.3V │    │    │    (5kΩ to 3.3V)       │
│                           │    │    │                        │
│  GPIO25 (LED) ─[ 220Ω ]──┼───[LED]─┤ (Optional ext LED)     │
│                           │    │    │                        │
│  3.3V ────────────────────┼────┘    │                        │
│  GND ─────────────────────┼─────────┤ Pin 5 (VSS)            │
│                           │         │                        │
│  5V (USB) ────────────────┼─────────┤ Pin 7 (VBB)            │
│           (or Lab Supply) │         │ Pin 8 (VREN) ─┐        │
│                           │         │               ├─ 5V    │
└───────────────────────────┘         └───────────────┼────────┘
                                                      │
                                      LIN Bus ────────┤ Pin 6 (LBUS)
                                                      │
Buttons (7x):                                         │
GPIO6  ──┤ Button 1 ├─ GND                           │
GPIO7  ──┤ Button 2 ├─ GND                           │
GPIO8  ──┤ Button 3 ├─ GND                           │
GPIO9  ──┤ Button 4 ├─ GND                           │
GPIO10 ──┤ Button 5 ├─ GND                           │
GPIO11 ──┤ Button 6 ├─ GND                           │
GPIO12 ──┤ Button 7 ├─ GND                           │
(Internal pull-ups enabled in code)                  │
                                                      │
Potentiometers (10kΩ):                               │
        3.3V                                          │
         │                                            │
    ┌────┴────┐                                      │
    │  POT 1  │                                      │
    │  10kΩ   │                                      │
    └────┬────┘                                      │
         ├───[ 100nF ]─ GND                          │
         └──► GPIO26 (ADC0)                          │
                                                      │
        3.3V                                          │
         │                                            │
    ┌────┴────┐                                      │
    │  POT 2  │                                      │
    │  10kΩ   │                                      │
    └────┬────┘                                      │
         ├───[ 100nF ]─ GND                          │
         └──► GPIO27 (ADC1)                          │
                                                      │
                                                      │
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━│━━
                       LIN BUS (Single Wire)         │
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━│━━
                                                      │
Arduino Uno R3                    MCP2003 LIN Transceiver
┌───────────────────────────┐         ┌────────────────────────┐
│                           │         │                        │
│  Pin 1 (TX) ──────────────┼────────►│ Pin 4 (TXD)            │
│                           │         │                        │
│  Pin 0 (RX) ◄─────────────┼─────────│ Pin 1 (RXD)            │
│             │             │    ┌────┤         ↑              │
│             └─[ 5kΩ ]────┼────┘    │    (5kΩ to 5V)         │
│                        5V │         │                        │
│                           │         │                        │
│  Pin 2 ─────────────────┬─┼────────►│ Pin 2 (CS)             │
│            [ 5kΩ ]──────┘ │    ┌────┤         ↑              │
│                        5V │    │    │    (5kΩ to 5V)         │
│                           │    │    │                        │
│  Pin 12 (Status) ─[ 220Ω ]┼───[LED]─┤ (Optional ext LED)     │
│  Pin 13 (Error)  ─[ 220Ω ]┼───[LED]─┤ (+ built-in LED)       │
│                           │    │    │                        │
│  5V ──────────────────────┼────┴────┤ Pin 7 (VBB)            │
│                           │         │ Pin 8 (VREN) ─┐        │
│  GND ─────────────────────┼─────────┤ Pin 5 (VSS)   ├─ 5V    │
│                           │         │               │        │
└───────────────────────────┘         └───────────────┼────────┘
                                                      │
                                      LIN Bus ────────┤ Pin 6 (LBUS)
                                                      │
                                                 [ 1kΩ ]
                                                      │
                                                     GND
                                                      ║
                                                 [ 100nF ]
                                                      │
                                                     GND
```

### Key Wiring Notes

1. **Pull-up Resistors (5kΩ)**:
   - RP2040: RXD → 3.3V, CS → 3.3V
   - Arduino: RXD → 5V, CS → 5V
   - These are CRITICAL for proper MCP2003 operation

2. **LIN Bus Termination (Arduino side only)**:
   - 1kΩ resistor from LBUS to GND
   - 100nF capacitor from LBUS to GND
   - Only on ONE end of the bus (master side)

3. **Potentiometer Connections (10kΩ)**:
   - Left pin: GND
   - Center pin (wiper): ADC input (GPIO26 or GPIO27)
   - Right pin: 3.3V
   - 100nF filter capacitor from wiper to GND

4. **Button Connections**:
   - One side: GPIO pin
   - Other side: GND
   - Internal pull-ups enabled in code (no external resistors needed)

5. **Power Supply**:
   - Both MCP2003 VBB pins: 5V from lab supply
   - RP2040: USB power (5V → 3.3V via onboard regulator)
   - Arduino: USB or external 5V

---

## 💻 Software Installation

### 1. Install Arduino IDE 2.x
Download from: https://www.arduino.cc/en/software

### 2. Install RP2040 Core (for RP2040 Slave)
1. Open Arduino IDE → Tools → Board Manager
2. Search for "Raspberry Pi Pico"
3. Install "Raspberry Pi Pico/RP2040" by Earle F. Philhower, III (version 4.0+)
4. Select Board: **Tools → Board → Raspberry Pi RP2040 Boards → Waveshare RP2040-Tiny**

### 3. Upload RP2040 Slave Program
1. Open `steering_wheel_controller.ino`
2. Verify board selection: **Waveshare RP2040-Tiny**
3. Select Port (RP2040 appears as USB Serial device)
4. Click **Upload**
5. **Expected behavior**:
   - LED blinks 3x quickly on startup
   - LED slow heartbeat (1 blink per second) when idle
   - LED blinks 100ms when LIN frame is transmitted

### 4. Upload Arduino Uno Master Program
1. Open `lin_bus_master.ino`
2. Select Board: **Arduino Uno**
3. Select Port
4. Click **Upload**
5. **Expected behavior**:
   - Both LEDs (Pin 12 and 13) blink 3x on startup
   - Pin 12 toggles on each valid frame received
   - Pin 13 blinks 3x on errors (timeout, checksum)

---

## ✅ Testing Procedure

### Test 1: Individual Board Power-Up

**RP2040 Slave (standalone):**
1. Connect RP2040 via USB (no LIN bus yet)
2. Upload code
3. **Expected**: LED blinks 3x, then slow heartbeat
4. Press any button → LED should blink briefly
5. **Conclusion**: RP2040 is waiting for LIN master requests

**Arduino Uno Master (standalone):**
1. Connect Arduino via USB (no LIN bus yet)
2. Upload code
3. **Expected**: Both LEDs blink 3x at startup
4. **Expected**: Pin 13 (error LED) blinks repeatedly (timeout - normal without slave)
5. **Conclusion**: Arduino is sending LIN requests but no slave responding

### Test 2: LIN Bus Connection

1. **Power off both boards**
2. **Wire the LIN bus**:
   - Connect RP2040 MCP2003 LBUS (Pin 6) to Arduino MCP2003 LBUS (Pin 6)
   - Ensure common GND between both boards
   - Add 1kΩ + 100nF termination on Arduino side
3. **Power on both boards**
4. **Expected behavior**:
   - RP2040 LED: Slow heartbeat + 100ms blinks (transmitting)
   - Arduino Pin 12: Toggling every 100ms (receiving valid frames)
   - Arduino Pin 13: OFF (no errors)

**Success indicators:**
✅ RP2040 LED blinks 100ms every 100ms (in sync with master polling)
✅ Arduino Pin 12 toggles ON/OFF every 100ms
✅ Arduino Pin 13 stays OFF (no errors)

### Test 3: Button Input

1. Press **Button 1** (GPIO6 to GND)
2. **Expected**: RP2040 LED blinks, Arduino continues receiving
3. Press multiple buttons simultaneously
4. **Expected**: All button states transmitted correctly
5. **Validation**: Use Serial Monitor (see below) to see button data

### Test 4: Potentiometer Input

1. Rotate **POT1** (GPIO26) from min to max
2. **Expected**: ADC values change from 0 to 4095
3. Rotate **POT2** (GPIO27) from min to max
4. **Expected**: ADC values change from 0 to 4095
5. **Note**: 10kΩ potis should give smooth, stable readings

### Test 5: Serial Monitor (Optional - See Note Below)

⚠️ **IMPORTANT**: Serial Monitor on Arduino Uno conflicts with LIN (uses same pins 0/1)!

**For initial testing only:**
1. Open `lin_bus_master.ino`
2. **Uncomment** lines 140-144 in `setup()`:
   ```cpp
   Serial.end();
   Serial.begin(115200);
   Serial.println(F("=== LIN BUS MASTER ==="));
   ```
3. **Uncomment** lines 298-324 in `printSteeringData()`
4. Re-upload to Arduino
5. Open Serial Monitor at **115200 baud**
6. **Expected output**:
   ```
   === LIN BUS MASTER ===
   Arduino Uno @ 5V Lab Supply
   Polling RP2040 slave every 100ms...
   BTN: [ ][ ][ ][ ][ ][ ][ ] | POT1: 0512 (12%) | POT2: 2048 (50%)
   BTN: [1][ ][ ][ ][ ][ ][ ] | POT1: 0512 (12%) | POT2: 2048 (50%)
   BTN: [1][2][3][ ][ ][ ][ ] | POT1: 1024 (25%) | POT2: 3456 (84%)
   ```

**After testing:**
- Comment out Serial.println() lines again
- Use LED status indicators for production

---

## 🚨 LED Status Indicators

### RP2040 Slave (GPIO25)
| Pattern | Meaning | Action |
|---------|---------|--------|
| 3 quick blinks at startup | System initialized | Normal |
| Slow blink (1 Hz) | Idle, waiting for master | Normal |
| 100ms blink | LIN frame transmitted | Normal operation |

### Arduino Uno Master

**Pin 12 (Status LED):**
| Pattern | Meaning | Action |
|---------|---------|--------|
| Toggles every 100ms | Valid frames received | Normal operation |
| Stays OFF | No communication | Check LIN bus wiring |

**Pin 13 (Error LED):**
| Pattern | Meaning | Action |
|---------|---------|--------|
| 3 quick blinks (repeating) | Timeout or checksum error | Check slave power, LIN wiring |
| Stays OFF | No errors | Normal operation |

---

## 🐛 Troubleshooting

### Issue: No Communication (Pin 12 stays OFF, Pin 13 blinks)

**Check:**
1. **Power**:
   - Both boards powered?
   - MCP2003 VBB = 5V on both?
   - Check with multimeter

2. **LIN Bus Wiring**:
   - LBUS pins connected? (MCP2003 Pin 6 on both sides)
   - Common ground between both boards?
   - Termination resistor (1kΩ) on Arduino side?

3. **Pull-up Resistors**:
   - RXD pull-ups installed? (5kΩ to 3.3V on RP2040, 5kΩ to 5V on Arduino)
   - CS pull-ups installed? (5kΩ to 3.3V on RP2040, 5kΩ to 5V on Arduino)
   - Measure voltage on CS pins → should be HIGH (~3.3V or ~5V)

4. **MCP2003 Connections**:
   - Verify pinout against datasheet (see diagram above)
   - Pin 1 (RXD) NOT swapped with Pin 4 (TXD)?
   - Pin 7 (VBB) connected to 5V?
   - Pin 5 (VSS) connected to GND?

**Test procedure:**
- Disconnect LIN bus wire
- Measure voltage on LBUS pins (should be ~2.5V idle with MCP2003 powered)
- Reconnect LIN bus
- Use oscilloscope on LBUS to verify LIN break field (700µs LOW every 100ms)

---

### Issue: Intermittent Communication (Pin 13 blinks occasionally)

**Causes:**
1. **Bad connections** - Check breadboard wiring, re-seat wires
2. **Missing termination** - Verify 1kΩ + 100nF on Arduino LBUS
3. **Power supply noise** - Add 100µF bulk capacitor near each MCP2003 VBB pin
4. **EMI** - Keep LIN bus wire away from power cables, use twisted pair if possible

**Solutions:**
- Solder connections instead of breadboard
- Add decoupling capacitors (100nF) near MCP2003 VCC pins
- Reduce LIN bus wire length (< 1m for testing)

---

### Issue: Buttons Not Registering

**Check:**
1. **Wiring**: Buttons connected between GPIO and GND?
2. **Code**: Internal pull-ups enabled in `steering_wheel_controller.ino`?
3. **Test**: Upload simple test code to verify GPIO:
   ```cpp
   void setup() {
     Serial.begin(115200);
     pinMode(6, INPUT_PULLUP);  // Button 1
   }
   void loop() {
     Serial.println(digitalRead(6));  // Should print 1 (not pressed) or 0 (pressed)
     delay(100);
   }
   ```

**Expected**: Pressing button → value changes from 1 to 0

---

### Issue: ADC Values Incorrect or Noisy

**Check:**
1. **Potentiometer Type**: Linear (B10K), not logarithmic (A10K)?
2. **Wiring**:
   - Left pin → GND
   - Center pin → GPIO26 or GPIO27
   - Right pin → 3.3V (NOT 5V!)
3. **Filter Capacitor**: 100nF from wiper to GND installed?
4. **ADC Resolution**: `analogReadResolution(12)` called in setup?

**Test with multimeter:**
- Measure voltage at center pin while rotating potentiometer
- Should vary smoothly from 0V to 3.3V
- If jumpy → bad potentiometer or poor contact

**Test code:**
```cpp
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}
void loop() {
  Serial.print("ADC0: ");
  Serial.println(analogRead(26));
  delay(100);
}
```

**Expected**: Values from 0 to 4095, smooth changes when rotating

---

### Issue: Checksum Errors (Pin 13 blinks)

**Causes:**
- Electrical noise on LIN bus
- Incorrect checksum calculation
- Data corruption during transmission

**Solutions:**
1. **Add decoupling capacitors**:
   - 100nF near each MCP2003 VBB pin
   - 100µF bulk capacitor on 5V power rail

2. **Verify code**:
   - Both programs use same checksum algorithm?
   - Check `calculateChecksum()` function in both files

3. **Reduce EMI**:
   - Use twisted pair for LIN bus wire
   - Keep wire away from power supplies, motors, relays
   - Add ferrite bead on LIN bus wire (optional)

4. **Check power supply**:
   - Stable 5V? (measure with multimeter under load)
   - If using USB power, try external 5V supply

---

### Issue: Arduino Upload Fails

**Problem**: "Programmer not responding" or "Serial port busy"

**Solution:**
- Arduino Uno uses pins 0/1 for programming AND LIN
- **During upload**: Disconnect MCP2003 from pins 0/1
- **After upload**: Reconnect MCP2003
- **Alternative**: Use ICSP header for programming (requires ISP programmer)

---

## 🔄 Upgrading to 12V Automotive Setup

### Current Setup (5V Laboratory)
```
5V Lab Supply ──→ MCP2003 VBB (Pin 7)
```

### Automotive Setup (12V Battery)
```
12V Battery ──→ [ DC-DC Converter ] ──→ 5V ──→ MCP2003 VBB (Pin 7)
                     (12V → 5V)
```

### Required Components for 12V Upgrade

1. **DC-DC Converter (Buck Converter)**:
   - Input: 9-16V (automotive battery range)
   - Output: 5V @ 500mA minimum
   - Recommended: LM2596 module or similar
   - Efficiency: > 85%

2. **Protection Components**:
   - Reverse polarity protection diode (1N5819 or similar)
   - TVS diode for voltage spike protection (SMAJ12A or similar)
   - Input capacitor: 100µF electrolytic, 25V
   - Output capacitor: 100µF electrolytic, 10V

3. **Wiring Changes**:
   - Use thicker wire for 12V input (18 AWG minimum)
   - Add fuse: 1A fast-blow on 12V input
   - Keep 5V wiring to MCP2003 same as current setup

### Conversion Steps

1. **Add DC-DC converter**:
   ```
   12V Battery (+) ──┬─[ Fuse 1A ]──┬─[ Reverse Diode ]──┬─[ TVS Diode ]──┬─ DC-DC IN+
                     │              │                     │                │
                     └─[ 100µF ]────┴─────────────────────┴────────────────┘
                            │                             │
                           GND                           GND

   DC-DC OUT+ (5V) ──┬─[ 100µF ]──┬─ MCP2003 VBB (both transceivers)
                     │            │
                    GND          GND
   ```

2. **No code changes needed** - Programs are voltage-independent

3. **Test procedure**:
   - Test DC-DC converter output with multimeter (should be 5.0V ± 0.1V)
   - Connect 5V output to MCP2003 VBB
   - Verify operation same as 5V lab setup

### Notes on Automotive Environment

⚠️ **Additional considerations for automotive use:**
- **EMI/EMC compliance**: Add ferrite beads, shielded cables
- **Temperature range**: Use automotive-grade components (-40°C to +125°C)
- **Vibration resistance**: Use locked connectors, strain relief
- **Moisture protection**: Conformal coating or potting

---

## 📊 Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Power Supply** | 5V ± 5% | Laboratory or USB |
| **Current Draw** | < 200mA | Both boards + transceivers |
| **LIN Baud Rate** | 19200 | Standard LIN rate |
| **Polling Rate** | 10 Hz | 100ms interval |
| **Button Response Time** | < 150ms | 100ms poll + 50ms debounce |
| **ADC Resolution** | 12-bit | 0-4095 (10kΩ potis) |
| **ADC Update Rate** | 10 Hz | Same as polling rate |
| **LIN Bus Length** | < 40m | Depends on transceiver, 1-10m recommended for testing |
| **Communication Reliability** | > 99% | With proper wiring and termination |

---

## 📚 References

- **LIN Specification 2.2A**: https://www.lin-cia.org/
- **MCP2003A/B Datasheet**: https://www.microchip.com/en-us/product/MCP2003B
- **RP2040 Datasheet**: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
- **Arduino-Pico Core**: https://github.com/earlephilhower/arduino-pico
- **ATmega328P Datasheet**: https://www.microchip.com/en-us/product/ATmega328P

---

## 📝 Component Notes

### Why 10kΩ Potentiometers?
- **Ideal impedance** for RP2040 ADC (SAR type, 500kHz sampling)
- **Fast settling time** with 100nF filter cap
- **Low current draw** (~330µA at 3.3V)
- **Better than 20kΩ**: Faster response, less noise
- **Better than 5kΩ**: Lower current, less power waste

### Why 5kΩ Pull-up Resistors?
- **MCP2003 requirement**: RXD and CS need pull-ups per datasheet
- **5kΩ recommended**: Balance between strong pull-up and power consumption
- **Too high (10kΩ+)**: Slow rise time, susceptible to noise
- **Too low (1kΩ)**: Excessive current draw

### Why 1kΩ LIN Termination?
- **LIN specification**: Recommends 1kΩ ± 10% termination
- **Single-ended termination**: Only on master (Arduino) side
- **100nF capacitor**: Filters high-frequency noise
- **Critical for stability**: Prevents reflections on LIN bus

---

## 🤝 Support

### Quick Diagnostics

1. **No communication at all**:
   - Check power (5V on VBB pins)
   - Check pull-up resistors (5kΩ on RXD, CS)
   - Verify MCP2003 pinout (Pin 1 = RXD, NOT TXD)

2. **Intermittent errors**:
   - Add decoupling capacitors (100nF near MCP2003)
   - Check termination resistor (1kΩ on Arduino LBUS)
   - Reduce LIN bus wire length

3. **ADC issues**:
   - Verify 10kΩ poti, not 20kΩ or other value
   - Check wiper to 3.3V, not 5V
   - Add 100nF filter cap if missing

4. **Button issues**:
   - Verify internal pull-ups enabled in code
   - Check button wiring (GPIO to GND when pressed)

### Still Having Issues?
- Double-check wiring against diagrams above
- Use multimeter to verify all voltages
- Test each board individually before connecting LIN bus
- Review code comments for detailed explanations

---

**Created with Claude Code - 2026-02-12**
**Version 2.0 - 5V Laboratory Test Setup**
