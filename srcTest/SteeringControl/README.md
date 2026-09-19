# Steering Control - LIN Bus Communication System
## 12V Laboratory Test Setup

LIN bus link for the steering wheel: **RP2040 (Slave)** reads buttons and potentiometers, **Arduino Uno R3 (Master)** polls them. Both use an **MCP2003** LIN transceiver powered from a **12V laboratory supply**.

> **Revision 3.0 (2026-09-18)** — wiring corrected against the MCP2003 datasheets
> ([DS20002230G](https://ww1.microchip.com/downloads/en/DeviceDoc/20002230G.pdf), [MCP2003B DS20005463C](https://ww1.microchip.com/downloads/en/DeviceDoc/2000546C3.pdf)).
> The previous 5V setup could not work:
> - **VBB = 5V is below the power-on threshold** (VBB > 5.5V, MCP2003B `VBBUV_RISE` 5.5–6.0V). The transceiver stays in POR with transmitter *and* receiver off.
> - **Master termination was inverted**: 1kΩ goes from LBUS **to VBB** (pull-up), not to GND. The bus capacitor is **220pF**, not 100nF.
> - **VREN (pin 8) is an output** — leave it unconnected.
> - **No pull-up on CS** — the datasheet advises against tying CS high; the MCU drives it.

---

## Table of Contents
- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [MCP2003 Pinout](#mcp2003-pinout)
- [Wiring](#wiring)
- [LIN Frame](#lin-frame)
- [Software](#software)
- [Testing Procedure](#testing-procedure)
- [Known Firmware Issues](#known-firmware-issues)
- [Troubleshooting](#troubleshooting)

---

## System Overview

```
┌─────────────────┐        LIN Bus        ┌─────────────────┐
│   RP2040 Slave  │◄─────────────────────►│ Arduino Master  │
│  + MCP2003 #1   │     (single wire)     │  + MCP2003 #2   │
└─────────────────┘                       └─────────────────┘
      ↓                                           ↓
  7 Buttons                                  Status LEDs
  2 Pots (10kΩ)                              Pin 12, 13
```

### Power
| Rail | Source | Consumers |
|------|--------|-----------|
| **12V** | Lab supply | MCP2003 #1 VBB, MCP2003 #2 VBB, master 1kΩ pull-up |
| 5V | Uno (USB) | Uno logic, master RXD pull-up |
| 3.3V | RP2040 onboard regulator (USB) | RP2040 logic, slave RXD pull-up, pots |
| **GND** | **common** | Lab supply, Uno, RP2040, both MCP2003 |

- MCP2003 logic inputs (TXD, CS) accept **VIH ≥ 2.0V** → 3.3V from the RP2040 is fine.
- RXD is **open-drain** → pull it up to the MCU's own supply (3.3V on RP2040, 5V on Uno). Never to 12V.

---

## Hardware Requirements

### RP2040 Slave
| Component | Specification | Qty |
|-----------|---------------|-----|
| Waveshare RP2040-Tiny | Main controller | 1 |
| MCP2003 / MCP2003A / MCP2003B | PDIP-8 or SOIC-8 | 1 |
| Tactile push buttons | to GND, internal pull-ups | 7 |
| Linear potentiometer | **10kΩ** (B10K) | 2 |
| Resistor | **4.7–5kΩ** (RXD pull-up to 3.3V) | 1 |
| Capacitor | 100nF ceramic (pot filter) | 2 |
| Capacitor | 100nF ceramic (VBB decoupling) | 1 |
| Capacitor | 220pF ceramic (LBUS to GND) | 1 |
| External LED + 220Ω | optional status LED | 1 |

### Arduino Uno Master
| Component | Specification | Qty |
|-----------|---------------|-----|
| Arduino Uno R3 | ATmega328P | 1 |
| MCP2003 / MCP2003A / MCP2003B | PDIP-8 or SOIC-8 | 1 |
| Resistor | **4.7–5kΩ** (RXD pull-up to 5V) | 1 |
| Resistor | **1kΩ** (master LIN pull-up to VBB) | 1 |
| Diode | 1N4148 (in series with the 1kΩ) | 1 |
| Capacitor | 100nF ceramic (VBB decoupling) | 1 |
| Capacitor | 220pF ceramic (LBUS to GND) | 1 |
| LEDs + 220Ω | status (pin 12), error (pin 13) | 2 |

### Power Supply
- **Lab supply set to 12V**, current limit ~100mA is plenty for two transceivers
- Microcontrollers stay on USB

---

## MCP2003 Pinout

```
        MCP2003
      ┌─────────┐
RXD  1│●        │8  VREN  (output! leave open)
CS   2│         │7  VBB   (12V)
WAKE 3│         │6  LBUS  (LIN bus)
TXD  4│         │5  VSS   (GND)
      └─────────┘
       (top view)
```

| Pin | Name | Type | Connection |
|-----|------|------|------------|
| 1 | RXD | open-drain output, mirrors LBUS | MCU RX + **pull-up to MCU supply** |
| 2 | CS | input, internal pull-down | MCU GPIO, driven HIGH by firmware. **No pull-up** |
| 3 | WAKE | input, internal 800k pull-up (MCP2003/A only) | leave open |
| 4 | TXD | input (VIH ≥ 2.0V) | MCU TX, direct |
| 5 | VSS | ground | common GND |
| 6 | LBUS | LIN bus, internal 30k pull-up + diode to VBB | bus wire + 220pF to GND |
| 7 | VBB | supply, **> 5.5V to start** | 12V lab supply + 100nF to GND |
| 8 | VREN | output, high (VBB level) in all modes except power-down | **not connected** |

### Mode behaviour worth knowing
- Power-on with CS LOW → Ready mode (receiver on, transmitter off).
- CS goes HIGH with TXD HIGH → Operation mode (transmitter on). With TXD LOW → Transmitter-Off mode.
  → Initialise the UART (TXD idles HIGH) **before** setting CS HIGH.
- **RXD monitoring:** RXD must read HIGH (> 2.5V typ.) while the bus is recessive, otherwise the chip faults into Transmitter-Off mode. A missing RXD pull-up therefore also kills transmitting.
- RXD always mirrors the bus — **every node receives its own transmitted bytes (echo)**.

---

## Wiring

### Slave: RP2040-Tiny ↔ MCP2003 #1
| From | To | Note |
|------|----|------|
| GPIO0 (UART0 TX) | MCP2003 pin 4 (TXD) | direct |
| GPIO1 (UART0 RX) | MCP2003 pin 1 (RXD) | + 4.7–5kΩ pull-up to **3.3V** |
| GPIO2 | MCP2003 pin 2 (CS) | direct, no pull-up |
| GND | MCP2003 pin 5 (VSS) | |
| 12V lab supply | MCP2003 pin 7 (VBB) | + 100nF to GND |
| — | MCP2003 pin 8 (VREN) | open |
| — | MCP2003 pin 3 (WAKE) | open |
| MCP2003 pin 6 (LBUS) | LIN bus wire | + 220pF to GND |
| GPIO6 … GPIO12 | button 1 … 7 → GND | internal pull-ups |
| GPIO26 (ADC0) | POT1 wiper | pot ends: GND / 3.3V, 100nF wiper→GND |
| GPIO27 (ADC1) | POT2 wiper | pot ends: GND / 3.3V, 100nF wiper→GND |
| GPIO14 | 220Ω → LED → GND | status LED (see note) |

> **Status LED:** the RP2040-Tiny has **no plain LED on GPIO25** (that is the Pico). Its onboard LED is a
> **WS2812 RGB on GPIO16** ([schematic](https://files.waveshare.com/upload/7/7a/RP2040-Tiny_Schematic.pdf)),
> which needs a NeoPixel driver. Use an external LED on GPIO14 instead.

### Master: Arduino Uno ↔ MCP2003 #2
| From | To | Note |
|------|----|------|
| D1 (TX) | MCP2003 pin 4 (TXD) | direct — **disconnect during upload** |
| D0 (RX) | MCP2003 pin 1 (RXD) | + 4.7–5kΩ pull-up to **5V** — **disconnect during upload** |
| D2 | MCP2003 pin 2 (CS) | direct, no pull-up |
| GND | MCP2003 pin 5 (VSS) | |
| 12V lab supply | MCP2003 pin 7 (VBB) | + 100nF to GND |
| — | MCP2003 pin 8 (VREN) | open |
| — | MCP2003 pin 3 (WAKE) | open |
| MCP2003 pin 6 (LBUS) | LIN bus wire | + 220pF to GND |
| 12V → 1N4148 → 1kΩ | LBUS | **master pull-up** (anode at 12V) |
| D12 | 220Ω → LED → GND | status (toggles per valid frame) |
| D13 | 220Ω → LED → GND | error (+ onboard LED) |

### Bus
```
                 12V
                  │
                 ─┴─ 1N4148
                 ─┬─
                  │
                [1kΩ]   (master only)
                  │
MCP2003 #2 LBUS ──┴──────────── LIN wire ────────────┬── MCP2003 #1 LBUS
                  │                                  │
               [220pF]                            [220pF]
                  │                                  │
GND ──────────────┴──────────── common GND ──────────┴── GND
```

- Recessive (idle) bus level ≈ VBB minus a diode drop (~11.3V). Dominant ≈ 0–1.2V.
- LIN wire up to a few metres is fine for the lab.

---

## LIN Frame

```
Master: | BREAK (≥13 bit LOW) | SYNC 0x55 | PID 0x3C |
Slave:                                              | BTN | ADC0_H | ADC0_L | ADC1_H | ADC1_L | CHK |
```

| Byte | Content |
|------|---------|
| BTN | bits 0–6 = buttons 1–7 (1 = pressed) |
| ADC0_H / ADC0_L | POT1, 12 bit: upper 8 bits / lower 4 bits in the upper nibble |
| ADC1_H / ADC1_L | POT2, same layout |
| CHK | sum of the 5 data bytes & 0xFF (not LIN-conformant: no inversion, no carry) |

- 19200 baud, master polls every 100ms, response timeout 20ms.
- Break is generated by sending `0x00` at 9600 baud (≈ 937µs LOW).

---

## Software

PlatformIO, one environment per board. `set_src_dir.py` routes each environment to its folder.

```bash
pio run -e rp2040_slave -t upload   # slave/steering_wheel_controller.ino
pio run -e uno_master  -t upload    # master/lin_bus_master.ino
```

- RP2040 not detected → hold BOOTSEL while plugging in USB.
- Uno upload fails → disconnect MCP2003 from D0/D1 first.

| Environment | Folder | Purpose |
|-------------|--------|---------|
| `rp2040_slave` | `slave/` | LIN slave firmware v3.0 |
| `uno_master` | `master/` | LIN master firmware v3.0 |
| `rp2040_bus_test` | `test_rp2040/` | Wiring test: echo self-test, counts bytes from the Uno (USB serial) |
| `uno_bus_test` | `test_uno/` | Wiring test: D7→GND sends 0x55 + checks echo, LEDs show bytes from the RP2040 |

The former GPIO/UART diagnostic and LED input test sketches are in commit `1ccbb05` and earlier.

---

## Testing Procedure

### 1. Transceivers alone (no MCU traffic)
1. Lab supply 12V, current limit ~100mA, both MCP2003 wired, bus connected.
2. Measure LBUS against GND: **≈ 11–12V** (recessive). ≈ 0V means something pulls the bus down.
3. Measure VBB on both chips: 12V.

### 2. Slave alone
- Upload slave firmware, check that buttons and pots are read (serial debug over USB works on the RP2040).

### 3. Master alone
- Without a slave the error LED (D13) must blink (timeout).
- Oscilloscope on LBUS: break (≈ 1ms LOW), then 0x55, 0x3C every 100ms, swinging between ~0V and ~11V.

### 4. Together
- Status LED (D12) toggles every 100ms, error LED stays off.
- Press buttons / turn pots → values change on the master.

---

## Firmware Fixes (v3.0)

Issues found by code review of commit `65c6823` and fixed in `master/` and `slave/` v3.0:

1. **Master read its own echo.** RXD mirrors the bus, so after sending `0x55 0x3C` the master's RX buffer contained
   those two bytes before the slave's response → checksum failed every frame.
   *Fix:* `requestSlaveData()` reads and checks the 2 echo bytes, then the 6 response bytes.
2. **Slave blocked for 100ms after each response** (`delay(LED_BLINK_TIME)`), equal to the master's poll interval,
   while the master only waits 20ms → roughly every second request timed out.
   *Fix:* non-blocking status LED (`updateStatusLED()`).
3. **Slave frame detection dropped the SYNC byte.** The "break gap" was measured when new bytes had already
   arrived and then flushed the RX buffer. Arduino-Pico discards the break byte (framing error), so the first byte
   after the 100ms pause is always `0x55` — and exactly that byte was flushed: the old slave never answered.
   *Fix:* byte-stream state machine (`0x55` directly followed by `0x3C` = request).
4. **Slave received the echo of its own response** (`0x55` in buttons + `0x3C` in ADC0_MSB would look like a request).
   *Fix:* the 6 echo bytes are read back right after sending.
5. **Status LED on GPIO25** does not exist on the RP2040-Tiny. *Fix:* GPIO14.
6. **Master error LED blocked the poll loop for 300ms.** *Fix:* non-blocking blink.

The original versions are in `wokwi/simA-master` and `wokwi/simB-slave` to reproduce the bugs in simulation.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| No response at all, RXD always HIGH | VBB < 5.5V (chip stuck in POR), or CS never driven HIGH |
| LBUS ≈ 0V permanently | a resistor from LBUS to GND (old README wiring), short, or TXD held LOW |
| Checksum error on every frame | firmware older than v3.0 (echo not discarded) |
| Every second frame times out | firmware older than v3.0 (blocking delay in slave) |
| Wiring unclear | flash `uno_bus_test` / `rp2040_bus_test` and follow the measurement guide |
| Chip transmits nothing although VBB is fine | RXD pull-up missing → RXD monitoring fault → Transmitter-Off mode |
| Uno upload fails | MCP2003 still connected to D0/D1 |
| Pots noisy | 100nF wiper→GND missing, or pot connected to 5V instead of 3.3V |

---

## Moving to the Vehicle

The lab setup is already 12V, so for the vehicle only protection is added in front of VBB:
reverse-polarity diode, TVS (datasheet example: 43V), fuse, and 100nF + bulk capacitor at each transceiver.
The MCP2003 datasheet recommends an external reverse-battery blocking diode on VBB.

---

**Revision 3.0 — 2026-09-18** (wiring corrected against MCP2003 datasheets)
Revision 2.0 — 2026-02-12 (5V lab setup, superseded)
