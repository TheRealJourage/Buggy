# Dual BNO08x Head Tracking System - Wiring Guide

## Overview
This system uses two BNO08x sensors for relative head tracking in a moving car:
- **Helmet Sensor** (0x4B): Mounted on the back of helmet, tracks head orientation
- **Car Sensor** (0x4A): Mounted on the car, provides reference orientation

The system calculates **relative orientation** to control a searchlight regardless of car movement.

---

## Pin Connections

### Shared I2C Bus (both sensors)
| Connection | Arduino Nano Every Pin | Notes |
|------------|------------------------|-------|
| SDA (both) | A4 | I2C Data |
| SCL (both) | A5 | I2C Clock |
| VIN (both) | 3.3V | Power (NOT 5V!) |
| GND (both) | GND | Ground |

### Helmet Sensor (Address 0x4B)
| GY-BNO08x Pin | Arduino Pin | Notes |
|---------------|-------------|-------|
| PS0 | D6 | Protocol Select 0 (set LOW for I2C) |
| PS1 | D7 | Protocol Select 1 (set LOW for I2C) |
| RST | D4 | Reset pin |
| SA0 | **Leave floating or tie HIGH** | Sets address to 0x4B |
| INT | D2 (optional) | Interrupt pin |

### Car Reference Sensor (Address 0x4A)
| GY-BNO08x Pin | Arduino Pin | Notes |
|---------------|-------------|-------|
| PS0 | D8 | Protocol Select 0 (set LOW for I2C) |
| PS1 | D9 | Protocol Select 1 (set LOW for I2C) |
| RST | D5 | Reset pin |
| SA0 | **GND** | Sets address to 0x4A (CRITICAL!) |
| INT | - (not used) | Optional |

---

## Critical: SA0 Pin Configuration

The **SA0 pin determines the I2C address**:
- **Helmet sensor**: SA0 floating or HIGH → Address 0x4B
- **Car sensor**: SA0 tied to GND → Address 0x4A

**You MUST configure SA0 differently on each sensor!**

---

## Sensor Orientation

### Axes Definition
```
        Y (Left)
        ↑
        |
        |
X ←-----●----→
(Back)    (Forward)
        |
        |
        ↓
      Z (Down)
```

### Mounting Guidelines

**Helmet Sensor:**
- Mount on the back of helmet
- Chip's marked side should face **forward** (direction you're looking)
- X-axis points forward, Y-axis points left, Z-axis points down

**Car Sensor:**
- Mount rigidly on car body (roof, dashboard, etc.)
- Orient the same way: marked side forward (car's direction of travel)
- X-axis points forward, Y-axis points left, Z-axis points down

**Important:** Both sensors should have the **same orientation** relative to their mounting surface for the math to work correctly!

---

## Serial Output Format

```
REL=> Y:15.3 P:-8.2 R:3.1 | Helmet=> Y:45.2 P:12.3 R:-5.1 | Car=> Y:29.9 P:20.5 R:-8.2
```

### Output Explained:
- **REL** = Relative orientation (HEAD relative to CAR) ← **Use this for searchlight control!**
  - `Y`: Yaw difference (left/right) - positive = head turned left relative to car
  - `P`: Pitch difference (up/down) - positive = head looking up relative to car
  - `R`: Roll difference (tilt) - positive = head tilted left relative to car

- **Helmet** = Absolute helmet orientation (in world space)
- **Car** = Absolute car orientation (in world space)

### For Searchlight Control:
Use the **REL** values to drive servos/motors:
- **REL Y**: Controls horizontal rotation (pan)
- **REL P**: Controls vertical angle (tilt)
- **REL R**: Usually ignored for searchlight (or used for stabilization)

Range: -180° to +180° for all axes

---

## Testing Without Second Sensor

If you only have ONE sensor connected right now, the code will stop at:
```
FEHLER: Car sensor nicht gefunden!
Stelle sicher dass SA0 auf GND liegt!
```

To test with just the helmet sensor, you can temporarily comment out the car sensor initialization in `setup()`.

---

## Upload Command

```bash
pio run -t upload && pio device monitor -b 115200
```

---

## Troubleshooting

### Only one sensor detected
- Check SA0 wiring - helmet sensor SA0 should be floating, car sensor SA0 to GND
- Run I2C scan - you should see TWO devices (0x4A and 0x4B)

### Relative values don't make sense
- Check that both sensors are mounted with same orientation
- Both sensors' forward direction should align with their mounting surface

### Noisy readings
- Add 0.1µF capacitors near each sensor's power pins
- Shorten I2C wires if possible
- Add 4.7kΩ pull-up resistors on SDA and SCL lines

