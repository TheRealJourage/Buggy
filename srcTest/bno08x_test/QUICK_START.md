# Quick Start Guide - Dual BNO08x Head Tracking

This guide will get your dual sensor head tracking system up and running.

---

## What You Need

### Hardware
- [ ] 1x Arduino Nano Every
- [ ] 2x GY-BNO08x IMU sensors (BNO085/BNO086)
- [ ] Breadboard or prototype PCB
- [ ] Jumper wires (at least 20)
- [ ] USB cable (Arduino to computer)
- [ ] Optional: 2x 0.1µF capacitors (for noise filtering)
- [ ] Optional: 2x 4.7kΩ resistors (I2C pull-ups if needed)

### Software
- [x] PlatformIO installed (VS Code extension)
- [x] This project downloaded
- [x] USB driver for Arduino Nano Every (usually automatic)

---

## Step 1: Wiring the First Sensor (Helmet Sensor)

### Power Connections
1. **GY-BNO08x VIN** → Arduino **3.3V** ⚠️ NOT 5V!
2. **GY-BNO08x GND** → Arduino **GND**

### I2C Bus (shared between both sensors)
3. **GY-BNO08x SDA** → Arduino **A4**
4. **GY-BNO08x SCL** → Arduino **A5**

### Control Pins (Helmet Sensor Specific)
5. **GY-BNO08x PS0** → Arduino **D6**
6. **GY-BNO08x PS1** → Arduino **D7**
7. **GY-BNO08x RST** → Arduino **D4**
8. **GY-BNO08x SA0** → **Leave floating** (do NOT connect)
9. **GY-BNO08x INT** → Arduino **D2** (optional, can skip)

### Visual Check
```
Helmet Sensor (0x4B):
  VIN ──┬→ Arduino 3.3V
        │
  GND ──┴→ Arduino GND

  SDA ────→ Arduino A4 (shared)
  SCL ────→ Arduino A5 (shared)

  PS0 ────→ Arduino D6
  PS1 ────→ Arduino D7
  RST ────→ Arduino D4
  SA0 ────  NOT CONNECTED (floating = 0x4B address)
  INT ────→ Arduino D2 (optional)
```

---

## Step 2: Test First Sensor

Before wiring the second sensor, verify the first one works.

### Upload Code
```bash
cd c:\Users\ft-kr\Documents\PlatformIO\Projects\bno08x_test
pio run -t upload
```

### Expected Serial Output (with only one sensor)
```
==== DUAL BNO08x Head Tracking System ====
Helmet sensor: 0x4B | Car sensor: 0x4A

Scanning I2C bus...
  Device at 0x4B
Found 1 device(s)

Initializing HELMET sensor (0x4B)...
  Helmet sensor gefunden!
  Helmet sensor bereit!
Initializing CAR sensor (0x4A)...
FEHLER: Car sensor nicht gefunden!  ← EXPECTED! Second sensor not connected yet
Stelle sicher dass SA0 auf GND liegt!
```

✅ **If you see "Helmet sensor bereit!" the first sensor is working!**
❌ **If you see "FEHLER: Helmet sensor nicht gefunden!", check wiring above**

---

## Step 3: Wiring the Second Sensor (Car Reference)

Now add the second sensor to the same breadboard.

### Power (Parallel to First Sensor)
10. **2nd BNO08x VIN** → Arduino **3.3V** (same rail as sensor 1)
11. **2nd BNO08x GND** → Arduino **GND** (same rail as sensor 1)

### I2C Bus (Shared with First Sensor)
12. **2nd BNO08x SDA** → Arduino **A4** (same as sensor 1)
13. **2nd BNO08x SCL** → Arduino **A5** (same as sensor 1)

### Control Pins (Car Sensor Specific)
14. **2nd BNO08x PS0** → Arduino **D8**
15. **2nd BNO08x PS1** → Arduino **D9**
16. **2nd BNO08x RST** → Arduino **D5**
17. **2nd BNO08x SA0** → Arduino **GND** ⚠️ CRITICAL! This changes address to 0x4A
18. **2nd BNO08x INT** → Leave unconnected

### Visual Check
```
Car Sensor (0x4A):
  VIN ──┬→ Arduino 3.3V (shared rail)
        │
  GND ──┴→ Arduino GND (shared rail)

  SDA ────→ Arduino A4 (SHARED with helmet sensor)
  SCL ────→ Arduino A5 (SHARED with helmet sensor)

  PS0 ────→ Arduino D8
  PS1 ────→ Arduino D9
  RST ────→ Arduino D5
  SA0 ────→ Arduino GND ⚠️ MUST be grounded for 0x4A address!
  INT ────  NOT CONNECTED
```

---

## Step 4: Complete Wiring Diagram

### Both Sensors Connected

```
Arduino Nano Every                GY-BNO08x #1 (Helmet)     GY-BNO08x #2 (Car)
┌─────────────────┐              ┌──────────────────┐      ┌──────────────────┐
│                 │              │    Address 0x4B  │      │    Address 0x4A  │
│  3.3V  ○────────┼──────────────┤ VIN              │──────┤ VIN              │
│                 │              │                  │      │                  │
│  GND   ○────────┼──────────────┤ GND              │──┬───┤ GND              │
│                 │              │                  │  │   │                  │
│  A4(SDA) ○──────┼──────────────┤ SDA              │──┼───┤ SDA              │
│                 │              │                  │  │   │                  │
│  A5(SCL) ○──────┼──────────────┤ SCL              │──┼───┤ SCL              │
│                 │              │                  │  │   │                  │
│  D2    ○────────┼──────────────┤ INT (optional)   │  │   │                  │
│  D4    ○────────┼──────────────┤ RST              │  │   │                  │
│  D5    ○────────┼──────────────┼──────────────────┼──┼───┤ RST              │
│  D6    ○────────┼──────────────┤ PS0              │  │   │                  │
│  D7    ○────────┼──────────────┤ PS1              │  │   │                  │
│  D8    ○────────┼──────────────┼──────────────────┼──┼───┤ PS0              │
│  D9    ○────────┼──────────────┼──────────────────┼──┼───┤ PS1              │
│                 │              │                  │  │   │                  │
│                 │              │ SA0 (floating)   │  │   │ SA0 ─────────────┤─→ GND ⚠️
│                 │              │                  │  │   │                  │
└─────────────────┘              └──────────────────┘  │   └──────────────────┘
                                                        │
                                                     GND RAIL
```

---

## Step 5: Upload and Test Both Sensors

### Upload Code
```bash
pio run -t upload && pio device monitor -b 115200
```

### Expected Serial Output (SUCCESS)
```
==== DUAL BNO08x Head Tracking System ====
Helmet sensor: 0x4B | Car sensor: 0x4A

Scanning I2C bus...
  Device at 0x4A
  Device at 0x4B
Found 2 device(s)  ← You should see TWO devices!

Initializing HELMET sensor (0x4B)...
  Helmet sensor gefunden!
  Helmet sensor bereit!
Initializing CAR sensor (0x4A)...
  Car sensor gefunden!
  Car sensor bereit!

==== System Ready! ====
Format: RelYaw RelPitch RelRoll | Helmet | Car

REL=> Y:0.3 P:-0.1 R:0.2 | Helmet=> Y:45.2 P:12.3 R:-5.1 | Car=> Y:44.9 P:12.4 R:-5.3
REL=> Y:0.5 P:-0.3 R:0.1 | Helmet=> Y:45.4 P:12.1 R:-5.2 | Car=> Y:44.9 P:12.4 R:-5.3
...
```

✅ **Success!** You should see continuous data streaming with REL, Helmet, and Car values.

---

## Step 6: Testing the System

### Test 1: Both Sensors Stationary
Keep both sensors still on the table.
- **Expected**: REL values should be close to 0.0 (small noise ±1-2°)
- **If not**: Check that both sensors have the same physical orientation

### Test 2: Move Helmet Sensor Only
Rotate the helmet sensor (sensor #1) left/right:
- **Expected**: Helmet Y changes, Car Y stays constant, REL Y = Helmet Y - Car Y
- **Example**:
  ```
  Helmet Y: 45° → REL Y: +5° (if Car Y = 40°)
  Helmet Y: 35° → REL Y: -5° (if Car Y = 40°)
  ```

### Test 3: Move Car Sensor Only
Rotate the car sensor (sensor #2) left/right:
- **Expected**: Car Y changes, Helmet Y stays constant, REL Y compensates
- **Example**:
  ```
  Car rotates right 10° → REL Y increases by +10°
  (helmet now appears to be looking left relative to car)
  ```

### Test 4: Move Both Together
Rotate both sensors together in the same direction:
- **Expected**: REL values stay approximately constant
- **This simulates**: Car turning while head stays straight relative to car

---

## Step 7: Understanding the Output

### Serial Format
```
REL=> Y:15.3 P:-8.2 R:3.1 | Helmet=> Y:45.2 P:12.3 R:-5.1 | Car=> Y:29.9 P:20.5 R:-8.2
```

### Column Meanings

#### REL (Relative Orientation) ← USE THIS FOR SEARCHLIGHT
- **Y (Yaw)**: Head rotation left/right relative to car
  - `+` = head turned LEFT relative to car
  - `-` = head turned RIGHT relative to car
  - Range: -180° to +180°

- **P (Pitch)**: Head tilt up/down relative to car
  - `+` = head looking UP relative to car
  - `-` = head looking DOWN relative to car
  - Range: -180° to +180°

- **R (Roll)**: Head tilt left/right relative to car
  - `+` = head tilted LEFT shoulder down
  - `-` = head tilted RIGHT shoulder down
  - Range: -180° to +180°

#### Helmet (Absolute)
World-space orientation of helmet sensor

#### Car (Absolute)
World-space orientation of car sensor

---

## Step 8: Physical Mounting

### Helmet Sensor Mounting
1. Mount on the **back** of the helmet (rear of head)
2. Orient so the **chip's marked side faces forward** (direction you look)
3. Secure firmly (hot glue, velcro, 3D printed mount)
4. Route wires along helmet strap to avoid snag

### Car Sensor Mounting
1. Mount rigidly on car body (roof, dashboard, roll cage)
2. Orient the **same way** as helmet sensor (forward marker to front of car)
3. Keep away from magnets, speakers, motors (magnetic interference)
4. Secure permanently - this is your reference frame

⚠️ **CRITICAL**: Both sensors must have the **same orientation** relative to their mounting surface!

### Example Good Mounting
```
Helmet Sensor:                Car Sensor:
   Forward →                     Forward →
   ┌─────┐                       ┌─────┐
   │ ● ← Marker                  │ ● ← Marker
   │ BNO │                       │ BNO │
   └─────┘                       └─────┘
```

---

## Step 9: Integration with Searchlight

### Hardware Addition
Add servo motors for pan/tilt control:

```cpp
#include <Servo.h>

#define PAN_SERVO_PIN  10
#define TILT_SERVO_PIN 11

Servo panServo;
Servo tiltServo;

void setup() {
  // ... existing setup ...
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
}

void loop() {
  // ... existing loop ...

  if (helmet_updated && car_updated) {
    // Calculate relative angles
    float rel_yaw = normalizeAngle(helmet_yaw - car_yaw);
    float rel_pitch = normalizeAngle(helmet_pitch - car_pitch);

    // Map to servo range (adjust min/max for your servos)
    int pan = map(rel_yaw, -90, 90, 0, 180);
    int tilt = map(rel_pitch, -45, 45, 0, 180);

    // Constrain to safe range
    pan = constrain(pan, 0, 180);
    tilt = constrain(tilt, 0, 180);

    // Drive servos
    panServo.write(pan);
    tiltServo.write(tilt);

    // ... existing print statements ...
  }
}
```

---

## Troubleshooting

### Problem: "Device at 0x4B" only, no 0x4A

**Cause**: Second sensor's SA0 not grounded

**Fix**: Double-check that Car sensor's SA0 pin is connected to GND

---

### Problem: Both sensors show 0x4B

**Cause**: Car sensor's SA0 is floating instead of grounded

**Fix**:
1. Verify wire connection from Car sensor SA0 to Arduino GND
2. Check for broken wire or poor breadboard contact
3. Temporarily solder SA0 directly to GND pad on sensor

---

### Problem: I2C scanner shows no devices

**Cause**: I2C wiring problem

**Fix**:
1. Check SDA to A4 and SCL to A5
2. Verify 3.3V power is connected
3. Check ground connections
4. Try slower I2C speed: `Wire.setClock(50000);`

---

### Problem: Sensor found but no data output

**Cause**: Report enable failed

**Fix**:
1. Increase delay after hardwareReset from 300ms to 500ms
2. Check for I2C bus contention
3. Try different report type: `SH2_GAME_ROTATION_VECTOR`

---

### Problem: Noisy, jittery readings

**Cause**: Electrical noise or vibration

**Fix**:
1. Add 0.1µF capacitor between VIN and GND on each sensor
2. Add 4.7kΩ pull-up resistors on SDA and SCL lines
3. Shorten I2C wires
4. Keep wires away from motors and high-current lines
5. Implement software filtering (moving average)

---

### Problem: REL values wrong even when aligned

**Cause**: Sensors mounted with different orientations

**Fix**:
1. Verify both sensors' forward markers point forward
2. Implement calibration routine (store zero offset)
3. Physically remount sensors with same orientation

---

## Advanced: Calibration Procedure

Add a button to zero the current position:

```cpp
#define CALIBRATE_BUTTON 3

float yaw_offset = 0;
float pitch_offset = 0;

void setup() {
  // ... existing setup ...
  pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
}

void loop() {
  // ... existing loop ...

  // Check for calibration button press
  if (digitalRead(CALIBRATE_BUTTON) == LOW) {
    yaw_offset = helmet_yaw - car_yaw;
    pitch_offset = helmet_pitch - car_pitch;
    Serial.println("CALIBRATED! Current position = zero");
    delay(1000); // debounce
  }

  // Apply offsets to relative angles
  float rel_yaw = normalizeAngle(helmet_yaw - car_yaw - yaw_offset);
  float rel_pitch = normalizeAngle(helmet_pitch - car_pitch - pitch_offset);

  // ... rest of code ...
}
```

---

## Command Reference

### Upload code
```bash
pio run -t upload
```

### Upload and open serial monitor
```bash
pio run -t upload && pio device monitor -b 115200
```

### Just open serial monitor
```bash
pio device monitor -b 115200
```

### Clean build
```bash
pio run -t clean
```

### List connected devices
```bash
pio device list
```

---

## Next Steps

1. ✅ Get both sensors working on breadboard
2. ✅ Test relative orientation calculation
3. ⏳ Add servo control for searchlight
4. ⏳ Install in vehicle and test
5. ⏳ Add calibration button
6. ⏳ Implement filtering/smoothing
7. ⏳ Design permanent mounting solution
8. ⏳ Weatherproof connections

---

## Support

If you encounter issues not covered here:
1. Check [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md) for detailed technical info
2. Check [WIRING.md](WIRING.md) for detailed wiring explanations
3. Review Adafruit BNO08x examples: https://github.com/adafruit/Adafruit_BNO08x

---

**Last Updated**: 2026-02-10
