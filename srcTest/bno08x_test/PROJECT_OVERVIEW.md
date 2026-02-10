# Dual BNO08x Head Tracking System - Project Overview

## Project Purpose

This is a **head tracking system for controlling a searchlight on a moving car**. It uses two BNO08x 9-DOF IMU sensors to calculate the relative orientation of the driver's head compared to the car's movement, allowing the searchlight to follow where the driver is looking regardless of vehicle motion.

---

## System Architecture

### Hardware Components
- **2x GY-BNO08x IMU sensors** (BNO085/BNO086)
- **Arduino Nano Every** (ATmega4809)
- **I2C communication** at 100 kHz

### Sensor Configuration
1. **Helmet Sensor (0x4B)**: Mounted on the back of the driver's helmet
   - Tracks absolute head orientation in 3D space
   - I2C address: 0x4B (SA0 pin floating or HIGH)

2. **Car Reference Sensor (0x4A)**: Mounted rigidly on the car body
   - Tracks absolute car orientation in 3D space
   - I2C address: 0x4A (SA0 pin tied to GND)

### Communication Protocol
- **I2C Mode**: Both sensors use PS0=LOW, PS1=LOW
- **Report Type**: SH2_ROTATION_VECTOR (quaternion-based orientation)
- **Update Rate**: 5000 µs (200 Hz)
- **Data Format**: Quaternions (qw, qx, qy, qz) converted to Euler angles (Yaw, Pitch, Roll)

---

## Code Architecture

### File Structure
```
bno08x_test/
├── src/
│   └── main.cpp          # Main application code
├── include/              # Header files (if needed)
├── lib/                  # Custom libraries (if needed)
├── platformio.ini        # PlatformIO configuration
├── PROJECT_OVERVIEW.md   # This file
├── QUICK_START.md        # Wiring and startup guide
└── WIRING.md            # Detailed wiring documentation
```

### Dependencies (platformio.ini)
```ini
[env:nano_every]
platform = atmelmegaavr
board = nano_every
framework = arduino
lib_deps = adafruit/Adafruit BNO08x@^1.2.5
monitor_speed = 115200
```

### Code Structure (main.cpp)

#### 1. Global Objects and Configuration
```cpp
// Helmet sensor (0x4B) - SA0 floating/HIGH
Adafruit_BNO08x bno_helmet = Adafruit_BNO08x(HELMET_RST_PIN);

// Car sensor (0x4A) - SA0 tied to GND
Adafruit_BNO08x bno_car = Adafruit_BNO08x(CAR_RST_PIN);
```

#### 2. Setup Flow
1. Configure PS0/PS1 pins (LOW/LOW = I2C mode)
2. Reset both sensors simultaneously
3. Initialize I2C bus at 100 kHz
4. Scan I2C bus for devices (debugging)
5. Initialize helmet sensor at 0x4B
6. Initialize car sensor at 0x4A
7. Enable SH2_ROTATION_VECTOR reports on both
8. Enter main loop

#### 3. Main Loop Logic
```
Loop:
  ├─> Read helmet sensor
  │   └─> Convert quaternion to Euler angles
  │
  ├─> Read car sensor
  │   └─> Convert quaternion to Euler angles
  │
  └─> When both updated:
      ├─> Calculate relative orientation (helmet - car)
      ├─> Normalize angles to -180..+180
      └─> Output: REL, Helmet, Car orientations
```

#### 4. Key Functions

**`quatToEuler(qw, qx, qy, qz, &yaw, &pitch, &roll)`**
- Converts quaternion to Euler angles (radians)
- Uses standard aerospace sequence (ZYX)
- Handles gimbal lock protection with constrain()

**`normalizeAngle(angle)`**
- Wraps angles to -180° to +180° range
- Essential for calculating meaningful differences
- Example: 350° - 10° = -20° (not 340°)

---

## Coordinate System

### Sensor Axes (Right-Hand Rule)
```
        Y (Left)
        ↑
        |
        |
X ←-----●----→ (Forward, marked side of chip)
(Back)
        |
        |
        ↓
      Z (Down)
```

### Euler Angles
- **Yaw (ψ)**: Rotation around Z-axis (vertical)
  - Range: -180° to +180°
  - 0° = facing forward, +90° = turned left, -90° = turned right

- **Pitch (θ)**: Rotation around Y-axis (lateral)
  - Range: -180° to +180°
  - 0° = level, +90° = looking straight up, -90° = looking straight down

- **Roll (φ)**: Rotation around X-axis (longitudinal)
  - Range: -180° to +180°
  - 0° = upright, +90° = tilted left, -90° = tilted right

### Relative Orientation Calculation
```
REL_Yaw   = Helmet_Yaw   - Car_Yaw
REL_Pitch = Helmet_Pitch - Car_Pitch
REL_Roll  = Helmet_Roll  - Car_Roll

(all normalized to -180..+180)
```

**Interpretation for Searchlight Control:**
- **REL_Yaw > 0**: Head turned left relative to car → Pan searchlight left
- **REL_Yaw < 0**: Head turned right relative to car → Pan searchlight right
- **REL_Pitch > 0**: Head looking up relative to car → Tilt searchlight up
- **REL_Pitch < 0**: Head looking down relative to car → Tilt searchlight down

---

## Serial Output Format

```
REL=> Y:15.3 P:-8.2 R:3.1 | Helmet=> Y:45.2 P:12.3 R:-5.1 | Car=> Y:29.9 P:20.5 R:-8.2
```

- **REL**: Relative orientation (use for searchlight control)
- **Helmet**: Absolute helmet orientation (world space)
- **Car**: Absolute car orientation (world space)
- All values in degrees with 1 decimal precision

---

## Pin Assignments

### Arduino Nano Every Pin Usage

| Pin | Function | Notes |
|-----|----------|-------|
| A4 | I2C SDA | Shared by both sensors |
| A5 | I2C SCL | Shared by both sensors |
| D2 | INT (optional) | Interrupt pin for faster polling |
| D4 | Helmet RST | Reset pin for helmet sensor |
| D5 | Car RST | Reset pin for car sensor |
| D6 | Helmet PS0 | Protocol select 0 (I2C mode) |
| D7 | Helmet PS1 | Protocol select 1 (I2C mode) |
| D8 | Car PS0 | Protocol select 0 (I2C mode) |
| D9 | Car PS1 | Protocol select 1 (I2C mode) |
| 3.3V | Power | Both sensors VIN |
| GND | Ground | Both sensors GND + Car sensor SA0 |

### Critical: SA0 Configuration
- **Helmet sensor SA0**: Leave floating or tie to 3.3V → Address 0x4B
- **Car sensor SA0**: Tie to GND → Address 0x4A

---

## Development Notes

### Current Status
- ✅ Dual sensor initialization
- ✅ I2C address management (0x4A and 0x4B)
- ✅ Quaternion to Euler conversion
- ✅ Relative orientation calculation
- ✅ Serial output formatting
- ⏳ Searchlight servo control (future)
- ⏳ Calibration routine (future)
- ⏳ Filtering/smoothing (future)

### Known Limitations
1. **No sensor fusion with car motion**: Currently simple subtraction, doesn't account for car acceleration
2. **Gimbal lock possible**: Near ±90° pitch, roll/yaw become ambiguous
3. **No calibration**: Assumes both sensors have same zero reference
4. **No filtering**: Raw data output, may be noisy in moving vehicle

### Future Enhancements

#### 1. Servo/Motor Control
Add PWM outputs to drive pan/tilt servos:
```cpp
#include <Servo.h>
Servo panServo;
Servo tiltServo;

// Map angles to servo range
int panAngle = map(rel_yaw, -90, 90, 0, 180);
int tiltAngle = map(rel_pitch, -45, 45, 0, 180);
panServo.write(panAngle);
tiltServo.write(tiltAngle);
```

#### 2. Kalman Filtering
Implement sensor fusion to smooth noisy readings:
```cpp
// Use Kalman filter library
// Especially important in vibrating car environment
```

#### 3. Calibration Routine
Store zero-position offsets in EEPROM:
```cpp
// Press button to store current position as "center"
// Subtract offsets from all future readings
float yaw_offset = 0;
float pitch_offset = 0;

void calibrate() {
  yaw_offset = helmet_yaw - car_yaw;
  pitch_offset = helmet_pitch - car_pitch;
  EEPROM.put(0, yaw_offset);
  EEPROM.put(4, pitch_offset);
}
```

#### 4. Deadzone/Sensitivity
Prevent micro-movements from causing jitter:
```cpp
float applyDeadzone(float value, float threshold) {
  if (abs(value) < threshold) return 0;
  return value;
}

float rel_yaw_filtered = applyDeadzone(rel_yaw, 2.0); // 2° deadzone
```

#### 5. Alternative: Quaternion Difference
More accurate than Euler subtraction, avoids gimbal lock:
```cpp
// Calculate relative quaternion: q_rel = q_helmet * conjugate(q_car)
// Convert q_rel to Euler angles
// More complex but mathematically correct
```

---

## Troubleshooting Guide

### Sensor Not Detected
**Symptom**: "FEHLER: Helmet/Car sensor nicht gefunden!"

**Solutions**:
1. Check I2C wiring (SDA to A4, SCL to A5)
2. Verify power (3.3V, not 5V)
3. Check SA0 configuration (helmet floating, car to GND)
4. Run I2C scanner - should see 0x4A and 0x4B
5. Verify PS0/PS1 are LOW during sensor boot

### No Data Output
**Symptom**: Initialization succeeds but no REL data

**Solutions**:
1. Check that `enableReport()` succeeded
2. Increase delay after `hardwareReset()` (currently 300ms)
3. Try slower I2C speed (50 kHz instead of 100 kHz)
4. Check for I2C bus conflicts (other devices)

### Noisy Readings
**Symptom**: Values jump erratically

**Solutions**:
1. Add 0.1µF capacitors near each sensor's VIN/GND
2. Shorten I2C wires (max 30cm recommended)
3. Add 4.7kΩ pull-up resistors on SDA/SCL
4. Implement moving average filter in software
5. Shield cables if near motors/high current

### Relative Angles Don't Make Sense
**Symptom**: REL values are wrong even when head is straight

**Solutions**:
1. Verify both sensors mounted with same orientation
2. Both sensors' forward markers should point forward
3. Implement calibration routine (store offsets)
4. Check for magnetic interference (car speakers, motors)

### Gimbal Lock
**Symptom**: Erratic yaw/roll when pitch near ±90°

**Solutions**:
1. Switch to quaternion-based difference calculation
2. Use game rotation vector (SH2_GAME_ROTATION_VECTOR)
3. Add gimbal lock detection and warning

---

## Performance Metrics

- **Update Rate**: ~200 Hz per sensor (5ms interval)
- **I2C Speed**: 100 kHz (can increase to 400 kHz for shorter wires)
- **Latency**: <10ms sensor-to-serial
- **Memory Usage**:
  - RAM: ~3100 bytes (50% of 6KB)
  - Flash: ~21.5KB (44% of 48KB)

---

## References

### Documentation
- [Adafruit BNO08x Library](https://github.com/adafruit/Adafruit_BNO08x)
- [BNO085 Datasheet](https://www.ceva-dsp.com/product/bno080-085/)
- [Arduino Nano Every](https://docs.arduino.cc/hardware/nano-every)

### Algorithms
- Quaternion to Euler: Standard aerospace sequence (ZYX/Tait-Bryan)
- Angle normalization: Modulo with sign preservation

### I2C Addresses
- BNO08x default: 0x4A (SA0=LOW) or 0x4B (SA0=HIGH)
- Max I2C speed: 400 kHz (use 100 kHz for reliability)

---

## License & Credits

**Hardware**: Arduino Nano Every + GY-BNO08x breakout boards
**Software**: Arduino framework with Adafruit BNO08x library
**Application**: Custom head tracking for vehicle searchlight control

Created: 2026-02-10
Last Updated: 2026-02-10
