# PlatformIO Quick Start Guide
## LIN Bus Steering Control System

This guide shows you how to build and upload the LIN bus programs using **PlatformIO** instead of Arduino IDE.

---

## 📦 Installation

### Option 1: VSCode Extension (Recommended)

1. **Install Visual Studio Code**
   - Download from: https://code.visualstudio.com/

2. **Install PlatformIO IDE Extension**
   - Open VSCode
   - Go to Extensions (Ctrl+Shift+X)
   - Search for "PlatformIO IDE"
   - Click Install
   - Restart VSCode

### Option 2: PlatformIO Core (CLI)

```bash
# Install via pip (Python required)
pip install platformio

# Or using installer script (Linux/macOS)
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
python3 get-platformio.py
```

---

## 🚀 Quick Start

### 1. Open Project in VSCode

```bash
# Navigate to project directory
cd srcTest/SteeringControl

# Open in VSCode
code .
```

### 2. Build Projects

**Using VSCode:**
- Open PlatformIO sidebar (Alien icon on left)
- Expand "rp2040_slave" or "uno_master"
- Click "Build"

**Using Terminal:**
```bash
# Build RP2040 Slave
pio run -e rp2040_slave

# Build Arduino Uno Master
pio run -e uno_master

# Build both
pio run
```

### 3. Upload to Boards

**RP2040 Slave:**
```bash
# Method 1: Auto-detect port
pio run -e rp2040_slave -t upload

# Method 2: Specify port (Windows)
pio run -e rp2040_slave -t upload --upload-port COM5

# Method 3: Specify port (Linux/macOS)
pio run -e rp2040_slave -t upload --upload-port /dev/ttyACM0
```

**Arduino Uno Master:**
```bash
# Auto-detect port
pio run -e uno_master -t upload

# Specify port (Windows)
pio run -e uno_master -t upload --upload-port COM3

# Specify port (Linux/macOS)
pio run -e uno_master -t upload --upload-port /dev/ttyUSB0
```

⚠️ **Important for Arduino Uno**: Disconnect MCP2003 from pins 0/1 during upload!

---

## 📊 Serial Monitor

### RP2040 Slave (Debug Output)

```bash
# Open monitor
pio device monitor -e rp2040_slave

# Open monitor with specific port
pio device monitor --port COM5 --baud 115200
```

### Arduino Uno Master (LIN conflicts!)

⚠️ **NOTE**: Serial Monitor on Uno conflicts with LIN (pins 0/1)

**For debugging only:**
1. Uncomment Serial.println() lines in `lin_bus_master.ino`
2. Build and upload
3. Disconnect MCP2003 from pins 0/1
4. Open Serial Monitor:
   ```bash
   pio device monitor -e uno_master
   ```
5. Reconnect MCP2003 for normal LIN operation

---

## 🔧 Common Tasks

### Check Available Ports
```bash
pio device list
```

### Clean Build Files
```bash
# Clean specific environment
pio run -e rp2040_slave -t clean

# Clean all
pio run -t clean
```

### Upload Specific File
```bash
# Upload specific .hex or .bin file
pio run -e uno_master -t upload --upload-port COM3
```

### Verbose Output
```bash
# See detailed build output
pio run -e rp2040_slave -v

# See detailed upload output
pio run -e rp2040_slave -t upload -v
```

---

## 🎯 VSCode PlatformIO Interface

### Project Tasks (Bottom Toolbar)

When you open the project in VSCode, you'll see these buttons:

| Icon | Action | Shortcut |
|------|--------|----------|
| ✓ | Build | Ctrl+Alt+B |
| → | Upload | Ctrl+Alt+U |
| 🗑️ | Clean | - |
| 🔍 | Test | - |
| 🔌 | Serial Monitor | - |

### Switching Between Environments

1. Click status bar (bottom left): **"Default (env:rp2040_slave)"**
2. Select environment:
   - `env:rp2040_slave` - Build/upload RP2040
   - `env:uno_master` - Build/upload Arduino Uno
   - `env:uno_master_debug` - Arduino Uno with Serial debug

### Building Both Projects

To upload to both boards sequentially:
1. Select `env:rp2040_slave`
2. Click Upload
3. Wait for completion
4. Select `env:uno_master`
5. Click Upload

---

## 🐛 Troubleshooting

### RP2040 Upload Issues

**Problem**: "No device found" or "picotool failed"

**Solution 1**: Use BOOTSEL mode
1. Disconnect RP2040 USB
2. Hold BOOTSEL button on RP2040
3. Connect USB while holding button
4. Release button
5. Board appears as USB drive (RPI-RP2)
6. Run upload command again

**Solution 2**: Change upload protocol
Edit `platformio.ini`:
```ini
[env:rp2040_slave]
upload_protocol = mbed  ; Instead of picotool
```

### Arduino Uno Upload Issues

**Problem**: "avrdude: stk500_recv(): programmer is not responding"

**Solution**:
1. Disconnect MCP2003 from pins 0/1 (RX/TX)
2. Check correct COM port: `pio device list`
3. Try slower upload speed:
   ```ini
   [env:uno_master]
   upload_speed = 57600  ; Instead of 115200
   ```
4. Reset Arduino board before upload
5. Reconnect MCP2003 after successful upload

### Missing Dependencies

**Problem**: "Platform raspberrypi is not installed"

**Solution**:
```bash
# Install platforms manually
pio pkg install --platform raspberrypi
pio pkg install --platform atmelavr
```

### Port Permission (Linux)

**Problem**: "Permission denied: '/dev/ttyUSB0'"

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Reboot or re-login
sudo reboot
```

### Windows COM Port Issues

**Problem**: "Could not open port 'COM5'"

**Solution**:
1. Check Device Manager → Ports (COM & LPT)
2. Install/update USB drivers
3. For RP2040: Install WinUSB driver (Zadig tool)
4. For Arduino: Install CH340/FTDI drivers

---

## 📝 Project Structure

```
SteeringControl/
├── platformio.ini              # PlatformIO configuration
├── steering_wheel_controller.ino  # RP2040 Slave code
├── lin_bus_master.ino          # Arduino Uno Master code
├── README.md                   # Hardware documentation
├── PLATFORMIO_GUIDE.md         # This file
│
├── .pio/                       # Build artifacts (auto-generated)
│   ├── build/
│   │   ├── rp2040_slave/
│   │   └── uno_master/
│   └── libdeps/
│
└── .vscode/                    # VSCode settings (optional)
    ├── extensions.json
    └── settings.json
```

---

## ⚙️ Advanced Configuration

### Custom Build Flags

Add to `platformio.ini`:

```ini
[env:rp2040_slave]
build_flags =
    ${common.build_flags}
    -D DEBUG_LIN_PROTOCOL      ; Enable LIN debug output
    -D ADC_SAMPLES=8           ; Override ADC averaging
```

### Upload via Debugger (Optional)

For advanced debugging with SWD/JTAG:

```ini
[env:rp2040_slave_debug]
extends = env:rp2040_slave
upload_protocol = cmsis-dap
debug_tool = cmsis-dap
debug_init_break = tbreak setup
```

### Library Dependencies

If you need additional libraries:

```ini
[env:rp2040_slave]
lib_deps =
    Wire
    SPI
    adafruit/Adafruit BusIO@^1.14.1
```

---

## 🔗 Useful Commands Cheat Sheet

```bash
# List all available targets
pio run --list-targets

# Update platforms and libraries
pio pkg update

# Show project info
pio project config

# Generate compilation database (for IntelliSense)
pio run -t compiledb

# Build firmware only (no upload)
pio run -e rp2040_slave

# Upload pre-built firmware
pio run -e rp2040_slave -t nobuild -t upload

# Monitor serial output (Ctrl+C to exit)
pio device monitor
```

---

## 🎓 Learning Resources

### PlatformIO Documentation
- **Getting Started**: https://docs.platformio.org/en/latest/integration/ide/vscode.html
- **Project Configuration**: https://docs.platformio.org/en/latest/projectconf/index.html
- **Platforms**: https://docs.platformio.org/en/latest/platforms/index.html

### Specific Platforms
- **RP2040**: https://docs.platformio.org/en/latest/platforms/raspberrypi.html
- **Arduino AVR**: https://docs.platformio.org/en/latest/platforms/atmelavr.html

### CLI Reference
- **Core CLI**: https://docs.platformio.org/en/latest/core/index.html
- **Device Commands**: https://docs.platformio.org/en/latest/core/userguide/device/index.html

---

## 💡 Tips & Best Practices

### 1. Use `.gitignore`
Add these to your `.gitignore`:
```
.pio/
.vscode/
*.bin
*.hex
*.elf
```

### 2. Pre-build Checks
Before uploading:
- Check hardware connections
- Verify correct environment selected
- Disconnect conflicting devices (MCP2003 on Uno during upload)

### 3. Faster Development
- Use `pio run -t upload` for quick rebuild + upload
- Keep Serial Monitor open in separate terminal
- Use VSCode tasks for one-click build+upload

### 4. Debugging
- For RP2040: Add `Serial.println()` statements (USB Serial works independently)
- For Arduino Uno: Use LED blink patterns (Serial conflicts with LIN)
- Use PlatformIO's built-in debugger for advanced debugging

---

## 🆚 PlatformIO vs Arduino IDE

| Feature | PlatformIO | Arduino IDE |
|---------|------------|-------------|
| **Multi-board support** | ✅ One config file | ❌ Manual switching |
| **Library management** | ✅ Automatic | ⚠️ Manual install |
| **Build speed** | ✅ Faster (ccache) | ⚠️ Slower |
| **IntelliSense** | ✅ Full support | ⚠️ Limited |
| **Version control** | ✅ Git-friendly | ⚠️ Less optimized |
| **Advanced debugging** | ✅ SWD/JTAG | ❌ Not supported |
| **Learning curve** | ⚠️ Steeper | ✅ Beginner-friendly |

**Recommendation**: Use PlatformIO for professional development, Arduino IDE for quick prototyping.

---

## ✅ Verification Checklist

After setup, verify everything works:

- [ ] PlatformIO installed and VSCode extension active
- [ ] Project opens without errors
- [ ] Both environments build successfully
- [ ] RP2040 uploads and LED blinks (3x startup)
- [ ] Arduino Uno uploads and LEDs blink (Pin 12, 13)
- [ ] Serial Monitor works for RP2040
- [ ] LIN communication works (Pin 12 toggles on Uno)

**If all checked**: You're ready to use PlatformIO! 🚀

---

**Created with Claude Code - 2026-02-12**
**For LIN Bus Steering Control System v2.0**
