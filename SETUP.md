# ShaperDrone Setup Guide

## Hardware Components

- **Flight Controller:** Flywoo GOKU GN745 V3 AIO (ArduPilot firmware)
- **Computer:** Raspberry Pi Zero 2 W
- **Camera:** Raspberry Pi Camera Module
- **Sensor:** Time-of-Flight (ToF) sensor (VL53L0X or VL53L1X)

## Hardware Connections

### Flight Controller to Raspberry Pi

1. **UART Connection:**
   - Connect Raspberry Pi GPIO 14 (TX) → FC UART RX
   - Connect Raspberry Pi GPIO 15 (RX) → FC UART TX
   - Connect Ground (GND) → FC GND
   
2. **Power:**
   - Flywoo GOKU GN745 V3 has 5V/2A BEC output
   - Can power Raspberry Pi Zero 2 W (requires ~1A)
   - Connect FC 5V BEC → Pi 5V
   - Connect FC GND → Pi GND

### Camera Connection

- Connect Raspberry Pi Camera Module to Pi's camera connector (CSI)

### ToF Sensor Connection

- Connect ToF sensor to Raspberry Pi I2C bus:
  - VCC → Pi 3.3V
  - GND → Pi GND
  - SDA → Pi GPIO 2 (I2C SDA)
  - SCL → Pi GPIO 3 (I2C SCL)

## ArduPilot Configuration

### 1. Flash ArduPilot Firmware

- Use Mission Planner or QGroundControl to flash ArduPilot firmware
- Select appropriate frame type (Quad, Hex, etc.)

### 2. Configure Serial Port for MAVLink

In Mission Planner or via CLI:

```
SERIALx_PROTOCOL = 2    # MAVLink protocol
SERIALx_BAUD = 57       # 57600 baud
```

Where `x` is the UART port number you're using (typically 1, 2, or 3).

### 3. Enable GUIDED Mode

GUIDED mode should be available by default in ArduPilot. Verify in Mission Planner:
- Flight Modes → GUIDED should be enabled

### 4. Calibration

Complete standard ArduPilot calibration:
- Accelerometer calibration
- Compass calibration
- Radio calibration
- ESC calibration

## Raspberry Pi Setup

### 1. Enable Interfaces

```bash
sudo raspi-config
```

Enable:
- Camera
- I2C
- Serial Port (disable login shell, enable serial port)

### 2. Install Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python packages
pip3 install opencv-python numpy pymavlink picamera2

# For ToF sensor (VL53L0X)
pip3 install VL53L0X

# Or for VL53L1X
pip3 install VL53L1X
```

### 3. Serial Port Permissions

Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

Log out and back in for changes to take effect.

### 4. Test Serial Connection

```bash
# Check if serial port is available
ls -l /dev/ttyAMA0

# Test connection (optional)
python3 -c "from pymavlink import mavutil; conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600); conn.wait_heartbeat(); print('Connected!')"
```

## Software Configuration

### 1. Configure ToF Sensor Type

Edit `shapeVideo.py`, line ~210:
```python
tof = TOFSensor(sensor_type='VL53L0X')  # or 'VL53L1X'
```

### 2. Adjust Navigation Parameters (Optional)

Edit `shape_navigator.py` to adjust:
- `approach_distance`: Target distance from shapes (default: 2.0m)
- `max_velocity`: Maximum flight velocity (default: 1.0 m/s)
- `search_timeout`: Time to search for each shape (default: 30s)

## Running the System

### 1. Start the Program

```bash
python3 shapeVideo.py
```

### 2. Learning Phase

- Show the drone 3 shapes in sequence
- System will learn the order automatically
- Wait for "ORDER LOCKED" message

### 3. Navigation Phase

- Navigation starts automatically when order is complete
- Or press 'n' to start manually
- Drone will navigate to each shape in sequence

### 4. Controls

- **'q' or ESC:** Quit program
- **'r':** Reset shape order
- **'n':** Manually start navigation
- **'s':** Emergency stop

## Troubleshooting

### Flight Controller Not Connecting

1. Check UART wiring (TX→RX, RX→TX, GND)
2. Verify ArduPilot SERIALx_PROTOCOL = 2
3. Check baud rate matches (57600)
4. Verify serial port permissions
5. Try different UART ports on FC

### Camera Not Working

1. Verify camera is connected to CSI port
2. Enable camera in raspi-config
3. Test with: `libcamera-hello -t 0`

### ToF Sensor Not Detected

1. Check I2C wiring
2. Enable I2C in raspi-config
3. Scan for device: `i2cdetect -y 1`
4. Verify sensor library is installed

### Navigation Issues

1. Ensure shapes are well-lit and high contrast
2. Adjust shape detection thresholds in `shapeVideo.py`
3. Check ToF sensor is providing valid readings
4. Verify flight controller is in GUIDED mode

## Safety Notes

- Always test in a safe, open area
- Start with low velocities
- Have emergency stop ready
- Verify RTL (Return to Launch) works before autonomous flight
- Check battery levels before flight
- Follow local drone regulations

## Specifications Reference

**Flywoo GOKU GN745 V3 AIO:**
- Processor: STM32F745 @ 216MHz
- UART Ports: 7 available
- BEC: 5V/2A (can power Pi Zero 2 W)
- Firmware: ArduPilot compatible

**Raspberry Pi Zero 2 W:**
- Processor: BCM2710A1 (Quad-core 64-bit)
- GPIO UART: /dev/ttyAMA0 (GPIO 14/15)
- I2C: GPIO 2 (SDA), GPIO 3 (SCL)
- Camera: CSI connector

