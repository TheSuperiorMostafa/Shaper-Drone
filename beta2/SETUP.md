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

### ToF Sensor Connections

**Forward-Facing ToF Sensor (for distance to shapes/gates):**
- Connect ToF sensor (VL53L0X) to Raspberry Pi I2C bus:
  - VCC → Pi 3.3V
  - GND → Pi GND
  - SDA → Pi GPIO 2 (I2C SDA)
  - SCL → Pi GPIO 3 (I2C SCL)
- Mount: Point forward, toward shapes/gates
- Purpose: Measures horizontal distance to shapes/gates

**Downward-Facing ToF Sensor (for altitude):**
- Connect second ToF sensor (VL53L0X) to Raspberry Pi I2C bus:
  - VCC → Pi 3.3V
  - GND → Pi GND
  - SDA → Pi GPIO 2 (I2C SDA) - can share same bus
  - SCL → Pi GPIO 3 (I2C SCL) - can share same bus
- **Important:** If using two VL53L0X sensors, you need to change one's I2C address
  - Default VL53L0X address: 0x29
  - Change one sensor's address to 0x30 (or another available address)
  - Or use one VL53L0X and one VL53L1X (they have different default addresses)
- Mount: Point downward, toward ground
- Purpose: Measures altitude (height above ground)

### Optical Flow Sensor Connection (MTF-02B)

- Connect MTF-02B to Raspberry Pi I2C bus:
  - VCC → Pi 3.3V (or 5V, check sensor specifications)
  - GND → Pi GND
  - SDA → Pi GPIO 2 (I2C SDA)
  - SCL → Pi GPIO 3 (I2C SCL)
- Default I2C address: 0x42
- Note: Both ToF and Optical Flow sensors can share the same I2C bus

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

# For Optical Flow sensor
# MTF-02B uses smbus (usually pre-installed, or install with: pip3 install smbus2)
# For PMW3901: pip3 install pmw3901
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

### 1. Configure Sensor Types

Edit `shapeVideo.py`, line ~237-248:
```python
# Forward-facing ToF for distance to shapes/gates
tof_forward = TOFSensor(sensor_type='VL53L0X', i2c_bus=1, address=None)

# Downward-facing ToF for altitude
tof_altitude = TOFSensor(sensor_type='VL53L0X', i2c_bus=1, address=None)
# Note: If both are VL53L0X, configure different I2C addresses
# Or use one VL53L0X and one VL53L1X
```

Edit `shapeVideo.py`, line ~242:
```python
optical_flow = OpticalFlowSensor(sensor_type='MTF-02B', interface='I2C', bus=1, device=0x42)
# MTF-02B is the default. Change to 'PMW3901', 'PX4Flow', or 'simulated' as needed
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

### Optical Flow Sensor (MTF-02B) Not Detected

1. **Check I2C wiring:**
   - Verify SDA and SCL connections
   - Ensure proper power (3.3V or 5V depending on sensor)
   - Check ground connection

2. **Enable I2C:**
   - Enable I2C in raspi-config
   - Reboot if necessary

3. **Verify sensor detection:**
   - Scan for device: `i2cdetect -y 1`
   - MTF-02B should appear at address 0x42
   - If not detected, check wiring and power

4. **Check sensor requirements:**
   - Ensure sensor has good lighting
   - Optical flow requires textured surface to work properly
   - Check sensor height (should be 0.5-4m for best results)
   - Sensor should be pointing downward

5. **For other sensors:**
   - **PMW3901 (SPI):** Check SPI wiring, enable SPI, verify library installed
   - **PX4Flow (I2C):** Similar to MTF-02B, check I2C address

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

