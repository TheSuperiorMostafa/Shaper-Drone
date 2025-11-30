# ShaperDrone - Autonomous Shape Navigation System

Autonomous navigation system for a sub-250g drone that learns a sequence of shapes and navigates to them in order using computer vision and time-of-flight sensing.

## Hardware

- **Flight Controller:** Flywoo GOKU GN745 V3 AIO (ArduPilot firmware)
- **Computer:** Raspberry Pi Zero 2 W
- **Camera:** Raspberry Pi Camera Module
- **Sensor:** Time-of-Flight (ToF) sensor (VL53L0X or VL53L1X)

## Features

- **Learning Phase:** Detects and stores 3 shapes in sequence with debouncing
- **Navigation Phase:** Autonomously navigates to each shape in learned order
- **Visual Servoing:** Uses camera feedback for precise navigation
- **Distance Control:** ToF sensor provides accurate distance measurements
- **Safety Features:** Timeouts, error recovery, emergency stop, automatic RTL

## Installation

See [SETUP.md](SETUP.md) for complete hardware and software setup instructions.

### Quick Start

```bash
# Install dependencies
pip3 install opencv-python numpy pymavlink picamera2 VL53L0X

# Run the system
python3 shapeVideo.py
```

## Controls

- **'q' or ESC:** Quit program
- **'r':** Reset shape order
- **'n':** Manually start navigation
- **'s':** Emergency stop

## Project Structure

- `shapeVideo.py` - Main program with camera, shape detection, and navigation
- `navigation.py` - MAVLink flight controller communication
- `shape_navigator.py` - Navigation state machine and visual servoing
- `tof_sensor.py` - Time-of-flight sensor interface
- `SETUP.md` - Complete setup guide

## License

[Add your license here]

