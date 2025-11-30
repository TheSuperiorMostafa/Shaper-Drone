# ShaperDrone - Autonomous Shape Navigation System

Autonomous navigation system for a sub-250g drone that learns a sequence of shapes and navigates to them in order using computer vision and time-of-flight sensing.

## Hardware

- **Flight Controller:** Flywoo GOKU GN745 V3 AIO (ArduPilot firmware)
- **Computer:** Raspberry Pi Zero 2 W
- **Camera:** Raspberry Pi Camera Module
- **Sensor:** Time-of-Flight (ToF) sensor (VL53L0X or VL53L1X)

## Features

- **Learning Phase:** Detects and stores 3 shapes in sequence with debouncing
- **Automatic Takeoff:** Automatically takes off to safe altitude after shape learning
- **Gate Navigation:** Flies through shaped gates (not just to them)
- **Navigation Phase:** Autonomously navigates through gates in learned order
- **Visual Servoing:** Uses camera feedback for precise navigation
- **Distance Control:** ToF sensor provides accurate distance measurements
- **Motor Kill Switch:** Emergency stop immediately disarms motors (safety requirement)
- **Remote Manual Override:** Network interface for external emergency stop and control
- **Safety Features:** Timeouts, error recovery, automatic RTL

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

### Local Keyboard Controls
- **'q' or ESC:** Quit program
- **'r':** Reset shape order
- **'n':** Manually start navigation
- **'s':** Emergency stop (local)

### Remote Control (Network)
The system includes a remote control server on port 8888 for external manual override:
- **Emergency Stop:** `python3 remote_client.py <pi_ip> emergency_stop`
- **Status Check:** `python3 remote_client.py <pi_ip> status`
- **Telnet:** `telnet <pi_ip> 8888` then type `emergency_stop` or `status`

The remote emergency stop immediately disarms motors (motor kill switch) for safety.

## Project Structure

- `shapeVideo.py` - Main program with camera, shape detection, and navigation
- `navigation.py` - MAVLink flight controller communication (includes takeoff, emergency disarm)
- `shape_navigator.py` - Navigation state machine with gate navigation and visual servoing
- `tof_sensor.py` - Time-of-flight sensor interface
- `remote_control.py` - Remote control server for manual override and emergency stop
- `remote_client.py` - Client script for remote control commands
- `SETUP.md` - Complete setup guide

## License

[Add your license here]

