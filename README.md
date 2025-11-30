# ShaperDrone - Autonomous Shape Navigation System

Software repository for the ShaperDrone Engineering Team's autonomous drone project.

## Repository Structure

### Beta 1 (`/beta1`)
Initial implementation with C++ shape detection and navigation modules.
- C++ navigation classes
- Xcode project files
- Drone simulation examples

### Beta 2 (`/beta2`)
Complete Python-based navigation system with full hardware integration.
- MAVLink integration for Flywoo GOKU GN745 V3 AIO flight controller
- Raspberry Pi camera support
- ToF sensor integration
- Autonomous navigation with visual servoing
- Complete setup documentation

## Quick Start

### Beta 2 (Recommended)
```bash
cd beta2
pip3 install opencv-python numpy pymavlink picamera2 VL53L0X
python3 shapeVideo.py
```

See `beta2/SETUP.md` for complete installation and hardware setup instructions.

## Project Overview

The ShaperDrone system enables autonomous navigation through a sequence of shapes. The drone learns a sequence of 3 shapes and then autonomously navigates to each one in order using computer vision and time-of-flight sensing.

## Team

ShaperDrone Engineering Team

