# Drone Shape Navigation

This C++ module provides the core functionality for an indoor drone's shape detection and navigation system.

## Features

- Shape detection
- Navigation to detected shapes
- Landing on designated pads
- Course execution with multiple shapes

## Files

- `DroneShapeNavigation.h` - Header file with class definition
- `DroneShapeNavigation.cpp` - Implementation of the DroneShapeNavigation class
- `main.cpp` - Example usage of the DroneShapeNavigation class

## Usage

```cpp
#include "DroneShapeNavigation.h"

DroneShapeNavigation drone;
drone.detectShape("circle");
drone.navigateToShape("circle");
drone.landOnPad();
```

## Compilation

```bash
g++ -o drone main.cpp DroneShapeNavigation.cpp
```

