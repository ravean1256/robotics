# Autonomous Mecanum Robot Navigation

Arduino Nano mecanum robot navigation prototype with occupancy-grid mapping, frontier exploration, and A* path planning.

## Files

- `MecanumNavigator.ino` - full Arduino sketch implementing sensor smoothing, sensor fusion, fixed-grid mapping, frontier selection, A* planning, and mecanum movement commands.
- `claude.md` - original project description and README contents copied for reference.

## Overview

This project is designed for an Arduino Nano-based mecanum robot. It assumes the low-level motor driver API exists externally and focuses on higher-level navigation logic.

The system reads ultrasonic and IR sensors, smooths measurements, fuses front obstacle distance, builds a small occupancy grid, selects a nearest frontier cell, plans a path using A*, and commands one motion step at a time.

## Hardware

- Arduino Nano
- Mecanum-wheel robot chassis
- Motor controller board with the following assumed motion functions:
  - `moveForward(int speed)`
  - `moveBackward(int speed)`
  - `strafeLeft(int speed)`
  - `strafeRight(int speed)`
  - `rotateLeft(int speed)`
  - `rotateRight(int speed)`
  - `stopMotors()`
- Ultrasonic distance sensor
- IR proximity/distance sensors
- Power supply / battery

## What to adjust

### Sensor pins

Update the pin definitions in `MecanumNavigator.ino` to match your wiring:

- `PIN_US_TRIG`
- `PIN_US_ECHO`
- `PIN_IR_FRONT`
- `PIN_IR_LEFT`
- `PIN_IR_RIGHT`

### Grid parameters

- `GRID_SIZE` - size of the occupancy grid. Keep this small (e.g. 10) for Arduino Nano RAM.
- `CELL_SIZE_CM` - physical distance per grid cell. Tune to the distance your robot travels per step.
- `MAX_SENSOR_RANGE_CM` - maximum detected distance for sensors.
- `COLLISION_MARGIN_CM` - distance threshold for marking obstacles.

### Movement timing

- `MOVE_DURATION_MS` - how long the robot drives for one grid cell.
- `STOP_DELAY_MS` - pause after stopping before the next loop.
- `TRAVEL_SPEED` - speed value passed to the movement functions.

### Debugging

- `DEBUG_SERIAL` - set to `1` to print grid and status messages over serial.
- The sketch includes `printGrid()` to display the occupancy map in text form.

### Fallback mode

- Set `USE_SIMPLE_FALLBACK` to `1` in `MecanumNavigator.ino` to disable full A* and use a lighter greedy frontier stepper.

## Why one file is okay

For a compact Arduino Nano prototype, keeping the sketch in one `.ino` file is acceptable and simplifies upload. If the project grows, you can split the code into headers and helper `.cpp` files for maintainability.

## Limitations

- This is not full SLAM or exact localisation.
- The robot uses an estimated grid position and assumes one cell per motion step.
- The occupancy grid is small to fit Nano RAM and is intended for short local exploration.

## Notes

- The sketch avoids dynamic allocation and STL containers.
- It uses fixed-size arrays for A* and mapping.
- It is intended to be practical and easy to paste into the Arduino IDE.
