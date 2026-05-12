# Autonomous Mecanum Robot Navigation

Arduino Nano robotics project implementing autonomous navigation using ultrasonic and IR sensors, occupancy-grid mapping, frontier exploration, and A* pathfinding.

## Overview

This project controls a mecanum-wheel robot using a modular embedded C++ architecture. Low-level wheel movement functions are assumed to already exist, allowing the project to focus on navigation logic, sensor processing, mapping, and path planning.

The robot estimates its position on a simplified 2D grid, updates obstacle information from noisy distance sensors, selects unexplored frontier cells, and plans paths using A* search.

## Features

- Ultrasonic and IR sensor reading
- Sensor smoothing for noisy measurements
- Basic sensor fusion between distance sensors
- Fixed-size 2D occupancy grid
- Obstacle and free-space marking
- Frontier-cell detection for autonomous exploration
- A* pathfinding using Manhattan distance
- Modular Arduino C++ structure
- Designed for Arduino Nano memory constraints

## Hardware

- Arduino Nano
- Mecanum-wheel robot chassis
- Motor controller board
- Ultrasonic distance sensor(s)
- IR proximity/distance sensor(s)
- Battery supply

## Assumed Motor Functions

The low-level mecanum wheel control is assumed to already be implemented:

```cpp
moveForward(int speed);
moveBackward(int speed);
strafeLeft(int speed);
strafeRight(int speed);
rotateLeft(int speed);
rotateRight(int speed);
stopMotors();
```

## System Architecture

```text
Sensors
  ↓
Sensor Smoothing / Fusion
  ↓
Occupancy Grid Mapping
  ↓
Frontier Selection
  ↓
A* Path Planning
  ↓
Movement Controller
```

## Main Control Loop

```text
1. Read ultrasonic and IR sensors
2. Smooth and combine sensor values
3. Update the occupancy grid
4. Detect unexplored frontier cells
5. Select a target frontier
6. Plan a path using A*
7. Move one grid step
8. Stop and replan if an obstacle is detected
```

## Limitations

This is not full SLAM. The robot uses a simplified estimated grid position updated after each movement command. The aim is to build a clear, defensible embedded navigation system rather than a perfect robotics localisation stack.

## Future Improvements

- Add encoder-based odometry
- Add IMU heading correction
- Improve sensor calibration
- Visualise the occupancy grid over serial
- Benchmark different pathfinding heuristics
- Add probabilistic occupancy updates
