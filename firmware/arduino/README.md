# Arduino Firmware

Main controller firmware for the 12-DOF quadruped robot.

## Quick Start

1. Install Arduino IDE and required libraries
2. Open `main/main.ino`
3. Select Board: Arduino UNO R4
4. Upload to Arduino
5. Open Serial Monitor (115200 baud)

## Required Libraries

- Adafruit PWM Servo Driver Library
- Wire (built-in)

## Features

- 50Hz servo control loop
- Inverse kinematics solver
- Multiple gait patterns (walk, trot, crawl)
- Serial command interface
- Smooth servo interpolation
- Safety limits

## Serial Commands

Send commands via Serial Monitor:
```
GAIT:walk       - Set walk gait
SPEED:1.5       - Set speed
MOVE:50,0       - Move forward
STOP            - Emergency stop
```

## Pin Connections

- SDA: A4 (Arduino) → SDA (PCA9685)
- SCL: A5 (Arduino) → SCL (PCA9685)
- TX: D1 (Arduino) → RX (ESP32)
- RX: D0 (Arduino) → TX (ESP32)
