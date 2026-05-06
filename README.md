# Line Balancer

This project is an Arduino-based line/rod balancing system controlled with a PID algorithm.  
The system reads the position of the rod using an analog sensor, converts the raw sensor value into an estimated position, and adjusts a servo motor to keep the rod close to the selected target position.

## Features

- PID control using proportional, integral, and derivative terms
- Servo motor control through the Arduino `Servo` library
- Analog sensor reading with averaging for smoother measurements
- Low-pass filtering for noisy sensor values
- Calibration table for converting raw sensor readings into rod position
- Serial commands for changing target position and tuning PID parameters
- Soft output limits near the edges of the working range
- Debug output through the Serial Monitor

## Hardware

The project uses:

- Arduino-compatible board
- Servo motor
- Analog position sensor
- Balancing rod or line mechanism
- Jumper wires and power supply

## Pin Configuration

| Component | Arduino Pin |
|---|---|
| Servo motor | D9 |
| Analog sensor | A0 |

```cpp
#define MOTOR_PIN  9
#define SENSOR_PIN A0
