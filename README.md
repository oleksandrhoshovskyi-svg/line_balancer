# Line Balancer

Arduino PID control project for balancing a rod/line mechanism using a servo motor and an analog position sensor.

The system reads the current position from the sensor, converts the raw analog value into an estimated rod position, and adjusts the servo angle to move the system toward the selected target position.

## Main Features

- PID-based position control
- Servo control with configurable neutral angle
- Analog sensor averaging and filtering
- Raw sensor value to position conversion using calibration points
- Runtime PID tuning through Serial Monitor
- Target position control through Serial commands
- Soft output limits near the mechanical edges
- Debug output for monitoring sensor, position, target, and PID values

## Hardware Pins

| Component | Pin |
|---|---|
| Servo motor | D9 |
| Analog sensor | A0 |

## Default Settings

| Parameter | Value |
|---|---:|
| Servo neutral position | 90 |
| Servo minimum angle | 65 |
| Servo maximum angle | 115 |
| Target range | -14 to 14 |
| PID period | 20 ms |
| Debug period | 200 ms |
| Serial baud rate | 9600 |

## Default PID Values

| Parameter | Value |
|---|---:|
| Kp | 1.3 |
| Ki | 0.15 |
| Kd | 0.35 |

## Calibration Table

| Rod position | Raw sensor value |
|---:|---:|
| -14 | 388 |
| -7 | 278 |
| 0 | 220 |
| 7 | 181 |
| 14 | 159 |

## Serial Commands

| Command | Description |
|---|---|
| `STOP` | Disable PID control and return servo to neutral |
| `RUN` | Enable PID control |
| `T10` | Set target position to `10` |
| `10` | Set target position to `10` |
| `-7` | Set target position to `-7` |
| `KP1.3` | Set proportional gain |
| `KI0.15` | Set integral gain and reset integral |
| `KD0.35` | Set derivative gain |
| `M090` | Set neutral servo angle to `90` |
| `DIR-1` | Set control direction to `-1` |
| `DIR1` | Set control direction to `1` |

## Example Serial Input

```text
T10
KP1.4
KI0.12
KD0.3
RUN
```

## Debug Output

The program prints debug information every 200 ms:

```text
EN RAW:220.5 POS:0.02 T:10.00 dir:-1 lim:30 servo0:90 Kp:1.30 Ki:0.150 Kd:0.35
```

Displayed values include:

- control state
- filtered raw sensor value
- estimated rod position
- target position
- control direction
- current soft limit
- neutral servo angle
- PID coefficients

## Project Structure

```text
line_balancer.ino
README.md
```

## Notes

The calibration values, PID coefficients, servo limits, and control direction are setup-specific and may need adjustment for a different physical build.

If the servo reacts in the wrong direction, change the direction using `DIR1` or `DIR-1`.

If the balanced neutral point is not correct, adjust the servo center using `M0`, for example `M090`.

This repository was imported from my previous GitHub account as part of organizing my projects in one place.
