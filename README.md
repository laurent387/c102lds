# c102lds

LDS Xiaomi C102 / X20 reverse engineering on ESP32-S3 with Arduino/PlatformIO.

This repository contains the tested firmware and notes needed to:

- drive the LDS motor with PWM on `GPIO14`
- read lidar data on `GPIO18`
- optionally transmit on `GPIO17`
- inspect the stream in HEX/ASCII with timestamps
- view a 2D map on the ESP webserver

## Current firmware

The main firmware lives in `src/main_s3.cpp` and has been flashed and tested on an ESP32-S3 DevKitC-1.

Observed working settings:

- protocol starts with `55 AA`
- motor behaves best with `freq=20000`
- `pwm 500` was a usable starting point on the bench

## Wiring

| ESP32-S3 DevKitC-1 | GPIO | LDS |
| --- | --- | --- |
| GPIO14 | PWM / MOT_EN | motor control |
| GPIO18 | RX | lidar TX to ESP |
| GPIO17 | TX optional | lidar RX from ESP |
| GND | GND | common ground |

## USB serial commands

```text
help
motor on
motor off
pwm 0-1023
freq 1000
freq 5000
freq 10000
freq 20000
invertpwm 0
invertpwm 1
baud 9600|19200|38400|57600|115200|230400
status
send $
send startlds$
send start
send stop
```

## Web server

- `/` or `/map` for the 2D view
- `/scan` for JSON scan data
- `/cmd?c=...` for commands

## Build

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## More notes

- `docs/pinout.md`
- `docs/protocol.md`
- `docs/notes.md`
