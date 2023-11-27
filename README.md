# rtk-sender-example
Example that connects to a GPS and sends RTCM RTK data via MAVLink using MAVSDK.

## Prerequisites

1. Install [MAVSDK](https://github.com/mavlink/MAVSDK) on your system.
2. Install `cmake` and a compiler such as `GCC`.
3. Then cloning the rtk-sender-example
4. Then go inside the rtk-sender-example directory. 
5. Get the git submodule: `git submodule update --init --recursive`.

## Build

```
cmake -Bbuild -H.
cmake --build build
```

## Run

Connect GPS over serial, find the serial device, as well as baudrate.
Also find the MAVSDK connection URL to connect to the vehicle using MAVLink.

```
usage: build/rtk-sender-example <serial device> <baudrate> <mavlink connection>

e.g.: build/rtk-sender-example /dev/ttyUSB0 38400 udp://:14550

```
Note: use baudrate 0 to determine baudrate automatically
