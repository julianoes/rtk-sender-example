# rtk-sender-example
Example that connects to a GPS and sends RTCM RTK data via MAVLink using MAVSDK.

## Prerequisites

1. Install [MAVSDK](https://github.com/mavlink/MAVSDK) on your system.
2. Install `cmake` and a compiler such as `GCC`.
3. Get the git submodule: `git submodule update --init --recursive`.

## Build

```
cmake -Bbuild -H.
cmake --build build
```

## Run

Connect GPS over serial, find the serial device and run the example against it:

```
build/rtk-sender-example /dev/ttyUSB0
```
