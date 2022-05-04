#include <cstdio>
#include "PX4-GPSDrivers/src/gps_helper.h"
#include "serial-comms.h"


int main(int argc, char* argv[])
{
    if (argc != 3) {
        printf("\n");
        printf("usage: %s <serial device> <baudrate>\n", argv[0]);
        printf("e.g.: %s /dev/ttyUSB0 38400\n", argv[0]);
        return 1;
    }

    SerialComms serial_comms;
    if (!serial_comms.init(argv[1], std::stoi(argv[2]))) {
        return 2;
    }


    while (true) {
        if (serial_comms.read()) {
            printf("received: %zu\n", serial_comms.buffer().size());
        }
    }

    return 0;
}
