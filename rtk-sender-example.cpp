#include <cstdio>
#include <memory>

#include "PX4-GPSDrivers/src/gps_helper.h"
#include "PX4-GPSDrivers/src/ubx.h"
#include "serial-comms.h"
#include "driver-interface.h"


int main(int argc, char* argv[])
{
    if (argc != 3) {
        printf("\n");
        printf("usage: %s <serial device> <baudrate>\n", argv[0]);
        printf("e.g.: %s /dev/ttyUSB0 38400\n", argv[0]);
        printf("Note: use baudrate 0 to determine baudrate automatically\n");
        return 1;
    }

    unsigned baudrate = std::stoi(argv[2]);

    SerialComms serial_comms;
    if (!serial_comms.init(argv[1])) {
        return 2;
    }


    DriverInterface driver_interface(serial_comms);

    auto driver = std::make_unique<GPSDriverUBX>(
            GPSDriverUBX::Interface::UART,
            &DriverInterface::callback_entry, &driver_interface,
            &driver_interface.gps_pos, &driver_interface.sat_info);

    GPSHelper::GPSConfig gps_config {};
    gps_config.output_mode = GPSHelper::OutputMode::GPS; // for now
    //gps_config.output_mode = GPSHelper::OutputMode::RTCM; // eventually
    gps_config.gnss_systems = GPSHelper::GNSSSystemsMask::RECEIVER_DEFAULTS;

    if (driver->configure(baudrate, gps_config) != 0) {
        printf("configure failed\n");
        return 3;
    }

    printf("configure done!\n");


    while (true) {
        // Keep running, and don't stop on timeout.
        // Every now and then it timeouts but I'm not sure if that's actually
        // warranted given correct messages are still arriving.
        const unsigned timeout_ms = 5000;
        driver->receive(timeout_ms);
    }
    printf("timed out\n");

    return 4;
}
