#pragma once

#include "definitions.h"
#include "serial-comms.h"
#include "gps_helper.h"

class DriverInterface {
public:
    DriverInterface(SerialComms& serial_comms) :
        serial_comms_(serial_comms) {}

    static int callback_entry(GPSCallbackType type, void* data1, int data2, void* user);

    int callback(GPSCallbackType type, void* data1, int data2);

    struct sensor_gps_s        gps_pos;
	struct satellite_info_s    sat_info;

private:
    SerialComms& serial_comms_;
};
