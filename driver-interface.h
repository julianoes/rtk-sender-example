#pragma once

#include "definitions.h"
#include "serial-comms.h"
#include "gps_helper.h"

#include <memory>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/rtk/rtk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

class DriverInterface {
public:
    DriverInterface(SerialComms& serial_comms, mavsdk::Mavsdk& mavsdk) :
        serial_comms_(serial_comms),
        mavsdk_(mavsdk) {}

    static int callback_entry(GPSCallbackType type, void* data1, int data2, void* user);

    int callback(GPSCallbackType type, void* data1, int data2);

    void send_rtcm_data(const uint8_t* data, int data_len);

    struct sensor_gps_s        gps_pos;
	struct satellite_info_s    sat_info;

private:
    SerialComms& serial_comms_;
    mavsdk::Mavsdk& mavsdk_;
    std::shared_ptr<mavsdk::Rtk> rtk_plugin_{};
    std::shared_ptr<mavsdk::Telemetry> telemetry_plugin_{};
};
