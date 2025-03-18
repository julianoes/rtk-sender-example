#include "driver-interface.h"
#include "gps_helper.h"
#include <iostream>
#include <string>
#include <mavsdk/base64.h>

int DriverInterface::callback_entry(GPSCallbackType type, void* data1, int data2, void* user)
{
    auto self = reinterpret_cast<DriverInterface*>(user);
    return self->callback(type, data1, data2);
}

int DriverInterface::callback(GPSCallbackType type, void* data1, int data2)
{
    switch (type) {
        case GPSCallbackType::readDeviceData:
            return serial_comms_.read(static_cast<uint8_t*>(data1), data2);

        case GPSCallbackType::writeDeviceData:
            return serial_comms_.write(static_cast<const uint8_t*>(data1), data2);

        case GPSCallbackType::setBaudrate:
            return (serial_comms_.set_baudrate(data2) ? 0 : 1);

        case GPSCallbackType::gotRTCMMessage:
            std::cout << "gotRTCM\n";
            send_rtcm_data(static_cast<const uint8_t*>(data1), data2);
            return 0;

        case GPSCallbackType::gotRelativePositionMessage:
            std::cout << "gotRelativePosition\n";
            return 0;

        case GPSCallbackType::surveyInStatus:
            {
            auto* status = reinterpret_cast<SurveyInStatus*>(data1);
            std::cout << "surveyInStatus: "<< std::to_string(status->flags) << ", accuracy: " << 1e-3 * (double)status->mean_accuracy << " m, duration: " << status->duration << " s\n";
            return 0;
            }


        default:
            // Ignore rest.
            return 0;
    }
}

void DriverInterface::send_rtcm_data(const uint8_t* data, int data_len)
{
    if (!rtk_plugin_) {
        if (mavsdk_.systems().empty()) {
            printf("No system available yet\n");
            return;
        }

        for (auto system : mavsdk_.systems()) {
            if (system->has_autopilot()) {
                rtk_plugin_ = std::make_shared<mavsdk::Rtk>(system);
                telemetry_plugin_ = std::make_shared<mavsdk::Telemetry>(system);
            }
        }
    }

    std::vector<uint8_t> data_as_vec(data, data + data_len);

    mavsdk::Rtk::RtcmData rtcm_data;
    rtcm_data.data_base64 = mavsdk::base64_encode(data_as_vec);
    rtk_plugin_->send_rtcm_data(rtcm_data);

    std::cout << "Fix type: " << telemetry_plugin_->gps_info().fix_type << '\n';
}
