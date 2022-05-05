#include "driver-interface.h"

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
            return 0;

        default:
            // Ignore rest.
            return 0;
    }
}
