#pragma once

#include <cstdint>
#include <vector>
#include <string>

class SerialComms {
public:
    SerialComms() = default;
    ~SerialComms();

    bool init(std::string path);
    bool set_baudrate(unsigned baudrate);

    ssize_t read(uint8_t* bytes, unsigned bytes_len);
    ssize_t write(const uint8_t* bytes, unsigned bytes_len);
    void reset();

    const std::vector<uint8_t>& buffer() const { return buffer_; }

private:
    static int define_from_baudrate(int baudrate);

    int fd_{-1};

    static constexpr unsigned len = 1024;
    std::vector<uint8_t> buffer_;
};
