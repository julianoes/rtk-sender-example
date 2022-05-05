#include "serial-comms.h"
#include <cstdio>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

SerialComms::~SerialComms()
{
    reset();
}

bool SerialComms::init(std::string path) 
{
    // open() sometimes hangs on macOS or Linux devices unless you give it O_NONBLOCK.
    fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ == -1) {
        printf("open failed: %s\n", strerror(errno));
        return false;
    }
    // We need to clear the O_NONBLOCK again.
    if (fcntl(fd_, F_SETFL, 0) == -1) {
        printf("open failed: %s\n", strerror(errno));
        return false;
    }

    return true;
}

bool SerialComms::set_baudrate(unsigned baudrate)
{
    const int baudrate_define = define_from_baudrate(baudrate);

    struct termios tc {};

    if (tcgetattr(fd_, &tc) != 0) {
        printf("tcgetattr failed: %s\n", strerror(errno));
        reset();
        return false;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    tc.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
    tc.c_cflag |= CS8;

    tc.c_cc[VMIN] = 0; // Let's wait for 10 bytes.
    tc.c_cc[VTIME] = 10; // Timeout after 1 second.

    tc.c_cflag |= CLOCAL; // Without this a write() blocks indefinitely.

    if (cfsetispeed(&tc, baudrate_define) != 0) {
        printf("cfsetispeed failed: %s\n", strerror(errno));
        close(fd_);
        return false;
    }

    if (cfsetospeed(&tc, baudrate_define) != 0) {
        printf("cfsetospeed failed: %s\n", strerror(errno));
        close(fd_);
        return false;
    }

    if (tcsetattr(fd_, TCSANOW, &tc) != 0) {
        printf("tcsetattr failed: %s\n", strerror(errno));
        close(fd_);
        return false;
    }

    printf("set baudrate to %u\n", baudrate);

    return true;
}

ssize_t SerialComms::read(uint8_t* bytes, unsigned bytes_len)
{
    ssize_t result = ::read(fd_, bytes, bytes_len);
    if (result >= 0) {
        printf("read %zi bytes\n", result);
    } else {
        printf("read failed: %s\n", strerror(errno));
    }
    return result;
}

ssize_t SerialComms::write(const uint8_t* bytes, unsigned bytes_len)
{
    ssize_t result = ::write(fd_, bytes, bytes_len);
    if (result >= 0) {
        printf("wrote %zi bytes\n", result);
    } else {
        printf("write failed: %s\n", strerror(errno));
    }
    return result;
}

void SerialComms::reset()
{
    if (fd_ > 0) {
        close(fd_);
        fd_ = -1;
    }
    buffer_.clear();
}

int SerialComms::define_from_baudrate(int baudrate)
{
    switch (baudrate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default: {
            printf("Unknown baudrate\n");
            return -1;
        }
    }
}

