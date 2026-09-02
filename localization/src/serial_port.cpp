#include "serial_port.hpp"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace omni {
namespace {

speed_t baud_constant(int baud)
{
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
        default: throw std::runtime_error("unsupported UART baud: " + std::to_string(baud));
    }
}

}  // namespace

SerialPort::SerialPort(std::string path, int baud)
    : path_(std::move(path)), baud_(baud)
{
}

SerialPort::~SerialPort()
{
    close_port();
}

void SerialPort::open_port()
{
    close_port();
    descriptor_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor_ < 0) {
        throw std::runtime_error("open " + path_ + ": " + std::strerror(errno));
    }
    termios attributes{};
    if (tcgetattr(descriptor_, &attributes) != 0) {
        const std::string message = std::strerror(errno);
        close_port();
        throw std::runtime_error("tcgetattr " + path_ + ": " + message);
    }
    cfmakeraw(&attributes);
    const speed_t speed = baud_constant(baud_);
    cfsetispeed(&attributes, speed);
    cfsetospeed(&attributes, speed);
    attributes.c_cflag |= CLOCAL | CREAD;
    attributes.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    attributes.c_cflag |= CS8;
    attributes.c_cc[VMIN] = 0;
    attributes.c_cc[VTIME] = 0;
    if (tcsetattr(descriptor_, TCSANOW, &attributes) != 0) {
        const std::string message = std::strerror(errno);
        close_port();
        throw std::runtime_error("tcsetattr " + path_ + ": " + message);
    }
    tcflush(descriptor_, TCIFLUSH);
}

void SerialPort::close_port()
{
    if (descriptor_ >= 0) {
        ::close(descriptor_);
        descriptor_ = -1;
    }
}

int SerialPort::read_some(std::uint8_t *data, std::size_t capacity, int timeout_ms)
{
    if (descriptor_ < 0) return -1;
    pollfd descriptor{descriptor_, POLLIN, 0};
    const int status = ::poll(&descriptor, 1, timeout_ms);
    if (status == 0) return 0;
    if (status < 0) return errno == EINTR ? 0 : -1;
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) return -1;
    const ssize_t count = ::read(descriptor_, data, capacity);
    if (count < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    return static_cast<int>(count);
}

bool SerialPort::write_all(const std::uint8_t *data, std::size_t size, int timeout_ms)
{
    if (descriptor_ < 0 || (!data && size != 0)) return false;
    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (written < size) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) return false;
        pollfd descriptor{descriptor_, POLLOUT, 0};
        const int status = ::poll(&descriptor, 1, static_cast<int>(remaining));
        if (status < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (status == 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            return false;
        }
        const ssize_t count = ::write(descriptor_, data + written, size - written);
        if (count < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return false;
        }
        written += static_cast<std::size_t>(count);
    }
    return true;
}

}  // namespace omni
