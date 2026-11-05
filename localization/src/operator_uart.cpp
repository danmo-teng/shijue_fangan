#include "operator_uart.hpp"

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace omni {

bool validate_operator_frame(const std::uint8_t *data, std::size_t size)
{
    if (size != kFrameSize || data[0] != kFrameHead1 || data[1] != kFrameHead2 ||
        data[14] != kFrameTail) return false;
    const auto crc = modbus_crc16(data + 2, 10);
    if (data[12] != (crc & 0xff) || data[13] != (crc >> 8)) return false;
    if (data[2] == kOperatorBeepMessageType) {
        if (data[4] != 1 || data[5] < 1 || data[5] > 4 || data[6] > 1) return false;
        for (int i = 7; i < 12; ++i) if (data[i] != 0) return false;
        return true;
    }
    if (data[2] == kMissionCommandMessageType && data[4] == 7 && data[5] == 1) {
        for (int i = 6; i < 12; ++i) if (data[i] != 0) return false;
        return true;
    }
    return false;
}

bool is_operator_abort(const std::array<std::uint8_t, kFrameSize> &frame)
{
    return frame[2] == kMissionCommandMessageType && frame[4] == 7;
}

OperatorUartRelay::OperatorUartRelay(const std::string &name, Send send)
    : send_(std::move(send))
{
    sockaddr_un address{};
    if (name.size() < 2 || name[0] != '@' || name.size() >= sizeof(address.sun_path))
        throw std::invalid_argument("operator socket requires a short @abstract name");
    fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0) throw std::runtime_error("operator socket: " + std::string(std::strerror(errno)));
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path + 1, name.data() + 1, name.size() - 1);
    const auto length = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + name.size());
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), length) != 0) {
        const std::string message = std::strerror(errno);
        ::close(fd_); fd_ = -1;
        throw std::runtime_error("operator socket bind: " + message);
    }
    try { thread_ = std::thread(&OperatorUartRelay::run, this); }
    catch (...) { ::close(fd_); fd_ = -1; throw; }
    std::cerr << "[OPERATOR] socket=" << name << " ready; beep/manual ABORT only\n";
}

OperatorUartRelay::~OperatorUartRelay()
{
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) ::close(fd_);
}

void OperatorUartRelay::run()
{
    while (running_.load()) {
        pollfd item{fd_, POLLIN, 0};
        const int ready = ::poll(&item, 1, 50);
        if (ready <= 0) continue;
        std::array<std::uint8_t, kFrameSize> frame{};
        // MSG_TRUNC makes oversized requests fail size validation, not truncate
        // into a potentially valid command. No retries or periodic heartbeat.
        const auto count = ::recv(fd_, frame.data(), frame.size(), MSG_TRUNC);
        if (count < 0) continue;
        if (!validate_operator_frame(frame.data(), static_cast<std::size_t>(count))) {
            std::cerr << "[OPERATOR] malformed or unsupported request ignored\n";
            continue;
        }
        try {
            const bool sent = send_(frame);
            std::cerr << "[OPERATOR] " << (is_operator_abort(frame) ? "manual ABORT" : "beep")
                      << " uart_write=" << (sent ? "true" : "false")
                      << " (no firmware acknowledgement)\n";
        } catch (const std::exception &error) {
            std::cerr << "[OPERATOR] write failed: " << error.what() << '\n';
        }
    }
}

}  // namespace omni
