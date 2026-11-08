#include "f405_telemetry.hpp"
#include "f407_protocol.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace omni {
F405MixedFrames::F405MixedFrames(Callback legacy, Callback extended,bool all_legacy)
    : legacy_(std::move(legacy)), extended_(std::move(extended)),all_legacy_(all_legacy) {}

void F405MixedFrames::feed(const std::uint8_t *data, std::size_t size)
{
    // Incremental bounded parsing, without expiring incomplete packets by age.
    for (std::size_t i = 0; i < size; ++i) {
        buffer_.push_back(data[i]);
        while (buffer_.size() >= 2) {
            const bool legacy = buffer_[0] == 0xa3 && buffer_[1] == 0xb3;
            const bool extended = buffer_[0] == 0xd4 && buffer_[1] == 0x4d;
            if (!legacy && !extended) { buffer_.erase(buffer_.begin()); continue; }
            if (legacy && buffer_.size() >= 4 && buffer_[2] == 1 && buffer_[3] == 0xc3) {
                buffer_.erase(buffer_.begin(), buffer_.begin()+4); continue;
            }
            std::size_t length = 15;
            if (extended) {
                if (buffer_.size() < 8) break;
                length = (static_cast<std::size_t>(buffer_[4]) << 8) | buffer_[5];
                if (buffer_[2] != 1 || length > 160) { buffer_.erase(buffer_.begin()); continue; }
                length += 11;
            }
            if (buffer_.size() < length) break;
            const auto crc = modbus_crc16(buffer_.data()+2, length-5);
            const auto wire_crc = buffer_[length-3] | (buffer_[length-2] << 8);
            if (buffer_[length-1] != (legacy ? 0xc3 : 0xe4) || crc != wire_crc) {
                buffer_.erase(buffer_.begin()); continue;
            }
            if (extended) extended_(buffer_.data(), length);
            else if (all_legacy_ || buffer_[2] == kOdomMessageType || buffer_[2] == kStmStatusMessageType ||
                     buffer_[2] == kStatusContextMessageType) legacy_(buffer_.data(), length);
            buffer_.erase(buffer_.begin(), buffer_.begin()+length);
        }
    }
}

WheelTelemetryTap::WheelTelemetryTap(const std::string &address) : address_(address)
{
    if (address.empty()) return;
    if (address[0] != '@' || address.size() >= sizeof(sockaddr_un{}.sun_path))
        throw std::invalid_argument("wheel feedback tap needs a short @abstract address");
    fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    // Missing consumer/socket is not a reason to stop the competition UART.
    if(fd_<0)return;
    sockaddr_un target{};target.sun_family=AF_UNIX;
    std::memcpy(target.sun_path+1,address.data()+1,address.size()-1);
    const int budget=262144;::setsockopt(fd_,SOL_SOCKET,SO_SNDBUF,&budget,sizeof(budget));
    if(::connect(fd_,reinterpret_cast<sockaddr *>(&target),offsetof(sockaddr_un,sun_path)+address.size())!=0) {
        ::close(fd_);fd_=-1;
    }
}

WheelTelemetryTap::~WheelTelemetryTap() { if (fd_ >= 0) ::close(fd_); }

void WheelTelemetryTap::publish(const std::uint8_t *data, std::size_t size, std::uint64_t received_ns)
{
    if (fd_ < 0 || size > 171) return;
    std::uint8_t message[179];
    for (int i = 0; i < 8; ++i) message[i] = static_cast<std::uint8_t>(received_ns >> ((7-i)*8));
    std::memcpy(message+8, data, size);
    // Whole datagram or drop; no replay, growing queue, ACK or retry loop.
    (void)::send(fd_,message,size+8,MSG_DONTWAIT|MSG_NOSIGNAL);
}
} // namespace omni
