#ifndef T265_OPERATOR_UART_HPP
#define T265_OPERATOR_UART_HPP

#include "f407_protocol.hpp"
#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace omni {

// A small operator-only channel: one-shot buzzer feedback and manual ABORT.
// It cannot issue CONFIG, move commands, or replace the existing mission path.
constexpr std::uint8_t kOperatorBeepMessageType = 0x20;
bool validate_operator_frame(const std::uint8_t *data, std::size_t size);
bool is_operator_abort(const std::array<std::uint8_t, kFrameSize> &frame);

class OperatorUartRelay {
public:
    using Send = std::function<bool(const std::array<std::uint8_t, kFrameSize> &)>;
    OperatorUartRelay(const std::string &socket_name, Send send);
    ~OperatorUartRelay();
    OperatorUartRelay(const OperatorUartRelay &) = delete;
    OperatorUartRelay &operator=(const OperatorUartRelay &) = delete;
private:
    void run();
    int fd_ = -1;
    Send send_;
    std::atomic<bool> running_{true};
    std::thread thread_;
};

}  // namespace omni
#endif
