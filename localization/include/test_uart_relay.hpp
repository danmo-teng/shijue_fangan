#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <mutex>

namespace omni {
// Local transport only: sole C++ UART owner, existing Python action pairing.
// No F405 ACK/health messages, motion generation, automatic STOP or replay.
class TestUartRelay {
public:
    using Write=std::function<bool(const std::uint8_t *,std::size_t)>;
    TestUartRelay(std::string address,Write);
    ~TestUartRelay();
    void publish(const std::uint8_t *,std::size_t,std::uint64_t receive_ns);
    static bool allowed(const std::uint8_t *,std::size_t);
private:
    void run();
    std::string address_;
    Write write_;
    int server_=-1,rx_server_=-1,rx_peer_=-1;
    std::mutex rx_mutex_;
    std::atomic<bool> stop_{false},connected_{false};
    std::thread thread_;
    std::uint64_t drops_=0;
};
} // namespace omni
