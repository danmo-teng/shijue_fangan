#pragma once
#include "f407_protocol.hpp"
#include "async_latest_file.hpp"
#include <json.hpp>
#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace omni {
// Transport only: DynamicFence computes the pose. No second UART or estimator.
class DynamicPoseRelay {
public:
    using Json = nlohmann::json;
    using Send = std::function<bool(const std::array<std::uint8_t, kFrameSize> &)>;
    DynamicPoseRelay(const std::string &address, const std::string &session,
                     const std::string &output, Send send);
    ~DynamicPoseRelay();
    void start();
    void stop();
    bool accept(Json packet); // Existing regression target uses a fake UART sink.
    static FusedPoseFrame frame(const Json &packet, std::uint8_t sequence);
private:
    void run();
    int fd_ = -1;
    std::string session_, output_;
    Send send_;
    AsyncLatestFile snapshot_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
    bool have_packet_ = false;
    std::uint64_t packet_id_ = 0, tx_ok_ = 0, tx_error_ = 0;
    std::uint8_t sequence_ = 0;
};
}
