#pragma once
#include "config.hpp"
#include <json.hpp>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <string>

namespace omni {
// Same measured-interval / trapezoidal gyro calculation as the Python bridge.
// Runs in the SDK owner, with no Python/GIL, UART ownership or device calls.
class F405WheelMeasurements {
public:
    using Json = nlohmann::json;
    using Record = std::function<void(Json)>;
    F405WheelMeasurements(const LocalizationConfig &, std::string session, Record);
    void request(const std::uint8_t *, std::size_t, std::uint64_t written_before_ns);
    void feedback(const std::uint8_t *, std::size_t, std::uint64_t received_ns);
    Json latest(); // consume ready intervals once; coalesce, never replay a backlog
    // A new SDK session/source boundary discards pending motion, while retaining
    // matched identity/clock context; it never sends another MCU handshake.
    void discard_motion_support();
private:
    Json decode(const std::uint8_t *, std::size_t, std::uint64_t);
    Json prepare(const Json &, std::string &reason);
    LocalizationConfig config_;
    std::string session_;
    Record record_;
    Json identity_, sync_,motion_request_;
    std::map<std::uint64_t, Json> requests_;
    std::deque<Json> rates_, pending_;
    std::string wheel_epoch_, rate_epoch_;
    std::uint32_t wheel_id_=0, rate_id_=0;
    bool have_wheel_=false,have_rate_=false,have_end_=false;
    std::uint64_t last_end_=0,next_packet_=0;
};
} // namespace omni
