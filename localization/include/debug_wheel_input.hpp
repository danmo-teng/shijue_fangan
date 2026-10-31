#pragma once

#include "config.hpp"
#include <librealsense2/rs.hpp>
#include <json.hpp>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

namespace omni {
// Opt-in debug ingress in the SAME SDK owner. No chassis UART or pose fusion.
class DebugWheelInput {
public:
    using Json = nlohmann::json;
    using Send = std::function<bool(std::uint32_t, const rs2_vector &)>;
    DebugWheelInput(const LocalizationConfig &, const std::string &session,
                   const std::string &address, const std::string &log_path);
    ~DebugWheelInput();
    static void validate_config(const Json &);
    void configure(const rs2::pose_sensor &, const std::string &config_path);
    void poll(std::uint64_t now_ns);
    // Also exercised by the existing regression executable with a fake SDK sink.
    bool process(Json packet, std::uint64_t now_ns, const Send &send);
private:
    void record(Json);
    LocalizationConfig config_;
    std::string session_, status_path_;
    std::ofstream log_;
    int socket_ = -1;
    std::unique_ptr<rs2::wheel_odometer> wheel_;
    std::uint64_t next_frame_ = 0, last_packet_ = 0, last_end_ = 0;
    bool have_packet_ = false, have_epoch_ = false, sdk_failed_ = false;
    Json epoch_;
    std::uint64_t accepted_ = 0, rejected_ = 0;
};
} // namespace omni
