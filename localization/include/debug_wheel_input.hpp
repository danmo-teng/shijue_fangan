#pragma once

#include "config.hpp"
#include "f405_wheel_measurements.hpp"
#include <librealsense2/rs.hpp>
#include <json.hpp>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>

namespace omni {
// Opt-in debug ingress in the SAME SDK owner. No chassis UART or pose fusion.
class DebugWheelInput {
public:
    using Json = nlohmann::json;
    using Send = std::function<bool(std::uint32_t, const rs2_vector &)>;
    DebugWheelInput(const LocalizationConfig &, const std::string &session,
                   const std::string &address, const std::string &log_path, bool synthetic_input=false);
    ~DebugWheelInput();
    static void validate_config(const Json &);
    void configure(const rs2::pose_sensor &, const std::string &config_path);
    void poll(std::uint64_t now_ns);
    void start();
    void stop();
    void use_native_feedback();
    void wire_feedback(const std::uint8_t *,std::size_t,std::uint64_t receive_ns);
    void wire_request(const std::uint8_t *,std::size_t,std::uint64_t before_ns);
    // Also exercised by the existing regression executable with a fake SDK sink.
    bool process(Json packet, std::uint64_t now_ns, const Send &send);
private:
    void record(Json);
    void write_logs();
    void wire_packet(const std::uint8_t *,std::size_t,std::uint64_t,bool request);
    void native_loop();
    LocalizationConfig config_;
    std::string session_, status_path_;
    std::ofstream log_;
    int socket_ = -1;
    std::unique_ptr<rs2::wheel_odometer> wheel_;
    std::uint64_t next_frame_ = 0, last_packet_ = 0, last_end_ = 0;
    bool have_packet_ = false, have_epoch_ = false, last_sdk_call_failed_ = false;
    Json epoch_;
    std::string alignment_clock_;
    std::uint64_t accepted_ = 0, rejected_ = 0, sdk_failed_samples_ = 0;
    bool config_loaded_=false;
    bool synthetic_input_=false;
    std::uint64_t last_success_ns_=0;
    std::atomic<bool> stopping_{false},logging_stopping_{false},logging_failed_{false};
    std::atomic<std::uint64_t> dropped_logs_{0};
    std::thread input_thread_,log_thread_;
    std::mutex log_mutex_;
    std::condition_variable log_ready_;
    std::deque<Json> records_;
    struct WirePacket {std::vector<std::uint8_t> data;std::uint64_t ns;bool request;};
    std::unique_ptr<F405WheelMeasurements> native_;
    std::mutex input_mutex_;
    std::condition_variable input_ready_;
    std::deque<WirePacket> input_;
    std::atomic<bool> started_{false};
    std::uint64_t input_drops_=0,prestart_feedback_=0;
};
} // namespace omni
