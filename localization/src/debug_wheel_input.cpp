#include "debug_wheel_input.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace omni {
namespace {
constexpr std::uint64_t max_age = 50000000, max_uncertainty = 5000000;
std::array<double, 3> vector3(const DebugWheelInput::Json &value)
{
    if (!value.is_array() || value.size() != 3) throw std::runtime_error("expected vector3");
    std::array<double, 3> out;
    for (int i = 0; i < 3; ++i) {
        out[i] = value.at(i).get<double>();
        if (!std::isfinite(out[i])) throw std::runtime_error("nonfinite vector3");
    }
    return out;
}
}

DebugWheelInput::DebugWheelInput(const LocalizationConfig &config, const std::string &session,
                               const std::string &address, const std::string &log_path)
    : config_(config), session_(session), log_(log_path, std::ios::app)
{
    if (!log_) throw std::runtime_error("cannot open SDK wheel log");
    status_path_ = log_path.substr(0, log_path.find_last_of('/')+1)+"wheel_sdk_status.json";
    // Abstract Unix socket: no filesystem socket or stale file to clean up.
    if (!address.empty()) {
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        if (address[0] != '@' || address.size() >= sizeof(addr.sun_path))
            throw std::runtime_error("debug wheel socket must be a short @abstract address");
        std::memcpy(addr.sun_path+1, address.data()+1, address.size()-1);
        socket_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket_ < 0) throw std::runtime_error("cannot create debug wheel socket");
        if (::bind(socket_, reinterpret_cast<sockaddr *>(&addr),
                   offsetof(sockaddr_un, sun_path)+address.size()) != 0) {
            ::close(socket_); socket_ = -1;
            throw std::runtime_error("cannot bind debug wheel socket (owner already running?)");
        }
    }
    record({{"event", "wheel_input_session"}, {"input_frame", "C_at_tracking_origin"},
            {"dependencies", {"measured_wheel_feedback", "primary_gyro_rate"}},
            {"accuracy_verified", false}});
}

DebugWheelInput::~DebugWheelInput() { if (socket_ >= 0) ::close(socket_); }

void DebugWheelInput::validate_config(const Json &config)
{
    const auto &sensors = config.at("velocimeters");
    if (!sensors.is_array() || sensors.size() != 1) throw std::runtime_error("exactly one C-frame velocity source required");
    const auto &s = sensors.at(0);
    const std::vector<double> identity{1,0,0,0,1,0,0,0,1};
    if (s.at("scale_and_alignment").get<std::vector<double>>() != identity)
        throw std::runtime_error("wheel scale/alignment must be identity for converted C input");
    for (const auto *key : {"T", "W"})
        for (double v : vector3(s.at("extrinsics").at(key)))
            if (v != 0) throw std::runtime_error("C input requires zero SDK T and W");
    const double noise = s.at("noise_variance").get<double>();
    if (!std::isfinite(noise) || noise <= 0) throw std::runtime_error("positive noise variance required");
}

void DebugWheelInput::record(Json event)
{
    event["log_monotonic_ns"] = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    event["session_id"] = session_;
    event["sdk_api_success_samples"] = accepted_;
    event["rejected_samples"] = rejected_;
    event["result_scope"] = "host_SDK_call_return_only; no_device_fusion_ACK";
    event["device_fusion_confirmed"] = false;
    event["wheel_dependency_present"] = accepted_ > 0;
    event["wheel_dependency_suspect"] = sdk_failed_;
    log_ << event.dump() << '\n';
    log_.flush();
    if (!log_) throw std::runtime_error("SDK wheel log write failed; refusing unlogged injection");
    const auto temp = status_path_+".tmp";
    std::ofstream status(temp);
    status << event.dump() << '\n';
    status.close();
    if (!status || std::rename(temp.c_str(), status_path_.c_str()) != 0)
        throw std::runtime_error("cannot save SDK wheel status");
}

void DebugWheelInput::configure(const rs2::pose_sensor &sensor, const std::string &path)
{
    std::ifstream file(path, std::ios::binary);
    const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(file), {}};
    if (!file || bytes.empty() || bytes.size() >= 10000)
        throw std::runtime_error("cannot read wheel input config or exceeds SDK calibration limit");
    const auto config = Json::parse(bytes.begin(), bytes.end());
    validate_config(config);
    wheel_.reset(new rs2::wheel_odometer(sensor));
    if (!*wheel_) throw std::runtime_error("SDK pose sensor has no wheel_odometer extension");
    bool loaded = false;
    try { loaded = wheel_->load_wheel_odometery_config(bytes); }
    catch (const std::exception &e) {
        record({{"event", "wheel_config_load"}, {"sdk_result", false}, {"error", e.what()}});
        throw;
    }
    record({{"event", "wheel_config_load"}, {"before_pipeline_start", true},
            {"sdk_result", loaded}, {"config", config}});
    if (!loaded) throw std::runtime_error("SDK rejected wheel config; debug session not started");
}

bool DebugWheelInput::process(Json p, std::uint64_t now, const Send &send)
{
    const auto began = std::chrono::steady_clock::now();
    std::string reason;
    std::uint64_t packet_id = 0, start = 0, end = 0;
    rs2_vector v{};
    try {
        if (p.at("schema") != 1 || p.at("session_id") != session_ ||
            p.at("source") != "measured_wheel_feedback" || p.at("capture_time_qualified") != true ||
            p.at("measurement_valid") != true)
            throw std::runtime_error("invalid_session_or_source");
        auto tick = [&](const char *name) {
            const auto &j = p.at(name);
            if (!j.is_number_integer() || j.get<double>() < 0) throw std::runtime_error("invalid_time_or_id");
            return j.get<std::uint64_t>();
        };
        packet_id = tick("packet_id"); start = tick("interval_start_ns"); end = tick("interval_end_ns");
        const auto uncertainty = tick("time_uncertainty_ns");
        const auto receive = tick("receive_monotonic_ns"), gyro_receive = tick("gyro_receive_ns");
        const auto gs = tick("gyro_start_ns"), ge = tick("gyro_end_ns");
        if (uncertainty > max_uncertainty || start >= end || end-start > max_age ||
            now < start || now-start > max_age-uncertainty ||
            receive < end || receive > now || gyro_receive < ge || gyro_receive > now ||
            gs > start || ge < end || ge > now || now-gs > max_age-uncertainty)
            throw std::runtime_error("stale_or_unaligned_input");
        if (have_packet_ && (packet_id <= last_packet_ || start < last_end_))
            throw std::runtime_error("duplicate_or_out_of_order");
        if (p.at("mcu_epoch").is_null()) throw std::runtime_error("unknown_epoch");
        if (have_epoch_ && p.at("mcu_epoch") != epoch_)
            throw std::runtime_error("epoch_changed_new_sdk_session_required");
        if (!have_epoch_) { epoch_ = p.at("mcu_epoch"); have_epoch_ = true; }
        const auto body = vector3(p.at("v_body_mps"));
        const auto omega = vector3(p.at("omega_body_rad_s"));
        const auto camera = vector3(p.at("v_camera_mps"));
        const auto lever = vector3(p.at("lever_body_mps"));
        if (std::hypot(body[0], body[1]) > 1.5 || std::abs(body[2]) > 1e-9 ||
            std::abs(omega[0])+std::abs(omega[1]) > 1e-9 || std::abs(omega[2]) > 8.)
            throw std::runtime_error("implausible_planar_velocity");
        const std::array<double,3> expected_lever{-omega[2]*config_.camera_offset_left_m,
                                                omega[2]*config_.camera_offset_forward_m, 0};
        const auto &f = config_.camera_robot_forward_axis;
        const auto &u = config_.camera_robot_up_axis;
        const std::array<double,3> l{u[1]*f[2]-u[2]*f[1], u[2]*f[0]-u[0]*f[2], u[0]*f[1]-u[1]*f[0]};
        for (int i = 0; i < 3; ++i) {
            const double expected = f[i]*(body[0]+expected_lever[0])+l[i]*(body[1]+expected_lever[1]);
            if (std::abs(camera[i]-expected) > 1e-6 || std::abs(lever[i]-expected_lever[i]) > 1e-6)
                throw std::runtime_error("mount_conversion_mismatch");
        }
        if (sdk_failed_) throw std::runtime_error("sdk_stream_failed_new_session_required");
        if (next_frame_ > 0xffffffffULL) throw std::runtime_error("frame_sequence_exhausted_new_session_required");
        v = {float(camera[0]), float(camera[1]), float(camera[2])};
    } catch (const std::exception &e) { reason = e.what(); }
    if (!reason.empty()) {
        ++rejected_;
        record({{"event", "wheel_input_rejected"}, {"reason", reason}, {"sdk_call_attempted", false},
                {"timestamp_monotonic_ns", now}, {"packet", p}});
        return false;
    }
    last_packet_ = packet_id; last_end_ = end; have_packet_ = true;
    const auto frame = std::uint32_t(next_frame_++); // SDK sequence, never a timestamp.
    record({{"event", "wheel_sdk_attempt"}, {"frame_num", frame}, {"sensor_id", 0},
            {"timestamp_monotonic_ns", now}, {"age_from_interval_start_ns", now-start}, {"packet", p}});
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now()-began).count();
    const auto oldest_source = std::min(start, p.at("gyro_start_ns").get<std::uint64_t>());
    if (now-oldest_source+elapsed > max_age-p.at("time_uncertainty_ns").get<std::uint64_t>()) {
        ++rejected_;
        record({{"event", "wheel_input_rejected"}, {"frame_num", frame},
                {"reason", "expired_before_sdk_call"}, {"sdk_call_attempted", false}});
        return false;
    }
    bool ok = false;
    std::string error;
    const auto call_start = std::chrono::steady_clock::now();
    try { ok = send(frame, v); } catch (const std::exception &e) { error = e.what(); }
    if (ok) ++accepted_; else { ++rejected_; sdk_failed_ = true; }
    record({{"event", "wheel_sdk_result"}, {"frame_num", frame}, {"sdk_result", ok},
            {"packet_id", packet_id}, {"error", error},
            {"sdk_duration_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now()-call_start).count()},
            {"timestamp_monotonic_ns", now}, {"accuracy_verified", false}});
    return ok;
}

void DebugWheelInput::poll(std::uint64_t now)
{
    // Bounded per iteration. Never block Pose or replay a stale backlog.
    for (int count = 0; count < 64; ++count) {
        char bytes[16384];
        const auto size = ::recv(socket_, bytes, sizeof(bytes), MSG_DONTWAIT | MSG_TRUNC);
        if (size < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;
            throw std::runtime_error("debug wheel IPC receive failed");
        }
        Json p;
        try {
            if (size >= static_cast<ssize_t>(sizeof(bytes))) throw std::runtime_error("oversized wheel packet");
            p = Json::parse(bytes, bytes+size);
        } catch (const std::exception &e) {
            ++rejected_;
            record({{"event", "wheel_input_malformed"}, {"error", e.what()}});
            continue;
        }
        const auto current = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        process(std::move(p), static_cast<std::uint64_t>(current), [&](std::uint32_t frame, const rs2_vector &velocity) {
            return wheel_->send_wheel_odometry(0, frame, velocity);
        });
    }
}
} // namespace omni
