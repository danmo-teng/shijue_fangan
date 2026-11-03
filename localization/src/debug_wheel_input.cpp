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
    log_thread_=std::thread([this]{write_logs();});
    record({{"event", "wheel_input_session"}, {"input_frame", "C_at_tracking_origin"},
            {"dependencies", {"measured_wheel_feedback", "primary_gyro_rate"}},
            {"accuracy_verified", false}});
}

DebugWheelInput::~DebugWheelInput()
{
    stop();logging_stopping_=true;log_ready_.notify_one();
    if(log_thread_.joinable())log_thread_.join();
    if (socket_ >= 0) ::close(socket_);
}

void DebugWheelInput::start()
{
    input_thread_=std::thread([this]{
        while(!stopping_) {
            try { poll(0); }
            catch(const std::exception &e) {
                logging_failed_=true;
                record({{"event","wheel_input_worker_failed"},{"error",e.what()}});
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
}

void DebugWheelInput::stop()
{
    stopping_=true;if(input_thread_.joinable())input_thread_.join();
}

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
    event["config_loaded"]=config_loaded_;
    event["last_sdk_success_ns"]=last_success_ns_;
    std::lock_guard<std::mutex> lock(log_mutex_);
    if(records_.size()>=4096) {logging_failed_=true;++dropped_logs_;return;}
    records_.push_back(std::move(event));log_ready_.notify_one();
}

void DebugWheelInput::write_logs()
{
    Json latest;
    std::deque<std::uint64_t> success_times;
    auto last_write=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    while(true) {
        std::deque<Json> batch;
        bool ending=false;
        {
            std::unique_lock<std::mutex> lock(log_mutex_);
            log_ready_.wait_for(lock,std::chrono::milliseconds(50),[&]{return !records_.empty()||logging_stopping_;});
            batch.swap(records_);
            ending=batch.empty()&&logging_stopping_;
        }
        for(const auto &event:batch) {
            log_<<event.dump()<<'\n';latest=event;
            if(event.value("event",std::string())=="wheel_sdk_result" && event.value("sdk_result",false))
                success_times.push_back(event["last_sdk_success_ns"].get<std::uint64_t>());
        }
        const auto now=std::chrono::steady_clock::now();
        if(!log_)logging_failed_=true;
        if(!latest.is_null() && (now-last_write>=std::chrono::milliseconds(100)||logging_stopping_)) {
            const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            while(!success_times.empty() && ns-success_times.front()>1000000000)success_times.pop_front();
            latest["sdk_api_rate_hz"]=success_times.size(); // Actual calls in the trailing 1-second window.
            latest["status_monotonic_ns"]=ns;
            latest["logging_healthy"]=!logging_failed_;
            latest["dropped_log_records"]=dropped_logs_.load();
            const auto temp=status_path_+".tmp";
            std::ofstream status(temp);status<<latest.dump()<<'\n';status.close();
            if(!status||std::rename(temp.c_str(),status_path_.c_str())!=0)logging_failed_=true;
            log_.flush();last_write=now;
        }
        if(ending)break;
    }
    if(dropped_logs_)log_<<Json({{"event","logging_incomplete"},{"dropped_records",dropped_logs_.load()}}).dump()<<'\n';
    log_.flush();
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
    config_loaded_=loaded;
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
        if (p.at("schema") != 2 || p.at("session_id") != session_ ||
            p.at("time_policy") != "received_feedback_no_age_gate_v1" ||
            p.at("source") != "measured_wheel_feedback" || !p.at("capture_time_qualified").is_boolean() ||
            p.at("measurement_valid") != true)
            throw std::runtime_error("invalid_session_or_source");
        auto tick = [&](const char *name) {
            const auto &j = p.at(name);
            if (!j.is_number_integer() || j.get<double>() < 0) throw std::runtime_error("invalid_time_or_id");
            return j.get<std::uint64_t>();
        };
        packet_id = tick("packet_id"); start = tick("alignment_start_ns"); end = tick("alignment_end_ns");
        tick("receive_monotonic_ns"); tick("gyro_receive_ns");
        const auto gs = tick("alignment_gyro_start_ns"), ge = tick("alignment_gyro_end_ns");
        const auto clock=p.at("alignment_clock").get<std::string>();
        if ((clock!="mcu_event" && clock!="host_monotonic_event") ||
            (have_packet_ && clock!=alignment_clock_) || start>=end || gs>start || ge<end)
            throw std::runtime_error("invalid_alignment_interval");
        // Unknown capture/drift bounds and sample age are diagnostic only.
        // These ticks describe one source interval, not an SDK timestamp.
        if (have_packet_ && (packet_id <= last_packet_ || start < last_end_))
            throw std::runtime_error("duplicate_or_out_of_order");
        if (p.at("mcu_epoch").is_null()) throw std::runtime_error("unknown_epoch");
        if (have_epoch_ && p.at("mcu_epoch") != epoch_)
            throw std::runtime_error("epoch_changed_new_sdk_session_required");
        if (!have_epoch_) { epoch_ = p.at("mcu_epoch"); have_epoch_ = true; }
        alignment_clock_=clock;
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
        if (logging_failed_) throw std::runtime_error("logging_incomplete_not_injected");
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
            {"timestamp_monotonic_ns", now},
            {"age_from_wheel_receive_ns",static_cast<std::int64_t>(now)-p.at("receive_monotonic_ns").get<std::int64_t>()},
            {"time_policy","received_feedback_no_age_gate_v1"},{"packet", p}});
    if (logging_failed_) {
        ++rejected_;
        record({{"event", "wheel_input_rejected"}, {"frame_num", frame},
                {"reason", "logging_incomplete_not_injected"}, {"sdk_call_attempted", false}});
        return false;
    }
    bool ok = false;
    std::string error;
    const auto call_start = std::chrono::steady_clock::now();
    try { ok = send(frame, v); } catch (const std::exception &e) { error = e.what(); }
    if (ok) {++accepted_;last_success_ns_=now+std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now()-began).count();}
    else { ++rejected_; sdk_failed_ = true; }
    record({{"event", "wheel_sdk_result"}, {"frame_num", frame}, {"sdk_result", ok},
            {"packet_id", packet_id}, {"error", error},
            {"sdk_duration_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now()-call_start).count()},
            {"timestamp_monotonic_ns", now}, {"accuracy_verified", false}});
    return ok;
}

void DebugWheelInput::poll(std::uint64_t now)
{
    // Bounded per iteration. No age gate and no duplicate source injection.
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
