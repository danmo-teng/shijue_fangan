#include <librealsense2/rs.hpp>

#include "config.hpp"
#include "f407_protocol.hpp"
#include "fusion.hpp"
#include "serial_port.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void signal_handler(int)
{
    g_stop = 1;
}

struct Options {
    std::string config_path = "config/localization.example.conf";
    std::string uart_path;
    std::string serial;
    std::string output_path = "localization_result.json";
    std::string csv_path;
    std::string command_file_path;
    std::string stm_status_output_path;
    int baud = 115200;
    double output_rate_hz = 20.0;
    double tx_rate_hz = 0.0;
    double duration_sec = 0.0;
    bool debug_sdk = false;
    bool ignore_encoders = false;
};

struct TimedEncoderFrame {
    omni::EncoderFrame frame;
    std::chrono::steady_clock::time_point received;
};

struct WheelDebugState {
    bool have_latest_frame = false;
    omni::EncoderFrame latest_frame{};
    std::uint64_t latest_frame_ns = 0;
    std::uint64_t latest_update_ns = 0;
    bool updated_this_pose = false;
    bool accepted_this_pose = false;
    bool rejected_this_pose = false;
    bool have_latest_increment = false;
    bool latest_increment_accepted = false;
    omni::WheelIncrement latest_increment{};
    std::uint64_t odom_updates = 0;
};

class EncoderQueue {
public:
    void push(const omni::EncoderFrame &frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frames_.size() >= 512) {
            frames_.pop_front();
            ++dropped_;
        }
        frames_.push_back({frame, std::chrono::steady_clock::now()});
    }

    std::deque<TimedEncoderFrame> drain()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::deque<TimedEncoderFrame> result;
        result.swap(frames_);
        return result;
    }

    std::uint64_t dropped() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

private:
    mutable std::mutex mutex_;
    std::deque<TimedEncoderFrame> frames_;
    std::uint64_t dropped_ = 0;
};

double parse_nonnegative(const std::string &text, const char *name)
{
    try {
        std::size_t used = 0;
        const double result = std::stod(text, &used);
        if (used != text.size() || result < 0.0) throw std::invalid_argument("range");
        return result;
    } catch (const std::exception &) {
        throw std::invalid_argument(std::string("invalid value for ") + name + ": " + text);
    }
}

int parse_positive_int(const std::string &text, const char *name)
{
    const double value = parse_nonnegative(text, name);
    const int result = static_cast<int>(value);
    if (result <= 0 || static_cast<double>(result) != value) {
        throw std::invalid_argument(std::string("invalid value for ") + name + ": " + text);
    }
    return result;
}

void usage(const char *program)
{
    std::cout
        << "T265-primary + three-wheel omni encoder localization\n\n"
        << "Usage: " << program << " [options]\n"
        << "  --config FILE       localization config\n"
        << "  --uart DEVICE       F407 UART, e.g. /dev/ttyS3 (optional)\n"
        << "  --baud BAUD         UART baud, default 115200\n"
        << "  --serial SERIAL     select T265 serial\n"
        << "  --output FILE       atomic JSON output\n"
        << "  --csv FILE          full-rate T265/odometry diagnostic CSV log\n"
        << "  --command-file FILE relay new valid TYPE 0x11/0x12/0x18 frames\n"
        << "  --stm-status FILE   atomic TYPE 0x17 status JSON output\n"
        << "  --rate HZ           stdout/JSON rate, default 20\n"
        << "  --tx-rate HZ        legacy fused-pose UART rate (default 0/off)\n"
        << "  --ignore-encoders   keep UART/task active but do not fuse wheel odometry\n"
        << "  --duration SEC      0 runs until Ctrl-C\n"
        << "  --debug-sdk         detailed librealsense log\n"
        << "  -h, --help          show help\n";
}

Options parse_options(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        auto value = [&](const char *name) -> std::string {
            if (++i >= argc) throw std::invalid_argument(std::string("missing value for ") + name);
            return argv[i];
        };
        if (argument == "-h" || argument == "--help") {
            usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else if (argument == "--config") {
            options.config_path = value("--config");
        } else if (argument == "--uart") {
            options.uart_path = value("--uart");
        } else if (argument == "--baud") {
            options.baud = parse_positive_int(value("--baud"), "--baud");
        } else if (argument == "--serial") {
            options.serial = value("--serial");
        } else if (argument == "--output") {
            options.output_path = value("--output");
        } else if (argument == "--csv") {
            options.csv_path = value("--csv");
        } else if (argument == "--command-file") {
            options.command_file_path = value("--command-file");
        } else if (argument == "--stm-status") {
            options.stm_status_output_path = value("--stm-status");
        } else if (argument == "--rate") {
            options.output_rate_hz = parse_nonnegative(value("--rate"), "--rate");
        } else if (argument == "--tx-rate") {
            options.tx_rate_hz = parse_nonnegative(value("--tx-rate"), "--tx-rate");
        } else if (argument == "--duration") {
            options.duration_sec = parse_nonnegative(value("--duration"), "--duration");
        } else if (argument == "--debug-sdk") {
            options.debug_sdk = true;
        } else if (argument == "--ignore-encoders") {
            options.ignore_encoders = true;
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    return options;
}

std::string device_info(const rs2::device &device, rs2_camera_info field)
{
    try {
        return device.supports(field) ? device.get_info(field) : "-";
    } catch (const rs2::error &) {
        return "-";
    }
}

std::uint64_t monotonic_ns()
{
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::uint64_t steady_time_ns(std::chrono::steady_clock::time_point time)
{
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        time.time_since_epoch()).count());
}

double age_ms(std::uint64_t now_ns, std::uint64_t event_ns)
{
    if (event_ns == 0 || now_ns < event_ns) return -1.0;
    return static_cast<double>(now_ns - event_ns) / 1000000.0;
}

const char *quality_name(std::uint8_t tracker_confidence,
                         std::uint8_t mapper_confidence,
                         bool t265_update_accepted)
{
    if (tracker_confidence >= 2 && mapper_confidence > 0 &&
        t265_update_accepted) return "GOOD";
    if (tracker_confidence >= 1) return "DEGRADED";
    return "LOST";
}

bool write_stm_status_json(const std::string &path,
                           const omni::StmStatusFrame &status,
                           std::uint64_t relay_frames,
                           std::uint64_t relay_errors,
                           std::uint8_t relay_sequence,
                           std::uint64_t relay_last_tx_ns)
{
    if (path.empty()) return true;
    const std::uint64_t now_ns = monotonic_ns();
    const double relay_age_ms = relay_last_tx_ns == 0 ? -1.0 :
        static_cast<double>(now_ns - relay_last_tx_ns) / 1000000.0;
    const std::string temporary = path + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    if (!file) return false;
    file << "{\n"
         << "  \"schema_version\": 1,\n"
         << "  \"timestamp_monotonic_ns\": " << monotonic_ns() << ",\n"
         << "  \"sequence\": " << static_cast<unsigned>(status.sequence) << ",\n"
         << "  \"flags\": " << static_cast<unsigned>(status.flags) << ",\n"
         << "  \"mode\": " << static_cast<unsigned>(status.mode) << ",\n"
         << "  \"camera_pitch_cdeg\": " << status.camera_pitch_cdeg << ",\n"
         << "  \"acknowledged_sequence\": "
         << static_cast<unsigned>(status.acknowledged_sequence) << ",\n"
         << "  \"fault_code\": " << static_cast<unsigned>(status.fault_code) << ",\n"
         << "  \"relay\": {\"tx_frames\": " << relay_frames
         << ", \"tx_errors\": " << relay_errors
         << ", \"last_sequence\": " << static_cast<unsigned>(relay_sequence)
         << ", \"last_tx_age_ms\": " << std::fixed << std::setprecision(3)
         << relay_age_ms << "}\n"
         << "}\n";
    file.close();
    return file && std::rename(temporary.c_str(), path.c_str()) == 0;
}

bool read_relay_frame(const std::string &path,
                      std::array<std::uint8_t, omni::kFrameSize> &result)
{
    if (path.empty()) return false;
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    file.read(reinterpret_cast<char *>(result.data()), result.size());
    if (file.gcount() != static_cast<std::streamsize>(result.size())) return false;
    char extra = 0;
    if (file.get(extra)) return false;
    return omni::validate_relay_frame(result.data(), result.size());
}

void write_atomic_json(const std::string &path,
                       const omni::Pose2d &fused,
                       const omni::T265FieldPose &t265,
                       const omni::PlanarEkf &filter,
                       const omni::PlanarOdometry &odometry,
                       const WheelDebugState &wheel_debug,
                       const char *quality,
                       const std::string &wheel_gate,
                       bool uart_fresh,
                       std::uint64_t wheel_accepted,
                       std::uint64_t wheel_rejected,
                       std::uint64_t uart_frames,
                       std::uint64_t uart_crc_errors,
                       std::uint64_t uart_sequence_gaps,
                       std::uint64_t pose_tx_frames,
                       std::uint64_t pose_tx_errors,
                       bool navigation_active,
                       std::uint8_t navigation_code,
                       std::uint16_t navigation_remaining,
                       std::uint16_t navigation_heading,
                       double navigation_wheel_progress_m,
                       bool t265_position_corrected,
                       double t265_position_sigma_multiplier,
                       double t265_innovation_m)
{
    if (path.empty()) return;
    const std::string temporary = path + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    if (!file) throw std::runtime_error("cannot write JSON: " + temporary);
    const std::uint64_t now_ns = monotonic_ns();
    const omni::Pose2d odom_pose = odometry.pose();
    const omni::WheelIncrement &increment = wheel_debug.latest_increment;
    const bool odom_available = wheel_debug.odom_updates > 0;
    const double fused_odom_delta_m = odom_available
        ? std::hypot(fused.x_m - odom_pose.x_m, fused.y_m - odom_pose.y_m)
        : -1.0;
    const double fused_odom_yaw_delta_deg = odom_available
        ? omni::degrees(omni::wrap_angle(fused.yaw_rad - odom_pose.yaw_rad))
        : 0.0;
    file << std::fixed << std::setprecision(9)
         << "{\n"
         << "  \"schema_version\": 2,\n"
         << "  \"timestamp_monotonic_ns\": " << now_ns << ",\n"
         << "  \"frame\": \"field\",\n"
         << "  \"quality\": \"" << quality << "\",\n"
         << "  \"pose\": {\"x_m\": " << fused.x_m
         << ", \"y_m\": " << fused.y_m
         << ", \"yaw_rad\": " << fused.yaw_rad
         << ", \"yaw_deg\": " << omni::degrees(fused.yaw_rad) << "},\n"
         << "  \"sigma\": {\"position_m\": " << filter.position_sigma_m()
         << ", \"yaw_rad\": " << filter.yaw_sigma_rad() << "},\n"
         << "  \"t265\": {\"x_m\": " << t265.pose.x_m
         << ", \"y_m\": " << t265.pose.y_m
         << ", \"yaw_rad\": " << t265.pose.yaw_rad
         << ", \"body_forward_velocity_mps\": "
         << t265.body_forward_velocity_mps
         << ", \"body_left_velocity_mps\": "
         << t265.body_left_velocity_mps
         << ", \"yaw_rate_radps\": " << t265.yaw_rate_radps
         << ", \"tracker_confidence\": " << static_cast<unsigned>(t265.tracker_confidence)
         << ", \"mapper_confidence\": " << static_cast<unsigned>(t265.mapper_confidence)
         << ", \"travel_from_start_m\": " << t265.travel_from_origin_m
         << ", \"forward_world\": [" << t265.forward_world[0] << ", "
         << t265.forward_world[1] << ", " << t265.forward_world[2] << "]"
         << ", \"left_world\": [" << t265.left_world[0] << ", "
         << t265.left_world[1] << ", " << t265.left_world[2] << "]},\n"
         << "  \"wheel\": {\"gate\": \"" << wheel_gate
         << "\", \"uart_fresh\": " << (uart_fresh ? "true" : "false")
         << ", \"accepted\": " << wheel_accepted
         << ", \"rejected\": " << wheel_rejected
         << ", \"last_frame_age_ms\": "
         << age_ms(now_ns, wheel_debug.latest_frame_ns)
         << ", \"last_update_age_ms\": "
         << age_ms(now_ns, wheel_debug.latest_update_ns) << "},\n"
         << "  \"wheel_odom\": {\"available\": "
         << (odom_available ? "true" : "false")
         << ", \"x_m\": " << odom_pose.x_m
         << ", \"y_m\": " << odom_pose.y_m
         << ", \"yaw_rad\": " << odom_pose.yaw_rad
         << ", \"yaw_deg\": " << omni::degrees(odom_pose.yaw_rad)
         << ", \"travel_m\": " << odometry.travel_m()
         << ", \"forward_velocity_mps\": "
         << (wheel_debug.have_latest_increment ? increment.forward_velocity_mps : 0.0)
         << ", \"left_velocity_mps\": "
         << (wheel_debug.have_latest_increment ? increment.left_velocity_mps : 0.0)
         << ", \"yaw_rate_radps\": "
         << ((wheel_debug.have_latest_increment && increment.dt_s > 0.0)
                 ? increment.yaw_rad / increment.dt_s : 0.0)
         << ", \"increment\": {\"forward_m\": "
         << (wheel_debug.have_latest_increment ? increment.forward_m : 0.0)
         << ", \"left_m\": "
         << (wheel_debug.have_latest_increment ? increment.left_m : 0.0)
         << ", \"yaw_rad\": "
         << (wheel_debug.have_latest_increment ? increment.yaw_rad : 0.0)
         << ", \"dt_s\": "
         << (wheel_debug.have_latest_increment ? increment.dt_s : 0.0)
         << ", \"sequence_step\": "
         << (wheel_debug.have_latest_increment
                 ? static_cast<unsigned>(increment.sequence_step) : 0u) << "}"
         << ", \"updates\": " << wheel_debug.odom_updates
         << ", \"last_update_age_ms\": "
         << age_ms(now_ns, wheel_debug.latest_update_ns)
         << ", \"updated_this_pose\": "
         << (wheel_debug.updated_this_pose ? "true" : "false")
         << ", \"accepted_this_pose\": "
         << (wheel_debug.accepted_this_pose ? "true" : "false")
         << ", \"rejected_this_pose\": "
         << (wheel_debug.rejected_this_pose ? "true" : "false")
         << ", \"last_increment_accepted\": "
         << (wheel_debug.latest_increment_accepted ? "true" : "false")
         << ", \"last_frame\": {\"available\": "
         << (wheel_debug.have_latest_frame ? "true" : "false")
         << ", \"sequence\": "
         << (wheel_debug.have_latest_frame
                 ? static_cast<unsigned>(wheel_debug.latest_frame.sequence) : 0u)
         << ", \"m1\": "
         << (wheel_debug.have_latest_frame ? wheel_debug.latest_frame.position[0] : 0u)
         << ", \"m2\": "
         << (wheel_debug.have_latest_frame ? wheel_debug.latest_frame.position[1] : 0u)
         << ", \"m3\": "
         << (wheel_debug.have_latest_frame ? wheel_debug.latest_frame.position[2] : 0u)
         << ", \"sample_period_ms\": "
         << (wheel_debug.have_latest_frame
                 ? static_cast<unsigned>(wheel_debug.latest_frame.sample_period_ms) : 0u)
         << ", \"status\": "
         << (wheel_debug.have_latest_frame
                 ? static_cast<unsigned>(wheel_debug.latest_frame.status) : 0u)
         << "}},\n"
         << "  \"comparison\": {\"fused_vs_wheel_odom_distance_m\": "
         << fused_odom_delta_m
         << ", \"fused_vs_wheel_odom_yaw_deg\": "
         << fused_odom_yaw_delta_deg << "},\n"
         << "  \"navigation\": {\"active\": "
         << (navigation_active ? "true" : "false")
         << ", \"command\": " << static_cast<unsigned>(navigation_code)
         << ", \"remaining_mm\": " << navigation_remaining
         << ", \"heading_cdeg\": " << navigation_heading
         << ", \"wheel_progress_m\": " << navigation_wheel_progress_m
         << ", \"t265_position_corrected\": "
         << (t265_position_corrected ? "true" : "false")
         << ", \"t265_position_sigma_multiplier\": "
         << t265_position_sigma_multiplier
         << ", \"t265_innovation_m\": " << t265_innovation_m << "},\n"
         << "  \"uart\": {\"frames\": " << uart_frames
         << ", \"crc_errors\": " << uart_crc_errors
         << ", \"sequence_gaps\": " << uart_sequence_gaps
         << ", \"pose_tx_frames\": " << pose_tx_frames
         << ", \"pose_tx_errors\": " << pose_tx_errors << "}\n"
         << "}\n";
    file.close();
    if (!file) throw std::runtime_error("failed while writing JSON: " + temporary);
    if (std::rename(temporary.c_str(), path.c_str()) != 0) {
        throw std::runtime_error("cannot replace JSON output: " + path);
    }
}

}  // namespace

int main(int argc, char **argv)
{
    std::atomic<bool> running{true};
    std::thread uart_thread;
    std::thread relay_thread;
    try {
        const Options options = parse_options(argc, argv);
        const omni::LocalizationConfig config = omni::load_config(options.config_path);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        rs2::log_to_console(options.debug_sdk ? RS2_LOG_SEVERITY_DEBUG : RS2_LOG_SEVERITY_WARN);

        std::cerr << "[CONFIG] start_zone=" << config.start_zone
                  << " wheel=" << config.wheel_diameter_m * 1000.0 << " mm"
                  << " counts/rev=" << config.counts_per_wheel_revolution
                  << " startup_gate=" << config.startup_wheel_disable_distance_m << " m\n";

        rs2::context context;
        rs2::device selected;
        const rs2::device_list devices = context.query_devices(RS2_PRODUCT_LINE_T200);
        for (std::size_t i = 0; i < devices.size(); ++i) {
            if (options.serial.empty() ||
                device_info(devices[i], RS2_CAMERA_INFO_SERIAL_NUMBER) == options.serial) {
                selected = devices[i];
                break;
            }
        }
        if (!selected) {
            throw std::runtime_error("no running T265; use run_localization.sh to boot 03e7:2150 first");
        }
        const std::string serial = device_info(selected, RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cerr << "[T265] serial=" << serial
                  << " firmware=" << device_info(selected, RS2_CAMERA_INFO_FIRMWARE_VERSION)
                  << " librealsense=" << RS2_API_VERSION_STR << '\n';
        selected = rs2::device();

        EncoderQueue encoder_queue;
        omni::ParserStats final_parser_stats{};
        std::atomic<std::uint64_t> live_uart_frames{0};
        std::atomic<std::uint64_t> live_crc_errors{0};
        std::atomic<std::uint64_t> live_sequence_gaps{0};
        std::atomic<std::uint64_t> status_write_errors{0};
        std::atomic<std::int64_t> last_uart_ns{0};
        std::atomic<std::uint64_t> relay_tx_frames{0};
        std::atomic<std::uint64_t> relay_tx_errors{0};
        std::atomic<std::uint8_t> relay_last_sequence{0};
        std::atomic<std::uint64_t> relay_last_tx_ns{0};
        std::atomic<bool> navigation_command_active{false};
        std::atomic<std::uint8_t> navigation_command_code{0};
        std::atomic<std::uint16_t> navigation_remaining_mm{0};
        std::atomic<std::uint16_t> navigation_heading_cdeg{0};
        std::mutex uart_tx_mutex;
        std::unique_ptr<omni::SerialPort> uart;
        if (!options.uart_path.empty()) {
            uart.reset(new omni::SerialPort(options.uart_path, options.baud));
            uart->open_port();
            std::cerr << "[UART] " << options.uart_path << " @ " << options.baud
                      << " 8N1, RX TYPE=0x15/0x17, TX TYPE=0x16 @ "
                      << options.tx_rate_hz
                      << " Hz, mission heartbeat=100 Hz (T265-independent)\n";
            uart_thread = std::thread([&]() {
                omni::F407FrameParser parser([&](const omni::EncoderFrame &frame) {
                    if (!options.ignore_encoders) encoder_queue.push(frame);
                    last_uart_ns.store(static_cast<std::int64_t>(monotonic_ns()),
                                       std::memory_order_relaxed);
                }, [&](const omni::StmStatusFrame &status) {
                    if (!write_stm_status_json(
                            options.stm_status_output_path, status,
                            relay_tx_frames.load(std::memory_order_relaxed),
                            relay_tx_errors.load(std::memory_order_relaxed),
                            relay_last_sequence.load(std::memory_order_relaxed),
                            relay_last_tx_ns.load(std::memory_order_relaxed))) {
                        status_write_errors.fetch_add(1, std::memory_order_relaxed);
                    }
                });
                std::uint8_t buffer[256];
                while (running.load(std::memory_order_relaxed) && !g_stop) {
                    const int count = uart->read_some(buffer, sizeof(buffer), 50);
                    if (count < 0) {
                        std::cerr << "[UART ERROR] read failed on " << uart->path() << '\n';
                        running.store(false, std::memory_order_relaxed);
                        break;
                    }
                    if (count > 0) parser.feed(buffer, static_cast<std::size_t>(count));
                    const omni::ParserStats &stats = parser.stats();
                    live_uart_frames.store(stats.frames_ok, std::memory_order_relaxed);
                    live_crc_errors.store(stats.crc_errors, std::memory_order_relaxed);
                    live_sequence_gaps.store(stats.sequence_gaps, std::memory_order_relaxed);
                }
                final_parser_stats = parser.stats();
            });
            if (!options.command_file_path.empty()) {
                relay_thread = std::thread([&]() {
                    constexpr auto heartbeat_period = std::chrono::milliseconds(10);
                    std::array<std::uint8_t, omni::kFrameSize> last_input{};
                    std::array<std::uint8_t, omni::kFrameSize> active_mission{};
                    bool have_last_input = false;
                    bool have_active_mission = false;
                    std::uint8_t mission_sequence = 0;
                    auto next_heartbeat = std::chrono::steady_clock::now();
                    auto last_input_change = next_heartbeat;
                    constexpr auto heartbeat_bridge_limit =
                        std::chrono::milliseconds(250);

                    auto transmit = [&](const std::array<std::uint8_t, omni::kFrameSize> &frame) {
                        std::lock_guard<std::mutex> lock(uart_tx_mutex);
                        if (uart->write_all(frame.data(), frame.size(), 50)) {
                            relay_tx_frames.fetch_add(1, std::memory_order_relaxed);
                            relay_last_sequence.store(frame[3], std::memory_order_relaxed);
                            relay_last_tx_ns.store(monotonic_ns(), std::memory_order_relaxed);
                        } else {
                            relay_tx_errors.fetch_add(1, std::memory_order_relaxed);
                        }
                    };

                    while (running.load(std::memory_order_relaxed) && !g_stop) {
                        std::array<std::uint8_t, omni::kFrameSize> input{};
                        const auto now = std::chrono::steady_clock::now();
                        if (read_relay_frame(options.command_file_path, input) &&
                            (!have_last_input || input != last_input)) {
                            last_input = input;
                            have_last_input = true;
                            last_input_change = now;
                            if (input[2] == omni::kMissionCommandMessageType) {
                                if (!have_active_mission) {
                                    mission_sequence = input[3];
                                }
                                active_mission = input;
                                have_active_mission = true;
                                const std::uint8_t command = input[4];
                                const bool distance_valid = (input[5] & 0x10u) != 0u;
                                navigation_command_code.store(command, std::memory_order_relaxed);
                                navigation_remaining_mm.store(
                                    static_cast<std::uint16_t>(
                                        (static_cast<std::uint16_t>(input[6]) << 8) |
                                        input[7]),
                                    std::memory_order_relaxed);
                                navigation_heading_cdeg.store(
                                    static_cast<std::uint16_t>(
                                        (static_cast<std::uint16_t>(input[10]) << 8) |
                                        input[11]),
                                    std::memory_order_relaxed);
                                navigation_command_active.store(
                                    distance_valid && (command == 3u || command == 8u),
                                    std::memory_order_relaxed);
                                next_heartbeat = now;
                            } else {
                                have_active_mission = false;
                                navigation_command_active.store(false, std::memory_order_relaxed);
                                transmit(input);
                            }
                        }
                        if (have_active_mission &&
                            now - last_input_change > heartbeat_bridge_limit) {
                            have_active_mission = false;
                            navigation_command_active.store(false, std::memory_order_relaxed);
                        }
                        if (have_active_mission && now >= next_heartbeat) {
                            auto heartbeat = active_mission;
                            if (omni::refresh_mission_frame_sequence(
                                    heartbeat, mission_sequence++)) {
                                transmit(heartbeat);
                            } else {
                                relay_tx_errors.fetch_add(1, std::memory_order_relaxed);
                                have_active_mission = false;
                            }
                            next_heartbeat = now + heartbeat_period;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    }
                });
            }
        } else {
            std::cerr << "[UART] disabled; output is T265-only\n";
        }

        std::ofstream csv;
        std::uint64_t csv_rows = 0;
        if (!options.csv_path.empty()) {
            csv.open(options.csv_path);
            if (!csv) throw std::runtime_error("cannot open CSV: " + options.csv_path);
            csv << "timestamp_monotonic_ns,elapsed_s,t265_timestamp_s,"
                   "raw_tx_m,raw_ty_m,raw_tz_m,raw_vx_mps,raw_vy_mps,raw_vz_mps,"
                   "raw_qx,raw_qy,raw_qz,raw_qw,raw_ang_vx_radps,raw_ang_vy_radps,"
                   "raw_ang_vz_radps,forward_world_x,forward_world_y,forward_world_z,"
                   "left_world_x,left_world_y,left_world_z,raw_chassis_yaw_deg,"
                   "relative_yaw_deg,t265_x_m,t265_y_m,t265_yaw_deg,"
                   "t265_forward_velocity_mps,t265_left_velocity_mps,t265_yaw_rate_degps,"
                   "t265_travel_m,fused_x_m,fused_y_m,fused_yaw_deg,odom_x_m,odom_y_m,"
                   "odom_yaw_deg,odom_travel_m,odom_forward_velocity_mps,"
                   "odom_left_velocity_mps,odom_yaw_rate_degps,fused_odom_delta_m,"
                   "fused_odom_yaw_delta_deg,odom_increment_forward_m,"
                   "odom_increment_left_m,odom_increment_yaw_deg,odom_increment_dt_s,"
                   "odom_increment_sequence_step,wheel_frame_sequence,wheel_m1_count,"
                   "wheel_m2_count,wheel_m3_count,wheel_sample_period_ms,wheel_status,"
                   "wheel_update_this_pose,wheel_update_accepted,wheel_update_rejected,"
                   "last_increment_accepted,wheel_gate,"
                   "wheel_accepted_count,wheel_rejected_count,wheel_odom_updates,"
                   "wheel_last_frame_age_ms,wheel_last_update_age_ms,tracker_confidence,"
                   "mapper_confidence,t265_update_accepted,t265_position_corrected,"
                   "t265_position_sigma_multiplier,t265_innovation_m,position_sigma_m,"
                   "yaw_sigma_deg,quality,navigation_active,navigation_command,"
                   "navigation_remaining_mm,navigation_heading_cdeg,"
                   "navigation_wheel_progress_m\n";
        }

        rs2::pipeline pipeline(context);
        rs2::config rs_config;
        rs_config.enable_device(serial);
        rs_config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        pipeline.start(rs_config);

        omni::T265FieldProjector projector(config);
        omni::OmniEncoderIntegrator encoder_integrator(config);
        omni::PlanarEkf filter;
        omni::PlanarOdometry odometry;
        WheelDebugState wheel_debug;
        omni::T265FieldPose latest_t265;
        std::string wheel_gate = options.uart_path.empty()
            ? "uart_disabled"
            : (options.ignore_encoders ? "t265_only" : "no_baseline");
        std::uint64_t wheel_accepted = 0;
        std::uint64_t wheel_rejected = 0;
        bool have_first_pose = false;
        bool t265_update_accepted = false;
        std::uint8_t previous_tracker_confidence = 0;
        auto first_pose_time = std::chrono::steady_clock::now();
        auto next_output = first_pose_time;
        const auto output_period = options.output_rate_hz > 0.0
            ? std::chrono::duration<double>(1.0 / options.output_rate_hz)
            : std::chrono::duration<double>(0.0);
        auto next_pose_tx = first_pose_time;
        const auto pose_tx_period = options.tx_rate_hz > 0.0
            ? std::chrono::duration<double>(1.0 / options.tx_rate_hz)
            : std::chrono::duration<double>(0.0);
        std::uint8_t pose_tx_sequence = 0;
        std::uint64_t pose_tx_frames = 0;
        std::uint64_t pose_tx_errors = 0;
        bool previous_navigation_active = false;
        double navigation_wheel_progress_m = 0.0;
        bool t265_position_corrected = true;
        double active_t265_position_multiplier = 1.0;
        auto next_navigation_position_correction = first_pose_time;
        std::cerr << "[RUN] field +X right, +Y up; yaw is counter-clockwise from +X\n";
        while (running.load(std::memory_order_relaxed) && !g_stop) {
            const rs2::frameset frames = pipeline.wait_for_frames(1000);
            const rs2::pose_frame pose_frame = frames.get_pose_frame();
            if (!pose_frame) continue;
            const rs2_pose pose = pose_frame.get_pose_data();
            omni::T265RawPose raw;
            raw.translation_m[0] = pose.translation.x;
            raw.translation_m[1] = pose.translation.y;
            raw.translation_m[2] = pose.translation.z;
            raw.velocity_mps[0] = pose.velocity.x;
            raw.velocity_mps[1] = pose.velocity.y;
            raw.velocity_mps[2] = pose.velocity.z;
            raw.rotation_xyzw[0] = pose.rotation.x;
            raw.rotation_xyzw[1] = pose.rotation.y;
            raw.rotation_xyzw[2] = pose.rotation.z;
            raw.rotation_xyzw[3] = pose.rotation.w;
            raw.angular_velocity_radps[0] = pose.angular_velocity.x;
            raw.angular_velocity_radps[1] = pose.angular_velocity.y;
            raw.angular_velocity_radps[2] = pose.angular_velocity.z;
            raw.timestamp_s = pose_frame.get_timestamp() * 0.001;
            raw.tracker_confidence = pose.tracker_confidence;
            raw.mapper_confidence = pose.mapper_confidence;
            latest_t265 = projector.project(raw);

            if (!filter.initialized() && latest_t265.tracker_confidence > 0) {
                filter.initialize(latest_t265.pose);
                odometry.initialize(latest_t265.pose);
                first_pose_time = std::chrono::steady_clock::now();
                next_output = first_pose_time;
                next_pose_tx = first_pose_time;
                have_first_pose = true;
                previous_tracker_confidence = latest_t265.tracker_confidence;
            }
            if (!filter.initialized()) continue;

            const auto now = std::chrono::steady_clock::now();
            const bool navigation_active =
                navigation_command_active.load(std::memory_order_relaxed);
            const std::uint8_t active_navigation_code =
                navigation_command_code.load(std::memory_order_relaxed);
            const std::uint16_t active_navigation_remaining_mm =
                navigation_remaining_mm.load(std::memory_order_relaxed);
            const std::uint16_t active_navigation_heading_cdeg =
                navigation_heading_cdeg.load(std::memory_order_relaxed);
            if (navigation_active && !previous_navigation_active) {
                navigation_wheel_progress_m = 0.0;
                next_navigation_position_correction = now;
            }
            previous_navigation_active = navigation_active;

            wheel_debug.updated_this_pose = false;
            wheel_debug.accepted_this_pose = false;
            wheel_debug.rejected_this_pose = false;

            for (const TimedEncoderFrame &timed : encoder_queue.drain()) {
                wheel_debug.have_latest_frame = true;
                wheel_debug.latest_frame = timed.frame;
                wheel_debug.latest_frame_ns = steady_time_ns(timed.received);
                omni::WheelIncrement increment;
                std::string integration_reason;
                if (!encoder_integrator.update(timed.frame, increment, integration_reason)) {
                    wheel_gate = integration_reason;
                    wheel_debug.rejected_this_pose = true;
                    continue;
                }
                const omni::WheelGateReason gate =
                    omni::evaluate_wheel_gate(config, latest_t265, increment);
                wheel_gate = omni::wheel_gate_reason_name(gate);
                wheel_debug.have_latest_increment = true;
                wheel_debug.latest_increment = increment;
                wheel_debug.latest_update_ns = wheel_debug.latest_frame_ns;
                wheel_debug.updated_this_pose = true;
                // Keep this raw encoder-only trajectory even when the same
                // increment is rejected by the EKF safety gate. The rejected
                // status is logged separately so drift and gating can be
                // diagnosed from one run.
                odometry.integrate(increment);
                ++wheel_debug.odom_updates;
                const double wheel_speed = std::hypot(
                    increment.forward_velocity_mps,
                    increment.left_velocity_mps);
                const double t265_speed = std::hypot(
                    latest_t265.body_forward_velocity_mps,
                    latest_t265.body_left_velocity_mps);
                const bool near_navigation_target = navigation_active &&
                    active_navigation_remaining_mm <=
                        static_cast<std::uint16_t>(
                            std::lround(config.navigation_near_target_m * 1000.0));
                const bool near_target_slip = near_navigation_target &&
                    wheel_speed >= config.navigation_slip_wheel_speed_mps &&
                    t265_speed <= config.navigation_slip_t265_speed_mps;
                const bool navigation_encoder_override = navigation_active &&
                    !near_navigation_target &&
                    latest_t265.mapper_confidence == 0 &&
                    gate == omni::WheelGateReason::VelocityMismatch;
                if (near_target_slip) {
                    wheel_gate = "navigation_near_target_slip";
                    wheel_debug.rejected_this_pose = true;
                    wheel_debug.latest_increment_accepted = false;
                    ++wheel_rejected;
                } else if (gate == omni::WheelGateReason::Accepted ||
                           navigation_encoder_override) {
                    if (navigation_encoder_override) {
                        wheel_gate = "navigation_encoder_override";
                    }
                    const omni::Pose2d before_predict = filter.pose();
                    const double c = std::cos(before_predict.yaw_rad);
                    const double s = std::sin(before_predict.yaw_rad);
                    const double field_dx =
                        c * increment.forward_m - s * increment.left_m;
                    const double field_dy =
                        s * increment.forward_m + c * increment.left_m;
                    if (navigation_active) {
                        const double command_heading = omni::radians(
                            static_cast<double>(active_navigation_heading_cdeg) * 0.01);
                        const double progress =
                            field_dx * std::cos(command_heading) +
                            field_dy * std::sin(command_heading);
                        navigation_wheel_progress_m += progress;
                    }
                    filter.predict(increment, config);
                    wheel_debug.accepted_this_pose = true;
                    wheel_debug.latest_increment_accepted = true;
                    ++wheel_accepted;
                } else {
                    wheel_debug.rejected_this_pose = true;
                    wheel_debug.latest_increment_accepted = false;
                    ++wheel_rejected;
                }
            }

            double innovation_m = 0.0;
            active_t265_position_multiplier = 1.0;
            t265_position_corrected = true;
            if (navigation_active) {
                active_t265_position_multiplier =
                    latest_t265.mapper_confidence == 0
                        ? std::max(
                            config.navigation_t265_position_sigma_multiplier,
                            config.mapper_zero_position_sigma_multiplier)
                        : config.navigation_t265_position_sigma_multiplier;
                t265_position_corrected =
                    now >= next_navigation_position_correction;
                if (t265_position_corrected) {
                    const auto period = std::chrono::duration<double>(
                        1.0 / config.navigation_t265_position_correction_rate_hz);
                    next_navigation_position_correction = now +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
                }
            }
            if (!navigation_active && previous_tracker_confidence == 0 &&
                latest_t265.tracker_confidence > 0 && have_first_pose) {
                // After a complete visual tracking outage, T265 is the primary
                // absolute source. Re-anchor instead of permanently rejecting a
                // legitimate reacquisition farther than the normal jump gate.
                filter.initialize(latest_t265.pose);
                t265_update_accepted = true;
                wheel_gate = "t265_reacquired";
            } else {
                t265_update_accepted = filter.correct_t265(
                    latest_t265, config, &innovation_m,
                    active_t265_position_multiplier,
                    t265_position_corrected);
            }
            previous_tracker_confidence = latest_t265.tracker_confidence;
            if (!t265_update_accepted && innovation_m > config.maximum_t265_innovation_m) {
                wheel_gate = "t265_jump_rejected";
            }

            const double elapsed = std::chrono::duration<double>(now - first_pose_time).count();
            if (have_first_pose && options.duration_sec > 0.0 && elapsed >= options.duration_sec) {
                break;
            }
            const omni::Pose2d fused = filter.pose();
            const omni::Pose2d odom = odometry.pose();
            const bool odom_available = wheel_debug.odom_updates > 0;
            const double fused_odom_delta_m = odom_available
                ? std::hypot(fused.x_m - odom.x_m, fused.y_m - odom.y_m) : -1.0;
            const double fused_odom_yaw_delta_deg = odom_available
                ? omni::degrees(omni::wrap_angle(fused.yaw_rad - odom.yaw_rad)) : 0.0;
            const char *quality = quality_name(
                latest_t265.tracker_confidence,
                latest_t265.mapper_confidence,
                t265_update_accepted);
            const std::int64_t last_ns = last_uart_ns.load(std::memory_order_relaxed);
            const std::uint64_t sample_ns = monotonic_ns();
            const bool uart_fresh = !options.uart_path.empty() && last_ns > 0 &&
                static_cast<std::int64_t>(sample_ns) - last_ns <=
                    static_cast<std::int64_t>(config.uart_stale_ms) * 1000000LL;

            if (uart && options.tx_rate_hz > 0.0 && now >= next_pose_tx) {
                next_pose_tx = now +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(pose_tx_period);
                const long x_mm_long = std::lround(fused.x_m * 1000.0);
                const long y_mm_long = std::lround(fused.y_m * 1000.0);
                double heading_deg = std::fmod(omni::degrees(fused.yaw_rad), 360.0);
                if (heading_deg < 0.0) heading_deg += 360.0;

                omni::FusedPoseFrame tx;
                tx.sequence = pose_tx_sequence++;
                tx.x_mm = static_cast<std::int16_t>(std::max<long>(
                    std::numeric_limits<std::int16_t>::min(),
                    std::min<long>(std::numeric_limits<std::int16_t>::max(), x_mm_long)));
                tx.y_mm = static_cast<std::int16_t>(std::max<long>(
                    std::numeric_limits<std::int16_t>::min(),
                    std::min<long>(std::numeric_limits<std::int16_t>::max(), y_mm_long)));
                tx.heading_cdeg = static_cast<std::uint16_t>(
                    std::lround(heading_deg * 100.0)) % 36000u;
                const bool obstacle_gate = wheel_gate == "startup_obstacle" ||
                                           wheel_gate == "corner_obstacle";
                const bool inside_field = std::fabs(fused.x_m) <= config.field_half_m &&
                                          std::fabs(fused.y_m) <= config.field_half_m;
                if (latest_t265.tracker_confidence > 0) tx.status |= omni::kPoseValid;
                if (latest_t265.tracker_confidence >= 2) tx.status |= omni::kPoseT265Good;
                if (uart_fresh && wheel_gate == "accepted") tx.status |= omni::kPoseWheelActive;
                if (obstacle_gate) tx.status |= omni::kPoseObstacleGate;
                if (uart_fresh) tx.status |= omni::kPoseOdomFresh;
                if (inside_field) tx.status |= omni::kPoseInsideField;
                if (!t265_update_accepted) tx.status |= omni::kPoseT265UpdateRejected;
                const unsigned sigma_cm = std::min<unsigned>(
                    15u, static_cast<unsigned>(std::lround(filter.position_sigma_m() * 100.0)));
                tx.confidence_and_sigma = static_cast<std::uint8_t>(
                    (sigma_cm << 4) |
                    ((static_cast<unsigned>(latest_t265.mapper_confidence) & 0x03u) << 2) |
                    (static_cast<unsigned>(latest_t265.tracker_confidence) & 0x03u));
                const auto bytes = omni::build_fused_pose_frame(tx);
                std::lock_guard<std::mutex> lock(uart_tx_mutex);
                if (uart->write_all(bytes.data(), bytes.size(), 50)) {
                    ++pose_tx_frames;
                } else {
                    ++pose_tx_errors;
                }
            }

            if (csv) {
                const bool have_increment = wheel_debug.have_latest_increment;
                const bool have_frame = wheel_debug.have_latest_frame;
                const double odom_yaw_rate_radps = have_increment &&
                    wheel_debug.latest_increment.dt_s > 0.0
                    ? wheel_debug.latest_increment.yaw_rad /
                        wheel_debug.latest_increment.dt_s : 0.0;
                csv << std::fixed << std::setprecision(9)
                    << sample_ns << ',' << elapsed << ',' << raw.timestamp_s << ','
                    << raw.translation_m[0] << ',' << raw.translation_m[1] << ','
                    << raw.translation_m[2] << ',' << raw.velocity_mps[0] << ','
                    << raw.velocity_mps[1] << ',' << raw.velocity_mps[2] << ','
                    << raw.rotation_xyzw[0] << ',' << raw.rotation_xyzw[1] << ','
                    << raw.rotation_xyzw[2] << ',' << raw.rotation_xyzw[3] << ','
                    << raw.angular_velocity_radps[0] << ','
                    << raw.angular_velocity_radps[1] << ','
                    << raw.angular_velocity_radps[2] << ','
                    << latest_t265.forward_world[0] << ','
                    << latest_t265.forward_world[1] << ','
                    << latest_t265.forward_world[2] << ','
                    << latest_t265.left_world[0] << ','
                    << latest_t265.left_world[1] << ','
                    << latest_t265.left_world[2] << ','
                    << omni::degrees(latest_t265.raw_chassis_yaw_rad) << ','
                    << omni::degrees(latest_t265.relative_yaw_rad) << ','
                    << latest_t265.pose.x_m << ',' << latest_t265.pose.y_m << ','
                    << omni::degrees(latest_t265.pose.yaw_rad) << ','
                    << latest_t265.body_forward_velocity_mps << ','
                    << latest_t265.body_left_velocity_mps << ','
                    << omni::degrees(latest_t265.yaw_rate_radps) << ','
                    << latest_t265.travel_from_origin_m << ','
                    << fused.x_m << ',' << fused.y_m << ','
                    << omni::degrees(fused.yaw_rad) << ','
                    << odom.x_m << ',' << odom.y_m << ','
                    << omni::degrees(odom.yaw_rad) << ',' << odometry.travel_m() << ','
                    << (have_increment
                            ? wheel_debug.latest_increment.forward_velocity_mps : 0.0)
                    << ','
                    << (have_increment
                            ? wheel_debug.latest_increment.left_velocity_mps : 0.0)
                    << ',' << omni::degrees(odom_yaw_rate_radps) << ','
                    << fused_odom_delta_m << ',' << fused_odom_yaw_delta_deg << ','
                    << (have_increment ? wheel_debug.latest_increment.forward_m : 0.0)
                    << ',' << (have_increment ? wheel_debug.latest_increment.left_m : 0.0)
                    << ',' << (have_increment
                            ? omni::degrees(wheel_debug.latest_increment.yaw_rad) : 0.0)
                    << ',' << (have_increment ? wheel_debug.latest_increment.dt_s : 0.0)
                    << ',' << (have_increment
                            ? static_cast<unsigned>(wheel_debug.latest_increment.sequence_step)
                            : 0u)
                    << ',' << (have_frame
                            ? static_cast<unsigned>(wheel_debug.latest_frame.sequence) : 0u)
                    << ',' << (have_frame ? wheel_debug.latest_frame.position[0] : 0u)
                    << ',' << (have_frame ? wheel_debug.latest_frame.position[1] : 0u)
                    << ',' << (have_frame ? wheel_debug.latest_frame.position[2] : 0u)
                    << ',' << (have_frame
                            ? static_cast<unsigned>(wheel_debug.latest_frame.sample_period_ms)
                            : 0u)
                    << ',' << (have_frame
                            ? static_cast<unsigned>(wheel_debug.latest_frame.status) : 0u)
                    << ',' << (wheel_debug.updated_this_pose ? 1 : 0)
                    << ',' << (wheel_debug.accepted_this_pose ? 1 : 0)
                    << ',' << (wheel_debug.rejected_this_pose ? 1 : 0)
                    << ',' << (wheel_debug.latest_increment_accepted ? 1 : 0)
                    << ',' << wheel_gate << ',' << wheel_accepted << ',' << wheel_rejected
                    << ',' << wheel_debug.odom_updates << ','
                    << age_ms(sample_ns, wheel_debug.latest_frame_ns) << ','
                    << age_ms(sample_ns, wheel_debug.latest_update_ns) << ','
                    << static_cast<unsigned>(latest_t265.tracker_confidence) << ','
                    << static_cast<unsigned>(latest_t265.mapper_confidence) << ','
                    << (t265_update_accepted ? 1 : 0) << ','
                    << (t265_position_corrected ? 1 : 0) << ','
                    << active_t265_position_multiplier << ',' << innovation_m << ','
                    << filter.position_sigma_m() << ','
                    << omni::degrees(filter.yaw_sigma_rad()) << ',' << quality << ','
                    << (navigation_active ? 1 : 0) << ','
                    << static_cast<unsigned>(active_navigation_code) << ','
                    << active_navigation_remaining_mm << ','
                    << active_navigation_heading_cdeg << ','
                    << navigation_wheel_progress_m << '\n';
                if ((++csv_rows % 20u) == 0u) csv.flush();
            }
            if (options.output_rate_hz > 0.0 && now < next_output) continue;
            if (options.output_rate_hz > 0.0) {
                next_output = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(output_period);
            }

            const std::uint64_t uart_frames = live_uart_frames.load(std::memory_order_relaxed);
            const std::uint64_t crc_errors = live_crc_errors.load(std::memory_order_relaxed);
            const std::uint64_t sequence_gaps = live_sequence_gaps.load(std::memory_order_relaxed);
            write_atomic_json(options.output_path, fused, latest_t265, filter, odometry,
                              wheel_debug, quality,
                              wheel_gate, uart_fresh, wheel_accepted, wheel_rejected,
                              uart_frames, crc_errors, sequence_gaps,
                              pose_tx_frames, pose_tx_errors,
                              navigation_active, active_navigation_code,
                              active_navigation_remaining_mm,
                              active_navigation_heading_cdeg,
                              navigation_wheel_progress_m,
                              t265_position_corrected,
                              active_t265_position_multiplier,
                              innovation_m);

            std::cout << std::fixed << std::setprecision(3)
                      << "POSE t=" << elapsed
                      << " field=(" << fused.x_m << ',' << fused.y_m << ')'
                      << " yaw=" << std::setprecision(1) << omni::degrees(fused.yaw_rad)
                      << "deg conf=" << static_cast<unsigned>(latest_t265.tracker_confidence)
                      << '/' << static_cast<unsigned>(latest_t265.mapper_confidence)
                      << " wheel=" << wheel_gate
                      << " uart=" << (uart_fresh ? "fresh" : "stale")
                      << " tx=" << pose_tx_frames << '/' << pose_tx_errors
                      << " nav=" << (navigation_active ? "wheel_primary" : "normal")
                      << " wprog=" << std::setprecision(3)
                      << navigation_wheel_progress_m << "m"
                      << " t265pos=" << (t265_position_corrected ? "correct" : "yaw_only")
                      << " x" << std::setprecision(1)
                      << active_t265_position_multiplier
                      << " sigma=" << std::setprecision(3) << filter.position_sigma_m() << "m\n";
        }

        pipeline.stop();
        running.store(false, std::memory_order_relaxed);
        if (uart_thread.joinable()) uart_thread.join();
        if (relay_thread.joinable()) relay_thread.join();
        std::cerr << "[SUMMARY] wheel accepted=" << wheel_accepted
                  << " rejected=" << wheel_rejected
                  << " UART frames=" << final_parser_stats.frames_ok
                  << " crc_errors=" << final_parser_stats.crc_errors
                  << " sequence_gaps=" << final_parser_stats.sequence_gaps
                  << " pose_tx=" << pose_tx_frames
                  << " pose_tx_errors=" << pose_tx_errors
                  << " relay_tx=" << relay_tx_frames.load(std::memory_order_relaxed)
                  << " relay_tx_errors=" << relay_tx_errors.load(std::memory_order_relaxed)
                  << " status_frames=" << final_parser_stats.status_frames
                  << " status_write_errors="
                  << status_write_errors.load(std::memory_order_relaxed)
                  << " queue_dropped=" << encoder_queue.dropped() << '\n';
        return EXIT_SUCCESS;
    } catch (const rs2::error &error) {
        running.store(false, std::memory_order_relaxed);
        if (uart_thread.joinable()) uart_thread.join();
        if (relay_thread.joinable()) relay_thread.join();
        std::cerr << "[RS2 ERROR] " << error.what() << " ("
                  << error.get_failed_function() << ' ' << error.get_failed_args() << ")\n";
        return 3;
    } catch (const std::exception &error) {
        running.store(false, std::memory_order_relaxed);
        if (uart_thread.joinable()) uart_thread.join();
        if (relay_thread.joinable()) relay_thread.join();
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
