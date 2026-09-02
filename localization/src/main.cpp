#include <librealsense2/rs.hpp>

#include "config.hpp"
#include "f407_protocol.hpp"
#include "fusion.hpp"
#include "serial_port.hpp"

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
    double tx_rate_hz = 20.0;
    double duration_sec = 0.0;
    bool debug_sdk = false;
};

struct TimedEncoderFrame {
    omni::EncoderFrame frame;
    std::chrono::steady_clock::time_point received;
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
        << "  --csv FILE          diagnostic CSV log\n"
        << "  --command-file FILE relay new valid TYPE 0x11/0x12/0x18 frames\n"
        << "  --stm-status FILE   atomic TYPE 0x17 status JSON output\n"
        << "  --rate HZ           stdout/JSON rate, default 20\n"
        << "  --tx-rate HZ        fused-pose UART rate; 0 disables TX (default 20)\n"
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

double heading_y(const rs2_quaternion &q)
{
    return std::atan2(2.0 * (q.w * q.y + q.x * q.z),
                      1.0 - 2.0 * (q.y * q.y + q.x * q.x));
}

std::uint64_t monotonic_ns()
{
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

const char *quality_name(std::uint8_t confidence, bool t265_update_accepted)
{
    if (confidence >= 2 && t265_update_accepted) return "GOOD";
    if (confidence >= 1) return "DEGRADED";
    return "LOST";
}

bool write_stm_status_json(const std::string &path, const omni::StmStatusFrame &status)
{
    if (path.empty()) return true;
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
         << "  \"fault_code\": " << static_cast<unsigned>(status.fault_code) << "\n"
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
                       const char *quality,
                       const std::string &wheel_gate,
                       bool uart_fresh,
                       std::uint64_t wheel_accepted,
                       std::uint64_t wheel_rejected,
                       std::uint64_t uart_frames,
                       std::uint64_t uart_crc_errors,
                       std::uint64_t uart_sequence_gaps,
                       std::uint64_t pose_tx_frames,
                       std::uint64_t pose_tx_errors)
{
    if (path.empty()) return;
    const std::string temporary = path + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    if (!file) throw std::runtime_error("cannot write JSON: " + temporary);
    file << std::fixed << std::setprecision(9)
         << "{\n"
         << "  \"schema_version\": 1,\n"
         << "  \"timestamp_monotonic_ns\": " << monotonic_ns() << ",\n"
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
         << ", \"tracker_confidence\": " << static_cast<unsigned>(t265.tracker_confidence)
         << ", \"mapper_confidence\": " << static_cast<unsigned>(t265.mapper_confidence)
         << ", \"travel_from_start_m\": " << t265.travel_from_origin_m << "},\n"
         << "  \"wheel\": {\"gate\": \"" << wheel_gate
         << "\", \"uart_fresh\": " << (uart_fresh ? "true" : "false")
         << ", \"accepted\": " << wheel_accepted
         << ", \"rejected\": " << wheel_rejected << "},\n"
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
        std::unique_ptr<omni::SerialPort> uart;
        if (!options.uart_path.empty()) {
            uart.reset(new omni::SerialPort(options.uart_path, options.baud));
            uart->open_port();
            std::cerr << "[UART] " << options.uart_path << " @ " << options.baud
                      << " 8N1, RX TYPE=0x15/0x17, TX TYPE=0x16 @ "
                      << options.tx_rate_hz << " Hz\n";
            uart_thread = std::thread([&]() {
                omni::F407FrameParser parser([&](const omni::EncoderFrame &frame) {
                    encoder_queue.push(frame);
                    last_uart_ns.store(static_cast<std::int64_t>(monotonic_ns()),
                                       std::memory_order_relaxed);
                }, [&](const omni::StmStatusFrame &status) {
                    if (!write_stm_status_json(options.stm_status_output_path, status)) {
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
        } else {
            std::cerr << "[UART] disabled; output is T265-only\n";
        }

        std::ofstream csv;
        if (!options.csv_path.empty()) {
            csv.open(options.csv_path);
            if (!csv) throw std::runtime_error("cannot open CSV: " + options.csv_path);
            csv << "time_s,fused_x_m,fused_y_m,fused_yaw_deg,t265_x_m,t265_y_m,"
                   "t265_yaw_deg,tracker_confidence,mapper_confidence,wheel_gate,"
                   "wheel_accepted,wheel_rejected,position_sigma_m,yaw_sigma_deg\n";
        }

        rs2::pipeline pipeline(context);
        rs2::config rs_config;
        rs_config.enable_device(serial);
        rs_config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        pipeline.start(rs_config);

        omni::T265FieldProjector projector(config);
        omni::OmniEncoderIntegrator encoder_integrator(config);
        omni::PlanarEkf filter;
        omni::T265FieldPose latest_t265;
        std::string wheel_gate = options.uart_path.empty() ? "uart_disabled" : "no_baseline";
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
        std::uint64_t relay_tx_frames = 0;
        std::uint64_t relay_tx_errors = 0;
        std::array<std::uint8_t, omni::kFrameSize> last_relay_frame{};
        bool have_last_relay_frame = false;

        std::cerr << "[RUN] field +X right, +Y up; yaw is counter-clockwise from +X\n";
        while (running.load(std::memory_order_relaxed) && !g_stop) {
            const rs2::frameset frames = pipeline.wait_for_frames(1000);
            const rs2::pose_frame pose_frame = frames.get_pose_frame();
            if (!pose_frame) continue;
            const rs2_pose pose = pose_frame.get_pose_data();
            omni::T265RawPose raw;
            raw.native_x_m = pose.translation.x;
            raw.native_z_m = pose.translation.z;
            raw.native_vx_mps = pose.velocity.x;
            raw.native_vz_mps = pose.velocity.z;
            raw.heading_y_rad = heading_y(pose.rotation);
            raw.angular_velocity_y_radps = pose.angular_velocity.y;
            raw.tracker_confidence = pose.tracker_confidence;
            raw.mapper_confidence = pose.mapper_confidence;
            latest_t265 = projector.project(raw);

            if (!filter.initialized() && latest_t265.tracker_confidence > 0) {
                filter.initialize(latest_t265.pose);
                first_pose_time = std::chrono::steady_clock::now();
                next_output = first_pose_time;
                next_pose_tx = first_pose_time;
                have_first_pose = true;
                previous_tracker_confidence = latest_t265.tracker_confidence;
            }
            if (!filter.initialized()) continue;

            for (const TimedEncoderFrame &timed : encoder_queue.drain()) {
                omni::WheelIncrement increment;
                std::string integration_reason;
                if (!encoder_integrator.update(timed.frame, increment, integration_reason)) {
                    wheel_gate = integration_reason;
                    continue;
                }
                const omni::WheelGateReason gate =
                    omni::evaluate_wheel_gate(config, latest_t265, increment);
                wheel_gate = omni::wheel_gate_reason_name(gate);
                if (gate == omni::WheelGateReason::Accepted) {
                    filter.predict(increment, config);
                    ++wheel_accepted;
                } else {
                    ++wheel_rejected;
                }
            }

            double innovation_m = 0.0;
            if (previous_tracker_confidence == 0 && latest_t265.tracker_confidence > 0 &&
                have_first_pose) {
                // After a complete visual tracking outage, T265 is the primary
                // absolute source. Re-anchor instead of permanently rejecting a
                // legitimate reacquisition farther than the normal jump gate.
                filter.initialize(latest_t265.pose);
                t265_update_accepted = true;
                wheel_gate = "t265_reacquired";
            } else {
                t265_update_accepted = filter.correct_t265(latest_t265, config, &innovation_m);
            }
            previous_tracker_confidence = latest_t265.tracker_confidence;
            if (!t265_update_accepted && innovation_m > config.maximum_t265_innovation_m) {
                wheel_gate = "t265_jump_rejected";
            }

            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - first_pose_time).count();
            if (have_first_pose && options.duration_sec > 0.0 && elapsed >= options.duration_sec) {
                break;
            }
            const omni::Pose2d fused = filter.pose();
            const std::int64_t last_ns = last_uart_ns.load(std::memory_order_relaxed);
            const bool uart_fresh = !options.uart_path.empty() && last_ns > 0 &&
                static_cast<std::int64_t>(monotonic_ns()) - last_ns <=
                    static_cast<std::int64_t>(config.uart_stale_ms) * 1000000LL;

            if (uart && !options.command_file_path.empty()) {
                std::array<std::uint8_t, omni::kFrameSize> relay_frame{};
                if (read_relay_frame(options.command_file_path, relay_frame) &&
                    (!have_last_relay_frame || relay_frame != last_relay_frame)) {
                    if (uart->write_all(relay_frame.data(), relay_frame.size(), 50)) {
                        ++relay_tx_frames;
                        last_relay_frame = relay_frame;
                        have_last_relay_frame = true;
                    } else {
                        ++relay_tx_errors;
                    }
                }
            }

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
                if (uart->write_all(bytes.data(), bytes.size(), 50)) {
                    ++pose_tx_frames;
                } else {
                    ++pose_tx_errors;
                }
            }
            if (options.output_rate_hz > 0.0 && now < next_output) continue;
            if (options.output_rate_hz > 0.0) {
                next_output = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(output_period);
            }

            const char *quality = quality_name(latest_t265.tracker_confidence,
                                               t265_update_accepted);
            const std::uint64_t uart_frames = live_uart_frames.load(std::memory_order_relaxed);
            const std::uint64_t crc_errors = live_crc_errors.load(std::memory_order_relaxed);
            const std::uint64_t sequence_gaps = live_sequence_gaps.load(std::memory_order_relaxed);
            write_atomic_json(options.output_path, fused, latest_t265, filter, quality,
                              wheel_gate, uart_fresh, wheel_accepted, wheel_rejected,
                              uart_frames, crc_errors, sequence_gaps,
                              pose_tx_frames, pose_tx_errors);

            std::cout << std::fixed << std::setprecision(3)
                      << "POSE t=" << elapsed
                      << " field=(" << fused.x_m << ',' << fused.y_m << ')'
                      << " yaw=" << std::setprecision(1) << omni::degrees(fused.yaw_rad)
                      << "deg conf=" << static_cast<unsigned>(latest_t265.tracker_confidence)
                      << '/' << static_cast<unsigned>(latest_t265.mapper_confidence)
                      << " wheel=" << wheel_gate
                      << " uart=" << (uart_fresh ? "fresh" : "stale")
                      << " tx=" << pose_tx_frames << '/' << pose_tx_errors
                      << " sigma=" << std::setprecision(3) << filter.position_sigma_m() << "m\n";
            if (csv) {
                csv << std::fixed << std::setprecision(9)
                    << elapsed << ',' << fused.x_m << ',' << fused.y_m << ','
                    << omni::degrees(fused.yaw_rad) << ',' << latest_t265.pose.x_m << ','
                    << latest_t265.pose.y_m << ',' << omni::degrees(latest_t265.pose.yaw_rad)
                    << ',' << static_cast<unsigned>(latest_t265.tracker_confidence)
                    << ',' << static_cast<unsigned>(latest_t265.mapper_confidence)
                    << ',' << wheel_gate << ',' << wheel_accepted << ',' << wheel_rejected
                    << ',' << filter.position_sigma_m() << ','
                    << omni::degrees(filter.yaw_sigma_rad()) << '\n';
            }
        }

        pipeline.stop();
        running.store(false, std::memory_order_relaxed);
        if (uart_thread.joinable()) uart_thread.join();
        std::cerr << "[SUMMARY] wheel accepted=" << wheel_accepted
                  << " rejected=" << wheel_rejected
                  << " UART frames=" << final_parser_stats.frames_ok
                  << " crc_errors=" << final_parser_stats.crc_errors
                  << " sequence_gaps=" << final_parser_stats.sequence_gaps
                  << " pose_tx=" << pose_tx_frames
                  << " pose_tx_errors=" << pose_tx_errors
                  << " relay_tx=" << relay_tx_frames
                  << " relay_tx_errors=" << relay_tx_errors
                  << " status_frames=" << final_parser_stats.status_frames
                  << " status_write_errors="
                  << status_write_errors.load(std::memory_order_relaxed)
                  << " queue_dropped=" << encoder_queue.dropped() << '\n';
        return EXIT_SUCCESS;
    } catch (const rs2::error &error) {
        running.store(false, std::memory_order_relaxed);
        if (uart_thread.joinable()) uart_thread.join();
        std::cerr << "[RS2 ERROR] " << error.what() << " ("
                  << error.get_failed_function() << ' ' << error.get_failed_args() << ")\n";
        return 3;
    } catch (const std::exception &error) {
        running.store(false, std::memory_order_relaxed);
        if (uart_thread.joinable()) uart_thread.join();
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
