#include <librealsense2/rs.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "config.hpp"
#include "fusion.hpp"
#include "scan_protocol.hpp"
#include "serial_port.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/stat.h>
#include <X11/Xlib.h>
#include <X11/keysym.h>

namespace {

volatile std::sig_atomic_t g_stop = 0;
constexpr double kPi = 3.14159265358979323846;
constexpr char kWindowName[] = "T265 MAP SCANNER";
constexpr char kMetadataSerial[] = "944222110255";
constexpr char kKnownFirmware[] = "0.2.0.951";
constexpr char kKnownLibrealsense[] = "2.50.0";
constexpr int kActionTurn90 = 1001;
constexpr int kActionTurn180 = 1002;
constexpr int kActionTurn360 = 1003;
constexpr int kActionMoveForward1M = 1004;
constexpr int kActionMoveLeft1M = 1005;
constexpr int kActionReturn = 1006;
constexpr int kActionResetOdom = 1007;
constexpr int kActionStop = 1008;
constexpr int kActionExport = 1009;
constexpr int kActionResetView = 1010;
constexpr int kActionTurnLeft10 = 1011;
constexpr int kActionTurnRight10 = 1012;

void signal_handler(int)
{
    g_stop = 1;
}

struct Options {
    std::string config_path = T265_MAP_DEFAULT_CONFIG;
    std::string session_dir = T265_MAP_DEFAULT_SESSION_ROOT;
    std::string load_map;
    std::string save_map;
    std::string uart = "/dev/ttyS1";
    int baud = 115200;
    double wait_sec = 30.0;
    double relocalization_timeout_sec = 30.0;
    int min_confidence = 2;
    int width = 0;
    int height = 0;
    bool fullscreen = false;
    bool use_uart = true;
    bool enable_motion = false;
    bool allow_pose_jumping = false;
    bool enable_fisheye = true;
};

double parse_nonnegative(const std::string &text, const char *name)
{
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used != text.size() || !std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument("range");
        }
        return value;
    } catch (const std::exception &) {
        throw std::invalid_argument(std::string("invalid value for ") + name + ": " + text);
    }
}

int parse_positive_int(const std::string &text, const char *name)
{
    try {
        std::size_t used = 0;
        const int value = std::stoi(text, &used);
        if (used != text.size() || value <= 0) throw std::invalid_argument("range");
        return value;
    } catch (const std::exception &) {
        throw std::invalid_argument(std::string("invalid value for ") + name + ": " + text);
    }
}

void usage(const char *program)
{
    std::cout
        << "T265 map scanner / localization-map import-export tool\n\n"
        << "Usage: " << program << " [options]\n"
        << "  --load-map FILE          import a T265 localization map before start\n"
        << "  --save-map FILE          export the current map here on save/exit\n"
        << "  --session-dir DIR        directory for metadata and diagnostic logs\n"
        << "  (device selection)       use the first running T265; no serial filter\n"
        << "  --config FILE            localization config for the calibrated lever arm\n"
        << "  --uart DEVICE            listen to F407 ODOM/status (default: /dev/ttyS1)\n"
        << "  --no-uart                do not open the F407 UART\n"
        << "  --enable-motion          enable the explicit 0x19 motion-command buttons\n"
        << "  --baud BAUD              UART baud (default: 115200)\n"
        << "  --wait SEC               wait for T265 (default: 30)\n"
        << "  --relocalization-timeout SEC  verify-mode wait (default: 30)\n"
        << "  --min-confidence N       accept tracker confidence N..3 (default: 2)\n"
        << "  --allow-pose-jumping     allow T265 map pose jumps (diagnostic only)\n"
        << "  --no-fisheye             do not display the two T265 fisheye streams\n"
        << "  --fullscreen              start fullscreen at the detected screen size\n"
        << "  --width PX --height PX   window size when not fullscreen\n"
        << "  -h, --help               show this help\n\n"
        << "Keys: W/S move forward/back 0.5 m, A/D move left/right 0.5 m,\n"
        << "      Left/Right turn 10 deg, X/Space STOP, 1..4 anchors, E export,\n"
        << "      R reset view, F fullscreen, +/- zoom, q or Esc quit.\n";
}

Options parse_options(int argc, char **argv)
{
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        auto value = [&](const char *name) -> std::string {
            if (++index >= argc) {
                throw std::invalid_argument(std::string("missing value for ") + name);
            }
            return argv[index];
        };
        if (argument == "-h" || argument == "--help") {
            usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else if (argument == "--load-map") {
            options.load_map = value("--load-map");
        } else if (argument == "--save-map") {
            options.save_map = value("--save-map");
        } else if (argument == "--session-dir") {
            options.session_dir = value("--session-dir");
        } else if (argument == "--config") {
            options.config_path = value("--config");
        } else if (argument == "--uart") {
            options.uart = value("--uart");
            options.use_uart = true;
        } else if (argument == "--no-uart") {
            options.use_uart = false;
        } else if (argument == "--enable-motion") {
            options.enable_motion = true;
        } else if (argument == "--baud") {
            options.baud = parse_positive_int(value("--baud"), "--baud");
        } else if (argument == "--wait") {
            options.wait_sec = parse_nonnegative(value("--wait"), "--wait");
        } else if (argument == "--relocalization-timeout") {
            options.relocalization_timeout_sec = parse_nonnegative(
                value("--relocalization-timeout"), "--relocalization-timeout");
        } else if (argument == "--min-confidence") {
            options.min_confidence = parse_positive_int(value("--min-confidence"), "--min-confidence");
            if (options.min_confidence > 3) {
                throw std::invalid_argument("--min-confidence must be in 1..3");
            }
        } else if (argument == "--allow-pose-jumping") {
            options.allow_pose_jumping = true;
        } else if (argument == "--no-fisheye") {
            options.enable_fisheye = false;
        } else if (argument == "--fullscreen") {
            options.fullscreen = true;
        } else if (argument == "--width") {
            options.width = parse_positive_int(value("--width"), "--width");
        } else if (argument == "--height") {
            options.height = parse_positive_int(value("--height"), "--height");
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    if (options.enable_motion && !options.use_uart) {
        throw std::invalid_argument("--enable-motion requires an F407 UART");
    }
    if (options.save_map.empty()) {
        options.save_map = options.session_dir + "/t265_localization.raw";
    }
    return options;
}

std::string now_iso8601()
{
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream output;
    output << std::put_time(&local, "%Y-%m-%dT%H:%M:%S%z");
    return output.str();
}

std::string timestamp_directory_name()
{
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream output;
    output << std::put_time(&local, "%Y%m%d_%H%M%S");
    return output.str();
}

double elapsed_seconds(const std::chrono::steady_clock::time_point &start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

std::string json_escape(const std::string &value)
{
    std::ostringstream output;
    for (const unsigned char character : value) {
        switch (character) {
            case '\\': output << "\\\\"; break;
            case '"': output << "\\\""; break;
            case '\n': output << "\\n"; break;
            case '\r': output << "\\r"; break;
            case '\t': output << "\\t"; break;
            default:
                if (character < 0x20u) {
                    output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                           << static_cast<int>(character) << std::dec << std::setfill(' ');
                } else {
                    output << static_cast<char>(character);
                }
        }
    }
    return output.str();
}

bool ensure_directory(const std::string &path)
{
    if (path.empty()) return false;
    std::string partial;
    std::size_t start = 0;
    if (path[0] == '/') {
        partial = "/";
        start = 1;
    }
    while (start <= path.size()) {
        const std::size_t slash = path.find('/', start);
        const std::size_t end = slash == std::string::npos ? path.size() : slash;
        if (end > start) {
            if (!partial.empty() && partial.back() != '/') partial.push_back('/');
            partial.append(path, start, end - start);
            if (::mkdir(partial.c_str(), 0755) != 0 && errno != EEXIST) return false;
        }
        if (slash == std::string::npos) break;
        start = slash + 1;
    }
    return true;
}

std::string parent_directory(const std::string &path)
{
    const std::size_t slash = path.find_last_of('/');
    if (slash == std::string::npos) return ".";
    if (slash == 0) return "/";
    return path.substr(0, slash);
}

bool file_exists(const std::string &path)
{
    struct stat status{};
    return ::stat(path.c_str(), &status) == 0 && S_ISREG(status.st_mode);
}

std::vector<std::uint8_t> read_binary(const std::string &path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open map: " + path);
    input.seekg(0, std::ios::end);
    const std::streamoff size = input.tellg();
    if (size <= 0 || size > static_cast<std::streamoff>(128u * 1024u * 1024u)) {
        throw std::runtime_error("map is empty or larger than 128 MiB: " + path);
    }
    input.seekg(0, std::ios::beg);
    std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
    input.read(reinterpret_cast<char *>(data.data()), size);
    if (!input) throw std::runtime_error("cannot read map: " + path);
    return data;
}

std::string hex_bytes(const std::uint8_t *data, std::size_t size)
{
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (std::size_t index = 0; index < size; ++index) {
        if (index != 0) output << ' ';
        output << std::setw(2) << static_cast<unsigned>(data[index]);
    }
    return output.str();
}

class EventLogger {
public:
    EventLogger(const std::string &path, const std::chrono::steady_clock::time_point &start)
        : start_(start)
    {
        if (!ensure_directory(parent_directory(path))) {
            throw std::runtime_error("cannot create log directory: " + parent_directory(path));
        }
        stream_.open(path, std::ios::out | std::ios::app);
        if (!stream_) throw std::runtime_error("cannot open event log: " + path);
    }

    void write(const std::string &event, const std::string &data = "{}")
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stream_ << "{\"host_time\":\"" << json_escape(now_iso8601())
                << "\",\"elapsed_s\":" << std::fixed << std::setprecision(6)
                << elapsed_seconds(start_) << ",\"event\":\"" << json_escape(event)
                << "\",\"data\":" << (data.empty() ? "{}" : data) << "}\n";
        stream_.flush();
    }

private:
    std::ofstream stream_;
    std::chrono::steady_clock::time_point start_;
    std::mutex mutex_;
};

struct MetadataState {
    bool map_imported = false;
    bool map_exported = false;
    std::size_t map_size_bytes = 0;
    std::string map_error;
    int relocalization_events = 0;
    std::map<int, bool> anchors;
};

std::string json_bool(bool value)
{
    return value ? "true" : "false";
}

void write_metadata(const std::string &path, const Options &options,
                    const omni::LocalizationConfig &config,
                    const std::string &actual_serial,
                    const std::string &actual_firmware,
                    const MetadataState &state,
                    bool fisheye_active)
{
    if (!ensure_directory(parent_directory(path))) {
        throw std::runtime_error("cannot create metadata directory: " + parent_directory(path));
    }
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) throw std::runtime_error("cannot write metadata: " + path);
    output << std::fixed << std::setprecision(7);
    output
        << "{\n"
        << "  \"schema_version\": 1,\n"
        << "  \"updated_at\": \"" << json_escape(now_iso8601()) << "\",\n"
        << "  \"mode\": \"" << (options.load_map.empty() ? "scan" : "verify") << "\",\n"
        << "  \"map_file\": \"" << json_escape(options.save_map) << "\",\n"
        << "  \"map_loaded_from\": \"" << json_escape(options.load_map) << "\",\n"
        << "  \"map_imported\": " << json_bool(state.map_imported) << ",\n"
        << "  \"map_exported\": " << json_bool(state.map_exported) << ",\n"
        << "  \"map_size_bytes\": " << state.map_size_bytes << ",\n"
        << "  \"map_export_error\": \"" << json_escape(state.map_error) << "\",\n"
        << "  \"t265_serial\": \"" << json_escape(actual_serial.empty() ? kMetadataSerial : actual_serial) << "\",\n"
        << "  \"t265_firmware\": \"" << json_escape(actual_firmware.empty() ? kKnownFirmware : actual_firmware) << "\",\n"
        << "  \"librealsense_version\": \"" << kKnownLibrealsense << "\",\n"
        << "  \"camera_offset_units\": \"metres\",\n"
        << "  \"camera_offset_reference\": \"T265 stereo-imager tracking origin relative to the three-wheel kinematic rotation centre\",\n"
        << "  \"camera_offset_forward_m\": " << config.camera_offset_forward_m << ",\n"
        << "  \"camera_offset_left_m\": " << config.camera_offset_left_m << ",\n"
        << "  \"installation_axes\": {\n"
        << "    \"description\": \"T265 lens-up installation; axes are robot axes expressed in T265 Pose coordinates\",\n"
        << "    \"robot_forward\": \"+X\",\n"
        << "    \"robot_left\": \"-Y\",\n"
        << "    \"robot_up\": \"-Z\",\n"
        << "    \"t265_native_x\": \"right\",\n"
        << "    \"t265_native_y\": \"up\",\n"
        << "    \"t265_native_z\": \"backward\"\n"
        << "  },\n"
        << "  \"t265_options\": {\n"
        << "    \"enable_mapping\": true,\n"
        << "    \"enable_relocalization\": true,\n"
        << "    \"enable_pose_jumping\": " << json_bool(options.allow_pose_jumping) << ",\n"
        << "    \"enable_map_preservation\": false,\n"
        << "    \"fisheye_streams_active\": " << json_bool(fisheye_active) << "\n"
        << "  },\n"
        << "  \"relocalization_event_count\": " << state.relocalization_events << ",\n"
        << "  \"static_nodes\": {\n"
        << "    \"scan_anchor_1\": " << json_bool(state.anchors.count(1) != 0 && state.anchors.at(1)) << ",\n"
        << "    \"scan_anchor_2\": " << json_bool(state.anchors.count(2) != 0 && state.anchors.at(2)) << ",\n"
        << "    \"scan_anchor_3\": " << json_bool(state.anchors.count(3) != 0 && state.anchors.at(3)) << ",\n"
        << "    \"scan_anchor_4\": " << json_bool(state.anchors.count(4) != 0 && state.anchors.at(4)) << "\n"
        << "  },\n"
        << "  \"motion_interface\": {\n"
        << "    \"uart\": \"" << json_escape(options.uart) << "\",\n"
        << "    \"baud\": " << options.baud << ",\n"
        << "    \"command_type\": \"0x19\",\n"
        << "    \"status_type\": \"0x1A\",\n"
        << "    \"motion_enabled\": " << json_bool(options.enable_motion) << ",\n"
        << "    \"frame\": \"A3 B3 TYPE SEQ P0..P7 CRC_LO CRC_HI C3, CRC-16/Modbus over TYPE..P7\"\n"
        << "  },\n"
        << "  \"logs\": {\n"
        << "    \"pose_csv\": \"pose.csv\",\n"
        << "    \"encoder_csv\": \"encoder.csv\",\n"
        << "    \"status_csv\": \"f407_status.csv\",\n"
        << "    \"events_jsonl\": \"events.jsonl\"\n"
        << "  },\n"
        << "  \"notes\": [\n"
        << "    \"The map is T265's visual localization map, not a CAD or occupancy map.\",\n"
        << "    \"The scanner does not fuse wheel odometry into T265; it logs both references for comparison.\",\n"
        << "    \"The camera offset is applied once by the existing robot-centre lever-arm projector.\"\n"
        << "  ]\n"
        << "}\n";
}

struct OdomSnapshot {
    bool has_frame = false;
    bool increment_accepted = false;
    bool yaw_valid = false;
    std::uint64_t frame_count = 0;
    std::uint8_t sequence = 0;
    std::uint16_t count[3] = {0, 0, 0};
    std::int32_t delta_count[3] = {0, 0, 0};
    std::uint8_t dt_ms = 0;
    std::uint8_t status = 0;
    double body_forward_increment_m = 0.0;
    double body_left_increment_m = 0.0;
    double wheel_center_forward_m = 0.0;
    double wheel_center_left_m = 0.0;
    double wheel_path_m = 0.0;
    double t265_yaw_rad = 0.0;
    double host_time_s = 0.0;
};

class WheelOdomTracker {
public:
    OdomSnapshot update(const t265_map::protocol::OdomPayload &odom,
                        double t265_yaw_rad, bool t265_yaw_valid,
                        double host_time_s)
    {
        OdomSnapshot result;
        result.has_frame = true;
        result.frame_count = ++frame_count_;
        result.sequence = odom.sequence;
        result.count[0] = odom.m1_count;
        result.count[1] = odom.m2_count;
        result.count[2] = odom.m3_count;
        result.dt_ms = odom.dt_ms;
        result.status = odom.status;
        result.t265_yaw_rad = t265_yaw_rad;
        result.yaw_valid = t265_yaw_valid;
        result.host_time_s = host_time_s;
        // Preserve the last accepted wheel-centre position across a reset,
        // invalid sample, or encoder-fault frame. The CSV still records the
        // rejected frame, but the live comparison must not jump back to 0.
        result.wheel_center_forward_m = wheel_center_forward_m_;
        result.wheel_center_left_m = wheel_center_left_m_;
        result.wheel_path_m = wheel_path_m_;

        const bool counter_reset = (odom.status & (1u << 3)) != 0u;
        const bool encoder_fault = (odom.status & (1u << 4)) != 0u;
        const bool all_valid = (odom.status & 0x07u) == 0x07u && !encoder_fault;
        if (!have_baseline_ || counter_reset) {
            previous_[0] = odom.m1_count;
            previous_[1] = odom.m2_count;
            previous_[2] = odom.m3_count;
            have_baseline_ = true;
            last_accepted_ = false;
            result.increment_accepted = false;
            return result;
        }

        for (int wheel = 0; wheel < 3; ++wheel) {
            result.delta_count[wheel] = signed_count_delta(
                odom_value(odom, wheel), previous_[wheel]);
            previous_[wheel] = odom_value(odom, wheel);
        }
        if (!all_valid) {
            last_accepted_ = false;
            return result;
        }

        constexpr double wheel_diameter_m = 0.070;
        constexpr double counts_per_revolution = 1768.0;
        constexpr double distance_per_count = kPi * wheel_diameter_m / counts_per_revolution;
        constexpr double sqrt3 = 1.73205080756887729353;
        // F407 convention: M1=right, M2=left, M3=rear. All three raw
        // encoder signs are -1, matching localization.example.conf and the
        // current lower-side Location.c implementation.
        const double m1 = -static_cast<double>(result.delta_count[0]) * distance_per_count;
        const double m2 = -static_cast<double>(result.delta_count[1]) * distance_per_count;
        const double m3 = -static_cast<double>(result.delta_count[2]) * distance_per_count;
        const double body_forward = (m1 - m2) / sqrt3;
        const double body_left = (m1 + m2 - 2.0 * m3) / 3.0;
        result.body_forward_increment_m = body_forward;
        result.body_left_increment_m = body_left;

        const double c = std::cos(t265_yaw_rad);
        const double s = std::sin(t265_yaw_rad);
        const double initial_forward = body_forward * c - body_left * s;
        const double initial_left = body_forward * s + body_left * c;
        result.wheel_center_forward_m = wheel_center_forward_m_ + initial_forward;
        result.wheel_center_left_m = wheel_center_left_m_ + initial_left;
        result.wheel_path_m = wheel_path_m_ + std::hypot(body_forward, body_left);
        wheel_center_forward_m_ = result.wheel_center_forward_m;
        wheel_center_left_m_ = result.wheel_center_left_m;
        wheel_path_m_ = result.wheel_path_m;
        result.increment_accepted = true;
        last_accepted_ = true;
        return result;
    }

    void reset_view()
    {
        wheel_center_forward_m_ = 0.0;
        wheel_center_left_m_ = 0.0;
        wheel_path_m_ = 0.0;
    }

private:
    static std::uint16_t odom_value(const t265_map::protocol::OdomPayload &odom, int wheel)
    {
        return wheel == 0 ? odom.m1_count : wheel == 1 ? odom.m2_count : odom.m3_count;
    }

    static std::int32_t signed_count_delta(std::uint16_t current, std::uint16_t previous)
    {
        std::int32_t delta = static_cast<std::int32_t>(current) -
                             static_cast<std::int32_t>(previous);
        if (delta > 32767) delta -= 65536;
        if (delta < -32768) delta += 65536;
        return delta;
    }

    bool have_baseline_ = false;
    bool last_accepted_ = false;
    std::uint16_t previous_[3] = {0, 0, 0};
    std::uint64_t frame_count_ = 0;
    double wheel_center_forward_m_ = 0.0;
    double wheel_center_left_m_ = 0.0;
    double wheel_path_m_ = 0.0;
};

struct StatusSnapshot {
    bool has_status = false;
    t265_map::protocol::MotionStatusPayload status{};
    double host_time_s = 0.0;
};

class MotionLink {
public:
    MotionLink(const Options &options, EventLogger &events,
               const std::chrono::steady_clock::time_point &start,
               const std::string &encoder_csv_path,
               const std::string &status_csv_path)
        : options_(options), events_(events), start_(start),
          parser_([this](std::uint8_t type, std::uint8_t sequence,
                         const std::array<std::uint8_t, t265_map::protocol::kPayloadSize> &payload) {
              on_frame(type, sequence, payload);
          })
    {
        encoder_csv_.open(encoder_csv_path, std::ios::out | std::ios::trunc);
        if (!encoder_csv_) throw std::runtime_error("cannot open encoder.csv");
        encoder_csv_ << "host_time_s,sequence,m1_count,m2_count,m3_count,dt_ms,status,"
                        "delta_m1_count,delta_m2_count,delta_m3_count,"
                        "body_forward_increment_m,body_left_increment_m,"
                        "wheel_center_forward_m,wheel_center_left_m,wheel_path_m,"
                        "t265_relative_yaw_deg,yaw_reference_valid,increment_accepted\n";
        status_csv_.open(status_csv_path, std::ios::out | std::ios::trunc);
        if (!status_csv_) throw std::runtime_error("cannot open f407_status.csv");
        status_csv_ << "host_time_s,frame_sequence,command_sequence,state,fault,command,"
                       "progress,heading_cdeg\n";
    }

    ~MotionLink()
    {
        stop();
    }

    void start()
    {
        port_.reset(new omni::SerialPort(options_.uart, options_.baud));
        port_->open_port();
        running_.store(true);
        reader_ = std::thread(&MotionLink::read_loop, this);
    }

    void stop()
    {
        running_.store(false);
        if (reader_.joinable()) reader_.join();
        if (port_) port_->close_port();
        port_.reset();
    }

    bool send(const t265_map::protocol::MotionCommandPayload &payload,
              std::uint8_t &sequence)
    {
        if (!port_) return false;
        sequence = next_sequence_++;
        const auto frame = t265_map::protocol::build_motion_command(sequence, payload);
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (!port_->write_all(frame.data(), frame.size(), 250)) {
            set_error("UART write failed");
            return false;
        }
        std::ostringstream data;
        data << "{\"sequence\":" << static_cast<unsigned>(sequence)
             << ",\"command\":\""
             << t265_map::protocol::motion_command_name(payload.command)
             << "\",\"command_code\":" << static_cast<unsigned>(payload.command)
             << ",\"flags\":" << static_cast<unsigned>(payload.flags)
             << ",\"arg1\":" << payload.arg1 << ",\"arg2\":" << payload.arg2
             << ",\"speed\":" << payload.speed
             << ",\"frame_hex\":\"" << json_escape(hex_bytes(frame.data(), frame.size())) << "\"}";
        events_.write("motion_command", data.str());
        return true;
    }

    bool send_stop()
    {
        t265_map::protocol::MotionCommandPayload payload;
        payload.command = t265_map::protocol::kMotionStop;
        payload.flags = t265_map::protocol::kFlagValid | t265_map::protocol::kFlagAckRequired;
        std::uint8_t sequence = 0;
        return send(payload, sequence);
    }

    void set_reference_yaw(double yaw_rad, bool valid)
    {
        reference_yaw_rad_.store(yaw_rad);
        reference_yaw_valid_.store(valid);
    }

    OdomSnapshot odom_snapshot() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return latest_odom_;
    }

    StatusSnapshot status_snapshot() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return latest_status_;
    }

    void reset_wheel_view()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        wheel_.reset_view();
        latest_odom_.wheel_center_forward_m = 0.0;
        latest_odom_.wheel_center_left_m = 0.0;
        latest_odom_.wheel_path_m = 0.0;
    }

    std::string error() const
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return error_;
    }

    std::uint64_t odom_frames() const noexcept { return odom_frames_.load(); }
    std::uint64_t status_frames() const noexcept { return status_frames_.load(); }
    std::uint64_t crc_errors() const noexcept { return crc_errors_.load(); }

private:
    void set_error(const std::string &error)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        error_ = error;
    }

    void read_loop()
    {
        std::array<std::uint8_t, 512> buffer{};
        while (running_.load()) {
            if (!port_) break;
            const int count = port_->read_some(buffer.data(), buffer.size(), 50);
            if (count < 0) {
                set_error("UART read failed");
                break;
            }
            if (count == 0) continue;
            parser_.feed(buffer.data(), static_cast<std::size_t>(count));
            crc_errors_.store(parser_.crc_errors());
        }
    }

    void on_frame(std::uint8_t type, std::uint8_t sequence,
                  const std::array<std::uint8_t, t265_map::protocol::kPayloadSize> &payload)
    {
        const double host_time = elapsed_seconds(start_);
        if (type == t265_map::protocol::kMsgOdom) {
            t265_map::protocol::OdomPayload odom;
            if (!t265_map::protocol::decode_odom(sequence, payload, odom)) return;
            const double yaw = reference_yaw_rad_.load();
            const bool yaw_valid = reference_yaw_valid_.load();
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_odom_ = wheel_.update(odom, yaw, yaw_valid, host_time);
            encoder_csv_ << std::fixed << std::setprecision(6)
                         << host_time << ',' << static_cast<unsigned>(odom.sequence) << ','
                         << odom.m1_count << ',' << odom.m2_count << ',' << odom.m3_count << ','
                         << static_cast<unsigned>(odom.dt_ms) << ',' << static_cast<unsigned>(odom.status) << ','
                         << latest_odom_.delta_count[0] << ',' << latest_odom_.delta_count[1] << ','
                         << latest_odom_.delta_count[2] << ',' << latest_odom_.body_forward_increment_m << ','
                         << latest_odom_.body_left_increment_m << ',' << latest_odom_.wheel_center_forward_m << ','
                         << latest_odom_.wheel_center_left_m << ',' << latest_odom_.wheel_path_m << ','
                         << yaw * 180.0 / kPi << ',' << (yaw_valid ? 1 : 0) << ','
                         << (latest_odom_.increment_accepted ? 1 : 0) << '\n';
            encoder_csv_.flush();
            odom_frames_.fetch_add(1);
        } else if (type == t265_map::protocol::kMsgScanMotionStatus) {
            t265_map::protocol::MotionStatusPayload status;
            if (!t265_map::protocol::decode_motion_status(sequence, payload, status)) return;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_status_.has_status = true;
                latest_status_.status = status;
                latest_status_.host_time_s = host_time;
                status_csv_ << std::fixed << std::setprecision(6)
                            << host_time << ',' << static_cast<unsigned>(status.frame_sequence) << ','
                            << static_cast<unsigned>(status.acknowledged_command_sequence) << ','
                            << static_cast<unsigned>(status.state) << ','
                            << static_cast<unsigned>(status.fault) << ','
                            << static_cast<unsigned>(status.command) << ','
                            << status.progress << ',' << status.heading_cdeg << '\n';
                status_csv_.flush();
            }
            std::ostringstream data;
            data << "{\"frame_sequence\":" << static_cast<unsigned>(status.frame_sequence)
                 << ",\"command_sequence\":" << static_cast<unsigned>(status.acknowledged_command_sequence)
                 << ",\"state\":\"" << t265_map::protocol::motion_state_name(status.state)
                 << "\",\"state_code\":" << static_cast<unsigned>(status.state)
                 << ",\"fault\":" << static_cast<unsigned>(status.fault)
                 << ",\"command\":\"" << t265_map::protocol::motion_command_name(status.command)
                 << "\",\"progress\":" << status.progress
                 << ",\"heading_cdeg\":" << status.heading_cdeg << "}";
            events_.write("motion_status", data.str());
            status_frames_.fetch_add(1);
        } else if (type == t265_map::protocol::kMsgLegacyStmStatus) {
            std::ostringstream data;
            data << "{\"sequence\":" << static_cast<unsigned>(sequence)
                 << ",\"payload_hex\":\"" << json_escape(hex_bytes(payload.data(), payload.size())) << "\"}";
            events_.write("legacy_stm_status", data.str());
        }
    }

    const Options &options_;
    EventLogger &events_;
    std::chrono::steady_clock::time_point start_;
    std::unique_ptr<omni::SerialPort> port_;
    std::thread reader_;
    std::atomic<bool> running_{false};
    std::mutex write_mutex_;
    mutable std::mutex data_mutex_;
    t265_map::protocol::StreamParser parser_;
    WheelOdomTracker wheel_;
    OdomSnapshot latest_odom_{};
    StatusSnapshot latest_status_{};
    std::ofstream encoder_csv_;
    std::ofstream status_csv_;
    std::atomic<double> reference_yaw_rad_{0.0};
    std::atomic<bool> reference_yaw_valid_{false};
    std::atomic<std::uint64_t> odom_frames_{0};
    std::atomic<std::uint64_t> status_frames_{0};
    std::atomic<std::uint64_t> crc_errors_{0};
    std::uint8_t next_sequence_ = 0;
    std::string error_;
};

struct Point2 {
    double left_m = 0.0;
    double forward_m = 0.0;
};

struct Button {
    std::string id;
    std::string label;
    cv::Rect rect;
    bool enabled = true;
};

struct MouseContext {
    std::vector<Button> *buttons = nullptr;
    std::string *clicked = nullptr;
};

void on_mouse(int event, int x, int y, int, void *userdata)
{
    if (event != cv::EVENT_LBUTTONDOWN || userdata == nullptr) return;
    auto *context = static_cast<MouseContext *>(userdata);
    if (context->buttons == nullptr || context->clicked == nullptr) return;
    for (const Button &button : *context->buttons) {
        if (button.enabled && button.rect.contains(cv::Point(x, y))) {
            *context->clicked = button.id;
            return;
        }
    }
}

std::pair<int, int> detect_screen_size()
{
    std::pair<int, int> result{1280, 900};
    FILE *pipe = ::popen("xrandr --current 2>/dev/null", "r");
    if (pipe == nullptr) return result;
    char line[256]{};
    while (std::fgets(line, sizeof(line), pipe) != nullptr) {
        int width = 0;
        int height = 0;
        if (std::sscanf(line, " Screen 0: minimum %*d x %*d, current %d x %d",
                        &width, &height) == 2 && width > 0 && height > 0) {
            result = {width, height};
            break;
        }
    }
    ::pclose(pipe);
    return result;
}

int normalize_key(int key)
{
    // OpenCV/X11 reports arrow keys differently depending on the backend:
    // waitKeyEx commonly returns 0x250000/0x270000, while some backends
    // return 81/83 or the X11 keysyms 65361/65363.
    switch (key) {
        case 81:
        case 0x250000:
        case 0x01000012:
        case 65361:
            return kActionTurnLeft10;
        case 83:
        case 0x270000:
        case 0x01000014:
        case 65363:
            return kActionTurnRight10;
        default:
            return key & 0xFF;
    }
}

class GlobalArrowInput {
public:
    GlobalArrowInput()
    {
        display_ = XOpenDisplay(nullptr);
        if (display_ == nullptr) return;
        left_keycode_ = XKeysymToKeycode(display_, XK_Left);
        right_keycode_ = XKeysymToKeycode(display_, XK_Right);
    }

    ~GlobalArrowInput()
    {
        if (display_ != nullptr) XCloseDisplay(display_);
    }

    int poll_pressed_edge()
    {
        if (display_ == nullptr || left_keycode_ == 0 || right_keycode_ == 0) return 0;
        char keymap[32]{};
        XQueryKeymap(display_, keymap);
        const bool left_pressed = key_is_down(keymap, left_keycode_);
        const bool right_pressed = key_is_down(keymap, right_keycode_);
        int action = 0;
        if (left_pressed && !left_was_pressed_) {
            action = kActionTurnLeft10;
        } else if (right_pressed && !right_was_pressed_) {
            action = kActionTurnRight10;
        }
        left_was_pressed_ = left_pressed;
        right_was_pressed_ = right_pressed;
        return action;
    }

private:
    static bool key_is_down(const char keymap[32], KeyCode keycode)
    {
        const unsigned int index = static_cast<unsigned int>(keycode) / 8u;
        const unsigned int bit = static_cast<unsigned int>(keycode) % 8u;
        return index < 32u && (static_cast<unsigned char>(keymap[index]) & (1u << bit)) != 0u;
    }

    Display *display_ = nullptr;
    KeyCode left_keycode_ = 0;
    KeyCode right_keycode_ = 0;
    bool left_was_pressed_ = false;
    bool right_was_pressed_ = false;
};

int text_baseline(const cv::Mat &canvas, int line)
{
    return std::min(canvas.rows - 8, 30 + line * 22);
}

void put_text(cv::Mat &canvas, const std::string &text, int x, int y,
              double scale = 0.52, const cv::Scalar &color = cv::Scalar(225, 225, 225),
              int thickness = 1)
{
    cv::putText(canvas, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv::LINE_AA);
}

double grid_step_m(double scale)
{
    static const double steps[] = {0.05, 0.10, 0.20, 0.25, 0.50, 1.0, 2.0, 5.0};
    for (const double step : steps) {
        if (step * scale >= 55.0) return step;
    }
    return 10.0;
}

cv::Point map_pixel(const Point2 &point, int left, int top, int size, double scale)
{
    return cv::Point(static_cast<int>(std::lround(left + size * 0.5 + point.left_m * scale)),
                     static_cast<int>(std::lround(top + size * 0.5 - point.forward_m * scale)));
}

void draw_path(cv::Mat &canvas, const std::deque<Point2> &path,
               int left, int top, int size, double scale,
               const cv::Scalar &color, int thickness)
{
    if (path.size() < 2) return;
    for (std::size_t index = 1; index < path.size(); ++index) {
        cv::line(canvas, map_pixel(path[index - 1], left, top, size, scale),
                 map_pixel(path[index], left, top, size, scale), color, thickness, cv::LINE_AA);
    }
}

void draw_button(cv::Mat &canvas, const Button &button)
{
    const cv::Scalar fill = button.enabled ? cv::Scalar(55, 70, 86) : cv::Scalar(38, 42, 48);
    const cv::Scalar border = button.enabled ? cv::Scalar(130, 175, 215) : cv::Scalar(75, 75, 75);
    cv::rectangle(canvas, button.rect, fill, cv::FILLED, cv::LINE_AA);
    cv::rectangle(canvas, button.rect, border, 1, cv::LINE_AA);
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(button.label, cv::FONT_HERSHEY_SIMPLEX,
                                               0.46, 1, &baseline);
    const int x = button.rect.x + (button.rect.width - text_size.width) / 2;
    const int y = button.rect.y + (button.rect.height + text_size.height) / 2;
    put_text(canvas, button.label, x, y, 0.46,
             button.enabled ? cv::Scalar(240, 240, 240) : cv::Scalar(125, 125, 125), 1);
}

cv::Mat fisheye_bgr(const rs2::video_frame &frame)
{
    if (!frame) return cv::Mat();
    const int width = frame.get_width();
    const int height = frame.get_height();
    if (width <= 0 || height <= 0 || frame.get_data() == nullptr) return cv::Mat();
    cv::Mat gray(height, width, CV_8UC1, const_cast<void *>(frame.get_data()),
                frame.get_stride_in_bytes());
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr.clone();
}

struct UiState {
    double scale = 180.0;
    bool fullscreen = false;
    std::string message = "T265 ready; press E to export map";
    std::deque<Point2> raw_path;
    std::deque<Point2> corrected_path;
    std::deque<Point2> wheel_path;
    std::uint64_t last_odom_point = 0;
};

void add_button(std::vector<Button> &buttons, const std::string &id,
                const std::string &label, int x, int y, int width, int height,
                bool enabled)
{
    buttons.push_back(Button{id, label, cv::Rect(x, y, width, height), enabled});
}

void draw_interface(cv::Mat &canvas, const Options &options, const std::string &serial,
                    const std::string &firmware, const omni::LocalizationConfig &config,
                    const MetadataState &metadata, const UiState &ui,
                    const omni::T265FieldPose *pose, bool have_pose,
                    const OdomSnapshot &odom, const StatusSnapshot &motion_status,
                    bool uart_connected, const std::string &uart_error,
                    const cv::Mat &left_fisheye, const cv::Mat &right_fisheye,
                    std::vector<Button> &buttons)
{
    canvas.setTo(cv::Scalar(22, 25, 30));
    buttons.clear();
    const int panel_width = std::max(315, canvas.cols / 3);
    const int map_left = 18;
    const int map_top = 75;
    const int map_size = std::max(300, std::min(canvas.rows - 150,
                                                canvas.cols - panel_width - 48));
    const int panel_x = map_left + map_size + 24;
    const int panel_right = canvas.cols - 14;
    const int button_gap = 8;
    const int button_width = std::max(120, (panel_right - panel_x - button_gap) / 2);
    const int button_height = 39;

    put_text(canvas, "T265 MAP SCANNER", 18, 35, 0.82, cv::Scalar(245, 245, 245), 2);
    put_text(canvas, options.load_map.empty() ? "SCAN / BUILD MAP" : "VERIFY / RELOCALIZE MAP",
             panel_x, 35, 0.55, cv::Scalar(100, 220, 255), 2);

    cv::rectangle(canvas, cv::Rect(map_left, map_top, map_size, map_size),
                  cv::Scalar(35, 40, 47), cv::FILLED);
    const cv::Point map_origin(map_left + map_size / 2, map_top + map_size / 2);
    const double step = grid_step_m(ui.scale);
    const int step_px = std::max(1, static_cast<int>(std::lround(step * ui.scale)));
    for (int x = map_origin.x; x < map_left + map_size; x += step_px)
        cv::line(canvas, cv::Point(x, map_top), cv::Point(x, map_top + map_size), cv::Scalar(54, 59, 66), 1);
    for (int x = map_origin.x - step_px; x >= map_left; x -= step_px)
        cv::line(canvas, cv::Point(x, map_top), cv::Point(x, map_top + map_size), cv::Scalar(54, 59, 66), 1);
    for (int y = map_origin.y; y < map_top + map_size; y += step_px)
        cv::line(canvas, cv::Point(map_left, y), cv::Point(map_left + map_size, y), cv::Scalar(54, 59, 66), 1);
    for (int y = map_origin.y - step_px; y >= map_top; y -= step_px)
        cv::line(canvas, cv::Point(map_left, y), cv::Point(map_left + map_size, y), cv::Scalar(54, 59, 66), 1);
    cv::line(canvas, cv::Point(map_origin.x, map_top), cv::Point(map_origin.x, map_top + map_size),
             cv::Scalar(120, 125, 135), 1);
    cv::line(canvas, cv::Point(map_left, map_origin.y), cv::Point(map_left + map_size, map_origin.y),
             cv::Scalar(120, 125, 135), 1);

    draw_path(canvas, ui.raw_path, map_left, map_top, map_size, ui.scale,
              cv::Scalar(0, 145, 255), 2);
    draw_path(canvas, ui.wheel_path, map_left, map_top, map_size, ui.scale,
              cv::Scalar(255, 120, 50), 2);
    draw_path(canvas, ui.corrected_path, map_left, map_top, map_size, ui.scale,
              cv::Scalar(65, 225, 95), 3);
    cv::circle(canvas, map_origin, 5, cv::Scalar(245, 245, 245), cv::FILLED, cv::LINE_AA);
    put_text(canvas, "+forward", map_origin.x + 8, map_top + 23, 0.42, cv::Scalar(180, 210, 180));
    put_text(canvas, "+left", map_left + map_size - 70, map_origin.y - 8, 0.42, cv::Scalar(180, 210, 180));
    put_text(canvas, "green=center  orange=raw T265  blue=wheel", map_left + 12,
             map_top + map_size - 12, 0.42, cv::Scalar(210, 210, 210));
    put_text(canvas, cv::format("scale %.0f px/m", ui.scale), map_left + 12,
             map_top + map_size + 27, 0.46, cv::Scalar(190, 190, 190));

    int line = 0;
    auto panel_line = [&](const std::string &value, const cv::Scalar &color = cv::Scalar(220, 220, 220)) {
        put_text(canvas, value, panel_x, text_baseline(canvas, line++), 0.45, color, 1);
    };
    panel_line("T265 " + serial);
    panel_line("FW " + firmware + "  SDK " + kKnownLibrealsense);
    panel_line(std::string("map ") + (options.load_map.empty() ? "internal mapping" : "imported map") +
               (metadata.map_exported ? " / exported" : " / not exported"));
    panel_line("relocalization events " + std::to_string(metadata.relocalization_events),
               options.load_map.empty() || metadata.relocalization_events > 0
                   ? cv::Scalar(145, 230, 160) : cv::Scalar(0, 165, 255));
    const int tracker = have_pose ? static_cast<int>(pose->tracker_confidence) : 0;
    const int mapper = have_pose ? static_cast<int>(pose->mapper_confidence) : 0;
    panel_line(cv::format("tracker %d   mapper %d   anchors 1:%d 2:%d 3:%d 4:%d",
                          tracker, mapper,
                          metadata.anchors.count(1) && metadata.anchors.at(1),
                          metadata.anchors.count(2) && metadata.anchors.at(2),
                          metadata.anchors.count(3) && metadata.anchors.at(3),
                          metadata.anchors.count(4) && metadata.anchors.at(4)),
               tracker >= options.min_confidence ? cv::Scalar(145, 230, 160) : cv::Scalar(0, 165, 255));
    panel_line(cv::format("offset F %+.4f  L %+.4f m", config.camera_offset_forward_m,
                          config.camera_offset_left_m));
    panel_line("axes F +X  L -Y  U -Z (lens-up)");
    if (have_pose) {
        panel_line(cv::format("raw dF %+.3f  dL %+.3f m",
                              pose->tracking_origin_delta_forward_m,
                              pose->tracking_origin_delta_left_m));
        panel_line(cv::format("center dF %+.3f  dL %+.3f m",
                              pose->robot_center_delta_forward_m,
                              pose->robot_center_delta_left_m));
        panel_line(cv::format("yaw %+.1f deg  gyro %+.1f deg",
                              pose->relative_yaw_rad * 180.0 / kPi,
                              pose->gyro_relative_yaw_rad * 180.0 / kPi));
    } else {
        panel_line("pose waiting for tracker...");
        panel_line("center correction not initialized");
    }
    if (odom.has_frame) {
        panel_line(cv::format("wheel dF %+.3f  dL %+.3f m",
                              odom.wheel_center_forward_m, odom.wheel_center_left_m));
        panel_line(cv::format("wheel path %.3f m  ODOM %llu",
                              odom.wheel_path_m,
                              static_cast<unsigned long long>(odom.frame_count)));
    } else {
        panel_line("wheel ODOM waiting...");
    }
    const std::string uart_text = uart_connected
        ? "UART connected / read-only"
        : (options.use_uart ? "UART unavailable" : "UART disabled");
    panel_line(uart_text, uart_connected ? cv::Scalar(145, 230, 160) : cv::Scalar(0, 165, 255));
    if (!uart_error.empty()) panel_line(uart_error, cv::Scalar(0, 165, 255));
    if (motion_status.has_status) {
        panel_line(std::string("F407 ") + t265_map::protocol::motion_state_name(motion_status.status.state) +
                   " cmd " + std::to_string(motion_status.status.acknowledged_command_sequence),
                   cv::Scalar(155, 205, 255));
    } else {
        panel_line(options.enable_motion ? "motion enabled / waiting 0x1A" : "motion OFF (use --enable-motion)",
                   options.enable_motion ? cv::Scalar(0, 190, 255) : cv::Scalar(180, 180, 180));
    }

    const int buttons_top = std::min(canvas.rows - 210, map_top + 270);
    const bool motion_buttons = options.enable_motion && uart_connected;
    const std::vector<std::pair<std::string, std::string>> button_defs = {
        {"turn90", "TURN +90"}, {"turn180", "TURN +180"},
        {"turn360", "TURN +360"}, {"move_forward", "MOVE FWD 1M"},
        {"move_left", "MOVE LEFT 1M"}, {"return", "OUT 1M + BACK"},
        {"reset_odom", "RESET ODOM"}, {"stop", "STOP / E-STOP"},
        {"export", "EXPORT MAP"}, {"reset_view", "RESET VIEW"},
    };
    for (std::size_t index = 0; index < button_defs.size(); ++index) {
        const int row = static_cast<int>(index / 2);
        const int column = static_cast<int>(index % 2);
        const int x = panel_x + column * (button_width + button_gap);
        const int y = buttons_top + row * (button_height + button_gap);
        const bool enabled = button_defs[index].first == "export" ||
                             button_defs[index].first == "reset_view" ||
                             (button_defs[index].first == "stop" && uart_connected) || motion_buttons;
        add_button(buttons, button_defs[index].first, button_defs[index].second,
                   x, y, button_width, button_height, enabled);
        draw_button(canvas, buttons.back());
    }
    const int control_help_top = buttons_top + 5 * (button_height + button_gap) + 20;
    put_text(canvas, "W/S: F/B 0.5M   A/D: L/R 0.5M", panel_x,
             control_help_top, 0.40, cv::Scalar(185, 185, 185));
    put_text(canvas, "<-/->: TURN 10 DEG   X/SPACE: STOP", panel_x,
             control_help_top + 21, 0.38, cv::Scalar(185, 185, 185));
    put_text(canvas, "1..4 anchors  E export  R view  F full  +/- zoom  q quit",
             panel_x, control_help_top + 42, 0.36, cv::Scalar(185, 185, 185));
    put_text(canvas, ui.message, panel_x, canvas.rows - 20, 0.47, cv::Scalar(80, 220, 255), 1);

    const int preview_top = std::min(canvas.rows - 155, control_help_top + 58);
    const int preview_height = std::max(70, canvas.rows - preview_top - 28);
    const int preview_width = (panel_right - panel_x - 8) / 2;
    auto draw_preview = [&](const cv::Mat &image, int x, const std::string &label) {
        cv::Rect target(x, preview_top, preview_width, preview_height);
        cv::rectangle(canvas, target, cv::Scalar(45, 48, 55), cv::FILLED);
        if (!image.empty()) {
            cv::Mat resized;
            cv::resize(image, resized, target.size(), 0.0, 0.0, cv::INTER_AREA);
            resized.copyTo(canvas(target));
        }
        put_text(canvas, label, x + 5, preview_top + 18, 0.40, cv::Scalar(245, 245, 245), 1);
    };
    draw_preview(left_fisheye, panel_x, "fisheye 1");
    draw_preview(right_fisheye, panel_x + preview_width + 8, "fisheye 2");
}

std::string make_data_number(double value)
{
    std::ostringstream output;
    output << std::fixed << std::setprecision(6) << value;
    return output.str();
}

int device_index(const rs2::device_list &devices)
{
    // query_devices(RS2_PRODUCT_LINE_T200) is already restricted to T265
    // products. Do not query or compare a serial number to decide whether a
    // usable device exists; some runtime/USB combinations expose the device
    // before camera-info strings are available.
    return devices.size() == 0 ? -1 : 0;
}

std::string device_info(const rs2::device &device, rs2_camera_info field)
{
    try {
        return device.supports(field) ? device.get_info(field) : "";
    } catch (const rs2::error &) {
        return "";
    }
}

omni::T265RawPose raw_pose_from_frame(const rs2::pose_frame &frame)
{
    const rs2_pose pose = frame.get_pose_data();
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
    raw.timestamp_s = frame.get_timestamp() * 0.001;
    raw.tracker_confidence = pose.tracker_confidence;
    raw.mapper_confidence = pose.mapper_confidence;
    return raw;
}

void log_pose_header(std::ofstream &output)
{
    output
        << "host_time_s,t265_timestamp_ms,tracking_origin_x_m,tracking_origin_y_m,tracking_origin_z_m,"
        << "tracking_origin_qx,tracking_origin_qy,tracking_origin_qz,tracking_origin_qw,"
        << "velocity_x_mps,velocity_y_mps,velocity_z_mps,angular_velocity_x_radps,"
        << "angular_velocity_y_radps,angular_velocity_z_radps,tracker_confidence,mapper_confidence,"
        << "map_loaded,relocalization_events,corrected_available,relative_yaw_deg,gyro_relative_yaw_deg,"
        << "raw_delta_forward_m,raw_delta_left_m,corrected_robot_center_forward_m,"
        << "corrected_robot_center_left_m,wheel_odom_forward_m,wheel_odom_left_m,wheel_odom_path_m\n";
}

void log_pose(std::ofstream &output, double host_time_s, const omni::T265RawPose &raw,
              const omni::T265FieldPose *pose, bool map_loaded, int relocalization_events,
              const OdomSnapshot &odom)
{
    output << std::fixed << std::setprecision(9)
           << host_time_s << ',' << raw.timestamp_s * 1000.0 << ','
           << raw.translation_m[0] << ',' << raw.translation_m[1] << ',' << raw.translation_m[2] << ','
           << raw.rotation_xyzw[0] << ',' << raw.rotation_xyzw[1] << ','
           << raw.rotation_xyzw[2] << ',' << raw.rotation_xyzw[3] << ','
           << raw.velocity_mps[0] << ',' << raw.velocity_mps[1] << ',' << raw.velocity_mps[2] << ','
           << raw.angular_velocity_radps[0] << ',' << raw.angular_velocity_radps[1] << ','
           << raw.angular_velocity_radps[2] << ',' << static_cast<unsigned>(raw.tracker_confidence) << ','
           << static_cast<unsigned>(raw.mapper_confidence) << ',' << (map_loaded ? 1 : 0) << ','
           << relocalization_events << ',';
    if (pose == nullptr) {
        output << "0,nan,nan,nan,nan,nan,nan,nan,nan,nan\n";
        return;
    }
    output << "1," << pose->relative_yaw_rad * 180.0 / kPi << ','
           << pose->gyro_relative_yaw_rad * 180.0 / kPi << ','
           << pose->tracking_origin_delta_forward_m << ','
           << pose->tracking_origin_delta_left_m << ','
           << pose->robot_center_delta_forward_m << ','
           << pose->robot_center_delta_left_m << ','
           << (odom.has_frame ? make_data_number(odom.wheel_center_forward_m) : "nan") << ','
           << (odom.has_frame ? make_data_number(odom.wheel_center_left_m) : "nan") << ','
           << (odom.has_frame ? make_data_number(odom.wheel_path_m) : "nan") << '\n';
}

bool export_map(const rs2::pose_sensor &pose_sensor, const std::string &path,
                EventLogger &events, MetadataState &metadata, std::string &message)
{
    try {
        const std::vector<std::uint8_t> map = pose_sensor.export_localization_map();
        if (map.empty()) throw std::runtime_error("T265 returned an empty localization map");
        if (!ensure_directory(parent_directory(path))) {
            throw std::runtime_error("cannot create map directory: " + parent_directory(path));
        }
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        if (!output) throw std::runtime_error("cannot write map: " + path);
        output.write(reinterpret_cast<const char *>(map.data()),
                    static_cast<std::streamsize>(map.size()));
        if (!output) throw std::runtime_error("short map write: " + path);
        metadata.map_exported = true;
        metadata.map_size_bytes = map.size();
        metadata.map_error.clear();
        message = "map exported " + std::to_string(map.size()) + " bytes";
        events.write("map_exported", "{\"path\":\"" + json_escape(path) +
                     "\",\"size_bytes\":" + std::to_string(map.size()) + "}");
        return true;
    } catch (const rs2::error &error) {
        metadata.map_exported = false;
        metadata.map_error = error.what();
        message = std::string("map export failed: ") + error.what();
        events.write("map_export_failed", "{\"error\":\"" + json_escape(error.what()) + "\"}");
        return false;
    } catch (const std::exception &error) {
        metadata.map_exported = false;
        metadata.map_error = error.what();
        message = std::string("map export failed: ") + error.what();
        events.write("map_export_failed", "{\"error\":\"" + json_escape(error.what()) + "\"}");
        return false;
    }
}

struct PlanStep {
    t265_map::protocol::MotionCommandPayload payload;
    std::string label;
};

struct MotionPlan {
    std::vector<PlanStep> steps;
    std::size_t next = 0;
    std::uint8_t active_sequence = 0;
    double sent_at = 0.0;
    std::string name;
};

t265_map::protocol::MotionCommandPayload move_body(std::int16_t forward_mm,
                                                   std::int16_t left_mm,
                                                   std::uint16_t speed_mm_s = 250)
{
    t265_map::protocol::MotionCommandPayload payload;
    payload.command = t265_map::protocol::kMotionMoveBody;
    payload.flags = t265_map::protocol::kFlagValid |
                    t265_map::protocol::kFlagKeepHeading |
                    t265_map::protocol::kFlagAckRequired;
    payload.arg1 = forward_mm;
    payload.arg2 = left_mm;
    payload.speed = speed_mm_s;
    return payload;
}

t265_map::protocol::MotionCommandPayload turn_relative(double angle_deg,
                                                       std::uint16_t speed_deci_deg_s = 900)
{
    if (angle_deg < -3276.7 || angle_deg > 3276.7) {
        throw std::invalid_argument("turn angle outside protocol range");
    }
    t265_map::protocol::MotionCommandPayload payload;
    payload.command = t265_map::protocol::kMotionTurnRelative;
    payload.flags = t265_map::protocol::kFlagValid | t265_map::protocol::kFlagAckRequired;
    payload.arg1 = static_cast<std::int16_t>(std::lround(angle_deg * 10.0));
    payload.speed = speed_deci_deg_s;
    return payload;
}

bool send_next_step(MotionLink &link, MotionPlan &plan, UiState &ui, double now_s)
{
    if (plan.next >= plan.steps.size()) {
        plan.steps.clear();
        ui.message = "motion plan complete";
        return true;
    }
    if (!link.send(plan.steps[plan.next].payload, plan.active_sequence)) {
        ui.message = "motion command send failed";
        plan.steps.clear();
        return false;
    }
    plan.sent_at = now_s;
    ui.message = "sent " + plan.steps[plan.next].label +
                 " seq " + std::to_string(plan.active_sequence);
    return true;
}

void update_motion_plan(MotionLink *link, MotionPlan &plan, UiState &ui,
                        double now_s)
{
    if (link == nullptr || plan.steps.empty()) return;
    const StatusSnapshot status = link->status_snapshot();
    if (status.has_status &&
        status.status.acknowledged_command_sequence == plan.active_sequence) {
        if (status.status.state == t265_map::protocol::kMotionDone) {
            ++plan.next;
            if (!send_next_step(*link, plan, ui, now_s)) return;
        } else if (status.status.state == t265_map::protocol::kMotionError ||
                   status.status.state == t265_map::protocol::kMotionStopped) {
            ui.message = "F407 stopped motion plan";
            plan.steps.clear();
        }
    }
    if (!plan.steps.empty() && now_s - plan.sent_at > 4.0) {
        link->send_stop();
        ui.message = "motion status timeout; STOP sent";
        plan.steps.clear();
    }
}

void start_motion_plan(MotionLink *link, bool motion_enabled,
                       const std::vector<PlanStep> &steps,
                       const std::string &name, MotionPlan &plan, UiState &ui,
                       double now_s)
{
    if (!motion_enabled || link == nullptr || steps.empty()) {
        ui.message = "motion disabled: start with --enable-motion and a working UART";
        return;
    }
    if (!plan.steps.empty()) {
        ui.message = "a motion plan is already running; wait for DONE or STOP";
        return;
    }
    plan.steps = steps;
    plan.next = 0;
    plan.name = name;
    if (!send_next_step(*link, plan, ui, now_s)) plan.steps.clear();
}

bool set_anchor(int number, const rs2::pose_sensor &pose_sensor,
                const omni::T265RawPose &raw, bool have_raw,
                const omni::T265FieldPose *pose, bool have_pose,
                MetadataState &metadata, EventLogger &events, UiState &ui)
{
    if (!have_raw || !have_pose || pose == nullptr) {
        ui.message = "anchor requires a valid corrected T265 pose";
        return false;
    }
    if (raw.tracker_confidence < 3) {
        ui.message = "anchor requires tracker confidence 3";
        return false;
    }
    const std::string name = "scan_anchor_" + std::to_string(number);
    const rs2_vector position{static_cast<float>(raw.translation_m[0]),
                              static_cast<float>(raw.translation_m[1]),
                              static_cast<float>(raw.translation_m[2])};
    const rs2_quaternion orientation{static_cast<float>(raw.rotation_xyzw[0]),
                                     static_cast<float>(raw.rotation_xyzw[1]),
                                     static_cast<float>(raw.rotation_xyzw[2]),
                                     static_cast<float>(raw.rotation_xyzw[3])};
    try {
        if (!pose_sensor.set_static_node(name, position, orientation)) {
            ui.message = "T265 rejected " + name;
            return false;
        }
        metadata.anchors[number] = true;
        events.write("static_node_set", "{\"name\":\"" + name + "\",\"tracker_confidence\":3}");
        ui.message = "saved " + name;
        return true;
    } catch (const rs2::error &error) {
        ui.message = std::string("anchor failed: ") + error.what();
        events.write("static_node_failed", "{\"name\":\"" + name +
                     "\",\"error\":\"" + json_escape(error.what()) + "\"}");
        return false;
    }
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        Options options = parse_options(argc, argv);
        if (options.session_dir == T265_MAP_DEFAULT_SESSION_ROOT) {
            options.session_dir += "/" + timestamp_directory_name();
            if (options.save_map == T265_MAP_DEFAULT_SESSION_ROOT + std::string("/t265_localization.raw")) {
                options.save_map = options.session_dir + "/t265_localization.raw";
            }
        }
        if (!ensure_directory(options.session_dir)) {
            throw std::runtime_error("cannot create session directory: " + options.session_dir);
        }
        if (!ensure_directory(parent_directory(options.save_map))) {
            throw std::runtime_error("cannot create map directory: " + parent_directory(options.save_map));
        }

        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        const auto app_start = std::chrono::steady_clock::now();
        EventLogger events(options.session_dir + "/events.jsonl", app_start);
        MetadataState metadata;

        omni::LocalizationConfig localization_config;
        if (file_exists(options.config_path)) {
            try {
                localization_config = omni::load_config(options.config_path);
            } catch (const std::exception &error) {
                events.write("config_load_failed", "{\"path\":\"" + json_escape(options.config_path) +
                             "\",\"error\":\"" + json_escape(error.what()) + "\"}");
            }
        } else {
            events.write("config_missing", "{\"path\":\"" + json_escape(options.config_path) + "\"}");
        }
        // This standalone program is deliberately relative-view only. Keep
        // the field coordinates untouched and use the same calibrated camera
        // lever arm/axes as the production projector.
        localization_config.camera_offset_forward_m = -0.0296;
        localization_config.camera_offset_left_m = -0.0301;
        localization_config.camera_robot_forward_axis[0] = 1.0;
        localization_config.camera_robot_forward_axis[1] = 0.0;
        localization_config.camera_robot_forward_axis[2] = 0.0;
        localization_config.camera_robot_up_axis[0] = 0.0;
        localization_config.camera_robot_up_axis[1] = 0.0;
        localization_config.camera_robot_up_axis[2] = -1.0;

        write_metadata(options.session_dir + "/metadata.json", options, localization_config,
                       kMetadataSerial, kKnownFirmware, metadata, false);
        events.write("program_start", "{\"mode\":\"" +
                     std::string(options.load_map.empty() ? "scan" : "verify") +
                     "\",\"session_dir\":\"" + json_escape(options.session_dir) + "\"}");

        std::ofstream pose_csv(options.session_dir + "/pose.csv", std::ios::out | std::ios::trunc);
        if (!pose_csv) throw std::runtime_error("cannot open pose.csv");
        log_pose_header(pose_csv);

        std::unique_ptr<MotionLink> motion_link;
        bool uart_connected = false;
        std::string uart_error;
        if (options.use_uart) {
            try {
                motion_link.reset(new MotionLink(options, events, app_start,
                                                 options.session_dir + "/encoder.csv",
                                                 options.session_dir + "/f407_status.csv"));
                motion_link->start();
                uart_connected = true;
                events.write("uart_opened", "{\"path\":\"" + json_escape(options.uart) +
                             "\",\"baud\":" + std::to_string(options.baud) + "}");
            } catch (const std::exception &error) {
                uart_error = error.what();
                events.write("uart_open_failed", "{\"error\":\"" + json_escape(error.what()) + "\"}");
                motion_link.reset();
                if (options.enable_motion) throw;
            }
        } else {
            // Keep the output contract stable even in T265-only mode.
            std::ofstream encoder(options.session_dir + "/encoder.csv");
            encoder << "host_time_s,sequence,m1_count,m2_count,m3_count,dt_ms,status,delta_m1_count,"
                       "delta_m2_count,delta_m3_count,body_forward_increment_m,body_left_increment_m,"
                       "wheel_center_forward_m,wheel_center_left_m,wheel_path_m,t265_relative_yaw_deg,"
                       "yaw_reference_valid,increment_accepted\n";
            std::ofstream status(options.session_dir + "/f407_status.csv");
            status << "host_time_s,frame_sequence,command_sequence,state,fault,command,progress,heading_cdeg\n";
        }

        rs2::log_to_console(RS2_LOG_SEVERITY_WARN);
        rs2::context context;
        rs2::pipeline pipeline(context);
        rs2::config pipeline_config;
        pipeline_config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        bool fisheye_active = false;
        if (options.enable_fisheye) {
            pipeline_config.enable_stream(RS2_STREAM_FISHEYE, 1);
            pipeline_config.enable_stream(RS2_STREAM_FISHEYE, 2);
        }
        rs2::device selected;
        const auto device_wait_start = std::chrono::steady_clock::now();
        while (!g_stop) {
            const rs2::device_list devices = context.query_devices(RS2_PRODUCT_LINE_T200);
            const int index = device_index(devices);
            if (index >= 0) {
                selected = devices[static_cast<std::size_t>(index)];
                break;
            }
            if (elapsed_seconds(device_wait_start) >= options.wait_sec) {
                throw std::runtime_error("no running T265 before --wait timeout");
            }
            std::cerr << "[WAIT] no running T265 found; retrying...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (g_stop) return 130;

        // Resolve the same device through the pipeline before obtaining the
        // pose sensor. This is the official T265 map sample's sequence and
        // avoids opening a second USB device handle from a separate query.
        selected = rs2::device();
        const rs2::pipeline_profile resolved_profile = pipeline_config.resolve(pipeline);
        const rs2::device resolved_device = resolved_profile.get_device();
        const std::string serial = device_info(resolved_device, RS2_CAMERA_INFO_SERIAL_NUMBER);
        const std::string firmware = device_info(resolved_device, RS2_CAMERA_INFO_FIRMWARE_VERSION);
        std::cerr << "[READY] T265 serial=" << serial << " firmware=" << firmware
                  << " librealsense=" << kKnownLibrealsense << '\n';
        events.write("t265_connected", "{\"serial\":\"" + json_escape(serial) +
                     "\",\"firmware\":\"" + json_escape(firmware) + "\",\"librealsense\":\"" +
                     kKnownLibrealsense + "\"}");

        rs2::pose_sensor pose_sensor = resolved_device.first<rs2::pose_sensor>();
        if (!pose_sensor) throw std::runtime_error("T265 pose sensor is unavailable");
        auto set_t265_option = [&](rs2_option option, float value, const char *name, bool required) {
            if (!pose_sensor.supports(option)) {
                if (required) throw std::runtime_error(std::string("T265 does not support ") + name);
                events.write("option_unavailable", std::string("{\"name\":\"") + name + "\"}");
                return;
            }
            pose_sensor.set_option(option, value);
            std::ostringstream data;
            data << "{\"name\":\"" << name << "\",\"value\":" << value << "}";
            events.write("t265_option", data.str());
        };
        set_t265_option(RS2_OPTION_ENABLE_MAPPING, 1.0f, "enable_mapping", true);
        set_t265_option(RS2_OPTION_ENABLE_RELOCALIZATION, 1.0f, "enable_relocalization", true);
        set_t265_option(RS2_OPTION_ENABLE_POSE_JUMPING,
                        options.allow_pose_jumping ? 1.0f : 0.0f,
                        "enable_pose_jumping", true);
        set_t265_option(RS2_OPTION_ENABLE_MAP_PRESERVATION, 0.0f,
                        "enable_map_preservation", false);

        if (!options.load_map.empty()) {
            const std::vector<std::uint8_t> imported = read_binary(options.load_map);
            if (!pose_sensor.import_localization_map(imported)) {
                throw std::runtime_error("T265 rejected localization map: " + options.load_map);
            }
            metadata.map_imported = true;
            events.write("map_imported", "{\"path\":\"" + json_escape(options.load_map) +
                         "\",\"size_bytes\":" + std::to_string(imported.size()) + "}");
        }

        std::atomic<int> relocalization_events{0};
        pose_sensor.set_notifications_callback([&](const rs2::notification &notification) {
            std::ostringstream data;
            data << "{\"category\":\"" << notification.get_category()
                 << "\",\"description\":\"" << json_escape(notification.get_description())
                 << "\",\"timestamp\":" << notification.get_timestamp()
                 << ",\"serialized\":\"" << json_escape(notification.get_serialized_data()) << "\"}";
            events.write("t265_notification", data.str());
            if (notification.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
                relocalization_events.fetch_add(1);
                events.write("t265_relocalized");
            }
        });

        pipeline.start(pipeline_config);
        fisheye_active = options.enable_fisheye;
        events.write("pipeline_started", "{\"fisheye_active\":" + json_bool(fisheye_active) + "}");
        write_metadata(options.session_dir + "/metadata.json", options, localization_config,
                       serial, firmware, metadata, fisheye_active);

        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
        const std::pair<int, int> screen = detect_screen_size();
        const int canvas_width = options.width > 0 ? options.width : (options.fullscreen ? screen.first : screen.first);
        const int canvas_height = options.height > 0 ? options.height : (options.fullscreen ? screen.second : screen.second);
        cv::resizeWindow(kWindowName, canvas_width, canvas_height);
        UiState ui;
        ui.fullscreen = options.fullscreen;
        if (ui.fullscreen) cv::setWindowProperty(kWindowName, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::Mat canvas(canvas_height, canvas_width, CV_8UC3);
        std::vector<Button> buttons;
        std::string clicked;
        MouseContext mouse_context{&buttons, &clicked};
        cv::setMouseCallback(kWindowName, on_mouse, &mouse_context);
        GlobalArrowInput global_arrows;

        std::unique_ptr<omni::T265FieldProjector> projector;
        omni::T265FieldPose latest_pose{};
        omni::T265RawPose latest_raw{};
        bool have_pose = false;
        bool have_raw = false;
        int previous_relocalization_events = 0;
        bool relocalization_timeout_reported = false;
        MotionPlan motion_plan;
        bool export_requested = false;
        const auto run_start = std::chrono::steady_clock::now();
        while (!g_stop) {
            const rs2::frameset frames = pipeline.wait_for_frames(1000);
            const rs2::pose_frame pose_frame = frames.get_pose_frame();
            cv::Mat left_fisheye;
            cv::Mat right_fisheye;
            if (fisheye_active) {
                left_fisheye = fisheye_bgr(frames.get_fisheye_frame(1));
                right_fisheye = fisheye_bgr(frames.get_fisheye_frame(2));
            }
            if (pose_frame) {
                const double host_time = elapsed_seconds(app_start);
                const double tracking_time = elapsed_seconds(run_start);
                latest_raw = raw_pose_from_frame(pose_frame);
                have_raw = true;
                const int current_relocalizations = relocalization_events.load();
                metadata.relocalization_events = current_relocalizations;
                const bool map_ready = options.load_map.empty() || current_relocalizations > 0;
                if (!options.load_map.empty() && current_relocalizations == 0 &&
                    tracking_time >= options.relocalization_timeout_sec &&
                    !relocalization_timeout_reported) {
                    relocalization_timeout_reported = true;
                    ui.message = "relocalization timeout; no corrected path initialized";
                    events.write("t265_relocalization_timeout", "{\"timeout_s\":" +
                                 make_data_number(options.relocalization_timeout_sec) + "}");
                }
                if (map_ready && !projector && latest_raw.tracker_confidence >= options.min_confidence) {
                    projector.reset(new omni::T265FieldProjector(localization_config));
                    events.write("relative_view_initialized", "{\"reason\":\"" +
                                 std::string(options.load_map.empty() ? "tracker_ready" : "relocalized") + "\"}");
                }
                if (projector) {
                    latest_pose = projector->project(latest_raw);
                    have_pose = true;
                    if (motion_link) {
                        motion_link->set_reference_yaw(latest_pose.gyro_relative_yaw_rad,
                                                       latest_pose.gyro_yaw_rate_valid);
                    }
                    const Point2 raw_point{latest_pose.tracking_origin_delta_left_m,
                                           latest_pose.tracking_origin_delta_forward_m};
                    const Point2 corrected_point{latest_pose.robot_center_delta_left_m,
                                                 latest_pose.robot_center_delta_forward_m};
                    ui.raw_path.push_back(raw_point);
                    ui.corrected_path.push_back(corrected_point);
                    if (ui.raw_path.size() > 20000u) ui.raw_path.pop_front();
                    if (ui.corrected_path.size() > 20000u) ui.corrected_path.pop_front();
                }
                OdomSnapshot odom = motion_link ? motion_link->odom_snapshot() : OdomSnapshot{};
                if (odom.has_frame && odom.frame_count != ui.last_odom_point &&
                    odom.increment_accepted) {
                    ui.wheel_path.push_back(Point2{odom.wheel_center_left_m,
                                                   odom.wheel_center_forward_m});
                    if (ui.wheel_path.size() > 20000u) ui.wheel_path.pop_front();
                    ui.last_odom_point = odom.frame_count;
                }
                log_pose(pose_csv, host_time, latest_raw, have_pose ? &latest_pose : nullptr,
                         !options.load_map.empty(), current_relocalizations, odom);
                pose_csv.flush();

                if (current_relocalizations != previous_relocalization_events) {
                    previous_relocalization_events = current_relocalizations;
                    ui.message = "T265 relocalized; verify trajectory and anchors";
                    if (options.load_map.empty() == false) {
                        for (int anchor = 1; anchor <= 4; ++anchor) {
                            rs2_vector node_position{};
                            rs2_quaternion node_orientation{};
                            const std::string node_name = "scan_anchor_" + std::to_string(anchor);
                            try {
                                if (pose_sensor.get_static_node(node_name, node_position, node_orientation)) {
                                    events.write("static_node_found", "{\"name\":\"" + node_name + "\"}");
                                }
                            } catch (const rs2::error &) {
                            }
                        }
                    }
                }
            }

            const double now_s = elapsed_seconds(run_start);
            if (motion_link) update_motion_plan(motion_link.get(), motion_plan, ui, now_s);
            const OdomSnapshot odom = motion_link ? motion_link->odom_snapshot() : OdomSnapshot{};
            const StatusSnapshot status = motion_link ? motion_link->status_snapshot() : StatusSnapshot{};
            draw_interface(canvas, options, serial, firmware, localization_config, metadata, ui,
                           have_pose ? &latest_pose : nullptr, have_pose, odom, status,
                           uart_connected, motion_link ? motion_link->error() : uart_error,
                           left_fisheye, right_fisheye, buttons);
            cv::imshow(kWindowName, canvas);
            const int raw_key = cv::waitKeyEx(1);
            int key = normalize_key(raw_key);
            const int global_arrow_action = global_arrows.poll_pressed_edge();
            if (global_arrow_action != 0) key = global_arrow_action;
            if (raw_key >= 0 || global_arrow_action != 0) {
                std::ostringstream key_data;
                key_data << "{\"raw_key\":" << raw_key
                         << ",\"normalized_key\":" << key
                         << ",\"x11_arrow_action\":" << global_arrow_action << "}";
                events.write("ui_key", key_data.str());
            }
            if (!clicked.empty()) {
                const std::string action = clicked;
                clicked.clear();
                if (action == "turn90") key = kActionTurn90;
                else if (action == "turn180") key = kActionTurn180;
                else if (action == "turn360") key = kActionTurn360;
                else if (action == "move_forward") key = kActionMoveForward1M;
                else if (action == "move_left") key = kActionMoveLeft1M;
                else if (action == "return") key = kActionReturn;
                else if (action == "reset_odom") key = kActionResetOdom;
                else if (action == "stop") key = kActionStop;
                else if (action == "export") key = kActionExport;
                else if (action == "reset_view") key = kActionResetView;
            }
            if (key == 27 || key == 'q') break;
            if (key == 'f' || key == 'F') {
                ui.fullscreen = !ui.fullscreen;
                cv::setWindowProperty(kWindowName, cv::WND_PROP_FULLSCREEN,
                                      ui.fullscreen ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);
            } else if (key == '+' || key == '=') {
                ui.scale = std::min(1200.0, ui.scale * 1.25);
            } else if (key == '-' || key == '_') {
                ui.scale = std::max(20.0, ui.scale / 1.25);
            } else if (key == 'e' || key == 'E' || key == kActionExport) {
                export_requested = true;
                ui.message = "export requested";
            } else if (key == 'r' || key == 'R' || key == kActionResetView) {
                projector.reset();
                have_pose = false;
                ui.raw_path.clear();
                ui.corrected_path.clear();
                ui.wheel_path.clear();
                ui.last_odom_point = 0;
                if (motion_link) {
                    motion_link->set_reference_yaw(0.0, false);
                    motion_link->reset_wheel_view();
                }
                ui.message = "view reset; next reliable pose becomes local origin";
                events.write("relative_view_reset");
            } else if (key == '1' || key == '2' || key == '3' || key == '4') {
                const int anchor = key - '0';
                set_anchor(anchor, pose_sensor, latest_raw, have_raw,
                           have_pose ? &latest_pose : nullptr, have_pose,
                           metadata, events, ui);
            } else if (key == 'x' || key == 'X' || key == ' ' || key == kActionStop) {
                motion_plan.steps.clear();
                if (motion_link) {
                    if (motion_link->send_stop()) ui.message = "STOP sent";
                    else ui.message = "STOP send failed";
                } else {
                    ui.message = "motion is disabled";
                }
            } else if (key == 'w' || key == 'W') {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{move_body(500, 0), "MOVE FWD 0.5M"}},
                                   "move_forward_half", motion_plan, ui, now_s);
            } else if (key == 's' || key == 'S') {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{move_body(-500, 0), "MOVE BACK 0.5M"}},
                                   "move_backward_half", motion_plan, ui, now_s);
            } else if (key == 'a' || key == 'A') {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{move_body(0, 500), "MOVE LEFT 0.5M"}},
                                   "move_left_half", motion_plan, ui, now_s);
            } else if (key == 'd' || key == 'D') {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{move_body(0, -500), "MOVE RIGHT 0.5M"}},
                                   "move_right_half", motion_plan, ui, now_s);
            } else if (key == kActionTurnLeft10) {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{turn_relative(10.0), "TURN LEFT 10"}},
                                   "turn_left_10", motion_plan, ui, now_s);
            } else if (key == kActionTurnRight10) {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{turn_relative(-10.0), "TURN RIGHT 10"}},
                                   "turn_right_10", motion_plan, ui, now_s);
            } else if (key == kActionTurn90) {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{turn_relative(90.0), "TURN +90"}},
                                   "turn90", motion_plan, ui, now_s);
            } else if (key == 'b' || key == 'B') {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{turn_relative(180.0), "TURN +180"}},
                                   "turn180", motion_plan, ui, now_s);
            } else if (key == 'c' || key == 'C') {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{turn_relative(360.0), "TURN +360"}},
                                   "turn360", motion_plan, ui, now_s);
            } else if (key == kActionMoveForward1M) {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{move_body(1000, 0), "MOVE FWD 1M"}},
                                   "move_forward", motion_plan, ui, now_s);
            } else if (key == kActionMoveLeft1M) {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{move_body(0, 1000), "MOVE LEFT 1M"}},
                                   "move_left", motion_plan, ui, now_s);
            } else if (key == 'l' || key == 'L') {
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{move_body(0, 1000), "MOVE LEFT 1M"}},
                                   "move_left", motion_plan, ui, now_s);
            } else if (key == 'v' || key == 'V' || key == kActionReturn) {
                start_motion_plan(motion_link.get(), options.enable_motion, {
                                       PlanStep{move_body(1000, 0), "OUT 1M"},
                                       PlanStep{turn_relative(180.0), "TURN 180"},
                                       PlanStep{move_body(1000, 0), "BACK 1M"},
                               }, "out_turn_back", motion_plan, ui, now_s);
            } else if (key == 'o' || key == 'O' || key == kActionResetOdom) {
                t265_map::protocol::MotionCommandPayload reset;
                reset.command = t265_map::protocol::kMotionResetOdom;
                reset.flags = t265_map::protocol::kFlagValid |
                              t265_map::protocol::kFlagAckRequired |
                              t265_map::protocol::kFlagClearFault;
                start_motion_plan(motion_link.get(), options.enable_motion,
                                   {PlanStep{reset, "RESET ODOM"}},
                                   "reset_odom", motion_plan, ui, now_s);
            }
            if (export_requested) {
                export_requested = false;
                std::string export_message;
                export_map(pose_sensor, options.save_map, events, metadata, export_message);
                ui.message = export_message;
                write_metadata(options.session_dir + "/metadata.json", options, localization_config,
                               serial, firmware, metadata, fisheye_active);
            }
        }

        if (motion_link && options.enable_motion) {
            try { motion_link->send_stop(); } catch (const std::exception &) {}
        }
        motion_plan.steps.clear();
        pipeline.stop();
        events.write("pipeline_stopped");
        std::string final_export_message;
        export_map(pose_sensor, options.save_map, events, metadata, final_export_message);
        metadata.relocalization_events = relocalization_events.load();
        write_metadata(options.session_dir + "/metadata.json", options, localization_config,
                       serial, firmware, metadata, fisheye_active);
        if (motion_link) motion_link->stop();
        cv::destroyWindow(kWindowName);
        return metadata.map_exported ? 0 : 2;
    } catch (const rs2::error &error) {
        std::cerr << "[RS2 ERROR] " << error.what() << '\n';
        return 3;
    } catch (const std::exception &error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
