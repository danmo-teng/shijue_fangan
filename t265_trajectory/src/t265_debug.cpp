#include <librealsense2/rs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void signal_handler(int)
{
    g_stop = 1;
}

struct Options {
    double wait_sec = 30.0;
    double duration_sec = 0.0;
    double print_rate_hz = 10.0;
    std::string csv_path;
    std::string serial;
    bool list_only = false;
    bool debug_sdk = false;
    bool reset = false;
};

struct UsbState {
    std::string sys_name;
    std::string vid;
    std::string pid;
    std::string bus;
    std::string device;
    std::string product;
};

std::string trim(std::string value)
{
    while (!value.empty() && (value.back() == '\n' || value.back() == '\r' || value.back() == ' ')) {
        value.pop_back();
    }
    return value;
}

std::string read_text(const std::string &path)
{
    std::ifstream file(path);
    std::string value;
    std::getline(file, value);
    return trim(value);
}

std::vector<UsbState> scan_t265_usb()
{
    std::vector<UsbState> result;
    const std::string root = "/sys/bus/usb/devices";
    DIR *directory = opendir(root.c_str());
    if (!directory) {
        return result;
    }

    while (const dirent *entry = readdir(directory)) {
        if (entry->d_name[0] == '.') {
            continue;
        }
        const std::string base = root + "/" + entry->d_name;
        std::string vid = read_text(base + "/idVendor");
        std::string pid = read_text(base + "/idProduct");
        std::transform(vid.begin(), vid.end(), vid.begin(), ::tolower);
        std::transform(pid.begin(), pid.end(), pid.begin(), ::tolower);
        const bool bootloader = vid == "03e7" && pid == "2150";
        const bool running = vid == "8087" && pid == "0b37";
        if (!bootloader && !running) {
            continue;
        }
        UsbState state;
        state.sys_name = entry->d_name;
        state.vid = vid;
        state.pid = pid;
        state.bus = read_text(base + "/busnum");
        state.device = read_text(base + "/devnum");
        state.product = read_text(base + "/product");
        result.push_back(state);
    }
    closedir(directory);
    return result;
}

std::string usb_state_name(const UsbState &state)
{
    if (state.vid == "03e7" && state.pid == "2150") {
        return "BOOTLOADER";
    }
    return "RUNNING";
}

void print_usb_state()
{
    const auto states = scan_t265_usb();
    if (states.empty()) {
        std::cerr << "[USB] T265 not enumerated (neither 03e7:2150 nor 8087:0b37)\n";
        return;
    }
    for (const auto &state : states) {
        std::cerr << "[USB] " << usb_state_name(state) << " " << state.vid << ':' << state.pid
                  << " bus=" << state.bus << " device=" << state.device
                  << " sysfs=" << state.sys_name;
        if (!state.product.empty()) {
            std::cerr << " product=\"" << state.product << '"';
        }
        std::cerr << '\n';
    }
}

double parse_nonnegative(const std::string &text, const char *name)
{
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used != text.size() || value < 0.0) {
            throw std::invalid_argument("range");
        }
        return value;
    } catch (const std::exception &) {
        throw std::invalid_argument(std::string("invalid value for ") + name + ": " + text);
    }
}

void usage(const char *program)
{
    std::cout
        << "T265 standalone diagnostic (librealsense 2.50)\n\n"
        << "Usage: " << program << " [options]\n"
        << "  --wait SEC          wait/retry for camera (default: 30)\n"
        << "  --duration SEC      stop after SEC; 0 runs until Ctrl-C (default: 0)\n"
        << "  --print-rate HZ     terminal output rate; 0 prints every pose (default: 10)\n"
        << "  --csv FILE          save every pose frame to CSV\n"
        << "  --serial SERIAL     select a specific T265\n"
        << "  --list              show USB/SDK state, then exit\n"
        << "  --debug-sdk         print detailed librealsense boot/USB log\n"
        << "  --reset             issue T265 hardware reset and exit\n"
        << "  -h, --help          show this help\n";
}

Options parse_options(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto value = [&](const char *name) -> std::string {
            if (++i >= argc) {
                throw std::invalid_argument(std::string("missing value for ") + name);
            }
            return argv[i];
        };
        if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else if (arg == "--wait") {
            options.wait_sec = parse_nonnegative(value("--wait"), "--wait");
        } else if (arg == "--duration") {
            options.duration_sec = parse_nonnegative(value("--duration"), "--duration");
        } else if (arg == "--print-rate") {
            options.print_rate_hz = parse_nonnegative(value("--print-rate"), "--print-rate");
        } else if (arg == "--csv") {
            options.csv_path = value("--csv");
        } else if (arg == "--serial") {
            options.serial = value("--serial");
        } else if (arg == "--list") {
            options.list_only = true;
        } else if (arg == "--debug-sdk") {
            options.debug_sdk = true;
        } else if (arg == "--reset") {
            options.reset = true;
        } else {
            throw std::invalid_argument("unknown option: " + arg);
        }
    }
    return options;
}

std::string info(const rs2::device &device, rs2_camera_info field)
{
    try {
        return device.supports(field) ? device.get_info(field) : "-";
    } catch (const rs2::error &) {
        return "-";
    }
}

void print_sdk_devices(const rs2::device_list &devices)
{
    std::cerr << "[SDK] RealSense/T200 devices: " << devices.size() << '\n';
    for (std::size_t i = 0; i < devices.size(); ++i) {
        const rs2::device device = devices[i];
        std::cerr << "[SDK] #" << i
                  << " name=\"" << info(device, RS2_CAMERA_INFO_NAME) << '"'
                  << " serial=" << info(device, RS2_CAMERA_INFO_SERIAL_NUMBER)
                  << " firmware=" << info(device, RS2_CAMERA_INFO_FIRMWARE_VERSION)
                  << " product_line=" << info(device, RS2_CAMERA_INFO_PRODUCT_LINE) << '\n';
    }
}

int select_device(const rs2::device_list &devices, const std::string &serial)
{
    for (std::size_t i = 0; i < devices.size(); ++i) {
        if (serial.empty() || info(devices[i], RS2_CAMERA_INFO_SERIAL_NUMBER) == serial) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

double heading_y_degrees(const rs2_quaternion &q)
{
    const double numerator = 2.0 * (q.w * q.y + q.x * q.z);
    const double denominator = 1.0 - 2.0 * (q.y * q.y + q.x * q.x);
    return std::atan2(numerator, denominator) * 180.0 / 3.14159265358979323846;
}

void write_csv_header(std::ofstream &csv)
{
    csv << "host_time_s,device_time_ms,frame,x_m,y_m,z_m,qx,qy,qz,qw,"
           "vx_mps,vy_mps,vz_mps,ax_mps2,ay_mps2,az_mps2,"
           "wx_radps,wy_radps,wz_radps,alphax_radps2,alphay_radps2,alphaz_radps2,"
           "tracker_confidence,mapper_confidence\n";
}

void write_csv_pose(std::ofstream &csv, double host_sec, double device_ms,
                    unsigned long long frame_number, const rs2_pose &p)
{
    csv << std::fixed << std::setprecision(9) << host_sec << ','
        << std::setprecision(6) << device_ms << ',' << frame_number << ','
        << std::setprecision(9)
        << p.translation.x << ',' << p.translation.y << ',' << p.translation.z << ','
        << p.rotation.x << ',' << p.rotation.y << ',' << p.rotation.z << ',' << p.rotation.w << ','
        << p.velocity.x << ',' << p.velocity.y << ',' << p.velocity.z << ','
        << p.acceleration.x << ',' << p.acceleration.y << ',' << p.acceleration.z << ','
        << p.angular_velocity.x << ',' << p.angular_velocity.y << ',' << p.angular_velocity.z << ','
        << p.angular_acceleration.x << ',' << p.angular_acceleration.y << ',' << p.angular_acceleration.z << ','
        << static_cast<unsigned>(p.tracker_confidence) << ','
        << static_cast<unsigned>(p.mapper_confidence) << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        const Options options = parse_options(argc, argv);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        rs2::log_to_console(options.debug_sdk ? RS2_LOG_SEVERITY_DEBUG : RS2_LOG_SEVERITY_WARN);
        std::cerr << "[INFO] linked librealsense API " << RS2_API_VERSION_STR << '\n';
        print_usb_state();

        const auto wait_start = std::chrono::steady_clock::now();
        std::unique_ptr<rs2::context> context;
        rs2::device selected;
        unsigned attempt = 0;

        while (!g_stop) {
            ++attempt;
            context.reset(new rs2::context());
            const rs2::device_list devices = context->query_devices(RS2_PRODUCT_LINE_T200);
            if (options.list_only) {
                print_sdk_devices(devices);
                print_usb_state();
                return devices.size() ? EXIT_SUCCESS : 2;
            }

            const int selected_index = select_device(devices, options.serial);
            if (selected_index >= 0) {
                selected = devices[static_cast<std::size_t>(selected_index)];
                break;
            }

            print_usb_state();
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - wait_start).count();
            if (elapsed >= options.wait_sec) {
                std::cerr << "[ERROR] no running T265 after " << std::fixed << std::setprecision(1)
                          << elapsed << " s and " << attempt << " SDK attempt(s)\n"
                          << "[HINT] 03e7:2150 means firmware boot failed; inspect --debug-sdk output.\n"
                          << "[HINT] no USB line means cable/power/port enumeration failed.\n";
                return 2;
            }
            std::cerr << "[WAIT] retrying SDK discovery in 2 s...\n";
            context.reset();
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        if (g_stop) {
            return 130;
        }

        std::cerr << "[READY] name=\"" << info(selected, RS2_CAMERA_INFO_NAME) << '"'
                  << " serial=" << info(selected, RS2_CAMERA_INFO_SERIAL_NUMBER)
                  << " firmware=" << info(selected, RS2_CAMERA_INFO_FIRMWARE_VERSION) << '\n';

        if (options.reset) {
            std::cerr << "[RESET] sending hardware reset; the USB device will disconnect briefly\n";
            selected.hardware_reset();
            return EXIT_SUCCESS;
        }

        const std::string selected_serial = info(selected, RS2_CAMERA_INFO_SERIAL_NUMBER);
        // A T265 rs2::device instance owns/claims its USB interface. Keep only
        // the serial number before pipeline.start(), otherwise the second device
        // instance created by the pipeline receives RS2_USB_STATUS_BUSY.
        selected = rs2::device();

        std::ofstream csv;
        if (!options.csv_path.empty()) {
            csv.open(options.csv_path);
            if (!csv) {
                throw std::runtime_error("cannot open CSV output: " + options.csv_path);
            }
            write_csv_header(csv);
            std::cerr << "[CSV] saving every pose frame to " << options.csv_path << '\n';
        }

        rs2::pipeline pipeline(*context);
        rs2::config config;
        config.enable_device(selected_serial);
        config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        pipeline.start(config);
        std::cerr << "[STREAM] pose started; native axes: +X right, +Y up, +Z backward\n";
        std::cerr << "[STREAM] confidence: 0=failed, 1=low, 2=medium, 3=high\n";

        auto run_start = std::chrono::steady_clock::now();
        auto last_print = run_start - std::chrono::hours(1);
        unsigned long long received = 0;
        unsigned long long missing = 0;
        unsigned long long last_frame = 0;
        bool have_last = false;
        unsigned low_confidence = 0;

        while (!g_stop) {
            const rs2::frameset frames = pipeline.wait_for_frames(5000);
            const rs2::pose_frame pose_frame = frames.get_pose_frame();
            if (!pose_frame) {
                continue;
            }
            const auto now = std::chrono::steady_clock::now();
            if (received == 0) {
                run_start = now;
                last_print = now - std::chrono::hours(1);
            }
            const double host_sec = std::chrono::duration<double>(now - run_start).count();
            const unsigned long long frame_number = pose_frame.get_frame_number();
            const double device_ms = pose_frame.get_timestamp();
            const rs2_pose pose = pose_frame.get_pose_data();
            ++received;
            if (pose.tracker_confidence < 2) {
                ++low_confidence;
            }
            if (have_last && frame_number > last_frame + 1) {
                missing += frame_number - last_frame - 1;
            }
            have_last = true;
            last_frame = frame_number;

            if (csv) {
                write_csv_pose(csv, host_sec, device_ms, frame_number, pose);
            }

            const bool print_now = options.print_rate_hz == 0.0 ||
                std::chrono::duration<double>(now - last_print).count() >= 1.0 / options.print_rate_hz;
            if (print_now) {
                last_print = now;
                std::cout << std::fixed << std::setprecision(3)
                          << "t=" << host_sec << "s frame=" << frame_number
                          << " p[m]=(" << pose.translation.x << ',' << pose.translation.y << ',' << pose.translation.z << ')'
                          << " q=(" << pose.rotation.x << ',' << pose.rotation.y << ','
                          << pose.rotation.z << ',' << pose.rotation.w << ')'
                          << " headingY=" << std::setprecision(1) << heading_y_degrees(pose.rotation) << "deg"
                          << " conf=" << static_cast<unsigned>(pose.tracker_confidence)
                          << '/' << static_cast<unsigned>(pose.mapper_confidence) << '\n';
            }

            if (options.duration_sec > 0.0 && host_sec >= options.duration_sec) {
                break;
            }
        }

        const auto stream_end = std::chrono::steady_clock::now();
        pipeline.stop();
        const double runtime = std::chrono::duration<double>(
            stream_end - run_start).count();
        std::cerr << std::fixed << std::setprecision(2)
                  << "[SUMMARY] runtime=" << runtime << " s received=" << received
                  << " average=" << (runtime > 0.0 ? received / runtime : 0.0) << " Hz"
                  << " missing_by_frame_number=" << missing
                  << " low_confidence_frames=" << low_confidence << '\n';
        return EXIT_SUCCESS;
    } catch (const rs2::error &error) {
        std::cerr << "[RS2 ERROR] " << error.what() << "\n"
                  << "  function: " << error.get_failed_function() << "\n"
                  << "  args: " << error.get_failed_args() << '\n';
        print_usb_state();
        return 3;
    } catch (const std::exception &error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
