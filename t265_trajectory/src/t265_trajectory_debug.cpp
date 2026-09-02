#include <librealsense2/rs.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_stop = 0;
constexpr double kPi = 3.14159265358979323846;

void signal_handler(int)
{
    g_stop = 1;
}

struct Options {
    double wait_sec = 30.0;
    double duration_sec = 0.0;
    double scale_px_per_m = 150.0;
    int width = 960;
    int height = 720;
    int min_confidence = 2;
    bool fullscreen = false;
    std::string serial;
    std::string csv_path;
};

struct PlanePoint {
    double right_m = 0.0;
    double forward_m = 0.0;
};

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

int parse_positive_int(const std::string &text, const char *name)
{
    try {
        std::size_t used = 0;
        const int value = std::stoi(text, &used);
        if (used != text.size() || value <= 0) {
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
        << "T265 2D trajectory display (librealsense 2.50)\n\n"
        << "Usage: " << program << " [options]\n"
        << "  --wait SEC             wait for a running T265 (default: 30)\n"
        << "  --duration SEC         stop after SEC; 0 runs until Q/Esc (default: 0)\n"
        << "  --serial SERIAL        select a specific T265\n"
        << "  --scale PX_PER_M       initial map scale (default: 150)\n"
        << "  --width PX             display width (default: 960)\n"
        << "  --height PX            display height (default: 720)\n"
        << "  --min-confidence N     record only tracker confidence N..3 (default: 2)\n"
        << "  --fullscreen           start with a fullscreen map\n"
        << "  --csv FILE             save accepted 2D samples\n"
        << "  -h, --help             show this help\n\n"
        << "Keys: R reset origin/path, F fullscreen, +/- zoom, Q or Esc quit.\n";
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
        } else if (arg == "--serial") {
            options.serial = value("--serial");
        } else if (arg == "--scale") {
            options.scale_px_per_m = parse_nonnegative(value("--scale"), "--scale");
            if (options.scale_px_per_m == 0.0) {
                throw std::invalid_argument("--scale must be positive");
            }
        } else if (arg == "--width") {
            options.width = parse_positive_int(value("--width"), "--width");
        } else if (arg == "--height") {
            options.height = parse_positive_int(value("--height"), "--height");
        } else if (arg == "--min-confidence") {
            options.min_confidence = parse_positive_int(value("--min-confidence"), "--min-confidence");
            if (options.min_confidence > 3) {
                throw std::invalid_argument("--min-confidence must be in 1..3");
            }
        } else if (arg == "--fullscreen") {
            options.fullscreen = true;
        } else if (arg == "--csv") {
            options.csv_path = value("--csv");
        } else {
            throw std::invalid_argument("unknown option: " + arg);
        }
    }
    return options;
}

std::string info(const rs2::device &device, rs2_camera_info field)
{
    try {
        return device.supports(field) ? device.get_info(field) : "";
    } catch (const rs2::error &) {
        return "";
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

double heading_y_rad(const rs2_quaternion &q)
{
    const double numerator = 2.0 * (q.w * q.y + q.x * q.z);
    const double denominator = 1.0 - 2.0 * (q.y * q.y + q.x * q.x);
    return std::atan2(numerator, denominator);
}

PlanePoint relative_plane_point(const rs2_pose &pose, const rs2_pose &origin)
{
    // T265 native: +X right, +Y up, +Z backward.  The robot map uses
    // right/forward, so forward is -Z.  The first accepted pose is the origin.
    PlanePoint result;
    result.right_m = pose.translation.x - origin.translation.x;
    result.forward_m = -(pose.translation.z - origin.translation.z);
    return result;
}

cv::Point canvas_point(const PlanePoint &point, int width, int height, double scale)
{
    return cv::Point(static_cast<int>(std::lround(width * 0.5 + point.right_m * scale)),
                     static_cast<int>(std::lround(height * 0.5 - point.forward_m * scale)));
}

double grid_step_m(double scale)
{
    static const double steps[] = {0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0};
    for (double step : steps) {
        if (step * scale >= 45.0) {
            return step;
        }
    }
    return 10.0;
}

void draw_map(cv::Mat &canvas,
              const std::deque<PlanePoint> &path,
              const PlanePoint *current,
              double heading,
              double scale,
              double path_length_m,
              int confidence,
              bool pose_accepted)
{
    canvas.setTo(cv::Scalar(22, 25, 30));
    const int width = canvas.cols;
    const int height = canvas.rows;
    const cv::Point origin(width / 2, height / 2);
    const double step_m = grid_step_m(scale);
    const int step_px = static_cast<int>(std::lround(step_m * scale));

    for (int x = origin.x; x < width; x += step_px) {
        cv::line(canvas, cv::Point(x, 0), cv::Point(x, height), cv::Scalar(47, 51, 56), 1);
    }
    for (int x = origin.x - step_px; x >= 0; x -= step_px) {
        cv::line(canvas, cv::Point(x, 0), cv::Point(x, height), cv::Scalar(47, 51, 56), 1);
    }
    for (int y = origin.y; y < height; y += step_px) {
        cv::line(canvas, cv::Point(0, y), cv::Point(width, y), cv::Scalar(47, 51, 56), 1);
    }
    for (int y = origin.y - step_px; y >= 0; y -= step_px) {
        cv::line(canvas, cv::Point(0, y), cv::Point(width, y), cv::Scalar(47, 51, 56), 1);
    }
    cv::line(canvas, cv::Point(origin.x, 0), cv::Point(origin.x, height), cv::Scalar(120, 120, 120), 1);
    cv::line(canvas, cv::Point(0, origin.y), cv::Point(width, origin.y), cv::Scalar(120, 120, 120), 1);

    if (path.size() > 1u) {
        for (std::size_t i = 1; i < path.size(); ++i) {
            cv::line(canvas, canvas_point(path[i - 1u], width, height, scale),
                     canvas_point(path[i], width, height, scale), cv::Scalar(50, 220, 80), 2, cv::LINE_AA);
        }
    }
    cv::circle(canvas, origin, 5, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    cv::putText(canvas, "start", origin + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
                0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    if (current != nullptr) {
        const cv::Point point = canvas_point(*current, width, height, scale);
        const double arrow_m = std::max(0.12, 32.0 / scale);
        // At zero yaw the camera/robot faces forward (-T265 Z). Positive T265
        // yaw turns that vector towards screen-left in the right/forward plane.
        const PlanePoint nose = {
            current->right_m - std::sin(heading) * arrow_m,
            current->forward_m + std::cos(heading) * arrow_m
        };
        const cv::Scalar color = pose_accepted ? cv::Scalar(0, 215, 255) : cv::Scalar(0, 80, 255);
        cv::circle(canvas, point, 8, color, -1, cv::LINE_AA);
        cv::arrowedLine(canvas, point, canvas_point(nose, width, height, scale), color, 3, cv::LINE_AA, 0, 0.28);
    }

    const std::string unit = (step_m < 1.0) ? " cm" : " m";
    const double shown_step = (step_m < 1.0) ? step_m * 100.0 : step_m;
    const int line_y = height - 24;
    cv::line(canvas, cv::Point(24, line_y), cv::Point(24 + step_px, line_y), cv::Scalar(255, 255, 255), 2);
    cv::putText(canvas, cv::format("grid %.0f%s", shown_step, unit.c_str()), cv::Point(24, line_y - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    cv::putText(canvas, cv::format("path %.3f m  tracker confidence %d%s", path_length_m, confidence,
                                   pose_accepted ? "" : " (not recorded)"),
                cv::Point(16, 28), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                pose_accepted ? cv::Scalar(235, 235, 235) : cv::Scalar(0, 100, 255), 2, cv::LINE_AA);
    cv::putText(canvas, "T265 local plane: +right=X, +forward=-Z | R reset | +/- zoom | Q quit",
                cv::Point(16, 54), cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        const Options options = parse_options(argc, argv);
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        rs2::log_to_console(RS2_LOG_SEVERITY_WARN);

        const auto wait_start = std::chrono::steady_clock::now();
        std::unique_ptr<rs2::context> context;
        rs2::device selected;
        while (!g_stop) {
            context.reset(new rs2::context());
            const rs2::device_list devices = context->query_devices(RS2_PRODUCT_LINE_T200);
            const int index = select_device(devices, options.serial);
            if (index >= 0) {
                selected = devices[static_cast<std::size_t>(index)];
                break;
            }
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() >= options.wait_sec) {
                throw std::runtime_error("no running T265 found before --wait timeout");
            }
            std::cerr << "[WAIT] no running T265; retrying in 2 s...\n";
            context.reset();
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        if (g_stop) {
            return 130;
        }

        const std::string serial = info(selected, RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::cerr << "[READY] " << info(selected, RS2_CAMERA_INFO_NAME) << " serial=" << serial << '\n';
        selected = rs2::device();  // Release the USB handle before pipeline.start().

        std::ofstream csv;
        if (!options.csv_path.empty()) {
            csv.open(options.csv_path);
            if (!csv) {
                throw std::runtime_error("cannot open CSV: " + options.csv_path);
            }
            csv << "host_time_s,right_m,forward_m,heading_deg,path_length_m,tracker_confidence\n";
        }

        rs2::pipeline pipeline(*context);
        rs2::config config;
        config.enable_device(serial);
        config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        pipeline.start(config);

        const std::string window_name = "T265 trajectory (R reset, F fullscreen, +/- zoom, Q quit)";
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name, options.width, options.height);
        bool fullscreen = options.fullscreen;
        if (fullscreen) {
            cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        }
        cv::Mat canvas(options.height, options.width, CV_8UC3);
        std::deque<PlanePoint> path;
        rs2_pose origin{};
        bool have_origin = false;
        PlanePoint current{};
        bool have_current = false;
        double heading = 0.0;
        int confidence = 0;
        bool last_accepted = false;
        double path_length_m = 0.0;
        double scale = options.scale_px_per_m;
        const auto run_start = std::chrono::steady_clock::now();

        while (!g_stop) {
            const rs2::frameset frames = pipeline.wait_for_frames(1000);
            const rs2::pose_frame pose_frame = frames.get_pose_frame();
            if (!pose_frame) {
                continue;
            }
            const rs2_pose pose = pose_frame.get_pose_data();
            confidence = static_cast<int>(pose.tracker_confidence);
            heading = heading_y_rad(pose.rotation);
            last_accepted = confidence >= options.min_confidence;

            if (last_accepted) {
                if (!have_origin) {
                    origin = pose;
                    have_origin = true;
                    path.clear();
                    path.push_back(PlanePoint{});
                    path_length_m = 0.0;
                }
                const PlanePoint next = relative_plane_point(pose, origin);
                if (have_current) {
                    path_length_m += std::hypot(next.right_m - current.right_m,
                                                next.forward_m - current.forward_m);
                }
                current = next;
                have_current = true;
                path.push_back(current);
                constexpr std::size_t max_path_points = 12000u;
                if (path.size() > max_path_points) {
                    path.pop_front();
                }
                if (csv) {
                    const double host_time = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - run_start).count();
                    csv << std::fixed << std::setprecision(6) << host_time << ','
                        << current.right_m << ',' << current.forward_m << ','
                        << heading * 180.0 / kPi << ',' << path_length_m << ',' << confidence << '\n';
                }
            }

            draw_map(canvas, path, have_current ? &current : nullptr, heading, scale,
                     path_length_m, confidence, last_accepted);
            cv::imshow(window_name, canvas);
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
            if (key == 'r' || key == 'R') {
                have_origin = false;
                have_current = false;
                path.clear();
                path_length_m = 0.0;
            } else if (key == 'f' || key == 'F') {
                fullscreen = !fullscreen;
                cv::setWindowProperty(
                    window_name, cv::WND_PROP_FULLSCREEN,
                    fullscreen ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);
            } else if (key == '+' || key == '=') {
                scale = std::min(scale * 1.25, 2000.0);
            } else if (key == '-' || key == '_') {
                scale = std::max(scale / 1.25, 10.0);
            }
            if (options.duration_sec > 0.0 &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - run_start).count() >= options.duration_sec) {
                break;
            }
        }
        pipeline.stop();
        cv::destroyWindow(window_name);
        return 0;
    } catch (const rs2::error &error) {
        std::cerr << "[RS2 ERROR] " << error.what() << '\n';
        return 3;
    } catch (const std::exception &error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
