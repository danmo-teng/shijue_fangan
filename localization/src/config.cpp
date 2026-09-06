#include "config.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace omni {
namespace {

std::string trim(const std::string &input)
{
    const auto first = std::find_if_not(input.begin(), input.end(),
                                        [](unsigned char c) { return std::isspace(c); });
    const auto last = std::find_if_not(input.rbegin(), input.rend(),
                                       [](unsigned char c) { return std::isspace(c); }).base();
    return first < last ? std::string(first, last) : std::string();
}

double number(const std::string &text, const std::string &key)
{
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used != text.size()) {
            throw std::invalid_argument("suffix");
        }
        return value;
    } catch (const std::exception &) {
        throw std::runtime_error("invalid numeric value for " + key + ": " + text);
    }
}

int integer(const std::string &text, const std::string &key)
{
    const double value = number(text, key);
    const int result = static_cast<int>(value);
    if (static_cast<double>(result) != value) {
        throw std::runtime_error("value for " + key + " must be an integer");
    }
    return result;
}

void axis_vector(const std::string &text, const std::string &key,
                 double output[3])
{
    const std::map<std::string, std::array<double, 3>> axes = {
        {"+x", {{1.0, 0.0, 0.0}}}, {"x", {{1.0, 0.0, 0.0}}},
        {"-x", {{-1.0, 0.0, 0.0}}},
        {"+y", {{0.0, 1.0, 0.0}}}, {"y", {{0.0, 1.0, 0.0}}},
        {"-y", {{0.0, -1.0, 0.0}}},
        {"+z", {{0.0, 0.0, 1.0}}}, {"z", {{0.0, 0.0, 1.0}}},
        {"-z", {{0.0, 0.0, -1.0}}},
    };
    std::string normalized;
    for (char c : text) {
        normalized.push_back(static_cast<char>(
            std::tolower(static_cast<unsigned char>(c))));
    }
    const auto found = axes.find(normalized);
    if (found == axes.end()) {
        throw std::runtime_error(key + " must be one of +/-x, +/-y, +/-z");
    }
    std::copy(found->second.begin(), found->second.end(), output);
}

}  // namespace

LocalizationConfig load_config(const std::string &path)
{
    LocalizationConfig config;
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open localization config: " + path);
    }

    std::string line;
    unsigned line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const std::size_t equals = line.find('=');
        if (equals == std::string::npos) {
            throw std::runtime_error("config line " + std::to_string(line_number) +
                                     " has no '='");
        }
        const std::string key = trim(line.substr(0, equals));
        const std::string value = trim(line.substr(equals + 1));

#define SET_DOUBLE(name) if (key == #name) { config.name = number(value, key); continue; }
        SET_DOUBLE(field_half_m)
        SET_DOUBLE(start_center_m)
        SET_DOUBLE(wheel_diameter_m)
        SET_DOUBLE(counts_per_wheel_revolution)
        SET_DOUBLE(wheel_center_radius_m)
        SET_DOUBLE(camera_offset_forward_m)
        SET_DOUBLE(camera_offset_left_m)
        SET_DOUBLE(startup_wheel_disable_distance_m)
        SET_DOUBLE(corner_exclusion_inner_m)
        SET_DOUBLE(maximum_wheel_speed_mps)
        SET_DOUBLE(maximum_velocity_residual_mps)
        SET_DOUBLE(navigation_t265_position_sigma_multiplier)
        SET_DOUBLE(navigation_t265_position_correction_rate_hz)
        SET_DOUBLE(mapper_zero_position_sigma_multiplier)
        SET_DOUBLE(navigation_near_target_m)
        SET_DOUBLE(navigation_slip_wheel_speed_mps)
        SET_DOUBLE(navigation_slip_t265_speed_mps)
        SET_DOUBLE(wheel_position_sigma_floor_m)
        SET_DOUBLE(wheel_position_sigma_per_meter)
        SET_DOUBLE(wheel_yaw_sigma_floor_deg)
        SET_DOUBLE(wheel_yaw_sigma_per_radian)
        SET_DOUBLE(t265_position_sigma_conf3_m)
        SET_DOUBLE(t265_position_sigma_conf2_m)
        SET_DOUBLE(t265_position_sigma_conf1_m)
        SET_DOUBLE(t265_yaw_sigma_conf3_deg)
        SET_DOUBLE(t265_yaw_sigma_conf2_deg)
        SET_DOUBLE(t265_yaw_sigma_conf1_deg)
        SET_DOUBLE(maximum_t265_innovation_m)
#undef SET_DOUBLE
        if (key == "start_zone") { config.start_zone = integer(value, key); continue; }
        if (key == "encoder_sign_m1") { config.encoder_sign[0] = integer(value, key); continue; }
        if (key == "encoder_sign_m2") { config.encoder_sign[1] = integer(value, key); continue; }
        if (key == "encoder_sign_m3") { config.encoder_sign[2] = integer(value, key); continue; }
        if (key == "camera_robot_forward_axis") {
            axis_vector(value, key, config.camera_robot_forward_axis); continue;
        }
        if (key == "camera_robot_up_axis") {
            axis_vector(value, key, config.camera_robot_up_axis); continue;
        }
        if (key == "uart_stale_ms") { config.uart_stale_ms = integer(value, key); continue; }
        throw std::runtime_error("unknown config key on line " +
                                 std::to_string(line_number) + ": " + key);
    }
    validate_config(config);
    return config;
}

void validate_config(const LocalizationConfig &c)
{
    if (c.start_zone < 1 || c.start_zone > 4) {
        throw std::runtime_error("start_zone must be 1..4");
    }
    if (c.field_half_m <= 0.0 || c.start_center_m <= 0.0 ||
        c.start_center_m >= c.field_half_m) {
        throw std::runtime_error("invalid field/start dimensions");
    }
    if (c.wheel_diameter_m <= 0.0 || c.counts_per_wheel_revolution <= 0.0) {
        throw std::runtime_error("wheel diameter and encoder counts must be positive");
    }
    for (int sign : c.encoder_sign) {
        if (sign != -1 && sign != 1) {
            throw std::runtime_error("every encoder_sign must be -1 or +1");
        }
    }
    if (c.wheel_center_radius_m < 0.0 || c.maximum_wheel_speed_mps <= 0.0 ||
        c.maximum_velocity_residual_mps <= 0.0 || c.uart_stale_ms <= 0) {
        throw std::runtime_error("invalid wheel/gating parameter");
    }
    if (c.navigation_t265_position_sigma_multiplier < 1.0 ||
        c.navigation_t265_position_correction_rate_hz <= 0.0 ||
        c.mapper_zero_position_sigma_multiplier < 1.0 ||
        c.navigation_near_target_m <= 0.0 ||
        c.navigation_slip_wheel_speed_mps <= 0.0 ||
        c.navigation_slip_t265_speed_mps < 0.0) {
        throw std::runtime_error("invalid navigation fusion parameter");
    }
    double forward_norm = 0.0;
    double up_norm = 0.0;
    double dot = 0.0;
    for (int i = 0; i < 3; ++i) {
        forward_norm += c.camera_robot_forward_axis[i] * c.camera_robot_forward_axis[i];
        up_norm += c.camera_robot_up_axis[i] * c.camera_robot_up_axis[i];
        dot += c.camera_robot_forward_axis[i] * c.camera_robot_up_axis[i];
    }
    if (std::fabs(forward_norm - 1.0) > 1e-9 ||
        std::fabs(up_norm - 1.0) > 1e-9 || std::fabs(dot) > 1e-9) {
        throw std::runtime_error("camera robot forward/up axes must be orthogonal unit axes");
    }
}

}  // namespace omni
