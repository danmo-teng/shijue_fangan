#ifndef T265_OMNI_CONFIG_HPP
#define T265_OMNI_CONFIG_HPP

#include <string>

namespace omni {

struct LocalizationConfig {
    int start_zone = 4;
    double field_half_m = 1.5;
    double start_center_m = 1.35;

    double wheel_diameter_m = 0.070;
    double counts_per_wheel_revolution = 1768.0;
    int encoder_sign[3] = {-1, -1, -1};
    double wheel_center_radius_m = 0.0;

    double camera_offset_forward_m = 0.0;
    double camera_offset_left_m = 0.0;

    double startup_wheel_disable_distance_m = 0.70;
    double corner_exclusion_inner_m = 0.75;
    double maximum_wheel_speed_mps = 1.50;
    double maximum_velocity_residual_mps = 0.70;
    int uart_stale_ms = 150;

    double wheel_position_sigma_floor_m = 0.002;
    double wheel_position_sigma_per_meter = 0.04;
    double wheel_yaw_sigma_floor_deg = 0.20;
    double wheel_yaw_sigma_per_radian = 0.08;

    double t265_position_sigma_conf3_m = 0.015;
    double t265_position_sigma_conf2_m = 0.040;
    double t265_position_sigma_conf1_m = 0.250;
    double t265_yaw_sigma_conf3_deg = 1.0;
    double t265_yaw_sigma_conf2_deg = 3.0;
    double t265_yaw_sigma_conf1_deg = 15.0;
    double maximum_t265_innovation_m = 0.60;
};

LocalizationConfig load_config(const std::string &path);
void validate_config(const LocalizationConfig &config);

}  // namespace omni

#endif
