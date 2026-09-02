#include "fusion.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace omni {
namespace {

Pose2d start_pose_for_zone(const LocalizationConfig &config)
{
    Pose2d pose;
    const bool right = config.start_zone == 2 || config.start_zone == 4;
    const bool top = config.start_zone == 1 || config.start_zone == 2;
    pose.x_m = right ? config.start_center_m : -config.start_center_m;
    pose.y_m = top ? config.start_center_m : -config.start_center_m;
    const double heading_deg[] = {0.0, 135.0, 45.0, 225.0, 315.0};
    pose.yaw_rad = radians(heading_deg[config.start_zone]);
    return pose;
}

bool inverse3(const double input[3][3], double output[3][3])
{
    const double determinant =
        input[0][0] * (input[1][1] * input[2][2] - input[1][2] * input[2][1]) -
        input[0][1] * (input[1][0] * input[2][2] - input[1][2] * input[2][0]) +
        input[0][2] * (input[1][0] * input[2][1] - input[1][1] * input[2][0]);
    if (std::fabs(determinant) < 1e-18) {
        return false;
    }
    const double inv = 1.0 / determinant;
    output[0][0] =  (input[1][1] * input[2][2] - input[1][2] * input[2][1]) * inv;
    output[0][1] = -(input[0][1] * input[2][2] - input[0][2] * input[2][1]) * inv;
    output[0][2] =  (input[0][1] * input[1][2] - input[0][2] * input[1][1]) * inv;
    output[1][0] = -(input[1][0] * input[2][2] - input[1][2] * input[2][0]) * inv;
    output[1][1] =  (input[0][0] * input[2][2] - input[0][2] * input[2][0]) * inv;
    output[1][2] = -(input[0][0] * input[1][2] - input[0][2] * input[1][0]) * inv;
    output[2][0] =  (input[1][0] * input[2][1] - input[1][1] * input[2][0]) * inv;
    output[2][1] = -(input[0][0] * input[2][1] - input[0][1] * input[2][0]) * inv;
    output[2][2] =  (input[0][0] * input[1][1] - input[0][1] * input[1][0]) * inv;
    return true;
}

}  // namespace

double radians(double value)
{
    return value * kPi / 180.0;
}

double degrees(double value)
{
    return value * 180.0 / kPi;
}

double wrap_angle(double value)
{
    while (value > kPi) value -= 2.0 * kPi;
    while (value <= -kPi) value += 2.0 * kPi;
    return value;
}

T265FieldProjector::T265FieldProjector(const LocalizationConfig &config)
    : config_(config), start_pose_(start_pose_for_zone(config))
{
}

T265FieldPose T265FieldProjector::project(const T265RawPose &raw)
{
    // T265 native: +X right, +Y up, +Z backward. Robot planar axes are
    // forward=-Z and left=-X.
    const double raw_forward = -raw.native_z_m;
    const double raw_left = -raw.native_x_m;
    if (!initialized_) {
        raw_forward_origin_m_ = raw_forward;
        raw_left_origin_m_ = raw_left;
        raw_heading_origin_rad_ = raw.heading_y_rad;
        initialized_ = true;
    }

    const double dh = wrap_angle(raw.heading_y_rad - raw_heading_origin_rad_);
    const double c0 = std::cos(raw_heading_origin_rad_);
    const double s0 = std::sin(raw_heading_origin_rad_);
    const double camera_df = raw_forward - raw_forward_origin_m_;
    const double camera_dl = raw_left - raw_left_origin_m_;
    const double camera_initial_forward = c0 * camera_df + s0 * camera_dl;
    const double camera_initial_left = -s0 * camera_df + c0 * camera_dl;

    // Fixed planar mounting correction from the T265-derived axes into chassis
    // axes. Positive rotates the measured vector counter-clockwise; this robot
    // uses -90 degrees because its old map motion was 90 degrees CCW from the
    // actual travel direction.
    const double mount = radians(config_.camera_to_robot_yaw_deg);
    const double cm = std::cos(mount);
    const double sm = std::sin(mount);
    const double mounted_initial_forward =
        cm * camera_initial_forward - sm * camera_initial_left;
    const double mounted_initial_left =
        sm * camera_initial_forward + cm * camera_initial_left;

    // Convert camera-origin motion to robot-centre motion using the configured
    // planar lever arm from robot centre to T265 tracking origin.
    const double cy = std::cos(dh);
    const double sy = std::sin(dh);
    const double r0f = config_.camera_offset_forward_m;
    const double r0l = config_.camera_offset_left_m;
    const double rotated_rf = cy * r0f - sy * r0l;
    const double rotated_rl = sy * r0f + cy * r0l;
    const double robot_initial_forward = mounted_initial_forward - (rotated_rf - r0f);
    const double robot_initial_left = mounted_initial_left - (rotated_rl - r0l);

    const double cs = std::cos(start_pose_.yaw_rad);
    const double ss = std::sin(start_pose_.yaw_rad);
    T265FieldPose result;
    result.pose.x_m = start_pose_.x_m +
        cs * robot_initial_forward - ss * robot_initial_left;
    result.pose.y_m = start_pose_.y_m +
        ss * robot_initial_forward + cs * robot_initial_left;
    result.pose.yaw_rad = wrap_angle(start_pose_.yaw_rad + dh);
    result.travel_from_origin_m = std::hypot(robot_initial_forward, robot_initial_left);

    const double native_forward_velocity = -raw.native_vz_mps;
    const double native_left_velocity = -raw.native_vx_mps;
    const double camera_vf = c0 * native_forward_velocity + s0 * native_left_velocity;
    const double camera_vl = -s0 * native_forward_velocity + c0 * native_left_velocity;
    const double mounted_vf = cm * camera_vf - sm * camera_vl;
    const double mounted_vl = sm * camera_vf + cm * camera_vl;
    const double omega = raw.angular_velocity_y_radps;
    const double lever_cross_f = -omega * rotated_rl;
    const double lever_cross_l = omega * rotated_rf;
    const double base_initial_vf = mounted_vf - lever_cross_f;
    const double base_initial_vl = mounted_vl - lever_cross_l;
    // Initial robot axes -> current robot axes.
    result.body_forward_velocity_mps = cy * base_initial_vf + sy * base_initial_vl;
    result.body_left_velocity_mps = -sy * base_initial_vf + cy * base_initial_vl;
    result.tracker_confidence = raw.tracker_confidence;
    result.mapper_confidence = raw.mapper_confidence;
    return result;
}

OmniEncoderIntegrator::OmniEncoderIntegrator(const LocalizationConfig &config)
    : config_(config)
{
}

void OmniEncoderIntegrator::reset()
{
    have_previous_ = false;
}

bool OmniEncoderIntegrator::update(const EncoderFrame &frame,
                                   WheelIncrement &increment,
                                   std::string &reason)
{
    const std::uint8_t all_valid = kOdomM1Valid | kOdomM2Valid | kOdomM3Valid;
    if ((frame.status & kOdomCounterReset) != 0u) {
        have_previous_ = false;
        reason = "counter_reset";
    }
    if ((frame.status & all_valid) != all_valid ||
        (frame.status & kOdomEncoderFault) != 0u) {
        have_previous_ = false;
        reason = "invalid_encoder";
        return false;
    }
    if (!have_previous_) {
        for (int i = 0; i < 3; ++i) previous_[i] = frame.position[i];
        previous_sequence_ = frame.sequence;
        have_previous_ = true;
        reason = "baseline";
        return false;
    }

    const std::uint8_t sequence_step =
        static_cast<std::uint8_t>(frame.sequence - previous_sequence_);
    if (sequence_step == 0u) {
        reason = "duplicate";
        return false;
    }
    const double dt = static_cast<double>(frame.sample_period_ms) *
                      static_cast<double>(sequence_step) * 0.001;
    if (dt <= 0.0 || dt > 1.0) {
        for (int i = 0; i < 3; ++i) previous_[i] = frame.position[i];
        previous_sequence_ = frame.sequence;
        reason = "invalid_dt";
        return false;
    }

    const double metres_per_count = kPi * config_.wheel_diameter_m /
                                    config_.counts_per_wheel_revolution;
    double wheel_m[3];
    for (int i = 0; i < 3; ++i) {
        const std::int16_t delta = static_cast<std::int16_t>(
            static_cast<std::uint16_t>(frame.position[i] - previous_[i]));
        wheel_m[i] = static_cast<double>(delta) *
                     static_cast<double>(config_.encoder_sign[i]) * metres_per_count;
        previous_[i] = frame.position[i];
    }
    previous_sequence_ = frame.sequence;

    increment.forward_m = (wheel_m[2] - wheel_m[0]) / std::sqrt(3.0);
    increment.left_m = (wheel_m[0] + wheel_m[2] - 2.0 * wheel_m[1]) / 3.0;
    const double rotation_tangent_m = (wheel_m[0] + wheel_m[1] + wheel_m[2]) / 3.0;
    increment.yaw_rad = config_.wheel_center_radius_m > 0.0
        ? rotation_tangent_m / config_.wheel_center_radius_m : 0.0;
    increment.dt_s = dt;
    increment.forward_velocity_mps = increment.forward_m / dt;
    increment.left_velocity_mps = increment.left_m / dt;
    increment.sequence_step = sequence_step;
    reason = "ok";
    return true;
}

const char *wheel_gate_reason_name(WheelGateReason reason)
{
    switch (reason) {
        case WheelGateReason::Accepted: return "accepted";
        case WheelGateReason::NoBaseline: return "no_baseline";
        case WheelGateReason::InvalidEncoder: return "invalid_encoder";
        case WheelGateReason::CounterReset: return "counter_reset";
        case WheelGateReason::StartupObstacle: return "startup_obstacle";
        case WheelGateReason::CornerObstacle: return "corner_obstacle";
        case WheelGateReason::ExcessiveSpeed: return "excessive_speed";
        case WheelGateReason::VelocityMismatch: return "velocity_mismatch";
    }
    return "unknown";
}

WheelGateReason evaluate_wheel_gate(const LocalizationConfig &config,
                                    const T265FieldPose &t265,
                                    const WheelIncrement &wheel)
{
    if (t265.travel_from_origin_m < config.startup_wheel_disable_distance_m) {
        return WheelGateReason::StartupObstacle;
    }
    if (std::fabs(t265.pose.x_m) > config.corner_exclusion_inner_m &&
        std::fabs(t265.pose.y_m) > config.corner_exclusion_inner_m) {
        return WheelGateReason::CornerObstacle;
    }
    const double wheel_speed = std::hypot(wheel.forward_velocity_mps,
                                          wheel.left_velocity_mps);
    if (wheel_speed > config.maximum_wheel_speed_mps) {
        return WheelGateReason::ExcessiveSpeed;
    }
    if (t265.tracker_confidence >= 2) {
        const double residual = std::hypot(
            wheel.forward_velocity_mps - t265.body_forward_velocity_mps,
            wheel.left_velocity_mps - t265.body_left_velocity_mps);
        if (residual > config.maximum_velocity_residual_mps) {
            return WheelGateReason::VelocityMismatch;
        }
    }
    return WheelGateReason::Accepted;
}

void PlanarEkf::initialize(const Pose2d &pose)
{
    state_[0] = pose.x_m;
    state_[1] = pose.y_m;
    state_[2] = wrap_angle(pose.yaw_rad);
    std::memset(covariance_, 0, sizeof(covariance_));
    covariance_[0][0] = 0.01 * 0.01;
    covariance_[1][1] = 0.01 * 0.01;
    covariance_[2][2] = radians(2.0) * radians(2.0);
    initialized_ = true;
}

void PlanarEkf::predict(const WheelIncrement &u, const LocalizationConfig &config)
{
    if (!initialized_) return;
    const double middle_yaw = state_[2] + 0.5 * u.yaw_rad;
    const double c = std::cos(middle_yaw);
    const double s = std::sin(middle_yaw);
    state_[0] += c * u.forward_m - s * u.left_m;
    state_[1] += s * u.forward_m + c * u.left_m;
    state_[2] = wrap_angle(state_[2] + u.yaw_rad);

    double f[3][3] = {
        {1.0, 0.0, -s * u.forward_m - c * u.left_m},
        {0.0, 1.0,  c * u.forward_m - s * u.left_m},
        {0.0, 0.0, 1.0}
    };
    double fp[3][3] = {{0.0}};
    double predicted[3][3] = {{0.0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                fp[i][j] += f[i][k] * covariance_[k][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                predicted[i][j] += fp[i][k] * f[j][k];

    const double distance = std::hypot(u.forward_m, u.left_m);
    const double pos_sigma = config.wheel_position_sigma_floor_m +
                             config.wheel_position_sigma_per_meter * distance;
    const double yaw_sigma = radians(config.wheel_yaw_sigma_floor_deg) +
                             config.wheel_yaw_sigma_per_radian * std::fabs(u.yaw_rad);
    predicted[0][0] += pos_sigma * pos_sigma;
    predicted[1][1] += pos_sigma * pos_sigma;
    predicted[2][2] += yaw_sigma * yaw_sigma;
    std::memcpy(covariance_, predicted, sizeof(covariance_));
}

bool PlanarEkf::correct_t265(const T265FieldPose &m,
                             const LocalizationConfig &config,
                             double *innovation_m)
{
    if (!initialized_ || m.tracker_confidence == 0) return false;
    double innovation[3] = {
        m.pose.x_m - state_[0],
        m.pose.y_m - state_[1],
        wrap_angle(m.pose.yaw_rad - state_[2])
    };
    const double position_innovation = std::hypot(innovation[0], innovation[1]);
    if (innovation_m) *innovation_m = position_innovation;
    if (position_innovation > config.maximum_t265_innovation_m) {
        return false;
    }

    double pos_sigma;
    double yaw_sigma;
    if (m.tracker_confidence >= 3) {
        pos_sigma = config.t265_position_sigma_conf3_m;
        yaw_sigma = radians(config.t265_yaw_sigma_conf3_deg);
    } else if (m.tracker_confidence == 2) {
        pos_sigma = config.t265_position_sigma_conf2_m;
        yaw_sigma = radians(config.t265_yaw_sigma_conf2_deg);
    } else {
        pos_sigma = config.t265_position_sigma_conf1_m;
        yaw_sigma = radians(config.t265_yaw_sigma_conf1_deg);
    }

    double s[3][3];
    std::memcpy(s, covariance_, sizeof(s));
    s[0][0] += pos_sigma * pos_sigma;
    s[1][1] += pos_sigma * pos_sigma;
    s[2][2] += yaw_sigma * yaw_sigma;
    double inverse_s[3][3];
    if (!inverse3(s, inverse_s)) return false;

    double gain[3][3] = {{0.0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                gain[i][j] += covariance_[i][k] * inverse_s[k][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            state_[i] += gain[i][j] * innovation[j];
    state_[2] = wrap_angle(state_[2]);

    double updated[3][3] = {{0.0}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            const double identity_minus_gain = (i == j ? 1.0 : 0.0) - gain[i][j];
            for (int k = 0; k < 3; ++k) {
                const double imk = (i == k ? 1.0 : 0.0) - gain[i][k];
                updated[i][j] += imk * covariance_[k][j];
            }
            (void)identity_minus_gain;
        }
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j) {
            const double symmetric = 0.5 * (updated[i][j] + updated[j][i]);
            updated[i][j] = updated[j][i] = symmetric;
        }
    // Consecutive T265 frames are strongly time-correlated. Without a floor,
    // treating 200 Hz frames as independent drives covariance unrealistically
    // close to zero while the camera is standing still.
    const double pos_floor = 0.5 * pos_sigma;
    const double yaw_floor = 0.5 * yaw_sigma;
    updated[0][0] = std::max(updated[0][0], pos_floor * pos_floor);
    updated[1][1] = std::max(updated[1][1], pos_floor * pos_floor);
    updated[2][2] = std::max(updated[2][2], yaw_floor * yaw_floor);
    std::memcpy(covariance_, updated, sizeof(covariance_));
    return true;
}

Pose2d PlanarEkf::pose() const
{
    return {state_[0], state_[1], state_[2]};
}

double PlanarEkf::position_sigma_m() const
{
    return std::sqrt(std::max(0.0, 0.5 * (covariance_[0][0] + covariance_[1][1])));
}

double PlanarEkf::yaw_sigma_rad() const
{
    return std::sqrt(std::max(0.0, covariance_[2][2]));
}

}  // namespace omni
