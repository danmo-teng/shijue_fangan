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

struct Vec3 {
    double x;
    double y;
    double z;
};

Vec3 cross(const Vec3 &a, const Vec3 &b)
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

double dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 rotate_by_quaternion(const double q_input[4], const Vec3 &v)
{
    const double norm = std::sqrt(
        q_input[0] * q_input[0] + q_input[1] * q_input[1] +
        q_input[2] * q_input[2] + q_input[3] * q_input[3]);
    if (norm < 1e-12) return v;
    const Vec3 q{q_input[0] / norm, q_input[1] / norm, q_input[2] / norm};
    const double w = q_input[3] / norm;
    const Vec3 t = cross(q, v);
    const Vec3 twice_t{2.0 * t.x, 2.0 * t.y, 2.0 * t.z};
    const Vec3 q_cross_t = cross(q, twice_t);
    return {v.x + w * twice_t.x + q_cross_t.x,
            v.y + w * twice_t.y + q_cross_t.y,
            v.z + w * twice_t.z + q_cross_t.z};
}

Vec3 normalized_ground_xz(const Vec3 &v)
{
    const double norm = std::hypot(v.x, v.z);
    if (norm < 1e-9) return {0.0, 0.0, 0.0};
    return {v.x / norm, 0.0, v.z / norm};
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
    const Vec3 forward_camera{
        config_.camera_robot_forward_axis[0],
        config_.camera_robot_forward_axis[1],
        config_.camera_robot_forward_axis[2]};
    const Vec3 up_camera{
        config_.camera_robot_up_axis[0],
        config_.camera_robot_up_axis[1],
        config_.camera_robot_up_axis[2]};
    // Robot body axes are right-handed: forward x left = up, hence
    // left = up x forward. Config vectors are robot axes expressed in T265.
    const Vec3 left_camera = cross(up_camera, forward_camera);
    const Vec3 forward_world = normalized_ground_xz(
        rotate_by_quaternion(raw.rotation_xyzw, forward_camera));
    const Vec3 left_world = normalized_ground_xz(
        rotate_by_quaternion(raw.rotation_xyzw, left_camera));
    const double raw_yaw = std::atan2(-forward_world.x, -forward_world.z);
    const Vec3 position{
        raw.translation_m[0], raw.translation_m[1], raw.translation_m[2]};

    if (!initialized_) {
        for (int i = 0; i < 3; ++i) {
            position_origin_m_[i] = raw.translation_m[i];
        }
        forward_world_origin_[0] = forward_world.x;
        forward_world_origin_[1] = forward_world.y;
        forward_world_origin_[2] = forward_world.z;
        left_world_origin_[0] = left_world.x;
        left_world_origin_[1] = left_world.y;
        left_world_origin_[2] = left_world.z;
        yaw_origin_rad_ = raw_yaw;
        previous_raw_yaw_rad_ = raw_yaw;
        previous_timestamp_s_ = raw.timestamp_s;
        accumulated_relative_yaw_rad_ = 0.0;
        filtered_yaw_rate_radps_ = 0.0;
        initialized_ = true;
    } else {
        const double yaw_delta = wrap_angle(raw_yaw - previous_raw_yaw_rad_);
        accumulated_relative_yaw_rad_ += yaw_delta;
        const double dt = raw.timestamp_s - previous_timestamp_s_;
        if (dt > 1e-4 && dt < 0.25) {
            const double measured_rate = yaw_delta / dt;
            constexpr double alpha = 0.25;
            filtered_yaw_rate_radps_ =
                alpha * measured_rate + (1.0 - alpha) * filtered_yaw_rate_radps_;
        }
        previous_raw_yaw_rad_ = raw_yaw;
        previous_timestamp_s_ = raw.timestamp_s;
    }

    const Vec3 origin{position_origin_m_[0], position_origin_m_[1], position_origin_m_[2]};
    const Vec3 delta{position.x - origin.x, position.y - origin.y, position.z - origin.z};
    const Vec3 initial_forward{forward_world_origin_[0], forward_world_origin_[1],
                               forward_world_origin_[2]};
    const Vec3 initial_left{left_world_origin_[0], left_world_origin_[1],
                            left_world_origin_[2]};
    const double camera_initial_forward = dot(delta, initial_forward);
    const double camera_initial_left = dot(delta, initial_left);
    const double dh = accumulated_relative_yaw_rad_;

    // Convert camera-origin motion to robot-centre motion using the configured
    // planar lever arm from robot centre to T265 tracking origin.
    const double cy = std::cos(dh);
    const double sy = std::sin(dh);
    const double r0f = config_.camera_offset_forward_m;
    const double r0l = config_.camera_offset_left_m;
    const double rotated_rf = cy * r0f - sy * r0l;
    const double rotated_rl = sy * r0f + cy * r0l;
    const double robot_initial_forward = camera_initial_forward - (rotated_rf - r0f);
    const double robot_initial_left = camera_initial_left - (rotated_rl - r0l);

    const double cs = std::cos(start_pose_.yaw_rad);
    const double ss = std::sin(start_pose_.yaw_rad);
    T265FieldPose result;
    result.pose.x_m = start_pose_.x_m +
        cs * robot_initial_forward - ss * robot_initial_left;
    result.pose.y_m = start_pose_.y_m +
        ss * robot_initial_forward + cs * robot_initial_left;
    result.pose.yaw_rad = wrap_angle(start_pose_.yaw_rad + dh);
    result.travel_from_origin_m = std::hypot(robot_initial_forward, robot_initial_left);

    const Vec3 velocity{raw.velocity_mps[0], raw.velocity_mps[1], raw.velocity_mps[2]};
    const double camera_vf = dot(velocity, forward_world);
    const double camera_vl = dot(velocity, left_world);
    const double omega = filtered_yaw_rate_radps_;
    result.body_forward_velocity_mps = camera_vf + omega * r0l;
    result.body_left_velocity_mps = camera_vl - omega * r0f;
    result.forward_world[0] = forward_world.x;
    result.forward_world[1] = forward_world.y;
    result.forward_world[2] = forward_world.z;
    result.left_world[0] = left_world.x;
    result.left_world[1] = left_world.y;
    result.left_world[2] = left_world.z;
    result.raw_chassis_yaw_rad = raw_yaw;
    result.relative_yaw_rad = accumulated_relative_yaw_rad_;
    result.yaw_rate_radps = filtered_yaw_rate_radps_;
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
                             double *innovation_m,
                             double position_sigma_multiplier,
                             bool correct_position)
{
    if (!initialized_ || m.tracker_confidence == 0) return false;
    double innovation[3] = {
        m.pose.x_m - state_[0],
        m.pose.y_m - state_[1],
        wrap_angle(m.pose.yaw_rad - state_[2])
    };
    const double position_innovation = std::hypot(innovation[0], innovation[1]);
    if (innovation_m) *innovation_m = position_innovation;
    if (correct_position &&
        position_innovation > config.maximum_t265_innovation_m) {
        return false;
    }

    double base_pos_sigma;
    double yaw_sigma;
    if (m.tracker_confidence >= 3) {
        base_pos_sigma = config.t265_position_sigma_conf3_m;
        yaw_sigma = radians(config.t265_yaw_sigma_conf3_deg);
    } else if (m.tracker_confidence == 2) {
        base_pos_sigma = config.t265_position_sigma_conf2_m;
        yaw_sigma = radians(config.t265_yaw_sigma_conf2_deg);
    } else {
        base_pos_sigma = config.t265_position_sigma_conf1_m;
        yaw_sigma = radians(config.t265_yaw_sigma_conf1_deg);
    }

    const double pos_sigma = correct_position
        ? base_pos_sigma * std::max(1.0, position_sigma_multiplier)
        : 1.0e6;
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
    const double pos_floor = 0.5 * base_pos_sigma;
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
