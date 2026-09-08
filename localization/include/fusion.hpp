#ifndef T265_OMNI_FUSION_HPP
#define T265_OMNI_FUSION_HPP

#include "config.hpp"
#include "f407_protocol.hpp"

#include <cstdint>
#include <deque>
#include <string>

namespace omni {

constexpr double kPi = 3.14159265358979323846;

double radians(double degrees);
double degrees(double radians_value);
double wrap_angle(double radians_value);

struct Pose2d {
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
};

struct T265RawPose {
    double translation_m[3] = {0.0, 0.0, 0.0};
    double velocity_mps[3] = {0.0, 0.0, 0.0};
    double rotation_xyzw[4] = {0.0, 0.0, 0.0, 1.0};
    double angular_velocity_radps[3] = {0.0, 0.0, 0.0};
    double timestamp_s = 0.0;
    std::uint8_t tracker_confidence = 0;
    std::uint8_t mapper_confidence = 0;
};

struct T265FieldPose {
    Pose2d pose;
    Pose2d tracking_origin_pose;
    double camera_offset_forward_m = 0.0;
    double camera_offset_left_m = 0.0;
    double tracking_origin_delta_forward_m = 0.0;
    double tracking_origin_delta_left_m = 0.0;
    double unscaled_robot_center_delta_forward_m = 0.0;
    double unscaled_robot_center_delta_left_m = 0.0;
    double robot_center_delta_forward_m = 0.0;
    double robot_center_delta_left_m = 0.0;
    double translation_scale = 1.0;
    bool translation_scale_enabled = false;
    double body_forward_velocity_mps = 0.0;
    double body_left_velocity_mps = 0.0;
    double travel_from_origin_m = 0.0;
    double forward_world[3] = {0.0, 0.0, -1.0};
    double left_world[3] = {-1.0, 0.0, 0.0};
    double raw_chassis_yaw_rad = 0.0;
    double relative_yaw_rad = 0.0;
    double yaw_rate_radps = 0.0;
    // Quaternion-difference yaw rate remains available for diagnostics. The
    // gyro rate is the high-rate yaw input used by wheel odometry prediction.
    double gyro_relative_yaw_rad = 0.0;
    double gyro_yaw_rate_radps = 0.0;
    bool gyro_yaw_rate_valid = false;
    // Signed difference between the unwrapped quaternion-yaw timeline and the
    // gyro timeline since the most recent periodic pose re-synchronization.
    double gyro_pose_sync_error_rad = 0.0;
    std::uint8_t tracker_confidence = 0;
    std::uint8_t mapper_confidence = 0;
};

class T265FieldProjector {
public:
    explicit T265FieldProjector(const LocalizationConfig &config);
    bool initialized() const noexcept { return initialized_; }
    T265FieldPose project(const T265RawPose &raw);

private:
    LocalizationConfig config_;
    bool initialized_ = false;
    double position_origin_m_[3] = {0.0, 0.0, 0.0};
    double forward_world_origin_[3] = {0.0, 0.0, -1.0};
    double left_world_origin_[3] = {-1.0, 0.0, 0.0};
    double yaw_origin_rad_ = 0.0;
    double previous_raw_yaw_rad_ = 0.0;
    double previous_timestamp_s_ = 0.0;
    double accumulated_relative_yaw_rad_ = 0.0;
    double filtered_yaw_rate_radps_ = 0.0;
    double accumulated_gyro_yaw_rad_ = 0.0;
    double filtered_gyro_yaw_rate_radps_ = 0.0;
    bool have_gyro_pose_resync_timestamp_ = false;
    double last_gyro_pose_resync_timestamp_s_ = 0.0;
    Pose2d start_pose_{};
};

// Host-time view of the T265 gyro-integrated yaw. T265 pose samples arrive
// asynchronously to F407 ODOM frames, so the latter must query this timeline
// at the encoder frame's receive time instead of using the newest gyro rate
// multiplied by an F407 sample interval.
class T265YawSynchronizer {
public:
    void reset();
    bool update(double host_time_s, const T265FieldPose &pose);
    bool yaw_at(double host_time_s, double &yaw_rad) const;
    bool initialized() const noexcept { return !samples_.empty(); }
    double latest_yaw_rad() const noexcept;

private:
    struct Sample {
        double host_time_s = 0.0;
        double yaw_rad = 0.0;
        double yaw_rate_radps = 0.0;
    };

    std::deque<Sample> samples_;
    bool pose_yaw_initialized_ = false;
    bool gyro_reference_initialized_ = false;
    double previous_pose_yaw_rad_ = 0.0;
    double pose_yaw_unwrapped_rad_ = 0.0;
    double gyro_yaw_anchor_rad_ = 0.0;
};

struct WheelIncrement {
    double forward_m = 0.0;
    double left_m = 0.0;
    double yaw_rad = 0.0;
    // Optional field yaw at the midpoint of this increment. It is populated
    // by the runtime from the T265 timeline and lets both wheel odometry and
    // EKF transform F407 body translation at the matching heading.
    double field_yaw_rad = 0.0;
    bool field_yaw_valid = false;
    double dt_s = 0.0;
    double forward_velocity_mps = 0.0;
    double left_velocity_mps = 0.0;
    std::uint8_t sequence_step = 0;
};

WheelIncrement apply_encoder_fusion_weight(const WheelIncrement &increment,
                                            double weight);
bool navigation_wheel_primary_enabled(bool navigation_active,
                                      bool encoders_enabled,
                                      bool accepted_wheel_fresh,
                                      double encoder_fusion_weight);
double weighted_t265_sigma_multiplier(double configured_multiplier,
                                      double encoder_fusion_weight);

class OmniEncoderIntegrator {
public:
    explicit OmniEncoderIntegrator(const LocalizationConfig &config);
    bool update(const EncoderFrame &frame, WheelIncrement &increment,
                std::string &reason);
    void reset();

private:
    LocalizationConfig config_;
    bool have_previous_ = false;
    std::uint16_t previous_[3] = {0, 0, 0};
    std::uint8_t previous_sequence_ = 0;
};

// Dead-reckon valid wheel increments in the same field frame as the EKF.  This
// is intentionally kept separate from PlanarEkf so the runtime diagnostic can
// show encoder-only drift alongside the T265 and fused poses.
class PlanarOdometry {
public:
    void initialize(const Pose2d &pose);
    bool initialized() const noexcept { return initialized_; }
    void integrate(const WheelIncrement &increment);
    Pose2d pose() const;
    double travel_m() const noexcept { return travel_m_; }

private:
    bool initialized_ = false;
    Pose2d pose_{};
    double travel_m_ = 0.0;
};

enum class WheelGateReason {
    Accepted,
    NoBaseline,
    InvalidEncoder,
    CounterReset,
    StartupObstacle,
    CornerObstacle,
    ExcessiveSpeed,
    VelocityMismatch
};

const char *wheel_gate_reason_name(WheelGateReason reason);

WheelGateReason evaluate_wheel_gate(const LocalizationConfig &config,
                                    const T265FieldPose &t265,
                                    const WheelIncrement &wheel);

class PlanarEkf {
public:
    void initialize(const Pose2d &pose);
    bool initialized() const noexcept { return initialized_; }
    void predict(const WheelIncrement &increment,
                 const LocalizationConfig &config);
    bool correct_t265(const T265FieldPose &measurement,
                      const LocalizationConfig &config,
                      double *innovation_m = nullptr,
                      double position_sigma_multiplier = 1.0,
                      bool correct_position = true);
    Pose2d pose() const;
    double position_sigma_m() const;
    double yaw_sigma_rad() const;

private:
    bool initialized_ = false;
    double state_[3] = {0.0, 0.0, 0.0};
    double covariance_[3][3] = {{0.0}};
};

}  // namespace omni

#endif
