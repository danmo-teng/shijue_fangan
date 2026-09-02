#ifndef T265_OMNI_FUSION_HPP
#define T265_OMNI_FUSION_HPP

#include "config.hpp"
#include "f407_protocol.hpp"

#include <cstdint>
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
    double native_x_m = 0.0;
    double native_z_m = 0.0;
    double native_vx_mps = 0.0;
    double native_vz_mps = 0.0;
    double heading_y_rad = 0.0;
    double angular_velocity_y_radps = 0.0;
    std::uint8_t tracker_confidence = 0;
    std::uint8_t mapper_confidence = 0;
};

struct T265FieldPose {
    Pose2d pose;
    double body_forward_velocity_mps = 0.0;
    double body_left_velocity_mps = 0.0;
    double travel_from_origin_m = 0.0;
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
    double raw_forward_origin_m_ = 0.0;
    double raw_left_origin_m_ = 0.0;
    double raw_heading_origin_rad_ = 0.0;
    Pose2d start_pose_{};
};

struct WheelIncrement {
    double forward_m = 0.0;
    double left_m = 0.0;
    double yaw_rad = 0.0;
    double dt_s = 0.0;
    double forward_velocity_mps = 0.0;
    double left_velocity_mps = 0.0;
    std::uint8_t sequence_step = 0;
};

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
                      double *innovation_m = nullptr);
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
