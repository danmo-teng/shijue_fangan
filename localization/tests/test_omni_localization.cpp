#include "config.hpp"
#include "f407_protocol.hpp"
#include "fusion.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char *message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool near(double a, double b, double tolerance = 1e-9)
{
    return std::fabs(a - b) <= tolerance;
}

void test_protocol()
{
    omni::EncoderFrame expected;
    expected.sequence = 0xfe;
    expected.position[0] = 0x1234;
    expected.position[1] = 0xabcd;
    expected.position[2] = 0xfff0;
    expected.sample_period_ms = 10;
    expected.status = 0x07;
    const auto bytes = omni::build_encoder_frame(expected);

    std::vector<omni::EncoderFrame> parsed;
    omni::F407FrameParser parser([&](const omni::EncoderFrame &frame) {
        parsed.push_back(frame);
    });
    const std::uint8_t noise[] = {0, 1, 0xa3, 0, 0xa3};
    parser.feed(noise, sizeof(noise));
    parser.feed(bytes.data(), 4);
    parser.feed(bytes.data() + 4, bytes.size() - 4);
    check(parsed.size() == 1, "chunked frame parses after noise");
    check(parsed[0].position[1] == expected.position[1], "big-endian position round trip");

    auto corrupt = bytes;
    corrupt[8] ^= 0x40;
    parser.feed(corrupt.data(), corrupt.size());
    parser.feed(bytes.data(), bytes.size());
    check(parsed.size() == 2, "parser recovers after CRC error");
    check(parser.stats().crc_errors == 1, "CRC error counted");
    check(parser.stats().duplicates == 1, "duplicate sequence counted");

    omni::FusedPoseFrame pose;
    pose.sequence = 7;
    pose.x_mm = 1350;
    pose.y_mm = -1350;
    pose.heading_cdeg = 31500;
    pose.status = omni::kPoseValid | omni::kPoseT265Good | omni::kPoseInsideField;
    pose.confidence_and_sigma = 0x22;
    const auto pose_bytes = omni::build_fused_pose_frame(pose);
    omni::FusedPoseFrame decoded;
    check(omni::decode_fused_pose_frame(pose_bytes.data(), pose_bytes.size(), decoded),
          "fused pose frame decodes");
    check(decoded.x_mm == 1350 && decoded.y_mm == -1350,
          "signed field millimetres round trip");
    check(decoded.heading_cdeg == 31500, "centidegree heading round trip");
    auto bad_pose = pose_bytes;
    bad_pose[6] ^= 1;
    check(!omni::decode_fused_pose_frame(bad_pose.data(), bad_pose.size(), decoded),
          "fused pose CRC corruption rejected");
}

void test_kinematics()
{
    omni::LocalizationConfig config;
    config.encoder_sign[0] = config.encoder_sign[1] = config.encoder_sign[2] = 1;
    config.wheel_center_radius_m = 0.1;
    omni::OmniEncoderIntegrator integrator(config);
    omni::EncoderFrame baseline;
    baseline.status = 0x07;
    baseline.sample_period_ms = 10;
    std::string reason;
    omni::WheelIncrement increment;
    check(!integrator.update(baseline, increment, reason), "first encoder frame is baseline");

    const double metres_per_count = omni::kPi * config.wheel_diameter_m /
                                    config.counts_per_wheel_revolution;
    omni::EncoderFrame forward = baseline;
    forward.sequence = 1;
    forward.position[0] = static_cast<std::uint16_t>(-10);
    forward.position[2] = 10;
    check(integrator.update(forward, increment, reason), "forward update accepted");
    check(near(increment.forward_m, 20.0 * metres_per_count / std::sqrt(3.0), 1e-12),
          "three-wheel forward kinematics");
    check(near(increment.left_m, 0.0, 1e-12), "forward has no lateral displacement");

    integrator.reset();
    baseline.sequence = 10;
    integrator.update(baseline, increment, reason);
    omni::EncoderFrame left = baseline;
    left.sequence = 11;
    left.position[0] = 10;
    left.position[1] = static_cast<std::uint16_t>(-20);
    left.position[2] = 10;
    check(integrator.update(left, increment, reason), "left update accepted");
    check(near(increment.left_m, 20.0 * metres_per_count, 1e-12),
          "three-wheel lateral kinematics");
}

void test_projection_gate_and_filter()
{
    omni::LocalizationConfig config;
    config.start_center_m = 1.20;
    const double expected_x[] = {0.0, -1.20, 1.20, -1.20, 1.20};
    const double expected_y[] = {0.0, 1.20, 1.20, -1.20, -1.20};
    const double expected_heading[] = {0.0, 135.0, 45.0, -135.0, -45.0};
    for (int zone = 1; zone <= 4; ++zone) {
        config.start_zone = zone;
        omni::T265FieldProjector zone_projector(config);
        omni::T265RawPose zone_raw;
        zone_raw.tracker_confidence = 3;
        const omni::T265FieldPose zone_field = zone_projector.project(zone_raw);
        check(near(zone_field.pose.x_m, expected_x[zone]), "selected zone initial field X");
        check(near(zone_field.pose.y_m, expected_y[zone]), "selected zone initial field Y");
        check(near(omni::degrees(zone_field.pose.yaw_rad), expected_heading[zone]),
              "selected zone outward heading");
    }

    config.start_zone = 4;
    omni::T265FieldProjector projector(config);
    omni::T265RawPose raw;
    raw.tracker_confidence = 3;
    omni::T265FieldPose field = projector.project(raw);
    check(near(field.pose.x_m, 1.20), "zone 4 initial field X");
    check(near(field.pose.y_m, -1.20), "zone 4 initial field Y");
    check(near(omni::degrees(field.pose.yaw_rad), -45.0), "zone 4 heading wraps to -45 degrees");

    omni::WheelIncrement wheel;
    wheel.forward_velocity_mps = 0.2;
    check(omni::evaluate_wheel_gate(config, field, wheel) ==
              omni::WheelGateReason::StartupObstacle,
          "wheel odometry disabled at startup bumps");
    field.travel_from_origin_m = 1.0;
    field.pose.x_m = 0.2;
    field.pose.y_m = 0.1;
    field.body_forward_velocity_mps = 0.2;
    check(omni::evaluate_wheel_gate(config, field, wheel) ==
              omni::WheelGateReason::Accepted,
          "wheel odometry enabled in central flat area");

    omni::PlanarEkf filter;
    filter.initialize(field.pose);
    wheel.forward_m = 0.1;
    wheel.dt_s = 0.5;
    filter.predict(wheel, config);
    const omni::Pose2d predicted = filter.pose();
    check(std::hypot(predicted.x_m - field.pose.x_m,
                     predicted.y_m - field.pose.y_m) > 0.09,
          "wheel prediction advances pose");
    check(filter.correct_t265(field, config), "T265 correction accepted");
    const omni::Pose2d corrected = filter.pose();
    check(std::hypot(corrected.x_m - field.pose.x_m,
                     corrected.y_m - field.pose.y_m) <
          std::hypot(predicted.x_m - field.pose.x_m,
                     predicted.y_m - field.pose.y_m),
          "T265-primary correction reduces wheel prediction error");
}

}  // namespace

int main()
{
    test_protocol();
    test_kinematics();
    test_projection_gate_and_filter();
    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "all protocol, kinematics, gate, and EKF tests passed\n";
    return EXIT_SUCCESS;
}
