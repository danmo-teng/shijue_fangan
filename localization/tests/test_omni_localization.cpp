#include "config.hpp"
#include "f407_protocol.hpp"
#include "fusion.hpp"

#include <algorithm>
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

struct TestQuaternion {
    double x;
    double y;
    double z;
    double w;
};

TestQuaternion multiply(const TestQuaternion &a, const TestQuaternion &b)
{
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

TestQuaternion axis_angle(double x, double y, double z, double degrees)
{
    const double half = omni::radians(degrees) * 0.5;
    const double sine = std::sin(half);
    return {x * sine, y * sine, z * sine, std::cos(half)};
}

omni::T265RawPose lens_up_pose(double yaw_deg, double timestamp_s = 0.0,
                               double gyro_yaw_rate_degps = 0.0)
{
    // Rx(+90) maps installed robot up (-Z Pose frame) to +Y world. A world-Y
    // rotation then represents chassis yaw without an Euler singularity.
    const TestQuaternion q = multiply(
        axis_angle(0.0, 1.0, 0.0, yaw_deg),
        axis_angle(1.0, 0.0, 0.0, 90.0));
    omni::T265RawPose raw;
    raw.rotation_xyzw[0] = q.x;
    raw.rotation_xyzw[1] = q.y;
    raw.rotation_xyzw[2] = q.z;
    raw.rotation_xyzw[3] = q.w;
    raw.timestamp_s = timestamp_s;
    raw.angular_velocity_radps[1] = omni::radians(gyro_yaw_rate_degps);
    raw.tracker_confidence = 3;
    raw.mapper_confidence = 3;
    return raw;
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
    check(parsed.size() == 1, "parser recovers after CRC error without forwarding duplicate");
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

    omni::StmStatusFrame expected_status;
    expected_status.sequence = 9;
    expected_status.flags = 0x09;
    expected_status.mode = 2;
    expected_status.camera_pitch_cdeg = 7350;
    expected_status.acknowledged_sequence = 8;
    expected_status.fault_code = 0;
    const auto status_bytes = omni::build_stm_status_frame(expected_status);
    std::vector<omni::StmStatusFrame> statuses;
    omni::F407FrameParser mixed_parser(
        [](const omni::EncoderFrame &) {},
        [&](const omni::StmStatusFrame &status) { statuses.push_back(status); });
    mixed_parser.feed(status_bytes.data(), status_bytes.size());
    mixed_parser.feed(status_bytes.data(), status_bytes.size());
    check(statuses.size() == 1, "TYPE 0x17 status parses on shared UART stream");
    check(statuses[0].camera_pitch_cdeg == 7350 && statuses[0].flags == 0x09,
          "STM32 camera/claw status round trip");
    check(mixed_parser.stats().status_frames == 1, "status duplicate does not refresh output");

    std::array<std::uint8_t, omni::kFrameSize> relay{};
    relay[0] = omni::kFrameHead1;
    relay[1] = omni::kFrameHead2;
    relay[2] = omni::kMissionCommandMessageType;
    relay[3] = 1;
    relay[4] = 3;
    const std::uint16_t relay_crc = omni::modbus_crc16(&relay[2], 10);
    relay[12] = static_cast<std::uint8_t>(relay_crc & 0xffu);
    relay[13] = static_cast<std::uint8_t>(relay_crc >> 8);
    relay[14] = omni::kFrameTail;
    check(omni::validate_relay_frame(relay.data(), relay.size()),
          "valid TYPE 0x18 application frame accepted by relay");
    const auto original_payload = relay;
    check(omni::refresh_mission_frame_sequence(relay, 9),
          "mission heartbeat refreshes sequence and CRC");
    check(relay[3] == 9 &&
          std::equal(relay.begin() + 4, relay.begin() + 12,
                     original_payload.begin() + 4),
          "mission heartbeat preserves command payload");
    check(omni::validate_relay_frame(relay.data(), relay.size()),
          "refreshed mission heartbeat remains valid");
    relay[2] = omni::kFusedPoseMessageType;
    check(!omni::validate_relay_frame(relay.data(), relay.size()),
          "relay rejects TYPE 0x16 generated internally by localization");
}

void test_kinematics()
{
    omni::LocalizationConfig config;
    config.encoder_sign[0] = config.encoder_sign[1] = config.encoder_sign[2] = 1;
    config.encoder_to_robot_yaw_deg = 0.0;
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
    forward.position[0] = 10;
    forward.position[1] = static_cast<std::uint16_t>(-10);
    check(integrator.update(forward, increment, reason), "forward update accepted");
    check(near(increment.forward_m, 20.0 * metres_per_count / std::sqrt(3.0), 1e-12),
          "F407 M1/M2 forward kinematics");
    check(near(increment.left_m, 0.0, 1e-12), "forward has no lateral displacement");

    integrator.reset();
    baseline.sequence = 10;
    integrator.update(baseline, increment, reason);
    omni::EncoderFrame left = baseline;
    left.sequence = 11;
    left.position[0] = 10;
    left.position[1] = 10;
    left.position[2] = static_cast<std::uint16_t>(-20);
    check(integrator.update(left, increment, reason), "left update accepted");
    check(near(increment.left_m, 20.0 * metres_per_count, 1e-12),
          "F407 M1/M2/M3 lateral kinematics");
    check(near(increment.forward_m, 0.0, 1e-12),
          "left has no forward displacement");

    omni::LocalizationConfig corrected_config = config;
    corrected_config.encoder_to_robot_yaw_deg = 90.0;
    omni::OmniEncoderIntegrator corrected_integrator(corrected_config);
    baseline.sequence = 20;
    corrected_integrator.update(baseline, increment, reason);
    omni::EncoderFrame physical_forward = baseline;
    physical_forward.sequence = 21;
    physical_forward.position[0] = 10;
    physical_forward.position[1] = static_cast<std::uint16_t>(-10);
    check(corrected_integrator.update(physical_forward, increment, reason),
          "corrected encoder forward sample accepted");
    check(near(increment.forward_m, 0.0, 1e-12) &&
          near(increment.left_m, 20.0 * metres_per_count / std::sqrt(3.0), 1e-12),
          "optional encoder correction rotates the F407 body vector");

    integrator.reset();
    baseline.sequence = 30;
    integrator.update(baseline, increment, reason);
    omni::EncoderFrame rotation = baseline;
    rotation.sequence = 31;
    rotation.position[0] = rotation.position[1] = rotation.position[2] = 10;
    check(integrator.update(rotation, increment, reason), "rotation update accepted");
    check(near(increment.forward_m, 0.0, 1e-12) &&
          near(increment.left_m, 0.0, 1e-12) &&
          increment.yaw_rad > 0.0,
          "common three-wheel rotation cancels from translation");

    omni::WheelIncrement sample;
    sample.forward_m = 0.40;
    sample.left_m = -0.20;
    sample.yaw_rad = 0.10;
    sample.forward_velocity_mps = 0.80;
    sample.left_velocity_mps = -0.40;
    const omni::WheelIncrement weighted =
        omni::apply_encoder_fusion_weight(sample, 0.25);
    check(near(weighted.forward_m, 0.10) && near(weighted.left_m, -0.05) &&
          near(weighted.forward_velocity_mps, 0.20) &&
          near(weighted.left_velocity_mps, -0.10) &&
          near(weighted.yaw_rad, sample.yaw_rad),
          "encoder fusion weight scales translation without weakening gyro yaw");

    check(!omni::navigation_wheel_primary_enabled(true, false, true, 1.0),
          "T265-only navigation never enables wheel-primary fusion");
    check(!omni::navigation_wheel_primary_enabled(true, true, true, 0.0),
          "zero encoder weight keeps navigation T265-primary");
    check(!omni::navigation_wheel_primary_enabled(true, true, false, 1.0),
          "stale accepted wheel data keeps navigation T265-primary");
    check(omni::navigation_wheel_primary_enabled(true, true, true, 0.25),
          "fresh enabled wheel data can enter weighted wheel-primary fusion");
    check(near(omni::weighted_t265_sigma_multiplier(12.0, 0.0), 1.0) &&
          near(omni::weighted_t265_sigma_multiplier(12.0, 0.25), 1.6875) &&
          near(omni::weighted_t265_sigma_multiplier(12.0, 1.0), 12.0),
          "encoder weight continuously controls navigation T265 weakening");
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
        omni::T265RawPose zone_raw = lens_up_pose(0.0);
        const omni::T265FieldPose zone_field = zone_projector.project(zone_raw);
        check(near(zone_field.pose.x_m, expected_x[zone]), "selected zone initial field X");
        check(near(zone_field.pose.y_m, expected_y[zone]), "selected zone initial field Y");
        check(near(omni::degrees(zone_field.pose.yaw_rad), expected_heading[zone]),
              "selected zone outward heading");
    }

    omni::LocalizationConfig lens_up_config = config;
    lens_up_config.start_zone = 2;
    check(near(lens_up_config.camera_offset_forward_m, -0.0296) &&
          near(lens_up_config.camera_offset_left_m, -0.0301),
          "default T265 lever-arm calibration uses measured stereo-centre offsets");
    check(!lens_up_config.navigation_distance_compensation_enabled,
          "raw navigation distance compensation is disabled by default");
    omni::T265FieldProjector still_projector(lens_up_config);
    const auto still_a = still_projector.project(lens_up_pose(0.0, 1.0));
    const auto still_b = still_projector.project(lens_up_pose(0.0, 1.01));
    check(near(still_a.relative_yaw_rad, 0.0) &&
          near(still_b.relative_yaw_rad, 0.0),
          "lens-up stationary quaternion has stable yaw");

    omni::T265FieldProjector gyro_projector(lens_up_config);
    gyro_projector.project(lens_up_pose(0.0, 1.0, 12.0));
    const auto gyro_sample = gyro_projector.project(lens_up_pose(1.2, 1.1, 12.0));
    check(gyro_sample.gyro_yaw_rate_valid &&
          near(omni::degrees(gyro_sample.gyro_yaw_rate_radps), 12.0, 1e-8) &&
          near(omni::degrees(gyro_sample.gyro_relative_yaw_rad), 1.2, 1e-8),
          "lens-up T265 gyro projects onto robot yaw axis");

    auto camera_rotation_pose = [&](double yaw_deg, double timestamp_s) {
        auto raw = lens_up_pose(yaw_deg, timestamp_s);
        const double yaw = omni::radians(yaw_deg);
        const double r0f = lens_up_config.camera_offset_forward_m;
        const double r0l = lens_up_config.camera_offset_left_m;
        const double current_forward = std::cos(yaw) * r0f -
                                        std::sin(yaw) * r0l;
        const double current_left = std::sin(yaw) * r0f +
                                    std::cos(yaw) * r0l;
        // With the calibrated lens-up pose axes, raw T265 X is initial
        // forward and raw T265 -Z is initial left.
        raw.translation_m[0] = current_forward - r0f;
        raw.translation_m[2] = -(current_left - r0l);
        return raw;
    };

    omni::T265FieldProjector lever_projector(lens_up_config);
    const auto lever_initial = lever_projector.project(camera_rotation_pose(0.0, 1.0));
    const double start_cs = std::cos(omni::radians(45.0));
    const double start_ss = std::sin(omni::radians(45.0));
    check(near(lever_initial.tracking_origin_pose.x_m,
               1.20 + start_cs * lens_up_config.camera_offset_forward_m -
                   start_ss * lens_up_config.camera_offset_left_m) &&
          near(lever_initial.tracking_origin_pose.y_m,
               1.20 + start_ss * lens_up_config.camera_offset_forward_m +
                   start_cs * lens_up_config.camera_offset_left_m),
          "raw tracking-origin pose includes the physical initial lever arm");
    const auto lever_90 = lever_projector.project(camera_rotation_pose(90.0, 2.0));
    check(near(lever_90.tracking_origin_delta_forward_m, 0.0597, 1e-8) &&
          near(lever_90.tracking_origin_delta_left_m, 0.0005, 1e-8),
          "measured T265 lever arm produces expected raw tracking-origin arc");
    check(std::hypot(lever_90.robot_center_delta_forward_m,
                     lever_90.robot_center_delta_left_m) < 1e-8,
          "90-degree lever-arm correction keeps robot centre stationary");

    const double combined_yaw = omni::radians(45.0);
    const double expected_forward = 0.50;
    const double expected_left = 0.15;
    auto combined_motion = lens_up_pose(45.0, 2.5);
    const double combined_rf = std::cos(combined_yaw) *
                                   lens_up_config.camera_offset_forward_m -
                               std::sin(combined_yaw) *
                                   lens_up_config.camera_offset_left_m;
    const double combined_rl = std::sin(combined_yaw) *
                                   lens_up_config.camera_offset_forward_m +
                               std::cos(combined_yaw) *
                                   lens_up_config.camera_offset_left_m;
    combined_motion.translation_m[0] = expected_forward +
                                      combined_rf - lens_up_config.camera_offset_forward_m;
    combined_motion.translation_m[2] = -(expected_left + combined_rl -
                                         lens_up_config.camera_offset_left_m);
    omni::T265FieldProjector combined_projector(lens_up_config);
    combined_projector.project(lens_up_pose(0.0, 1.5));
    const auto combined_result = combined_projector.project(combined_motion);
    check(near(combined_result.robot_center_delta_forward_m, expected_forward, 1e-8) &&
          near(combined_result.robot_center_delta_left_m, expected_left, 1e-8),
          "simultaneous translation and rotation removes only the lever-arm arc");

    omni::T265FieldProjector full_turn_lever_projector(lens_up_config);
    full_turn_lever_projector.project(camera_rotation_pose(0.0, 1.0));
    omni::T265FieldPose full_turn_lever;
    for (int angle = 90; angle <= 360; angle += 90) {
        full_turn_lever = full_turn_lever_projector.project(
            camera_rotation_pose(static_cast<double>(angle),
                                 1.0 + angle * 0.01));
        check(std::hypot(full_turn_lever.robot_center_delta_forward_m,
                         full_turn_lever.robot_center_delta_left_m) < 1e-7,
              "lever-arm correction keeps robot centre stationary through rotation");
    }

    const double omega = omni::radians(90.0);
    auto stationary_rotation = lens_up_pose(0.0, 1.0, 90.0);
    stationary_rotation.velocity_mps[0] = -omega * lens_up_config.camera_offset_left_m;
    stationary_rotation.velocity_mps[2] = -omega * lens_up_config.camera_offset_forward_m;
    omni::T265FieldProjector velocity_lever_projector(lens_up_config);
    const auto stationary_rotation_result =
        velocity_lever_projector.project(stationary_rotation);
    check(std::fabs(stationary_rotation_result.body_forward_velocity_mps) < 1e-8 &&
          std::fabs(stationary_rotation_result.body_left_velocity_mps) < 1e-8,
          "velocity lever-arm correction removes stationary camera rotation velocity");

    omni::T265FieldProjector ccw_projector(lens_up_config);
    ccw_projector.project(lens_up_pose(0.0, 1.0));
    const auto ccw = ccw_projector.project(lens_up_pose(90.0, 2.0));
    check(near(omni::degrees(ccw.relative_yaw_rad), 90.0, 1e-8),
          "lens-up chassis CCW 90 gives positive relative yaw");

    omni::T265FieldProjector cw_projector(lens_up_config);
    cw_projector.project(lens_up_pose(0.0, 1.0));
    const auto cw = cw_projector.project(lens_up_pose(-90.0, 2.0));
    check(near(omni::degrees(cw.relative_yaw_rad), -90.0, 1e-8),
          "lens-up chassis CW 90 gives negative relative yaw");

    omni::T265FieldProjector forward_projector(lens_up_config);
    auto forward_origin = lens_up_pose(0.0, 1.0);
    forward_projector.project(forward_origin);
    auto forward_motion = lens_up_pose(0.0, 2.0);
    forward_motion.translation_m[0] = 1.0;
    forward_motion.velocity_mps[0] = 1.0;
    const auto forward_result = forward_projector.project(forward_motion);
    const double diagonal = std::sqrt(0.5);
    check(near(forward_result.pose.x_m, 1.20 + diagonal, 1e-8) &&
          near(forward_result.pose.y_m, 1.20 + diagonal, 1e-8),
          "lens-up forward one metre follows selected start heading");
    check(near(forward_result.body_forward_velocity_mps, 1.0, 1e-8) &&
          near(forward_result.body_left_velocity_mps, 0.0, 1e-8),
          "world velocity projects onto current chassis forward axis");

    omni::LocalizationConfig scaled_t265_config = lens_up_config;
    scaled_t265_config.t265_translation_scale_enabled = true;
    scaled_t265_config.t265_translation_scale = 1.04;
    omni::T265FieldProjector scaled_t265_projector(scaled_t265_config);
    scaled_t265_projector.project(lens_up_pose(0.0, 1.0));
    auto scaled_forward_motion = lens_up_pose(0.0, 2.0);
    scaled_forward_motion.translation_m[0] = 1.0;
    scaled_forward_motion.velocity_mps[0] = 1.0;
    const auto scaled_forward_result =
        scaled_t265_projector.project(scaled_forward_motion);
    check(scaled_forward_result.translation_scale_enabled &&
          near(scaled_forward_result.translation_scale, 1.04, 1e-12) &&
          near(scaled_forward_result.unscaled_robot_center_delta_forward_m, 1.0, 1e-8) &&
          near(scaled_forward_result.robot_center_delta_forward_m, 1.04, 1e-8) &&
          near(scaled_forward_result.body_forward_velocity_mps, 1.04, 1e-8),
          "optional T265 translation scale affects centre translation and velocity");

    auto scaled_rotation_pose = camera_rotation_pose(90.0, 3.0);
    omni::T265FieldProjector scaled_rotation_projector(scaled_t265_config);
    scaled_rotation_projector.project(camera_rotation_pose(0.0, 2.0));
    const auto scaled_rotation = scaled_rotation_projector.project(scaled_rotation_pose);
    check(std::hypot(scaled_rotation.robot_center_delta_forward_m,
                     scaled_rotation.robot_center_delta_left_m) < 1e-8,
          "T265 translation scale preserves zero robot-centre motion during rotation");

    omni::T265FieldProjector left_projector(lens_up_config);
    left_projector.project(lens_up_pose(0.0, 1.0));
    auto left_motion = lens_up_pose(0.0, 2.0);
    left_motion.translation_m[2] = -1.0;
    left_motion.velocity_mps[2] = -1.0;
    const auto left_result = left_projector.project(left_motion);
    check(near(left_result.pose.x_m, 1.20 - diagonal, 1e-8) &&
          near(left_result.pose.y_m, 1.20 + diagonal, 1e-8),
          "lens-up left one metre follows selected field-left direction");
    check(near(left_result.body_forward_velocity_mps, 0.0, 1e-8) &&
          near(left_result.body_left_velocity_mps, 1.0, 1e-8),
          "lens-up left translation projects to left one metre");

    omni::T265FieldProjector continuity_projector(lens_up_config);
    continuity_projector.project(lens_up_pose(170.0, 1.0));
    const auto before_wrap = continuity_projector.project(lens_up_pose(179.9, 1.1));
    auto perturbed = lens_up_pose(-179.9, 1.2);
    const TestQuaternion noise = axis_angle(1.0, 0.0, 0.0, 0.05);
    const TestQuaternion base{perturbed.rotation_xyzw[0], perturbed.rotation_xyzw[1],
                              perturbed.rotation_xyzw[2], perturbed.rotation_xyzw[3]};
    const TestQuaternion noisy = multiply(base, noise);
    perturbed.rotation_xyzw[0] = noisy.x;
    perturbed.rotation_xyzw[1] = noisy.y;
    perturbed.rotation_xyzw[2] = noisy.z;
    perturbed.rotation_xyzw[3] = noisy.w;
    const auto after_wrap = continuity_projector.project(perturbed);
    check(std::fabs(omni::degrees(after_wrap.relative_yaw_rad -
                                  before_wrap.relative_yaw_rad)) < 1.0,
          "small quaternion perturbation remains continuous across +/-180");

    omni::T265FieldProjector full_turn_projector(lens_up_config);
    double previous_relative = -1e-9;
    for (int angle = 0; angle <= 360; angle += 10) {
        const auto sample = full_turn_projector.project(
            lens_up_pose(static_cast<double>(angle), 1.0 + angle * 0.01));
        check(sample.relative_yaw_rad + 1e-9 >= previous_relative,
              "lens-up full-turn relative yaw is monotonic");
        previous_relative = sample.relative_yaw_rad;
    }
    check(near(omni::degrees(previous_relative), 360.0, 1e-7),
          "lens-up full turn accumulates approximately 360 degrees");

    config.start_zone = 4;
    omni::T265FieldProjector projector(config);
    omni::T265RawPose raw = lens_up_pose(0.0);
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

    omni::PlanarOdometry odometry;
    odometry.initialize(field.pose);
    odometry.integrate(wheel);
    const omni::Pose2d odometry_pose = odometry.pose();
    check(near(odometry_pose.x_m, predicted.x_m) &&
          near(odometry_pose.y_m, predicted.y_m) &&
          near(odometry_pose.yaw_rad, predicted.yaw_rad),
          "wheel-only diagnostic odometry uses EKF motion convention");
    check(near(odometry.travel_m(), 0.1),
          "wheel-only diagnostic odometry accumulates planar travel");
    check(filter.correct_t265(field, config), "T265 correction accepted");
    const omni::Pose2d corrected = filter.pose();
    check(std::hypot(corrected.x_m - field.pose.x_m,
                     corrected.y_m - field.pose.y_m) <
          std::hypot(predicted.x_m - field.pose.x_m,
                     predicted.y_m - field.pose.y_m),
          "T265-primary correction reduces wheel prediction error");

    omni::PlanarEkf navigation_filter;
    navigation_filter.initialize(field.pose);
    navigation_filter.predict(wheel, config);
    const omni::Pose2d navigation_predicted = navigation_filter.pose();
    check(navigation_filter.correct_t265(field, config, nullptr, 8.0, false),
          "navigation yaw-only T265 update accepted");
    const omni::Pose2d yaw_only = navigation_filter.pose();
    check(std::hypot(yaw_only.x_m - navigation_predicted.x_m,
                     yaw_only.y_m - navigation_predicted.y_m) < 1e-6,
          "yaw-only T265 update preserves encoder translation progress");

    omni::PlanarEkf weak_position_filter;
    weak_position_filter.initialize(field.pose);
    weak_position_filter.predict(wheel, config);
    check(weak_position_filter.correct_t265(field, config, nullptr, 8.0, true),
          "weakened navigation position correction accepted");
    const omni::Pose2d weak_corrected = weak_position_filter.pose();
    check(std::hypot(weak_corrected.x_m - navigation_predicted.x_m,
                     weak_corrected.y_m - navigation_predicted.y_m) <
          std::hypot(corrected.x_m - navigation_predicted.x_m,
                     corrected.y_m - navigation_predicted.y_m),
          "navigation correction keeps more encoder progress than normal T265 correction");
}

void test_t265_yaw_time_sync()
{
    omni::LocalizationConfig config;
    config.start_zone = 2;
    config.t265_gyro_pose_resync_period_s = 0.25;
    omni::T265FieldProjector projector(config);

    const auto first = projector.project(lens_up_pose(0.0, 1.0, 90.0));
    const auto second = projector.project(lens_up_pose(9.0, 1.1, 90.0));
    const auto third = projector.project(lens_up_pose(27.0, 1.3, 90.0));
    check(near(omni::degrees(second.gyro_relative_yaw_rad), 9.0, 1e-8) &&
          near(omni::degrees(third.gyro_relative_yaw_rad), 27.0, 1e-8),
          "T265 gyro yaw integrates with the T265 timestamp intervals");

    omni::T265YawSynchronizer synchronizer;
    check(synchronizer.update(10.0, first) &&
          synchronizer.update(10.1, second) &&
          synchronizer.update(10.2, third),
          "T265 yaw timeline accepts monotonic host samples");
    double yaw_at_encoder = 0.0;
    check(synchronizer.yaw_at(10.05, yaw_at_encoder) &&
          near(omni::degrees(yaw_at_encoder - first.pose.yaw_rad), 4.5, 1e-8),
          "encoder timestamp uses interpolated T265 yaw instead of latest yaw");
    check(synchronizer.yaw_at(10.15, yaw_at_encoder) &&
          near(omni::degrees(yaw_at_encoder - first.pose.yaw_rad), 18.0, 1e-8),
          "queued encoder timestamp remains aligned between T265 samples");

    omni::LocalizationConfig resync_config = config;
    resync_config.t265_gyro_pose_resync_period_s = 0.20;
    omni::T265FieldProjector resync_projector(resync_config);
    resync_projector.project(lens_up_pose(0.0, 1.0, 0.0));
    const auto drifted = resync_projector.project(lens_up_pose(20.0, 1.1, 0.0));
    const auto resynchronized = resync_projector.project(lens_up_pose(30.0, 1.3, 0.0));
    check(std::fabs(omni::degrees(drifted.gyro_pose_sync_error_rad)) > 19.0,
          "gyro/pose yaw drift is visible before the resynchronization period");
    check(std::fabs(omni::degrees(resynchronized.gyro_pose_sync_error_rad)) < 1e-8 &&
          near(omni::degrees(resynchronized.gyro_relative_yaw_rad), 30.0, 1e-8),
          "T265 attitude yaw periodically re-synchronizes the gyro timeline");

    omni::T265FieldProjector full_turn_resync_projector(resync_config);
    omni::T265FieldPose full_turn_resync;
    for (int angle = 0; angle <= 360; angle += 10) {
        full_turn_resync = full_turn_resync_projector.project(
            lens_up_pose(static_cast<double>(angle),
                         2.0 + angle * 0.01, 0.0));
    }
    check(near(omni::degrees(full_turn_resync.gyro_relative_yaw_rad), 360.0, 1e-8),
          "unwrapped pose resynchronization preserves a full turn count");

    omni::PlanarOdometry odometry;
    odometry.initialize({0.0, 0.0, 0.0});
    omni::WheelIncrement increment;
    increment.forward_m = 1.0;
    increment.field_yaw_rad = omni::radians(90.0);
    increment.field_yaw_valid = true;
    odometry.integrate(increment);
    check(std::fabs(odometry.pose().x_m) < 1e-8 &&
          near(odometry.pose().y_m, 1.0, 1e-8),
          "wheel translation uses the T265 heading at the increment midpoint");

    omni::PlanarEkf filter;
    omni::LocalizationConfig filter_config;
    filter.initialize({0.0, 0.0, 0.0});
    filter.predict(increment, filter_config);
    check(std::fabs(filter.pose().x_m) < 1e-8 &&
          near(filter.pose().y_m, 1.0, 1e-8),
          "EKF wheel prediction uses the timestamp-aligned field heading");
}

}  // namespace

int main()
{
    test_protocol();
    test_kinematics();
    test_projection_gate_and_filter();
    test_t265_yaw_time_sync();
    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "all protocol, kinematics, gate, and EKF tests passed\n";
    return EXIT_SUCCESS;
}
