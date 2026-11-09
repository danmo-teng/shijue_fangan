#include "config.hpp"
#include "f407_protocol.hpp"
#include "f405_telemetry.hpp"
#include "fusion.hpp"
#include "debug_wheel_input.hpp"
#include "dynamic_pose_relay.hpp"
#include "async_csv_log.hpp"
#include "async_latest_file.hpp"
#include "serial_port.hpp"
#include "test_uart_relay.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
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
    check(omni::t265_pose_status(true, 2, true) == (omni::kPoseValid | omni::kPoseT265Good),
          "fresh accepted confidence-2 T265 pose can establish a recovery reference");
    check(omni::t265_pose_status(true, 3, false) == omni::kPoseT265UpdateRejected &&
          omni::t265_pose_status(false, 3, true) == omni::kPoseT265UpdateRejected &&
          omni::t265_pose_status(true, 0, true) == omni::kPoseT265UpdateRejected,
          "stale, rejected or lost T265 samples cannot advertise VALID/GOOD");
    omni::EncoderFrame expected;
    expected.sequence = 0xfe;
    expected.position[0] = 0x1234;
    expected.position[1] = 0xabcd;
    expected.position[2] = 0xfff0;
    expected.sample_period_ms = 10;
    expected.status = 0x07;
    const auto bytes = omni::build_encoder_frame(expected);

    // A valid legacy-looking payload inside an extended packet is not a
    // second UART frame. Fragmentation must not change task/status parsing.
    std::vector<std::uint8_t> extended{0xd4, 0x4d, 1, 7, 0, 15, 0, 1};
    extended.insert(extended.end(), bytes.begin(), bytes.end());
    auto crc = omni::modbus_crc16(extended.data()+2, extended.size()-2);
    extended.push_back(crc & 255); extended.push_back(crc >> 8); extended.push_back(0xe4);
    int legacy_count = 0, extended_count = 0;
    omni::F405MixedFrames mixed(
        [&](const std::uint8_t *p, std::size_t n) {
            check(n == bytes.size() && std::equal(p, p+n, bytes.begin()), "legacy packet unchanged");
            ++legacy_count;
        },
        [&](const std::uint8_t *p, std::size_t n) {
            check(n == extended.size() && std::equal(p, p+n, extended.begin()), "extended packet unchanged");
            ++extended_count;
        });
    for (auto byte : extended) mixed.feed(&byte, 1);
    check(legacy_count == 0 && extended_count == 1, "consume extended container atomically");
    mixed.feed(bytes.data(), bytes.size());
    check(legacy_count == 1, "legacy traffic continues after extended packet");

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
    left.position[0] = static_cast<std::uint16_t>(-10);
    left.position[1] = static_cast<std::uint16_t>(-10);
    left.position[2] = 20;
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

void test_new_mount()
{
    char filename[] = "/tmp/t265-axis-config-XXXXXX";
    const int fd = mkstemp(filename);
    check(fd >= 0, "create isolated axis-config fixture");
    if (fd < 0) return;
    close(fd);
    auto parse = [&](const char *forward, const char *up) {
        std::ofstream file(filename);
        file << "camera_robot_forward_axis = " << forward << '\n'
             << "camera_robot_up_axis = " << up << '\n';
        file.close();
        return omni::load_config(filename);
    };
    const auto old = parse("+x", "-y");
    check(near(old.camera_robot_forward_axis[0], 1), "legacy signed axes still parse");
    auto config = parse("0.8660254,0,0.5", "0,-1,0");
    check(near(config.camera_robot_forward_axis[0], std::sqrt(3.0)/2, 1e-8),
          "rounded vector direction cosines normalized");
    for (const char *bad : {"0,0,0", "nan,0,1", "inf,0,1", "1,0", "1,0,0,", "1,0,0,1"}) {
        bool rejected = false;
        try { parse(bad, "-y"); } catch (const std::exception &) { rejected = true; }
        check(rejected, "invalid axis vectors rejected");
    }
    bool rejected = false;
    try { parse("1,1,0", "-y"); } catch (const std::exception &) { rejected = true; }
    check(rejected, "nonorthogonal mounting axes rejected");
    unlink(filename);
    config.start_zone = 4;
    config.start_center_m = 0;
    config.start_heading_offset_deg = 45;
    config.camera_offset_forward_m = -0.0346105;
    config.camera_offset_left_m = -0.0599471445;
    const double *f = config.camera_robot_forward_axis;
    const double *u = config.camera_robot_up_axis;
    const double l[3] = {u[1]*f[2]-u[2]*f[1], u[2]*f[0]-u[0]*f[2], u[0]*f[1]-u[1]*f[0]};
    const TestQuaternion mount = multiply(axis_angle(1,0,0,180), axis_angle(0,1,0,30));
    auto rotate = [](const TestQuaternion &q, const double v[3], double out[3]) {
        const auto r = multiply(multiply(q, {v[0],v[1],v[2],0}), {-q.x,-q.y,-q.z,q.w});
        out[0]=r.x; out[1]=r.y; out[2]=r.z;
    };
    const double lever[3] = {f[0]*config.camera_offset_forward_m+l[0]*config.camera_offset_left_m,
        f[1]*config.camera_offset_forward_m+l[1]*config.camera_offset_left_m,
        f[2]*config.camera_offset_forward_m+l[2]*config.camera_offset_left_m};
    double initial_lever[3]; rotate(mount, lever, initial_lever);
    auto sample = [&](TestQuaternion q, double df, double dl, double t) {
        omni::T265RawPose raw;
        raw.rotation_xyzw[0]=q.x; raw.rotation_xyzw[1]=q.y;
        raw.rotation_xyzw[2]=q.z; raw.rotation_xyzw[3]=q.w;
        double r[3]; rotate(q, lever, r);
        for (int i=0;i<3;++i) raw.translation_m[i]=r[i]-initial_lever[i];
        raw.translation_m[0]+=df; raw.translation_m[2]-=dl;
        raw.timestamp_s=t; raw.tracker_confidence=3;
        return raw;
    };
    // Independent 1m cardinal motions: mounting yaw is not a map-heading offset.
    for (auto d : {std::pair<double,double>{1,0}, {-1,0}, {0,1}, {0,-1}}) {
        omni::T265FieldProjector p(config);
        auto origin=p.project(sample(mount,0,0,0));
        check(near(origin.up_world[1],1,1e-8), "vertical -Y mount maps to SDK gravity-up +Y");
        auto moved=p.project(sample(mount,d.first,d.second,1));
        check(near(moved.pose.x_m,d.first,1e-8) && near(moved.pose.y_m,d.second,1e-8),
              "new mount cardinal displacement uses rotation extrinsic");
        auto returned=p.project(sample(mount,0,0,2));
        check(std::hypot(returned.pose.x_m,returned.pose.y_m)<1e-8, "cardinal return closes at centre");
    }
    for (int sign : {-1,1}) {
        omni::T265FieldProjector p(config);
        p.project(sample(mount,0,0,0));
        for (int angle=10;angle<=360;angle+=10) {
            const auto q=multiply(axis_angle(0,1,0,sign*angle),mount);
            auto r=p.project(sample(q,0,0,angle*.01));
            check(std::hypot(r.pose.x_m,r.pose.y_m)<1e-8, "new lever arm removes rotation arc once");
            check(near(omni::degrees(r.relative_yaw_rad),sign*angle,1e-6),
                  "both directions retain unwrapped 90/180/360 degree turns");
        }
    }
    // Roll/pitch plus changing 3D angular rate must not create centre motion.
    omni::T265FieldProjector p(config);
    p.project(sample(mount,0,0,0));
    const auto q=multiply(multiply(axis_angle(0,0,1,25),axis_angle(0,1,0,55)),mount);
    auto tilted=sample(q,.4,-.2,1);
    double r[3],fw[3],lw[3]; rotate(q,lever,r); rotate(q,f,fw); rotate(q,l,lw);
    const double w[3]={.7,-.4,.3};
    const double spin[3]={w[1]*r[2]-w[2]*r[1],w[2]*r[0]-w[0]*r[2],w[0]*r[1]-w[1]*r[0]};
    for (int i=0;i<3;++i) {
        tilted.angular_velocity_radps[i]=w[i];
        tilted.velocity_mps[i]=.3*fw[i]-.2*lw[i]+spin[i];
    }
    auto out=p.project(tilted);
    check(near(out.pose.x_m,.4,1e-8) && near(out.pose.y_m,-.2,1e-8), "full attitude removes tilted lever arm");
    check(near(out.body_forward_velocity_mps,.3,1e-8) && near(out.body_left_velocity_mps,-.2,1e-8),
          "instantaneous omega cross lever velocity compensation uses full rotation");
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
        zone_raw.rotation_xyzw[0] = 1.0; // Rx(180): installed -Y points up.
        zone_raw.rotation_xyzw[3] = 0.0;
        zone_raw.tracker_confidence = 3;
        const omni::T265FieldPose zone_field = zone_projector.project(zone_raw);
        check(near(zone_field.pose.x_m, expected_x[zone]), "selected zone initial field X");
        check(near(zone_field.pose.y_m, expected_y[zone]), "selected zone initial field Y");
        check(near(omni::degrees(zone_field.pose.yaw_rad), expected_heading[zone]),
              "selected zone outward heading");
        zone_raw.translation_m[0] = 1.0;
        const auto forward = zone_projector.project(zone_raw);
        const double heading = omni::radians(expected_heading[zone]);
        check(near(forward.pose.x_m, expected_x[zone] + std::cos(heading)) &&
              near(forward.pose.y_m, expected_y[zone] + std::sin(heading)),
              "side-mounted forward motion follows each competition start heading");
        zone_raw.translation_m[0] = 0.0;
        zone_raw.translation_m[2] = -1.0;
        const auto left = zone_projector.project(zone_raw);
        check(near(left.pose.x_m, expected_x[zone] - std::sin(heading)) &&
              near(left.pose.y_m, expected_y[zone] + std::cos(heading)),
              "side-mounted left motion follows each competition start heading");
        const auto turned_q = multiply(axis_angle(0.0, 1.0, 0.0, 90.0),
                                       axis_angle(1.0, 0.0, 0.0, 180.0));
        zone_raw.rotation_xyzw[0] = turned_q.x;
        zone_raw.rotation_xyzw[1] = turned_q.y;
        zone_raw.rotation_xyzw[2] = turned_q.z;
        zone_raw.rotation_xyzw[3] = turned_q.w;
        const auto turned = zone_projector.project(zone_raw);
        check(near(omni::wrap_angle(turned.pose.yaw_rad - heading - omni::radians(90.0)), 0.0),
              "side-mounted counterclockwise turn increases field heading by 90 degrees");
    }

    // Keep the historical lens-up geometry cases explicit after changing defaults.
    config.camera_robot_up_axis[1] = 0.0;
    config.camera_robot_up_axis[2] = -1.0;
    omni::LocalizationConfig centre_config = config;
    centre_config.start_center_m = 0.0;
    centre_config.start_heading_offset_deg = 45.0;
    centre_config.camera_offset_forward_m = 0.0;
    centre_config.camera_offset_left_m = 0.0;
    omni::validate_config(centre_config);
    for (int zone = 1; zone <= 4; ++zone) {
        centre_config.start_zone = zone;
        omni::T265FieldProjector centre_projector(centre_config);
        const auto centre = centre_projector.project(lens_up_pose(0.0));
        check(near(centre.pose.x_m, 0.0) && near(centre.pose.y_m, 0.0),
              "debug centre-start initializes the actual projected pose at origin");
        check(near(omni::wrap_angle(centre.pose.yaw_rad -
                                  omni::radians(expected_heading[zone] + 45.0)), 0.0),
              "debug centre-start rotates initial heading 45 degrees counterclockwise");
    }

    omni::LocalizationConfig lens_up_config = config;
    lens_up_config.start_zone = 2;
    check(near(config.camera_offset_forward_m, -0.020) &&
          near(config.camera_offset_left_m, -0.070),
          "default T265 lever arm matches the validated side-mounted setup");
    lens_up_config.camera_offset_forward_m = -0.0296;
    lens_up_config.camera_offset_left_m = -0.0301;
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
    config.camera_robot_up_axis[1] = 0.0;
    config.camera_robot_up_axis[2] = -1.0;
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

void test_debug_wheel_input()
{
    using Input = omni::DebugWheelInput;
    using Json = Input::Json;
    const Json sdk{{"velocimeters", {{{"scale_and_alignment", {1,0,0,0,1,0,0,0,1}},
        {"noise_variance", .01}, {"extrinsics", {{"T", {0,0,0}}, {"W", {0,0,0}}}}}}}};
    Input::validate_config(sdk);
    for (const auto *axis : {"T", "W"}) {
        auto invalid = sdk;
        invalid["velocimeters"][0]["extrinsics"][axis] = {0,0,.1};
        bool rejected = false;
        try { Input::validate_config(invalid); } catch (const std::exception &) { rejected = true; }
        check(rejected, "SDK rejects repeated nonzero input extrinsic");
    }
    char temporary[] = "/tmp/t265-wheel-check-XXXXXX";
    const char *created = ::mkdtemp(temporary);
    check(created != nullptr, "wheel test temporary root");
    if (!created) return;
    const std::string root(created), log = root+"/wheel.jsonl";
    omni::LocalizationConfig config;
    const double forward[] = {.8660254037844386,0,.5}, up[] = {0,-1,0};
    std::copy(forward,forward+3,config.camera_robot_forward_axis);
    std::copy(up,up+3,config.camera_robot_up_axis);
    config.camera_offset_forward_m = -.0346105;
    config.camera_offset_left_m = -.0599471445;
    const double lever0 = -config.camera_offset_left_m, lever1 = config.camera_offset_forward_m;
    Json packet{{"schema", 2}, {"time_policy","received_feedback_no_age_gate_v1"},
        {"session_id", "sdk-test"}, {"packet_id", 0},
        {"source", "measured_wheel_feedback"}, {"measurement_valid", true},
        {"capture_time_qualified", false}, {"mcu_epoch", "boot1"},
        {"alignment_clock","mcu_event"},{"alignment_start_ns", 1000000000}, {"alignment_end_ns", 1010000000},
        {"receive_monotonic_ns", 1011000000}, {"gyro_receive_ns", 1011000000},
        {"alignment_gyro_start_ns", 1000000000}, {"alignment_gyro_end_ns", 1010000000}, {"time_uncertainty_ns", nullptr},
        {"v_body_mps", {0.,0.,0.}}, {"omega_body_rad_s", {0.,0.,1.}},
        {"lever_body_mps", {lever0,lever1,0.}},
        {"v_camera_mps", {.8660254037844386*lever0-.5*lever1,0.,.5*lever0+.8660254037844386*lever1}}};
    {
        Input input(config,"sdk-test","",root+"/missing/wheel.jsonl");
        int calls=0;
        check(input.process(packet,1020000000,[&](std::uint32_t,const rs2_vector &){++calls;return true;}) && calls==1,
              "unwritable diagnostic log does not gate a valid SDK call");
    }
    {
        Input input(config,"sdk-test","",log);
        int calls = 0;
        auto fake_sdk = [&](std::uint32_t seq, const rs2_vector &v) {
            check(seq == static_cast<std::uint32_t>(calls) && near(v.x, .069221, 1e-7) && near(v.z, 0, 1e-8),
                  "SDK gets once-converted camera velocity with sequence (not timestamp)");
            ++calls; return true;
        };
        check(input.process(packet,9000000000,fake_sdk), "old sample with unknown time quality reaches fake SDK");
        check(!input.process(packet,1020000000,fake_sdk) && calls == 1, "no duplicate injection");
        auto delayed = packet; delayed["packet_id"] = 1;
        delayed["alignment_start_ns"]=1010000000;delayed["alignment_end_ns"]=1020000000;
        delayed["alignment_gyro_end_ns"]=1020000000;
        check(input.process(delayed,10000000000ULL,fake_sdk) && calls == 2, "source interval accepted without IPC age gate");
        auto rotated_twice = packet; rotated_twice["v_camera_mps"] = {0.,0.,.069221};
        rotated_twice["packet_id"] = 2;
        rotated_twice["alignment_start_ns"] = 1020000000; rotated_twice["alignment_end_ns"] = 1030000000;
        rotated_twice["receive_monotonic_ns"] = 1020000000;
        rotated_twice["alignment_gyro_end_ns"] = 1030000000; rotated_twice["gyro_receive_ns"] = 1020000000;
        check(!input.process(rotated_twice,1020000000,fake_sdk) && calls == 2, "repeated/mismatched transform rejected");
    }
    {
        Input input(config,"sdk-test","",log);
        int calls = 0;
        auto intermittent_sdk = [&](std::uint32_t frame, const rs2_vector &) {
            check(frame == static_cast<std::uint32_t>(calls), "SDK failure still consumes its frame sequence");
            ++calls;
            if (calls == 1) return false;
            if (calls == 3) throw std::runtime_error("fixture SDK exception");
            return true;
        };
        auto sample = packet;
        check(!input.process(sample,1020000000,intermittent_sdk), "SDK false is failure, no accuracy claim");
        check(!input.process(sample,1020000000,intermittent_sdk) && calls == 1,
              "failed sample is not replayed");
        for (int index = 1; index <= 3; ++index) {
            sample["packet_id"] = index;
            sample["alignment_start_ns"] = 1000000000ULL + index*10000000ULL;
            sample["alignment_end_ns"] = 1010000000ULL + index*10000000ULL;
            sample["alignment_gyro_end_ns"] = sample["alignment_end_ns"];
            check(input.process(sample,1050000000,intermittent_sdk) == (index != 2),
                  "SDK false or exception does not block the next new measured sample");
            if (index == 2)
                check(!input.process(sample,1050000000,intermittent_sdk) && calls == 3,
                      "exception sample is not replayed");
        }
        check(calls == 4, "one SDK attempt per distinct measured sample, with no retry loop");
    }
    {
        Json status;
        std::ifstream file(root+"/wheel_sdk_status.json"); file >> status;
        check(status["sdk_result"] == true && status["sdk_api_success_samples"] == 2 &&
              status["sdk_api_failed_samples"] == 2 && status["rejected_samples"] == 4 &&
              status["last_sdk_call_failed"] == false && status["wheel_dependency_suspect"] == true &&
              status["sdk_failure_policy"] == "continue_with_next_new_sample",
              "recovered SDK status separates current success from historical failures");
        bool exception_recorded = false;
        std::ifstream records(log);
        for (std::string line; std::getline(records,line);) {
            const auto event = Json::parse(line);
            if (event.value("event",std::string()) == "wheel_sdk_result" &&
                event.value("error",std::string()) == "fixture SDK exception")
                exception_recorded = event["sdk_result"] == false && event["last_sdk_call_failed"] == true;
        }
        check(exception_recorded, "SDK exception remains a failed result in the original sample log");
    }
    {
        Json fixture=packet;
        fixture["source"]="synthetic_test_fixture";fixture["measurement_valid"]=false;
        fixture["test_fixture_valid"]=true;fixture["fixture_session_id"]="sdk-test";
        fixture["mcu_epoch"]=nullptr;fixture["alignment_clock"]="synthetic_host_clock";
        int calls=0;
        auto sink=[&](std::uint32_t,const rs2_vector &v){++calls;return near(v.x,.069221,1e-7);};
        {Input measured(config,"sdk-test","",log);
         check(!measured.process(fixture,10000000000ULL,sink) && calls==0,"normal SDK source cannot accept synthetic data");}
        {Input synthetic(config,"sdk-test","",log,true);
         check(synthetic.process(fixture,10000000000ULL,sink) && calls==1,"explicit isolated SDK experiment accepts labeled fixture");
         check(!synthetic.process(packet,10000000000ULL,sink) && calls==1,"synthetic SDK session cannot mix measured feedback");}
    }
    {
        Input input(config,"sdk-test","",log);int calls=0;
        auto sink=[&](std::uint32_t frame,const rs2_vector &){check(frame==static_cast<unsigned>(calls),"paused samples do not invent SDK frame numbers");++calls;return true;};
        check(input.process(packet,2000000000,sink)&&calls==1,"policy fixture starts with actual fake-sink call");
        Json policy{{"schema",3},{"kind","wheel_source_policy"},{"session_id","sdk-test"},{"pause",true},{"reason","wall-supported wheel suspect fixture"}};
        check(input.source_policy(policy),"same session source policy applies");
        auto next=packet;next["packet_id"]=1;next["alignment_start_ns"]=1010000000;next["alignment_end_ns"]=1020000000;
        next["alignment_gyro_end_ns"]=1020000000;
        check(!input.process(next,3000000000,sink)&&calls==1,"pause consumes measured sample without API/zero");
        auto wrong=policy;wrong["session_id"]="another-sdk";wrong["pause"]=false;
        check(!input.source_policy(wrong),"other SDK session cannot change source policy");
        policy["pause"]=false;check(input.source_policy(policy),"resume needs no estimator restart/config reload");
        check(!input.process(next,4000000000,sink)&&calls==1,"resume cannot replay a sample skipped while paused");
        next["packet_id"]=2;next["alignment_start_ns"]=1020000000;next["alignment_end_ns"]=1030000000;
        next["alignment_gyro_end_ns"]=1030000000;
        check(input.process(next,5000000000,sink)&&calls==2,"next genuinely new measured sample resumes normally");
    }
    ::unlink(log.c_str()); ::unlink((root+"/wheel_sdk_status.json").c_str()); ::rmdir(root.c_str());
}

void test_dynamic_pose_relay()
{
    using Relay = omni::DynamicPoseRelay;
    using Json = Relay::Json;
    const std::string path = "/tmp/omni-dynamic-pose-" + std::to_string(::getpid()) + ".json";
    std::vector<omni::FusedPoseFrame> frames;
    Relay relay("", "formal-fixture", path, [&](const std::array<std::uint8_t, omni::kFrameSize> &bytes) {
        omni::FusedPoseFrame f;
        check(omni::decode_fused_pose_frame(bytes.data(), bytes.size(), f), "dynamic 0x16 CRC/payload");
        frames.push_back(f); return true;
    });
    Json p = {{"schema_version", 1}, {"session_id", "formal-fixture"}, {"packet_id", 0},
        {"frame", "field"}, {"timestamp_monotonic_ns", 1}, {"quality", "DEGRADED"},
        {"pose", {{"x_m", -.123}, {"y_m", .527}, {"yaw_deg", 270.25}}},
        {"motion_source", "WHEEL_FALLBACK"}, {"t265", {{"tracker_confidence", 3}}}};
    check(relay.accept(p), "dynamic packet accepted without age gate or SDK Pose arrival");
    check(frames.size() == 1 && frames[0].x_mm == -123 && frames[0].y_mm == 527 &&
          frames[0].heading_cdeg == 27025, "dynamic field units and heading");
    check((frames[0].status & omni::kPoseValid) && (frames[0].status & omni::kPoseWheelActive) &&
          !(frames[0].status & omni::kPoseT265Good), "fallback remains valid, never mislabeled T265 good");
    check(!relay.accept(p), "duplicate transport packet not replayed");
    p["packet_id"] = 1; p["quality"] = "LOST";
    check(relay.accept(p) && !(frames.back().status & omni::kPoseValid), "reference invalid is not fabricated valid");
    p["packet_id"] = 2; p["session_id"] = "other";
    check(!relay.accept(p), "other session cannot override competition pose");
    relay.stop(); // Flush the one latest file snapshot, outside UART processing.
    Json saved; { std::ifstream file(path); file >> saved; }
    check(saved["pose"] == p["pose"] && saved["timestamp_monotonic_ns"] == 1 && saved["packet_id"] == 1,
          "formal JSON is the same latest dynamic snapshot and original sample time");
    ::unlink(path.c_str());

    // Exercise the actual receiver thread and scheduling with a local fake
    // UART sink; never open hardware or start a second SDK owner.
    const char *previous_rt = std::getenv("RDK_LOCALIZATION_RT");
    const bool had_rt = previous_rt != nullptr;
    const std::string saved_rt = previous_rt ? previous_rt : "";
    for (bool realtime : {true, false}) {
        ::setenv("RDK_LOCALIZATION_RT", realtime ? "1" : "off", 1);
        const std::string name = "@relay-thread-" + std::to_string(::getpid()) + (realtime ? "-rt" : "-normal");
        std::atomic<int> policy{-1}, priority{-1}, received{0};
        Relay threaded(name, "formal-fixture", path, [&](const std::array<std::uint8_t, omni::kFrameSize> &bytes) {
            omni::FusedPoseFrame frame;
            if (!omni::decode_fused_pose_frame(bytes.data(), bytes.size(), frame)) return false;
            sched_param actual{}; int actual_policy = -1;
            ::pthread_getschedparam(::pthread_self(), &actual_policy, &actual);
            policy = actual_policy; priority = actual.sched_priority; ++received;
            return true;
        });
        threaded.start();
        const int sender = ::socket(AF_UNIX, SOCK_DGRAM, 0);
        sockaddr_un address{}; address.sun_family = AF_UNIX;
        std::copy(name.begin() + 1, name.end(), address.sun_path + 1);
        p["session_id"] = "formal-fixture"; p["packet_id"] = 3; p["quality"] = "DEGRADED";
        const auto data = p.dump();
        check(::sendto(sender, data.data(), data.size(), 0, reinterpret_cast<sockaddr *>(&address),
                       offsetof(sockaddr_un, sun_path) + name.size()) == static_cast<ssize_t>(data.size()),
              "send original formal JSON to actual relay thread");
        for (int i = 0; i < 1000 && received == 0; ++i) ::usleep(1000);
        threaded.stop(); ::close(sender);
        check(received == 1, "actual relay thread delivers exactly one valid 0x16 to fake UART");
        check((policy == SCHED_FIFO && priority == 10) || (policy == SCHED_OTHER && priority == 0),
              "RT permission failure preserves working ordinary relay scheduling");
        if (!realtime) check(policy == SCHED_OTHER, "RT-off leaves relay in ordinary scheduling");
        ::unlink(path.c_str());
    }
    if (had_rt) ::setenv("RDK_LOCALIZATION_RT", saved_rt.c_str(), 1);
    else ::unsetenv("RDK_LOCALIZATION_RT");
}

void test_async_csv_logging()
{
    char pattern[] = "/tmp/t265-csv-XXXXXX";
    const char *directory = ::mkdtemp(pattern);
    check(directory != nullptr, "create isolated CSV directory");
    if (!directory) return;
    const std::string path = std::string(directory) + "/pose.csv";
    {
        omni::AsyncCsvLog log(path);
        log.submit("header\n");log.submit("first\n");log.submit("second\n");
        log.close();
        check(!log.failed() && log.dropped() == 0, "CSV close drains accepted rows");
    }
    std::ifstream file(path);
    const std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    check(content == "header\nfirst\nsecond\n", "CSV row order and contents survive async I/O");
    {
        omni::AsyncCsvLog appended(path, true);
        appended.submit("event\n");appended.close();
        std::ifstream events(path);
        const std::string complete((std::istreambuf_iterator<char>(events)), std::istreambuf_iterator<char>());
        check(complete == content + "event\n", "event logging preserves append behavior");
    }
    {
        omni::AsyncCsvLog failed(std::string(directory) + "/missing/pose.csv");
        failed.submit("real data\n");failed.close();
        check(failed.failed() && failed.dropped() == 1, "CSV failure is reported without throwing into control");
    }
    {
        omni::AsyncCsvLog full("/dev/full");
        full.submit("event\n");full.close();
        check(full.failed(), "disk-full flush failure is reported without stopping control");
    }
    ::unlink(path.c_str());::rmdir(directory);
}

void test_async_latest_snapshot()
{
    std::mutex mutex;
    std::condition_variable ready;
    bool entered = false, release = false;
    std::vector<std::string> written;
    omni::AsyncLatestFile latest("fixture", [&](const std::string &, const std::string &body) {
        std::unique_lock<std::mutex> lock(mutex);
        entered = true; ready.notify_one();
        ready.wait(lock, [&] { return release; });
        written.push_back(body);
        return true;
    });
    latest.submit("first");
    {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait(lock, [&] { return entered; });
    }
    bool formatted = false;
    const auto producer = std::this_thread::get_id();
    latest.submit("old pending");
    latest.submit([&] {
        check(std::this_thread::get_id() != producer, "snapshot formatting runs off the UART producer");
        formatted = true;
        return std::string("newest");
    });
    check(latest.coalesced() == 1, "snapshot producer completes while disk is blocked, bounded latest slot");
    check(!formatted, "blocked SD also defers formatting without blocking the producer");
    { std::lock_guard<std::mutex> lock(mutex); release = true; }
    ready.notify_one(); latest.close();
    check(written == std::vector<std::string>({"first", "newest"}), "close preserves the newest pending snapshot");
    omni::AsyncLatestFile failed("fixture", [](const std::string &, const std::string &) { return false; });
    failed.submit("pose"); failed.close();
    check(failed.errors() == 1, "snapshot failure is reported without throwing into control");
}

void test_serial_921600()
{
    // Real termios and duplex I/O, using only a PTY (no hardware UART or SDK).
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    check(master >= 0, "create serial PTY");
    if (master < 0) return;
    if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
        check(false, "prepare serial PTY"); ::close(master); return;
    }
    const std::string path(::ptsname(master));
    omni::SerialPort port(path, 921600);
    port.open_port();
    const int slave = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    termios attributes{};
    check(slave >= 0 && ::tcgetattr(slave, &attributes) == 0 &&
          ::cfgetispeed(&attributes) == B921600 && ::cfgetospeed(&attributes) == B921600,
          "921600 is actually applied to termios");
    const std::uint8_t packet[]{0xd4, 0x4d, 1, 7, 0xe4};
    std::uint8_t buffer[sizeof(packet)]{};
    check(::write(master, packet, sizeof(packet)) == sizeof(packet) &&
          port.read_some(buffer, sizeof(buffer), 100) == sizeof(packet) &&
          std::equal(std::begin(packet), std::end(packet), std::begin(buffer)), "921600 PTY RX bytes unchanged");
    check(port.write_all(packet, sizeof(packet), 100) &&
          ::read(master, buffer, sizeof(buffer)) == sizeof(packet) &&
          std::equal(std::begin(packet), std::end(packet), std::begin(buffer)), "921600 PTY TX bytes unchanged");
    if (slave >= 0) ::close(slave);
    ::close(master);
}

int replay_native_wheels(const char *input_path,const char *config_path)
{
    using Json=omni::DebugWheelInput::Json;
    const auto config=omni::load_config(config_path);
    const bool trace=std::getenv("RDK_WHEEL_REPLAY_TRACE")!=nullptr;
    omni::F405WheelMeasurements input(config,"parity-check",[&](Json j){if(trace)std::cerr<<j.dump()<<'\n';});
    std::ifstream file(input_path);if(!file)throw std::runtime_error("missing replay input");
    std::string line;
    while(std::getline(file,line)) {
        const auto event=Json::parse(line);const auto hex=event.at("raw_hex").get<std::string>();
        std::vector<std::uint8_t> bytes;
        for(std::size_t i=0;i<hex.size();i+=2)bytes.push_back(std::stoul(hex.substr(i,2),nullptr,16));
        const auto ns=event.at("timestamp_ns").get<std::uint64_t>();
        if(event.value("request",false))input.request(bytes.data(),bytes.size(),ns);
        else input.feedback(bytes.data(),bytes.size(),ns);
        if(event.value("drain",true)) {
            const auto ready=input.latest();if(!ready.is_null())std::cout<<ready.dump()<<'\n';
        }
    }
    return 0;
}

void test_native_uart_relay()
{
    const auto name="@test-native-uart-"+std::to_string(::getpid());
    const auto rxname=name+"-rx";
    const int peer=::socket(AF_UNIX,SOCK_SEQPACKET,0),receiver=::socket(AF_UNIX,SOCK_SEQPACKET,0);
    sockaddr_un address{};address.sun_family=AF_UNIX;
    std::memcpy(address.sun_path+1,rxname.data()+1,rxname.size()-1);
    timeval timeout{1,0};::setsockopt(peer,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    ::setsockopt(receiver,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    std::vector<std::uint8_t> actual;
    {
        omni::TestUartRelay relay(name,[&](const std::uint8_t *p,std::size_t n){actual.assign(p,p+n);return true;});
        check(::connect(receiver,reinterpret_cast<sockaddr *>(&address),offsetof(sockaddr_un,sun_path)+rxname.size())==0,"native transport RX connects");
        address={};address.sun_family=AF_UNIX;std::memcpy(address.sun_path+1,name.data()+1,name.size()-1);
        check(::connect(peer,reinterpret_cast<sockaddr *>(&address),offsetof(sockaddr_un,sun_path)+name.size())==0,"native transport connects");
        std::vector<std::uint8_t> request{0xd4,0x4d,1,1,0,8,0,7,0,0,0,0,0,0,0,5};
        auto crc=omni::modbus_crc16(request.data()+2,request.size()-2);
        request.push_back(crc&255);request.push_back(crc>>8);request.push_back(0xe4);
        check(::send(peer,request.data(),request.size(),0)==static_cast<ssize_t>(request.size()),"native test request sends once");
        std::uint8_t ack[17];
        check(::recv(peer,ack,sizeof(ack),0)==sizeof(ack)&&ack[0]==1&&actual==request,"native actual UART write result and bytes");
        relay.publish(request.data(),request.size(),1234567);
        std::uint8_t received[256];const auto n=::recv(receiver,received,sizeof(received),0);
        check(n==static_cast<ssize_t>(request.size()+16)&&std::equal(request.begin(),request.end(),received+16),"native RX retains complete UART bytes");
        std::uint64_t stamp=0;if(n>=8)for(int i=0;i<8;++i)stamp=(stamp<<8)|received[i];
        check(stamp==1234567,"native RX retains actual owner receive timestamp");
        request[3]=0x11;actual.clear();
        ::send(peer,request.data(),request.size(),0);
        check(::recv(peer,ack,sizeof(ack),0)==sizeof(ack)&&ack[0]==0&&actual.empty(),"test transport never forwards formal commands or bad CRC");
    }
    ::close(peer);::close(receiver);
}

void test_motion_feedback_tap()
{
    const auto name="@test-motion-feedback-"+std::to_string(::getpid());
    const int server=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_NONBLOCK,0);
    sockaddr_un address{};address.sun_family=AF_UNIX;
    std::memcpy(address.sun_path+1,name.data()+1,name.size()-1);
    const auto length=offsetof(sockaddr_un,sun_path)+name.size();
    if (server<0 || ::bind(server,reinterpret_cast<sockaddr *>(&address),length)!=0 ||
        ::listen(server,1)!=0) {
        check(false,"motion feedback fixture binds local listener");
        if(server>=0)::close(server);
        return;
    }
    auto frame=[](unsigned type) {
        std::vector<std::uint8_t> result{0xd4,0x4d,1,static_cast<std::uint8_t>(type),0,8,0,9,
                                       0x55,0,0x12,0x34,0x56,0x78,0x90,0};
        const auto crc=omni::modbus_crc16(result.data()+2,result.size()-2);
        result.push_back(crc&255);result.push_back(crc>>8);result.push_back(0xe4);
        return result;
    };
    {
        omni::MotionTelemetryTap tap(name);
        const int peer=::accept4(server,nullptr,nullptr,SOCK_NONBLOCK);
        check(peer>=0 && tap.stats().connected,"motion feedback independently connects");
        if(peer>=0) {
            std::uint8_t received[179];
            check(::recv(peer,received,sizeof(received),MSG_DONTWAIT)<0 &&
                  (errno==EAGAIN || errno==EWOULDBLOCK),"motion feedback sends no MCU or local handshake");
            const std::uint64_t stamp=0x0102030405060708ULL;
            int legacy=0;
            omni::F405MixedFrames mixed(
                [&](const std::uint8_t *,std::size_t){++legacy;},
                [&](const std::uint8_t *p,std::size_t n){tap.publish(p,n,stamp);});
            for(unsigned type: {2u,4u,6u,7u,8u}) {
                const auto raw=frame(type);
                for(auto byte:raw)mixed.feed(&byte,1);
                const auto n=::recv(peer,received,sizeof(received),MSG_DONTWAIT);
                check(n==static_cast<ssize_t>(raw.size()+8) &&
                      std::equal(raw.begin(),raw.end(),received+8),"motion feedback keeps full original extended bytes");
                std::uint64_t actual=0;
                if(n>=8)for(int i=0;i<8;++i)actual=(actual<<8)|received[i];
                check(actual==stamp,"motion feedback retains original big-endian UART receive time");
            }
            const auto filtered=frame(10);
            mixed.feed(filtered.data(),filtered.size());
            auto corrupt=frame(7);corrupt[8]^=1;
            mixed.feed(corrupt.data(),corrupt.size());
            const auto counts=omni::build_encoder_frame(omni::EncoderFrame{});
            mixed.feed(counts.data(),counts.size());
            check(::recv(peer,received,sizeof(received),MSG_DONTWAIT)<0 &&
                  (errno==EAGAIN || errno==EWOULDBLOCK),"motion feedback excludes stats, bad CRC and legacy frames");
            check(tap.stats().eligible_frames==5 && tap.stats().sent_frames==5 &&
                  tap.stats().filtered_frames==1 && legacy==1,"motion feedback counts only passive allowed extended frames");
            const auto wheel=frame(7);
            for(int i=0;i<4096 && !tap.stats().dropped_frames;++i)
                tap.publish(wheel.data(),wheel.size(),stamp);
            check(tap.stats().dropped_frames>0 && tap.stats().send_errors>0 &&
                  tap.stats().connected,"full motion feedback socket drops without blocking UART publisher");
            const auto dropped=tap.stats().dropped_frames;
            ::close(peer);
            tap.publish(wheel.data(),wheel.size(),stamp);
            check(tap.stats().dropped_frames==dropped+1 && !tap.stats().connected,
                  "disconnected motion consumer never blocks or replays feedback");
        }
    }
    ::close(server);
    {
        omni::MotionTelemetryTap missing(name+"-absent");
        const auto wheel=frame(7);
        missing.publish(wheel.data(),wheel.size(),1);
        check(!missing.stats().connected && missing.stats().connection_error!=0 &&
              missing.stats().eligible_frames==1 && missing.stats().dropped_frames==1,
              "missing motion consumer is counted without a UART/SDK gate");
    }
    {
        omni::MotionTelemetryTap disabled("");
        const auto wheel=frame(7);disabled.publish(wheel.data(),wheel.size(),1);
        check(disabled.stats().eligible_frames==0 && disabled.stats().dropped_frames==0,
              "unspecified motion lane preserves previous behaviour");
    }
}

int main(int argc,char **argv)
{
    if(argc==4 && std::string(argv[1])=="--wheel-replay")return replay_native_wheels(argv[2],argv[3]);
    test_motion_feedback_tap();
    test_native_uart_relay();
    test_serial_921600();
    test_async_latest_snapshot();
    test_async_csv_logging();
    test_dynamic_pose_relay();
    test_debug_wheel_input();
    test_protocol();
    test_kinematics();
    test_new_mount();
    test_projection_gate_and_filter();
    test_t265_yaw_time_sync();
    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "all protocol, kinematics, gate, and EKF tests passed\n";
    return EXIT_SUCCESS;
}
