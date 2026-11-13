#pragma once

#include <json.hpp>
#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace omni {

// Inline optional storage, retaining the project's C++14 build and avoiding
// allocations in the numerical state. False is the Python None equivalent.
template<class T> struct NativeMotionValue {
    bool present = false;
    T value{};
    explicit operator bool() const noexcept { return present; }
    T &operator*() { return value; }
    const T &operator*() const { return value; }
    T *operator->() { return &value; }
    const T *operator->() const { return &value; }
    NativeMotionValue &operator=(T next) {
        value = std::move(next); present = true; return *this;
    }
    void reset() noexcept { present = false; }
};

// Legacy policy remains identical to lidar_yaw_debug/motion_source.py; the
// explicit robust experiment adds continuous SDK coordinates and wheel-only
// bootstrap. Numerical state only: no UART, SDK, clock reads, locks or threads.
class NativeMotionSource {
public:
    using Json = nlohmann::json;
    using Vec2 = std::array<double, 2>;
    using Vec3 = std::array<double, 3>;
    using Row = std::array<double, 4>;
    using History = std::deque<Row>;
    using YawHistory = std::deque<std::array<double, 2>>;
    using Basis = std::array<Vec2, 2>;
    struct Pair { Vec2 xy; Vec3 wheel; };
    struct WallWindow {
        double t;
        Vec2 wall, vio, wheel;
        double travel;
    };

    void configure(const Json &configuration);
    void reset_sdk_session();
    void sdk_unavailable(const std::string &reason);
    Vec2 raw_sdk_point(const Vec2 &point) const;
    void set_wheel_suppressed(bool suppressed, const std::string &reason);
    bool wheel_input_paused() const noexcept { return phase_wheel_suppressed || wheel_suspect; }
    std::string wheel_input_reason() const;
    void pose(const Json &sample);
    void yaw(std::int64_t host_ns, double yaw_rad, const Json &source);
    void wheel(const Json &sample, const YawHistory &host_yaws);
    // Negative reference_ns is the Python None used by geometry-only callers.
    void observe_walls(const std::string &scan_id, const std::vector<std::string> &axes,
                       std::int64_t reference_ns, const Json &center_xy,
                       const History &raw_poses, bool reacquired = false);
    Json snapshot(bool include_evidence = true) const;
    static Vec3 sample(const History &history, double t);
    static double sample_yaw(const YawHistory &history, double t);
    static const Json &policy();
    Vec3 wheel_at(double t) const;

    History wheels, pending, history;
    YawHistory native_yaws;
    std::deque<Json> events;
    Json reference = nullptr;
    Basis basis{{Vec2{{1., 0.}}, Vec2{{0., 1.}}}};
    NativeMotionValue<std::pair<Json, std::uint32_t>> last_wheel;
    NativeMotionValue<Vec2> anchor, anchor_wheel;
    Vec2 bias{{0., 0.}};
    NativeMotionValue<Pair> previous, window, recovery_window;
    std::string mode = "T265_NO_WHEEL_REFERENCE", reason = "no_measured_wheel_reference";
    double recovery_travel = 0.;
    int wall_fixes = 0;
    NativeMotionValue<std::string> last_wall;
    NativeMotionValue<std::int64_t> last_paired_ns, last_raw_ns;
    NativeMotionValue<int> last_confidence;
    double last_conflict_m = 0.;
    Json wheel_time_basis = nullptr, wheel_yaw_support = nullptr;
    std::uint64_t pending_pose_drops = 0;
    NativeMotionValue<Vec2> last_raw_xy;
    NativeMotionValue<std::int64_t> last_tracked_ns, resume_anchor_ns;
    NativeMotionValue<Vec2> last_tracked_xy, resume_anchor_xy;
    bool phase_wheel_suppressed = false;
    std::string phase_wheel_reason;
    bool phase_restore_wheel_fallback = false;
    std::uint64_t phase_position_bridges = 0;
    NativeMotionValue<std::uint64_t> phase_previous_frame;
    NativeMotionValue<Vec2> phase_previous_velocity;
    bool wheel_suspect = false;
    NativeMotionValue<WallWindow> wall_window;
    Json wall_evidence = nullptr;
    NativeMotionValue<std::string> evidence_verdict;
    int evidence_confirmations = 0;
    NativeMotionValue<std::string> last_evidence_scan;

    // Explicit experimental policy. Raw SDK/counter histories are never rewritten.
    bool robust_odometry_enabled = false;
    bool t265_suspect = false;
    std::uint64_t sdk_position_bridges = 0;
    History continuous_poses;
    Vec2 sdk_coordinate_bias{{0., 0.}};
    Json latest_pose = nullptr, previous_valid_pose = nullptr;
    bool sdk_adopted = false;
    Basis sdk_basis{{Vec2{{1., 0.}}, Vec2{{0., 1.}}}};
    YawHistory host_yaw_reference;
    bool sdk_rotation_pending = false;
    bool sdk_stream_available = false;
    Vec2 sdk_frame_translation{{0., 0.}};
    bool sdk_frame_translation_pending = false;
    History raw_sdk_frame_poses;

private:
    void robust_pose(const Json &sample);
    void robust_process();
    void robust_fallback(const std::string &reason, double t);
    void process();
    void fallback(const std::string &reason, double t);
    void publish();
    void publish_vio(double t, const Vec2 &xy);
    void select_wall_supported_vio(double t, const History &raw_poses);
    void select_wall_supported_wheel(double t);
    void event(Json value);
};

} // namespace omni
