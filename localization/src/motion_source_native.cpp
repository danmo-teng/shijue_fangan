#include "motion_source_native.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace omni {
namespace {
using Source = NativeMotionSource;
using Vec2 = Source::Vec2;
using Vec3 = Source::Vec3;
using Json = Source::Json;

template<class T> void append_bounded(std::deque<T> &rows, T row, std::size_t capacity = 4096) {
    if (rows.size() == capacity) rows.pop_front();
    rows.push_back(std::move(row));
}
Vec2 first2(const Vec3 &v) { return {{v[0], v[1]}}; }
Vec2 add(const Vec2 &a, const Vec2 &b) { return {{a[0] + b[0], a[1] + b[1]}}; }
Vec2 subtract(const Vec2 &a, const Vec2 &b) { return {{a[0] - b[0], a[1] - b[1]}}; }
Vec2 rotate(const Source::Basis &r, const Vec2 &v) {
    return {{r[0][0] * v[0] + r[0][1] * v[1],
             r[1][0] * v[0] + r[1][1] * v[1]}};
}
Vec2 xy_json(const Json &v) { return {{v.at(0).get<double>(), v.at(1).get<double>()}}; }
double norm2(const Vec2 &v) { return std::hypot(v[0], v[1]); }
// The lower-frequency wall verdict used np.linalg.norm, while the high-frequency
// checks used math.hypot. Retain that distinction and the original operations.
double wall_norm2(const Vec2 &v) { return std::sqrt(v[0] * v[0] + v[1] * v[1]); }
bool finite(const Vec2 &v) { return std::isfinite(v[0]) && std::isfinite(v[1]); }
bool nearby_orientation(const Json &a, const Json &b) {
    if (!a.count("rotation_xyzw") || !b.count("rotation_xyzw")) return false;
    const auto &qa = a.at("rotation_xyzw"), &qb = b.at("rotation_xyzw");
    if (!qa.is_array() || !qb.is_array() || qa.size() != 4 || qb.size() != 4) return false;
    double aa = 0., bb = 0., ab = 0.;
    for (std::size_t i = 0; i < 4; ++i) {
        if (!qa.at(i).is_number() || !qb.at(i).is_number()) return false;
        const double x = qa.at(i).get<double>(), y = qb.at(i).get<double>();
        if (!std::isfinite(x) || !std::isfinite(y)) return false;
        aa += x * x; bb += y * y; ab += x * y;
    }
    if (aa < 1e-12 || bb < 1e-12) return false;
    // q and -q represent the same orientation. Check all three attitude axes,
    // including obstacle-induced roll/pitch, rather than planar yaw alone.
    return 2. * std::acos(std::min(1., std::fabs(ab) / std::sqrt(aa * bb))) <= .10;
}
bool equal_optional(const NativeMotionValue<std::string> &value, const std::string &other) {
    return value && *value == other;
}
template<class T> Json optional_json(const NativeMotionValue<T> &value) {
    return value ? Json(*value) : Json(nullptr);
}
std::int64_t reference_time(const Json &reference) {
    return reference.value("reference_sample_ns", reference.value("timestamp_monotonic_ns", std::int64_t(0)));
}
} // namespace

const NativeMotionSource::Json &NativeMotionSource::policy() {
    static const Json value{{"single_increment_conflict_m", .05}, {"displacement_conflict_m", .05},
        {"comparison_travel_m", .25}, {"recovery_residual_m", .025},
        {"recovery_travel_m", .50}, {"recovery_two_axis_fixes", 2}};
    return value;
}

NativeMotionSource::Vec3 NativeMotionSource::sample(const History &rows, double t) {
    if (rows.empty()) throw std::invalid_argument("no motion samples");
    auto values = [](const Row &row) -> Vec3 { return {{row[1], row[2], row[3]}}; };
    if (t <= rows.front()[0]) return values(rows.front());
    if (t >= rows.back()[0]) return values(rows.back());
    auto right = std::lower_bound(rows.begin(), rows.end(), t,
        [](const Row &row, double query) { return row[0] < query; });
    if (right == rows.begin()) return values(*right);
    if (right == rows.end()) return values(rows.back());
    const auto &a = *(right - 1), &b = *right;
    const double f = (t - a[0]) / (b[0] - a[0]);
    return {{a[1] * (1. - f) + b[1] * f,
             a[2] * (1. - f) + b[2] * f,
             a[3] * (1. - f) + b[3] * f}};
}

double NativeMotionSource::sample_yaw(const YawHistory &rows, double t) {
    if (rows.empty()) throw std::invalid_argument("no yaw samples");
    if (t <= rows.front()[0]) return rows.front()[1];
    if (t >= rows.back()[0]) return rows.back()[1];
    auto right = std::lower_bound(rows.begin(), rows.end(), t,
        [](const std::array<double, 2> &row, double query) { return row[0] < query; });
    const auto &a = *(right - 1), &b = *right;
    const double f = (t - a[0]) / (b[0] - a[0]);
    return a[1] * (1. - f) + b[1] * f;
}

void NativeMotionSource::event(Json value) { append_bounded(events, std::move(value), 128); }

void NativeMotionSource::yaw(std::int64_t host_ns, double yaw_rad, const Json &source) {
    if (robust_odometry_enabled && (host_yaw_reference.empty() || host_ns >= host_yaw_reference.back()[0])) {
        if (!host_yaw_reference.empty() && host_ns == host_yaw_reference.back()[0]) host_yaw_reference.pop_back();
        append_bounded(host_yaw_reference, std::array<double, 2>{{static_cast<double>(host_ns), yaw_rad}});
    }
    const Json raw = source.value("raw", Json::object());
    if (raw.find("yaw_receive_us") == raw.end()) return;
    const auto t = raw.at("yaw_receive_us").get<std::int64_t>() * 1000;
    if (!native_yaws.empty() && t < native_yaws.back()[0]) return;
    if (!native_yaws.empty() && t == native_yaws.back()[0]) native_yaws.pop_back();
    append_bounded(native_yaws, std::array<double, 2>{{static_cast<double>(t), yaw_rad}});
}

std::string NativeMotionSource::wheel_input_reason() const {
    return phase_wheel_suppressed ? phase_wheel_reason : wheel_suspect
        ? (robust_odometry_enabled && t265_suspect ? "both_motion_sources_suspect" :
            "two_axis_wall_motion_wheel_suspect") : "measured_wheel_input_enabled";
}

void NativeMotionSource::set_wheel_suppressed(bool suppressed, const std::string &why) {
    if (suppressed == phase_wheel_suppressed) {
        if (suppressed) { phase_wheel_reason = why; reason = why; }
        return;
    }
    const auto previous_mode = mode;
    phase_wheel_suppressed = suppressed; phase_wheel_reason = suppressed ? why : "";
    NativeMotionValue<Vec2> selected;
    NativeMotionValue<std::int64_t> t;
    if (!history.empty()) {
        selected = Vec2{{history.back()[1], history.back()[2]}};
        t = static_cast<std::int64_t>(history.back()[0]);
    } else if (last_tracked_xy) { selected = *last_tracked_xy; t = *last_tracked_ns; }
    if (selected && last_tracked_xy) bias = subtract(*selected, *last_tracked_xy);
    pending.clear(); previous.reset(); window.reset(); recovery_window.reset();
    recovery_travel = 0.; wall_fixes = 0; last_wall.reset();
    wall_window.reset(); wall_evidence = nullptr;
    evidence_verdict.reset(); evidence_confirmations = 0; last_evidence_scan.reset();
    resume_anchor_ns.reset(); resume_anchor_xy.reset();
    if (suppressed) {
        mode = "T265_PHASE_NO_WHEEL"; reason = why;
        if (history.empty() && !reference.is_null() && last_tracked_ns)
            publish_vio(*last_tracked_ns, *last_tracked_xy);
    } else if (wheel_suspect) {
        if (robust_odometry_enabled && t265_suspect && selected && !wheels.empty()) {
            anchor = *selected; anchor_wheel = first2(wheel_at(wheels.back()[0]));
            mode = "WHEEL_UNCERTAIN_BOTH_SUSPECT"; reason = "phase_exit_both_motion_sources_suspect";
        } else {
            mode = "T265_PRIMARY_WHEEL_SUSPECT"; reason = "two_axis_walls_support_T265_over_wheel";
        }
    } else if (!reference.is_null() && selected) {
        // Anchor only once actual wheel support reaches the selected Pose.
        // Excluded wheel totals are never replayed as a post-phase tail.
        resume_anchor_ns = wheels.empty() ? *t : std::max(*t, static_cast<std::int64_t>(wheels.back()[0]));
        resume_anchor_xy = *selected;
        anchor.reset(); anchor_wheel.reset();
        mode = robust_odometry_enabled && t265_suspect ? "WHEEL_FALLBACK" : "T265_WITH_WHEEL_TAIL";
        reason = "phase_exit_rebased";
        process();
    } else { mode = "T265_NO_WHEEL_REFERENCE"; reason = "no_measured_wheel_reference"; }
    event({{"event", "wheel_phase_suppression_changed"}, {"sample_monotonic_ns", optional_json(t)},
        {"previous_source", previous_mode}, {"motion_source", mode}, {"suppressed", suppressed}, {"reason", why},
        {"selected_anchor_m", optional_json(anchor)}, {"wheel_anchor_m", optional_json(anchor_wheel)}});
}

void NativeMotionSource::configure(const Json &configuration) {
    const bool keep_sdk_frame = robust_odometry_enabled && configuration.value("robust_odometry_enabled", false) &&
        configuration.value("reference_coordinate_source", std::string()) == "selected_odometry_local";
    const Basis kept_sdk_basis = sdk_basis;
    const Vec2 kept_sdk_coordinate_bias = sdk_coordinate_bias, kept_bias = bias;
    const Vec2 kept_frame_translation = sdk_frame_translation;
    robust_odometry_enabled = configuration.value("robust_odometry_enabled", false);
    t265_suspect = false; sdk_position_bridges = 0; sdk_adopted = false;
    continuous_poses.clear(); sdk_coordinate_bias = {{0., 0.}}; previous_valid_pose = nullptr;
    sdk_basis = {{Vec2{{1., 0.}}, Vec2{{0., 1.}}}};
    sdk_rotation_pending = robust_odometry_enabled &&
        configuration.value("reference_coordinate_source", std::string()) == "measured_wheel_local";
    sdk_frame_translation = {{0., 0.}}; sdk_frame_translation_pending = robust_odometry_enabled;
    raw_sdk_frame_poses.clear();
    if (keep_sdk_frame) {
        sdk_basis = kept_sdk_basis; sdk_coordinate_bias = kept_sdk_coordinate_bias;
        sdk_frame_translation = kept_frame_translation; sdk_frame_translation_pending = false;
    }
    if (robust_odometry_enabled && sdk_stream_available && !sdk_rotation_pending && !latest_pose.is_null() && latest_pose.value("tracker_confidence", 0) > 0) {
        previous_valid_pose = latest_pose;
        const Vec2 xy = add(rotate(sdk_basis, Vec2{{latest_pose.at("x_m").get<double>(), latest_pose.at("y_m").get<double>()}}), sdk_coordinate_bias);
        append_bounded(continuous_poses, Row{{latest_pose.at("timestamp_monotonic_ns").get<double>(),
            xy[0], xy[1], static_cast<double>(latest_pose.at("tracker_confidence").get<int>())}});
        // The reference was formed in this already-observed SDK local frame.
        // There is no delayed first-session transform left to discover.
        sdk_frame_translation_pending = false;
    }
    reference = configuration; history.clear(); anchor.reset();
    bias = {{0., 0.}}; previous.reset(); window.reset(); recovery_window.reset();
    if (keep_sdk_frame) bias = kept_bias;
    recovery_travel = 0.; wall_fixes = 0; last_wall.reset();
    last_paired_ns.reset(); mode = "T265_NO_WHEEL_REFERENCE";
    reason = "no_measured_wheel_reference";
    wheel_suspect = false; wall_window.reset(); wall_evidence = nullptr;
    evidence_verdict.reset(); evidence_confirmations = 0; last_evidence_scan.reset();
    resume_anchor_ns.reset(); resume_anchor_xy.reset();
    const double angle = reference.at("yaw_offset").get<double>() - reference.at("vio_rotation").get<double>();
    const double c = std::cos(angle), s = std::sin(angle);
    basis = {{Vec2{{c, -s}}, Vec2{{s, c}}}};
    const auto t = reference_time(reference);
    pending.erase(std::remove_if(pending.begin(), pending.end(),
        [t](const Row &p) { return p[0] < t; }), pending.end());
    if (phase_wheel_suppressed) {
        pending.clear(); mode = "T265_PHASE_NO_WHEEL"; reason = phase_wheel_reason;
        if (last_tracked_ns && *last_tracked_ns >= t) publish_vio(*last_tracked_ns, *last_tracked_xy);
        return;
    }
    process();
}

void NativeMotionSource::reset_sdk_session() {
    if (!robust_odometry_enabled) return;
    pending.clear(); continuous_poses.clear(); previous_valid_pose = nullptr; latest_pose = nullptr;
    raw_sdk_frame_poses.clear(); sdk_frame_translation = {{0., 0.}}; sdk_frame_translation_pending = true;
    last_raw_ns.reset(); last_raw_xy.reset(); last_tracked_ns.reset(); last_tracked_xy.reset();
    last_confidence.reset(); previous.reset(); window.reset(); recovery_window.reset();
    wall_window.reset(); wall_evidence = nullptr; evidence_verdict.reset(); evidence_confirmations = 0;
    sdk_coordinate_bias = {{0., 0.}}; sdk_rotation_pending = true; sdk_adopted = false;
    t265_suspect = true;
    sdk_stream_available = false;
    // Keep the selected coordinate frame and real wheel accumulation. A new SDK
    // session establishes its own rotation/translation only from actual Pose.
    if (!phase_wheel_suppressed && !wheels.empty()) {
        if (!history.empty()) {
            const double t = std::min(history.back()[0], wheels.back()[0]);
            anchor = first2(sample(history, t)); anchor_wheel = first2(wheel_at(t));
        }
        mode = wheel_suspect ? "WHEEL_UNCERTAIN_BOTH_SUSPECT" : "WHEEL_ONLY_NO_SDK";
        reason = "SDK_session_changed_keep_field_and_wheel_reference";
    }
    event({{"event", "SDK_session_reference_reset"}, {"field_reference_preserved", true},
        {"raw_wheel_accumulation_preserved", true}, {"wheel_motion_suspect", wheel_suspect}});
}

NativeMotionSource::Vec2 NativeMotionSource::raw_sdk_point(const Vec2 &point) const {
    return robust_odometry_enabled ? add(rotate(sdk_basis, point), sdk_frame_translation) : point;
}

void NativeMotionSource::sdk_unavailable(const std::string &why) {
    if (!robust_odometry_enabled) return;
    sdk_stream_available = false; t265_suspect = true; sdk_adopted = false;
    pending.clear(); previous.reset(); window.reset(); recovery_window.reset();
    if (phase_wheel_suppressed) {
        mode = "T265_PHASE_NO_WHEEL";
    } else if (!wheels.empty()) {
        if (!history.empty()) {
            const double t = std::min(history.back()[0], wheels.back()[0]);
            anchor = first2(sample(history, t)); anchor_wheel = first2(wheel_at(t));
        }
        mode = wheel_suspect ? "WHEEL_UNCERTAIN_BOTH_SUSPECT" : "WHEEL_ONLY_NO_SDK";
    } else mode = "NO_MOTION_SOURCE_SUPPORT";
    reason = why;
    event({{"event", "SDK_unavailable_keep_supported_odometry"}, {"reason", why},
        {"motion_source", mode}, {"field_reference_preserved", true},
        {"wheel_motion_suspect", wheel_suspect}, {"synthetic_SDK_sample", false}});
}

void NativeMotionSource::pose(const Json &p) {
    if (robust_odometry_enabled) { robust_pose(p); return; }
    const auto t = p.at("timestamp_monotonic_ns").get<std::int64_t>();
    const Vec2 xy{{p.at("x_m").get<double>(), p.at("y_m").get<double>()}};
    if (!finite(xy) || (last_raw_ns && t <= *last_raw_ns)) return;
    const double raw_step = mode == "T265_PRIMARY_WHEEL_SUSPECT" && last_raw_xy
        ? norm2(subtract(xy, *last_raw_xy)) : 0.;
    const int confidence = p.at("tracker_confidence").get<int>();
    last_raw_ns = t; last_raw_xy = xy; last_confidence = confidence;
    if (confidence > 0) { last_tracked_ns = t; last_tracked_xy = xy; }
    if (phase_wheel_suppressed) {
        if (confidence > 0 && !reference.is_null()) publish_vio(t, xy);
        return;
    }
    if (resume_anchor_ns && confidence > 0) publish_vio(t, xy);
    if (pending.size() == 4096) ++pending_pose_drops;
    append_bounded(pending, Row{{static_cast<double>(t), xy[0], xy[1], static_cast<double>(confidence)}});
    if (confidence <= 0 && anchor) fallback("SDK_tracking_lost", t);
    else if (mode == "T265_PRIMARY_WHEEL_SUSPECT" && raw_step > .05)
        fallback("SDK_single_step_jump_while_wheel_suspect", t);
    else if (mode == "T265_PRIMARY_WHEEL_SUSPECT") publish_vio(t, xy);
    process();
}

void NativeMotionSource::wheel(const Json &p, const YawHistory &host_yaws) {
    if (p.value("source", std::string()) != "measured_wheel_feedback" ||
        !p.value("measurement_valid", false) || p.value("mcu_epoch", Json()).is_null()) return;
    const auto &raw = p.at("raw_mcu_sample");
    const auto key = std::make_pair(p.at("mcu_epoch"), p.at("sample_id").get<std::uint32_t>());
    if (last_wheel && key.first != last_wheel->first) return;
    if (last_wheel) {
        const std::uint32_t step = key.second - last_wheel->second;
        if (!(step > 0 && step < 0x80000000u)) return;
    }
    const auto a = raw.at("t0_us").get<std::int64_t>() * 1000;
    const auto b = raw.at("t1_us").get<std::int64_t>() * 1000;
    const Vec2 v = xy_json(p.at("v_body_mps"));
    if (a >= b || !finite(v) || host_yaws.empty()) return;
    const Json js = p.value("interval_start_ns", Json()), je = p.value("interval_end_ns", Json());
    std::int64_t start = 0, end = 0;
    wheel_time_basis = "mapped_MCU_event_estimate";
    if (js.is_null() || je.is_null() || (!wheels.empty() && je.get<double>() < wheels.back()[0])) {
        end = p.at("receive_monotonic_ns").get<std::int64_t>(); start = end - (b - a);
        wheel_time_basis = "receive_end_with_native_interval_duration";
    } else { start = js.get<std::int64_t>(); end = je.get<std::int64_t>(); }
    if (!wheels.empty() && end < wheels.back()[0]) return;
    double ya, yb;
    if (!native_yaws.empty()) {
        ya = sample_yaw(native_yaws, a); yb = sample_yaw(native_yaws, b);
        wheel_yaw_support = native_yaws.front()[0] <= a && b <= native_yaws.back()[0]
            ? "MCU_interval_bracketed" : "MCU_interval_endpoint_held";
    } else {
        ya = sample_yaw(host_yaws, start); yb = sample_yaw(host_yaws, end);
        wheel_yaw_support = "host_interval_estimate";
    }
    const double angle = (ya + yb) / 2., dt = (b - a) / 1e9;
    const Vec2 delta{{dt * (std::cos(angle) * v[0] - std::sin(angle) * v[1]),
                      dt * (std::sin(angle) * v[0] + std::cos(angle) * v[1])}};
    if (wheels.empty()) append_bounded(wheels, Row{{static_cast<double>(start), 0., 0., 0.}});
    const Row old = wheels.back();
    const Vec2 xy{{old[1] + delta[0], old[2] + delta[1]}};
    const double travel = old[3] + norm2(v) * dt;
    if (end == old[0]) wheels.pop_back();
    append_bounded(wheels, Row{{static_cast<double>(end), xy[0], xy[1], travel}});
    last_wheel = key;
    process();
}

NativeMotionSource::Vec3 NativeMotionSource::wheel_at(double t) const {
    const Vec3 w = sample(wheels, t);
    const Vec2 xy = rotate(basis, first2(w));
    return {{xy[0], xy[1], w[2]}};
}

void NativeMotionSource::fallback(const std::string &why, double t) {
    if (robust_odometry_enabled) { robust_fallback(why, t); return; }
    if (phase_wheel_suppressed) return;
    if (mode == "T265_PRIMARY_WHEEL_SUSPECT" && !wheels.empty() && !history.empty()) {
        const double end = wheels.back()[0];
        anchor = first2(sample(history, end)); anchor_wheel = first2(wheel_at(end));
    }
    const bool entering = mode != "WHEEL_FALLBACK";
    if (entering) event({{"event", "motion_source_changed"},
        {"sample_monotonic_ns", static_cast<std::int64_t>(t)}, {"previous_source", mode},
        {"motion_source", "WHEEL_FALLBACK"}, {"reason", why}, {"spatial_conflict_m", last_conflict_m}});
    mode = "WHEEL_FALLBACK";
    // Keep the first cause through later paired displacement conflicts.
    // A new explicit wall verdict is a different diagnosis, not a repeat.
    if (entering || why == "two_axis_walls_support_wheel_over_T265") reason = why;
    recovery_window.reset(); recovery_travel = 0.; wall_fixes = 0;
}

void NativeMotionSource::process() {
    if (robust_odometry_enabled) { robust_process(); return; }
    if (phase_wheel_suppressed) return;
    if (reference.is_null() || wheels.empty()) return;
    if (resume_anchor_ns) {
        if (wheels.back()[0] < *resume_anchor_ns) return;
        const double t = std::max(static_cast<double>(*resume_anchor_ns), wheels.front()[0]);
        anchor = history.empty() ? *resume_anchor_xy : first2(sample(history, t));
        anchor_wheel = first2(wheel_at(t));
        pending.erase(std::remove_if(pending.begin(), pending.end(),
            [t](const Row &p) { return p[0] < t; }), pending.end());
        resume_anchor_ns.reset(); resume_anchor_xy.reset();
    }
    const auto ref_ns = reference_time(reference);
    if (!anchor && wheels.front()[0] <= ref_ns && ref_ns <= wheels.back()[0]) {
        const Vec2 xy = xy_json(reference.at("pose_origin")); const Vec3 w = wheel_at(ref_ns);
        anchor = xy; anchor_wheel = first2(w);
        previous = Pair{xy, w}; window = Pair{xy, w};
        mode = "T265_WITH_WHEEL_TAIL"; reason = "paired_field_reference"; last_paired_ns = ref_ns;
    }
    while (!pending.empty() && pending.front()[0] <= wheels.back()[0]) {
        const Row p = pending.front(); pending.pop_front();
        const double t = p[0]; const Vec2 xy{{p[1], p[2]}}; const int confidence = static_cast<int>(p[3]);
        if (t < wheels.front()[0]) continue;
        const Vec3 w = wheel_at(t);
        if (!anchor) {
            if (confidence <= 0) continue;
            anchor = xy; anchor_wheel = first2(w);
            mode = "T265_WITH_WHEEL_TAIL"; reason = "paired_actual_feedback";
            previous = Pair{xy, w}; window = Pair{xy, w};
        } else if (confidence <= 0) {
            fallback("SDK_tracking_lost", t); previous.reset(); window.reset(); continue;
        } else if (!previous) { previous = Pair{xy, w}; window = Pair{xy, w}; }
        else {
            const Pair prev = *previous, base = *window;
            const double step = norm2(subtract(subtract(xy, prev.xy), subtract(first2(w), first2(prev.wheel))));
            const double conflict = norm2(subtract(subtract(xy, base.xy), subtract(first2(w), first2(base.wheel))));
            last_conflict_m = std::max(step, conflict);
            const bool bad = step > .05 || conflict > .05;
            if (bad) {
                if (mode != "T265_PRIMARY_WHEEL_SUSPECT") fallback("T265_wheel_displacement_conflict", t);
                else { recovery_window.reset(); recovery_travel = 0.; }
                window = Pair{xy, w};
            } else if (mode == "T265_WITH_WHEEL_TAIL") {
                anchor = add(xy, bias); anchor_wheel = first2(w);
                if (w[2] - base.wheel[2] >= .25) window = Pair{xy, w};
            } else {
                if (!recovery_window) recovery_window = Pair{xy, w};
                const Pair recovery = *recovery_window;
                const double residual = norm2(subtract(subtract(xy, recovery.xy), subtract(first2(w), first2(recovery.wheel))));
                if (residual > .025) { recovery_window = Pair{xy, w}; recovery_travel = 0.; }
                else if (w[2] - recovery.wheel[2] >= .25) {
                    recovery_travel += w[2] - recovery.wheel[2]; recovery_window = Pair{xy, w};
                }
                if (recovery_travel >= .50 && wall_fixes >= 2 &&
                    (!wheel_suspect || (equal_optional(evidence_verdict, "BOTH_AGREE") && evidence_confirmations >= 2))) {
                    const auto previous_mode = mode;
                    const Vec2 continuous = mode == "T265_PRIMARY_WHEEL_SUSPECT" ? add(xy, bias)
                        : subtract(add(*anchor, first2(w)), *anchor_wheel);
                    bias = subtract(continuous, xy); anchor = continuous; anchor_wheel = first2(w);
                    mode = "T265_WITH_WHEEL_TAIL"; reason = "motion_and_two_axis_walls_agree_rebased";
                    wheel_suspect = false; window = Pair{xy, w};
                    event({{"event", "motion_source_changed"}, {"sample_monotonic_ns", static_cast<std::int64_t>(t)},
                        {"previous_source", previous_mode}, {"motion_source", mode}, {"reason", reason},
                        {"t265_translation_rebase_m", bias}, {"recovery_travel_m", recovery_travel}});
                }
            }
            previous = Pair{xy, w};
        }
        last_paired_ns = static_cast<std::int64_t>(t);
    }
    publish();
}

void NativeMotionSource::robust_pose(const Json &p) {
    const auto t = p.at("timestamp_monotonic_ns").get<std::int64_t>();
    const Vec2 raw{{p.at("x_m").get<double>(), p.at("y_m").get<double>()}};
    if (!finite(raw) || (last_raw_ns && t <= *last_raw_ns)) return;
    const int confidence = p.at("tracker_confidence").get<int>();
    sdk_stream_available = true;
    if (sdk_rotation_pending && confidence > 0) {
        if (host_yaw_reference.empty() || !p.count("yaw_rad")) return;
        const double main_yaw = sample_yaw(host_yaw_reference, t);
        const double angle = main_yaw + std::atan2(basis[1][0], basis[0][0]) - p.at("yaw_rad").get<double>();
        const double c = std::cos(angle), s = std::sin(angle);
        sdk_basis = {{Vec2{{c, -s}}, Vec2{{s, c}}}}; sdk_rotation_pending = false;
        event({{"event", "actual_SDK_coordinate_rotation_bound"}, {"sample_monotonic_ns", t},
            {"sdk_to_selected_rotation_rad", angle}, {"main_yaw_rad", main_yaw},
            {"SDK_yaw_rad", p.at("yaw_rad")}, {"field_reference_preserved", true}});
    }
    Vec2 xy = add(rotate(sdk_basis, raw), sdk_coordinate_bias);
    bool unbridged_jump = false;
    if (confidence > 0 && !previous_valid_pose.is_null() && !continuous_poses.empty()) {
        const auto &old = previous_valid_pose;
        const double dt = (t - old.at("timestamp_monotonic_ns").get<std::int64_t>()) / 1e9;
        const Vec2 old_raw{{old.at("x_m").get<double>(), old.at("y_m").get<double>()}};
        const Vec2 step = rotate(sdk_basis, subtract(raw, old_raw));
        const bool consecutive = !latest_pose.is_null() && latest_pose.value("tracker_confidence", 0) > 0 &&
            p.count("frame_number") && old.count("frame_number") &&
            p.at("frame_number").get<std::uint64_t>() == old.at("frame_number").get<std::uint64_t>() + 1;
        bool bridged = false;
        bool velocity_displacement_conflict = false;
        if (consecutive && dt > 0. && p.count("vx_local_mps") && p.count("vy_local_mps") &&
            old.count("vx_local_mps") && old.count("vy_local_mps")) {
            const Vec2 va = rotate(sdk_basis, Vec2{{old.at("vx_local_mps").get<double>(), old.at("vy_local_mps").get<double>()}});
            const Vec2 vb = rotate(sdk_basis, Vec2{{p.at("vx_local_mps").get<double>(), p.at("vy_local_mps").get<double>()}});
            const Vec2 bridge{{.5 * (va[0] + vb[0]) * dt, .5 * (va[1] + vb[1]) * dt}};
            velocity_displacement_conflict = finite(va) && finite(vb) && norm2(subtract(step, bridge)) > .05;
            // This is a physical consistency classification of adjacent frames,
            // not an age gate. A long interval's actual velocity displacement is
            // included, and missing frames/velocity cannot qualify a bridge.
            if (finite(va) && finite(vb) && norm2(step) > .05 &&
                norm2(subtract(step, bridge)) > .05 && norm2(subtract(va, vb)) <= .20 &&
                norm2(va) <= 1.5 && norm2(vb) <= 1.5 && nearby_orientation(old, p)) {
                sdk_coordinate_bias = add(sdk_coordinate_bias, subtract(bridge, step));
                xy = add(rotate(sdk_basis, raw), sdk_coordinate_bias); bridged = true; ++sdk_position_bridges;
                event({{"event", "SDK_translation_jump_bridged"}, {"sample_monotonic_ns", t},
                    {"raw_position_step_m", step}, {"single_interval_bridge_m", bridge},
                    {"sdk_coordinate_rebase_m", sdk_coordinate_bias}, {"frame_number", p.at("frame_number")},
                    {"independent_ground_truth", false}});
            }
        }
        // Large displacement alone across a long/gapped interval is not a jump.
        // Only a direct adjacent position step is classified here; the paired
        // wheel displacement comparison handles accumulated disagreements.
        unbridged_jump = !bridged && norm2(step) > .05 &&
            (velocity_displacement_conflict || norm2(step) > 1.5 * dt + .05);
    }
    latest_pose = p; last_raw_ns = t; last_raw_xy = raw; last_confidence = confidence;
    if (confidence > 0) {
        last_tracked_ns = t; last_tracked_xy = xy; previous_valid_pose = p;
        append_bounded(continuous_poses, Row{{static_cast<double>(t), xy[0], xy[1], static_cast<double>(confidence)}});
        const Vec2 sdk_raw = rotate(sdk_basis, raw);
        append_bounded(raw_sdk_frame_poses, Row{{static_cast<double>(t), sdk_raw[0], sdk_raw[1], static_cast<double>(confidence)}});
    }
    if (confidence <= 0) {
        t265_suspect = true;
        if (!phase_wheel_suppressed) robust_fallback("SDK_tracking_lost", t);
        return;
    }
    if (!sdk_adopted && !history.empty()) {
        bias = subtract(first2(sample(history, t)), xy);
        if (phase_wheel_suppressed || wheels.empty()) {
            sdk_frame_translation = subtract(first2(sample(history, t)), rotate(sdk_basis, raw));
            sdk_frame_translation_pending = false; sdk_adopted = true;
        }
    } else if (!sdk_adopted && history.empty() && wheels.empty()) {
        // A genuine SDK-only start already defines this local coordinate frame;
        // do not re-anchor away the first physical increment on the next Pose.
        sdk_adopted = true; sdk_frame_translation_pending = false;
    }
    if (unbridged_jump) {
        t265_suspect = true;
        if (phase_wheel_suppressed || wheels.empty()) {
            // There is no supported alternate XY increment in this case. Keep
            // the previous selected estimate across this one uncertain step;
            // subsequent actual SDK increments remain available, marked suspect.
            if (!history.empty()) bias = subtract(first2(sample(history, t)), xy);
            mode = phase_wheel_suppressed ? "T265_PHASE_NO_WHEEL" : "T265_UNCERTAIN_NO_WHEEL";
            reason = "SDK_unclassified_jump_no_supported_alternate";
            event({{"event", "motion_increment_unknown"}, {"sample_monotonic_ns", t},
                {"reason", reason}, {"held_estimate_is_measured_static", false}});
        } else robust_fallback("SDK_position_and_motion_inconsistent", t);
    }
    if (phase_wheel_suppressed || wheels.empty() || mode == "T265_PRIMARY_WHEEL_SUSPECT" || resume_anchor_ns) {
        if (!reference.is_null()) publish_vio(t, xy);
        if (wheels.empty() && !phase_wheel_suppressed && !t265_suspect) {
            mode = "T265_NO_WHEEL_REFERENCE"; reason = "no_measured_wheel_reference";
        }
    }
    if (pending.size() == 4096) ++pending_pose_drops;
    append_bounded(pending, Row{{static_cast<double>(t), xy[0], xy[1], static_cast<double>(confidence)}});
    robust_process();
}

void NativeMotionSource::robust_fallback(const std::string &why, double t) {
    t265_suspect = true;
    if (phase_wheel_suppressed || wheels.empty()) return;
    const auto previous_mode = mode;
    const std::string target = wheel_suspect ? "WHEEL_UNCERTAIN_BOTH_SUSPECT" : "WHEEL_FALLBACK";
    if ((mode == "T265_PRIMARY_WHEEL_SUSPECT" || !anchor) && !history.empty()) {
        const double end = std::min(wheels.back()[0], history.back()[0]);
        anchor = first2(sample(history, end)); anchor_wheel = first2(wheel_at(end));
    }
    if (previous_mode != target) event({{"event", "motion_source_changed"},
        {"sample_monotonic_ns", static_cast<std::int64_t>(t)}, {"previous_source", previous_mode},
        {"motion_source", target}, {"reason", why}, {"wheel_motion_suspect", wheel_suspect},
        {"t265_motion_suspect", true}, {"independent_ground_truth", false}});
    mode = target;
    if (previous_mode != target || why == "two_axis_walls_support_wheel_over_T265" ||
        why == "two_axis_walls_contradict_both_motion_sources") reason = why;
    recovery_window.reset(); recovery_travel = 0.; wall_fixes = 0;
}

void NativeMotionSource::robust_process() {
    if (phase_wheel_suppressed || reference.is_null() || wheels.empty()) return;
    if (resume_anchor_ns) {
        if (wheels.back()[0] < *resume_anchor_ns) return;
        const double t = std::max(static_cast<double>(*resume_anchor_ns), wheels.front()[0]);
        anchor = history.empty() ? *resume_anchor_xy : first2(sample(history, t));
        anchor_wheel = first2(wheel_at(t));
        pending.erase(std::remove_if(pending.begin(), pending.end(),
            [t](const Row &p) { return p[0] < t; }), pending.end());
        resume_anchor_ns.reset(); resume_anchor_xy.reset();
    }
    const auto ref_ns = reference_time(reference);
    if (!anchor) {
        if (ref_ns > wheels.back()[0]) return;
        const double t = std::max(static_cast<double>(ref_ns), wheels.front()[0]);
        anchor = xy_json(reference.at("pose_origin")); anchor_wheel = first2(wheel_at(t));
        if (!history.empty()) anchor = first2(sample(history, t));
        if (continuous_poses.empty()) {
            mode = "WHEEL_ONLY_NO_SDK"; reason = "measured_wheel_and_primary_yaw_only";
        } else {
            mode = "T265_WITH_WHEEL_TAIL"; reason = "paired_field_reference";
            previous = Pair{subtract(*anchor, bias), wheel_at(t)};
            window = *previous; sdk_adopted = true;
        }
    }
    while (!pending.empty() && pending.front()[0] <= wheels.back()[0]) {
        const Row p = pending.front(); pending.pop_front(); const double t = p[0];
        if (t < wheels.front()[0] || t < ref_ns) continue;
        const Vec2 xy{{p[1], p[2]}}; const Vec3 w = wheel_at(t);
        if (p[3] <= 0.) { robust_fallback("SDK_tracking_lost", t); previous.reset(); window.reset(); continue; }
        if (!sdk_adopted) {
            // First real SDK sample after wheel-only startup: align it to the
            // already-published wheel trajectory; never manufacture a Pose.
            const Vec2 selected = subtract(add(*anchor, first2(w)), *anchor_wheel);
            bias = subtract(selected, xy); anchor = selected; anchor_wheel = first2(w);
            if (sdk_frame_translation_pending && !raw_sdk_frame_poses.empty()) {
                sdk_frame_translation = subtract(selected, first2(sample(raw_sdk_frame_poses, t)));
                sdk_frame_translation_pending = false;
            }
            mode = wheel_suspect ? (t265_suspect ? "WHEEL_UNCERTAIN_BOTH_SUSPECT" : "T265_PRIMARY_WHEEL_SUSPECT") : "T265_WITH_WHEEL_TAIL";
            reason = "first_actual_SDK_pose_rebased_to_wheel_only_trajectory";
            sdk_adopted = true; if (!wheel_suspect) t265_suspect = false;
            previous = Pair{xy, w}; window = *previous;
            event({{"event", "actual_SDK_pose_adopted"}, {"sample_monotonic_ns", static_cast<std::int64_t>(t)},
                {"t265_translation_rebase_m", bias}, {"synthetic_SDK_sample", false}});
        } else if (!previous || !window) { previous = Pair{xy, w}; window = *previous; }
        else {
            const Pair prev = *previous, base = *window;
            const double step = norm2(subtract(subtract(xy, prev.xy), subtract(first2(w), first2(prev.wheel))));
            const double conflict = norm2(subtract(subtract(xy, base.xy), subtract(first2(w), first2(base.wheel))));
            last_conflict_m = std::max(step, conflict);
            if (step > .05 || conflict > .05) {
                if (mode != "T265_PRIMARY_WHEEL_SUSPECT") robust_fallback("T265_wheel_accumulated_displacement_conflict", t);
                else { recovery_window.reset(); recovery_travel = 0.; }
                window = Pair{xy, w};
            } else if (mode == "T265_WITH_WHEEL_TAIL") {
                anchor = add(xy, bias); anchor_wheel = first2(w);
                // Do not reset this comparison every .25m anchor update: that
                // hides slow drift smaller than the per-leg conflict threshold.
            } else {
                if (!recovery_window) recovery_window = Pair{xy, w};
                const Pair recovery = *recovery_window;
                const double residual = norm2(subtract(subtract(xy, recovery.xy), subtract(first2(w), first2(recovery.wheel))));
                if (residual > .025) { recovery_window = Pair{xy, w}; recovery_travel = 0.; }
                else if (w[2] - recovery.wheel[2] >= .25) {
                    recovery_travel += w[2] - recovery.wheel[2]; recovery_window = Pair{xy, w};
                }
                if (recovery_travel >= .50 && wall_fixes >= 2 &&
                    (!wheel_suspect || (equal_optional(evidence_verdict, "BOTH_AGREE") && evidence_confirmations >= 2))) {
                    const Vec2 selected = mode == "T265_PRIMARY_WHEEL_SUSPECT" ? add(xy, bias) :
                        subtract(add(*anchor, first2(w)), *anchor_wheel);
                    bias = subtract(selected, xy); anchor = selected; anchor_wheel = first2(w);
                    mode = "T265_WITH_WHEEL_TAIL"; reason = "motion_and_two_axis_walls_agree_rebased";
                    t265_suspect = false; wheel_suspect = false; window = Pair{xy, w};
                    event({{"event", "motion_source_changed"}, {"sample_monotonic_ns", static_cast<std::int64_t>(t)},
                        {"motion_source", mode}, {"reason", reason}, {"t265_translation_rebase_m", bias}});
                }
            }
            previous = Pair{xy, w};
        }
        last_paired_ns = static_cast<std::int64_t>(t);
    }
    publish();
}

void NativeMotionSource::publish() {
    if (phase_wheel_suppressed || resume_anchor_ns) return;
    if (!anchor || wheels.empty() || mode == "T265_PRIMARY_WHEEL_SUSPECT") return;
    const double t = wheels.back()[0]; const Vec3 w = wheel_at(t);
    const Vec2 xy = subtract(add(*anchor, first2(w)), *anchor_wheel);
    if (!history.empty() && t < history.back()[0]) return;
    if (!history.empty() && t == history.back()[0]) history.pop_back();
    append_bounded(history, Row{{t, xy[0], xy[1], 0.}});
}

void NativeMotionSource::publish_vio(double t, const Vec2 &xy) {
    if (!history.empty() && t < history.back()[0]) return;
    if (!history.empty() && t == history.back()[0]) history.pop_back();
    const Vec2 selected = add(xy, bias);
    append_bounded(history, Row{{static_cast<double>(static_cast<std::int64_t>(t)), selected[0], selected[1], 0.}});
}

void NativeMotionSource::select_wall_supported_vio(double t, const History &raw_poses) {
    if (phase_wheel_suppressed) return;
    const History &sdk = robust_odometry_enabled ? continuous_poses : raw_poses;
    if (sdk.empty()) return;
    const auto previous_mode = mode;
    const Vec2 selected = first2(sample(history.empty() ? raw_poses : history, t));
    const Vec2 raw = first2(sample(sdk, t));
    bias = subtract(selected, raw);
    while (!history.empty() && history.back()[0] >= t) history.pop_back();
    append_bounded(history, Row{{static_cast<double>(static_cast<std::int64_t>(t)), selected[0], selected[1], 0.}});
    for (const auto &pose : sdk) if (pose[0] > t) publish_vio(pose[0], Vec2{{pose[1], pose[2]}});
    anchor = selected; anchor_wheel = first2(wheel_at(t));
    mode = "T265_PRIMARY_WHEEL_SUSPECT"; wheel_suspect = true;
    if (robust_odometry_enabled) {
        t265_suspect = false; previous = Pair{raw, wheel_at(t)}; window = *previous;
    }
    reason = "two_axis_walls_support_T265_over_wheel";
    recovery_window.reset(); recovery_travel = 0.; wall_fixes = 0;
    event({{"event", "motion_source_changed"}, {"sample_monotonic_ns", static_cast<std::int64_t>(t)},
        {"previous_source", previous_mode}, {"motion_source", mode}, {"reason", reason},
        {"wheel_input_pause_requested", true}, {"wall_motion_evidence", wall_evidence}});
}

void NativeMotionSource::select_wall_supported_wheel(double t) {
    if (phase_wheel_suppressed) return;
    const Vec2 selected = first2(sample(history, t)); const Vec3 w = wheel_at(t);
    while (!history.empty() && history.back()[0] >= t) history.pop_back();
    append_bounded(history, Row{{static_cast<double>(static_cast<std::int64_t>(t)), selected[0], selected[1], 0.}});
    for (const auto &row : wheels) if (row[0] > t) {
        const Vec2 xy = subtract(add(selected, first2(wheel_at(row[0]))), first2(w));
        append_bounded(history, Row{{static_cast<double>(static_cast<std::int64_t>(row[0])), xy[0], xy[1], 0.}});
    }
    wheel_suspect = false; fallback("two_axis_walls_support_wheel_over_T265", t);
    previous.reset(); window.reset();
}

void NativeMotionSource::observe_walls(const std::string &scan_id, const std::vector<std::string> &axes,
        std::int64_t reference_ns, const Json &center_xy, const History &raw_poses, bool reacquired) {
    if (phase_wheel_suppressed) return; // The outer geometric correction still applies.
    const bool full_xy = std::set<std::string>(axes.begin(), axes.end()) == std::set<std::string>{"x", "y"};
    if ((mode == "WHEEL_FALLBACK" || mode == "WHEEL_UNCERTAIN_BOTH_SUSPECT" || wheel_suspect) && !equal_optional(last_wall, scan_id)) {
        last_wall = scan_id; wall_fixes = full_xy ? wall_fixes + 1 : 0;
    }
    if (reference_ns < 0) return;
    if (equal_optional(last_evidence_scan, scan_id)) return;
    last_evidence_scan = scan_id;
    const History &sdk = robust_odometry_enabled ? continuous_poses : raw_poses;
    if (!full_xy || center_xy.is_null() || reference.is_null() || sdk.empty() || wheels.empty()) {
        wall_window.reset(); evidence_verdict.reset(); evidence_confirmations = 0;
        wall_evidence = {{"status", "NO_FULL_XY_WALL_EVIDENCE"}, {"scan_id", scan_id}}; return;
    }
    const double t = static_cast<double>(reference_ns);
    if (!(sdk.front()[0] <= t && t <= sdk.back()[0] && wheels.front()[0] <= t && t <= wheels.back()[0])) {
        wall_window.reset(); evidence_verdict.reset(); evidence_confirmations = 0;
        wall_evidence = {{"status", "NO_COMMON_INTERVAL_SUPPORT"}, {"scan_id", scan_id}}; return;
    }
    const double angle = reference.at("vio_rotation").get<double>(), c = std::cos(angle), s = std::sin(angle);
    const Basis r{{Vec2{{c, -s}}, Vec2{{s, c}}}};
    const Vec2 vio = add(rotate(r, subtract(first2(sample(sdk, t)), xy_json(reference.at("pose_origin")))),
                        xy_json(reference.at("start_center_xy_m")));
    const Vec3 w = wheel_at(t); const Vec2 wxy = rotate(r, first2(w));
    const WallWindow now{t, xy_json(center_xy), vio, wxy, w[2]};
    if (!wall_window || reacquired) {
        wall_window = now; evidence_verdict.reset(); evidence_confirmations = 0;
        wall_evidence = {{"status", "WALL_MOTION_BASELINE"}, {"scan_id", scan_id}, {"reference_ns", reference_ns}}; return;
    }
    const WallWindow old = *wall_window;
    if (t <= old.t) return;
    const Vec2 wall_delta = subtract(now.wall, old.wall), vio_delta = subtract(vio, old.vio), wheel_delta = subtract(wxy, old.wheel);
    const double activity = std::max({wall_norm2(wall_delta), wall_norm2(vio_delta), w[2] - old.travel});
    const double ve = wall_norm2(subtract(vio_delta, wall_delta)), we = wall_norm2(subtract(wheel_delta, wall_delta));
    if (activity < .25 && !(robust_odometry_enabled && std::max(ve, we) > .05)) return;
    const std::string verdict = ve <= .025 && we > .05 ? "WHEEL_SUSPECT" :
        we <= .025 && ve > .05 ? "T265_SUSPECT" : std::max(ve, we) <= .025 ? "BOTH_AGREE" :
        robust_odometry_enabled && ve > .05 && we > .05 ? "BOTH_SUSPECT" : "AMBIGUOUS";
    evidence_confirmations = equal_optional(evidence_verdict, verdict) ? evidence_confirmations + 1 : 1;
    evidence_verdict = verdict;
    wall_evidence = {{"status", verdict}, {"scan_id", scan_id}, {"reference_ns", reference_ns},
        {"baseline_ns", static_cast<std::int64_t>(old.t)}, {"movement_m", activity},
        {"t265_wall_displacement_error_m", ve}, {"wheel_wall_displacement_error_m", we},
        {"confirmations", evidence_confirmations}, {"t265_depends_on_sdk_wheel_input", true},
        {"walls_depend_on_primary_yaw_and_selected_deskew", true}, {"independent_ground_truth", false}};
    bool new_source_verdict = false;
    if (evidence_confirmations >= 2) {
        if (verdict == "WHEEL_SUSPECT" && mode != "T265_PRIMARY_WHEEL_SUSPECT") {
            select_wall_supported_vio(t, raw_poses); new_source_verdict = true;
        } else if (verdict == "T265_SUSPECT" && (wheel_suspect || robust_odometry_enabled)) {
            new_source_verdict = reason != "two_axis_walls_support_wheel_over_T265";
            select_wall_supported_wheel(t);
        } else if (verdict == "BOTH_SUSPECT") {
            new_source_verdict = !wheel_suspect || !t265_suspect ||
                reason != "two_axis_walls_contradict_both_motion_sources";
            wheel_suspect = true; t265_suspect = true;
            robust_fallback("two_axis_walls_contradict_both_motion_sources", t);
            wall_evidence["conflict_is_independent_fault_proof"] = false;
        }
    }
    if (robust_odometry_enabled && new_source_verdict) {
        // Recovery must describe new motion after the conflict, rather than
        // cancellation of an old wrong displacement inside the same baseline.
        wall_window = now; wall_evidence["next_interval_baseline_reset"] = true;
    } else if (activity >= .50 && (evidence_confirmations >= 2 || verdict == "AMBIGUOUS")) wall_window = now;
}

NativeMotionSource::Json NativeMotionSource::snapshot(bool include_evidence) const {
    Json result{{"mode", mode}, {"reason", reason}, {"spatial_conflict_m", last_conflict_m},
        {"last_paired_pose_ns", optional_json(last_paired_ns)}, {"last_raw_pose_ns", optional_json(last_raw_ns)},
        {"last_wheel_sample_id", last_wheel ? Json(last_wheel->second) : Json(nullptr)},
        {"wheel_time_basis", wheel_time_basis}, {"wheel_yaw_support", wheel_yaw_support},
        {"pending_pose_drops", pending_pose_drops}, {"recovery_travel_m", recovery_travel},
        {"recovery_two_axis_fixes", wall_fixes}, {"t265_translation_rebase_m", bias}, {"policy", policy()},
        {"wheel_motion_suspect", wheel_suspect}, {"wall_motion_evidence", wall_evidence},
        {"phase_wheel_suppressed", phase_wheel_suppressed}, {"phase_wheel_suppression_reason", phase_wheel_reason},
        {"wheel_input_policy", {{"pause_requested", wheel_input_paused()},
            {"reason", wheel_input_reason()},
            {"dependency_persists_after_pause", true}}},
        {"wheel_and_gyro_are_sdk_dependencies", true}, {"independent_ground_truth", false}};
    if (robust_odometry_enabled) {
        result["wheel_and_gyro_are_sdk_dependencies"] = nullptr;
        result["wheel_and_gyro_can_be_sdk_dependencies"] = true;
        result["actual_sdk_dependency_scope"] = "see SDK owner per-generation API log; not inferred from configuration";
        result["robust_odometry_enabled"] = true;
        result["t265_motion_suspect"] = t265_suspect;
        result["both_motion_suspect"] = t265_suspect && wheel_suspect;
        result["sdk_position_jump_bridges"] = sdk_position_bridges;
        result["sdk_coordinate_rebase_m"] = sdk_coordinate_bias;
        result["sdk_to_selected_rotation_rad"] = std::atan2(sdk_basis[1][0], sdk_basis[0][0]);
        result["sdk_rotation_pending"] = sdk_rotation_pending;
        result["sdk_stream_available"] = sdk_stream_available;
        result["sdk_frame_translation_m"] = sdk_frame_translation;
        result["source_conflict_is_independent_fault_diagnosis"] = false;
        result["sdk_pose_received"] = !continuous_poses.empty();
    }
    if (!include_evidence) result.erase("wall_motion_evidence");
    return result;
}

} // namespace omni
