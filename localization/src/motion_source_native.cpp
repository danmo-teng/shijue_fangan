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
    (void)host_ns;
    const Json raw = source.value("raw", Json::object());
    if (raw.find("yaw_receive_us") == raw.end()) return;
    const auto t = raw.at("yaw_receive_us").get<std::int64_t>() * 1000;
    if (!native_yaws.empty() && t < native_yaws.back()[0]) return;
    if (!native_yaws.empty() && t == native_yaws.back()[0]) native_yaws.pop_back();
    append_bounded(native_yaws, std::array<double, 2>{{static_cast<double>(t), yaw_rad}});
}

void NativeMotionSource::configure(const Json &configuration) {
    reference = configuration; history.clear(); anchor.reset();
    bias = {{0., 0.}}; previous.reset(); window.reset(); recovery_window.reset();
    recovery_travel = 0.; wall_fixes = 0; last_wall.reset();
    last_paired_ns.reset(); mode = "T265_NO_WHEEL_REFERENCE";
    reason = "no_measured_wheel_reference";
    wheel_suspect = false; wall_window.reset(); wall_evidence = nullptr;
    evidence_verdict.reset(); evidence_confirmations = 0; last_evidence_scan.reset();
    const double angle = reference.at("yaw_offset").get<double>() - reference.at("vio_rotation").get<double>();
    const double c = std::cos(angle), s = std::sin(angle);
    basis = {{Vec2{{c, -s}}, Vec2{{s, c}}}};
    const auto t = reference_time(reference);
    pending.erase(std::remove_if(pending.begin(), pending.end(),
        [t](const Row &p) { return p[0] < t; }), pending.end());
    process();
}

void NativeMotionSource::pose(const Json &p) {
    const auto t = p.at("timestamp_monotonic_ns").get<std::int64_t>();
    const Vec2 xy{{p.at("x_m").get<double>(), p.at("y_m").get<double>()}};
    if (!finite(xy) || (last_raw_ns && t <= *last_raw_ns)) return;
    const double raw_step = mode == "T265_PRIMARY_WHEEL_SUSPECT" && last_raw_xy
        ? norm2(subtract(xy, *last_raw_xy)) : 0.;
    const int confidence = p.at("tracker_confidence").get<int>();
    last_raw_ns = t; last_raw_xy = xy; last_confidence = confidence;
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
    if (mode == "T265_PRIMARY_WHEEL_SUSPECT" && !wheels.empty() && !history.empty()) {
        const double end = wheels.back()[0];
        anchor = first2(sample(history, end)); anchor_wheel = first2(wheel_at(end));
    }
    if (mode != "WHEEL_FALLBACK") event({{"event", "motion_source_changed"},
        {"sample_monotonic_ns", static_cast<std::int64_t>(t)}, {"previous_source", mode},
        {"motion_source", "WHEEL_FALLBACK"}, {"reason", why}, {"spatial_conflict_m", last_conflict_m}});
    mode = "WHEEL_FALLBACK"; reason = why;
    recovery_window.reset(); recovery_travel = 0.; wall_fixes = 0;
}

void NativeMotionSource::process() {
    if (reference.is_null() || wheels.empty()) return;
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

void NativeMotionSource::publish() {
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
    const auto previous_mode = mode;
    const Vec2 selected = first2(sample(history.empty() ? raw_poses : history, t));
    const Vec2 raw = first2(sample(raw_poses, t));
    bias = subtract(selected, raw);
    while (!history.empty() && history.back()[0] >= t) history.pop_back();
    append_bounded(history, Row{{static_cast<double>(static_cast<std::int64_t>(t)), selected[0], selected[1], 0.}});
    for (const auto &pose : raw_poses) if (pose[0] > t) publish_vio(pose[0], Vec2{{pose[1], pose[2]}});
    anchor = selected; anchor_wheel = first2(wheel_at(t));
    mode = "T265_PRIMARY_WHEEL_SUSPECT"; wheel_suspect = true;
    reason = "two_axis_walls_support_T265_over_wheel";
    recovery_window.reset(); recovery_travel = 0.; wall_fixes = 0;
    event({{"event", "motion_source_changed"}, {"sample_monotonic_ns", static_cast<std::int64_t>(t)},
        {"previous_source", previous_mode}, {"motion_source", mode}, {"reason", reason},
        {"wheel_input_pause_requested", true}, {"wall_motion_evidence", wall_evidence}});
}

void NativeMotionSource::select_wall_supported_wheel(double t) {
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
    const bool full_xy = std::set<std::string>(axes.begin(), axes.end()) == std::set<std::string>{"x", "y"};
    if ((mode == "WHEEL_FALLBACK" || wheel_suspect) && !equal_optional(last_wall, scan_id)) {
        last_wall = scan_id; wall_fixes = full_xy ? wall_fixes + 1 : 0;
    }
    if (reference_ns < 0) return;
    if (equal_optional(last_evidence_scan, scan_id)) return;
    last_evidence_scan = scan_id;
    if (!full_xy || center_xy.is_null() || reference.is_null() || raw_poses.empty() || wheels.empty()) {
        wall_window.reset(); evidence_verdict.reset(); evidence_confirmations = 0;
        wall_evidence = {{"status", "NO_FULL_XY_WALL_EVIDENCE"}, {"scan_id", scan_id}}; return;
    }
    const double t = static_cast<double>(reference_ns);
    if (!(raw_poses.front()[0] <= t && t <= raw_poses.back()[0] && wheels.front()[0] <= t && t <= wheels.back()[0])) {
        wall_window.reset(); evidence_verdict.reset(); evidence_confirmations = 0;
        wall_evidence = {{"status", "NO_COMMON_INTERVAL_SUPPORT"}, {"scan_id", scan_id}}; return;
    }
    const double angle = reference.at("vio_rotation").get<double>(), c = std::cos(angle), s = std::sin(angle);
    const Basis r{{Vec2{{c, -s}}, Vec2{{s, c}}}};
    const Vec2 vio = add(rotate(r, subtract(first2(sample(raw_poses, t)), xy_json(reference.at("pose_origin")))),
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
    if (activity < .25) return;
    const double ve = wall_norm2(subtract(vio_delta, wall_delta)), we = wall_norm2(subtract(wheel_delta, wall_delta));
    const std::string verdict = ve <= .025 && we > .05 ? "WHEEL_SUSPECT" :
        we <= .025 && ve > .05 ? "T265_SUSPECT" : std::max(ve, we) <= .025 ? "BOTH_AGREE" : "AMBIGUOUS";
    evidence_confirmations = equal_optional(evidence_verdict, verdict) ? evidence_confirmations + 1 : 1;
    evidence_verdict = verdict;
    wall_evidence = {{"status", verdict}, {"scan_id", scan_id}, {"reference_ns", reference_ns},
        {"baseline_ns", static_cast<std::int64_t>(old.t)}, {"movement_m", activity},
        {"t265_wall_displacement_error_m", ve}, {"wheel_wall_displacement_error_m", we},
        {"confirmations", evidence_confirmations}, {"t265_depends_on_sdk_wheel_input", true},
        {"walls_depend_on_primary_yaw_and_selected_deskew", true}, {"independent_ground_truth", false}};
    if (evidence_confirmations >= 2) {
        if (verdict == "WHEEL_SUSPECT" && mode != "T265_PRIMARY_WHEEL_SUSPECT") select_wall_supported_vio(t, raw_poses);
        else if (verdict == "T265_SUSPECT" && wheel_suspect) select_wall_supported_wheel(t);
    }
    if (activity >= .50 && (evidence_confirmations >= 2 || verdict == "AMBIGUOUS")) wall_window = now;
}

NativeMotionSource::Json NativeMotionSource::snapshot(bool include_evidence) const {
    Json result{{"mode", mode}, {"reason", reason}, {"spatial_conflict_m", last_conflict_m},
        {"last_paired_pose_ns", optional_json(last_paired_ns)}, {"last_raw_pose_ns", optional_json(last_raw_ns)},
        {"last_wheel_sample_id", last_wheel ? Json(last_wheel->second) : Json(nullptr)},
        {"wheel_time_basis", wheel_time_basis}, {"wheel_yaw_support", wheel_yaw_support},
        {"pending_pose_drops", pending_pose_drops}, {"recovery_travel_m", recovery_travel},
        {"recovery_two_axis_fixes", wall_fixes}, {"t265_translation_rebase_m", bias}, {"policy", policy()},
        {"wheel_motion_suspect", wheel_suspect}, {"wall_motion_evidence", wall_evidence},
        {"wheel_input_policy", {{"pause_requested", wheel_suspect},
            {"reason", wheel_suspect ? "two_axis_wall_motion_wheel_suspect" : "measured_wheel_input_enabled"},
            {"dependency_persists_after_pause", true}}},
        {"wheel_and_gyro_are_sdk_dependencies", true}, {"independent_ground_truth", false}};
    if (!include_evidence) result.erase("wall_motion_evidence");
    return result;
}

} // namespace omni
