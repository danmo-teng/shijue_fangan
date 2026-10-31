"""Fixed field-yaw wall geometry, for display/logging only.

Reference: fusion/fence_adapter.py _assign / fixed_yaw least-squares / enclosure
checks. No invented covariance, geometry qualification, control pose or yaw fit.
The legacy source clock is unqualified: rotation scans are never position fixes.
"""
from collections import deque
import math
import numpy as np

from f407_motion_protocol import imu_is_fresh
from .static import transform, extract_lines, outer_supported_lines


def load_config(path):
    import json
    config = json.loads(path.read_text())
    if config.get("schema_version") != 1:
        raise ValueError("unsupported diagnostic configuration")
    q = config["qualification"]
    expected = {"timestamp_semantics_verified","clock_calibrated","error_model_qualified",
                "rotation_deskew_qualified","translation_compensated","control_pose_publish_enabled"}
    if set(q)!=expected or any(v is not False for v in q.values()) or config["packet_timing"].get("timestamp_semantics_verified") is not False:
        raise ValueError("this diagnostic entry cannot promote calibration or publish control poses")
    if config["installation_confirmation"].get("unchanged_confirmed") is not True:
        raise ValueError("confirm actual mounting before using recorded extrinsics")
    m = config["installation"]
    numbers = [*m["extrinsic"], m["center_raw_deg"], m["fov_deg"], m["display_fov_deg"]]
    if not all(math.isfinite(x) for x in numbers) or not 0 < m["fov_deg"] <= m["display_fov_deg"] < 360:
        raise ValueError("invalid mounting geometry")
    if config["lidar"]["bearing_sign"] != 1 or config["lidar"]["physical_direction"] != 1:
        raise ValueError("recorded inverted mounting requires original positive raw scan direction")
    if config["map"]["arena_size_m"] != [3.,3.] or config["map"]["arena_origin_in_competition_m"] != [-1.5,-1.5]:
        raise ValueError("review map conversion separately; this entry uses fixed 3m centre frame")
    if config["map"]["corner_to_center_translation_m"] != [-1.5,-1.5]:
        raise ValueError("inconsistent corner-to-competition conversion")
    limits=config["limits"]
    if (not 0<limits["max_correction_m"]<=.10 or not 0<limits["max_position_residual_m"]<=.035 or
        not 0<limits["yaw_receive_timeout_ms"]<=150 or limits["still_window_ms"]<500 or
        not 0<limits["still_max_rate_deg_s"]<=1 or not 0<limits["still_yaw_span_deg"]<=1):
        raise ValueError("diagnostic admission limits cannot loosen reviewed static gates")
    return config


def fixed_yaw_position(points_lidar, prior_center_xy, yaw_rad, extrinsic, *,
                       max_correction_m=.10, max_residual_m=.035):
    """Nearby, operator-resolved square branch only. Unobserved XY stays a prior.

    Extract/envelope/associate as in the reviewed patch. Solve ONLY XY, never
    rotate the main gyro to make lidar residuals look better. No accuracy claim.
    """
    points = transform(np.asarray(points_lidar).reshape(-1,2), extrinsic)
    prior = np.array([prior_center_xy[0]+1.5, prior_center_xy[1]+1.5, yaw_rad],float)
    lines = outer_supported_lines(extract_lines(points))
    normals = np.array([[1.,0.],[1.,0.],[0.,1.],[0.,1.]])
    distances = np.array([0.,3.,0.,3.])
    assigned = []
    for line in lines:
        wp = transform(line["points"],prior)
        normal = transform([line["normal"]],[0,0,yaw_rad])[0]
        costs = np.median(abs(wp@normals.T-distances),axis=0)
        costs[abs(normals@normal) <= math.cos(math.radians(12))] = np.inf
        wid = int(np.argmin(costs))
        tangent = 1 if wid < 2 else 0
        support = np.mean((wp[:,tangent]>=-.05)&(wp[:,tangent]<=3.05))
        if costs[wid] < .20 and support > .85:
            assigned.append((wid,line["points"]))
    result = dict(position_observed_axes=[], rank_xy=0, candidate_center_xy_m=None,
                  inherited_axes=["x","y"], covariance=None, qualified_pose=False,
                  yaw_rad=yaw_rad, supported_lines=len(lines), wall_ids=[])
    if not assigned:
        return dict(result,reason="NO_SUPPORTED_WALL_AT_OPERATOR_BRANCH")
    ids = np.concatenate([np.full(len(p),wid,dtype=int) for wid,p in assigned])
    support = np.concatenate([p for _,p in assigned])
    ns = normals[ids]
    pose = prior.copy()
    for _ in range(15):
        residual = np.sum(transform(support,pose)*ns,axis=1)-distances[ids]
        weights = np.minimum(1.,.02/np.maximum(abs(residual),1e-12))
        delta = np.linalg.lstsq(ns*np.sqrt(weights[:,None]),-residual*np.sqrt(weights),rcond=1e-5)[0]
        pose[:2] += delta
        if np.linalg.norm(delta) < 1e-8:
            break
    residual = np.sum(transform(support,pose)*ns,axis=1)-distances[ids]
    rms = float(np.sqrt(np.mean(residual**2)))
    result.update(residual_rms_m=rms,wall_ids=sorted(set(ids.tolist())),
                  correction_m=float(np.linalg.norm(pose[:2]-prior[:2])))
    if (rms>max_residual_m or result["correction_m"]>max_correction_m or
            np.any(pose[:2]<0) or np.any(pose[:2]>3)):
        return dict(result,reason="RESIDUAL_BOUNDS_OR_LOCAL_CORRECTION_GATE")
    for line in lines:
        wp = transform(line["points"],pose)
        outside = np.any((wp<-.06)|(wp>3.06),axis=1)
        if outside.sum() >= 6 and outside.mean() > .25:
            return dict(result,reason="ENCLOSING_ARENA_CONTRADICTION")
    observed = [axis for i,axis in enumerate(("x","y")) if np.any(ns[:,i] != 0)]
    result.update(position_observed_axes=observed,rank_xy=len(observed),
        inherited_axes=[a for a in ("x","y") if a not in observed],
        candidate_center_xy_m=(pose[:2]-1.5).tolist(),
        reason="STATIC_GEOMETRY_DIAGNOSTIC_ONLY")
    return result


class FieldYaw:
    """Local cumulative 0x1F yaw + one operator-measured fixed field offset.

    No magnetic north, no automatic boot-zero alignment, no timestamp invention.
    Invalidity/discontinuity drops the reference; explicit static re-alignment is
    possible, without any chassis fault latch or gyro direction change.
    """
    def __init__(self, limits=None):
        self.limits = limits or {}
        self.history = deque(maxlen=512)
        self.offset_rad = None
        self.reference = None
        self.revision = 0
        self.reason = "FIELD_REFERENCE_REQUIRED"

    def invalidate(self, reason):
        self.offset_rad = None
        self.reference = None
        self.history.clear()
        self.reason = reason
        self.revision += 1

    def ingest(self, sample):
        if not sample.get("wire_valid"):
            self.invalidate("INVALID_MAIN_YAW_REQUIRES_REFERENCE_RECHECK")
            return
        previous = self.history[-1] if self.history else None
        if previous:
            dt = (sample["timestamp_monotonic_ns"]-previous["timestamp_monotonic_ns"])/1e9
            # Multiple packets in one RX batch have no independently usable times.
            if dt == 0:
                if abs(sample["yaw_deg"]-previous["yaw_deg"])>8:
                    self.invalidate("POSSIBLE_YAW_RESET_WITHIN_RX_BATCH")
                    self.history.append(sample)
                    return
                else:
                    self.history[-1] = sample
                    return
            delta = abs(sample["yaw_deg"]-previous["yaw_deg"])
            if dt < 0 or dt > .30 or delta > 400*dt+2:
                self.invalidate("YAW_RECEIVE_GAP_OR_POSSIBLE_RESET_RECHECK_REFERENCE")
        self.history.append(sample)

    def fresh(self, now):
        return bool(self.history and imu_is_fresh(self.history[-1],now) and
                    now-self.history[-1]["timestamp_monotonic_ns"] <=
                    self.limits.get("yaw_receive_timeout_ms",150)*1_000_000)

    def stable(self, now):
        if not self.fresh(now):
            return False
        duration = self.limits.get("still_window_ms",500)*1_000_000
        samples = [s for s in self.history if 0 <= now-s["timestamp_monotonic_ns"] <= duration+150_000_000]
        if len(samples)<3 or samples[-1]["timestamp_monotonic_ns"]-samples[0]["timestamp_monotonic_ns"]<duration:
            return False
        return (max(abs(s["yaw_rate_deg_s"]) for s in samples)<=self.limits.get("still_max_rate_deg_s",1.) and
                max(s["yaw_deg"] for s in samples)-min(s["yaw_deg"] for s in samples)<=self.limits.get("still_yaw_span_deg",1.) and
                all(b["timestamp_monotonic_ns"]-a["timestamp_monotonic_ns"] <= 150_000_000
                    for a,b in zip(samples,samples[1:])))

    def align(self, field_deg, description, now):
        if not math.isfinite(field_deg) or not description.strip() or not self.stable(now):
            raise ValueError("need fresh stable gyro and measured field angle/reference description")
        samples = [s for s in self.history if now-s["timestamp_monotonic_ns"]<=500_000_000]
        raw_deg = float(np.median([s["yaw_deg"] for s in samples]))
        self.offset_rad = math.radians(field_deg-raw_deg)
        self.revision += 1
        self.reference = dict(field_deg=field_deg,raw_local_yaw_deg=raw_deg,
            offset_rad=self.offset_rad,description=description,timestamp_monotonic_ns=now,
            reference_revision=self.revision,source="operator_ground_angle_reference",
            mcu_epoch=None,capture_time_qualified=False,accuracy_calibrated=False)
        self.reason = "OPERATOR_FIELD_REFERENCE_SET"
        return self.reference

    def field_rad(self, now):
        if self.offset_rad is None or not self.fresh(now):
            return None
        return math.radians(self.history[-1]["yaw_deg"])+self.offset_rad


class DiagnosticState:
    def __init__(self, config):
        self.config = config
        self.yaw = FieldYaw(config.get("limits"))
        self.wheels = deque(maxlen=256)
        self.operator_stationary = False
        self.branch = None
        self.prior = None
        self.phase = "static"
        self.static_review = None
        self.latest = None
        self.static_history = deque(maxlen=8)

    def note_wheels(self, sample):
        if sample.get("status")!=7:
            self.wheels.clear()
            return
        if self.wheels and sample["timestamp_monotonic_ns"]==self.wheels[-1]["timestamp_monotonic_ns"]:
            # Do not hide motion within a receive batch by replacing its first count.
            if any(sample[f"m{i}_count"]!=self.wheels[-1][f"m{i}_count"] for i in (1,2,3)):
                self.wheels.clear()
            else:
                self.wheels[-1]=sample
                return
        self.wheels.append(sample)

    def wheel_quiet(self, now):
        samples=[s for s in self.wheels if 0<=now-s["timestamp_monotonic_ns"]<=650_000_000]
        return (len(samples)>=3 and now-samples[-1]["timestamp_monotonic_ns"]<=150_000_000 and
            samples[-1]["timestamp_monotonic_ns"]-samples[0]["timestamp_monotonic_ns"]>=500_000_000 and
            all(b["timestamp_monotonic_ns"]-a["timestamp_monotonic_ns"]<=150_000_000 for a,b in zip(samples,samples[1:])) and
            all(all(s[f"m{i}_count"]==samples[0][f"m{i}_count"] for i in (1,2,3)) for s in samples))

    def seed(self, x, y, description):
        if not all(math.isfinite(v) and -1.5<=v<=1.5 for v in (x,y)) or not description.strip():
            raise ValueError("centre-frame XY within +/-1.5m and physical branch description required")
        self.prior = [x,y]
        self.branch = dict(initial_center_xy_m=[x,y],description=description,
                           source="operator_local_branch_seed_not_lidar_global_identity")
        self.static_review = None
        self.static_history.clear()

    def observe(self, points, now, scan_info):
        yaw = self.yaw.field_rad(now)
        result = dict(status="UNREFERENCED_DIAGNOSTIC",qualified_pose=False,
            full_pose_valid=False,control_pose_published=False,translation_compensated=False,
            rotation_compensated=False,field_yaw_rad=yaw,position_observed_axes=[],rank_xy=0,
            scan=scan_info,phase=self.phase,yaw_reference=self.yaw.reference)
        if self.prior is None or yaw is None:
            result["reason"] = "SET_PHYSICAL_MAP_BRANCH_AND_FIELD_YAW_REFERENCE"
        elif self.phase == "rotate" or not self.operator_stationary or not self.yaw.stable(now) or not self.wheel_quiet(now):
            result.update(status="ROTATION_OR_MOTION_CHAIN_ONLY",reason="RAW_MOVING_SCAN_NOT_A_POSITION_FIX",
                          display_prior_center_xy_m=list(self.prior))
        else:
            m = self.config["installation"]
            ext = [*m["extrinsic"][:2], math.radians(m["extrinsic"][2])]
            limits=self.config["limits"]
            fit = fixed_yaw_position(points,self.prior,yaw,ext,max_correction_m=limits["max_correction_m"],
                                     max_residual_m=limits["max_position_residual_m"])
            result.update(fit)
            result["status"] = "STATIC_XY_DIAGNOSTIC" if fit["rank_xy"]==2 else "STATIC_PARTIAL_DIAGNOSTIC"
            if fit["rank_xy"]:
                self.prior = list(fit["candidate_center_xy_m"])
                self.static_history.append((now,fit["rank_xy"],self.yaw.revision,list(self.prior)))
            else:
                self.static_history.clear()
        self.latest = result
        return result

    def review_static(self, description, now):
        h = self.static_history
        if (not description.strip() or not self.operator_stationary or not self.yaw.stable(now) or not self.wheel_quiet(now) or
            len(h)<5 or now-h[-1][0]>300_000_000 or h[-1][0]-h[0][0]<400_000_000 or
            any(b[0]-a[0]>200_000_000 for a,b in zip(h,list(h)[1:])) or
            any(rank!=2 or rev!=self.yaw.revision for _,rank,rev,_ in h) or
            np.max(np.ptp([xy for _,_,_,xy in h],axis=0))>.03):
            raise ValueError("need continuous static two-axis geometry + ground review before rotation phase")
        self.static_review = dict(timestamp_monotonic_ns=now,description=description,
            yaw_reference_revision=self.yaw.revision,accuracy_verified=False,
            meaning="operator-reviewed diagnostic static checkpoint, not qualification")
        return self.static_review

    def enter_rotation(self, now):
        if (not self.static_review or self.yaw.field_rad(now) is None or
                self.static_review["yaw_reference_revision"] != self.yaw.revision):
            raise ValueError("first complete static review with the current field reference")
        self.phase = "rotate"
        self.operator_stationary = False
