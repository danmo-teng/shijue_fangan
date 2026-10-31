"""Debug-only measured-wheel -> tracking-centre input; never owns USB/UART.

Current F405 0x15/0x1F have NO capture clock/epoch: they are logged and rejected.
The timestamped events below are a host-side contract, NOT invented wire opcodes.
An audited future chassis decoder must qualify its clock mapping before emitting
them. No target speeds, T265 velocity, or encoder-derived omega are accepted.
"""
from collections import deque
import json
import math
import queue
import socket
import time

MAX_AGE_NS = 50_000_000
MAX_UNCERTAINTY_NS = 5_000_000
MAX_GYRO_GAP_NS = 20_000_000
DEPENDENCIES = ["measured_wheel_feedback", "primary_gyro_rate"]


def camera_velocity(v_body, omega_body, forward, up, offset):
    """v_C^C = R_CB (v_B^B + omega_B^B x r_BC^B), all SI units."""
    if any(len(v) != 3 for v in (v_body, omega_body, forward, up, offset)):
        raise ValueError("three components required")
    def cross(a, b):
        return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    def unit(v):
        n = math.sqrt(sum(x*x for x in v))
        if not math.isfinite(n) or n < 1e-9:
            raise ValueError("invalid body axis")
        return [x/n for x in v]
    forward, up = unit(forward), unit(up)
    if abs(sum(a*b for a, b in zip(forward, up))) > 1e-6:
        raise ValueError("body axes must be orthogonal")
    left = cross(up, forward)
    lever = cross(omega_body, offset)
    result = [sum(axis[j]*(v_body[i]+lever[i]) for i, axis in enumerate((forward, left, up)))
              for j in range(3)]
    if not all(math.isfinite(x) for x in (*v_body, *omega_body, *offset, *result)):
        raise ValueError("nonfinite measured velocity")
    return lever, result


def write_sdk_config(path, noise_std_mps):
    if not math.isfinite(noise_std_mps) or noise_std_mps <= 0:
        raise ValueError("positive velocity noise standard deviation required")
    config = {"velocimeters": [{
        "scale_and_alignment": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        "noise_variance": noise_std_mps**2,
        # Input already at C, in C axes. NO second mounting/IMU compensation.
        "extrinsics": {"T": [0, 0, 0], "W": [0, 0, 0],
                       "T_variance": [0, 0, 0], "W_variance": [0, 0, 0]}}]}
    path.write_text(json.dumps(config, indent=2)+"\n", encoding="utf-8")
    return config


class WheelInputBridge:
    """Bounded subscription consumer. Only the existing C++ SDK owner sends.

    Qualified measured_body_velocity: source=measured_wheel_feedback, v_body_mps,
    interval_start_ns/end_ns, receive_monotonic_ns, time_uncertainty_ns,
    capture_time_qualified, measurement_valid, mcu_epoch, sample_id.
    Qualified imu: angular_rate_capture_ns (NOT yaw capture), yaw_rate_deg_s,
    receive_monotonic_ns, time_uncertainty_ns, capture_time_qualified,
    mcu_epoch, wire_valid. All capture times mapped to host CLOCK_MONOTONIC.
    """
    def __init__(self, root, config, session_id, enabled, *, clock=time.monotonic_ns):
        self.enabled, self.session_id, self.clock = enabled, session_id, clock
        self.address = "\0t265-wheel-"+session_id
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.log = (root/"wheel_input.jsonl").open("a", encoding="utf-8", buffering=1)
        self.inbox = queue.Queue(maxsize=256)
        self.overflow = False
        self.gyro = deque(maxlen=128)
        self.pending = deque()
        self.seen = set()
        self.last_end = None
        self.epoch = None
        self.next_id = 0
        self.context = {}
        self.stats = dict(sent_to_sdk_owner=0, rejected=0, last_reason="waiting_for_measured_feedback")
        self.forward = self._axis(config["camera_robot_forward_axis"])
        self.up = self._axis(config["camera_robot_up_axis"])
        self.offset = [float(config["camera_offset_forward_m"]), float(config["camera_offset_left_m"]), 0.]

    @staticmethod
    def _axis(text):
        if text in ("+x", "x", "-x", "+y", "y", "-y", "+z", "z", "-z"):
            result = [0., 0., 0.]
            result["xyz".index(text[-1])] = -1. if text.startswith("-") else 1.
            return result
        return [float(x) for x in text.split(",")]

    def telemetry(self, event):
        if event["kind"] not in ("imu", "host_wheel_recomputed", "measured_body_velocity"):
            return
        try:
            self.inbox.put_nowait(event)
        except queue.Full:
            self.overflow = True

    def _record(self, reason, sample=None, **extra):
        self.stats["last_reason"] = reason
        if reason != "sent_to_sdk_owner":
            self.stats["rejected"] += 1
        converted = extra.get("converted", {})
        body = (sample or {}).get("v_body_mps")
        if body is None and (sample or {}).get("forward_mps") is not None and sample.get("left_mps") is not None:
            body = [sample["forward_mps"], sample["left_mps"], 0.]
        self.log.write(json.dumps(dict(session_id=self.session_id, reason=reason,
            timestamp_monotonic_ns=self.clock(), sdk_result=None,
            action_context=self.context,
            sdk_call_attempted=None if reason == "sent_to_sdk_owner" else False,
            v_body_mps=body, omega_body_rad_s=converted.get("omega_body_rad_s"),
            lever_body_mps=converted.get("lever_body_mps"), v_camera_mps=converted.get("v_camera_mps"),
            gyro_midpoint_delta_ns=converted.get("gyro_midpoint_delta_ns"),
            # Transport success is not SDK acceptance or localisation accuracy.
            sample=sample, **extra), allow_nan=False)+"\n")

    def _qualified(self, sample):
        return (sample.get("capture_time_qualified") is True and sample.get("mcu_epoch") is not None
            and isinstance(sample.get("time_uncertainty_ns"), int)
            and 0 <= sample["time_uncertainty_ns"] <= MAX_UNCERTAINTY_NS)

    def poll(self, context=None):
        self.context = dict(context or {})
        if self.overflow:
            self.overflow = False
            self.pending.clear()
            self.gyro.clear()
            while not self.inbox.empty():
                self.inbox.get_nowait()
            self._record("subscriber_overflow_discarded_no_replay")
        while not self.inbox.empty():
            event = self.inbox.get_nowait()
            sample = dict(event["sample"], host_chassis_session_id=event.get("host_session_id"),
                          host_reference_epoch=event.get("host_reference_epoch"))
            if event["kind"] == "imu":
                if self._qualified(sample) and sample.get("wire_valid") is True:
                    self.gyro.append(sample)
                else:
                    # Invalid rate during an interval must not be interpolated through.
                    self.gyro.clear()
                continue
            if event["kind"] == "host_wheel_recomputed":
                self._record("unqualified_legacy_capture_time_and_epoch", sample,
                             injection_requested=self.enabled)
                continue
            if len(self.pending) >= 64:
                self._record("wheel_queue_overflow", self.pending.popleft())
            self.pending.append(sample)
        while self.pending:
            sample = self.pending[0]
            try:
                reason, packet = self._prepare(sample)
            except (ValueError, TypeError, KeyError, OverflowError):
                reason, packet = "malformed_measured_feedback", None
            if reason == "waiting_for_bracketing_gyro":
                break
            self.pending.popleft()
            if reason:
                self._record(reason, sample)
                continue
            if not self.enabled:
                self._record("injection_off", sample, converted=packet)
                continue
            try:
                self.sock.sendto(json.dumps(packet, allow_nan=False).encode(), self.address)
                self.stats["sent_to_sdk_owner"] += 1
                self._record("sent_to_sdk_owner", sample, converted=packet)
            except OSError as error:
                self._record("sdk_owner_unavailable_no_replay", sample, error=str(error), converted=packet)

    def _prepare(self, sample):
        now = self.clock()
        if not self._qualified(sample):
            return "unqualified_capture_time_or_epoch", None
        if sample.get("measurement_valid") is not True or sample.get("source") != "measured_wheel_feedback":
            return "not_valid_measured_wheel_feedback", None
        start, end = sample["interval_start_ns"], sample["interval_end_ns"]
        uncertainty = sample["time_uncertainty_ns"]
        if not (isinstance(start, int) and isinstance(end, int) and 0 < end-start <= MAX_AGE_NS):
            return "invalid_wheel_interval", None
        if not 0 <= now-start <= MAX_AGE_NS-uncertainty:
            return "stale_or_future_wheel_interval", None
        if not end <= sample["receive_monotonic_ns"] <= now:
            return "invalid_wheel_receive_time", None
        epoch = sample["mcu_epoch"]
        if self.epoch is None:
            self.epoch = epoch
        if epoch != self.epoch:
            return "epoch_changed_new_sdk_session_required", None
        identity = (str(epoch), sample["sample_id"])
        if identity in self.seen or (self.last_end is not None and start < self.last_end):
            return "duplicate_or_out_of_order_wheel_interval", None
        rates = [s for s in self.gyro if s["mcu_epoch"] == epoch]
        times = [s["angular_rate_capture_ns"] for s in rates]
        if any(b <= a for a, b in zip(times, times[1:])):
            return "nonmonotonic_gyro_capture_time", None
        if not times or times[-1] < end:
            return "waiting_for_bracketing_gyro", None
        before = [i for i, t in enumerate(times) if t <= start]
        if not before:
            return "no_gyro_at_interval_start", None
        first = before[-1]
        last = next(i for i, t in enumerate(times) if t >= end)
        selected = rates[first:last+1]
        if any(b["angular_rate_capture_ns"]-a["angular_rate_capture_ns"] > MAX_GYRO_GAP_NS
               for a, b in zip(selected, selected[1:])):
            return "gyro_capture_gap", None
        if any(not s["angular_rate_capture_ns"] <= s["receive_monotonic_ns"] <= now
               or not 0 <= now-s["angular_rate_capture_ns"] <= MAX_AGE_NS-s["time_uncertainty_ns"]
               for s in selected):
            return "stale_or_invalid_gyro_time", None
        integral = 0.
        for a, b in zip(selected, selected[1:]):
            ta, tb = a["angular_rate_capture_ns"], b["angular_rate_capture_ns"]
            lo, hi = max(start, ta), min(end, tb)
            va, vb = a["yaw_rate_deg_s"], b["yaw_rate_deg_s"]
            vlo = va+(vb-va)*(lo-ta)/(tb-ta)
            vhi = va+(vb-va)*(hi-ta)/(tb-ta)
            integral += (vlo+vhi)*.5*(hi-lo)
        omega = [0., 0., math.radians(integral/(end-start))]
        v_body = sample["v_body_mps"]
        if len(v_body) != 3 or abs(v_body[2]) > 1e-9 or math.hypot(*v_body[:2]) > 1.5 or abs(omega[2]) > 8.:
            return "implausible_planar_measured_velocity", None
        lever, v_camera = camera_velocity(v_body, omega, self.forward, self.up, self.offset)
        self.seen.add(identity)
        # last_end also excludes old intervals; keep memory bounded.
        if len(self.seen) > 512:
            self.seen = {identity}
        self.last_end = end
        packet = dict(schema=1, session_id=self.session_id, packet_id=self.next_id,
            mcu_epoch=epoch, sample_id=sample["sample_id"], capture_time_qualified=True,
            source="measured_wheel_feedback", measurement_valid=True, interval_start_ns=start, interval_end_ns=end,
            receive_monotonic_ns=sample["receive_monotonic_ns"],
            time_uncertainty_ns=max([uncertainty]+[s["time_uncertainty_ns"] for s in selected]),
            alignment_uncertainty_ns=uncertainty+max(s["time_uncertainty_ns"] for s in selected),
            gyro_start_ns=selected[0]["angular_rate_capture_ns"],
            gyro_end_ns=selected[-1]["angular_rate_capture_ns"],
            gyro_receive_ns=max(s["receive_monotonic_ns"] for s in selected),
            wheel_midpoint_ns=(start+end)//2,
            gyro_midpoint_delta_ns=(times[first]+times[last]-start-end)//2,
            gyro_samples=selected, v_body_mps=v_body, omega_body_rad_s=omega,
            action_context=self.context,
            lever_body_mps=lever, v_camera_mps=v_camera, dependency_sources=DEPENDENCIES)
        self.next_id += 1
        return None, packet

    def close(self):
        self.sock.close()
        self.log.close()
