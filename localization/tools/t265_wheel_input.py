"""Debug-only measured-wheel -> tracking-centre input; never owns USB/UART.

Current F405 0x15/0x1F remain count/gyro diagnostics; measured 07/08 provide
the actual wheel interval and rate events on one MCU clock. Timing quality
is recorded without gating adoption. No target speeds, T265 velocity, or
encoder-derived omega are accepted.
"""
from collections import deque
import json
import math
import queue
import socket
import time
import threading
from async_debug_log import AsyncDebugLog
from f405_timed_measurements import TimedMeasurements

TIME_POLICY = "received_feedback_no_age_gate_v1"
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

    Measured body velocity: source=measured_wheel_feedback, v_body_mps,
    alignment_start_ns/end_ns/clock, measurement_valid, mcu_epoch, sample_id.
    Gyro has its own alignment_capture_ns, yaw_rate_deg_s and wire_valid.
    The versioned path aligns original MCU events; mapped host capture times
    and their uncertainty may be null. Qualification is diagnostic only.
    """
    def __init__(self, root, config, session_id, enabled, *, clock=time.monotonic_ns):
        self.enabled, self.session_id, self.clock = enabled, session_id, clock
        self.address = "\0t265-wheel-"+session_id
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.log=AsyncDebugLog(root)
        self.inbox = queue.Queue(maxsize=256)
        self.overflow = False
        self.gyro = deque(maxlen=128)
        self.pending = deque()
        self.seen = set()
        self.last_end = None
        self.epoch = None
        self.next_id = 0
        self.context = {}
        self.lock=threading.RLock();self.wake=threading.Event();self.stop_event=threading.Event();self.thread=None
        self.measurements=TimedMeasurements();self.timed_mode=False
        self.stream_state=None
        self.latencies=deque(maxlen=1024)
        self.last_processed_ns=None;self.last_sample_age_ns=None;self.last_input_rejection=None
        self.last_sample_start_ns=None;self.worker_error=None
        self.last_qualification_reasons=[]
        self.last_gyro_reasons=[];self.latest_measured=None
        self.stats = dict(sent_to_sdk_owner=0, rejected=0, subscriber_dropped=0, overflow_discarded=0,
                          last_reason="waiting_for_measured_feedback")
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
        if event["kind"] not in ("imu", "host_wheel_recomputed", "measured_body_velocity","timed_identity",
            "timed_clock_sync","timed_stream_state","timed_wheel_diagnostic","timed_gyro_diagnostic"):
            return
        try:
            self.inbox.put_nowait(event)
            self.wake.set()
        except queue.Full:
            self.overflow = True
            self.stats["subscriber_dropped"]+=1

    def _record(self, reason, sample=None, **extra):
        self.stats["last_reason"] = reason
        if reason != "sent_to_sdk_owner":
            self.stats["rejected"] += 1
        converted = extra.get("converted", {})
        body = (sample or {}).get("v_body_mps")
        if body is None:body=converted.get("v_body_mps")
        if body is None and (sample or {}).get("forward_mps") is not None and sample.get("left_mps") is not None:
            body = [sample["forward_mps"], sample["left_mps"], 0.]
        document=dict(session_id=self.session_id, reason=reason,
            timestamp_monotonic_ns=self.clock(), sdk_result=None,
            action_context=self.context,
            sdk_call_attempted=None if reason == "sent_to_sdk_owner" else False,
            v_body_mps=body, omega_body_rad_s=converted.get("omega_body_rad_s"),
            lever_body_mps=converted.get("lever_body_mps"), v_camera_mps=converted.get("v_camera_mps"),
            gyro_midpoint_delta_ns=converted.get("gyro_midpoint_delta_ns"),
            # Transport success is not SDK acceptance or localisation accuracy.
            sample=sample, **extra)
        self.log.submit("wheel_input.jsonl",document)
        if reason=="sent_to_sdk_owner":self.last_input_rejection=None
        elif self.timed_mode is False or reason!="legacy_counts_reference_only":
            self.last_input_rejection=reason

    def start(self):
        if self.thread is not None:raise RuntimeError("wheel input worker already started")
        self.thread=threading.Thread(target=self._run,name="t265-wheel-input-worker",daemon=True);self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            self.wake.wait(.005);self.wake.clear()
            try:self.poll()
            except (ValueError,TypeError,KeyError,OverflowError) as error:
                with self.lock:
                    self.worker_error=str(error);self.pending.clear();self.gyro.clear()
                    self._record("measurement_worker_rejected_malformed_input",error=str(error))

    def set_context(self, context):
        with self.lock:self.context=dict(context)

    def snapshot(self, sdk=None):
        with self.lock:
            now=self.clock();sdk=sdk or {}
            if sdk.get("session_id")!=self.session_id:sdk={}
            loaded=sdk.get("config_loaded",False)
            successes=sdk.get("sdk_api_success_samples",0)
            stage="NOT_REQUESTED" if not self.enabled else ("SDK_API_ACCEPTED" if successes else
                  "SDK_CONFIG_LOADED" if loaded else "REQUESTED_BLOCKED")
            accepted_ns=sdk.get("last_sdk_success_ns")
            status_ns=sdk.get("status_monotonic_ns")
            status_fresh=isinstance(status_ns,int) and 0<=now-status_ns<=500_000_000
            measured=dict(self.latest_measured) if self.latest_measured else None
            if measured:measured["receive_age_ms"]=(now-measured["receive_monotonic_ns"])/1e6
            latency=sorted(self.latencies)
            return dict(requested=self.enabled,state=stage,time_policy=TIME_POLICY,config_loaded=loaded,sdk_api_success_samples=successes,
                sdk_api_rate_hz=sdk.get("sdk_api_rate_hz",0) if status_fresh else 0,
                sdk_status_fresh=status_fresh,last_sdk_success_ns=accepted_ns,
                sdk_success_age_ms=None if not accepted_ns else (now-accepted_ns)/1e6,
                last_input_rejection=self.last_input_rejection,clock=self.measurements.clock_status(now),
                qualification_missing=list(dict.fromkeys(self.last_qualification_reasons+self.last_gyro_reasons)),
                stream=self.stream_state,latest_measured=measured,
                last_sample_age_ms=None if self.last_sample_start_ns is None else (now-self.last_sample_start_ns)/1e6,
                sample_age_basis="mapped source event; unqualified when source bounds unknown",worker_error=self.worker_error,
                processing_latency_ms_max=max(self.latencies,default=0)/1e6,
                processing_latency_ms_p95=latency[int((len(latency)-1)*.95)]/1e6 if latency else None,
                processing_latency_ms_median=sorted(self.latencies)[len(self.latencies)//2]/1e6 if self.latencies else None,
                logging_healthy=self.log.healthy(),sdk_logging_healthy=sdk.get("logging_healthy"),
                stats=dict(self.stats),measurement_counts=dict(self.measurements.stats))

    def _usable_rate(self, sample):
        capture=sample.get("alignment_capture_ns",sample.get("angular_rate_capture_ns"))
        return (sample.get("mcu_epoch") is not None and sample.get("wire_valid") is True
                and isinstance(capture,int) and capture>=0
                and math.isfinite(sample.get("yaw_rate_deg_s",float("nan"))))

    def poll(self, context=None):
        with self.lock:
            self._poll(context)

    def _poll(self, context):
        if context is not None:self.context=dict(context)
        if self.overflow:
            self.overflow = False
            self.pending.clear()
            self.gyro.clear()
            discarded=0
            while not self.inbox.empty():
                self.inbox.get_nowait()
                discarded+=1
            self.stats["overflow_discarded"]+=discarded
            self._record("subscriber_overflow_discarded_no_replay",discarded_queued_events=discarded)
        for _ in range(256):
            if self.inbox.empty():break
            event = self.inbox.get_nowait()
            sample = dict(event["sample"], host_chassis_session_id=event.get("host_session_id"),
                          host_reference_epoch=event.get("host_reference_epoch"),
                          host_action_id_at_receive=event.get("host_action_id_at_receive"))
            now=self.clock()
            received=sample.get("receive_monotonic_ns",sample.get("timestamp_monotonic_ns"))
            if received is not None:self.latencies.append(max(0,now-received))
            self.last_processed_ns=now
            kind=event["kind"]
            if kind=="timed_identity":
                self.timed_mode=True;self.measurements.set_identity(sample);continue
            if kind=="timed_clock_sync":
                self.measurements.sync(sample);continue
            if kind=="timed_stream_state":
                self.stream_state=sample
                self.log.submit("wheel_measurements.jsonl",dict(event="stream_state",sample=sample))
                if sample["status"]!=0:self.last_input_rejection=f"measurement_stream_rejected_status_{sample['status']}"
                continue
            if kind in ("timed_wheel_diagnostic","timed_gyro_diagnostic"):
                self.timed_mode=True
                sample=self.measurements.wheel(sample,now) if kind=="timed_wheel_diagnostic" else self.measurements.gyro(sample,now)
                if sample is None:continue
                if kind=="timed_gyro_diagnostic":
                    self.last_gyro_reasons=sample.get("qualification_reasons",[])
                    self.log.submit("wheel_measurements.jsonl",sample)
                    if self._usable_rate(sample):self.gyro.append(sample)
                    else:self.gyro.clear()
                    continue
                self.log.submit("wheel_measurements.jsonl",sample)
                self.latest_measured={k:sample.get(k) for k in ("v_body_mps","host_recomputed_v_body_mps",
                    "receive_monotonic_ns","measurement_valid","capture_time_qualified")}
                kind="measured_body_velocity"
            if event["kind"] == "imu":
                if self.timed_mode:continue  # Legacy unqualified IMU must not erase valid timed-rate brackets.
                if self._usable_rate(sample):
                    self.gyro.append(sample)
                else:
                    # Invalid rate during an interval must not be interpolated through.
                    self.gyro.clear()
                continue
            if kind == "host_wheel_recomputed":
                self._record("legacy_counts_reference_only", sample,
                             injection_requested=self.enabled)
                continue
            if len(self.pending) >= 64:
                self._record("wheel_queue_overflow", self.pending.popleft())
            self.pending.append(sample)
        while self.pending:
            sample = self.pending[0]
            self.last_qualification_reasons=sample.get("qualification_reasons",[])
            start=sample.get("interval_start_ns")
            self.last_sample_start_ns=start
            self.last_sample_age_ns=None if start is None else self.clock()-start
            try:
                reason, packet = self._prepare(sample)
            except (ValueError, TypeError, KeyError, OverflowError):
                reason, packet = "malformed_measured_feedback", None
            if reason == "waiting_for_bracketing_gyro":
                self.last_input_rejection=reason
                break
            self.pending.popleft()
            if reason:
                self._record(reason, sample,qualification_reasons=sample.get("qualification_reasons",[]))
                continue
            if not self.enabled:
                self._record("injection_off", sample, converted=packet)
                continue
            # Log completeness is reported separately, never an input gate.
            try:
                self.sock.sendto(json.dumps(packet, allow_nan=False).encode(), self.address)
                self.stats["sent_to_sdk_owner"] += 1
                self._record("sent_to_sdk_owner", sample, converted=packet)
            except OSError as error:
                self._record("sdk_owner_unavailable_no_replay", sample, error=str(error), converted=packet)

    def _prepare(self, sample):
        now = self.clock()
        if sample.get("mcu_epoch") is None:
            return "unknown_measurement_epoch", None
        if sample.get("measurement_valid") is not True or sample.get("source") != "measured_wheel_feedback":
            return "not_valid_measured_wheel_feedback", None
        clock=sample.get("alignment_clock","host_monotonic_event")
        start=sample.get("alignment_start_ns",sample.get("interval_start_ns"))
        end=sample.get("alignment_end_ns",sample.get("interval_end_ns"))
        if not (isinstance(start,int) and isinstance(end,int) and 0<=start<end):
            return "invalid_wheel_interval", None
        epoch = sample["mcu_epoch"]
        if self.epoch is None:
            self.epoch = epoch
        if epoch != self.epoch:
            return "epoch_changed_new_sdk_session_required", None
        identity = (str(epoch), sample["sample_id"])
        if identity in self.seen or (self.last_end is not None and start < self.last_end):
            return "duplicate_or_out_of_order_wheel_interval", None
        rates = [s for s in self.gyro if s["mcu_epoch"] == epoch and
                 s.get("alignment_clock","host_monotonic_event")==clock]
        times = [s.get("alignment_capture_ns",s.get("angular_rate_capture_ns")) for s in rates]
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
        integral = 0.
        for i,(a,b) in enumerate(zip(selected,selected[1:]),first):
            ta,tb=times[i],times[i+1]
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
        errors=[sample.get("time_uncertainty_ns")]+[s.get("time_uncertainty_ns") for s in selected]
        errors_known=all(isinstance(e,int) and e>=0 for e in errors)
        qualified=sample.get("capture_time_qualified") is True and all(s.get("capture_time_qualified") is True for s in selected)
        packet = dict(schema=2,time_policy=TIME_POLICY,session_id=self.session_id, packet_id=self.next_id,
            mcu_epoch=epoch, sample_id=sample["sample_id"], capture_time_qualified=qualified,
            source="measured_wheel_feedback", measurement_valid=True,
            alignment_clock=clock,alignment_start_ns=start,alignment_end_ns=end,
            interval_start_ns=sample.get("interval_start_ns"),interval_end_ns=sample.get("interval_end_ns"),
            receive_monotonic_ns=sample["receive_monotonic_ns"],
            time_uncertainty_ns=max(errors) if errors_known else None,
            alignment_uncertainty_ns=errors[0]+max(errors[1:]) if errors_known else None,
            alignment_gyro_start_ns=times[first],alignment_gyro_end_ns=times[last],
            gyro_receive_ns=max(s["receive_monotonic_ns"] for s in selected),
            wheel_midpoint_ns=(start+end)//2,
            gyro_midpoint_delta_ns=(times[first]+times[last]-start-end)//2,
            gyro_max_gap_ns=max((b-a for a,b in zip(times[first:last],times[first+1:last+1])),default=0),
            age_from_wheel_receive_ns=now-sample["receive_monotonic_ns"],
            gyro_samples=selected, v_body_mps=v_body, omega_body_rad_s=omega,
            action_context=self.context,
            host_action_id_at_receive=sample.get("host_action_id_at_receive"),
            lever_body_mps=lever, v_camera_mps=v_camera, dependency_sources=DEPENDENCIES)
        self.next_id += 1
        return None, packet

    def close(self):
        self.stop_event.set();self.wake.set()
        if self.thread is not None:self.thread.join(timeout=1)
        self.sock.close()
        self.log.close()


class SyntheticWheelInputBridge(WheelInputBridge):
    """Explicit desktop SDK-response experiment. No UART or measured sources.
    One bounded sequence, generated at <=50Hz without catch-up. Fake omega is
    used ONLY for the lever term; send_wheel_odometry receives velocity, not yaw.
    """
    PHASES=[("zero_before",2.,[0.,0.,0.],[0.,0.,0.])]
    for _name,_body,_omega in (
        ("forward",[.2,0,0],[0,0,0]),("backward",[-.2,0,0],[0,0,0]),
        ("left",[0,.2,0],[0,0,0]),("right",[0,-.2,0],[0,0,0]),
        ("ccw_lever_only",[0,0,0],[0,0,.5]),("cw_lever_only",[0,0,0],[0,0,-.5])):
        PHASES.extend([(_name,1.,_body,_omega),(_name+"_zero",.5,[0,0,0],[0,0,0])])
    PHASES.append(("zero_after",2.,[0.,0.,0.],[0.,0.,0.]))

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.fixture_start_ns=None;self.fixture_phase=None;self.done=threading.Event()
        self.log.submit("wheel_fixture.jsonl",dict(event="fixture_plan",session_id=self.session_id,
            source="synthetic_test_fixture",actual_measurement=False,motor_commands=False,
            phases=self.PHASES,noise_changed=False,clean_device_state_verified=False))

    def telemetry(self,event):
        raise RuntimeError("synthetic SDK experiment cannot subscribe to chassis measurements")

    def activate_fixture(self):
        with self.lock:
            if self.fixture_start_ns is None:
                self.fixture_start_ns=self.clock();self.wake.set()

    def packet_for(self,name,body,omega,now):
        lever,camera=camera_velocity(body,omega,self.forward,self.up,self.offset)
        return dict(schema=2,time_policy=TIME_POLICY,session_id=self.session_id,packet_id=self.next_id,
            source="synthetic_test_fixture",measurement_valid=False,test_fixture_valid=True,
            fixture_session_id=self.session_id,fixture_phase=name,mcu_epoch=None,
            capture_time_qualified=False,time_uncertainty_ns=None,alignment_uncertainty_ns=None,
            alignment_clock="synthetic_host_clock",alignment_start_ns=now-20_000_000,alignment_end_ns=now,
            interval_start_ns=now-20_000_000,interval_end_ns=now,receive_monotonic_ns=now,
            alignment_gyro_start_ns=now-20_000_000,alignment_gyro_end_ns=now,gyro_receive_ns=now,
            v_body_mps=body,omega_body_rad_s=omega,lever_body_mps=lever,v_camera_mps=camera,
            dependency_sources=["synthetic_test_fixture"],motor_commands_sent=0)

    def _run(self):
        while not self.stop_event.is_set() and not self.done.is_set():
            self.wake.wait(.02);self.wake.clear()
            with self.lock:
                if self.fixture_start_ns is None:continue
                now=self.clock();elapsed=(now-self.fixture_start_ns)/1e9;end=0.;phase=None
                for candidate in self.PHASES:
                    end+=candidate[1]
                    if elapsed<end:phase=candidate;break
                if phase is None:
                    self.log.submit("wheel_fixture.jsonl",dict(event="fixture_complete",timestamp_monotonic_ns=now,
                        source="synthetic_test_fixture",session_id=self.session_id))
                    self.done.set();break
                name,_,body,omega=phase
                if name!=self.fixture_phase:
                    self.fixture_phase=name
                    self.log.submit("wheel_fixture.jsonl",dict(event="fixture_phase",phase=name,
                        timestamp_monotonic_ns=now,elapsed_s=elapsed,source="synthetic_test_fixture"))
                packet=self.packet_for(name,body,omega,now);self.next_id+=1
                try:
                    self.sock.sendto(json.dumps(packet,allow_nan=False).encode(),self.address)
                    self.stats["sent_to_sdk_owner"]+=1
                    self._record("sent_to_sdk_owner",converted=packet,synthetic=True)
                except OSError as error:
                    self._record("sdk_owner_unavailable_no_replay",error=str(error),synthetic=True)

    def snapshot(self,sdk=None):
        result=super().snapshot(sdk)
        result.update(input_source="synthetic_test_fixture",fixture_phase=self.fixture_phase,
                      fixture_complete=self.done.is_set(),actual_measurement=False)
        return result
