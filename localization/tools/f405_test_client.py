"""Single-owner F405 NormalDebug test client, contract a54ee465 / firmware 14157e6.

No import-time I/O, capability probes, wheel injection or motion on start/close.
Host identities and receive-time estimates are NEVER MCU epochs/capture ticks.
Subscribers reuse this instance; a second process must not open the chassis UART.
"""
from __future__ import annotations

import copy
from collections import deque
import json
import math
from pathlib import Path
import secrets
import threading
import time
import uuid

import serial

from f407_motion_protocol import (
    StreamParser, move_body_frame, turn_ccw_frame, motion_stop_frame,
    motion_reset_frame, MOTION_CMD_STOP, MOTION_CMD_RESET_ODOM,
    MOTION_CMD_TURN_REL, MOTION_STATE_RUNNING, MOTION_STATE_DONE,
    MOTION_STATE_FAULT, MOTION_STATE_STOPPED,
)

IMPLEMENTATION_COMMIT = "14157e6bf2600ff0480b133ab763298e12ce400b"
DOCUMENT_COMMIT = "a54ee465bf6ac7c35dc6236c61ee580e71fbcfc0"
CONTRACT_PATH = Path(__file__).with_name("contracts") / "f405_normaldebug_20261030.json"
CONTRACT = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
STATUS_TIMEOUT_NS = 1_000_000_000


class UnsupportedCapability(NotImplementedError):
    pass


class WheelReference:
    """F405 count-delta cross-check; no true sampling tick exists on 0x15.
    Rates are receive-interval estimates, not lower-published measured speeds.
    One byte-read batch contributes one latest count snapshot, not fake 10ms samples.
    """
    def __init__(self):
        self.previous = None

    def update(self, sample, *, batch_reset=False):
        counts = [sample[f"m{i}_count"] for i in (1, 2, 3)]
        result = dict(raw_counts=counts, raw_sequence=sample["sequence"],
            receive_monotonic_ns=sample["timestamp_monotonic_ns"],
            capture_monotonic_ns=None, mcu_epoch=None,
            capture_time_qualified=False, measurement_valid_for_fusion=False,
            source="host_f405_raw_count_recomputation", interval_kind="host_receive_approximation",
            nominal_period_ms=sample["dt_ms"], status=sample["status"], valid=False,
            forward_mps=None, left_mps=None, encoder_model_ccw_rad_s=None,
            lower_measured_forward_mps=None, lower_measured_left_mps=None,
            lower_velocity_unavailable_reason="not implemented on UART4")
        old = self.previous
        dt = None if old is None else (sample["timestamp_monotonic_ns"]-old["timestamp_monotonic_ns"])/1e9
        if sample["status"] != 7 or batch_reset or old is None or dt is None or not 0 < dt <= .25:
            self.previous = dict(sample)
            result["reason"] = "reset_invalid_or_new_baseline" if batch_reset or sample["status"] != 7 else "new_or_gapped_baseline"
            return result
        step = (sample["sequence"]-old["sequence"]) & 255
        if not 0 < step <= 127:
            result["reason"] = "duplicate_or_out_of_order"
            return result
        self.previous = dict(sample)
        delta = [(n-old[f"m{i}_count"]+32768)%65536-32768 for i,n in zip((1,2,3),counts)]
        if any(d == -32768 for d in delta):
            result["reason"] = "half_counter_range_ambiguous"
            return result
        k = math.pi * .070 / 1061.268
        wheel = [-k*d for d in delta]  # Apply encoder feedback signs exactly once.
        forward = (wheel[1]-wheel[0])/math.sqrt(3)
        left = (2*wheel[2]-wheel[0]-wheel[1])/3
        result.update(valid=True, reason="receive_time_estimate_only", interval_s=dt,
            sequence_step=step, raw_delta=delta, feedback_wheel_mps=[d/dt for d in wheel],
            forward_delta_m=forward, left_delta_m=left, forward_mps=forward/dt, left_mps=left/dt,
            encoder_model_ccw_rad_s=-sum(wheel)/(3*.130*dt))
        return result


class ChassisTestClient:
    def __init__(self, device: str, baud: int, status_path: Path,
                 status_log_path: Path, *, identity_confirmed=False, serial_factory=serial.Serial):
        self.device, self.baud = device, baud
        if baud != CONTRACT["transport"]["baud"]:
            raise ValueError("audited F405 contract requires 115200 baud")
        self.status_path, self.status_log_path = status_path, status_log_path
        self.identity_confirmed = identity_confirmed
        self.serial_factory = serial_factory
        self.serial = None
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.log_error = None
        self.link_failed = False
        self.rx_log = None
        self.host_session_id = uuid.uuid4().hex
        self.host_reference_epoch = 0
        self.action_number = 0
        self.next_sequence = secrets.randbelow(256)
        self.used_sequences = {}
        self.pending = None
        self.last_action = None
        self.action_results = deque(maxlen=256)
        self.needs_stop_ack = False
        self.latest_status = None
        self.latest_imu = None
        self.latest_wheel = None
        self.latest_wheel_reference = None
        self.wheel_reference = WheelReference()
        self.wheel_increments = deque(maxlen=256)
        self.subscribers = []
        self.batch_odometry = []
        self.client_stats = dict(unmatched_status=0, repeated_status=0, retired_status=0,
                                 subscriber_errors=0, reference_queue_overflow=0)
        self.parser = StreamParser(on_odom=self._on_odom, on_imu=self._on_imu,
                                   on_motion_status=self._on_status)

    def capabilities(self):
        return {"source":"pinned_source_contract_not_device_query", "document_commit":DOCUMENT_COMMIT,
            "expected_implementation_commit":IMPLEMENTATION_COMMIT, "wire_protocol_version":None,
            "move_body":True, "turn_ccw":True, "stop":True, "reset_reference":True,
            "move_spin_angle":False, "capability_query":False, "versioned_body_velocity":False,
            "boot_epoch":False, "reference_epoch":False, "strict_stale_ack_exclusion":False}

    def query_capabilities(self):
        raise UnsupportedCapability("UART4 capability query is not implemented; use capabilities() for the pinned source manifest")

    def move_spin_angle(self, speed_mm_s, direction_deg, turn_deg, max_rate_dps):
        raise UnsupportedCapability("move_spin_angle has no UART4 request/ACK contract; no packet sent")

    def telemetry(self, callback):
        """Subscribe to this owner's receive stream; callback must not block."""
        with self.lock:
            self.subscribers.append(callback)
        def unsubscribe():
            with self.lock:
                if callback in self.subscribers:
                    self.subscribers.remove(callback)
        return unsubscribe

    def _log(self, filename, document):
        try:
            with (self.status_path.parent / filename).open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(document, ensure_ascii=False)+"\n")
        except OSError as error:
            self.log_error = str(error)

    def _publish(self, kind, sample):
        event = {"kind":kind, "sample":copy.deepcopy(sample), "host_session_id":self.host_session_id,
                 "host_reference_epoch":self.host_reference_epoch, "mcu_epoch":None,
                 "wire_protocol_version":None, "contract_document_revision":1}
        self._log("chassis_telemetry.jsonl", event)
        for callback in tuple(self.subscribers):
            try:
                callback(copy.deepcopy(event))
            except Exception:
                self.client_stats["subscriber_errors"] += 1

    def _on_odom(self, sample):
        self.batch_odometry.append(sample)
        self.latest_wheel = dict(sample)
        self._publish("raw_wheel_counts", sample)

    def _on_imu(self, sample):
        self.latest_imu = dict(sample)
        self._log("imu_telemetry.jsonl", sample)
        self._publish("imu", sample)

    def _finish(self, outcome, now_ns, **extra):
        if self.pending is None:
            return
        self.last_action = dict(self.pending, outcome=outcome, ended_ns=now_ns, **extra)
        self.action_results.append(copy.deepcopy(self.last_action))
        self._log("chassis_actions.jsonl", self.last_action)
        self.pending = None

    def poll(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        with self.lock:
            if self.pending and (now_ns > self.pending["deadline_ns"] or
                    now_ns-self.pending["last_matching_ns"] > STATUS_TIMEOUT_NS):
                self._finish("unknown_timeout_no_motion_replayed", now_ns)
                self.needs_stop_ack = True  # Recoverable by a user-requested matching STOPPED.

    def _on_status(self, sample):
        now = sample["timestamp_monotonic_ns"]
        self.poll(now)
        sample = dict(sample, health=None, mcu_epoch=None, capture_monotonic_ns=None,
                      host_action_id=None, correlated=False, physical_stopped_verified=False)
        old = self.latest_status
        if old is not None and now-old["timestamp_monotonic_ns"] <= STATUS_TIMEOUT_NS:
            delta = (sample["sequence"]-old["sequence"]) & 255
            if delta == 0 or delta > 127:
                self.client_stats["repeated_status" if delta == 0 else "retired_status"] += 1
                self._publish("uncorrelated_status", sample)
                return
        pending = self.pending
        if (pending and now > pending["sent_ns"] and sample["command_sequence"] == pending["wire_sequence"]
                and sample["command"] == pending["opcode"]):
            sample.update(correlated=True, host_action_id=pending["host_action_id"],
                          correlation="receive_after_send_seq_opcode_best_effort_not_boot_proof")
            pending["last_matching_ns"] = now
            if sample["state"] == MOTION_STATE_RUNNING:
                pending["running_seen"] = True
            sample["running_seen"] = pending["running_seen"]
            if sample["command"] == MOTION_CMD_TURN_REL:
                sample["application_progress_ccw_deg"] = -sample["progress"]*.1
            terminal = ((sample["state"] == MOTION_STATE_STOPPED and pending["opcode"] == MOTION_CMD_STOP) or
                        (sample["state"] == MOTION_STATE_DONE and pending["opcode"] != MOTION_CMD_STOP) or
                        sample["state"] == MOTION_STATE_FAULT)
            if terminal:
                if sample["state"] == MOTION_STATE_STOPPED and not sample["fault"]:
                    self.needs_stop_ack = False
                if pending["opcode"] == MOTION_CMD_RESET_ODOM and sample["state"] == MOTION_STATE_DONE and not sample["fault"]:
                    self.host_reference_epoch += 1
                    self.wheel_reference.previous = None
                    self.wheel_increments.clear()
                self._finish("reported_"+sample["state_name"].lower(), now, status=sample)
        else:
            self.client_stats["unmatched_status"] += 1
        self.latest_status = sample
        self._log(self.status_log_path.name, sample)
        try:
            temporary = self.status_path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(sample,ensure_ascii=False)+"\n",encoding="utf-8")
            temporary.replace(self.status_path)
        except OSError as error:
            self.log_error = str(error)
        self._publish("motion_status", sample)

    def feed(self, data, receive_ns):
        """Same receiver used by live UART and offline/PTY checks; never writes."""
        with self.lock:
            self.batch_odometry = []
            self.parser.feed(data, receive_ns=receive_ns)
            if self.batch_odometry:
                reset = any(s["status"] != 7 for s in self.batch_odometry)
                if reset and self.wheel_reference.previous is not None:
                    self._finish("unknown_counter_reset_no_boot_epoch", receive_ns)
                    self.needs_stop_ack = True
                result = self.wheel_reference.update(self.batch_odometry[-1], batch_reset=reset)
                result["coalesced_frames"] = len(self.batch_odometry)-1
                if len(self.wheel_increments) == self.wheel_increments.maxlen:
                    self.wheel_increments.clear()
                    self.client_stats["reference_queue_overflow"] += 1
                    result.update(valid=False, reason="reference_queue_overflow")
                self.wheel_increments.append(result)
                self.latest_wheel_reference = result
                self._publish("host_wheel_recomputed", result)

    def start(self):
        if self.serial is not None:
            raise RuntimeError("client already owns a UART")
        self.serial = self.serial_factory(self.device,self.baud,timeout=.05,write_timeout=.2,exclusive=True)
        try:
            self.rx_log = (self.status_path.parent/"uart_rx.jsonl").open("a",encoding="utf-8",buffering=1)
        except OSError:
            self.serial.close()
            self.serial = None
            raise
        self.thread = threading.Thread(target=self._read_loop, name="f405-test-uart-owner",daemon=True)
        self.thread.start()

    def _read_loop(self):
        while not self.stop_event.is_set():
            try:
                data = self.serial.read(min(self.serial.in_waiting or 1,4096))
            except (serial.SerialException,OSError) as error:
                self.link_failed = True
                self._log("chassis_actions.jsonl", {"event":"link_error", "error":str(error)})
                with self.lock:
                    self._finish("unknown_link_error_no_replay",time.monotonic_ns())
                    self.needs_stop_ack = True
                return
            now = time.monotonic_ns()
            if data:
                try:
                    self.rx_log.write(json.dumps({"timestamp_monotonic_ns":now,"data_hex":data.hex()})+"\n")
                except OSError as error:
                    self.log_error = str(error)
                self.feed(data,now)
            self.poll(now)

    def _submit(self, encoder, name, parameters, timeout_s):
        with self.lock:
            now = time.monotonic_ns()
            self.poll(now)
            if self.serial is None or self.stop_event.is_set():
                raise RuntimeError("chassis UART is not connected")
            if name != "stop":
                if self.link_failed:
                    raise RuntimeError("receive link failed; start a deliberate new client session, without replay")
                if not self.identity_confirmed:
                    raise RuntimeError("confirm actual F405 NormalDebug MOTIONDEBUG service and firmware before motion")
                if self.pending is not None or self.needs_stop_ack:
                    raise RuntimeError("wait for this action, or request STOP and await matching STOPPED first")
                if self.latest_status is None or not 0 <= now-self.latest_status["timestamp_monotonic_ns"] <= STATUS_TIMEOUT_NS:
                    raise RuntimeError("no fresh 0x1A state; current mechanical state is unknown")
                if self.latest_status["state"] in (MOTION_STATE_RUNNING, MOTION_STATE_FAULT):
                    raise RuntimeError("lower reports RUNNING/FAULT; finish the action or explicitly STOP before a new request")
                if self.log_error:
                    raise RuntimeError("test log is unavailable: "+self.log_error)
            sequence = None
            for _ in range(256):
                candidate = self.next_sequence
                self.next_sequence = (candidate+1)&255
                if ((name == "stop" or now-self.used_sequences.get(candidate,-10**20) >= 30_000_000_000) and
                        (self.pending is None or candidate != self.pending["wire_sequence"]) and
                        (self.latest_status is None or candidate != self.latest_status["command_sequence"])):
                    sequence = candidate
                    break
            if sequence is None:
                raise RuntimeError("all uint8 sequences are inside the short reuse guard; wait before submitting")
            packet = encoder(sequence)
            self.used_sequences[sequence] = now
            if self.pending is not None:
                self._finish("superseded_by_stop_not_physical_stop_proof",now)
            self.action_number += 1
            action = dict(host_session_id=self.host_session_id,
                host_action_id=f"{self.host_session_id}:{self.action_number}", wire_sequence=sequence,
                opcode=packet[4], name=name, parameters=parameters, packet_hex=packet.hex(),
                sent_ns=now, last_matching_ns=now, deadline_ns=now+int(timeout_s*1e9),
                host_reference_epoch=self.host_reference_epoch, mcu_epoch=None,
                running_seen=False, stale_reply_exclusion_guaranteed=False)
            self.pending = action
            if name == "stop":
                self.needs_stop_ack = True
            self._log("chassis_actions.jsonl",dict(action,event="submit"))
            if self.log_error and name != "stop":
                self._finish("not_sent_log_unavailable",now)
                raise RuntimeError("action not sent because test logging failed")
            try:
                if self.serial.write(packet) != len(packet):
                    raise RuntimeError("partial UART write; action status is unknown, no automatic retransmit")
            except Exception:
                self._finish("unknown_write_failure_no_replay",time.monotonic_ns())
                self.needs_stop_ack = True
                raise
            return dict(action)

    def move_body(self, forward_mm, left_mm, speed_mm_s=300):
        move_body_frame(0,forward_mm,left_mm,speed_mm_s)
        speed = speed_mm_s or (500 if left_mm == 0 else 300)
        deadline = min(120,max(5,4*math.hypot(forward_mm,left_mm)/speed+5))+2
        return self._submit(lambda s:move_body_frame(s,forward_mm,left_mm,speed_mm_s),"move_body",
            dict(forward_mm=forward_mm,left_mm=left_mm,speed_mm_s=speed_mm_s),deadline)

    def turn_ccw(self, angle_deg, rate_deg_s=60):
        turn_ccw_frame(0,angle_deg,rate_deg_s)
        return self._submit(lambda s:turn_ccw_frame(s,angle_deg,rate_deg_s),"turn_ccw",
                            dict(angle_ccw_deg=angle_deg,rate_deg_s=rate_deg_s),12)

    def stop(self):
        return self._submit(motion_stop_frame,"stop",{},2)

    def reset_reference(self, *, stationary_confirmed=False):
        if not stationary_confirmed:
            raise ValueError("operator must confirm physically stationary; RESET DONE does not prove it")
        return self._submit(motion_reset_frame,"reset_reference",dict(operator_stationary_confirmed=True),2)

    def status_snapshot(self):
        with self.lock:
            return copy.deepcopy(self.latest_status), dict(self.parser.stats,**self.client_stats)

    def imu_snapshot(self):
        with self.lock:
            return copy.deepcopy(self.latest_imu)

    def drain_wheel_increments(self):
        with self.lock:
            result = list(self.wheel_increments)
            self.wheel_increments.clear()
            return result

    def drain_action_results(self):
        with self.lock:
            result = list(self.action_results)
            self.action_results.clear()
            return result

    def close(self):
        """No implicit STOP/reset/replay. UI handles any user-authorized exit STOP."""
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1)
        if self.serial is not None:
            self.serial.close()
            self.serial = None
        if self.rx_log is not None:
            self.rx_log.close()
        with self.lock:
            self._finish("unknown_client_closed_no_replay",time.monotonic_ns())
