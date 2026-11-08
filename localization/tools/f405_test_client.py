"""Single-owner F405 NormalDebug client; legacy + audited timed v1 transport.

No import-time I/O, wheel injection or motion on start/close. Timed startup and
periodic health checks send ONLY the documented read-only identity request.
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
import struct

import serial
import f405_timed_protocol as timed
from async_debug_log import AsyncDebugLog

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
TIMED_DOCUMENT_COMMIT = "f81e9ef9a33981d8453eafe9574572e87dea90c6"
TIMED_IMPLEMENTATION_COMMIT = "43fa6ab456ff26f2cc5661d2b3934efbf482fb45"
# f81e9ef changes only exported docs/artifacts, not compiled control/transport
# sources. Rebuilding there legitimately embeds that Git revision (actual board).
TIMED_COMMITS = (TIMED_IMPLEMENTATION_COMMIT, "cd26448d624b56ede38fa665564687c439bc2bdf", TIMED_DOCUMENT_COMMIT)
TIMED_V11_IMPLEMENTATION_COMMIT = "512038d2cfe83653d1c3036ad003fc23117df344"
TIMED_V11_DOCUMENT_COMMIT = "3f61b7ac38bcad0e2e43e13688e548892b974965"
TIMED_V11_COMMITS = (TIMED_V11_IMPLEMENTATION_COMMIT,TIMED_V11_DOCUMENT_COMMIT,
                     "11eeacfda71f419834ff78401faf1a0d35a3831e")
TIMED_V13_IMPLEMENTATION_COMMIT = "0a2dbf7f523cba577c6ccacc3db5fae31b8dae48"
TIMED_V13_DOCUMENT_COMMIT = "4ac1d3d5006df1d4bced6a9f9e23fa68cef2da96"
TIMED_V13_COMMITS = (TIMED_V13_IMPLEMENTATION_COMMIT, TIMED_V13_DOCUMENT_COMMIT)
# Reviewed layouts stay compatible; v1.3 removes quality pages and separates
# recovery state from value validity. Bind each source revision to its semantics.
TIMED_REVIEWED_VERSIONS = {**{sha:(1,0) for sha in TIMED_COMMITS},
                          **{sha:(1,1) for sha in TIMED_V11_COMMITS},
                          **{sha:(1,3) for sha in TIMED_V13_COMMITS}}
TIMED_COMMITS = tuple(TIMED_REVIEWED_VERSIONS)
IDENTITY_TIMEOUT_NS = 6_000_000_000


def timed_contract_profile(identity):
    version = (identity["protocol_major"], identity["protocol_minor"])
    if version == (1, 3):
        return TIMED_V13_IMPLEMENTATION_COMMIT, TIMED_V13_DOCUMENT_COMMIT
    if version == (1, 1):
        return TIMED_V11_IMPLEMENTATION_COMMIT, TIMED_V11_DOCUMENT_COMMIT
    return TIMED_IMPLEMENTATION_COMMIT, TIMED_DOCUMENT_COMMIT


def firmware_compatible(value):
    value = value.strip().lower()
    return value in ("", "unknown", "unconfirmed") or (7 <= len(value) <= 40 and
        all(c in "0123456789abcdef" for c in value) and
        any(commit.startswith(value) for commit in (IMPLEMENTATION_COMMIT, *TIMED_COMMITS)))


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
        if sample["status"] != 7 or batch_reset or old is None or dt is None or dt <= 0:
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
                 status_log_path: Path, *, identity_confirmed=False, serial_factory=serial.Serial,
                 identity_policy="auto", asserted_firmware="unconfirmed"):
        self.device, self.baud = device, baud
        if baud not in (115200,230400,921600):
            raise ValueError("F405 link supports 115200, 230400 or 921600 baud")
        if identity_policy not in ("auto","legacy","timed") or (identity_policy=="legacy" and baud!=115200):
            raise ValueError("legacy profile requires 115200; timed profile uses a read-only identity handshake")
        self.require_device_identity = identity_policy=="timed" or baud in (230400,921600)
        self.identity_policy = identity_policy
        self.asserted_firmware = asserted_firmware.strip().lower()
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
        self.async_log = None
        self.timed_requested=False;self.timed_stream_active=False;self.timed_enable_attempted=False
        self.have_sync_exchange=False;self.next_sync_ns=0;self.next_config_ns=0
        self.timed_error=None;self.timed_enable_writes=0;self.timed_disable_writes=0
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
        self.device_identity = None
        self.identity_received_ns = 0
        self.identity_error = "waiting for read-only device identity" if self.require_device_identity else None
        self.identity_queries = {}
        self.next_query_sequence = secrets.randbelow(65536)
        self.next_identity_query_ns = 0
        self.wheel_reference = WheelReference()
        self.wheel_increments = deque(maxlen=256)
        self.subscribers = []
        self.batch_odometry = []
        self.client_stats = dict(unmatched_status=0, repeated_status=0, retired_status=0,
                                 subscriber_errors=0, reference_queue_overflow=0,
                                 identity_queries=0, identity_timeouts=0, unmatched_extended=0)
        self.parser = StreamParser(on_odom=self._on_odom, on_imu=self._on_imu,
                                   on_motion_status=self._on_status)
        self.mixed = timed.MixedStream(lambda raw,ns:self.parser.feed(raw,receive_ns=ns),self._on_extended)

    def capabilities(self):
        if self.device_identity is not None:
            d=self.device_identity; bits=d["capabilities"]
            return dict(source="nonce_correlated_device_identity_reply",
                document_commit=timed_contract_profile(d)[1],
                actual_firmware_commit=d["git_commit"],wire_protocol_version=[d["protocol_major"],d["protocol_minor"]],
                move_body=bool(bits&64),turn_ccw=bool(bits&64),stop=bool(bits&64),reset_reference=bool(bits&64),
                move_spin_angle=False,capability_query=bool(bits&1),quality_query_available=bool(bits&1024),
                stats_query_available=bool(bits&32),versioned_body_velocity=bool(bits&8),
                boot_epoch=d["epoch_valid"],reference_epoch=bool(bits&256),strict_stale_ack_exclusion=False,
                capture_time_qualified=False,sdk_time_qualified=False)
        return {"source":"pinned_source_contract_not_device_query", "document_commit":DOCUMENT_COMMIT,
            "expected_implementation_commit":IMPLEMENTATION_COMMIT, "wire_protocol_version":None,
            "move_body":True, "turn_ccw":True, "stop":True, "reset_reference":True,
            "move_spin_angle":False, "capability_query":False, "versioned_body_velocity":False,
            "boot_epoch":False, "reference_epoch":False, "strict_stale_ack_exclusion":False}

    def query_capabilities(self, timeout_s=1.0):
        """Read-only v1 identity query. No stream subscription or clock promotion."""
        event=self._request_identity()
        if not event.wait(timeout_s):
            raise RuntimeError(f"no identity reply at {self.baud}; check firmware preset/baud and legacy 0x1A")
        if getattr(event,"identity_reply",None) is None:
            raise RuntimeError(f"read-only identity query failed/timed out at {self.baud}; previous replies are not this query's result")
        if self.identity_error:
            raise RuntimeError(self.identity_error)
        return self.capabilities()

    def _request_identity(self):
        return self._request_timed(timed.ID_REQ)

    def _request_timed(self, kind, *, enable=False):
        with self.lock:
            if self.serial is None or self.stop_event.is_set():
                raise RuntimeError("chassis UART is not connected for identity query")
            for q in self.identity_queries.values():
                if q["request_type"]==kind:return q["event"]
            # Set the next attempt BEFORE logging/writing: a failed request must
            # not be retried on every UART/GUI poll or flood its diagnostic log.
            attempt=time.monotonic_ns()
            if kind==timed.ID_REQ:self.next_identity_query_ns=attempt+2_000_000_000
            if kind==timed.SYNC_REQ:self.next_sync_ns=attempt+500_000_000
            if kind==timed.CONFIG_REQ:self.next_config_ns=attempt+2_000_000_000
            nonce=secrets.randbits(64)
            seq=self.next_query_sequence;self.next_query_sequence=(seq+1)&65535
            packet=timed.request(kind,nonce,seq,enable=enable)
            event=threading.Event()
            event.identity_reply=None
            self._log("chassis_link.jsonl",dict(event="identity_request" if kind==timed.ID_REQ else "timed_request",nonce=nonce,sequence=seq,
                request_type=kind,packet_hex=packet.hex(),baud=self.baud,read_only=kind!=timed.CONFIG_REQ,motor_command=False))
            # Diagnostic persistence is not a UART/identity prerequisite.
            # Keep matching real replies even if a bounded log queue overflowed.
            before=time.monotonic_ns()
            self.identity_queries[nonce]=dict(sequence=seq,event=event,sent_ns=before,deadline_ns=before+1_000_000_000,
                                              request_type=kind,reply_type=kind+1)
            if kind==timed.ID_REQ:
                self.next_identity_query_ns=before+2_000_000_000;self.client_stats["identity_queries"]+=1
            if kind==timed.SYNC_REQ:self.next_sync_ns=before+500_000_000
            if kind==timed.CONFIG_REQ:
                self.next_config_ns=before+2_000_000_000
                if enable:self.timed_enable_attempted=True
            try:
                if self.serial.write(packet)!=len(packet):raise RuntimeError("partial read-only identity write")
                if kind==timed.CONFIG_REQ:
                    if enable:self.timed_enable_writes+=1
                    else:self.timed_disable_writes+=1
                self._log("chassis_link.jsonl",dict(event="identity_request_written" if kind==timed.ID_REQ else "timed_request_written",
                    request_type=kind,nonce=nonce,sequence=seq,
                    write_before_monotonic_ns=before,write_after_monotonic_ns=time.monotonic_ns(),
                    read_only=kind!=timed.CONFIG_REQ,motor_command=False))
            except (OSError,RuntimeError) as error:
                self.identity_queries.pop(nonce,None)
                if kind==timed.ID_REQ:self.identity_error=str(error)
                else:self.timed_error=str(error)
                event.set()
                self._log("chassis_link.jsonl",dict(event="identity_write_failed",error=str(error)))
            return event

    def _identity_problem(self, d):
        if (d["board"],d["mode"]) != (405,5) or d["service"] not in (1,2):
            return "device feedback is not F405 NormalDebug MOTIONDEBUG/RESCUE_TASK"
        version=(d["protocol_major"],d["protocol_minor"])
        if version not in ((1,0),(1,1),(1,3)):
            return f"device protocol {version[0]}.{version[1]} is not reviewed (supported 1.0/1.1/1.3)"
        if d["uart_baud"]!=self.baud or not d["capabilities"]&64 or d["max_payload"]!=160:
            return "device UART configuration or legacy compatibility does not match this client"
        # The wire layout determines compatibility. A rebuild at a new SHA
        # (including the 921600/100 Hz change) must not require a host whitelist
        # update. Preserve the actual SHA/dirty flag as provenance, never
        # substitute an expected revision. Known revisions retain their layout.
        if d["git_commit"] in TIMED_REVIEWED_VERSIONS and TIMED_REVIEWED_VERSIONS[d["git_commit"]]!=version:
            return "device protocol version differs from the reviewed firmware revision"
        if version==(1,1) and not d["capabilities"]&1024:
            return "reviewed v1.1 firmware is missing its quality-query capability"
        if self.asserted_firmware not in ("","unknown","unconfirmed") and not d["git_commit"].startswith(self.asserted_firmware):
            return "operator firmware assertion differs from actual device reply"
        return None

    def _on_extended(self, raw, receive_ns):
        try:d=timed.decode(raw)
        except (ValueError,struct.error):
            self.mixed.stats["mixed_malformed"]+=1;return
        d.update(receive_monotonic_ns=receive_ns,capture_time_qualified=False,sdk_time_qualified=False)
        # Raw timestamps are MCU events. Unknown uncertainty FFFFFFFF remains
        # unknown, never converted into a qualified host capture timestamp.
        self._log("chassis_extended.jsonl",d)
        if d["type"]==timed.ID_REPLY:
            q=self.identity_queries.get(d["nonce"])
            if not q or q["reply_type"]!=d["type"] or q["sequence"]!=d["sequence"] or not q["sent_ns"]<=receive_ns<=q["deadline_ns"]:
                self.client_stats["unmatched_extended"]+=1;return
            self.identity_queries.pop(d["nonce"])
            clock_ok=(d["protocol_major"],d["protocol_minor"])!=(1,1) or bool(d["clock_flags"]&1)
            d["epoch_valid"]=(d["epoch_fault"]==0 and bool(d["clock_flags"]&2) and clock_ok and d["boot_epoch"]!="00"*16)
            previous=self.device_identity
            if previous and (previous["boot_epoch"],previous["firmware_id"],previous["git_commit"]) != (d["boot_epoch"],d["firmware_id"],d["git_commit"]):
                had_action=self.pending is not None
                self._finish("unknown_device_restart_or_image_change_no_replay",receive_ns)
                self.needs_stop_ack=self.needs_stop_ack or had_action
                self.latest_status=None;self.wheel_reference.previous=None;self.wheel_increments.clear()
                self.have_sync_exchange=False;self.timed_stream_active=False
                self._publish("device_identity_changed",dict(previous_boot_epoch=previous["boot_epoch"],new_boot_epoch=d["boot_epoch"]))
            self.device_identity=d;self.identity_received_ns=receive_ns
            self.identity_error=self._identity_problem(d)
            d["interface_compatible"]=self.identity_error is None
            self.parser.imu_protocol_version = ((d["protocol_major"],d["protocol_minor"])
                                                if self.identity_error is None else None)
            self._log("chassis_link.jsonl",dict(event="identity_reply",identity=d,compatibility_error=self.identity_error,
                round_trip_ns=receive_ns-q["sent_ns"],clock_mapping_qualified=False))
            try:
                path=self.status_path.parent/"chassis_identity.json"
                self._atomic_log(path.name,d)
            except OSError as error:self.log_error=str(error)
            q["event"].identity_reply=copy.deepcopy(d)
            q["event"].set()
            self._publish("timed_identity",d)
        elif d["type"] in (timed.SYNC_REPLY,timed.CONFIG_REPLY,timed.STATS_REPLY):
            q=self.identity_queries.get(d.get("nonce"))
            if (not q or q["reply_type"]!=d["type"] or q["sequence"]!=d["sequence"] or
                not q["sent_ns"]<=receive_ns<=q["deadline_ns"] or not self.device_identity or
                d["boot_epoch"]!=self.device_identity["boot_epoch"]):
                self.client_stats["unmatched_extended"]+=1;return
            self.identity_queries.pop(d["nonce"]);d["host_write_before_ns"]=q["sent_ns"]
            if d["type"]==timed.SYNC_REPLY:
                self.have_sync_exchange=d["epoch_valid"]==1
                self._publish("timed_clock_sync",d)
            elif d["type"]==timed.CONFIG_REPLY:
                self.timed_stream_active=d["status"]==0 and d["enabled"]==1
                if d["status"]==0 and d["enabled"]==0:self.timed_enable_attempted=False
                self.timed_error=None if d["status"]==0 else f"stream configuration rejected status={d['status']}"
                self._publish("timed_stream_state",d)
            q["event"].identity_reply=copy.deepcopy(d);q["event"].set()
        elif d["type"] in (timed.WHEEL,timed.GYRO):
            self._publish("timed_wheel_diagnostic" if d["type"]==timed.WHEEL else "timed_gyro_diagnostic",d)
        elif d["type"]==timed.ERROR_REPLY:
            q=self.identity_queries.get(d.get("nonce"))
            if q and q["sequence"]==d["sequence"] and d["request_type"]==q["request_type"] and q["sent_ns"]<=receive_ns<=q["deadline_ns"]:
                self.identity_queries.pop(d["nonce"])
                if q["request_type"]==timed.ID_REQ:self.identity_error="device rejected identity request"
                else:self.timed_error=f"device rejected timed request status={d['status']}"
                q["event"].set()

    def start_timed_measurements(self):
        """Explicit test request: documented 07/08 plus diagnostic clock sync.
        Missing clock bounds/exchanges must not block measurement subscription.
        """
        with self.lock:
            self.timed_requested=True;self.next_sync_ns=0;self.next_config_ns=0

    def measurement_snapshot(self):
        with self.lock:
            return dict(requested=self.timed_requested,stream_active=self.timed_stream_active,
                        have_matched_sync=self.have_sync_exchange,error=self.timed_error,
                        enable_packets_written=self.timed_enable_writes,disable_packets_written=self.timed_disable_writes,
                        logging_statistics=self.async_log.statistics() if self.async_log else None,
                        capture_time_qualified=False,logging_healthy=self.async_log.healthy() if self.async_log else self.log_error is None)

    def control_identity_problem(self, now_ns=None):
        now=time.monotonic_ns() if now_ns is None else now_ns
        with self.lock:
            if self.device_identity is not None:
                if self.identity_error:return self.identity_error
                if self.device_identity["service"]!=1:
                    return "motion commands require MOTIONDEBUG; RESCUE_TASK feedback is read-only here"
                return None
            if self.require_device_identity:
                return f"no validated identity at {self.baud}; check matching F405 baud (921600/230400 timed, 115200 legacy) and actual board/mode/service"
            if not self.identity_confirmed or not firmware_compatible(self.asserted_firmware):
                return "confirm actual F405 NormalDebug MOTIONDEBUG legacy profile before motion"
            return None

    def connection_snapshot(self):
        with self.lock:
            d=copy.deepcopy(self.device_identity)
            profile=timed_contract_profile(d) if d else None
            return dict(baud=self.baud,identity_policy=self.identity_policy,
                motion_identity_ready=self.control_identity_problem() is None,
                reason=self.control_identity_problem(),identity=d,
                actual_lower_firmware=d["git_commit"] if d else "unconfirmed",
                actual_lower_mode=("NormalRun" if d["service"]==2 else "NormalDebug") if d and d["mode"]==5 else "unconfirmed",
                actual_lower_service=({1:"MOTIONDEBUG",2:"RESCUE_TASK"}.get(d["service"],"unconfirmed") if d else "unconfirmed"),
                firmware_identity_reported_by_uart=d is not None,cryptographically_attested=False,
                mcu_boot_epoch=d["boot_epoch"] if d and d["epoch_valid"] else None,
                contract_document_commit=profile[1] if profile else TIMED_DOCUMENT_COMMIT if self.require_device_identity else DOCUMENT_COMMIT,
                expected_lower_firmware=profile[0] if profile else TIMED_IMPLEMENTATION_COMMIT if self.require_device_identity else IMPLEMENTATION_COMMIT,
                capture_time_qualified=False,sdk_time_qualified=False,stats=dict(self.mixed.stats,**self.client_stats))

    def move_spin_angle(self, speed_mm_s, direction_deg, turn_deg, max_rate_dps):
        raise UnsupportedCapability("move_spin_angle has no UART4 request/ACK contract; no packet sent")

    def telemetry(self, callback, *, readonly=False):
        """Subscribe without blocking. Internal readonly consumers promise not
        to mutate the frozen receive snapshot; other consumers get deep copies.
        """
        subscription=(callback,readonly)
        with self.lock:
            self.subscribers.append(subscription)
        def unsubscribe():
            with self.lock:
                if subscription in self.subscribers:
                    self.subscribers.remove(subscription)
        return unsubscribe

    def _log(self, filename, document):
        if self.async_log is not None:
            if not self.async_log.submit(filename,document):self.log_error=self.async_log.error or "bounded log queue overflow; session incomplete"
            return
        try:
            with (self.status_path.parent / filename).open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(document, ensure_ascii=False)+"\n")
        except OSError as error:
            self.log_error = str(error)

    def _atomic_log(self, filename, document):
        if self.async_log is not None:
            if not self.async_log.submit(filename,document,atomic=True):self.log_error=self.async_log.error or "bounded log queue overflow"
        else:
            path=self.status_path.parent/filename;tmp=path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(document,ensure_ascii=False)+"\n",encoding="utf-8");tmp.replace(path)

    def _publish(self, kind, sample):
        event = {"kind":kind, "sample":copy.deepcopy(sample), "host_session_id":self.host_session_id,
                 "host_reference_epoch":self.host_reference_epoch, "mcu_epoch":None,
                 "wire_protocol_version":None, "contract_document_revision":1}
        action=self.pending or self.last_action
        event["host_action_id_at_receive"]=action["host_action_id"] if action else None
        event["action_identity_basis"]="host receive context, not MCU acquisition identity"
        self._log("chassis_telemetry.jsonl", event)
        for callback,readonly in tuple(self.subscribers):
            try:
                callback(event if readonly else copy.deepcopy(event))
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
            if self.async_log and not self.async_log.healthy():self.log_error=self.async_log.error or "diagnostic log overflow"
            self.mixed.expire(now_ns)
            for nonce,q in list(self.identity_queries.items()):
                if now_ns>q["deadline_ns"]:
                    self.identity_queries.pop(nonce);self.client_stats["identity_timeouts"]+=1
                    self._log("chassis_link.jsonl",dict(event="identity_timeout",baud=self.baud,nonce=nonce))
                    q["event"].set()
            if self.pending and now_ns > self.pending["deadline_ns"]:
                self._finish("unknown_timeout_no_motion_replayed", now_ns)
                self.needs_stop_ack = True  # Recoverable by a user-requested matching STOPPED.
            if self.require_device_identity and self.serial is not None and not self.stop_event.is_set() and now_ns>=self.next_identity_query_ns:
                try:self._request_identity()
                except (OSError,RuntimeError) as error:self.identity_error=str(error)
            if self.timed_requested and self.device_identity and self.identity_error is None:
                try:
                    if now_ns>=self.next_sync_ns:self._request_timed(timed.SYNC_REQ)
                    if not self.timed_stream_active and now_ns>=self.next_config_ns:
                        self._request_timed(timed.CONFIG_REQ,enable=True)
                except (OSError,RuntimeError) as error:self.timed_error=str(error)

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
            self._atomic_log(self.status_path.name,sample)
        except OSError as error:
            self.log_error = str(error)
        self._publish("motion_status", sample)

    def feed(self, data, receive_ns):
        """Same receiver used by live UART and offline/PTY checks; never writes."""
        with self.lock:
            self.batch_odometry = []
            self.mixed.feed(data, receive_ns)
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
        self.async_log=AsyncDebugLog(self.status_path.parent)
        self.thread = threading.Thread(target=self._read_loop, name="f405-test-uart-owner",daemon=True)
        self.thread.start()
        if self.require_device_identity:
            # Read-only handshake; failure leaves the map listening with a clear
            # reason, never sends STOP/move/reset or enables the new high-rate flow.
            try:self.query_capabilities(timeout_s=1.1)
            except RuntimeError as error:self.identity_error=str(error)

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
                    self._log("uart_rx.jsonl",{"timestamp_monotonic_ns":now,"data_hex":data.hex()})
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
                identity_problem=self.control_identity_problem(now)
                if identity_problem:raise RuntimeError(identity_problem)
                if self.pending is not None or self.needs_stop_ack:
                    raise RuntimeError("wait for this action, or request STOP and await matching STOPPED first")
                if self.latest_status is None:
                    raise RuntimeError("no 0x1A state received; current mechanical state is unknown")
                if self.latest_status["state"] in (MOTION_STATE_RUNNING, MOTION_STATE_FAULT):
                    raise RuntimeError("lower reports RUNNING/FAULT; finish the action or explicitly STOP before a new request")
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
                observed_boot_epoch_at_send=self.connection_snapshot()["mcu_boot_epoch"],
                running_seen=False, stale_reply_exclusion_guaranteed=False)
            self.pending = action
            if name == "stop":
                self.needs_stop_ack = True
            self._log("chassis_actions.jsonl",dict(action,event="submit"))
            # A missing log record marks the session incomplete; it must not
            # prevent an otherwise valid operator command or its real receipt.
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
            stats=dict(self.parser.stats,**self.mixed.stats,**self.client_stats)
            stats["crc_errors"]+=self.mixed.stats["mixed_crc_errors"]
            return copy.deepcopy(self.latest_status), stats

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
        if self.timed_enable_attempted and self.serial is not None and not self.stop_event.is_set():
            self.timed_requested=False
            with self.lock:
                for nonce,q in list(self.identity_queries.items()):
                    if q["request_type"]==timed.CONFIG_REQ:
                        self.identity_queries.pop(nonce);q["event"].set()
            try:self._request_timed(timed.CONFIG_REQ,enable=False).wait(.3)
            except (OSError,RuntimeError):pass
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
        if self.async_log:
            self.async_log.close()
            if not self.async_log.healthy():self.log_error=self.async_log.error or "diagnostic logging incomplete"
