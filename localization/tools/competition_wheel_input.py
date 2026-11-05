"""Competition adapter for the existing tested wheel bridge; no UART or SDK owner.

C++ forwards complete 07/08/identity containers with their host receive time.
All measurement/gyro pairing, C-frame conversion and SDK ingress remain in the
same WheelInputBridge/TimedMeasurements/DebugWheelInput used by the test map.
"""
import json
import secrets
import socket
import struct
import threading

import f405_timed_protocol as wire
from t265_wheel_input import WheelInputBridge, write_sdk_config


class CompetitionWheelInput:
    def __init__(self, root, config, session_id, telemetry_sink=None):
        self.root, self.session_id = root, session_id
        self.identity_nonce, self.config_nonce = secrets.randbits(64), secrets.randbits(64)
        self.requests_path = root / "wheel_feedback_requests.bin"
        self.requests_path.write_bytes(wire.request(wire.ID_REQ, self.identity_nonce, 0) +
                                       wire.request(wire.CONFIG_REQ, self.config_nonce, 1, enable=True))
        write_sdk_config(root / "wheel_sdk_config.json", .10)
        self.address = "\0f405-feedback-" + session_id
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.bind(self.address)
        self.sock.settimeout(.05)
        self.bridge = WheelInputBridge(root, config, session_id, True)
        self.identity = None
        self.error = None
        self.stop_event = threading.Event()
        self.thread = None
        self.telemetry_sink = telemetry_sink

    def start(self):
        self.bridge.start()
        self.thread = threading.Thread(target=self._run, name="competition-wheel-feedback", daemon=True)
        self.thread.start()

    def accept(self, data):
        received_ns, = struct.unpack(">Q", data[:8])
        sample = wire.decode(data[8:])
        sample.update(receive_monotonic_ns=received_ns, capture_time_qualified=False, sdk_time_qualified=False)
        kind = sample["type"]
        if kind == wire.ID_REPLY:
            if sample["nonce"] != self.identity_nonce or sample["sequence"] != 0:
                return
            version = (sample["protocol_major"], sample["protocol_minor"])
            clock_ok = version != (1, 1) or bool(sample["clock_flags"] & 1)
            sample["epoch_valid"] = (sample["epoch_fault"] == 0 and bool(sample["clock_flags"] & 2)
                                     and clock_ok and sample["boot_epoch"] != "00"*16)
            # Packet semantics, not a Git SHA whitelist or a control admission test.
            sample["interface_compatible"] = sample["board"] == 405 and version in {(1,0),(1,1),(1,3)}
            self.identity = dict(sample)
            self.bridge.log.submit("chassis_identity.json", sample, atomic=True)
            event = "timed_identity"
        elif kind == wire.CONFIG_REPLY:
            if sample["nonce"] != self.config_nonce or sample["sequence"] != 1:
                return
            event = "timed_stream_state"
        elif kind in (wire.WHEEL, wire.GYRO):
            event = "timed_wheel_diagnostic" if kind == wire.WHEEL else "timed_gyro_diagnostic"
        else:
            self.bridge.log.submit("chassis_extended.jsonl", sample)
            return
        message = dict(kind=event, sample=sample, host_session_id=self.session_id)
        self.bridge.telemetry(message)
        if self.telemetry_sink is not None:
            self.telemetry_sink(message)

    def _run(self):
        while not self.stop_event.is_set():
            try:
                data = self.sock.recv(512)
                self.accept(data)
            except socket.timeout:
                continue
            except (ValueError, KeyError, struct.error, OSError) as error:
                self.error = str(error)
                self.bridge.log.submit("chassis_extended.jsonl", dict(event="feedback_decode_error", error=str(error)))

    def snapshot(self):
        try:
            sdk = json.loads((self.root / "wheel_sdk_status.json").read_text())
        except (OSError, ValueError):
            sdk = {}
        return dict(self.bridge.snapshot(sdk), actual_lower_identity=self.identity, receiver_error=self.error)

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join()
        self.sock.close()
        self.bridge.close()
