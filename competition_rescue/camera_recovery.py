"""Fault-only node polling and recovery milestones; no healthy-stream polling."""
from pathlib import Path


class CameraRecovery:
    NODE_POLL_NS = 200_000_000

    def __init__(self, events, clock):
        self.events, self.clock = events, clock
        self.active = False
        self.incident = 0

    def begin(self, device, error, context=None):
        if self.active:
            return
        self.active = True
        self.incident += 1
        self.device = device
        self.started_ns = self.clock()
        self.next_probe_ns = self.next_open_ns = 0
        self.present = None
        self.last_missing_ns = self.node_seen_ns = None
        self.first_frame = None
        self.open_attempts = 0
        self.last_resolution_error = None
        self.events.write('camera_recovery_started', dict(
            recovery_id=self.incident, device=device, error=str(error),
            error_observed_monotonic_ns=self.started_ns, **(context or {})))

    def observe_node(self, device, present):
        now = self.clock()
        self.device = device
        if not present:
            self.last_missing_ns = now
        if present != self.present:
            if present:
                self.node_seen_ns = now
            self.events.write('camera_node_present' if present else 'camera_node_missing', dict(
                recovery_id=self.incident, device=device,
                node_observed_monotonic_ns=now,
                last_missing_monotonic_ns=self.last_missing_ns,
                observation_gap_ms=(None if not present or self.last_missing_ns is None else
                                    (now-self.last_missing_ns)/1e6),
                meaning='first observed existence, not exact physical reconnect time'))
        self.present = present
        return now

    def ready(self, device, now_ns):
        if now_ns < self.next_probe_ns:
            return False
        # Called only while no camera is open. Failed pipeline starts retain
        # their original slower retry while node transitions are still observed.
        if device is not None:
            try:
                present = Path(device).exists()
            except OSError:
                # Preserve open_camera's existing exception handling; a failed
                # stat is not evidence that the node physically disappeared.
                self.next_probe_ns = now_ns + self.NODE_POLL_NS
                return now_ns >= self.next_open_ns
            now_ns = self.observe_node(device, present)
        self.next_probe_ns = now_ns + self.NODE_POLL_NS
        return (device is None or self.present) and now_ns >= self.next_open_ns

    def open_started(self):
        start = self.clock()
        self.open_attempts += 1
        self.events.write('camera_open_attempt', dict(recovery_id=self.incident,
            attempt=self.open_attempts, device=self.device, open_started_monotonic_ns=start))
        return start

    def open_finished(self, start, error=None, retry_s=2.):
        end = self.clock()
        self.events.write('camera_open_finished', dict(recovery_id=self.incident,
            attempt=self.open_attempts, device=self.device, success=error is None,
            open_started_monotonic_ns=start, open_finished_monotonic_ns=end,
            open_duration_ms=None if start is None else (end-start)/1e6,
            error=None if error is None else str(error)))
        if error is not None:
            self.next_open_ns = end + int(retry_s*1e9)

    def observe_frame(self, packet):
        if not self.active or self.first_frame is not None:
            return
        self.first_frame = dict(first_frame_observed_monotonic_ns=self.clock(),
            first_frame_published_monotonic_ns=packet.published_ns,
            first_frame_id=packet.frame_id, first_frame_open_attempt=self.open_attempts)
        self.events.write('camera_first_frame_observed', dict(
            recovery_id=self.incident, **self.first_frame,
            meaning='first decoded frame observed by runner; a latest-frame camera may skip earlier frames'))

    def finish(self, packet):
        self.observe_frame(packet)
        end = self.clock()
        details = dict(recovery_id=self.incident, error_observed_monotonic_ns=self.started_ns,
            last_missing_monotonic_ns=self.last_missing_ns,
            node_present_observed_monotonic_ns=self.node_seen_ns,
            result_ready_monotonic_ns=end, total_recovery_ms=(end-self.started_ns)/1e6,
            result_frame_published_monotonic_ns=packet.published_ns,
            open_attempts=self.open_attempts, **self.first_frame)
        self.active = False
        return details
