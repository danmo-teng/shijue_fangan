"""Formal transport around the unchanged desktop DynamicFence/RadarWorker chain.

No chassis UART, SDK, motor commands, position blending or time admission gates.
The sole C++ owner publishes both the JSON pose and its matching 0x16 frame.
"""
import json
import queue
import socket
import threading
import time

from f405_timed_measurements import TimedMeasurements
from lidar_yaw_debug.inputs import forward_motion_event
from lidar_yaw_debug.worker import RadarWorker
from low_latency_scheduling import set_realtime_thread
from local_packet_receiver import LocalPacketReceiver
from lidar_yaw_debug.control_packet import format_control_packet


class CompetitionDynamicLocalization:
    def __init__(self, root, session_id, config, field_start, log, *, defer_reference=False, native_feedback=False):
        self.root, self.session_id, self.config = root, session_id, config
        self.field_start, self.log = field_start, log
        self.radar = RadarWorker(control_updates=True)
        self.native_feedback = bool(native_feedback)
        if self.native_feedback:self.radar.enable_native_feedback()
        self.clock_map = TimedMeasurements()
        self.timed_yaw_seen = False
        self.last_yaw_id = None
        self.imu_packets = self.pose_packets = 0
        self.chassis_inbox = queue.Queue(maxsize=2048)
        self.pose_address = '\0competition-vio-' + session_id
        self.control_address = '\0competition-pose-' + session_id
        self.pose_socket = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.pose_socket.bind(self.pose_address)
        self.pose_socket.listen(1)
        self.pose_socket.settimeout(.005)
        self.output = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.output.setblocking(False)
        self.stop_event = threading.Event()
        self.thread = None
        self.input_threads = []
        self.reference_requested = False
        self.defer_reference = defer_reference
        self.error = None
        self.state = {}
        self.control_state = {}
        self.scheduling = {}
        self.latest_pose = {}
        self.input_drops = self.output_drops = 0
        self.lidar_enabled = True
        self.packet_id = 0
        self.last_control = None
        self.start_request = None
        self.start_request_sent = False

    def devent(self, event, **data):
        self.log.submit('dynamic_events.jsonl', dict(event=event,
            timestamp_monotonic_ns=time.monotonic_ns(), **data))

    def telemetry(self, event):
        if self.native_feedback:
            if (event['kind']=='imu' and
                    self.radar.state.get('native_feedback',{}).get('timed_yaw_seen')):
                return  # Same worker already uses versioned yaw; adapter also retains the original rule.
            # Complete 07/08 frames already go from the sole C++ UART owner to
            # the motion worker. Only matched identity/clock context and the
            # existing legacy-yaw fallback pass through this diagnostic parent.
            if event['kind'] in ('timed_identity','timed_clock_sync','device_identity_changed','imu'):
                if not self.radar.send('feedback_context',event):self.input_drops += 1
            return
        try:
            self.chassis_inbox.put_nowait(event)
        except queue.Full:
            self.input_drops += 1

    def start(self):
        self.radar.start(self.config, self.config['lidar']['port'], self.config['lidar']['baud'], self.root,
            sdk_policy=dict(session_id=self.session_id,address='\0t265-wheel-'+self.session_id),
            control_output=(dict(session_id=self.session_id,address=self.control_address)
                            if self.native_feedback else None),
            pose_input=(dict(socket=self.pose_socket,session_id=self.session_id)
                        if self.native_feedback else None))
        self.devent('realtime_permission_setup',**self.radar.scheduling_setup)
        self.input_threads = [threading.Thread(target=self._publish_status, name='dynamic-display-status', daemon=True)]
        if not self.native_feedback:
            self.input_threads.insert(0,threading.Thread(target=self._read_pose,name='dynamic-pose-input',daemon=True))
            self.input_threads.insert(0,threading.Thread(target=self._read_chassis,
                name='dynamic-motion-input',daemon=True))
        for thread in self.input_threads:
            thread.start()
        self.thread = threading.Thread(target=self._run, name='competition-dynamic-transport', daemon=True)
        self.thread.start()

    def set_lidar_enabled(self, enabled):
        if self.radar.send('lidar_enabled', dict(enabled=bool(enabled), reason='operator_obstacle_toggle')):
            self.lidar_enabled = bool(enabled)

    def request_start(self, field_start, round_token):
        # One local coordinate bind; no MCU handshake or data-age policy.
        self.start_request = dict(center_xy_m=[field_start['x_m'],field_start['y_m']],
            yaw_deg=field_start['yaw_deg'],first_sample=False,round_token=round_token,
            description='operator start: bind current warm Pose and gyro to selected formal start')
        self.start_request_sent = False
        self.radar.control.wake()

    def control_packet(self, state):
        packet=format_control_packet(state,self.latest_pose,self.session_id,self.packet_id,
            self.last_control,lidar_state=self.control_state)
        if state.get('ready'):self.last_control=packet
        return packet

    def _read_chassis(self):
        self._schedule_thread('motion_input')
        while not self.stop_event.is_set():
            try:
                event = self.chassis_inbox.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                forward_motion_event(self, event)
            except (ValueError, KeyError, TypeError) as exc:
                self.devent('motion_input_error', reason=str(exc))

    def _read_pose(self):
        self._schedule_thread('pose_input')
        connection = None
        receiver = None
        try:
            while not self.stop_event.is_set():
                try:
                    if connection is None:
                        connection, _ = self.pose_socket.accept()
                        connection.settimeout(.005)
                        connection.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
                        receiver = LocalPacketReceiver(connection, 16384)
                    data = receiver.recv()
                    if data:
                        pose = json.loads(data)
                        if pose['session_id'] == self.session_id:
                            self.latest_pose = pose
                            self.radar.send('pose', pose)
                            self.pose_packets += 1
                            self.log.submit('pose_motion.jsonl', pose)
                    else:
                        connection.close()
                        connection = None
                except socket.timeout:
                    continue
                except (ValueError, KeyError, TypeError) as exc:
                    self.devent('pose_input_error', reason=str(exc))
        except OSError as exc:
            if not self.stop_event.is_set():
                self.devent('pose_input_error', reason=str(exc))
        finally:
            if connection is not None:
                connection.close()

    def _run(self):
        self._schedule_thread('control_output')
        try:
            while not self.stop_event.is_set():
                value = self.radar.control.get(timeout=.05)
                if self.native_feedback:
                    # Control goes straight from worker to C++; this loop only
                    # handles display/reference requests at the existing 20Hz.
                    value=self.radar.state or None
                if self.stop_event.is_set():
                    break
                if value is not None:
                    self.control_state = value
                    self.state = value['last']
                    if self.native_feedback:
                        direct=value.get('direct_output',{})
                        self.packet_id=direct.get('packets_sent',self.packet_id)
                        self.output_drops=direct.get('output_drops',self.output_drops)
                        if direct.get('last_packet') is not None:self.last_control=direct['last_packet']
                state = self.control_state
                if (not self.defer_reference and not self.reference_requested and state.get('have_tracked_pose')
                        and state.get('have_primary_yaw')):
                    self.reference_requested = self.radar.send('field_start', dict(
                        center_xy_m=[self.field_start['x_m'], self.field_start['y_m']],
                        yaw_deg=self.field_start['yaw_deg'],
                        description='selected formal competition start; operator selects zone; not lidar calibration'))
                if (self.start_request is not None and not self.start_request_sent
                        and state.get('have_tracked_pose') and state.get('have_primary_yaw')):
                    self.start_request_sent = self.radar.send('field_start', self.start_request)
                if self.radar.process.exitcode is not None:
                    self.error = self.error or 'radar worker exited'
                    break  # Do not invent a replacement position or send STOP.
                if value is None:
                    continue  # Idle wakeups/start requests are not sensor updates.
                if self.native_feedback:
                    continue  # Worker is the only control producer; parent only observes/binds the start.
                packet = self.control_packet(self.state)
                if packet is not None:
                    try:
                        self.output.sendto(json.dumps(packet, allow_nan=False).encode(), self.control_address)
                        self.log.submit('control_pose.jsonl', packet)
                        self.packet_id += 1
                    except (BlockingIOError, FileNotFoundError, ConnectionRefusedError):
                        self.output_drops += 1
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.error = str(exc)
            self.devent('dynamic_transport_error', reason=self.error)

    def _publish_status(self):
        # Display/fit details remain 20Hz; neither GUI polling nor JSON file
        # snapshots can hold the event-driven control consumer here.
        while not self.stop_event.wait(.05):
            for reply in self.radar.poll():
                if reply['kind'] == 'error':
                    self.error = reply['error']
                    self.devent('worker_error', reason=self.error)
            self.log.submit('dynamic_status.json', self.snapshot(), atomic=True)

    def _schedule_thread(self, role):
        result=set_realtime_thread()
        self.scheduling[role]=result
        self.devent('thread_scheduling',role=role,**result)

    def snapshot(self):
        control = self.state
        display = self.radar.state.get('last', {})
        # Do not draw scan points from the previous round/reference over a new
        # high-frequency control pose while the display snapshot catches up.
        state = dict(display) if display.get('reference') == control.get('reference') else dict(points_field=[], last_fit=None)
        state.update(control)
        if self.defer_reference and self.start_request is None:
            state.update(reason='预热接收中；等待按3绑定正式起点',reference_phase='prewarm_waiting_for_operator')
        return dict(state, transport_error=self.error, pose_packets=(self.radar.state.get('pose_ingested',0) if self.native_feedback else self.pose_packets),
            have_tracked_pose=bool(self.control_state.get('have_tracked_pose')),
            have_primary_yaw=bool(self.control_state.get('have_primary_yaw')),
            lidar_connected=self.control_state.get('lidar_connected',False),
            lidar_error=self.control_state.get('lidar_error'),
            primary_yaw_packets=(self.radar.state.get('primary_yaw_ingested',0)
                if self.native_feedback else self.imu_packets),input_drops=self.input_drops,
            native_feedback=self.radar.state.get('native_feedback',{}) if self.native_feedback else None,
            worker_input_drops=self.radar.input_drops, output_drops=self.output_drops,
            control_packets_sent=self.packet_id, control_pose_publish_enabled=True,
            thread_scheduling=dict(self.scheduling),
            accuracy_verified=False)

    def close(self):
        self.stop_event.set()
        self.radar.control.wake()
        if self.thread:
            self.thread.join()
        for thread in self.input_threads:
            thread.join()
        self.radar.close()
        if self.native_feedback:
            # End-of-session counters come from the stopped owner, not the
            # last 20Hz UI observation. This is reporting, never an input gate.
            direct=self.radar.state.get('direct_output') or {}
            self.packet_id=direct.get('packets_sent',self.packet_id)
            self.output_drops=direct.get('output_drops',self.output_drops)
            if direct.get('last_packet') is not None:self.last_control=direct['last_packet']
        self.pose_socket.close()
        self.output.close()
        self.log.submit('dynamic_summary.json', dict(self.snapshot(),
            worker_stopped_cleanly=self.radar.stopped_cleanly,
            worker_summary=self.radar.state), atomic=True)
