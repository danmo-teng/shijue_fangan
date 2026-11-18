"""Four-button session controls, shared by the map and headless launcher.

Only the existing C++ owner opens UART. Idle owns UART without SDK/CONFIG;
starting a session first closes that owner. GPIO STOP goes directly to the owner
and the runner control file, independently of map redraw or event processing.
"""
from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import queue
import secrets
import subprocess
import threading
import time

try:
    from .button_input import SysfsButtonInput
    from .operator_protocol import send_abort, send_beep, send_alarm
except ImportError:
    from button_input import SysfsButtonInput
    from operator_protocol import send_abort, send_beep, send_alarm

try:
    from field_model import initial_pose
except ImportError:
    from rescue_map.field_model import initial_pose

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNTIME = PROJECT_ROOT / 'rescue_map/runtime'


def _read_json(path):
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def _write_json(path, document):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(document, ensure_ascii=False) + '\n', encoding='utf-8')
    temporary.replace(path)


class OperatorPanel:
    def __init__(self, app):
        self.app = app
        self.socket_name = '@rescue-operator-' + str(os.getuid())
        control_root = (Path('/dev/shm') / f'rdk-rescue-match-{os.getuid()}'
                        if os.environ.get('RDK_MATCH_RECORDING') == '0' else RUNTIME)
        self.control_path = control_root / 'operator_control.json'
        self.selection_path = RUNTIME / 'operator_selection.json'
        self.stopped = False
        self.transitioning = False
        self.active = False
        self._lock_file = None
        self._standby = None
        self._standby_reported = False
        self._thread = None
        self._quit = threading.Event()
        self._events = queue.Queue(maxsize=32)
        self._state_lock = threading.Lock()
        self._stop_serial = 0
        self._beep_sequence = 0
        self._pending_beep = None
        self._pending_alarm = None
        self._alarm_on = False
        self._ready_feedback = False
        self._sound_lock = threading.RLock()
        self._confirming_prewarm = False
        self._next_ready_beep = 0.0
        self._start_press_token = None
        self.button_log = None
        self.button_trace_run = secrets.token_hex(8)
        self.buttons = SysfsButtonInput(trace=self._trace_button)
        selection = _read_json(self.selection_path)
        if not getattr(app.options, 'zone', None) and selection.get('zone') in (1, 2, 3, 4):
            app.zone = selection['zone']
        if not getattr(app.options, 'side', None) and selection.get('side') in ('red', 'blue'):
            app.side = selection['side']
        app.pose = initial_pose(app.zone, app.corner_offset_m)
        app.selecting = True

    def _record(self, event, source):
        try:
            self.app.record_reset_event(event, source)
        except (OSError, ValueError, AttributeError):
            pass  # Diagnostic output cannot block a button action.

    def _notice(self, text, event='operator_error'):
        self.app.message = text
        self._record(event, text)

    def save_selection(self):
        try:
            RUNTIME.mkdir(parents=True, exist_ok=True)
            _write_json(self.selection_path, dict(zone=self.app.zone, side=self.app.side))
        except OSError as error:
            self._notice(f'按键选择保存失败：{error}')

    def _start_standby(self):
        if self._standby is not None and self._standby.poll() is None:
            return
        options = self.app.options
        command = [str(Path(os.environ.get('RDK_FORMAL_LOCALIZATION_BUILD',PROJECT_ROOT/'localization/build')) / 't265_omni_localizer'),
                   '--operator-uart-only', '--operator-socket', self.socket_name,
                   '--uart', str(options.uart), '--baud', str(options.baud)]
        try:
            self._standby = subprocess.Popen(command, cwd=PROJECT_ROOT / 'localization',
                                              stdout=subprocess.DEVNULL)
            self._standby_reported = False
        except OSError as error:
            self._standby = None
            self._notice(f'待机底盘通信启动失败：{error}')

    def _stop_standby(self):
        process = self._standby
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                # Do not hand UART to another process until its former owner
                # actually exits; this is ownership transfer, not a data gate.
                process.wait(timeout=2)
        self._standby = None

    def _trace_button(self, stage, key, when, **details):
        if self.button_log is not None:
            self.button_log.append(RUNTIME / 'operator_button_edges.jsonl', dict(
                stage=stage, key=key, monotonic_s=when, run=self.button_trace_run,
                session=getattr(self.app, 'reset_session_token', None), **details))

    def start(self):
        if self.active:
            return
        RUNTIME.mkdir(parents=True, exist_ok=True)
        lock_file = (RUNTIME / 'operator.lock').open('a')
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lock_file.close()
            raise RuntimeError('已有实体按键/比赛入口运行，请使用现有入口') from None
        from async_diagnostics import AsyncDiagnosticWriter
        if os.environ.get("RDK_MATCH_RECORDING") != "0":
            self.button_log = AsyncDiagnosticWriter(RUNTIME / "operator_button_logging_status.json")
        self._lock_file = lock_file
        self.active = True
        self._quit.clear()
        self.control_path.parent.mkdir(parents=True, exist_ok=True)
        self._start_standby()
        self.silence()
        self._thread = threading.Thread(target=self._poll_buttons, name='competition-buttons', daemon=True)
        self._thread.start()

    def _enqueue(self, event):
        try:
            self._events.put_nowait(event)
        except queue.Full:
            # Human controls never accumulate indefinitely. STOP was already
            # delivered outside this queue and cannot be lost to a busy UI.
            try:
                self._events.get_nowait()
            except queue.Empty:
                pass
            try:
                self._events.put_nowait(event)
            except queue.Full:
                pass

    def _write_stop(self):
        _write_json(self.control_path, dict(session_token=self.app.reset_session_token,
                                          action='stop', stop_serial=self._stop_serial))

    def request_stop(self):
        # UART first; no acknowledgement or GUI frame is required.
        # This counter has one producer: the GPIO polling thread. Publish the
        # new serial before UART, so an older resume cannot unlock this STOP.
        self._stop_serial += 1
        sent = send_abort(self.socket_name, sequence=self._stop_serial & 255)
        with self._state_lock:
            self.stopped = True
            try:
                self._write_stop()
            except OSError as error:
                self._enqueue(dict(action='input_error', key=4, reason=f'STOP记录失败：{error}'))
        return sent

    def _poll_buttons(self):
        while not self._quit.is_set():
            for event in self.buttons.poll():
                if event['action'] == 'start_pressed':
                    # Capture the readiness of this press, before the GUI can
                    # process a later release after prewarming has completed.
                    self._start_press_token = (self.app.reset_session_token
                        if ((getattr(self.app, 'warm_ready', False) and not self.stopped) or
                            (self.stopped and getattr(self.app, 'session_started', False))) else None)
                    continue
                if event['action'] == 'start':
                    event['_ready_press_token'] = self._start_press_token
                    self._start_press_token = None
                if event['action'] == 'stop':
                    self.request_stop()
                if event['action'] == 'prewarm' and self.stopped and event.get('release_only'):
                    continue  # A polling gap on release cannot discard a stopped match.
                if (event['action'] == 'start' and self.transitioning
                        and not getattr(self.app, 'prewarm_enabled', False)):
                    continue
                event['_stop_serial'] = self._stop_serial
                self._enqueue(event)
            self._quit.wait(.02)

    def _confirm_prewarm(self):
        # Confirm long 3 before archiving or spawning.
        with self._sound_lock:
            self._confirming_prewarm = True
            self._alarm_on = self._ready_feedback = False
            self._beep_sequence = (self._beep_sequence + 1) & 255
            self._pending_alarm = (False, self._beep_sequence)
            self._pending_beep = (1, False, self._beep_sequence)
            self._flush_beep()

    def _flush_beep(self):
        with self._sound_lock:
            return self._send_pending_sound()

    def _send_pending_sound(self):
        if self._pending_alarm is not None:
            enabled, sequence = self._pending_alarm
            if not send_alarm(self.socket_name, enabled, sequence):
                return False
            self._pending_alarm = None
        if self._alarm_on or (self.stopped and not self._confirming_prewarm):
            self._pending_beep = None
            return True
        if self._pending_beep is None:
            return True
        count, long, sequence = self._pending_beep
        if send_beep(self.socket_name,count=count,long=long,sequence=sequence):
            self._pending_beep = None
            return True
        return False

    def _beep(self, count, long=False):
        with self._sound_lock:
            if self._confirming_prewarm:
                return
            if self._alarm_on or self.stopped:
                return
            self._beep_sequence = (self._beep_sequence + 1) & 255
            # One replaceable local notification, never a UART acknowledgment wait.
            self._pending_beep = (count, long, self._beep_sequence)
            if not self._flush_beep():
                self._record('operator_feedback_pending', '本地IPC暂未绑定；保留一条声音请求，不等待下位机回执')

    def set_alarm(self, enabled):
        with self._sound_lock:
            if self._confirming_prewarm:
                if not enabled:
                    return  # Owner cleanup must not cut off the accepted key's tone.
                self._confirming_prewarm = False  # Prewarm failed; the alarm takes priority.
            self._ready_feedback = False
            self._pending_beep = None
            self._alarm_on = bool(enabled)
            self._beep_sequence = (self._beep_sequence + 1) & 255
            self._pending_alarm = (self._alarm_on, self._beep_sequence)
            if not self._flush_beep():
                self._record('operator_feedback_pending', '保留最新警报/消音请求，等待本地串口进程绑定；不等下位回执')

    def silence(self):
        self.set_alarm(False)

    def set_ready_feedback(self, ready):
        ready = bool(ready) and not self.stopped
        if ready and not self._ready_feedback:
            self.silence()
            self._next_ready_beep = 0.0
        elif not ready and self._ready_feedback:
            self.silence()
        self._ready_feedback = ready

    def _ready_heartbeat(self):
        if self._ready_feedback and not self.stopped:
            now = time.monotonic()
            if now >= self._next_ready_beep:
                self._next_ready_beep = now + 3.0
                self._beep(2)

    def _query_zone(self):
        self.app.message = f'当前出发区：{self.app.zone}号'
        self._beep(self.app.zone)

    def _query_side(self):
        red = self.app.side == 'red'
        self.app.message = '当前安全区：' + ('红方' if red else '蓝方')
        self._beep(1 if red else 2, long=True)

    def tick(self):
        self._flush_beep()
        if self._standby is not None and self._standby.poll() is not None and not self._standby_reported:
            self._standby_reported = True
            self._notice(f'待机底盘通信已退出：{self._standby.returncode}；可选择和启动，声音可能不可用')
        # Bound one UI turn even while the input thread is running.
        for _ in range(self._events.maxsize):
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            action = event['action']
            if action == 'input_error':
                self._notice(f"按键{event.get('key', '?')}：{event.get('reason', '读取失败')}")
            elif action == 'input_recovered':
                self._record('operator_input_recovered', f"key{event['key']}")
            elif action == 'stop':
                self.silence()
                self.app.vision_start_pending = False
                self.app.start_requested = False
                message = ('已停车并保留本局；松开4，收纳后摆回原出发位短按3重发'
                           if self.app.session_started else '尚未发车，已停车；松开4，长按3可重开一局并预热')
                if not self.app.session_started and getattr(self.app, 'start_released', False):
                    message = '已发出启动请求，已停车'
                self._notice(message + '；另开新局请松开4后长按3预热', 'operator_stop')
            elif action in ('query_zone', 'next_zone'):
                if action == 'next_zone' and self.app.selecting and not self.transitioning:
                    self.app.zone = self.app.zone % 4 + 1
                    self.app.pose = initial_pose(self.app.zone, self.app.corner_offset_m)
                    self.save_selection()
                self._query_zone()
            elif action in ('query_side', 'toggle_side'):
                if action == 'toggle_side' and self.app.selecting and not self.transitioning:
                    self.app.side = 'blue' if self.app.side == 'red' else 'red'
                    self.save_selection()
                self._query_side()
            elif action == 'prewarm':
                self._trace_button('dispatch',3,event.get('monotonic'),action=action,
                    stopped=self.stopped,selecting=self.app.selecting,transitioning=self.transitioning,
                    same_stop=event.get('_stop_serial') == self._stop_serial)
                self._accept_prewarm(event.get('_stop_serial'))
            elif action == 'start':
                self._trace_button('dispatch', 3, event['monotonic'] if 'monotonic' in event else time.monotonic(),
                    action='start', stopped=self.stopped, selecting=self.app.selecting,
                    session_started=self.app.session_started, transitioning=self.transitioning,
                    same_stop=event.get('_stop_serial') == self._stop_serial,
                    press_token_matches=(event.get('_ready_press_token') is not None and
                                         event.get('_ready_press_token') == self.app.reset_session_token))
                if ((self.app.selecting or (self.stopped and self.app.session_started)) and not self.transitioning and
                        event.get('_stop_serial') == self._stop_serial):
                    if (getattr(self.app, 'prewarm_enabled', False) and
                            (event.get('_ready_press_token') is None or
                             event['_ready_press_token'] != self.app.reset_session_token)):
                        self.app.reject_unready_start()
                    elif self.app.start_session():
                        self._beep(1)
        self._ready_heartbeat()

    def _accept_prewarm(self, serial):
        if not self.app.prewarm_enabled or self.transitioning or serial != self._stop_serial:
            return
        restart = self.stopped
        if not restart and (not self.app.selecting or self.app.session_started or self.app.start_released or self.app.warming):
            return
        self._confirm_prewarm()
        try:
            if restart:
                self.app.reset_session(source='gpio_prewarm_new_round', stop_serial=serial)
            if serial == self._stop_serial and not self.stopped:
                self.app.prepare_warm_session()
        finally:
            with self._sound_lock:
                self._confirming_prewarm = False

    def before_start(self):
        self.silence()
        with self._state_lock:
            if getattr(self.app, 'prewarm_enabled', False) and self.stopped:
                return False  # STOP may arrive after the caller's initial check.
            self.transitioning = True
            self.stopped = False
            self.app.reset_session_token = secrets.token_hex(16)
            try:
                self.control_path.unlink(missing_ok=True)
            except OSError as error:
                self._notice(f'旧按键控制记录清理失败：{error}')
        self.save_selection()
        self._stop_standby()
        # Carry the latest sound state across UART ownership transfer. This is
        # one local request, not a firmware acknowledgment or periodic retry.
        with self._sound_lock:
            if not self._confirming_prewarm:
                self._pending_alarm = (self._alarm_on, self._beep_sequence)
        return True

    def release_start(self, zone, side, opening_strategy):
        # Serialize with the independent GPIO STOP writer: a STOP must never
        # be overwritten by a concurrently prepared START.
        with self._state_lock:
            if self.stopped:
                return False
            _write_json(self.control_path, dict(session_token=self.app.reset_session_token,
                action='start', start_zone=zone, side=side, opening_strategy=opening_strategy,
                reference_token=self.app.reset_session_token))
        return True

    def release_resume(self, zone, side, opening_strategy, reference_token, stop_serial):
        with self._state_lock:
            if not self.stopped or self._stop_serial != stop_serial:
                return False
            _write_json(self.control_path, dict(session_token=self.app.reset_session_token,
                action='resume', start_zone=zone, side=side, opening_strategy=opening_strategy,
                reference_token=reference_token, stop_serial=stop_serial))
            self.stopped = False
        return True

    def after_start(self):
        with self._state_lock:
            if self.stopped:
                self.app.vision_start_pending = False
                try:
                    self._write_stop()
                except OSError as error:
                    self._notice(f'启动期间STOP记录失败：{error}')
            self.transitioning = False

    def after_reset(self, stop_serial=None):
        with self._state_lock:
            self.app.reset_session_token = None
            if stop_serial is None or stop_serial == self._stop_serial:
                self.stopped = False
                try:
                    self.control_path.unlink(missing_ok=True)
                except OSError as error:
                    self._notice(f'按键控制记录清理失败：{error}')
            self.transitioning = False
        if self.active:
            self._start_standby()
        with self._sound_lock:
            if self._confirming_prewarm:
                self._flush_beep()  # If no owner was bound, deliver through standby.
            else:
                self.silence()

    def close(self):
        self.silence()
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=1)
            self._thread = None
        try:
            self._stop_standby()
        finally:
            if self._lock_file is not None:
                fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
                self._lock_file.close()
                self._lock_file = None
            if self.button_log is not None:
                self.button_log.close()
                self.button_log = None
            self.active = False
