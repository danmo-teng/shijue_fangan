#!/usr/bin/env python3
"""Map/start selector for the independent complete competition flow.

The formal C++ process owns UART and SDK. Official wheel input reuses the
existing test bridge; the field start and mission/task pairing stay formal.
"""
from __future__ import annotations

import json
import os
import secrets
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT / "rescue_map"))
sys.path.insert(0, str(PROJECT_ROOT / "localization/tools"))

from map_app import RUNTIME, WINDOW_NAME, RescueMapApp, parse_args  # noqa: E402
from competition_wheel_input import CompetitionWheelInput
from competition_dynamic_localization import CompetitionDynamicLocalization
from lidar_yaw_debug.diagnostic import load_config
from lidar_yaw_debug.motion_source import POLICY as MOTION_SOURCE_POLICY
from motion_log_storage import read_json, WHEEL_SESSION_FILES
from operator_panel import OperatorPanel
from field_model import initial_pose

WHEEL_OWNER = "competition_wheel_input"
WHEEL_FILES = WHEEL_SESSION_FILES
VISION_WINDOW_NAME = "complete_rescue_competition"


class CompetitionMapApp(RescueMapApp):
    """Reuse the existing selector and map, but launch the new task runner."""

    def __init__(self, options) -> None:
        super().__init__(options)
        self.reset_request_path = RUNTIME / "startup_reset_request.json"
        self.reset_session_token = None
        self.dynamic_localization = getattr(options, "dynamic_localization", False)
        self.dynamic = None
        self.official_wheel_input = self.dynamic_localization or getattr(options, "official_wheel_input", False)
        self.wheel_input = None
        self.wheel_run_dir = None
        self.pose_age_gate = False
        self.operator = None
        self.prewarm_enabled = bool(getattr(options, 'gpio_buttons', False) and self.dynamic_localization
            and not options.demo and not options.screenshot)
        self.warming = False
        self.start_requested = False
        self.start_released = False
        self.warm_ready_announced = False
        self.warm_error = None
        self.window_exit_requested = False
        self.warm_status_path = RUNTIME / 'operator_warm_status.json'
        if self.localization_mode == "t265" or self.official_wheel_input:
            # Mounting is independent of SDK wheel injection. The normal writer
            # still sets the selected competition corner/heading, not the test centre.
            self.localization_config_template = PROJECT_ROOT / "localization/config/t265_debug_new_mount_candidate.conf"
        if self.official_wheel_input:
            self.localization_mode = "t265"
            self.encoder_weight = 0.0
            self.t265_translation_scale_enabled = False
            self.t265_translation_scale = 1.0
            self.t265_map_enabled = False
        if (getattr(options, 'gpio_buttons', False) and not options.demo and not options.screenshot):
            self.operator = OperatorPanel(self)

    def prepare_warm_session(self):
        """Start the one SDK/UART and vision owners, with command production disarmed."""
        if self.warming or self.session_started:
            return
        self.warm_error = None
        self.start_requested = self.start_released = self.warm_ready_announced = False
        self.warm_status_path.unlink(missing_ok=True)
        try:
            self.operator.before_start()
            super().start_session()
            if not self.session_started:
                raise RuntimeError(self.message)
            self.warming = True
            # Base start prepares all existing session/log paths. Warm vision
            # starts independently of Pose so USB and model loading overlap.
            self.session_started = False
            self.selecting = True
            self.vision_start_pending = False
            if self.options.launch_vision:
                self.vision_process = self.start_owned_process(self.vision_command(), PROJECT_ROOT / 'mission_test')
            self.message = '预热T265、雷达和视觉；可选区，按3一次记录开赛请求'
            self.record_reset_event('prewarm_started', 'same_sdk_session_until_start')
        except Exception as error:
            self._warm_failed(str(error))
        finally:
            self.operator.after_start()

    def _warm_failed(self, reason):
        self.warm_error = reason
        self.start_requested = self.start_released = self.warming = False
        self.session_started = False
        self.selecting = True
        self.stop_session_processes()
        self.operator._start_standby()
        self.message = f'预热失败：{reason}；修复后按3重试，未发送开赛命令'
        self.record_reset_event('prewarm_failed', reason)
        self.operator._beep(3, long=True)

    def _update_warm_start(self):
        for name, process in (('定位', self.localization_process), ('视觉', self.vision_process)):
            if process is not None and process.poll() is not None:
                self._warm_failed(f'{name}进程退出({process.returncode})')
                return
        if self.dynamic is not None and getattr(self.dynamic, 'error', None):
            self._warm_failed(f'动态定位准备失败：{self.dynamic.error}')
            return
        status = read_json(self.warm_status_path)
        arrivals = {
            'T265 Pose': bool(self.dynamic is not None and self.dynamic.radar.state.get('have_tracked_pose')),
            '主yaw': bool(self.dynamic is not None and self.dynamic.radar.state.get('have_primary_yaw')),
            '视觉推理': bool(status.get('session_token') == self.reset_session_token and status.get('ready')),
        }
        ready = all(arrivals.values())
        if ready and not self.warm_ready_announced:
            self.warm_ready_announced = True
            self.operator._beep(1)
            self.record_reset_event('prewarm_ready', 'one_short_beep; SDK Pose, gyro and actual inference present')
        if self.operator.stopped:
            self.start_requested = False
            self.message = '已请求停止；长按4退出后自动重新预热'
            return
        if self.start_requested and ready and not self.start_released:
            selected = initial_pose(self.zone, self.corner_offset_m)
            start = dict(x_m=selected.x_m, y_m=selected.y_m, yaw_deg=selected.yaw_deg)
            self.dynamic.request_start(start, self.reset_session_token)
            metadata = read_json(RUNTIME / 'session.json')
            metadata.setdefault('warm_selection',dict(start_zone=metadata.get('start_zone'),side=metadata.get('side'),initial_pose=metadata.get('initial_pose')))
            metadata.update(start_zone=self.zone,side=self.side,opening_strategy=self.opening_strategy,
                actual_start_selection=dict(start_zone=self.zone,side=self.side,initial_pose=start),
                initial_pose=start,round_reference_token=self.reset_session_token)
            try:
                temporary = RUNTIME / 'session.json.tmp'
                temporary.write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
                temporary.replace(RUNTIME / 'session.json')
            except OSError as error:
                self.record_reset_event('session_metadata_write_failed', str(error))
            if self.wheel_input is not None:
                self.wheel_input.bridge.log.submit('metadata.json',metadata,atomic=True)
            # The runner requires this exact local reference in final Pose JSON
            # before CONFIG; it never waits for a lower-side acknowledgment.
            try:
                self.start_released = self.operator.release_start(self.zone, self.side, self.opening_strategy)
            except OSError as error:
                self._warm_failed(f'开赛控制文件写入失败：{error}')
                return
            self.record_reset_event('warm_start_requested', 'bind_latest_pose_and_primary_yaw')
        if self.start_released and status.get('competition_started'):
            self.warming = False
            self.session_started = True
            self.selecting = False
            self.started_monotonic = time.monotonic()
            self.trajectory.reset()
            self.odometry_trajectory.reset()
            self.message = '本轮已启动；SDK会话保持，正式起点已按按3时刻重建'
            return
        if not ready:
            received = '、'.join(name for name, present in arrivals.items() if present) or '无'
            missing = '、'.join(name for name, present in arrivals.items() if not present)
            request_note = '已记录开赛请求，4号可取消' if self.start_requested else '按3可提前记录开赛请求'
            self.message = f'预热已到：{received}；待到：{missing}。{request_note}'
        elif self.start_requested:
            self.message = '已记录开赛请求，正在绑定正式起点；4号可取消'
        else:
            self.message = '预热完成：按3开始'

    def start_session(self):
        if self.prewarm_enabled:
            if self.session_started:
                return
            if self.operator.stopped:
                self.message = '本轮已停止；长按4退出后可重新开始'
                return
            if not self.warming:
                self.prepare_warm_session()
            if self.warming:
                self.start_requested = True
                self.selecting = False
                self.record_reset_event('operator_start_requested', 'gpio_or_map; no SDK restart')
            return
        if self.session_started:
            return  # A new round starts after explicit exit/reset, never resumes old actions.
        try:
            if self.operator is not None:
                self.operator.before_start()
            super().start_session()
        except Exception as error:
            if self.operator is None:
                raise
            self.session_started = False
            self.selecting = True
            self.message = f"比赛准备失败：{error}；修复后可再次按3号启动"
            self.record_reset_event("session_start_failed", str(error))
        finally:
            if self.operator is not None:
                self.operator.after_start()
        if self.operator is not None and self.selecting and not self.session_started:
            self._restore_operator_standby()

    def _restore_operator_standby(self):
        """Release failed session owners before handing UART back to standby."""
        self.operator.transitioning = True
        try:
            self.stop_session_processes()
            self.operator.after_reset()
        finally:
            self.operator.transitioning = False

    def maybe_start_vision(self):
        if self.prewarm_enabled:
            return
        was_started = self.session_started
        super().maybe_start_vision()
        # The base launcher can return to selection after an asynchronous
        # localizer exit or a failed vision spawn. Finish that failed session,
        # including its wheel/lidar adapters, before restoring feedback UART.
        if (self.operator is not None and was_started and
                self.selecting and not self.session_started):
            self._restore_operator_standby()

    def run(self):
        try:
            if self.operator is not None:
                self.operator.start()
                if self.prewarm_enabled:
                    self.prepare_warm_session()
            return super().run()
        finally:
            # Also covers setup/window errors before the base run-loop finally.
            try:
                self.stop_session_processes()
            finally:
                if self.operator is not None:
                    self.operator.close()

    def localization_command(self) -> list[str]:
        if self.wheel_input is not None:
            return list(self.wheel_command)
        command = super().localization_command()
        if self.operator is not None:
            command += ['--operator-socket', self.operator.socket_name]
        if not self.official_wheel_input:
            return command
        root = RUNTIME / "history" / WHEEL_OWNER
        root.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S") + "_competition_wheel"
        run_dir, suffix = root / stamp, 0
        while run_dir.exists():
            suffix += 1
            run_dir = root / f"{stamp}_{suffix:02d}"
        run_dir.mkdir()
        session_id = secrets.token_hex(16)
        self.wheel_run_dir = run_dir
        config_path = run_dir / "localization.conf"
        shutil.copyfile(RUNTIME / "localization.conf", config_path)
        config = dict(line.split("#", 1)[0].strip().split("=", 1)
                      for line in config_path.read_text().splitlines() if "=" in line.split("#", 1)[0])
        config = {k.strip(): v.strip() for k, v in config.items()}
        self.wheel_input = CompetitionWheelInput(run_dir, config, session_id)
        metadata = read_json(RUNTIME / "session.json")
        # The old formal template's expected F407 identity is not actual F405.
        for key in ("expected_lower_firmware", "expected_lower_mode"):
            metadata.pop(key, None)
        metadata.update(localization_log_dir=str(run_dir), owner=WHEEL_OWNER, mode="competition_t265_wheel_input", sdk_session_id=session_id,
                        wheel_input_requested=True, actual_lower_firmware="unconfirmed", actual_lower_mode="unconfirmed",
                        lower_identity_source="chassis_identity.json: actual UART reply, when received",
                        pose_source="t265_direct_with_camera_offset", encoder_fusion_weight=0.0,
                        pose_dependencies=["T265 SDK", "measured wheel feedback", "main gyro angular rate"],
                        wheel_processing_owner="native_cpp_sdk_owner",
                        wheel_observer_transport="bounded_SEQPACKET_with_native_receive_timestamp",
                        wheel_reference_note="SDK input dependency, not independent position truth",
                        fresh_session_note="map preservation disabled; not proof of device estimator reset")
        metadata["heading_conventions"]["wheel_reference"] = "no legacy F407 wheel projection; measured F405 velocity is SDK input"
        metadata["heading_conventions"]["f407_raw_imu"] = "F405 0x08 measured angular rate; gyro CCW positive"
        # The runner also reads session.json: do not leave an old expected F407
        # identity there while the actual UART identity is recorded separately.
        session_tmp = RUNTIME / "session.json.tmp"
        session_tmp.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
        session_tmp.replace(RUNTIME / "session.json")
        if self.dynamic_localization:
            dynamic_config = load_config(self.options.dynamic_config)
            dynamic_config['lidar'].update(port=self.options.lidar, baud=self.options.lidar_baud)
            if self.options.lidar == self.options.uart:
                raise ValueError("雷达与底盘不能共用同一UART")
            self.dynamic = CompetitionDynamicLocalization(run_dir, session_id, dynamic_config,
                metadata['initial_pose'], self.wheel_input.bridge.log,defer_reference=self.prewarm_enabled,
                native_feedback=True)
            self.wheel_input.telemetry_sink = self.dynamic.telemetry
            metadata['heading_conventions']['pose'] = 'primary gyro, fixed selected field start reference, CCW positive'
            metadata['pose_dependencies'].append('STL50B2 observable fence constraints')
            metadata.update(localization_chain='desktop_DynamicFence_RadarWorker_MotionSource',
                pose_source='lidar_t265_measured_wheel_fallback',
                initial_reference_policy='selected_formal_start_no_wall_wait',
                lidar_installation=dynamic_config['installation'],
                qualification=dynamic_config['qualification'],
                control_pose_publish_enabled=True, accuracy_verified=False,
                motion_feedback_transport='native_cpp_complete_frames_direct_to_motion_worker',
                control_pose_producer='motion_worker_direct_to_sole_cpp_relay',
                motion_source_policy=dict(MOTION_SOURCE_POLICY),
                obstacle_policy='operator_lidar_pause_resume_same_as_desktop_test',
                prewarm_enabled=self.prewarm_enabled,
                raw_projected_pose_frame='warm_SDK_local_projection' if self.prewarm_enabled else 'field',
                round_start_reference='latest_warm_pose_and_primary_yaw_on_operator_start' if self.prewarm_enabled else 'first_pose')
            session_tmp.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
            session_tmp.replace(RUNTIME / "session.json")
            self.dynamic.start()
        self.wheel_input.bridge.log.submit("metadata.json", metadata, atomic=True)
        self.wheel_input.start()
        # Same SDK path/flags as the test; only UART ownership and field start differ.
        for flag, value in (("--config", config_path), ("--csv", run_dir / "localization_debug.csv"),
                            ("--events", run_dir / "localization_events.jsonl")):
            command[command.index(flag) + 1] = str(value)
        command += ["--ignore-encoders", "--debug-fresh-session", "--debug-test-session", session_id,
                    "--raw-pose-csv", str(run_dir / "t265_raw_pose.csv"),
                    "--debug-wheel-socket", "@t265-wheel-" + session_id,
                    "--debug-wheel-config", str(run_dir / "wheel_sdk_config.json"),
                    "--debug-wheel-log", str(run_dir / "wheel_sdk.jsonl"),
                    "--debug-wheel-session", session_id,
                    "--wheel-feedback-socket", "@" + self.wheel_input.address[1:],
                    "--wheel-feedback-requests", str(self.wheel_input.requests_path)]
        if self.dynamic is not None:
            # C++ raw output is diagnostic only. Its dynamic relay writes the
            # formal JSON and 0x16 from the exact same worker snapshot.
            command[command.index('--output') + 1] = str(run_dir / 'raw_t265_result.json')
            command += ['--motion-feedback-socket','@' + self.dynamic.radar.feedback_address[1:],
                        '--debug-pose-socket', '@' + self.dynamic.pose_address[1:],
                        '--dynamic-pose-socket', '@' + self.dynamic.control_address[1:],
                        '--dynamic-pose-output', str(self.localization_json)]
        self.wheel_command = list(command)
        return command

    def wheel_input_status(self):
        return self.wheel_input.snapshot() if self.wheel_input else {}

    def draw_panel_heading(self, text):
        if self.selecting or not self.official_wheel_input:
            return super().draw_panel_heading(text)
        text.add("救援场地定位", (920, 45), 30, (245, 245, 245), True)
        identity = self.wheel_input.identity if self.wheel_input is not None else None
        if not identity:
            text.add("下位机实际身份：unconfirmed", (920, 80), 16, (200, 210, 225))
            text.add("尚未收到实际身份回包", (920, 105), 14, (165, 165, 170))
            return
        if identity['mode_name'].startswith('unknown'):
            role = f"unknown B{identity['board']} M{identity['mode']} S{identity['service']}"
        else:
            role = f"{identity['board_name']} {identity['mode_name']}/{identity['service_name']}"
        text.add("实际：" + role, (920, 80), 15, (200, 210, 225))
        build = (f"{identity['git_commit'][:8]} dirty={identity['source_dirty']} · "
                 f"v{identity['protocol_major']}.{identity['protocol_minor']} · {identity['uart_baud']}")
        text.add(build, (920, 105), 13, (165, 165, 170))

    def stop_localization(self) -> None:
        process = self.localization_process
        super().stop_localization()
        adapter = self.wheel_input
        self.wheel_input = None
        if adapter is None:
            return
        dynamic = self.dynamic
        self.dynamic = None
        dynamic_complete = True
        if dynamic is not None:
            adapter.telemetry_sink = None
            dynamic.close()
            dynamic_complete = (dynamic.radar.stopped_cleanly and dynamic.error is None
                and dynamic.radar.state.get('logging_healthy') is True)
        status = adapter.snapshot()
        if dynamic is not None:
            status['dynamic'] = dynamic.snapshot()
        adapter.bridge.log.submit("final_status.json", status, atomic=True)
        adapter.close()
        # Keep the completion boundary for reviewed, module-local manual
        # cleanup. Expanded SD storage no longer prunes by session count.
        try:
            run_dir = adapter.root
            required = ("metadata.json", "localization_debug.csv", "t265_raw_pose.csv", "final_status.json")
            replay_path = run_dir / 'vision_replay_manifest.json'
            replay_complete = not replay_path.exists() or read_json(replay_path).get('complete') is True
            if (replay_complete and dynamic_complete and process is not None and process.returncode == 0 and adapter.bridge.log.healthy()
                    and status.get("sdk_logging_healthy") is True
                    and all((run_dir / n).is_file() and (run_dir / n).stat().st_size > 0 for n in required)):
                markers = [read_json(p) for p in run_dir.parent.glob("*/session_complete.json")]
                index = max((m.get("session_index", 0) for m in markers if m.get("owner") == WHEEL_OWNER), default=0) + 1
                (run_dir / "session_complete.json").write_text(json.dumps(dict(owner=WHEEL_OWNER,
                    complete=True, session_index=index, accuracy_verified=False)) + "\n")
        except (OSError, ValueError, TypeError) as error:
            print(f"轮速会话收尾失败（原始文件保留）：{error}", file=sys.stderr)

    def draw_panel(self, canvas, text):
        super().draw_panel(canvas, text)
        if self.operator is not None:
            y = 890 if self.selecting else 570
            for line in ('实体键：1长按选起点 / 2长按选红蓝',
                         '3启动 / 4按下停止，长按退出本轮',
                         '1、2短按：声音查询当前选择'):
                text.add(line, (920, y), 14, (200, 210, 225))
                y += 25
        if self.dynamic is not None and not self.selecting:
            state = self.dynamic.snapshot()
            mode = state.get('motion_selection', {}).get('mode', 'WAIT')
            label = {'WHEEL_FALLBACK': '轮速＋主yaw降级',
                     'T265_PRIMARY_WHEEL_SUSPECT': 'T265＋主yaw（轮式可疑）',
                     'T265_PHASE_NO_WHEEL': 'T265＋主yaw（安全区停轮式）',
                     'T265_WITH_WHEEL_TAIL': 'T265＋轮反馈续推'}.get(mode, mode)
            axes = '/'.join(state.get('current_wall_axes', [])) or '暂无约束'
            text.add('当前来源：' + label, (920, 455), 17, (80, 205, 245))
            radar_text = ('雷达断开重连中；运动传播继续' if state.get('lidar_error')
                          else '雷达修正：' + axes)
            text.add(radar_text, (920, 490), 17, (80, 205, 245))
            self.button(canvas, text, 'lidar_toggle',
                '越障：暂停雷达 [L]' if self.dynamic.lidar_enabled else '恢复雷达修正 [L]',
                (920, 515, 1245, 555), self.dynamic.lidar_enabled)
            if state.get('transport_error'):
                self.message = '定位链错误：' + state['transport_error']
        self.button(canvas, text, 'window_minimize', '最小化', (920, 6, 1000, 40))
        self.button(canvas, text, 'window_fullscreen',
            '窗口' if self.fullscreen else '全屏', (1006, 6, 1066, 40))
        self.button(canvas, text, 'window_vision', '识别窗口', (1072, 6, 1176, 40))
        self.button(canvas, text, 'window_exit', '退出',
            (1182, 6, 1245, 40), True, (55, 55, 160))

    def show_vision_window(self):
        xdotool = shutil.which('xdotool')
        if xdotool is None:
            self.message = '当前桌面无法调出识别窗口，请使用系统窗口列表'
            return
        try:
            result = subprocess.run(
                [xdotool, 'search', '--name', f'^{VISION_WINDOW_NAME}$',
                 'windowmap', 'windowraise', 'windowactivate'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            if result.returncode:
                self.message = '识别窗口尚未打开或已关闭；识别进程未重启'
        except OSError as error:
            self.message = f'识别窗口置前未成功：{error}'

    def minimize_window(self):
        # Restrict the desktop request to this process's map window. Hiding
        # the GUI must not stop or reset the running competition session.
        xdotool = shutil.which('xdotool')
        if xdotool is None:
            self.message = '最小化不可用：系统缺少xdotool，可点“窗口”使用系统标题栏'
            return
        try:
            result = subprocess.run(
                [xdotool, 'search', '--pid', str(os.getpid()), '--name',
                 f'^{WINDOW_NAME}$', 'windowminimize'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            if result.returncode:
                self.message = '窗口最小化未成功，可点“窗口”使用系统标题栏'
        except OSError as error:
            self.message = f'窗口最小化未成功：{error}'

    def toggle_window_fullscreen(self):
        self.fullscreen = not self.fullscreen
        cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL)
        if not self.fullscreen:
            cv2.resizeWindow(WINDOW_NAME, self.width, self.height)

    def select_at(self, x, y):
        for name, action in (
            ('window_minimize', self.minimize_window),
            ('window_fullscreen', self.toggle_window_fullscreen),
            ('window_vision', self.show_vision_window),
            ('window_exit', lambda: setattr(self, 'window_exit_requested', True)),
        ):
            rect = self.hitboxes.get(name)
            if rect and rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]:
                action()
                return
        rect = self.hitboxes.get('lidar_toggle')
        if self.dynamic is not None and rect and rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]:
            self.dynamic.set_lidar_enabled(not self.dynamic.lidar_enabled)
            return
        super().select_at(x, y)

    def handle_key(self, key):
        if self.window_exit_requested:
            return False  # The existing run-loop finally owns STOP and cleanup.
        if key & 0xFF in (ord('f'), ord('F')):
            self.toggle_window_fullscreen()
            return True
        if key & 0xFF in (ord('l'), ord('L')) and self.dynamic is not None:
            self.dynamic.set_lidar_enabled(not self.dynamic.lidar_enabled)
            return True
        return super().handle_key(key)

    def record_reset_event(self, event: str, source: str) -> None:
        try:
            RUNTIME.mkdir(parents=True, exist_ok=True)
            with (RUNTIME / "startup_events.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({
                    "event": event, "source": source,
                    "timestamp_monotonic_ns": time.monotonic_ns(),
                    "side": self.side, "start_zone": self.zone,
                    "opening_strategy": self.opening_strategy,
                }, ensure_ascii=False) + "\n")
        except OSError as error:
            print(f"复位日志写入失败：{error}", file=sys.stderr)

    def reset_session(self, source: str = "map_window") -> None:
        self.warming = self.start_requested = self.start_released = False
        if self.operator is not None:
            self.operator.transitioning = True
        self.record_reset_event("session_reset_requested", source)
        super().reset_session()
        self.reset_session_token = None
        self.reset_request_path.unlink(missing_ok=True)
        self.record_reset_event("session_reset_completed", source)
        if self.operator is not None:
            self.operator.after_reset()
            if self.prewarm_enabled:
                self.prepare_warm_session()

    def consume_reset_request(self) -> bool:
        try:
            request = json.loads(self.reset_request_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return False
        except (OSError, ValueError):
            return False
        self.reset_request_path.unlink(missing_ok=True)
        # A leftover request must never reset a later session or another runner.
        if (not isinstance(request, dict) or not (self.session_started or self.warming)
                or self.reset_session_token is None
                or request.get("session_token") != self.reset_session_token):
            return False
        self.reset_session(source="vision_window")
        return True

    def archive_previous_runtime(self) -> None:
        super().archive_previous_runtime()
        candidates = (
            RUNTIME / "competition_diagnostics.json",
            RUNTIME / "competition_detections.jsonl",
            RUNTIME / "competition_events.jsonl",
            RUNTIME / "competition_logging_status.json",
        )
        existing = [path for path in candidates if path.is_file()]
        if not existing:
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        archive_dir = RUNTIME / "history" / f"{stamp}_competition"
        suffix = 1
        while archive_dir.exists():
            suffix += 1
            archive_dir = RUNTIME / "history" / f"{stamp}_competition_{suffix:02d}"
        archive_dir.mkdir(parents=True, exist_ok=False)
        for path in existing:
            try:
                shutil.copy2(path, archive_dir / path.name)
            except OSError:
                continue

    def vision_command(self) -> list[str]:
        self.reset_request_path.unlink(missing_ok=True)
        if self.reset_session_token is None:
            self.reset_session_token = secrets.token_hex(16)
        command = [
            str(PROJECT_ROOT / "competition_rescue/run_competition_rescue.sh"),
            "--session", str(RUNTIME / "session.json"),
            "--pose", str(self.localization_json),
            "--stm-status", str(RUNTIME / "stm32_status.json"),
            "--command-file", str(RUNTIME / "uart_command.bin"),
            "--config", str(PROJECT_ROOT / "vision/config/rescue_vision.json"),
            "--homography", str(PROJECT_ROOT / "vision/config/homography.txt"),
            "--diagnostics", str(RUNTIME / "competition_diagnostics.json"),
            "--detections-log", str(RUNTIME / "competition_detections.jsonl"),
            "--events-log", str(RUNTIME / "competition_events.jsonl"),
            "--reset-request", str(self.reset_request_path),
            "--reset-session-token", self.reset_session_token,
            "--window-mode", "normal",
            "--enable-bulldozer",
            "--startup-timeout", f"{max(60.0, self.relocalization_timeout_s + 20.0):.3f}",
        ]
        if self.operator is not None:
            command += ['--operator-control', str(self.operator.control_path)]
        if self.dynamic is not None:
            address = self.dynamic.wheel_policy_address
            command += ['--localization-wheel-policy-socket',
                        '@' + address[1:] if address.startswith('\0') else address,
                        '--localization-wheel-policy-session', self.dynamic.session_id]
        if self.prewarm_enabled:
            command += ['--wait-for-operator', '--warm-status', str(self.warm_status_path)]
        if getattr(self.options, 'headless', False):
            command += ['--no-display']
        return command

    def update_pose(self) -> None:
        if self.operator is not None:
            self.operator.tick()
        if self.consume_reset_request():
            return
        if self.warming:
            selected = self.selecting
            self.selecting = False
            super().update_pose()
            self.selecting = selected
            self._update_warm_start()
            return
        super().update_pose()
        if (
            not self.selecting and
            self.vision_process is not None and
            self.vision_process.poll() is not None
        ):
            self.message = (
                f"完整识别进程已退出：{self.vision_process.returncode}；"
                "请查看competition_events.jsonl"
            )


def main() -> int:
    try:
        return CompetitionMapApp(parse_args()).run()
    except (ValueError, RuntimeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
