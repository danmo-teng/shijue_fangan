#!/usr/bin/env python3
"""Map/start selector for the independent complete competition flow.

The formal C++ process owns UART and SDK. Official wheel input reuses the
existing test bridge; the field start and mission/task pairing stay formal.
"""
from __future__ import annotations

import json
import secrets
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT / "rescue_map"))
sys.path.insert(0, str(PROJECT_ROOT / "localization/tools"))

from map_app import RUNTIME, RescueMapApp, parse_args  # noqa: E402
from competition_wheel_input import CompetitionWheelInput
from competition_dynamic_localization import CompetitionDynamicLocalization
from lidar_yaw_debug.diagnostic import load_config
from lidar_yaw_debug.motion_source import POLICY as MOTION_SOURCE_POLICY
from motion_log_storage import list_sessions, delete_sessions, read_json, DYNAMIC_SESSION_FILES

WHEEL_OWNER = "competition_wheel_input"
WHEEL_FILES = frozenset({"metadata.json", "localization.conf", "session_complete.json",
    "wheel_feedback_requests.bin", "t265_raw_pose.csv", "localization_debug.csv",
    "localization_events.jsonl", "wheel_sdk_config.json", "wheel_sdk.jsonl", "wheel_sdk_status.json",
    "wheel_input.jsonl", "wheel_measurements.jsonl", "chassis_identity.json",
    "chassis_extended.jsonl", "logging_events.jsonl", "final_status.json", "raw_t265_result.json", "control_pose.jsonl", "vision_replay_manifest.json"}) | DYNAMIC_SESSION_FILES


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
        if self.official_wheel_input:
            # Use the same file as the test, but the normal writer sets the
            # selected competition corner/heading (never the test centre).
            self.localization_config_template = PROJECT_ROOT / "localization/config/t265_debug_new_mount_candidate.conf"
            self.localization_mode = "t265"
            self.encoder_weight = 0.0
            self.t265_translation_scale_enabled = False
            self.t265_translation_scale = 1.0
            self.t265_map_enabled = False

    def localization_command(self) -> list[str]:
        if self.wheel_input is not None:
            return list(self.wheel_command)
        command = super().localization_command()
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
                metadata['initial_pose'], self.wheel_input.bridge.log)
            self.wheel_input.telemetry_sink = self.dynamic.telemetry
            metadata['heading_conventions']['pose'] = 'primary gyro, fixed selected field start reference, CCW positive'
            metadata['pose_dependencies'].append('STL50B2 observable fence constraints')
            metadata.update(localization_chain='desktop_DynamicFence_RadarWorker_MotionSource',
                pose_source='lidar_t265_measured_wheel_fallback',
                initial_reference_policy='selected_formal_start_no_wall_wait',
                lidar_installation=dynamic_config['installation'],
                qualification=dynamic_config['qualification'],
                control_pose_publish_enabled=True, accuracy_verified=False,
                motion_source_policy=dict(MOTION_SOURCE_POLICY),
                obstacle_policy='operator_lidar_pause_resume_same_as_desktop_test')
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
            command += ['--debug-pose-socket', '@' + self.dynamic.pose_address[1:],
                        '--dynamic-pose-socket', '@' + self.dynamic.control_address[1:],
                        '--dynamic-pose-output', str(self.localization_json)]
        self.wheel_command = list(command)
        return command

    def wheel_input_status(self):
        return self.wheel_input.snapshot() if self.wheel_input else {}

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
        # Log retention is module-local and runs only after a complete saved
        # session. It cannot block active control or delete test/user data.
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
                rows = [r for r in list_sessions(run_dir.parent, run_dir, WHEEL_FILES, owner=WHEEL_OWNER) if not r["reason"]]
                rows.sort(key=lambda r: (r["order"], r["name"]), reverse=True)
                delete_sessions(run_dir.parent, rows[4:], run_dir, WHEEL_FILES, owner=WHEEL_OWNER)
        except (OSError, ValueError, TypeError) as error:
            print(f"轮速会话收尾失败（原始文件保留）：{error}", file=sys.stderr)

    def draw_panel(self, canvas, text):
        super().draw_panel(canvas, text)
        if self.dynamic is not None and not self.selecting:
            state = self.dynamic.snapshot()
            mode = state.get('motion_selection', {}).get('mode', 'WAIT')
            label = {'WHEEL_FALLBACK': '轮速＋主yaw降级',
                     'T265_WITH_WHEEL_TAIL': 'T265＋轮反馈续推'}.get(mode, mode)
            axes = '/'.join(state.get('current_wall_axes', [])) or '暂无约束'
            text.add('当前来源：' + label, (920, 455), 17, (80, 205, 245))
            text.add('雷达修正：' + axes, (920, 490), 17, (80, 205, 245))
            self.button(canvas, text, 'lidar_toggle',
                '越障：暂停雷达 [L]' if self.dynamic.lidar_enabled else '恢复雷达修正 [L]',
                (920, 515, 1245, 555), self.dynamic.lidar_enabled)
            if state.get('transport_error'):
                self.message = '定位链错误：' + state['transport_error']

    def select_at(self, x, y):
        rect = self.hitboxes.get('lidar_toggle')
        if self.dynamic is not None and rect and rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]:
            self.dynamic.set_lidar_enabled(not self.dynamic.lidar_enabled)
            return
        super().select_at(x, y)

    def handle_key(self, key):
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
        self.record_reset_event("session_reset_requested", source)
        super().reset_session()
        self.reset_session_token = None
        self.reset_request_path.unlink(missing_ok=True)
        self.record_reset_event("session_reset_completed", source)

    def consume_reset_request(self) -> bool:
        try:
            request = json.loads(self.reset_request_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return False
        except (OSError, ValueError):
            return False
        self.reset_request_path.unlink(missing_ok=True)
        # A leftover request must never reset a later session or another runner.
        if (not isinstance(request, dict) or not self.session_started
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
        self.reset_session_token = secrets.token_hex(16)
        return [
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
            "--startup-timeout", f"{max(60.0, self.relocalization_timeout_s + 20.0):.3f}",
        ]

    def update_pose(self) -> None:
        if self.consume_reset_request():
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
