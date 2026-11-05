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
from motion_log_storage import list_sessions, delete_sessions, read_json

WHEEL_OWNER = "competition_wheel_input"
WHEEL_FILES = frozenset({"metadata.json", "localization.conf", "session_complete.json",
    "wheel_feedback_requests.bin", "t265_raw_pose.csv", "localization_debug.csv",
    "localization_events.jsonl", "wheel_sdk_config.json", "wheel_sdk.jsonl", "wheel_sdk_status.json",
    "wheel_input.jsonl", "wheel_measurements.jsonl", "chassis_identity.json",
    "chassis_extended.jsonl", "logging_events.jsonl", "final_status.json"})


class CompetitionMapApp(RescueMapApp):
    """Reuse the existing selector and map, but launch the new task runner."""

    def __init__(self, options) -> None:
        super().__init__(options)
        self.reset_request_path = RUNTIME / "startup_reset_request.json"
        self.reset_session_token = None
        self.official_wheel_input = getattr(options, "official_wheel_input", False)
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
        metadata.update(owner=WHEEL_OWNER, mode="competition_t265_wheel_input", sdk_session_id=session_id,
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
        status = adapter.snapshot()
        adapter.bridge.log.submit("final_status.json", status, atomic=True)
        adapter.close()
        # Log retention is module-local and runs only after a complete saved
        # session. It cannot block active control or delete test/user data.
        try:
            run_dir = adapter.root
            required = ("metadata.json", "localization_debug.csv", "t265_raw_pose.csv", "final_status.json")
            if (process is not None and process.returncode == 0 and adapter.bridge.log.healthy()
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
