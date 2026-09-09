#!/usr/bin/env python3
"""Map/start selector for the independent complete competition flow.

The mature ``rescue_map.map_app`` UI owns the T265/map/fusion controls.  This
thin subclass changes only the child mission runner, so the old debug program
remains intact and both programs keep the same localization configuration.
"""
from __future__ import annotations

import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT / "rescue_map"))

from map_app import RUNTIME, RescueMapApp, parse_args  # noqa: E402


class CompetitionMapApp(RescueMapApp):
    """Reuse the existing selector and map, but launch the new task runner."""

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
            "--window-mode", "normal",
            "--startup-timeout", f"{max(60.0, self.relocalization_timeout_s + 20.0):.3f}",
        ]

    def update_pose(self) -> None:
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
