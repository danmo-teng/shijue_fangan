#!/usr/bin/env python3
"""Capture and analyze T265 versus the F407 three-wheel odometry interface.

The live mode deliberately does not send TYPE=0x11/0x12/0x18 frames.  It
starts the already-tested upper-computer localizer in T265-only fusion mode,
while still listening to F407 TYPE=0x15 odometry and TYPE=0x17 status frames.
The localizer's full-rate CSV is retained for every run and can be analyzed
again without hardware.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
LOCALIZATION_ROOT = PROJECT_ROOT / "localization"
DEFAULT_LOCALIZER = LOCALIZATION_ROOT / "build" / "t265_omni_localizer"
DEFAULT_CONFIG = LOCALIZATION_ROOT / "config" / "localization.example.conf"
DEFAULT_RUN_ROOT = PROJECT_ROOT / "rescue_map" / "runtime" / "history" / "t265_f407_debug"

PROTOCOL = {
    "uart": "USART3 via RDK /dev/ttyS1",
    "baud": 115200,
    "frame_bytes": 15,
    "frame_head": "A3 B3",
    "frame_tail": "C3",
    "crc": "CRC-16/Modbus over TYPE..P7, little-endian CRC",
    "odom_type": "0x15",
    "status_type": "0x17",
    "m1": "right wheel",
    "m2": "left wheel",
    "m3": "rear wheel",
    "encoder_sign": [-1, -1, -1],
    "sample_period": "F407 encoder sample/control period, normally 10 ms",
}


def finite(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return float(value)


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def integer(row: dict[str, str], key: str, default: int = 0) -> int:
    value = number(row, key, math.nan)
    return default if not math.isfinite(value) else int(round(value))


def wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def read_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def find_csv(path: Path) -> tuple[Path, Path | None]:
    if path.is_file():
        return path, path.parent if path.name.endswith(".csv") else None
    csv_path = path / "localization_debug.csv"
    if not csv_path.exists():
        matches = sorted(path.glob("*.csv"))
        if matches:
            csv_path = matches[0]
    if not csv_path.exists():
        raise FileNotFoundError(f"未找到定位CSV：{path}")
    return csv_path, path


def initial_reference(
    first: dict[str, str], offset_forward: float, offset_left: float
) -> dict[str, Any]:
    yaw_deg = number(first, "t265_yaw_deg")
    raw_x = number(first, "tracking_origin_x_m")
    raw_y = number(first, "tracking_origin_y_m")
    center_x = number(first, "t265_x_m")
    center_y = number(first, "t265_y_m")
    if not all(math.isfinite(value) for value in (yaw_deg, raw_x, raw_y, center_x, center_y)):
        return {"available": False}
    yaw = math.radians(yaw_deg)
    expected_x = math.cos(yaw) * offset_forward - math.sin(yaw) * offset_left
    expected_y = math.sin(yaw) * offset_forward + math.cos(yaw) * offset_left
    observed_x = raw_x - center_x
    observed_y = raw_y - center_y
    return {
        "available": True,
        "start_yaw_deg": yaw_deg,
        "observed_tracking_origin_minus_robot_center_m": {
            "x": observed_x,
            "y": observed_y,
            "norm": math.hypot(observed_x, observed_y),
        },
        "expected_rotated_offset_m": {
            "x": expected_x,
            "y": expected_y,
            "norm": math.hypot(expected_x, expected_y),
        },
        "error_m": math.hypot(observed_x - expected_x, observed_y - expected_y),
    }


def rotation_check(
    rows: list[dict[str, str]], offset_forward: float, offset_left: float
) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    usable = [
        row for row in rows
        if math.isfinite(number(row, "relative_yaw_deg"))
        and math.isfinite(number(row, "tracking_origin_delta_forward_m"))
        and math.isfinite(number(row, "robot_center_delta_forward_m"))
    ]
    for target in (90.0, 180.0, 270.0, 360.0):
        if not usable:
            result.append({"target_deg": target, "available": False})
            continue
        row = min(
            usable,
            key=lambda item: abs(abs(number(item, "relative_yaw_deg")) - target),
        )
        yaw_deg = number(row, "relative_yaw_deg")
        yaw_error = abs(abs(yaw_deg) - target)
        if yaw_error > max(6.0, target * 0.05):
            result.append({
                "target_deg": target,
                "available": False,
                "reason": "capture_did_not_reach_target_yaw",
                "nearest_relative_yaw_deg": yaw_deg,
            })
            continue
        yaw = math.radians(yaw_deg)
        expected_forward = (
            math.cos(yaw) * offset_forward
            - math.sin(yaw) * offset_left
            - offset_forward
        )
        expected_left = (
            math.sin(yaw) * offset_forward
            + math.cos(yaw) * offset_left
            - offset_left
        )
        raw_forward = number(row, "tracking_origin_delta_forward_m")
        raw_left = number(row, "tracking_origin_delta_left_m")
        corrected_forward = number(row, "robot_center_delta_forward_m")
        corrected_left = number(row, "robot_center_delta_left_m")
        result.append({
            "target_deg": target,
            "available": True,
            "actual_relative_yaw_deg": yaw_deg,
            "target_yaw_error_deg": yaw_error,
            "raw_tracking_origin_delta_m": {
                "forward": raw_forward,
                "left": raw_left,
            },
            "expected_pure_rotation_delta_m": {
                "forward": expected_forward,
                "left": expected_left,
            },
            "raw_arc_error_m": math.hypot(
                raw_forward - expected_forward, raw_left - expected_left
            ),
            "corrected_robot_center_norm_m": math.hypot(
                corrected_forward, corrected_left
            ),
        })
    return result


def fit_wheel_to_t265(rows: list[dict[str, str]], max_yaw_deg: float | None) -> dict[str, Any]:
    """Fit T265 body increments = A * raw F407 wheel increments.

    A diagonal value near 1 means scale agreement.  A large off-diagonal
    value means cross-axis rotation or wheel-order/sign error.  This uses only
    frames where the F407 increment passed the upper-computer gate and avoids
    changing any live calibration.
    """
    samples: list[tuple[float, float, float, float]] = []
    for index in range(1, len(rows)):
        row = rows[index]
        previous = rows[index - 1]
        if integer(row, "wheel_update_this_pose") != 1:
            continue
        if integer(row, "wheel_update_accepted") != 1:
            continue
        wheel_forward = number(row, "odom_increment_forward_m")
        wheel_left = number(row, "odom_increment_left_m")
        yaw_step = number(row, "odom_increment_yaw_deg")
        if not all(math.isfinite(value) for value in (wheel_forward, wheel_left, yaw_step)):
            continue
        if max_yaw_deg is not None and abs(yaw_step) > max_yaw_deg:
            continue
        if math.hypot(wheel_forward, wheel_left) < 1e-5:
            continue
        current_x = number(row, "t265_x_m")
        current_y = number(row, "t265_y_m")
        previous_x = number(previous, "t265_x_m")
        previous_y = number(previous, "t265_y_m")
        current_yaw = number(row, "t265_yaw_deg")
        previous_yaw = number(previous, "t265_yaw_deg")
        if not all(math.isfinite(value) for value in (
            current_x, current_y, previous_x, previous_y, current_yaw, previous_yaw
        )):
            continue
        yaw_mid = math.radians(previous_yaw + 0.5 * wrap_degrees(current_yaw - previous_yaw))
        field_x = current_x - previous_x
        field_y = current_y - previous_y
        t265_forward = math.cos(yaw_mid) * field_x + math.sin(yaw_mid) * field_y
        t265_left = -math.sin(yaw_mid) * field_x + math.cos(yaw_mid) * field_y
        samples.append((wheel_forward, wheel_left, t265_forward, t265_left))

    if len(samples) < 3:
        return {"available": False, "samples": len(samples)}

    xx00 = sum(sample[0] * sample[0] for sample in samples)
    xx01 = sum(sample[0] * sample[1] for sample in samples)
    xx11 = sum(sample[1] * sample[1] for sample in samples)
    determinant = xx00 * xx11 - xx01 * xx01
    if abs(determinant) < 1e-12:
        return {"available": False, "samples": len(samples), "reason": "insufficient_direction_excitation"}

    inv00 = xx11 / determinant
    inv01 = -xx01 / determinant
    inv11 = xx00 / determinant
    b_f0 = sum(sample[0] * sample[2] for sample in samples)
    b_f1 = sum(sample[1] * sample[2] for sample in samples)
    b_l0 = sum(sample[0] * sample[3] for sample in samples)
    b_l1 = sum(sample[1] * sample[3] for sample in samples)
    matrix = [
        [inv00 * b_f0 + inv01 * b_f1, inv01 * b_f0 + inv11 * b_f1],
        [inv00 * b_l0 + inv01 * b_l1, inv01 * b_l0 + inv11 * b_l1],
    ]
    residuals = []
    for wheel_forward, wheel_left, t265_forward, t265_left in samples:
        predicted_forward = matrix[0][0] * wheel_forward + matrix[0][1] * wheel_left
        predicted_left = matrix[1][0] * wheel_forward + matrix[1][1] * wheel_left
        residuals.append(math.hypot(predicted_forward - t265_forward,
                                    predicted_left - t265_left))
    return {
        "available": True,
        "samples": len(samples),
        "t265_body_from_wheel_body_matrix": matrix,
        "rmse_m": math.sqrt(sum(value * value for value in residuals) / len(residuals)),
        "max_residual_m": max(residuals),
        "interpretation": {
            "forward_scale": matrix[0][0],
            "left_scale": matrix[1][1],
            "forward_from_left_cross_axis": matrix[0][1],
            "left_from_forward_cross_axis": matrix[1][0],
        },
    }


def analyze(csv_path: Path, output_path: Path | None, trial: str,
            offset_forward: float, offset_left: float) -> dict[str, Any]:
    rows = read_rows(csv_path)
    if not rows:
        raise RuntimeError(f"CSV没有数据：{csv_path}")
    first = rows[0]
    last = rows[-1]
    first_pose = (number(first, "t265_x_m"), number(first, "t265_y_m"))
    last_pose = (number(last, "t265_x_m"), number(last, "t265_y_m"))
    first_odom = (number(first, "odom_x_m"), number(first, "odom_y_m"))
    last_odom = (number(last, "odom_x_m"), number(last, "odom_y_m"))
    t265_delta = (last_pose[0] - first_pose[0], last_pose[1] - first_pose[1])
    odom_delta = (last_odom[0] - first_odom[0], last_odom[1] - first_odom[1])
    start_yaw = math.radians(number(first, "t265_yaw_deg", 0.0))
    t265_body_delta = (
        math.cos(start_yaw) * t265_delta[0] + math.sin(start_yaw) * t265_delta[1],
        -math.sin(start_yaw) * t265_delta[0] + math.cos(start_yaw) * t265_delta[1],
    )
    selected_yaw_limit = 8.0 if trial == "linear" else None
    result: dict[str, Any] = {
        "schema_version": 1,
        "source_csv": str(csv_path),
        "trial": trial,
        "protocol": PROTOCOL,
        "rows": len(rows),
        "elapsed_s": finite(number(last, "elapsed_s") - number(first, "elapsed_s")),
        "camera_offset_m": {
            "forward": offset_forward,
            "left": offset_left,
        },
        "initial_reference_check": initial_reference(first, offset_forward, offset_left),
        "rotation_check": rotation_check(rows, offset_forward, offset_left),
        "trajectory": {
            "t265_field_delta_m": {"x": t265_delta[0], "y": t265_delta[1]},
            "t265_start_body_delta_m": {
                "forward": t265_body_delta[0], "left": t265_body_delta[1]
            },
            "wheel_odom_field_delta_m": {"x": odom_delta[0], "y": odom_delta[1]},
            "t265_vs_wheel_end_error_m": math.hypot(
                t265_delta[0] - odom_delta[0], t265_delta[1] - odom_delta[1]
            ),
            "t265_travel_m": number(last, "t265_travel_m"),
            "wheel_odom_travel_m": number(last, "odom_travel_m"),
        },
        "wheel_to_t265_fit": fit_wheel_to_t265(rows, selected_yaw_limit),
        "interface_observation": {
            "valid_encoder_updates": sum(
                1 for row in rows if integer(row, "wheel_update_this_pose") == 1
            ),
            "accepted_encoder_updates": sum(
                1 for row in rows if integer(row, "wheel_update_accepted") == 1
            ),
            "wheel_odom_updates_last": integer(last, "wheel_odom_updates"),
            "last_wheel_status": integer(last, "wheel_status"),
            "last_wheel_sequence": integer(last, "wheel_frame_sequence"),
        },
    }
    if output_path is not None:
        output_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    return result


def print_report(result: dict[str, Any]) -> None:
    print(json.dumps(result, ensure_ascii=False, indent=2))


def safe_label(label: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", label.strip())
    return cleaned or "manual"


def run_capture(args: argparse.Namespace) -> int:
    if not args.localizer.exists():
        raise FileNotFoundError(f"定位可执行文件不存在：{args.localizer}")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_name = f"{timestamp}_{safe_label(args.label)}"
    run_dir = args.output_root / base_name
    suffix = 1
    while run_dir.exists():
        run_dir = args.output_root / f"{base_name}_{suffix:02d}"
        suffix += 1
    run_dir.mkdir(parents=True, exist_ok=False)
    csv_path = run_dir / "localization_debug.csv"
    json_path = run_dir / "localization_result.json"
    status_path = run_dir / "stm32_status.json"
    log_path = run_dir / "localizer.log"
    analysis_path = run_dir / "analysis.json"
    command = [
        str(args.localizer),
        "--config", str(args.config),
        "--output", str(json_path),
        "--csv", str(csv_path),
        "--rate", str(args.rate),
        "--tx-rate", "0",
        "--ignore-encoders",
        "--stm-status", str(status_path),
    ]
    if not args.no_uart:
        command += ["--uart", args.uart, "--baud", str(args.baud)]
    if args.serial:
        command += ["--serial", args.serial]
    if args.duration > 0.0:
        command += ["--duration", str(args.duration)]
    metadata = {
        "schema_version": 1,
        "created_local": datetime.now().isoformat(timespec="seconds"),
        "label": args.label,
        "trial": args.trial,
        "command": command,
        "listen_only": True,
        "f407_interface": PROTOCOL,
        "upper_repository": "danmo-teng/shijue_fangan",
        "lower_repository": "gandizm/F407-Rescue-Robot@68a0802",
        "files": {
            "csv": csv_path.name,
            "localization_json": json_path.name,
            "stm_status_json": status_path.name,
            "localizer_log": log_path.name,
            "analysis_json": analysis_path.name,
        },
    }
    (run_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"日志目录：{run_dir}")
    print("监听模式：不发送 TYPE=0x11/0x12/0x18，不启动 F407 任务")
    print("执行：" + " ".join(command))
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            command,
            cwd=str(LOCALIZATION_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        try:
            assert process.stdout is not None
            for line in process.stdout:
                print(line, end="")
                log.write(line)
                log.flush()
        except KeyboardInterrupt:
            print("\n收到停止信号，正在结束定位采集…")
            process.send_signal(signal.SIGINT)
        finally:
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3.0)
    if csv_path.exists():
        result = analyze(csv_path, analysis_path, args.trial,
                         args.offset_forward, args.offset_left)
        print_report(result)
    return int(process.returncode or 0)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="T265/F407三轮里程计只监听调试程序")
    parser.add_argument("--analyze", type=Path, help="分析已有CSV或一次运行日志目录")
    parser.add_argument("--localizer", type=Path, default=DEFAULT_LOCALIZER)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--uart", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--no-uart", action="store_true", help="只采集T265，不打开F407串口")
    parser.add_argument("--serial", help="T265 serial number")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=10.0,
                        help="JSON/stdout rate; CSV仍按T265全速率保存")
    parser.add_argument("--label", default="manual")
    parser.add_argument("--trial", choices=("unknown", "still", "rotate", "linear", "combined"),
                        default="unknown")
    parser.add_argument("--offset-forward", type=float, default=-0.0296)
    parser.add_argument("--offset-left", type=float, default=-0.0301)
    return parser.parse_args()


def main() -> int:
    args = arguments()
    if args.analyze:
        csv_path, run_dir = find_csv(args.analyze)
        output = None if run_dir is None else run_dir / "analysis.json"
        print_report(analyze(csv_path, output, args.trial,
                             args.offset_forward, args.offset_left))
        return 0
    if args.duration < 0.0 or args.rate < 0.0:
        raise ValueError("duration和rate不能为负数")
    args.output_root.mkdir(parents=True, exist_ok=True)
    return run_capture(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
