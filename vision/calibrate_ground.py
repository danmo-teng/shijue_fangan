#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from rescue_vision.camera import LatestFrameCamera
from rescue_vision.config import load_config, require_native_resolution
from rescue_vision.localizer import GroundLocalizer, ApproachRangeCalibration


def main() -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="点击地面标定点，计算图像到车体地面坐标的单应矩阵")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--camera-fps", type=int)
    parser.add_argument("--config", default=str(root / "config" / "rescue_vision.json"))
    parser.add_argument("--output", default=str(root / "config" / "homography.txt"))
    parser.add_argument("--range-pitch", type=int, choices=(105, 120))
    parser.add_argument("--range-band", action="append", default=[], metavar="Y_MIN:Y_MAX:ADVANCE_MM",
                        help="实测目标中心Y分段→车体应走距离，已扣除夹爪入口及保留余量")
    parser.add_argument("--range-camera-mount", help="本次测量的相机安装位置/零点描述")
    parser.add_argument("--range-output", default=str(root / "config" / "approach_range.json"))
    args = parser.parse_args()
    config = load_config(args.config)
    camera_config = config.get("camera", {})
    width = int(args.width or camera_config.get("width", 1280))
    height = int(args.height or camera_config.get("height", 720))
    require_native_resolution(width, height)
    camera_fps = int(args.camera_fps or camera_config.get("fps", 180))
    if args.range_pitch is not None:
        if not args.range_band or not args.range_camera_mount:
            parser.error("分段标定必须提供实测--range-band及--range-camera-mount，不生成猜测距离")
        try:
            if any(len(value.split(":")) != 3 for value in args.range_band):
                raise ValueError("每个分段必须恰好为Y_MIN:Y_MAX:ADVANCE_MM")
            bands = [dict(zip(("y_min", "y_max", "advance_mm"), map(int, v.split(":")))) for v in args.range_band]
            data = ApproachRangeCalibration.load(args.range_output).data
            data.setdefault("angles", {})[str(args.range_pitch * 100)] = {
                "measured": True, "width": width, "height": height,
                "pixel_reference": "bbox_center", "camera_mount": args.range_camera_mount,
                "distance_reference": "measured_chassis_travel_to_claw_entry", "bands": bands,
            }
            calibration = ApproachRangeCalibration(data)
            for band in bands:
                distance, reason = calibration.distance(args.range_pitch * 100,
                    (width / 2, (band["y_min"] + band["y_max"]) / 2), (width, height))
                if distance is None:
                    raise ValueError(reason)
            target = Path(args.range_output)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"{args.range_pitch}°实测分段已保存：{target}；另一角度单独标定，重启比赛加载")
            return 0
        except (ValueError, KeyError, TypeError) as error:
            parser.error(str(error))
    ground_points = [tuple(point) for point in config["calibration_ground_points_mm"]]
    clicked: list[tuple[float, float]] = []
    frozen = None
    window = "Ground Homography Calibration"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    def mouse(event, x, y, _flags, _data):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < len(ground_points):
            clicked.append((float(x), float(y)))
            print(f"点{len(clicked)} image=({x},{y}) ground={ground_points[len(clicked)-1]}mm")

    cv2.setMouseCallback(window, mouse)
    camera = LatestFrameCamera(args.device, width, height, camera_fps)
    camera.start()
    print("按空格冻结画面；按给定地面坐标顺序点击；R重置；S计算并保存；Q退出")
    print("地面点顺序：", ground_points)
    try:
        while True:
            packet = camera.latest()
            if packet is None:
                time.sleep(0.01)
                continue
            if frozen is None:
                frame = packet.image.copy()
            else:
                frame = frozen.copy()
            for index, point in enumerate(clicked):
                cv2.circle(frame, (int(point[0]), int(point[1])), 5, (0, 255, 255), -1)
                cv2.putText(frame, str(index + 1), (int(point[0]) + 7, int(point[1]) - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            next_index = min(len(clicked), len(ground_points) - 1)
            text = f"Click {len(clicked)+1}/{len(ground_points)}: ground {ground_points[next_index]} mm" if len(clicked) < len(ground_points) else "All points collected - press S"
            cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                frozen = None if frozen is not None else packet.image.copy()
            elif key == ord("r"):
                clicked.clear()
            elif key == ord("s"):
                if len(clicked) != len(ground_points):
                    print("点数不足，不能保存")
                    continue
                localizer, rmse = GroundLocalizer.calibrate(clicked, ground_points)
                localizer.save(args.output, (width, height))
                print(f"标定已保存：{args.output}，重投影RMSE={rmse:.2f}mm")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
