#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from rescue_vision.camera import LatestFrameCamera
from rescue_vision.config import load_config
from rescue_vision.localizer import GroundLocalizer


def main() -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="点击地面标定点，计算图像到车体地面坐标的单应矩阵")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--camera-fps", type=int)
    parser.add_argument("--config", default=str(root / "config" / "rescue_vision.json"))
    parser.add_argument("--output", default=str(root / "config" / "homography.txt"))
    args = parser.parse_args()
    config = load_config(args.config)
    camera_config = config.get("camera", {})
    width = int(args.width or camera_config.get("width", 1280))
    height = int(args.height or camera_config.get("height", 720))
    camera_fps = int(args.camera_fps or camera_config.get("fps", 180))
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
