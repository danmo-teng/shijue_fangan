#!/usr/bin/env python3
"""Read-only camera/BPU view for manually measuring fixed-pitch range bands."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="测距标定画面：物资中心坐标、横向分区；不发送串口或运动命令")
    parser.add_argument("--pitch", type=int, choices=(105, 120), default=105,
                        help="当前手动设置的相机角度标签；本程序不控制舵机")
    parser.add_argument("--bands", type=int, default=6)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--model", type=Path, default=root / "models/best_bayese_320x320_nv12.bin")
    parser.add_argument("--output", type=Path, default=root / "captures/range_calibration")
    args = parser.parse_args()
    if not 2 <= args.bands <= 12:
        parser.error("--bands须为2..12")
    import cv2
    from run_yolo_x5 import X5YoloV8, load_labels, DEFAULT_LABELS
    from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
    from rescue_vision.vision_protocol import IMAGE_WIDTH, IMAGE_HEIGHT

    detector = X5YoloV8(args.model, load_labels(DEFAULT_LABELS), .30, .45, 0, [0])
    camera = LatestFrameCamera(resolve_camera_device(args.device), IMAGE_WIDTH, IMAGE_HEIGHT, 180,
                               decoder="jpu", decode_fps=15, output_format="bgr")
    boundaries = [round(i * IMAGE_HEIGHT / args.bands) for i in range(args.bands + 1)]
    bands = [{"band": i + 1, "y_min": boundaries[i], "y_max": boundaries[i + 1]}
             for i in range(args.bands)]
    window = "Range calibration: 1=105 2=120 SPACE=freeze S=save Q=quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1000, 800)
    frame = None
    detections = []
    frame_id = -1
    capture_ns = None
    frozen = False
    print("请先停止比赛进程，固定相机为105°或120°。1/2仅切换记录标签，不控制舵机。")
    print("默认6个横向区域，坐标按原始1280×1024记录；空格冻结，S保存截图/中心点，Q退出。")
    print("请记录每个区域车体到夹爪入口的实际应走距离及预留补进余量，不以相机斜距代替。")
    try:
        camera.start()
        while True:
            error = camera.check_error()
            if error is not None:
                raise RuntimeError(error)
            packet = camera.latest()
            if not frozen and packet is not None and packet.frame_id != frame_id:
                frame_id, capture_ns = packet.frame_id, packet.published_ns
                frame = packet.image.copy()
                found, _ = detector.infer(frame)
                detections = [d for d in found if d.class_name in {
                    "green_supply", "core_black", "injured_orange", "danger_cyan"}
                    and (d.class_name != "green_supply" or d.confidence >= .50)]
            if frame is None:
                if cv2.waitKey(20) & 0xFF in (ord("q"), 27):
                    break
                continue
            shown = frame.copy()
            for band in bands:
                y = band["y_min"]
                cv2.line(shown, (0, y), (IMAGE_WIDTH - 1, y), (255, 200, 0), 2)
                cv2.putText(shown, f'B{band["band"]}: Y[{y},{band["y_max"]})',
                            (8, min(IMAGE_HEIGHT - 10, y + 65)), cv2.FONT_HERSHEY_SIMPLEX,
                            .65, (255, 220, 0), 2)
            records = []
            for detection in detections:
                x, y, w, h = detection.bbox
                cx, cy = detection.center
                band = next((b["band"] for b in bands if b["y_min"] <= cy < b["y_max"]), None)
                cv2.rectangle(shown, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.drawMarker(shown, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
                label = f'{detection.class_name} ({cx},{cy}) B{band} {detection.confidence:.2f}'
                cv2.putText(shown, label, (max(0, x), max(90, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, .58, (0, 255, 255), 2)
                records.append({"class": detection.class_name, "bbox": detection.bbox,
                                "center_px": [cx, cy], "band": band, "confidence": detection.confidence})
            cv2.putText(shown, f'{args.pitch} deg LABEL ONLY - set servo manually | {"FROZEN" if frozen else "LIVE"}',
                        (8, 27), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
            cv2.imshow(window, shown)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key == ord(" "):
                frozen = not frozen
            elif key in (ord("1"), ord("2")):
                args.pitch = 105 if key == ord("1") else 120
            elif key in (ord("s"), ord("S")):
                args.output.mkdir(parents=True, exist_ok=True)
                stem = args.output / f'{args.pitch}deg_{time.time_ns()}'
                if not cv2.imwrite(str(stem) + ".jpg", shown):
                    raise OSError("标定截图保存失败")
                Path(str(stem) + ".json").write_text(json.dumps({
                    "pitch_label_deg": args.pitch, "actual_pitch_verified": False,
                    "width": IMAGE_WIDTH, "height": IMAGE_HEIGHT, "frame_id": frame_id,
                    "capture_monotonic_ns": capture_ns, "pixel_reference": "bbox_center",
                    "bands": bands, "detections": records, "measured_advance_mm": None,
                }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                print(f"已保存 {stem}.jpg / .json；这是测量记录，未写入正式距离表。")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
