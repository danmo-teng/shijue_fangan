#!/usr/bin/env python3
"""Read-only camera/BPU view for manually measuring fixed-pitch range bands."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


class RangeBands:
    """Editable image partitions, independently stored for each servo label."""

    def __init__(self, directory: Path, width: int, height: int, count: int):
        self.directory, self.width, self.height = directory, width, height
        # Convenient starting layout only: narrow at the top, wide below.
        # No camera geometry or measured distance is inferred from this curve.
        initial = [round(height * (i / count) ** 1.5) for i in range(count + 1)]
        self.edges = {pitch: list(initial) for pitch in (105, 120)}
        self.saved = {pitch: False for pitch in self.edges}
        for pitch in self.edges:
            path = self.path(pitch)
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                edges = data["boundaries_y"]
                if (data["pitch_label_deg"] != pitch or data["width"] != width or data["height"] != height
                        or not isinstance(edges, list) or len(edges) != count + 1
                        or any(type(y) is not int for y in edges)
                        or edges[0] != 0 or edges[-1] != height
                        or any(b - a < 16 for a, b in zip(edges, edges[1:]))):
                    raise ValueError("分区数量、尺寸或边界不匹配")
                self.edges[pitch], self.saved[pitch] = edges, True
            except FileNotFoundError:
                pass
            except (OSError, ValueError, KeyError, TypeError) as error:
                print(f"未采用分区设置 {path}：{error}；使用可调整初值。")

    def path(self, pitch: int) -> Path:
        return self.directory / f"{pitch}deg.json"

    def bands(self, pitch: int) -> list[dict]:
        edges = self.edges[pitch]
        return [dict(band=i + 1, y_min=a, y_max=b)
                for i, (a, b) in enumerate(zip(edges, edges[1:]))]

    def move(self, pitch: int, index: int, y: int) -> None:
        edges = self.edges[pitch]
        # Keep all five regions ordered and draggable; outer edges stay fixed.
        new_y = max(edges[index - 1] + 16, min(edges[index + 1] - 16, int(y)))
        if new_y != edges[index]:
            edges[index] = new_y
            self.saved[pitch] = False

    def save(self, pitch: int) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.path(pitch)
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(dict(
            pitch_label_deg=pitch, width=self.width, height=self.height,
            boundaries_y=self.edges[pitch], measured=False,
            purpose="view partitions only; not a measured distance calibration"),
            ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)
        self.saved[pitch] = True


def main() -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="测距标定画面：物资中心坐标、横向分区；不发送串口或运动命令")
    parser.add_argument("--pitch", type=int, choices=(105, 120), default=105,
                        help="当前手动设置的相机角度标签；本程序不控制舵机")
    parser.add_argument("--bands", type=int, default=5)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--model", type=Path, default=root / "models/best_bayese_320x320_nv12.bin")
    parser.add_argument("--output", type=Path, default=root / "captures/range_calibration")
    args = parser.parse_args()
    if not 2 <= args.bands <= 12:
        parser.error("--bands须为2..12")
    import cv2
    from run_yolo_x5 import X5YoloV8, load_labels, DEFAULT_LABELS, DEFAULT_YOLO_SCORE_THRESHOLD
    from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
    from rescue_vision.vision_protocol import IMAGE_WIDTH, IMAGE_HEIGHT

    detector = X5YoloV8(args.model, load_labels(DEFAULT_LABELS), DEFAULT_YOLO_SCORE_THRESHOLD, .45, 0, [0])
    camera = LatestFrameCamera(resolve_camera_device(args.device), IMAGE_WIDTH, IMAGE_HEIGHT, 180,
                               decoder="jpu", decode_fps=15, output_format="bgr")
    layout = RangeBands(args.output / "layouts", IMAGE_WIDTH, IMAGE_HEIGHT, args.bands)
    window = "Range calibration: drag lines | 1=105 2=120 SPACE=freeze S=save Q=quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1000, 800)
    frame = None
    detections = []
    frame_id = -1
    capture_ns = None
    frozen = False
    save_requested = False
    dragging = None
    message = ""

    def mouse(event, _x, y, flags, _parameter):
        nonlocal dragging
        # HighGUI maps mouse coordinates back to the displayed image pixels.
        if event == cv2.EVENT_LBUTTONDOWN:
            edges = layout.edges[args.pitch]
            index = min(range(1, len(edges) - 1), key=lambda i: abs(edges[i] - y))
            dragging = index if abs(edges[index] - y) <= 18 else None
        elif event == cv2.EVENT_MOUSEMOVE and dragging is not None:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                layout.move(args.pitch, dragging, y)
            else:
                dragging = None
        elif event == cv2.EVENT_LBUTTONUP:
            if dragging is not None:
                layout.move(args.pitch, dragging, y)
            dragging = None

    cv2.setMouseCallback(window, mouse)
    print("请先停止比赛进程，固定相机为105°或120°。1/2仅切换记录标签，不控制舵机。")
    print(f"当前{args.bands}个不等宽横向区域；拖动内部横线调整，空格冻结后仍可调整。")
    print("1/2切换105°/120°各自的分区；S保存当前角度分区、截图和中心点，Q退出。")
    print(f"分区设置自动从 {layout.directory} 读取；初值仅方便调整，不代表已实测距离。")
    print("请记录每个区域车体到夹爪入口的实际应走距离及预留补进余量，不以相机斜距代替。")
    try:
        camera.start()
        while True:
            error = camera.check_error()
            if error is not None:
                raise RuntimeError(error)
            packet = camera.latest()
            if not frozen and not save_requested and packet is not None and packet.frame_id != frame_id:
                frame_id, capture_ns = packet.frame_id, packet.published_ns
                frame = packet.image.copy()
                found, _ = detector.infer(frame)
                detections = [d for d in found if d.class_name in {
                    "green_supply", "core_black", "injured_orange", "danger_cyan"}]
            if frame is None:
                if cv2.waitKey(20) & 0xFF in (ord("q"), 27):
                    break
                continue
            shown = frame.copy()
            bands = layout.bands(args.pitch)
            for band in bands:
                y = band["y_min"]
                color = (0, 220, 255) if dragging == band["band"] - 1 else (255, 200, 0)
                cv2.line(shown, (0, y), (IMAGE_WIDTH - 1, y), color, 3)
                if y > 0:
                    cv2.rectangle(shown, (IMAGE_WIDTH - 100, y - 10), (IMAGE_WIDTH - 10, y + 10), color, -1)
                cv2.putText(shown, f'B{band["band"]}: Y[{y},{band["y_max"]})',
                            (8, min(band["y_max"] - 3, y + 65)), cv2.FONT_HERSHEY_SIMPLEX,
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
            cv2.putText(shown, f'Drag lines | {"SAVED" if layout.saved[args.pitch] else "UNSAVED: press S"} {message}',
                        (8, IMAGE_HEIGHT - 15), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
            if save_requested:
                save_requested = False
                layout.save(args.pitch)
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
                print(f"当前角度分区已保存：{layout.path(args.pitch)}")
                message = "Capture saved"
            cv2.imshow(window, shown)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key == ord(" "):
                frozen = not frozen
            elif key in (ord("1"), ord("2")):
                dragging = None
                args.pitch = 105 if key == ord("1") else 120
                # A frozen image belongs to its old angle label. Resume the
                # view when changing labels, ready for the manually moved camera.
                frozen = False
                message = ""
            elif key in (ord("s"), ord("S")):
                # Redraw after any mouse callback before saving matching lines/JSON.
                save_requested = True
    finally:
        camera.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
