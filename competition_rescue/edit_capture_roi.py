#!/usr/bin/env python3
"""Edit the claw-capture polygon on the live competition YOLO image."""
from __future__ import annotations

import argparse
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
VISION_ROOT = PROJECT_ROOT / "vision"
sys.path.insert(0, str(VISION_ROOT))
sys.path.insert(0, str(ROOT))

from capture_grip_bands import GripBands, CLASSES  # noqa: E402
from capture_roi_trails import CaptureTrail  # noqa: E402
from sweep_boundaries import load_boundaries, save_boundaries
from capture_roi import (  # noqa: E402
    CaptureRois,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    bbox_in_capture_roi,
    capture_side_for_bbox,
    capture_overlap_for_class,
    load_capture_rois,
    save_capture_rois,
    split_capture_roi,
)
from rescue_vision.camera import LatestFrameCamera, resolve_camera_device  # noqa: E402
from rescue_vision.vse import VseScaler  # noqa: E402
from run_yolo_x5 import (  # noqa: E402
    DEFAULT_LABELS,
    DEFAULT_MODEL,
    DEFAULT_YOLO_SCORE_THRESHOLD,
    X5Yolo26,
    load_labels,
)


WINDOW_NAME = "capture_roi_editor"
DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 768
PANEL_HEIGHT = 230
INITIAL_WINDOW_WIDTH = 640  # Full toolbar fits the RDK's 1024x768 desktop.
GREEN_SUPPLY_SCORE_THRESHOLD = DEFAULT_YOLO_SCORE_THRESHOLD


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="正式比赛夹内多边形区域标定")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--decoder", choices=("jpu", "software"), default="jpu")
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--decode-fps", type=float, default=60.0)
    parser.add_argument("--vision-fps", type=float, default=50.0)
    parser.add_argument("--preprocess", choices=("auto", "vse", "cpu"), default="auto")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--score-thres", type=float, default=DEFAULT_YOLO_SCORE_THRESHOLD)
    parser.add_argument("--nms-thres", type=float, default=0.45)
    parser.add_argument("--priority", type=int, default=0)
    parser.add_argument("--bpu-cores", type=int, nargs="+", default=[0, 1])
    parser.add_argument(
        "--output",
        type=Path,
        default=VISION_ROOT / "config/capture_roi.json",
    )
    parser.add_argument("--grip-bands", type=Path, default=VISION_ROOT / "config/capture_grip_bands.json")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    if args.vision_fps <= 0.0:
        raise ValueError("vision-fps must be positive")
    detector = X5Yolo26(
        args.model,
        load_labels(args.labels),
        args.score_thres,
        args.nms_thres,
        args.priority,
        args.bpu_cores,
    )
    scaler: VseScaler | None = None
    use_vse = args.preprocess in {"auto", "vse"} and args.decoder == "jpu"
    if args.preprocess == "vse" and args.decoder != "jpu":
        raise ValueError("VSE NV12 preprocessing requires JPU decoding")
    if use_vse:
        scale = min(
            detector.input_width / IMAGE_WIDTH,
            detector.input_height / IMAGE_HEIGHT,
        )
        content_width = max(2, int(round(IMAGE_WIDTH * scale)) // 2 * 2)
        content_height = max(2, int(round(IMAGE_HEIGHT * scale)) // 2 * 2)
        try:
            scaler = VseScaler(
                IMAGE_WIDTH, IMAGE_HEIGHT, content_width, content_height
            )
        except Exception:
            if args.preprocess == "vse":
                raise
            scaler = None

    camera = LatestFrameCamera(
        resolve_camera_device(args.device),
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        args.camera_fps,
        decoder=args.decoder,
        decode_fps=args.decode_fps,
        output_format="nv12" if scaler is not None else "bgr",
    )
    loaded_rois = load_capture_rois(args.output)
    roi_points = {
        "overall": list(loaded_rois.overall),
        "left": list(loaded_rois.left),
        "right": list(loaded_rois.right),
    }
    band_path = getattr(args, "grip_bands", args.output.with_name("capture_grip_bands.json"))
    loaded_bands = GripBands.load(band_path)
    band_points = {name: list(getattr(loaded_bands, name) or ()) for name in CLASSES}
    edit_band = None
    sweep_path = args.output.with_name('sweep_boundaries.json')
    sweep_data = load_boundaries(sweep_path)
    sweep_mode = None
    sweep_side = 'left'
    for angle in ('11500', '12000'):
        sweep_data.setdefault(angle, {'left': [], 'right': []})
    drag_line = None
    active_roi = "overall"
    status = "G:green bands  O:orange bands  P:ROI  S:save current mode"
    trail: CaptureTrail | None = None
    recording = False
    show_trails = True
    buttons = []
    font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 18)

    def trail_canvas():
        if trail is None:
            return latest_image.copy()
        canvas = trail.background.copy()
        trail.paint(canvas, (255, 210, 0))
        return canvas

    def save_bands():
        nonlocal status
        if any(len(values) == 1 for values in band_points.values()):
            status = "mark both lines before saving"
            return
        try:
            bands = GripBands(**{name: tuple(sorted(values)) if values else None
                                 for name, values in band_points.items()})
            bands.save(band_path)
            status = "grip bands saved; claw ROI unchanged"
        except (ValueError, OSError) as error:
            status = str(error)

    def trail_action(action):
        nonlocal trail, recording, show_trails, status, edit_band, drag_line, sweep_mode, sweep_side
        if action in {'11500', '12000'}:
            sweep_mode, edit_band = action, None
            status = f'{int(action)//100}deg sweep: left-click add points; right-click undo; S save'
            return
        if action in {'sweep_left', 'sweep_right'}:
            sweep_side = action.split('_')[1]
            return
        if action == 'save_sweep':
            if sweep_mode is None:
                status = 'select 115 or 120 sweep first'
                return
            try:
                saved = load_boundaries(sweep_path)
                saved[sweep_mode] = sweep_data[sweep_mode]
                save_boundaries(sweep_path, saved)
                status = 'sweep boundaries saved; restart competition to load'
            except (ValueError, KeyError, OSError) as error:
                status = str(error)
            return
        if action in {"roi", *CLASSES}:
            sweep_mode = None
            edit_band = None if action == "roi" else action
            drag_line = None
            status = "editing claw ROI" if edit_band is None else "140deg: click two lines; drag to adjust; S:save bands"
            return
        if action == "save_bands":
            save_bands()
            return
        if action == "record":
            if last_frame_id == 0:
                status = "wait for camera image before recording"
                return
            if not recording and trail is None:
                trail = CaptureTrail(latest_image)
            recording = not recording
            show_trails = True
        elif action == "clear":
            recording = False
            trail = None
        elif action == "view":
            show_trails = not show_trails
        elif action == "export":
            if trail is None:
                status = "no trail to export"
                return
            folder = args.output.parent / "capture_roi_trails"
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = folder / f"{stamp}_trail.png"
            if not cv2.imwrite(str(path), trail_canvas()):
                raise OSError(f"cannot save trail image: {path}")
            status = f"trail saved: {path.name} (ROI unchanged)"
            return
        status = "recording trail" if recording else "trail paused"

    def mouse(event: int, x: int, y: int, _flags: int, _data) -> None:
        nonlocal status, drag_line
        if event == cv2.EVENT_LBUTTONUP:
            drag_line = None
        if y >= DISPLAY_HEIGHT:
            if event == cv2.EVENT_LBUTTONDOWN:
                for x0, y0, x1, y1, action in buttons:
                    if x0 <= x < x1 and y0 <= y < y1:
                        trail_action(action)
                        break
            return
        points = roi_points["overall"]
        image_x = max(0, min(IMAGE_WIDTH - 1, round(x * IMAGE_WIDTH / DISPLAY_WIDTH)))
        image_y = max(0, min(IMAGE_HEIGHT - 1, round(y * IMAGE_HEIGHT / DISPLAY_HEIGHT)))
        if sweep_mode is not None:
            values = sweep_data[sweep_mode][sweep_side]
            if event == cv2.EVENT_LBUTTONDOWN:
                values.append([image_x, image_y])
            elif event == cv2.EVENT_RBUTTONDOWN and values:
                values.pop()
            status = f'{int(sweep_mode)//100}deg {sweep_side}: {len(values)} points; S save'
            return
        if edit_band is not None:
            values = band_points[edit_band]
            image_y = max(1, min(IMAGE_HEIGHT-1, image_y))
            if event == cv2.EVENT_LBUTTONDOWN:
                if len(values) < 2:
                    values.append(image_y)
                    drag_line = len(values)-1
                else:
                    drag_line = min(range(2), key=lambda i: abs(values[i]-image_y))
                    values[drag_line] = image_y
            elif event == cv2.EVENT_MOUSEMOVE and drag_line is not None and _flags & cv2.EVENT_FLAG_LBUTTON:
                values[drag_line] = image_y
            elif event == cv2.EVENT_RBUTTONDOWN:
                values.clear()
                drag_line = None
            status = f"{edit_band}: {sorted(values)}; S:save bands (ROI unchanged)"
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((image_x, image_y))
            status = f"point added: ({image_x}, {image_y})"
        elif event == cv2.EVENT_RBUTTONDOWN and points:
            removed = points.pop()
            status = f"point removed: {removed}"

    running = True

    def stop(_signal, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    # Keep drawing/mouse coordinates in the original canvas; HighGUI scales
    # its display. The former 918px-high window hid the new bottom buttons.
    cv2.resizeWindow(WINDOW_NAME, INITIAL_WINDOW_WIDTH,
                     round((DISPLAY_HEIGHT + PANEL_HEIGHT) * INITIAL_WINDOW_WIDTH / DISPLAY_WIDTH))
    cv2.setMouseCallback(WINDOW_NAME, mouse)
    camera.start()
    last_frame_id = 0
    next_inference = time.monotonic()
    latest_image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
    latest_detections = []
    try:
        while running:
            error = camera.check_error()
            if error:
                raise RuntimeError(error)
            packet = camera.latest()
            now = time.monotonic()
            if (
                packet is not None and
                packet.frame_id != last_frame_id and
                now >= next_inference
            ):
                last_frame_id = packet.frame_id
                next_inference = now + 1.0 / args.vision_fps
                latest_image = (
                    cv2.cvtColor(packet.image, cv2.COLOR_YUV2BGR_NV12)
                    if packet.pixel_format == "nv12" else packet.image.copy()
                )
                if packet.pixel_format == "nv12":
                    detections, _ = detector.infer_nv12(
                        packet.image, IMAGE_WIDTH, IMAGE_HEIGHT, scaler
                    )
                else:
                    detections, _ = detector.infer(packet.image)
                latest_detections = [
                    item for item in detections
                    if item.class_name != "green_supply" or
                    float(item.confidence) >= GREEN_SUPPLY_SCORE_THRESHOLD
                ]

                if recording:
                    trail.add(latest_image)

            shown = cv2.resize(
                trail_canvas() if show_trails else latest_image,
                (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                interpolation=cv2.INTER_AREA,
            )
            scale_x = DISPLAY_WIDTH / IMAGE_WIDTH
            scale_y = DISPLAY_HEIGHT / IMAGE_HEIGHT
            overall = tuple(roi_points["overall"])
            left, right = split_capture_roi(overall)
            roi_points["left"][:] = left
            roi_points["right"][:] = right
            rois = CaptureRois(overall, left, right)
            # Historical image pixels are only a drawing aid, never detections.
            preview_items = latest_detections if not show_trails or trail is None else ()
            for item in preview_items:
                x, y, width, height = item.bbox
                capture_side = (
                    capture_side_for_bbox(
                        item.bbox,
                        rois,
                        require_full_overlap=item.class_name == "core_black",
                        minimum_overlap_ratio=capture_overlap_for_class(item.class_name),
                    )
                    if all(
                        len(polygon) >= 3
                        for polygon in (rois.overall, rois.left, rois.right)
                    ) else None
                )
                inside_overall = (
                    bbox_in_capture_roi(
                        item.bbox,
                        rois.overall,
                        require_full_overlap=item.class_name == "core_black",
                        minimum_overlap_ratio=capture_overlap_for_class(item.class_name),
                        require_bottom_center=item.class_name != "green_supply",
                    )
                    if len(rois.overall) >= 3 else False
                )
                if inside_overall and capture_side is None and item.class_name == "green_supply":
                    capture_side = "left" if x + width * .5 <= IMAGE_WIDTH // 2 else "right"
                color = (
                    (0, 255, 255)
                    if capture_side is not None else
                    ((255, 0, 255) if inside_overall else (0, 200, 0))
                )
                p0 = (round(x * scale_x), round(y * scale_y))
                p1 = (
                    round((x + width) * scale_x),
                    round((y + height) * scale_y),
                )
                center = (
                    round((x + width * 0.5) * scale_x),
                    round((y + height * 0.5) * scale_y),
                )
                cv2.rectangle(shown, p0, p1, color, 2)
                cv2.circle(shown, center, 4, color, -1)
                cv2.putText(
                    shown,
                    f"{item.class_name} {item.confidence:.2f} {capture_side or '-'}",
                    (p0[0], max(18, p0[1] - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv2.LINE_AA,
                )
            for roi_name, color in (
                ("overall", (0, 255, 255)),
                ("left", (255, 128, 0)),
                ("right", (0, 128, 255)),
            ):
                if sweep_mode is not None:
                    continue
                points = roi_points[roi_name]
                if not points:
                    continue
                scaled = np.asarray([
                    (round(x * scale_x), round(y * scale_y))
                    for x, y in points
                ], dtype=np.int32)
                cv2.polylines(
                    shown,
                    [scaled],
                    len(points) >= 3,
                    color,
                    3 if roi_name == active_roi else 2,
                    cv2.LINE_AA,
                )
                if roi_name == active_roi:
                    for index, point in enumerate(scaled):
                        cv2.circle(shown, tuple(point), 5, (255, 255, 255), -1)
                        cv2.putText(
                            shown,
                            str(index + 1),
                            (int(point[0]) + 6, int(point[1]) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            (255, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )
            if edit_band is not None:
                lines = sorted(band_points[edit_band])
                for label, image_y in zip(("upper/middle", "middle/lower"), lines):
                    y = round(image_y * scale_y)
                    cv2.line(shown, (0, y), (DISPLAY_WIDTH-1, y), (0, 255, 255), 2)
                    cv2.putText(shown, f"{label} y={image_y}", (10, max(65, y-6)),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1, cv2.LINE_AA)
            if sweep_mode is not None:
                for side, color in [('left', (255, 220, 0)), ('right', (255, 0, 220))]:
                    values = sorted(sweep_data[sweep_mode][side], key=lambda p:p[1])
                    points = np.array([(round(x*scale_x), round(y*scale_y)) for x,y in values], dtype=np.int32)
                    if len(points) > 1:
                        cv2.polylines(shown, [points], False, color, 3)
                    for point in points:
                        cv2.circle(shown, tuple(point), 5, color, -1)
            cv2.rectangle(shown, (0, 0), (DISPLAY_WIDTH, 48), (0, 0, 0), -1)
            cv2.putText(
                shown,
                status,
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                shown,
                (f"SWEEP {int(sweep_mode)//100}deg / {sweep_side}" if sweep_mode else
                 f"active={active_roi} points={len(roi_points[active_roi])} yellow=accepted"),
                (10, 41),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            panel = Image.new("RGB", (DISPLAY_WIDTH, PANEL_HEIGHT), (28, 28, 28))
            draw = ImageDraw.Draw(panel)
            buttons.clear()
            entries = (("record", "暂停延时摄影" if recording else "开始／继续延时摄影"),
                       ("clear", "清除叠影"), ("view", "看实时" if show_trails else "看叠影"),
                       ("export", "保存叠影图片"))
            button_width = DISPLAY_WIDTH // len(entries)
            for column, (action, label) in enumerate(entries):
                x0, y0 = column * button_width + 3, 2
                x1, y1 = (column + 1) * button_width - 3, 34
                active = action == "record" and recording
                draw.rectangle((x0, y0, x1, y1), fill=(42, 108, 73) if active else (65, 65, 65))
                draw.text((x0 + 8, y0 + 3), label, font=font, fill="white")
                buttons.append((x0, y0 + DISPLAY_HEIGHT, x1, y1 + DISPLAY_HEIGHT, action))
            count = trail.samples if trail is not None else 0
            draw.text((8, 40), f"{'记录中' if recording else '已暂停'} · 已留影 {count} 次 · 开始前移开物块和手；固定相机和爪子。",
                      font=font, fill=(255, 220, 120))
            band_entries = (("roi", "P：编辑爪内ROI"), ("green_supply", "G：绿色分区"),
                            ("injured_orange", "O：橙色分区"), ("save_bands", "保存绿橙分区"))
            for column, (action, label) in enumerate(band_entries):
                x0, y0, x1, y1 = column*button_width+3, 75, (column+1)*button_width-3, 108
                active = action == edit_band or (action == "roi" and edit_band is None)
                draw.rectangle((x0, y0, x1, y1), fill=(42, 108, 73) if active else (65, 65, 65))
                draw.text((x0+8, y0+3), label, font=font, fill="white")
                buttons.append((x0, y0+DISPLAY_HEIGHT, x1, y1+DISPLAY_HEIGHT, action))
            hint = ("分区：点两次画线，拖动调整；S保存，P回ROI；相机固定140°。" if edit_band else
                    "按G画绿色分区，按O画橙色分区；分区单独保存，不改爪内ROI。")
            draw.text((8, 116), hint, font=font, fill=(255, 220, 120))
            sweep_entries = [('11500','扫障115°'),('12000','扫障120°'),
                             ('sweep_left','画左边界'),('sweep_right','画右边界'),('save_sweep','保存本角度')]
            for col,(action,label) in enumerate(sweep_entries):
                x0,y0,x1,y1=col*192+3,150,(col+1)*192-3,184
                active=action==sweep_mode or action=='sweep_'+sweep_side
                draw.rectangle((x0,y0,x1,y1),fill=(42,108,73) if active else (65,65,65))
                draw.text((x0+8,y0+3),label,font=font,fill='white')
                buttons.append((x0,y0+DISPLAY_HEIGHT,x1,y1+DISPLAY_HEIGHT,action))
            draw.text((8,192),'先人工调整实际相机角度；沿左右边界连点描线，右键撤点，C清当前线。',font=font,fill=(255,220,120))
            shown = np.vstack((shown, cv2.cvtColor(np.asarray(panel), cv2.COLOR_RGB2BGR)))
            cv2.imshow(WINDOW_NAME, shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("g"), ord("G")):
                trail_action("green_supply")
            elif key in (ord("o"), ord("O")):
                trail_action("injured_orange")
            elif key in (ord("p"), ord("P")):
                trail_action("roi")
            if sweep_mode is not None:
                if key in (ord('s'), ord('S')):
                    trail_action('save_sweep')
                elif key in (ord('c'), ord('C')):
                    sweep_data[sweep_mode][sweep_side].clear()
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
                continue
            if edit_band is not None:
                if key in (ord("s"), ord("S")):
                    save_bands()
                elif key in (ord("c"), ord("C")):
                    band_points[edit_band].clear()
                    drag_line = None
                elif key in (ord("r"), ord("R")):
                    reloaded = GripBands.load(band_path)
                    band_points[edit_band][:] = getattr(reloaded, edit_band) or ()
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    break
                continue
            if key == ord("1"):
                active_roi = "overall"
                status = "editing overall ROI"
            elif key == ord("2"):
                active_roi = "overall"
                status = "left ROI is derived automatically from x=640"
            elif key == ord("3"):
                active_roi = "overall"
                status = "right ROI is derived automatically from x=640"
            if key in (ord("c"), ord("C")):
                roi_points["overall"].clear()
                status = "overall polygon cleared"
            elif key in (ord("r"), ord("R")):
                loaded_rois = load_capture_rois(args.output)
                roi_points["overall"][:] = loaded_rois.overall
                roi_points["left"][:] = loaded_rois.left
                roi_points["right"][:] = loaded_rois.right
                status = "all polygons reloaded"
            elif key in (ord("s"), ord("S")):
                if len(roi_points["overall"]) < 3:
                    status = "overall ROI requires at least three points"
                else:
                    left, right = split_capture_roi(
                        tuple(roi_points["overall"])
                    )
                    save_capture_rois(
                        args.output,
                        CaptureRois(
                            tuple(roi_points["overall"]),
                            left,
                            right,
                        ),
                    )
                    status = f"saved: {args.output}"
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        camera.stop()
        if scaler is not None:
            scaler.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
