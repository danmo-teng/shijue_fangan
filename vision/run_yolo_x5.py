#!/usr/bin/env python3
"""Low-latency YOLOv8 detection on the RDK X5 BPU.

The runtime expects an RDK X5 Bayes-e ``.bin`` produced by the official
D-Robotics Ultralytics conversion flow. Camera capture reuses the project's
latest-frame MJPEG/JPU path so inference never queues stale frames.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import hbm_runtime
import numpy as np

from rescue_vision.camera import LatestFrameCamera, resolve_camera_device
from rescue_vision.config import require_native_resolution
from rescue_vision.localizer import GroundLocalizer
from rescue_vision.vse import VseScaler


ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = ROOT / "models" / "best_bayese_320x320_nv12.bin"
DEFAULT_LABELS = ROOT / "config" / "yolo_labels.txt"
DEFAULT_OUTPUT = ROOT / "runtime_result.json"
LABEL_ALIASES = {
    "conmon": "green_supply",  # Preserve the class spelling stored in the checkpoint.
    "common": "green_supply",
    "kernel": "core_black",
    "risk": "danger_cyan",
    "wound": "injured_orange",
}


@dataclass(frozen=True)
class YoloDetection:
    class_id: int
    raw_class_name: str
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]

    @property
    def center(self) -> tuple[int, int]:
        x, y, width, height = self.bbox
        return x + width // 2, y + height // 2


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RDK X5 BPU YOLOv8实时物资识别")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS)
    parser.add_argument("--homography", type=Path, default=ROOT / "config" / "homography.txt")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=1024)
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--decoder", choices=("jpu", "software"), default="jpu")
    parser.add_argument("--decode-fps", type=float, default=60.0)
    parser.add_argument("--preprocess", choices=("auto", "vse", "cpu"), default="auto")
    parser.add_argument("--score-thres", type=float, default=0.50)
    parser.add_argument("--nms-thres", type=float, default=0.45)
    parser.add_argument("--priority", type=int, default=0)
    parser.add_argument("--bpu-cores", type=int, nargs="+", default=[0, 1])
    parser.add_argument("--window-mode", choices=("fullscreen", "normal"), default="normal")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--display-fps", type=float, default=15.0)
    parser.add_argument("--display-width", type=int, default=640)
    parser.add_argument("--display-height", type=int, default=512)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--output-fps", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until stopped")
    return parser.parse_args()


def load_labels(path: Path) -> list[str]:
    labels = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not labels:
        raise ValueError(f"类别文件为空：{path}")
    return labels


def dequantize(tensor: np.ndarray, quant) -> np.ndarray:
    quant_name = getattr(quant.quant_type, "name", str(quant.quant_type)).upper()
    if "SCALE" not in quant_name:
        return tensor.astype(np.float32, copy=False)
    values = tensor.astype(np.float32)
    scale = np.asarray(quant.scale, dtype=np.float32)
    zero = np.asarray(quant.zero_point, dtype=np.float32)
    if zero.size == 0:
        zero = np.zeros_like(scale)
    if scale.ndim == 0 or scale.size == 1:
        return (values - float(zero.reshape(-1)[0])) * float(scale.reshape(-1)[0])
    if values.ndim == 2 and scale.size == values.shape[-1]:
        return (values - zero.reshape(1, -1)) * scale.reshape(1, -1)
    shape = [1] * values.ndim
    shape[int(quant.axis)] = scale.size
    return (values - zero.reshape(shape)) * scale.reshape(shape)


def as_nhwc(tensor: np.ndarray, channels: int) -> np.ndarray:
    if tensor.ndim != 4:
        raise ValueError(f"YOLO输出必须为4维，当前形状为{tensor.shape}")
    if tensor.shape[-1] == channels:
        return tensor
    if tensor.shape[1] == channels:
        return tensor.transpose(0, 2, 3, 1)
    raise ValueError(f"无法在输出形状{tensor.shape}中找到{channels}个通道")


def sigmoid(values: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(values, -30.0, 30.0)))


def dfl_decode(box_rows: np.ndarray, selected: np.ndarray,
               height: int, width: int, stride: int) -> np.ndarray:
    distributions = box_rows.reshape(-1, 4, 16)
    distributions -= distributions.max(axis=2, keepdims=True)
    probabilities = np.exp(distributions)
    probabilities /= probabilities.sum(axis=2, keepdims=True)
    distances = (probabilities * np.arange(16, dtype=np.float32)).sum(axis=2)
    grid_x, grid_y = np.meshgrid(
        np.arange(width, dtype=np.float32) + 0.5,
        np.arange(height, dtype=np.float32) + 0.5,
    )
    anchors = np.stack((grid_x.reshape(-1), grid_y.reshape(-1)), axis=1)[selected]
    top_left = anchors - distances[:, :2]
    bottom_right = anchors + distances[:, 2:]
    return np.concatenate((top_left, bottom_right), axis=1) * float(stride)


def classwise_nms(boxes: np.ndarray, scores: np.ndarray, classes: np.ndarray, threshold: float) -> list[int]:
    keep: list[int] = []
    for class_id in np.unique(classes):
        indices = np.flatnonzero(classes == class_id)
        x1, y1, x2, y2 = boxes[indices].T
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores[indices].argsort()[::-1]
        while order.size:
            current = int(order[0])
            keep.append(int(indices[current]))
            if order.size == 1:
                break
            remaining = order[1:]
            xx1 = np.maximum(x1[current], x1[remaining])
            yy1 = np.maximum(y1[current], y1[remaining])
            xx2 = np.minimum(x2[current], x2[remaining])
            yy2 = np.minimum(y2[current], y2[remaining])
            intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[current] + areas[remaining] - intersection
            iou = intersection / np.maximum(union, 1e-6)
            order = remaining[iou <= threshold]
    return keep


def letterbox(image: np.ndarray, width: int, height: int) -> tuple[np.ndarray, float, float, float]:
    image_height, image_width = image.shape[:2]
    scale = min(width / image_width, height / image_height)
    resized_width = max(2, int(round(image_width * scale)) // 2 * 2)
    resized_height = max(2, int(round(image_height * scale)) // 2 * 2)
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    left = (width - resized_width) // 2
    top = (height - resized_height) // 2
    canvas = np.full((height, width, 3), 114, dtype=np.uint8)
    canvas[top:top + resized_height, left:left + resized_width] = resized
    return canvas, scale, float(left), float(top)


def bgr_to_nv12(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    area = width * height
    i420 = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    y = i420[:area]
    u = i420[area:area + area // 4]
    v = i420[area + area // 4:]
    nv12 = np.empty(area * 3 // 2, dtype=np.uint8)
    nv12[:area] = y
    nv12[area::2] = u
    nv12[area + 1::2] = v
    return nv12.reshape(1, height * 3 // 2, width, 1)


def nv12_letterbox(image: np.ndarray, content_width: int, content_height: int,
                   input_width: int, input_height: int) -> tuple[np.ndarray, float, float]:
    if image.shape != (content_height * 3 // 2, content_width):
        raise ValueError(f"NV12缩放结果形状错误：{image.shape}")
    left = (input_width - content_width) // 2
    top = (input_height - content_height) // 2
    if min(left, top) < 0 or left % 2 or top % 2:
        raise ValueError("NV12 letterbox边距必须是非负偶数")
    canvas = np.empty((input_height * 3 // 2, input_width), dtype=np.uint8)
    canvas[:input_height].fill(114)
    canvas[input_height:].fill(128)
    canvas[top:top + content_height, left:left + content_width] = image[:content_height]
    uv_top = input_height + top // 2
    canvas[uv_top:uv_top + content_height // 2, left:left + content_width] = image[content_height:]
    return canvas.reshape(1, input_height * 3 // 2, input_width, 1), float(left), float(top)


class X5YoloV8:
    def __init__(self, model_path: Path, labels: list[str], score: float, nms: float,
                 priority: int, bpu_cores: list[int]) -> None:
        if not model_path.exists():
            raise FileNotFoundError(
                f"缺少X5 BPU模型：{model_path}\n"
                "yolo.zip内的model.onnx/model.rknn不能直接替代，请先在电脑端转换为Bayes-e .bin。"
            )
        self.labels = labels
        self.score_threshold = score
        self.nms_threshold = nms
        self.runtime = hbm_runtime.HB_HBMRuntime(str(model_path))
        if len(self.runtime.model_names) != 1:
            raise ValueError("运行脚本要求.bin中恰好包含一个YOLO模型")
        self.model_name = self.runtime.model_names[0]
        self.input_name = self.runtime.input_names[self.model_name][0]
        input_shape = self.runtime.input_shapes[self.model_name][self.input_name]
        self.input_height = int(input_shape[2])
        self.input_width = int(input_shape[3])
        self.output_names = self.runtime.output_names[self.model_name]
        self.output_shapes = self.runtime.output_shapes[self.model_name]
        self.output_quants = self.runtime.output_quants[self.model_name]
        self.runtime.set_scheduling_params(
            priority={self.model_name: priority},
            bpu_cores={self.model_name: bpu_cores},
        )
        self.flat_output = self._discover_flat_output()
        self.layers = [] if self.flat_output else self._discover_layers()

    def _discover_flat_output(self) -> str | None:
        if len(self.output_names) != 1:
            return None
        channels = 4 + len(self.labels)
        for name in self.output_names:
            shape = self.output_shapes[name]
            if len(shape) == 4 and channels in shape and math.prod(shape) % channels == 0:
                return name
        return None

    def _discover_layers(self) -> list[tuple[str, str, int]]:
        boxes: dict[tuple[int, int], str] = {}
        classes: dict[tuple[int, int], str] = {}
        for name in self.output_names:
            shape = self.output_shapes[name]
            if len(shape) != 4:
                continue
            if shape[-1] in (64, len(self.labels)):
                height, width, channels = int(shape[1]), int(shape[2]), int(shape[3])
            elif shape[1] in (64, len(self.labels)):
                channels, height, width = int(shape[1]), int(shape[2]), int(shape[3])
            else:
                continue
            key = (height, width)
            if channels == 64:
                boxes[key] = name
            elif channels == len(self.labels):
                classes[key] = name
        keys = sorted(set(boxes) & set(classes), key=lambda item: item[0] * item[1], reverse=True)
        if len(keys) != 3:
            raise ValueError(
                f"无法识别YOLOv8三层输出；labels={len(self.labels)}，outputs={self.output_shapes}"
            )
        layers: list[tuple[str, str, int]] = []
        for height, width in keys:
            stride_x = self.input_width / width
            stride_y = self.input_height / height
            if stride_x != stride_y or not float(stride_x).is_integer():
                raise ValueError(f"不支持的YOLO特征层尺寸：{height}x{width}")
            layers.append((classes[(height, width)], boxes[(height, width)], int(stride_x)))
        return layers

    def infer(self, image: np.ndarray) -> tuple[list[YoloDetection], dict[str, float]]:
        started = time.perf_counter()
        prepared, scale, pad_x, pad_y = letterbox(image, self.input_width, self.input_height)
        tensor = bgr_to_nv12(prepared)
        after_preprocess = time.perf_counter()
        return self._infer_tensor(
            tensor, image.shape[1], image.shape[0], scale, pad_x, pad_y,
            started, after_preprocess,
        )

    def infer_nv12(self, image: np.ndarray, source_width: int, source_height: int,
                   scaler: VseScaler) -> tuple[list[YoloDetection], dict[str, float]]:
        started = time.perf_counter()
        scaled = scaler.scale(image)
        tensor, pad_x, pad_y = nv12_letterbox(
            scaled, scaler.output_width, scaler.output_height,
            self.input_width, self.input_height,
        )
        scale = min(self.input_width / source_width, self.input_height / source_height)
        after_preprocess = time.perf_counter()
        return self._infer_tensor(
            tensor, source_width, source_height, scale, pad_x, pad_y,
            started, after_preprocess,
        )

    def _infer_tensor(self, tensor: np.ndarray, original_width: int, original_height: int,
                      scale: float, pad_x: float, pad_y: float,
                      started: float, after_preprocess: float
                      ) -> tuple[list[YoloDetection], dict[str, float]]:
        nested = {self.model_name: {self.input_name: tensor}}
        outputs = self.runtime.run(nested)[self.model_name]
        after_inference = time.perf_counter()

        boxes_all: list[np.ndarray] = []
        scores_all: list[np.ndarray] = []
        classes_all: list[np.ndarray] = []
        if self.flat_output:
            prediction = dequantize(
                np.asarray(outputs[self.flat_output]), self.output_quants[self.flat_output]
            ).squeeze()
            channels = 4 + len(self.labels)
            if prediction.ndim != 2:
                raise ValueError(f"单张量YOLO输出形状不受支持：{prediction.shape}")
            if prediction.shape[0] == channels:
                prediction = prediction.T
            elif prediction.shape[1] != channels:
                raise ValueError(f"单张量YOLO输出缺少{channels}个通道：{prediction.shape}")
            class_scores = prediction[:, 4:]
            class_ids = class_scores.argmax(axis=1)
            scores = class_scores[np.arange(len(class_scores)), class_ids]
            if scores.min(initial=0.0) < 0.0 or scores.max(initial=1.0) > 1.0:
                scores = sigmoid(scores)
            selected = np.flatnonzero(scores >= self.score_threshold)
            if selected.size:
                xywh = prediction[selected, :4]
                boxes_all.append(np.column_stack((
                    xywh[:, 0] - xywh[:, 2] * 0.5,
                    xywh[:, 1] - xywh[:, 3] * 0.5,
                    xywh[:, 0] + xywh[:, 2] * 0.5,
                    xywh[:, 1] + xywh[:, 3] * 0.5,
                )))
                scores_all.append(scores[selected])
                classes_all.append(class_ids[selected])
        for class_name, box_name, stride in self.layers:
            class_output = as_nhwc(
                dequantize(np.asarray(outputs[class_name]), self.output_quants[class_name]),
                len(self.labels),
            )
            raw_box_output = as_nhwc(np.asarray(outputs[box_name]), 64)
            flattened = class_output.reshape(-1, len(self.labels))
            max_scores = flattened.max(axis=1)
            if "class_scores" in class_name.lower():
                # The ONNX included in yolo.zip exports probabilities.
                selected = np.flatnonzero(max_scores >= self.score_threshold)
                scores = max_scores[selected]
            else:
                # Official X5 exports expose logits. Filter before sigmoid so
                # thousands of low-confidence grid cells remain cheap.
                raw_threshold = -math.log(1.0 / self.score_threshold - 1.0)
                selected = np.flatnonzero(max_scores >= raw_threshold)
                scores = sigmoid(max_scores[selected])
            if selected.size:
                class_ids = flattened[selected].argmax(axis=1)
                # Bounding-box tensors are the largest quantized outputs. Only
                # copy and dequantize rows whose class score passed the filter.
                box_rows = raw_box_output.reshape(-1, 64)[selected]
                box_rows = dequantize(box_rows, self.output_quants[box_name])
                boxes_all.append(dfl_decode(
                    box_rows, selected, raw_box_output.shape[1], raw_box_output.shape[2], stride
                ))
                scores_all.append(scores)
                classes_all.append(class_ids)

        detections: list[YoloDetection] = []
        if boxes_all:
            boxes = np.concatenate(boxes_all)
            scores = np.concatenate(scores_all)
            classes = np.concatenate(classes_all)
            keep = classwise_nms(boxes, scores, classes, self.nms_threshold)
            boxes = boxes[keep]
            scores = scores[keep]
            classes = classes[keep]
            boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / scale
            boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / scale
            boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, original_width - 1)
            boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, original_height - 1)
            for box, class_id, confidence in zip(boxes, classes, scores):
                x1, y1, x2, y2 = (int(round(value)) for value in box)
                if x2 <= x1 or y2 <= y1:
                    continue
                raw_name = self.labels[int(class_id)]
                detections.append(YoloDetection(
                    int(class_id), raw_name, LABEL_ALIASES.get(raw_name, raw_name),
                    float(confidence), (x1, y1, x2 - x1, y2 - y1),
                ))
        finished = time.perf_counter()
        return detections, {
            "preprocess_ms": (after_preprocess - started) * 1000.0,
            "inference_ms": (after_inference - after_preprocess) * 1000.0,
            "postprocess_ms": (finished - after_inference) * 1000.0,
            "total_ms": (finished - started) * 1000.0,
        }


def draw(image: np.ndarray, detections: list[YoloDetection], timing: dict[str, float], fps: float) -> np.ndarray:
    view = image.copy()
    colors = ((55, 220, 55), (40, 40, 220), (220, 220, 30), (30, 150, 255))
    for detection in detections:
        x, y, width, height = detection.bbox
        color = colors[detection.class_id % len(colors)]
        cv2.rectangle(view, (x, y), (x + width, y + height), color, 3)
        center = detection.center
        cv2.drawMarker(view, center, color, cv2.MARKER_CROSS, 18, 2)
        label = f"{detection.class_name} {detection.confidence:.2f} ({center[0]},{center[1]})"
        cv2.putText(view, label, (x, max(28, y - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, color, 2, cv2.LINE_AA)
    status = (
        f"YOLO {fps:.1f} FPS  pre={timing['preprocess_ms']:.1f}ms "
        f"bpu={timing['inference_ms']:.1f}ms post={timing['postprocess_ms']:.1f}ms"
    )
    cv2.putText(view, status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.68, (255, 255, 0), 2, cv2.LINE_AA)
    return view


def fit_complete_image(image: np.ndarray, canvas_width: int, canvas_height: int) -> np.ndarray:
    scale = min(canvas_width / image.shape[1], canvas_height / image.shape[0])
    width = max(1, int(round(image.shape[1] * scale)))
    height = max(1, int(round(image.shape[0] * scale)))
    resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    top = (canvas_height - height) // 2
    left = (canvas_width - width) // 2
    return cv2.copyMakeBorder(resized, top, canvas_height - height - top,
                              left, canvas_width - width - left,
                              cv2.BORDER_CONSTANT, value=(0, 0, 0))


def result_document(frame_id: int, frame_age_ms: float,
                    detections: list[YoloDetection], timing: dict[str, float],
                    localizer: GroundLocalizer) -> dict:
    items = []
    tracks = []
    for index, item in enumerate(detections, start=1):
        x, y, width, height = item.bbox
        ground = localizer.image_to_ground((x + width * 0.5, y + height))
        position = ground if ground is not None else item.center
        record = {
            "class_id": item.class_id,
            "raw_class": item.raw_class_name,
            "class": item.class_name,
            "confidence": round(item.confidence, 4),
            "bbox": list(item.bbox),
            "center": list(item.center),
        }
        items.append(record)
        tracks.append({
            "id": f"Y{index}",
            "class": item.class_name,
            "confidence": round(item.confidence, 4),
            "state": "CONFIRMED",
            "position": [round(float(value), 2) for value in position],
            "coordinate_system": "ground_mm" if ground is not None else "image_px",
            "bbox": list(item.bbox),
        })
    return {
        "schema_version": 1,
        "timestamp_monotonic_ns": time.monotonic_ns(),
        "frame_id": frame_id,
        "frame_age_ms": round(frame_age_ms, 3),
        "coordinate_system": "image_px_1280x1024",
        "calibrated": localizer.calibrated,
        "timing_ms": {key: round(value, 3) for key, value in timing.items()},
        "detections": items,
        "tracks": tracks,
    }


class InferenceLoop:
    """Run inference/output independently so X11 rendering cannot throttle it."""

    def __init__(self, detector: X5YoloV8, camera: LatestFrameCamera,
                 output: Path, output_fps: float, scaler: VseScaler | None,
                 source_width: int, source_height: int,
                 localizer: GroundLocalizer) -> None:
        self.detector = detector
        self.camera = camera
        self.output = output
        self.output_fps = output_fps
        self.scaler = scaler
        self.source_width = source_width
        self.source_height = source_height
        self.localizer = localizer
        self.lock = threading.Lock()
        self.running = False
        self.thread: threading.Thread | None = None
        self.latest = None
        self.error: str | None = None

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._run, name="x5-yolo-inference", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        started = time.perf_counter()
        last_frame_id = 0
        completed = report_completed = 0
        fps = 0.0
        report_started = started
        next_report = started + 1.0
        next_output = started
        try:
            while self.running:
                packet = self.camera.latest()
                if packet is None or packet.frame_id == last_frame_id:
                    time.sleep(0.0005)
                    continue
                if packet.pixel_format == "nv12":
                    if self.scaler is None:
                        raise RuntimeError("收到NV12帧但VSE缩放器未初始化")
                    detections, timing = self.detector.infer_nv12(
                        packet.image, self.source_width, self.source_height, self.scaler
                    )
                else:
                    detections, timing = self.detector.infer(packet.image)
                completed += 1
                last_frame_id = packet.frame_id
                now = time.perf_counter()
                if now >= next_report:
                    fps = (completed - report_completed) / max(now - report_started, 1e-6)
                    print(
                        f"detect={fps:.1f}fps "
                        f"decode={self.camera.decoded_count()/max(now-started,1e-6):.1f}fps "
                        f"pre={timing['preprocess_ms']:.1f}ms "
                        f"bpu={timing['inference_ms']:.1f}ms "
                        f"post={timing['postprocess_ms']:.1f}ms targets={len(detections)}"
                    )
                    report_started, report_completed = now, completed
                    next_report = now + 1.0
                if self.output_fps > 0 and now >= next_output:
                    next_output = now + 1.0 / self.output_fps
                    age_ms = (time.monotonic_ns() - packet.published_ns) / 1_000_000.0
                    document = result_document(
                        packet.frame_id, age_ms, detections, timing, self.localizer
                    )
                    temporary = self.output.with_suffix(self.output.suffix + ".tmp")
                    temporary.write_text(json.dumps(document, ensure_ascii=False), encoding="utf-8")
                    temporary.replace(self.output)
                with self.lock:
                    self.latest = (packet, detections, timing, fps)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            self.running = False

    def snapshot(self):
        with self.lock:
            return self.latest, self.error

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=3.0)
            self.thread = None


def main() -> int:
    args = arguments()
    require_native_resolution(args.width, args.height)
    if not 0.0 < args.score_thres < 1.0 or not 0.0 < args.nms_thres < 1.0:
        raise ValueError("score-thres和nms-thres必须在0..1之间")
    if args.display_fps <= 0 or args.display_width <= 0 or args.display_height <= 0:
        raise ValueError("显示帧率和尺寸必须为正数")
    labels = load_labels(args.labels)
    localizer = GroundLocalizer.load(args.homography, (args.width, args.height))
    detector = X5YoloV8(args.model, labels, args.score_thres, args.nms_thres,
                        args.priority, args.bpu_cores)
    scaler: VseScaler | None = None
    use_vse = args.preprocess in ("auto", "vse") and args.decoder == "jpu"
    if args.preprocess == "vse" and args.decoder != "jpu":
        raise ValueError("VSE NV12路径要求--decoder jpu")
    if use_vse:
        scale = min(detector.input_width / args.width, detector.input_height / args.height)
        content_width = max(2, int(round(args.width * scale)) // 2 * 2)
        content_height = max(2, int(round(args.height * scale)) // 2 * 2)
        try:
            scaler = VseScaler(args.width, args.height, content_width, content_height)
        except Exception as error:
            if args.preprocess == "vse":
                raise
            print(f"警告：VSE初始化失败，回退CPU预处理：{error}")
    camera = LatestFrameCamera(
        resolve_camera_device(args.device), args.width, args.height, args.camera_fps,
        decoder=args.decoder, decode_fps=args.decode_fps,
        output_format="nv12" if scaler is not None else "bgr",
    )
    print(
        f"模型：{args.model}，输入{detector.input_width}x{detector.input_height}，"
        f"类别={labels}，BPU cores={args.bpu_cores}"
    )
    print(
        f"相机：MJPEG {args.width}x{args.height}@{args.camera_fps}，"
        f"{args.decoder}解码上限{args.decode_fps:g} FPS，"
        f"预处理={'JPU NV12 + VSE' if scaler is not None else 'CPU BGR'}"
    )

    running = True
    def stop(_signal, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    window = "RDK X5 YOLO material detection"
    if not args.no_display:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        if args.window_mode == "fullscreen":
            cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    camera.start()
    started = time.perf_counter()
    inference = InferenceLoop(
        detector, camera, args.output, args.output_fps, scaler,
        args.width, args.height, localizer,
    )
    inference.start()
    next_display = started
    try:
        while running:
            error = camera.check_error()
            if error:
                raise RuntimeError(error)
            snapshot, inference_error = inference.snapshot()
            if inference_error:
                raise RuntimeError(inference_error)
            now = time.perf_counter()
            if not args.no_display:
                if snapshot is not None and now >= next_display:
                    packet, detections, timing, fps = snapshot
                    display_image = (
                        cv2.cvtColor(packet.image, cv2.COLOR_YUV2BGR_NV12)
                        if packet.pixel_format == "nv12" else packet.image
                    )
                    shown = draw(display_image, detections, timing, fps)
                    cv2.imshow(window, fit_complete_image(
                        shown, args.display_width, args.display_height
                    ))
                    next_display = now + 1.0 / args.display_fps
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
            if args.duration > 0 and now - started >= args.duration:
                break
            time.sleep(0.001)
    finally:
        inference.stop()
        camera.stop()
        if scaler is not None:
            scaler.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}")
        raise SystemExit(1)
