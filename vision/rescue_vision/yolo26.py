"""Shared YOLO26 X5 decoder for competition and live benchmarks.

Reference: D-Robotics rdk_model_zoo ultralytics_yolo26/runtime/python/yolo26_det.py.
The six-head model uses raw class logits and four LTRB distances, not YOLOv8 DFL.
"""
from __future__ import annotations
import math
import time
from pathlib import Path
import cv2
import numpy as np

DEFAULT_MODEL = Path(__file__).resolve().parents[2] / "yolo26n/x5_output_640/yolo26n_6cls_640_bpu_bayese_640x640_nv12.bin"

class X5Yolo26:
    def __init__(self, model, labels, threshold=.2, nms=.45, priority=0, bpu_cores=None):
        import hbm_runtime
        from run_yolo_x5 import load_labels
        self.labels = list(labels) if isinstance(labels, (list, tuple)) else load_labels(labels)
        self.threshold, self.nms = threshold, nms
        self.runtime = hbm_runtime.HB_HBMRuntime(str(model))
        self.name = self.runtime.model_names[0]
        self.input = self.runtime.input_names[self.name][0]
        shape = self.runtime.input_shapes[self.name][self.input]
        if tuple(shape) != (1, 3, 640, 640):
            raise ValueError(f'Expected 1x3x640x640, got {shape}')
        self.runtime.set_scheduling_params(priority={self.name: priority}, bpu_cores={self.name: bpu_cores or [0]})
        self.shapes = self.runtime.output_shapes[self.name]
        self.quants = self.runtime.output_quants[self.name]
        self.input_width = self.input_height = 640
        self.layers = []
        for size in (80, 40, 20):
            classes = [n for n, s in self.shapes.items() if tuple(s) == (1, size, size, len(self.labels))]
            boxes = [n for n, s in self.shapes.items() if tuple(s) == (1, size, size, 4)]
            if len(classes) != 1 or len(boxes) != 1:
                raise ValueError(f'Unsupported YOLO26 outputs/labels: {self.shapes}')
            y, x = np.mgrid[:size, :size]
            anchors = np.column_stack((x.ravel(), y.ravel())).astype(np.float32) + .5
            self.layers.append((classes[0], boxes[0], 640 // size, anchors))
        self.logit_threshold = math.log(threshold / (1 - threshold))

    def run(self, tensor):
        return self.runtime.run({self.name: {self.input: tensor}})[self.name]

    def decode(self, outputs):
        from run_yolo_x5 import dequantize, sigmoid
        candidates = []
        for cn, bn, stride, anchors in self.layers:
            cls = np.asarray(outputs[cn])
            # hbm_runtime normally returns already-dequantized float32.
            if not np.issubdtype(cls.dtype, np.floating):
                cls = dequantize(cls, self.quants[cn])
            cls = cls.reshape(-1, len(self.labels))
            scores = cls.max(axis=1)
            selected = np.flatnonzero(scores >= self.logit_threshold)
            if not selected.size:
                continue
            box = np.asarray(outputs[bn]).reshape(-1, 4)[selected]
            if not np.issubdtype(box.dtype, np.floating):
                box = dequantize(box, self.quants[bn])
            grid = anchors[selected]
            xyxy = np.concatenate((grid - box[:, :2], grid + box[:, 2:]), axis=1) * stride
            xyxy = np.clip(xyxy, 0, 639)
            valid = (xyxy[:, 2] > xyxy[:, 0]) & (xyxy[:, 3] > xyxy[:, 1])
            candidates.append(np.column_stack((xyxy[valid], sigmoid(scores[selected][valid]),
                                               cls[selected][valid].argmax(axis=1))))
        if not candidates:
            return np.empty((0, 6), np.float32)
        dets = np.concatenate(candidates)
        kept = []
        for category in np.unique(dets[:, 5]):
            ids = np.flatnonzero(dets[:, 5] == category)
            boxes = dets[ids, :4].copy()
            boxes[:, 2:] -= boxes[:, :2]
            # Native OpenCV NMS avoids Python's per-box IoU loop; retain all candidates.
            indices = cv2.dnn.NMSBoxes(boxes.tolist(), dets[ids, 4].tolist(), 0.0, self.nms)
            if len(indices):
                kept.extend(ids[np.asarray(indices).reshape(-1)].tolist())
        return dets[kept]

    def infer(self, image):
        from run_yolo_x5 import letterbox, bgr_to_nv12
        started = time.monotonic()
        canvas, scale, pad_x, pad_y = letterbox(image, 640, 640)
        tensor = bgr_to_nv12(canvas)
        return self._infer_prepared(tensor, image.shape[1], image.shape[0], scale, pad_x, pad_y, started)

    def infer_nv12(self, image, source_width, source_height, scaler):
        from run_yolo_x5 import nv12_letterbox
        started = time.monotonic()
        tensor, pad_x, pad_y = nv12_letterbox(scaler.scale(image), scaler.output_width,
                                             scaler.output_height, 640, 640)
        scale = min(640 / source_width, 640 / source_height)
        return self._infer_prepared(tensor, source_width, source_height, scale, pad_x, pad_y, started)

    def _infer_prepared(self, tensor, width, height, scale, pad_x, pad_y, started):
        from run_yolo_x5 import YoloDetection, LABEL_ALIASES
        preprocessed = time.monotonic()
        outputs = self.run(tensor)
        inferred = time.monotonic()
        predictions = self.decode(outputs)
        # Restore native camera coordinates before tracking, ROI or range lookup.
        detections = []
        if len(predictions):
            coordinates = predictions[:, :4].copy()
            coordinates[:, [0,2]] = np.clip((coordinates[:, [0,2]]-pad_x)/scale, 0, width-1)
            coordinates[:, [1,3]] = np.clip((coordinates[:, [1,3]]-pad_y)/scale, 0, height-1)
            coordinates = np.rint(coordinates).astype(np.int32)
            for (x1,y1,x2,y2), (_,_,_,_,confidence,category) in zip(coordinates, predictions):
                if x2 <= x1 or y2 <= y1:
                    continue
                category = int(category)
                raw_name = self.labels[category]
                detections.append(YoloDetection(category, raw_name, LABEL_ALIASES.get(raw_name, raw_name),
                    float(confidence), (int(x1),int(y1),int(x2-x1),int(y2-y1))))
        finished = time.monotonic()
        return detections, {'preprocess_ms':(preprocessed-started)*1000,
                            'inference_ms':(inferred-preprocessed)*1000,
                            'postprocess_ms':(finished-inferred)*1000,
                            'total_ms':(finished-started)*1000}

