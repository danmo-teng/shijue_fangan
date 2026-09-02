from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .config import load_config, save_config
from .models import Detection, Track, TrackState


TRACKBARS = [
    ("H min", "hsv", 0, 179), ("S min", "hsv", 1, 255), ("V min", "hsv", 2, 255),
    ("H max", "hsv", 3, 179), ("S max", "hsv", 4, 255), ("V max", "hsv", 5, 255),
    ("L min", "lab", 0, 255), ("A min", "lab", 1, 255), ("B min", "lab", 2, 255),
    ("L max", "lab", 3, 255), ("A max", "lab", 4, 255), ("B max", "lab", 5, 255),
]


class ThresholdEditor:
    def __init__(self, config: dict[str, Any], config_path: str, capture_dir: str) -> None:
        self.config = config
        self.config_path = config_path
        self.capture_dir = Path(capture_dir)
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.classes = list(config["classes"])
        self.class_index = 0
        self.frozen = False
        self.frozen_frame: np.ndarray | None = None
        self.mask_stage = "final"
        self.last_mouse_text = "点击原图读取 BGR / HSV / Lab"
        self.window = "Rescue Vision Editor"
        self.controls = "Threshold Controls"
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.controls, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 1280, 800)
        cv2.setMouseCallback(self.window, self._mouse)
        self._last_original: np.ndarray | None = None
        self._create_trackbars()
        self._sync_trackbars()

    @property
    def selected_class(self) -> str:
        return self.classes[self.class_index]

    def _create_trackbars(self) -> None:
        noop = lambda _value: None
        for label, _, _, maximum in TRACKBARS:
            cv2.createTrackbar(label, self.controls, 0, maximum, noop)
        cv2.createTrackbar("Open", self.controls, 0, 31, noop)
        cv2.createTrackbar("Close", self.controls, 0, 31, noop)
        cv2.createTrackbar("Area min", self.controls, 0, 20000, noop)
        cv2.createTrackbar("Area max / 10", self.controls, 1, 50000, noop)
        cv2.createTrackbar("Score %", self.controls, 0, 100, noop)

    def _sync_trackbars(self) -> None:
        profile = self.config["classes"][self.selected_class]
        for label, space, index, _ in TRACKBARS:
            cv2.setTrackbarPos(label, self.controls, int(profile[space][index]))
        cv2.setTrackbarPos("Open", self.controls, int(profile["morphology"]["open"]))
        cv2.setTrackbarPos("Close", self.controls, int(profile["morphology"]["close"]))
        cv2.setTrackbarPos("Area min", self.controls, int(profile["candidate"]["area_px"][0]))
        cv2.setTrackbarPos("Area max / 10", self.controls, max(1, int(profile["candidate"]["area_px"][1] / 10)))
        cv2.setTrackbarPos("Score %", self.controls, int(float(profile["score_min"]) * 100))

    def pull_parameters(self) -> None:
        profile = self.config["classes"][self.selected_class]
        for label, space, index, _ in TRACKBARS:
            profile[space][index] = cv2.getTrackbarPos(label, self.controls)
        profile["morphology"]["open"] = cv2.getTrackbarPos("Open", self.controls)
        profile["morphology"]["close"] = cv2.getTrackbarPos("Close", self.controls)
        profile["candidate"]["area_px"][0] = cv2.getTrackbarPos("Area min", self.controls)
        profile["candidate"]["area_px"][1] = max(10, cv2.getTrackbarPos("Area max / 10", self.controls) * 10)
        profile["score_min"] = cv2.getTrackbarPos("Score %", self.controls) / 100.0

    def choose_frame(self, live: np.ndarray) -> np.ndarray:
        if self.frozen:
            if self.frozen_frame is None:
                self.frozen_frame = live.copy()
            return self.frozen_frame
        self.frozen_frame = None
        return live

    def _mouse(self, event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self._last_original is None:
            return
        display_height, display_width = 360, 480
        if not (0 <= x < display_width and 0 <= y < display_height):
            return
        image = self._last_original
        u = min(image.shape[1] - 1, int(x * image.shape[1] / display_width))
        v = min(image.shape[0] - 1, int(y * image.shape[0] / display_height))
        pixel = image[v : v + 1, u : u + 1]
        bgr = tuple(int(value) for value in pixel[0, 0])
        hsv = tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0, 0])
        lab = tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_BGR2LAB)[0, 0])
        self.last_mouse_text = f"({u},{v}) BGR={bgr} HSV={hsv} Lab={lab}"

    @staticmethod
    def _label(frame: np.ndarray, text: str, line: int, color=(255, 255, 255)) -> None:
        cv2.putText(frame, text, (10, 24 + line * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

    def render(
        self,
        frame: np.ndarray,
        masks: dict[str, dict[str, np.ndarray]],
        detections: list[Detection],
        tracks: list[Track],
        rejected: list[Any],
        metrics: dict[str, float],
    ) -> int:
        self._last_original = frame
        annotated = frame.copy()
        selected = self.selected_class
        for detection in detections:
            x, y, width, height = detection.bbox
            color = tuple(self.config["classes"][detection.class_name].get("display_bgr", [0, 255, 0]))
            cv2.rectangle(annotated, (x, y), (x + width, y + height), color, 2)
            position = "px"
            if detection.ground_xy_mm is not None:
                position = f"{detection.ground_xy_mm[0]:.0f},{detection.ground_xy_mm[1]:.0f}mm"
            cv2.putText(annotated, f"{detection.class_name} {detection.confidence:.2f} {position}", (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv2.LINE_AA)
        for track in tracks:
            if track.state == TrackState.CONFIRMED:
                x, y, _, _ = track.last_detection.bbox
                cv2.putText(annotated, f"ID{track.track_id} CONF", (x, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        for candidate in rejected:
            if candidate.class_name != selected:
                continue
            x, y, width, height = candidate.bbox
            cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 0, 160), 1)
            cv2.putText(annotated, candidate.reject_reason or f"score {candidate.score:.2f}", (x, y + height + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 220), 1)

        stages = masks.get(selected, {})
        mask = stages.get(self.mask_stage)
        if mask is None:
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        isolated = cv2.bitwise_and(frame, frame, mask=mask)
        info = np.zeros_like(frame)
        profile = self.config["classes"][selected]
        lines = [
            f"Class: {selected}  fusion={profile['fusion']}  mask={self.mask_stage}",
            f"decode={metrics.get('decode_fps', 0):.1f} vision={metrics.get('vision_fps', 0):.1f} GUI={metrics.get('gui_fps', 0):.1f}",
            f"vision cost={metrics.get('vision_ms', 0):.2f}ms frame age={metrics.get('frame_age_ms', 0):.2f}ms",
            f"detections={len(detections)} confirmed={sum(t.state == TrackState.CONFIRMED for t in tracks)}",
            self.last_mouse_text,
            "Keys: [ ] class | 1 HSV 2 Lab 3 Final | F fusion | SPACE freeze",
            "S save | L reload | C capture | Q quit",
        ]
        for index, text in enumerate(lines):
            self._label(info, text, index, (0, 255, 255) if index == 0 else (230, 230, 230))
        roi = self.config.get("roi_polygon", [])
        if len(roi) >= 3:
            cv2.polylines(annotated, [np.asarray(roi, np.int32)], True, (255, 255, 0), 1)

        panels = [cv2.resize(item, (480, 360), interpolation=cv2.INTER_AREA) for item in (annotated, mask_bgr, isolated, info)]
        canvas = np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))
        cv2.imshow(self.window, canvas)
        return cv2.waitKey(1) & 0xFF

    def handle_key(self, key: int, frame: np.ndarray, annotated: np.ndarray | None = None) -> bool:
        if key in (ord("q"), 27):
            return False
        if key == ord("["):
            self.pull_parameters()
            self.class_index = (self.class_index - 1) % len(self.classes)
            self._sync_trackbars()
        elif key == ord("]"):
            self.pull_parameters()
            self.class_index = (self.class_index + 1) % len(self.classes)
            self._sync_trackbars()
        elif key == ord("1"):
            self.mask_stage = "hsv"
        elif key == ord("2"):
            self.mask_stage = "lab"
        elif key == ord("3"):
            self.mask_stage = "final"
        elif key == ord("f"):
            profile = self.config["classes"][self.selected_class]
            modes = ["and", "or", "hsv", "lab"]
            profile["fusion"] = modes[(modes.index(profile.get("fusion", "and")) + 1) % len(modes)]
        elif key == ord(" "):
            self.frozen = not self.frozen
            if not self.frozen:
                self.frozen_frame = None
        elif key == ord("s"):
            self.pull_parameters()
            save_config(self.config_path, self.config)
            print(f"配置已保存：{self.config_path}")
        elif key == ord("l"):
            loaded = load_config(self.config_path)
            self.config.clear()
            self.config.update(loaded)
            self.classes = list(self.config["classes"])
            self.class_index = min(self.class_index, len(self.classes) - 1)
            self._sync_trackbars()
            print(f"配置已重新加载：{self.config_path}")
        elif key == ord("c"):
            stamp = time.strftime("%Y%m%d_%H%M%S")
            path = self.capture_dir / f"{stamp}_{self.selected_class}.png"
            cv2.imwrite(str(path), frame)
            print(f"已保存现场图片：{path}")
        return True

    def close(self) -> None:
        cv2.destroyAllWindows()
