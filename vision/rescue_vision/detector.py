from __future__ import annotations

import math
import copy
from typing import Any

import cv2
import numpy as np

from .localizer import GroundLocalizer
from .models import Candidate, Detection
from .segmenter import Segmenter


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def range_score(value: float, low: float, high: float, softness: float = 0.25) -> float:
    if low <= value <= high:
        return 1.0
    span = max(high - low, 1.0)
    distance = low - value if value < low else value - high
    return clamp01(1.0 - distance / (span * softness + 1e-6))


class TraditionalDetector:
    def __init__(self, config: dict[str, Any], localizer: GroundLocalizer) -> None:
        self.config = config
        self.segmenter = Segmenter(config)
        self.localizer = localizer

    def update_config(self, config: dict[str, Any]) -> None:
        self.config = config
        self.segmenter.update_config(config)

    @staticmethod
    def _bottom_point(contour: np.ndarray) -> tuple[float, float]:
        points = contour[:, 0, :]
        max_y = int(points[:, 1].max())
        band = points[points[:, 1] >= max_y - 3]
        return float(band[:, 0].mean()), float(band[:, 1].mean())

    @staticmethod
    def _masked_mean(image: np.ndarray, contour: np.ndarray, bbox: tuple[int, int, int, int]) -> tuple[float, float, float]:
        x, y, width, height = bbox
        local = image[y : y + height, x : x + width]
        local_mask = np.zeros((height, width), dtype=np.uint8)
        shifted = contour.copy()
        shifted[:, 0, 0] -= x
        shifted[:, 0, 1] -= y
        cv2.drawContours(local_mask, [shifted], -1, 255, -1)
        mean = cv2.mean(local, mask=local_mask)
        return float(mean[0]), float(mean[1]), float(mean[2])

    @staticmethod
    def _contrast(gray: np.ndarray, contour: np.ndarray, bbox: tuple[int, int, int, int]) -> float:
        x, y, width, height = bbox
        margin = max(5, int(max(width, height) * 0.25))
        x0, y0 = max(0, x - margin), max(0, y - margin)
        x1, y1 = min(gray.shape[1], x + width + margin), min(gray.shape[0], y + height + margin)
        region = gray[y0:y1, x0:x1]
        inside = np.zeros(region.shape, dtype=np.uint8)
        shifted = contour.copy()
        shifted[:, 0, 0] -= x0
        shifted[:, 0, 1] -= y0
        cv2.drawContours(inside, [shifted], -1, 255, -1)
        outside = cv2.bitwise_not(inside)
        inside_mean = cv2.mean(region, mask=inside)[0]
        outside_mean = cv2.mean(region, mask=outside)[0]
        return float(outside_mean - inside_mean)

    def _candidate(
        self,
        class_name: str,
        profile: dict[str, Any],
        area_scale: float,
        linear_scale: float,
        contour: np.ndarray,
        class_mask: np.ndarray,
        hsv: np.ndarray,
        lab: np.ndarray,
        gray: np.ndarray,
        coordinate_offset: tuple[int, int] = (0, 0),
        image_shape: tuple[int, int] | None = None,
        reference_index: int = 0,
        reference_name: str = "base",
    ) -> Candidate | None:
        rules = profile["candidate"]
        use_shape = bool(rules.get("use_shape", True))
        use_size = bool(rules.get("use_size", True))
        area = float(cv2.contourArea(contour))
        reference_area = area / max(area_scale, 1e-6)
        if area <= 0:
            return None
        local_x, local_y, width, height = cv2.boundingRect(contour)
        offset_x, offset_y = coordinate_offset
        x, y = local_x + offset_x, local_y + offset_y
        rect = cv2.minAreaRect(contour)
        rw, rh = rect[1]
        long_side, short_side = max(rw, rh), max(min(rw, rh), 1e-6)
        aspect = long_side / short_side
        extent = area / max(rw * rh, 1.0)
        hull = cv2.convexHull(contour)
        solidity = area / max(cv2.contourArea(hull), 1.0)
        perimeter = float(cv2.arcLength(contour, True))
        circularity = 4.0 * math.pi * area / max(perimeter * perimeter, 1.0)
        epsilon = float(profile.get("polygon_epsilon", 0.035)) * perimeter
        vertices = len(cv2.approxPolyDP(contour, epsilon, True))
        local_bbox = (local_x, local_y, width, height)
        mean_hsv = self._masked_mean(hsv, contour, local_bbox)
        mean_lab = self._masked_mean(lab, contour, local_bbox)
        color_fill = float(cv2.countNonZero(class_mask[local_y : local_y + height, local_x : local_x + width])) / max(width * height, 1)
        contrast = self._contrast(gray, contour, local_bbox) if profile.get("kind") == "core_black" else 0.0
        local_bottom = self._bottom_point(contour)
        bottom = (local_bottom[0] + offset_x, local_bottom[1] + offset_y)
        ground = self.localizer.image_to_ground(bottom)
        global_center = (rect[0][0] + offset_x, rect[0][1] + offset_y)
        size_mm = self.localizer.image_segment_size_mm(global_center, rw, rh)

        rejected: list[str] = []
        if not rules["area_px"][0] <= reference_area <= rules["area_px"][1]:
            rejected.append("像素面积")
        if use_shape and not rules["aspect"][0] <= aspect <= rules["aspect"][1]:
            rejected.append("长宽比")
        if use_shape and extent < rules.get("extent_min", 0.0):
            rejected.append("填充率")
        if use_shape and solidity < rules.get("solidity_min", 0.0):
            rejected.append("实心度")
        if color_fill < rules.get("color_fill_min", 0.0):
            rejected.append("颜色占比")
        margin = int(round(float(rules.get("border_margin", 0)) * linear_scale))
        full_height, full_width = image_shape or gray.shape[:2]
        if x <= margin or y <= margin or x + width >= full_width - margin or y + height >= full_height - margin:
            rejected.append("接触图像边界")

        size_score = 1.0
        if use_size and size_mm is not None and "size_mm" in rules:
            measured = sorted(size_mm)
            expected = sorted(rules["size_mm"])
            size_score = 0.5 * range_score(measured[0], expected[0][0], expected[0][1]) + 0.5 * range_score(measured[1], expected[1][0], expected[1][1])
            if size_score <= 0.05:
                rejected.append("实际尺寸")

        shape_score = (
            0.4 * range_score(aspect, rules["aspect"][0], rules["aspect"][1])
            + 0.3 * clamp01(extent / max(rules.get("extent_target", 0.75), 0.1))
            + 0.3 * clamp01(solidity / max(rules.get("solidity_target", 0.9), 0.1))
        )
        if profile.get("kind") == "core_black":
            contrast_score = range_score(contrast, rules.get("contrast_min", 12.0), rules.get("contrast_max", 120.0))
            vertex_score = 1.0 if rules.get("vertices", [3, 8])[0] <= vertices <= rules.get("vertices", [3, 8])[1] else 0.4
            shape_score = 0.45 * shape_score + 0.35 * contrast_score + 0.20 * vertex_score

        color_score = clamp01(color_fill / max(rules.get("color_fill_target", 0.55), 0.05))
        weights = profile.get("weights", {})
        color_weight = float(weights.get("color", 0.55))
        shape_weight = float(weights.get("shape", 0.25)) if use_shape else 0.0
        size_weight = float(weights.get("size", 0.20)) if use_size and size_mm is not None else 0.0
        total_weight = max(color_weight + shape_weight + size_weight, 1e-6)
        score = clamp01((color_weight * color_score + shape_weight * shape_score + size_weight * size_score) / total_weight)

        global_contour = contour.copy()
        global_contour[:, 0, 0] += offset_x
        global_contour[:, 0, 1] += offset_y
        rotated_box = cv2.boxPoints(rect).astype(np.int32)
        rotated_box[:, 0] += offset_x
        rotated_box[:, 1] += offset_y
        return Candidate(
            class_name=class_name,
            contour=global_contour,
            bbox=(x, y, width, height),
            rotated_box=rotated_box,
            bottom_point=bottom,
            area_px=area,
            aspect=aspect,
            extent=extent,
            solidity=solidity,
            circularity=circularity,
            vertices=vertices,
            mean_hsv=mean_hsv,
            mean_lab=mean_lab,
            color_fill=color_fill,
            contrast=contrast,
            score=score,
            reject_reason="、".join(rejected),
            ground_xy_mm=ground,
            size_mm=size_mm,
            reference_index=reference_index,
            reference_name=reference_name,
        )

    @staticmethod
    def _iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        x0, y0 = max(ax, bx), max(ay, by)
        x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
        intersection = max(0, x1 - x0) * max(0, y1 - y0)
        union = aw * ah + bw * bh - intersection
        return intersection / max(union, 1)

    def _detect_native(
        self,
        frame: np.ndarray,
        class_names: list[str] | None = None,
        collect_rejected: bool = False,
        reference_index: int | None = None,
        collect_debug: bool = False,
        coordinate_offset: tuple[int, int] = (0, 0),
        image_shape: tuple[int, int] | None = None,
        scale_resolution: tuple[int, int] | None = None,
        apply_roi: bool = True,
    ) -> tuple[list[Detection], dict[str, Any]]:
        hsv, lab = self.segmenter.color_spaces(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        reference_width, reference_height = self.config.get(
            "threshold_reference_resolution", [frame.shape[1], frame.shape[0]]
        )
        scale_width, scale_height = scale_resolution or (frame.shape[1], frame.shape[0])
        area_scale = (scale_width * scale_height) / max(reference_width * reference_height, 1)
        linear_scale = math.sqrt(area_scale)
        full_height, full_width = image_shape or frame.shape[:2]
        offset_x, offset_y = coordinate_offset
        accepted: list[Candidate] = []
        rejected: list[Candidate] = []
        quick_rejected = 0
        masks: dict[str, dict[str, np.ndarray]] = {}
        enabled_names = set(class_names) if class_names is not None else None
        for class_name, profile in self.config["classes"].items():
            if enabled_names is not None and class_name not in enabled_names:
                continue
            if not profile.get("enabled", True):
                continue
            base_profile = {key: copy.deepcopy(value) for key, value in profile.items() if key != "references"}
            reference_profiles = [("基础参考", base_profile)]
            for index, reference in enumerate(profile.get("references", []), start=1):
                variant = copy.deepcopy(base_profile)
                for key, value in reference.items():
                    if key != "reference_name":
                        variant[key] = copy.deepcopy(value)
                reference_profiles.append((reference.get("reference_name", f"参考{index}"), variant))
            if reference_index is not None:
                if not 0 <= reference_index < len(reference_profiles):
                    raise ValueError(f"{class_name}不存在参考阈值{reference_index}")
                reference_profiles = [reference_profiles[reference_index]]
            combined_stages: dict[str, np.ndarray] = {}
            for current_reference_index, (reference_name, variant) in enumerate(reference_profiles):
                mask, stages = self.segmenter.segment_profile(
                    frame,
                    variant,
                    hsv,
                    lab,
                    apply_roi=apply_roi,
                    scale_resolution=(scale_width, scale_height),
                    collect_stages=collect_debug,
                )
                for stage_name, stage_mask in stages.items():
                    if stage_name not in combined_stages:
                        combined_stages[stage_name] = stage_mask.copy()
                    else:
                        combined_stages[stage_name] = cv2.bitwise_or(combined_stages[stage_name], stage_mask)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    # Reject obvious noise before masked color statistics,
                    # hulls, contrast and polygon approximation.
                    area = float(cv2.contourArea(contour))
                    reference_area = area / max(area_scale, 1e-6)
                    area_low, area_high = variant["candidate"]["area_px"]
                    if reference_area < area_low or reference_area > area_high:
                        quick_rejected += 1
                        continue
                    x, y, width, height = cv2.boundingRect(contour)
                    margin = int(round(float(variant["candidate"].get("border_margin", 0)) * linear_scale))
                    global_x, global_y = x + offset_x, y + offset_y
                    if global_x <= margin or global_y <= margin or global_x + width >= full_width - margin or global_y + height >= full_height - margin:
                        quick_rejected += 1
                        continue
                    candidate = self._candidate(
                        class_name, variant, area_scale, linear_scale,
                        contour, mask, hsv, lab, gray,
                        coordinate_offset=coordinate_offset,
                        image_shape=(full_height, full_width),
                        reference_index=(reference_index if reference_index is not None else current_reference_index),
                        reference_name=reference_name,
                    )
                    if candidate is None:
                        continue
                    threshold = float(variant.get("score_min", 0.65))
                    if not candidate.reject_reason and candidate.score >= threshold:
                        accepted.append(candidate)
                    elif collect_rejected:
                        rejected.append(candidate)
            if collect_debug:
                masks[class_name] = combined_stages

        # Cross-class NMS. Dangerous-object candidates win close ambiguity so
        # an uncertain cyan cube cannot be treated as ordinary cargo.
        accepted.sort(
            key=lambda item: item.score + (0.20 if item.class_name == "danger_cyan" else 0.0),
            reverse=True,
        )
        selected: list[Candidate] = []
        for candidate in accepted:
            if any(self._iou(candidate.bbox, other.bbox) > 0.45 for other in selected):
                continue
            selected.append(candidate)

        detections = [
            Detection(
                class_name=item.class_name,
                confidence=item.score,
                bbox=item.bbox,
                bottom_point=item.bottom_point,
                ground_xy_mm=item.ground_xy_mm,
                size_mm=item.size_mm,
                contour=item.contour,
                features={
                    "area_px": item.area_px,
                    "aspect": item.aspect,
                    "extent": item.extent,
                    "solidity": item.solidity,
                    "circularity": item.circularity,
                    "vertices": float(item.vertices),
                    "color_fill": item.color_fill,
                    "contrast": item.contrast,
                },
            )
            for item in selected
        ]
        valid_masks: dict[str, np.ndarray] = {}
        if collect_debug:
            valid_masks = {
                name: np.zeros((full_height, full_width), dtype=np.uint8)
                for name in (class_names or list(self.config["classes"]))
            }
            for item in selected:
                if item.class_name not in valid_masks:
                    valid_masks[item.class_name] = np.zeros((full_height, full_width), dtype=np.uint8)
                cv2.drawContours(valid_masks[item.class_name], [item.contour], -1, 255, -1)
        return detections, {
            "masks": masks,
            "valid_masks": valid_masks,
            "accepted": selected,
            "rejected": rejected,
            "quick_rejected_count": quick_rejected,
        }

    def _profile_variant(self, class_name: str, index: int) -> dict[str, Any]:
        profile = self.config["classes"][class_name]
        base = {key: copy.deepcopy(value) for key, value in profile.items() if key != "references"}
        if index == 0:
            return base
        references = profile.get("references", [])
        if not 0 <= index - 1 < len(references):
            return base
        for key, value in references[index - 1].items():
            if key != "reference_name":
                base[key] = copy.deepcopy(value)
        return base

    @staticmethod
    def _to_detections(candidates: list[Candidate]) -> list[Detection]:
        return [
            Detection(
                class_name=item.class_name,
                confidence=item.score,
                bbox=item.bbox,
                bottom_point=item.bottom_point,
                ground_xy_mm=item.ground_xy_mm,
                size_mm=item.size_mm,
                contour=item.contour,
                features={
                    "area_px": item.area_px,
                    "aspect": item.aspect,
                    "extent": item.extent,
                    "solidity": item.solidity,
                    "circularity": item.circularity,
                    "vertices": float(item.vertices),
                    "color_fill": item.color_fill,
                    "contrast": item.contrast,
                },
            )
            for item in candidates
        ]

    def detect(
        self,
        frame: np.ndarray,
        class_names: list[str] | None = None,
        collect_rejected: bool = False,
        reference_index: int | None = None,
        collect_debug: bool = False,
    ) -> tuple[list[Detection], dict[str, Any]]:
        """Two-stage detector: small full-frame proposal pass, then native ROI validation."""
        performance = self.config.get("performance", {})
        coarse_width = int(performance.get("coarse_width", 640))
        coarse_height = int(performance.get("coarse_height", 360))
        two_stage = bool(performance.get("two_stage", True))
        height, width = frame.shape[:2]
        if not two_stage or width <= coarse_width or height <= coarse_height:
            return self._detect_native(
                frame,
                class_names,
                collect_rejected,
                reference_index,
                collect_debug=collect_debug,
            )

        coarse = cv2.resize(frame, (coarse_width, coarse_height), interpolation=cv2.INTER_AREA)
        _, coarse_debug = self._detect_native(
            coarse,
            class_names,
            collect_rejected,
            reference_index,
            collect_debug=collect_debug,
        )
        sx, sy = width / coarse_width, height / coarse_height
        reference_width, reference_height = self.config.get("threshold_reference_resolution", [width, height])
        area_scale = (width * height) / max(reference_width * reference_height, 1)
        linear_scale = math.sqrt(area_scale)
        refined: list[Candidate] = []
        refined_rejected: list[Candidate] = []

        for proposal in coarse_debug["accepted"]:
            px, py, pw, ph = proposal.bbox
            mapped_x, mapped_y = int(px * sx), int(py * sy)
            mapped_w, mapped_h = max(1, int(math.ceil(pw * sx))), max(1, int(math.ceil(ph * sy)))
            padding = max(int(performance.get("refine_padding_px", 12)), int(max(mapped_w, mapped_h) * 0.18))
            x0, y0 = max(0, mapped_x - padding), max(0, mapped_y - padding)
            x1, y1 = min(width, mapped_x + mapped_w + padding), min(height, mapped_y + mapped_h + padding)
            crop = frame[y0:y1, x0:x1]
            if crop.size == 0:
                continue
            variant = self._profile_variant(proposal.class_name, proposal.reference_index)
            hsv, lab = self.segmenter.color_spaces(crop)
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            mask, _ = self.segmenter.segment_profile(
                crop,
                variant,
                hsv,
                lab,
                apply_roi=False,
                scale_resolution=(width, height),
                collect_stages=False,
            )
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            expected_center = (mapped_x + mapped_w * 0.5 - x0, mapped_y + mapped_h * 0.5 - y0)
            contours = sorted(
                contours,
                key=lambda contour: (
                    (cv2.boundingRect(contour)[0] + cv2.boundingRect(contour)[2] * 0.5 - expected_center[0]) ** 2
                    + (cv2.boundingRect(contour)[1] + cv2.boundingRect(contour)[3] * 0.5 - expected_center[1]) ** 2
                ),
            )
            for contour in contours:
                area = float(cv2.contourArea(contour))
                area_low, area_high = variant["candidate"]["area_px"]
                if not area_low <= area / max(area_scale, 1e-6) <= area_high:
                    continue
                candidate = self._candidate(
                    proposal.class_name,
                    variant,
                    area_scale,
                    linear_scale,
                    contour,
                    mask,
                    hsv,
                    lab,
                    gray,
                    coordinate_offset=(x0, y0),
                    image_shape=(height, width),
                    reference_index=proposal.reference_index,
                    reference_name=proposal.reference_name,
                )
                if candidate is None:
                    continue
                threshold = float(variant.get("score_min", 0.65))
                if not candidate.reject_reason and candidate.score >= threshold:
                    refined.append(candidate)
                    break
                if collect_rejected:
                    refined_rejected.append(candidate)

        refined.sort(key=lambda item: item.score + (0.20 if item.class_name == "danger_cyan" else 0.0), reverse=True)
        selected: list[Candidate] = []
        for candidate in refined:
            if not any(self._iou(candidate.bbox, other.bbox) > 0.45 for other in selected):
                selected.append(candidate)

        masks: dict[str, dict[str, np.ndarray]] = {}
        valid_masks: dict[str, np.ndarray] = {}
        if collect_debug:
            for name, stages in coarse_debug["masks"].items():
                masks[name] = {
                    stage_name: cv2.resize(stage, (width, height), interpolation=cv2.INTER_NEAREST)
                    for stage_name, stage in stages.items()
                }
            valid_masks = {
                name: np.zeros((height, width), dtype=np.uint8)
                for name in (class_names or list(self.config["classes"]))
            }
            for item in selected:
                valid_masks.setdefault(item.class_name, np.zeros((height, width), dtype=np.uint8))
                cv2.drawContours(valid_masks[item.class_name], [item.contour], -1, 255, -1)
        return self._to_detections(selected), {
            "masks": masks,
            "valid_masks": valid_masks,
            "accepted": selected,
            "rejected": refined_rejected,
            "quick_rejected_count": coarse_debug["quick_rejected_count"],
        }
