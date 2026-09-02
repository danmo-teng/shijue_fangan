from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def _bounds(channel: np.ndarray, low_percentile: float, high_percentile: float, margin: int, maximum: int) -> tuple[int, int]:
    low = int(np.percentile(channel, low_percentile)) - margin
    high = int(np.percentile(channel, high_percentile)) + margin
    return max(0, low), min(maximum, high)


def auto_sample_profile(
    frame: np.ndarray,
    rectangle: tuple[int, int, int, int],
    profile: dict[str, Any],
    reference_resolution: tuple[int, int] | list[int] | None = None,
) -> dict[str, Any]:
    """Estimate color and shape ranges from a user-drawn target rectangle."""
    x0, y0, x1, y1 = rectangle
    x0, x1 = sorted((max(0, x0), min(frame.shape[1], x1)))
    y0, y1 = sorted((max(0, y0), min(frame.shape[0], y1)))
    if x1 - x0 < 8 or y1 - y0 < 8:
        raise ValueError("框选区域过小，宽高至少8像素")

    # Shrink the box slightly so its edge/background is not used as a color
    # sample. The full rectangle is retained for approximate geometry.
    inset_x = max(1, int((x1 - x0) * 0.08))
    inset_y = max(1, int((y1 - y0) * 0.08))
    crop = frame[y0 + inset_y : y1 - inset_y, x0 + inset_x : x1 - inset_x]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB).reshape(-1, 3)

    kind = profile.get("kind", "material")
    if kind == "core_black":
        # Hue is meaningless for near-black pixels.
        hsv_bounds = [0, 0, 0, 179, 255, min(255, int(np.percentile(hsv[:, 2], 97)) + 12)]
    else:
        h_low, h_high = _bounds(hsv[:, 0], 3, 97, 3, 179)
        s_low, s_high = _bounds(hsv[:, 1], 3, 97, 12, 255)
        v_low, v_high = _bounds(hsv[:, 2], 2, 98, 15, 255)
        hsv_bounds = [h_low, s_low, v_low, h_high, s_high, v_high]
    lab_bounds = []
    for index in range(3):
        low, high = _bounds(lab[:, index], 2, 98, 10, 255)
        lab_bounds.append(low)
    for index in range(3):
        low, high = _bounds(lab[:, index], 2, 98, 10, 255)
        lab_bounds.append(high)

    profile["hsv"] = hsv_bounds
    profile["lab"] = lab_bounds

    # Estimate contour features using color distance from the rectangle border.
    full_crop = frame[y0:y1, x0:x1]
    full_hsv = cv2.cvtColor(full_crop, cv2.COLOR_BGR2HSV)
    low = np.array(hsv_bounds[:3], np.uint8)
    high = np.array(hsv_bounds[3:], np.uint8)
    mask = cv2.inRange(full_hsv, low, high)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea) if contours else None
    rules = profile["candidate"]
    estimated: dict[str, float] = {}
    if contour is not None and cv2.contourArea(contour) >= 20:
        area = float(cv2.contourArea(contour))
        if reference_resolution is None:
            reference_resolution = (frame.shape[1], frame.shape[0])
        reference_width, reference_height = reference_resolution
        area_scale = (frame.shape[1] * frame.shape[0]) / max(reference_width * reference_height, 1)
        reference_area = area / max(area_scale, 1e-6)
        rect = cv2.minAreaRect(contour)
        rw, rh = rect[1]
        aspect = max(rw, rh) / max(min(rw, rh), 1.0)
        extent = area / max(rw * rh, 1.0)
        hull_area = max(cv2.contourArea(cv2.convexHull(contour)), 1.0)
        solidity = area / hull_area
        perimeter = cv2.arcLength(contour, True)
        vertices = len(cv2.approxPolyDP(contour, 0.04 * perimeter, True))
        rules["area_px"] = [max(10, int(reference_area * 0.35)), int(reference_area * 2.8)]
        rules["aspect"] = [max(1.0, aspect * 0.55), max(1.3, aspect * 1.8)]
        rules["extent_min"] = max(0.1, extent * 0.55)
        rules["solidity_min"] = max(0.2, solidity * 0.65)
        if kind == "core_black":
            rules["vertices"] = [max(3, vertices - 2), min(12, vertices + 3)]
        estimated = {
            "area_px": reference_area,
            "area_px_actual": area,
            "area_px_reference": reference_area,
            "aspect": aspect,
            "extent": extent,
            "solidity": solidity,
            "vertices": vertices,
        }

    return {
        "rectangle": [x0, y0, x1, y1],
        "hsv": hsv_bounds,
        "lab": lab_bounds,
        "estimated_shape": estimated,
        "note": "自动范围来自框内中间84%像素；请换距离和光照复核后再保存。",
    }


def diagnose_frame(
    frame: np.ndarray,
    exposure: int | None = None,
    max_exposure: int = 20,
) -> dict[str, Any]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, (320, 240), interpolation=cv2.INTER_AREA)
    mean = float(small.mean())
    hsv_small = cv2.cvtColor(cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2HSV)
    median_saturation = float(np.median(hsv_small[:, :, 1]))
    median_value = float(np.median(hsv_small[:, :, 2]))
    p05, p95 = (float(value) for value in np.percentile(small, [5, 95]))
    dark = float(np.mean(small <= 8))
    bright = float(np.mean(small >= 247))
    sharpness = float(cv2.Laplacian(small, cv2.CV_32F).var())
    center = frame[frame.shape[0] // 4 : frame.shape[0] * 3 // 4, frame.shape[1] // 4 : frame.shape[1] * 3 // 4]
    mean_bgr = tuple(float(value) for value in cv2.mean(center)[:3])
    advice: list[str] = []
    suggested = exposure
    if exposure is not None and median_value > 1:
        # Aim for a well-separated color signal without driving highlights
        # into clipping. The caller derives the ceiling from frame interval.
        suggested = int(np.clip(round(exposure * 165.0 / median_value), 4, max_exposure))
    if mean < 55 or p95 < 120:
        advice.append("画面偏暗：先增强无频闪直流补光，再小幅增加曝光或增益。")
        if exposure is not None:
            suggested = min(max_exposure, max(exposure + 1, suggested))
    elif mean > 195 or bright > 0.06:
        advice.append("画面偏亮/局部过曝：降低曝光或补光亮度。")
        if exposure is not None:
            suggested = max(4, min(exposure - 1, suggested))
    else:
        advice.append("整体亮度处于可用范围。")
    if dark > 0.20:
        advice.append("暗部剪切较多，黑色核心物资会与阴影混淆。")
    if sharpness < 45:
        advice.append("清晰度偏低：检查固定焦距；运动中模糊则缩短曝光并增强补光。")
    else:
        advice.append("静态清晰度正常。")
    advice.append("白平衡应固定；把白/灰参考物放入现场光照下，调到R/G/B均值接近后锁定。")
    if max(mean_bgr) / max(min(mean_bgr), 1.0) > 1.18:
        advice.append("中央区域存在明显色偏；若中央放的是白/灰参考物，请微调白平衡温度直到B/G/R更接近。")
    return {
        "mean_luma": round(mean, 2), "p05": round(p05, 2), "p95": round(p95, 2),
        "median_hsv_s": round(median_saturation, 2), "median_hsv_v": round(median_value, 2),
        "dark_ratio": round(dark, 4), "bright_ratio": round(bright, 4),
        "sharpness": round(sharpness, 2), "current_exposure": exposure,
        "center_mean_bgr": [round(value, 2) for value in mean_bgr],
        "suggested_exposure": suggested, "advice": advice,
    }
