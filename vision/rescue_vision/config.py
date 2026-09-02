from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any


NATIVE_WIDTH = 1280
NATIVE_HEIGHT = 1024


def require_native_resolution(width: int, height: int) -> None:
    if (int(width), int(height)) != (NATIVE_WIDTH, NATIVE_HEIGHT):
        raise ValueError(
            f"本项目的采集、处理和UART坐标固定为{NATIVE_WIDTH}x{NATIVE_HEIGHT}，"
            f"不能使用{width}x{height}"
        )


def load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    validate_config(data)
    return data


def save_config(path: str | Path, config: dict[str, Any]) -> None:
    validate_config(config)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(config, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    temporary.replace(target)


def validate_config(config: dict[str, Any]) -> None:
    camera = config.get("camera", {})
    for key, fallback in (("width", NATIVE_WIDTH), ("height", NATIVE_HEIGHT), ("fps", 180)):
        if int(camera.get(key, fallback)) <= 0:
            raise ValueError(f"camera.{key}必须大于0")
    reference = config.get("threshold_reference_resolution")
    expected_reference = [int(camera.get("width", NATIVE_WIDTH)), int(camera.get("height", NATIVE_HEIGHT))]
    require_native_resolution(*expected_reference)
    if reference != expected_reference:
        raise ValueError(
            f"threshold_reference_resolution必须与原生处理分辨率一致：{expected_reference}"
        )
    performance = config.get("performance", {})
    if bool(performance.get("two_stage", False)):
        raise ValueError("本项目固定使用1280x1024原图处理，performance.two_stage必须为false")
    runtime = config.get("runtime", {})
    rate_limits = {
        "danger_fps": (60, 120),
        "material_fps": (60, 90),
        "zone_fps": (20, 30),
    }
    for key, (low, high) in rate_limits.items():
        value = float(runtime.get(key, high))
        if not low <= value <= high:
            raise ValueError(f"runtime.{key}必须在{low}..{high}之间")
    if not isinstance(config.get("classes"), dict) or not config["classes"]:
        raise ValueError("配置必须包含非空classes对象")
    for name, profile in config["classes"].items():
        if not isinstance(profile.get("references", []), list):
            raise ValueError(f"{name}.references必须是数组")
        variants = [("基础参考", profile)] + [
            (str(item.get("reference_name", f"参考{index}")), item)
            for index, item in enumerate(profile.get("references", []), start=1)
        ]
        for reference_name, variant in variants:
            for space in ("hsv", "lab"):
                values = variant.get(space)
                if not isinstance(values, list) or len(values) != 6:
                    raise ValueError(f"{name}.{reference_name}.{space}必须是6个整数")
            morph = variant.get("morphology", {})
            for key in ("open", "close"):
                value = int(morph.get(key, 0))
                if value < 0 or value > 31:
                    raise ValueError(f"{name}.{reference_name}.morphology.{key}超出0..31")


def clone_config(config: dict[str, Any]) -> dict[str, Any]:
    return copy.deepcopy(config)
