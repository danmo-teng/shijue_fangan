#!/usr/bin/env python3
"""evdev discovery, normalization, and the hand-control mapping."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

try:
    from .protocol import (
        BUTTON_A,
        BUTTON_B,
        BUTTON_CLAW_CLOSE,
        BUTTON_CLAW_OPEN,
        BUTTON_LIFT_DOWN,
        BUTTON_LIFT_UP,
        BUTTON_X,
        BUTTON_Y,
    )
except ImportError:  # Running directly from gamepad_control.py.
    from protocol import (
        BUTTON_A,
        BUTTON_B,
        BUTTON_CLAW_CLOSE,
        BUTTON_CLAW_OPEN,
        BUTTON_LIFT_DOWN,
        BUTTON_LIFT_UP,
        BUTTON_X,
        BUTTON_Y,
    )


@dataclass
class PadState:
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    hat_x: int = 0
    hat_y: int = 0
    rb: bool = False
    lb: bool = False
    start: bool = False
    select: bool = False
    m1: bool = False
    m2: bool = False
    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False


def shaped_axis(value: float, deadzone: float, exponent: float = 1.45) -> float:
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    scaled = min(1.0, (magnitude - deadzone) / (1.0 - deadzone))
    result = scaled**exponent
    return -result if value < 0.0 else result


def shaped_stick(
    x: float, y: float, deadzone: float, exponent: float = 1.35
) -> tuple[float, float]:
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0
    bounded = min(1.0, magnitude)
    output = ((bounded - deadzone) / (1.0 - deadzone)) ** exponent
    return x / magnitude * output, y / magnitude * output


def command_from_pad(
    state: PadState, deadzone: float, precision_percent: int
) -> tuple[int, int, int, int, int, int, bool]:
    """Return forward, left, yaw, camera, buttons, speed, RB-enabled.

    The signs match the F407 Gamepad firmware: forward and left are body-frame
    commands, and positive yaw is counter-clockwise.
    """

    right_x, right_y = shaped_stick(state.right_x, state.right_y, deadzone)
    forward = round(-right_y * 100.0)
    left = round(-right_x * 100.0)
    yaw = round(-shaped_axis(state.left_x, deadzone) * 100.0)
    camera = round(shaped_axis(state.left_y, deadzone) * 100.0)

    buttons = 0
    if state.hat_x < 0:
        buttons |= BUTTON_CLAW_CLOSE
    elif state.hat_x > 0:
        buttons |= BUTTON_CLAW_OPEN
    if state.hat_y < 0:
        buttons |= BUTTON_LIFT_UP
    elif state.hat_y > 0:
        buttons |= BUTTON_LIFT_DOWN
    if state.a:
        buttons |= BUTTON_A
    if state.b:
        buttons |= BUTTON_B
    if state.x:
        buttons |= BUTTON_X
    if state.y:
        buttons |= BUTTON_Y
    speed_percent = precision_percent if state.lb else 100
    return forward, left, yaw, camera, buttons, speed_percent, state.rb


class ArmGate:
    """Require a neutral, disarmed startup before honoring the deadman key."""

    def __init__(self, deadzone: float) -> None:
        self.deadzone = deadzone
        self.startup_safe = False

    def update(self, state: PadState) -> bool:
        neutral = (
            math.hypot(state.right_x, state.right_y) <= self.deadzone
            and abs(state.left_x) <= self.deadzone
            and abs(state.left_y) <= self.deadzone
        )
        if not self.startup_safe:
            if not state.rb and neutral:
                self.startup_safe = True
            return False
        return state.rb


class EvdevGamepad:
    """Select and read the first gamepad exposing both analog sticks."""

    def __init__(self, path: str | None = None) -> None:
        try:
            from evdev import InputDevice, ecodes, list_devices
        except ImportError as error:  # pragma: no cover - depends on target OS
            raise RuntimeError(
                "缺少 python3-evdev，请安装：sudo apt install python3-evdev"
            ) from error

        self.ecodes = ecodes
        paths = [path] if path else list_devices()
        required_axes = {ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY}
        candidates = []
        for candidate_path in paths:
            if not candidate_path:
                continue
            try:
                device = InputDevice(candidate_path)
                absolute = set(device.capabilities().get(ecodes.EV_ABS, []))
                keys = set(device.capabilities().get(ecodes.EV_KEY, []))
                face_keys = {
                    getattr(ecodes, "BTN_SOUTH", -1),
                    getattr(ecodes, "BTN_A", -1),
                }
                if required_axes.issubset(absolute) and keys.intersection(face_keys):
                    name = device.name.lower()
                    score = 10 if ("flydigi" in name or "vader" in name) else 0
                    score += 3 if "xbox" in name else 0
                    candidates.append((score, device))
                else:
                    device.close()
            except OSError:
                continue
        if not candidates:
            raise RuntimeError(
                "没有找到带左右摇杆的手柄，请检查接收器或使用 --device /dev/input/eventN"
            )
        candidates.sort(key=lambda item: item[0], reverse=True)
        self.device = candidates[0][1]
        for _, extra in candidates[1:]:
            extra.close()
        try:
            self.device.grab()
        except OSError as error:
            self.device.close()
            raise RuntimeError(f"无法独占手柄输入设备：{error}") from error
        os.set_blocking(self.device.fd, False)
        self.state = PadState()

        self.axis_info = {}
        for code in required_axes | {ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y}:
            try:
                self.axis_info[code] = self.device.absinfo(code)
            except OSError:
                pass
        self.has_hat_x = ecodes.ABS_HAT0X in self.axis_info
        self.has_hat_y = ecodes.ABS_HAT0Y in self.axis_info
        self._key_sets = {
            "rb": self._codes("BTN_TR"),
            "lb": self._codes("BTN_TL"),
            "start": self._codes("BTN_START"),
            "select": self._codes("BTN_SELECT"),
            "m1": self._codes("BTN_TRIGGER_HAPPY1"),
            "m2": self._codes("BTN_TRIGGER_HAPPY2"),
            "a": self._codes("BTN_SOUTH", "BTN_A"),
            "b": self._codes("BTN_EAST", "BTN_B"),
            "x": self._codes("BTN_WEST", "BTN_X"),
            "y": self._codes("BTN_NORTH", "BTN_Y"),
        }
        self._initialize_state()

    def _codes(self, *names: str) -> set[int]:
        return {
            code
            for name in names
            if (code := getattr(self.ecodes, name, None)) is not None
        }

    @property
    def fd(self) -> int:
        return self.device.fd

    @property
    def name(self) -> str:
        return self.device.name

    def _normal_axis(self, code: int, value: int) -> float:
        info = self.axis_info[code]
        center = (info.min + info.max) * 0.5
        span = max(center - info.min, info.max - center, 1.0)
        return max(-1.0, min(1.0, (value - center) / span))

    def _initialize_state(self) -> None:
        for code, attribute in (
            (self.ecodes.ABS_X, "left_x"),
            (self.ecodes.ABS_Y, "left_y"),
            (self.ecodes.ABS_RX, "right_x"),
            (self.ecodes.ABS_RY, "right_y"),
        ):
            setattr(self.state, attribute, self._normal_axis(code, self.axis_info[code].value))
        if self.has_hat_x:
            self.state.hat_x = int(self.axis_info[self.ecodes.ABS_HAT0X].value)
        if self.has_hat_y:
            self.state.hat_y = int(self.axis_info[self.ecodes.ABS_HAT0Y].value)
        self._update_keys(set(self.device.active_keys()))

    def _update_keys(self, active: set[int]) -> None:
        for attribute, codes in self._key_sets.items():
            setattr(self.state, attribute, bool(active.intersection(codes)))
        if not self.has_hat_x:
            left = getattr(self.ecodes, "BTN_DPAD_LEFT", -1) in active
            right = getattr(self.ecodes, "BTN_DPAD_RIGHT", -1) in active
            self.state.hat_x = -1 if left else 1 if right else 0
        if not self.has_hat_y:
            up = getattr(self.ecodes, "BTN_DPAD_UP", -1) in active
            down = getattr(self.ecodes, "BTN_DPAD_DOWN", -1) in active
            self.state.hat_y = -1 if up else 1 if down else 0

    def poll(self) -> set[str]:
        actions: set[str] = set()
        try:
            events = self.device.read()
        except BlockingIOError:
            return actions
        for event in events:
            if event.type == self.ecodes.EV_ABS:
                mapping = {
                    self.ecodes.ABS_X: "left_x",
                    self.ecodes.ABS_Y: "left_y",
                    self.ecodes.ABS_RX: "right_x",
                    self.ecodes.ABS_RY: "right_y",
                }
                if event.code in mapping:
                    setattr(self.state, mapping[event.code], self._normal_axis(event.code, event.value))
                elif event.code == self.ecodes.ABS_HAT0X:
                    self.state.hat_x = int(event.value)
                elif event.code == self.ecodes.ABS_HAT0Y:
                    self.state.hat_y = int(event.value)
            elif event.type == self.ecodes.EV_KEY:
                if event.value == 1:
                    if event.code in self._key_sets["start"] | self._key_sets["m1"]:
                        actions.add("map_start")
                    if event.code in self._key_sets["select"] | self._key_sets["m2"]:
                        actions.add("map_stop")
                self._update_keys(set(self.device.active_keys()))
        return actions

    def close(self) -> None:
        try:
            try:
                self.device.ungrab()
            except OSError:
                pass
        finally:
            self.device.close()
