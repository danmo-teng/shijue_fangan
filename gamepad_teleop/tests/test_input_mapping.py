#!/usr/bin/env python3
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from input_mapping import ArmGate, PadState, command_from_pad  # noqa: E402
from protocol import (  # noqa: E402
    BUTTON_CLAW_CLOSE,
    BUTTON_LIFT_UP,
    BUTTON_X,
    BUTTON_Y,
)


class InputMappingTests(unittest.TestCase):
    def test_right_stick_up_is_forward(self) -> None:
        result = command_from_pad(PadState(right_y=-1.0, rb=True), 0.12, 35)
        self.assertEqual(result[0], 100)
        self.assertEqual(result[1:4], (0, 0, 0))
        self.assertTrue(result[-1])

    def test_left_stick_left_is_counter_clockwise(self) -> None:
        result = command_from_pad(PadState(left_x=-1.0, rb=True), 0.12, 35)
        self.assertEqual(result[2], 100)

    def test_dpad_and_precision(self) -> None:
        result = command_from_pad(
            PadState(hat_x=-1, hat_y=-1, lb=True, rb=True), 0.12, 35
        )
        self.assertEqual(result[4], BUTTON_CLAW_CLOSE | BUTTON_LIFT_UP)
        self.assertEqual(result[5], 35)

    def test_x_and_y_are_reserved_for_map_window(self) -> None:
        result = command_from_pad(PadState(x=True, y=True, rb=True), 0.12, 35)
        self.assertEqual(result[4] & (BUTTON_X | BUTTON_Y), 0)

    def test_startup_arm_gate_requires_neutral_disarmed_state(self) -> None:
        gate = ArmGate(0.12)
        self.assertFalse(gate.update(PadState(rb=True)))
        self.assertFalse(gate.update(PadState(right_y=-1.0, rb=False)))
        self.assertFalse(gate.update(PadState(rb=False)))
        self.assertTrue(gate.update(PadState(rb=True)))


if __name__ == "__main__":
    unittest.main()
