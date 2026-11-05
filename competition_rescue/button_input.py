"""Read the existing four active-low inputs without owning GPIO configuration.

The boot service exports the pins. This module never exports, changes direction,
unexports, or writes a pin (in particular the temporary BOARD 23 supply).
Call ``poll()`` about every 20 ms from the launcher; it returns plain event dicts.
The optional ``read_level(key)`` dependency is for the existing offline checks.
"""
from dataclasses import dataclass
from pathlib import Path
import time


BUTTON_LINES = {
    1: ('34130000.gpio', 8),   # BOARD 27
    2: ('34120000.gpio', 20),  # BOARD 29
    3: ('34120000.gpio', 21),  # BOARD 31
    4: ('34130000.gpio', 10),  # BOARD 33
}


@dataclass
class _Button:
    raw: bool | None = None
    stable: bool | None = None
    changed_at: float = 0.0
    pressed_at: float = 0.0
    armed: bool = False
    long_sent: bool = False


class SysfsButtonInput:
    """Nonblocking polling adapter; no camera, UART, threads, or GPIO writes.

    Business events have ``action``, ``key`` (1..4), and ``monotonic`` fields.
    A read failure yields ``input_error`` with ``reason`` once per changed error;
    recovery yields ``input_recovered``. The affected input is rearmed only after
    release, so a missing pin, reconnect, or initial held key never creates a
    synthetic press. Other readable keys, including STOP, remain usable.
    """

    def __init__(self, sysfs_root='/sys/class/gpio', *, read_level=None,
                 debounce_s=0.03):
        self.sysfs_root = Path(sysfs_root)
        self._read_level = read_level or self._read_sysfs_level
        self.debounce_s = float(debounce_s)
        if self.debounce_s < 0:
            raise ValueError('debounce_s must be nonnegative')
        self._buttons = {key: _Button() for key in BUTTON_LINES}
        self._paths = {}
        self._errors = {}
        self._chord_blocked = False

    def _value_path(self, key):
        if key not in self._paths:
            expected_label, offset = BUTTON_LINES[key]
            for chip in self.sysfs_root.glob('gpiochip*'):
                try:
                    label = (chip / 'label').read_text().strip()
                    if label != expected_label:
                        continue
                    base = int((chip / 'base').read_text().strip())
                    count = int((chip / 'ngpio').read_text().strip())
                except (OSError, ValueError):
                    continue
                if offset >= count:
                    raise OSError(f'{expected_label} has no line {offset}')
                self._paths[key] = self.sysfs_root / f'gpio{base + offset}' / 'value'
                break
            else:
                raise OSError(f'GPIO controller {expected_label} unavailable')
        return self._paths[key]

    def _read_sysfs_level(self, key):
        try:
            value = self._value_path(key).read_text().strip()
        except OSError:
            self._paths.pop(key, None)
            raise
        if value not in ('0', '1'):
            raise ValueError(f'invalid GPIO level {value!r}')
        return int(value)

    def poll(self, now=None):
        now = time.monotonic() if now is None else float(now)
        events = []
        transitions = []
        previous_down = {key for key, b in self._buttons.items() if b.raw or b.stable}

        def emit(action, key, **extra):
            events.append(dict(action=action, key=key, monotonic=now, **extra))

        # Read all four before dispatching so simultaneous keys cannot issue
        # START/selection alongside STOP. No combos are assigned to the keys.
        for key, button in self._buttons.items():
            try:
                level = self._read_level(key)
                if level not in (0, 1):
                    raise ValueError(f'invalid GPIO level {level!r}')
            except (OSError, ValueError) as error:
                reason = str(error)
                if self._errors.get(key) != reason:
                    emit('input_error', key, reason=reason)
                self._errors[key] = reason
                self._buttons[key] = _Button()
                continue
            if key in self._errors:
                del self._errors[key]
                emit('input_recovered', key)
            pressed = level == 0
            if button.raw is None:
                button.raw = button.stable = pressed
                button.changed_at = now
                button.armed = not pressed
                continue
            if pressed != button.raw:
                button.raw = pressed
                button.changed_at = now
            if (button.raw != button.stable and
                    now - button.changed_at >= self.debounce_s):
                button.stable = button.raw
                transitions.append((key, button.stable))

        # Include undebounced presses: STOP's contact debounce must not leave a
        # window in which another key's release starts the robot.
        down = [key for key, b in self._buttons.items() if b.raw or b.stable]
        if len(set(down) | previous_down) > 1:
            self._chord_blocked = True

        for key, pressed in transitions:
            button = self._buttons[key]
            if pressed:
                button.pressed_at = button.changed_at
                button.long_sent = False
                if key == 4 and button.armed:
                    emit('stop', key)
            else:
                if (button.armed and not button.long_sent and
                        (key == 4 or not self._chord_blocked)):
                    # A delayed launcher poll may first see a long hold at
                    # release; it must not turn that hold into a short press.
                    held_s = button.changed_at - button.pressed_at
                    long_action, duration = {
                        1: ('next_zone', 1.0), 2: ('toggle_side', 1.0),
                        4: ('exit_session', 2.0),
                    }.get(key, (None, 0.0))
                    action = (long_action if long_action and held_s >= duration else
                              {1: 'query_zone', 2: 'query_side', 3: 'start'}.get(key))
                    if action:
                        emit(action, key)
                button.armed = True
                button.long_sent = False

        for key, button in self._buttons.items():
            if not (button.raw and button.stable and button.armed) or button.long_sent:
                continue
            action, duration = {
                1: ('next_zone', 1.0), 2: ('toggle_side', 1.0),
                4: ('exit_session', 2.0),
            }.get(key, (None, 0.0))
            if action and now - button.pressed_at >= duration:
                if key == 4 or not self._chord_blocked:
                    emit(action, key)
                button.long_sent = True

        if not down and not self._errors:
            self._chord_blocked = False
        # Even if a single poll observes a release at the same time as STOP,
        # STOP has priority over every other business action.
        if any(event['action'] in ('stop', 'exit_session') for event in events):
            events = [event for event in events if event['action'] in
                      ('stop', 'exit_session', 'input_error', 'input_recovered')]
        return events
