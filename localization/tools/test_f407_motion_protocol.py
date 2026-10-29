#!/usr/bin/env python3
"""Protocol regression checks for the F407 UART motion-debug branch."""
from __future__ import annotations

import time
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace

from f407_motion_protocol import (
    FRAME_HEAD,
    FRAME_TAIL,
    MOTION_CMD_TURN_REL,
    MOTION_CMD_MOVE_DISTANCE,
    MOTION_STATE_RUNNING,
    MSG_MOTION_STATUS,
    MSG_ODOM,
    StreamParser,
    crc16_modbus,
    motion_move_frame,
    motion_stop_frame,
    motion_reset_frame,
    motion_turn_frame,
    parse_motion_status,
)


def inbound(message_type: int, sequence: int, payload: bytes) -> bytes:
    body = bytes((message_type, sequence)) + payload
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, "little") + bytes((FRAME_TAIL,))


def main() -> None:
    # Payloads match F407 4071c71 vision.c and t265_map/scan_protocol.hpp.
    turn = motion_turn_frame(0, 90.0, 300)
    assert turn[2] == 0x19 and turn[4:12] == bytes.fromhex("02 09 03 84 00 00 05 2A")
    assert motion_turn_frame(1, -180.0)[6:8] == (-1800).to_bytes(2, "big", signed=True)
    assert motion_turn_frame(2, 360.0)[6:8] == (3600).to_bytes(2, "big", signed=True)
    left = motion_move_frame(3, 90.0, 630, 300)
    assert left[4:12] == bytes.fromhex("03 0B 00 00 02 76 01 2C")
    assert motion_move_frame(4, 0.0, 740, 300)[4:12] == bytes.fromhex("03 0B 02 E4 00 00 01 2C")
    assert motion_stop_frame(5)[4:12] == bytes.fromhex("00 09 00 00 00 00 00 00")
    assert motion_reset_frame(6)[4:12] == bytes.fromhex("05 09 00 00 00 00 00 00")
    for frame in (turn, left, motion_stop_frame(5)):
        assert len(frame) == 15
        assert crc16_modbus(frame[2:12]) == int.from_bytes(frame[12:14], "little")

    odom = inbound(MSG_ODOM, 7, bytes((0, 1, 0, 2, 0, 3, 10, 7)))
    status = inbound(
        MSG_MOTION_STATUS, 4,
        bytes((9, MOTION_STATE_RUNNING, 0, MOTION_CMD_TURN_REL, 0, 100, 0x23, 0x28)),
    )
    seen = []
    parser = StreamParser(
        on_odom=lambda value: seen.append(("odom", value)),
        on_motion_status=lambda value: seen.append(("status", value)),
    )
    parser.feed(b"noise" + odom[:5])
    parser.feed(odom[5:] + status)
    assert seen[0][0] == "odom" and seen[0][1]["m3_count"] == 3
    assert seen[1][0] == "status" and seen[1][1]["command_sequence"] == 9
    assert seen[1][1]["heading_cdeg"] == 9000 and seen[1][1]["progress"] == 100
    assert parser.stats["frames"] == 2
    assert parser.stats["crc_errors"] == 0
    negative = inbound(MSG_MOTION_STATUS, 5,
                       bytes((10, 2, 0, MOTION_CMD_MOVE_DISTANCE, 0xFE, 0x0C, 0, 0)))
    assert parse_motion_status(negative)["progress"] == -500
    corrupt = bytearray(status)
    corrupt[8] ^= 1
    parser.feed(corrupt + negative)
    assert parser.stats["crc_errors"] == 1
    assert seen[-1][1]["command_sequence"] == 10
    malformed = inbound(MSG_MOTION_STATUS, 6, bytes((10, 99, 0, 2, 0, 0, 0, 0)))
    parser.feed(malformed + odom)
    assert parser.stats["malformed"] >= 1
    from t265_f407_motion_map import MotionMapApp, DEBUG_COMMAND_LEFT_SIGN, DEBUG_COMMAND_TURN_SIGN
    assert DEBUG_COMMAND_LEFT_SIGN == 1 and DEBUG_COMMAND_TURN_SIGN == -1
    app = MotionMapApp.__new__(MotionMapApp)
    app.active_test = "turn_and_return"
    app.return_phase = "turn"
    app.return_distance_mm = 1000
    app.active_command_sequence = 7
    app.active_command_opcode = MOTION_CMD_TURN_REL
    app.active_command_sent_ns = time.monotonic_ns()
    app.last_matching_status_ns = app.active_command_sent_ns
    app.record_event = lambda *args: None
    sent = []
    app.send_move = lambda *args: (sent.append(args), 8)[1]
    status = dict(command_sequence=7, command=MOTION_CMD_TURN_REL, state=2,
                  timestamp_monotonic_ns=app.active_command_sent_ns - 1)
    app.handle_motion_status(status)
    assert not sent
    status.update(timestamp_monotonic_ns=time.monotonic_ns(), command=MOTION_CMD_MOVE_DISTANCE)
    app.handle_motion_status(status)
    assert not sent
    status.update(command=MOTION_CMD_TURN_REL, command_sequence=6)
    app.handle_motion_status(status)
    assert not sent
    status.update(command_sequence=7, timestamp_monotonic_ns=time.monotonic_ns())
    app.handle_motion_status(status)
    assert sent == [(0.0, 1000)] and app.return_phase == "move"
    for terminal in (3, 4):
        app.active_test = "turn_and_return"
        app.return_phase = "turn"
        app.active_command_sequence = 7
        status.update(state=terminal, timestamp_monotonic_ns=time.monotonic_ns())
        app.handle_motion_status(status)
        assert app.active_test is None and len(sent) == 1
    app.active_test = "turn_and_return"
    app.return_phase = "turn"
    app.active_command_sequence = 7
    app.active_command_sent_ns = app.last_matching_status_ns = time.monotonic_ns() - 2_000_000_000
    status.update(state=2, timestamp_monotonic_ns=time.monotonic_ns())
    app.handle_motion_status(status)
    assert len(sent) == 1
    app.localization_json = None
    app.bridge = None
    app.update_data()
    assert app.active_test is None and app.return_phase is None
    # Exercise the same desktop entry's candidate/config and controls without
    # opening hardware, and the deletion boundary only in a temporary root.
    from t265_f407_motion_map import DEFAULT_CONFIG, prune_completed_sessions
    from t265_f407_debug import write_debug_config, action_measurements
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        config = root / "localization.conf"
        write_debug_config(DEFAULT_CONFIG, config, start_zone=4, center_origin=True,
                           start_heading_offset_deg=45, unit_translation_scale=True)
        text = config.read_text()
        assert "camera_robot_forward_axis = 0.8660254,0,0.5" in text
        assert "camera_offset_forward_m = -0.0346105" in text
        assert "start_center_m = 0.0" in text and "t265_translation_scale_enabled = false" in text
        app.options = SimpleNamespace(actual_lower_mode="unconfirmed", actual_lower_firmware="unconfirmed",
                                      reference_description="test tape zero and ground angle line")
        assert not app.motion_identity_confirmed()
        app.options.actual_lower_mode = "MotionDebug"
        app.options.actual_lower_firmware = "777acad"
        assert app.motion_identity_confirmed()
        app.measurement_buffer = None
        app.distance_mm = 1000
        moves, turns = [], []
        app.toggle_test = lambda name, action: action()
        app.send_move = lambda *args: moves.append(args)
        app.send_turn = lambda angle: turns.append(angle)
        for key in "bh123": app.handle_key(ord(key))
        assert moves == [(180.,1000),(270.,1000)] and turns == [-90.,-180.,-360.]
        app.run_dir, app.test_id, app.active_test = root, 1, None
        app.active_test_parameters = {"direction_deg_target":0,"distance_mm_target":1000}
        app.localization, app.motion_status = {}, {}
        app.save_ground_truth("988,2,?")
        measurement = json.loads((root/"ground_truth.jsonl").read_text())
        assert measurement["forward_mm"] == 988 and measurement["turn_deg_unwrapped"] is None
        try:
            app.save_ground_truth("nan,0,0")
            assert False, "nonfinite ground reference accepted"
        except ValueError:
            pass
        measurement.update(timestamp_monotonic_ns=300)
        (root/"ground_truth.jsonl").write_text(json.dumps(measurement)+"\n")
        (root/"events.jsonl").write_text(json.dumps({"test_id":1,"event":"forward_start",
                                                   "timestamp_monotonic_ns":100})+"\n")
        rows = [dict(timestamp_monotonic_ns="100",t265_x_m="0",t265_y_m="0",
                     t265_yaw_deg="0",relative_yaw_deg="0",tracker_confidence="3"),
                dict(timestamp_monotonic_ns="300",t265_x_m=".99",t265_y_m=".002",
                     t265_yaw_deg="0",relative_yaw_deg="0",tracker_confidence="3")]
        report=action_measurements(root/"localization_debug.csv",rows)[0]
        assert abs(report["t265_minus_measurement"]["forward_mm"]-2)<1e-9
        assert report["physical_motion_minus_command"]["forward_mm"] == -12
        sessions=[]
        for i in range(8):
            path=root/f"20261029_00000{i}_interactive"
            path.mkdir()
            (path/"metadata.json").write_text(json.dumps({"mode":"interactive_f407_motion_debug"}))
            (path/"session_complete.json").write_text(json.dumps({"owner":"t265_f407_motion_map",
                                                                    "complete":True,"session_index":i}))
            sessions.append(path)
        # User additions and incomplete sessions are outside automatic deletion.
        (sessions[0]/"用户笔记.txt").write_text("preserve")
        (sessions[1]/"session_complete.json").unlink()
        assert prune_completed_sessions(root,sessions[-1]) == [sessions[2].name]
        assert sessions[0].exists() and sessions[1].exists() and sessions[-1].exists()
        assert not sessions[2].exists()
        # A legacy session needs an explicit terminal event, not just a folder name.
        (sessions[1]/"events.jsonl").write_text(json.dumps({"event":"session_stopped"})+"\n")
        assert prune_completed_sessions(root,sessions[-1]) == [sessions[1].name]
        assert prune_completed_sessions(root,sessions[0]) == [sessions[3].name]
        assert (sessions[0]/"用户笔记.txt").read_text() == "preserve"
    print("F407 motion protocol PASS")


if __name__ == "__main__":
    main()
