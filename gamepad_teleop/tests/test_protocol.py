#!/usr/bin/env python3
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from protocol import (  # noqa: E402
    CMD_TELEOP,
    FLAG_ACK_REQUIRED,
    FLAG_TELEOP_ENABLE,
    FLAG_VALID,
    MSG_MOTION_COMMAND,
    MSG_MOTION_STATUS,
    STATE_RUNNING,
    StreamParser,
    build_frame,
    crc16_modbus,
    decode_motion_status,
    parse_frame,
    stop_frame,
    teleop_frame,
)


class ProtocolTests(unittest.TestCase):
    def test_teleop_layout_and_crc(self) -> None:
        frame = teleop_frame(7, 100, -100, 25, -25, 0xA5, 35, True)
        self.assertEqual(len(frame), 15)
        self.assertEqual(frame[:2], bytes((0xA3, 0xB3)))
        self.assertEqual(frame[2], MSG_MOTION_COMMAND)
        self.assertEqual(frame[3], 7)
        self.assertEqual(
            frame[4:12],
            bytes((CMD_TELEOP, FLAG_VALID | FLAG_ACK_REQUIRED | FLAG_TELEOP_ENABLE,
                   100, 156, 25, 231, 0xA5, 35)),
        )
        self.assertEqual(int.from_bytes(frame[12:14], "little"), crc16_modbus(frame[2:12]))

    def test_stop_frame(self) -> None:
        frame = parse_frame(stop_frame(255), {MSG_MOTION_COMMAND})
        self.assertEqual(frame.sequence, 255)
        self.assertEqual(frame.payload, bytes((0, FLAG_VALID | FLAG_ACK_REQUIRED, 0, 0, 0, 0, 0, 0)))

    def test_stream_parser_resynchronizes(self) -> None:
        status_payload = bytes((9, STATE_RUNNING, 0, CMD_TELEOP, 0, 12, 0, 90))
        status = build_frame(MSG_MOTION_STATUS, 3, status_payload)
        parser = StreamParser()
        frames = parser.feed(b"noise" + status[:5])
        self.assertEqual(frames, [])
        frames = parser.feed(status[5:])
        self.assertEqual(len(frames), 1)
        decoded = decode_motion_status(frames[0])
        self.assertEqual(decoded.command_sequence, 9)
        self.assertEqual(decoded.progress, 12)
        self.assertEqual(parser.frames, 1)

    def test_bad_crc_is_rejected(self) -> None:
        frame = bytearray(stop_frame(1))
        frame[5] ^= 0x40
        parser = StreamParser()
        self.assertEqual(parser.feed(bytes(frame)), [])
        self.assertEqual(parser.crc_errors, 1)


if __name__ == "__main__":
    unittest.main()
