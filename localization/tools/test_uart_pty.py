#!/usr/bin/env python3
"""End-to-end UART test using a pseudo-terminal and a connected T265."""
from __future__ import annotations

import json
import os
import pathlib
import pty
import select
import struct
import subprocess
import tempfile
import threading
import time


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def frame(sequence: int, positions: tuple[int, int, int]) -> bytes:
    body = bytes((0x15, sequence & 0xFF))
    body += struct.pack(">HHHBB", *(value & 0xFFFF for value in positions), 10, 0x07)
    return b"\xA3\xB3" + body + struct.pack("<H", crc16(body)) + b"\xC3"


def main() -> int:
    root = pathlib.Path(__file__).resolve().parents[1]
    source_config = (root / "config/localization.example.conf").read_text(encoding="utf-8")
    source_config = source_config.replace(
        "startup_wheel_disable_distance_m = 0.70",
        "startup_wheel_disable_distance_m = 0.0",
    ).replace(
        "corner_exclusion_inner_m = 0.75",
        "corner_exclusion_inner_m = 2.0",
    )

    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    received = bytearray()
    reader_stop = threading.Event()

    def read_pose_frames() -> None:
        while not reader_stop.is_set():
            ready, _, _ = select.select([master], [], [], 0.05)
            if ready:
                try:
                    received.extend(os.read(master, 4096))
                except OSError:
                    return

    reader = threading.Thread(target=read_pose_frames, daemon=True)
    reader.start()
    with tempfile.TemporaryDirectory(prefix="t265-uart-test-") as temporary:
        temporary_path = pathlib.Path(temporary)
        config_path = temporary_path / "test.conf"
        output_path = temporary_path / "pose.json"
        config_path.write_text(source_config, encoding="utf-8")
        command = [
            str(root / "build/t265_omni_localizer"),
            "--config", str(config_path),
            "--uart", slave_path,
            "--duration", "2",
            "--rate", "5",
            "--output", str(output_path),
        ]
        process = subprocess.Popen(
            command,
            cwd=root,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            time.sleep(0.7)
            positions = [1000, 2000, 3000]
            for sequence in range(180):
                os.write(master, frame(sequence, tuple(positions)))
                time.sleep(0.01)
            output, _ = process.communicate(timeout=8)
        except Exception:
            process.kill()
            output, _ = process.communicate()
            print(output)
            raise
        finally:
            reader_stop.set()
            reader.join(timeout=1)
            os.close(master)
            os.close(slave)

        print(output)
        if process.returncode != 0:
            raise SystemExit(f"localizer exited with {process.returncode}")
        result = json.loads(output_path.read_text(encoding="utf-8"))
        assert result["uart"]["frames"] >= 100, result
        assert result["uart"]["crc_errors"] == 0, result
        assert result["wheel"]["accepted"] > 0, result
        pose_frames = []
        index = 0
        while index + 15 <= len(received):
            if received[index:index + 2] != b"\xA3\xB3":
                index += 1
                continue
            candidate = bytes(received[index:index + 15])
            body = candidate[2:12]
            if candidate[2] == 0x16 and candidate[14] == 0xC3 and \
                    struct.unpack("<H", candidate[12:14])[0] == crc16(body):
                pose_frames.append(candidate)
                index += 15
            else:
                index += 1
        assert pose_frames, received.hex()
        latest = pose_frames[-1]
        x_mm, y_mm, heading_cdeg = struct.unpack(">hhH", latest[4:10])
        assert abs(x_mm - 1350) <= 5, (x_mm, latest.hex())
        assert abs(y_mm + 1350) <= 5, (y_mm, latest.hex())
        assert 31400 <= heading_cdeg <= 31600, (heading_cdeg, latest.hex())
        assert latest[10] & 0x01, latest.hex()
        assert len(pose_frames) >= result["uart"]["pose_tx_frames"], (
            result, len(pose_frames)
        )
        print("PTY UART full-duplex test passed:", result["uart"],
              "received_pose_frames=", len(pose_frames))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
