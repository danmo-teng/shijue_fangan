#!/usr/bin/env python3
"""Existing UART PTY checks; --robust-no-sdk never accesses physical devices."""
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
import socket
import sys
import uuid
import signal


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


def mission_frame(sequence: int = 7) -> bytes:
    payload = bytes.fromhex("03 17 04 34 00 00 66 CE")
    body = bytes((0x18, sequence & 0xFF)) + payload
    return b"\xA3\xB3" + body + struct.pack("<H", crc16(body)) + b"\xC3"


def robust_without_sdk() -> int:
    """Real owner/PTY, mocked SDK creation/device events, no physical devices."""
    from f405_timed_protocol import packet, WHEEL, GYRO
    root = pathlib.Path(__file__).resolve().parents[1]
    cache = (root / "build/CMakeCache.txt").read_text()
    sdk_root = next(line.split("=", 1)[1] for line in cache.splitlines()
                    if line.startswith("REALSENSE_ROOT:PATH="))
    with tempfile.TemporaryDirectory(prefix="t265-no-sdk-pty-") as directory:
        temporary = pathlib.Path(directory)
        stub = temporary / "sdk_start_failure.cpp"
        library = temporary / "sdk_start_failure.so"
        stub.write_text('''#include <librealsense2/hpp/rs_types.hpp>
#include <thread>
#include <cstdlib>
struct rs2_device_list { int count; };
struct rs2_device {};
struct rs2_context { rs2_devices_changed_callback *callback=nullptr; std::thread event; };
extern "C" rs2_context *rs2_create_context(int version, rs2_error **error) {
    (void)version;
    if (std::getenv("RDK_PTY_DEVICE_EVENT")) return new rs2_context;
    *error = rs2_create_error("fixture SDK startup failure; no USB accessed",
        "rs2_create_context", "", RS2_EXCEPTION_TYPE_BACKEND);
    return nullptr;
}
extern "C" void rs2_delete_context(rs2_context *context) {
    if (!context) return;
    if (context->event.joinable()) context->event.join();
    if (context->callback) context->callback->release();
    delete context;
}
extern "C" void rs2_set_devices_changed_callback_cpp(rs2_context *context,
        rs2_devices_changed_callback *callback, rs2_error **) {
    context->callback=callback;
    context->event=std::thread([context] {
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        context->callback->on_devices_changed(new rs2_device_list{0},new rs2_device_list{1});
    });
}
extern "C" rs2_device_list *rs2_query_devices_ex(const rs2_context *, int, rs2_error **) {
    return new rs2_device_list{0};
}
extern "C" int rs2_get_device_count(const rs2_device_list *list,rs2_error **) {return list->count;}
extern "C" void rs2_delete_device_list(rs2_device_list *list) {delete list;}
extern "C" rs2_device *rs2_create_device(const rs2_device_list *,int,rs2_error **) {return new rs2_device;}
extern "C" void rs2_delete_device(rs2_device *device) {delete device;}
extern "C" int rs2_supports_device_info(const rs2_device *,rs2_camera_info,rs2_error **) {return 1;}
extern "C" const char *rs2_get_device_info(const rs2_device *,rs2_camera_info info,rs2_error **) {
    return info==RS2_CAMERA_INFO_PRODUCT_LINE?std::getenv("RDK_PTY_DEVICE_EVENT"):"fixture";
}
''')
        subprocess.run(["c++", "-shared", "-fPIC", "-std=c++14", "-pthread", "-I" + sdk_root +
                        "/third_party/librealsense/include", str(stub), "-o", str(library)], check=True)
        environment = dict(os.environ, LD_PRELOAD=str(library))
        environment.pop("RDK_PTY_DEVICE_EVENT", None)
        identifier = uuid.uuid4().hex
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        listener.bind("\0no-sdk-motion-" + identifier)
        listener.listen(1)
        listener.settimeout(3)
        master, slave = pty.openpty()
        command_path, events = temporary / "command.bin", temporary / "events.jsonl"
        command_path.write_bytes(mission_frame())
        base = [str(root / "build/t265_omni_localizer"), "--config",
                str(root / "config/localization.example.conf"), "--uart", os.ttyname(slave),
                "--duration", "0.9", "--rate", "0", "--command-file", str(command_path),
                "--events", str(events), "--output", str(temporary / "unused_pose.json"),
                "--debug-test-session", identifier]
        process = subprocess.Popen(base + ["--experimental-robust-localization",
            "--motion-feedback-socket", "@no-sdk-motion-" + identifier], cwd=root,
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        connection = None
        received, forwarded = bytearray(), []
        try:
            connection, _ = listener.accept()
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and process.poll() is None:
                if events.exists() and "t265_sdk_unavailable" in events.read_text():
                    break
                time.sleep(.005)
            assert process.poll() is None, "owner exited at SDK startup failure"
            epoch, sequence = bytes.fromhex("01" * 16), 0
            next_send = time.monotonic()
            while process.poll() is None and time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_send:
                    t0, t1 = sequence * 10000, (sequence + 1) * 10000
                    wheel = struct.pack(">16sIIQQiiIIIiiiIII", epoch, 0, sequence, t0, t1,
                                        100000, 0, 1, 2, 3, 1, 2, 3, 1, 0, 0)
                    gyro = struct.pack(">16sIIqQIIIiQIIIQ", epoch, 0, sequence, 0, t1,
                                       sequence, 1, 0, 0, t1, sequence, 1, 0, t1)
                    os.write(master, packet(WHEEL, sequence, wheel) + packet(GYRO, sequence, gyro))
                    sequence += 1
                    if sequence == 20:
                        os.kill(process.pid, signal.SIGUSR1)
                    next_send = now + .01
                ready, _, _ = select.select([master, connection], [], [], .005)
                for stream in ready:
                    if stream == master:
                        received.extend(os.read(master, 4096))
                    else:
                        data = connection.recv(4096)
                        if data:
                            forwarded.append(data)
            output, _ = process.communicate(timeout=3)
            assert process.returncode == 0, output
            records = [json.loads(line) for line in events.read_text().splitlines()]
            assert sum(item.get("event") == "t265_sdk_unavailable" for item in records) == 2, records
            assert sum(item.get("event") == "t265_sdk_reopen_requested" for item in records) == 1, records
            assert not (temporary / "unused_pose.json").exists(), "SDK failure fabricated a Pose"
            heartbeats = []
            for offset in range(len(received) - 14):
                candidate = received[offset:offset + 15]
                if candidate[:3] == b"\xa3\xb3\x18" and candidate[-1] == 0xc3 and \
                        crc16(candidate[2:12]) == int.from_bytes(candidate[12:14], "little"):
                    heartbeats.append(bytes(candidate))
            assert len(heartbeats) >= 20, (len(heartbeats), output)
            assert all(((b[3] - a[3]) & 255) == 1 for a, b in zip(heartbeats, heartbeats[1:]))
            assert {message[11] for message in forwarded} == {WHEEL, GYRO}, forwarded
            assert len(forwarded) >= 40, len(forwarded)
            default = subprocess.run(base, cwd=root, env=environment, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, timeout=3)
            assert default.returncode == 3, default.stdout
            availability = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            availability.bind("\0no-sdk-pose-" + identifier)
            availability.listen(1)
            availability.settimeout(2)
            check = None
            try:
                check = subprocess.Popen([str(root / "build/t265_omni_localizer"), "--config",
                    str(root / "config/localization.example.conf"), "--duration", "0.1", "--rate", "0",
                    "--experimental-robust-localization", "--debug-test-session", identifier,
                    "--debug-pose-socket", "@no-sdk-pose-" + identifier, "--debug-fresh-session",
                    "--t265-only-output", "--ignore-encoders", "--enable-pose-jumping"],
                    cwd=root, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True)
                peer, _ = availability.accept()
                with peer:
                    peer.settimeout(2)
                    state = json.loads(peer.recv(8192))
                check_output, _ = check.communicate(timeout=3)
                assert check.returncode == 0 and state["kind"] == "sdk_availability", (state, check_output)
                assert state["available"] is False and "x_m" not in state, state
            finally:
                if check is not None and check.poll() is None:
                    check.kill()
                    check.communicate()
                availability.close()
            attach_retries = {}
            for product_line, expected in (("T200", 1), ("D400", 0)):
                event_path = temporary / (product_line + ".jsonl")
                attach = subprocess.run([str(root / "build/t265_omni_localizer"), "--config",
                    str(root / "config/localization.example.conf"), "--duration", "0.3", "--rate", "0",
                    "--experimental-robust-localization", "--events", str(event_path)],
                    cwd=root, env=dict(environment,RDK_PTY_DEVICE_EVENT=product_line),
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=3)
                assert attach.returncode == 0, attach.stdout
                event_rows = [json.loads(line) for line in event_path.read_text().splitlines()]
                retries = sum(row.get("event") == "t265_sdk_reopen_requested" for row in event_rows)
                assert retries == expected, (product_line,event_rows,attach.stdout)
                assert sum(row.get("event") == "t265_sdk_unavailable" for row in event_rows) == 1 + expected
                attach_retries[product_line] = retries
            print(json.dumps({"sdk_stub": "startup_failure_without_USB", "physical_uart": False,
                "mission_heartbeats_after_sdk_failure": len(heartbeats),
                "wheel_gyro_forwarded_after_sdk_failure": len(forwarded),
                "default_exit_code": default.returncode, "availability_ipc": state["kind"],
                "explicit_retry_attempts": 1, "timer_retry_attempts": 0,
                "mock_added_device_retries": attach_retries,
                "pose_fabricated": False}))
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
            if connection is not None:
                connection.close()
            listener.close()
            os.close(master)
            os.close(slave)
    return 0


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
        command_path = temporary_path / "uart_command.bin"
        config_path.write_text(source_config, encoding="utf-8")
        command_path.write_bytes(mission_frame())
        command = [
            str(root / "build/t265_omni_localizer"),
            "--config", str(config_path),
            "--uart", slave_path,
            "--duration", "2",
            "--rate", "5",
            "--tx-rate", "20",
            "--output", str(output_path),
            "--command-file", str(command_path),
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
        mission_frames = []
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
            elif candidate[2] == 0x18 and candidate[14] == 0xC3 and \
                    struct.unpack("<H", candidate[12:14])[0] == crc16(body):
                mission_frames.append(candidate)
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
        # The unchanged mission input remains active without a 250 ms cutoff.
        assert len(mission_frames) >= 100, len(mission_frames)
        assert all(((b[3]-a[3]) & 0xff) == 1
                   for a,b in zip(mission_frames,mission_frames[1:]))
        assert all(item[4:12] == mission_frames[0][4:12]
                   for item in mission_frames)
        print("PTY UART full-duplex test passed:", result["uart"],
              "received_pose_frames=", len(pose_frames))
    return 0


if __name__ == "__main__":
    raise SystemExit(robust_without_sdk() if sys.argv[1:] == ["--robust-no-sdk"] else main())
