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
    parse_imu,
    imu_is_fresh,
    MSG_IMU,
)


def inbound(message_type: int, sequence: int, payload: bytes) -> bytes:
    body = bytes((message_type, sequence)) + payload
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, "little") + bytes((FRAME_TAIL,))


def test_official_wheel_input():
    # Offline math + local IPC only. Does NOT attest SDK hardware/vehicle accuracy.
    import math
    import socket
    import uuid
    from t265_f407_motion_map import config_values, DEFAULT_CONFIG
    from t265_wheel_input import camera_velocity, WheelInputBridge, write_sdk_config
    cfg = config_values(DEFAULT_CONFIG)
    f, u, r = [0.8660254,0,.5], [0,-1,0], [-.0346105,-.0599471445,0]
    for sign in (-1, 1):
        for body, omega, expected in (([sign,0,0], [0,0,0], [sign*.8660254,0,sign*.5]),
                                     ([0,sign,0], [0,0,0], [-sign*.5,0,sign*.8660254]),
                                     ([0,0,0], [0,0,sign], [sign*.069221,0,0])):
            _, v = camera_velocity(body, omega, f, u, r)
            assert max(abs(a-b) for a,b in zip(v,expected)) < 1e-8
    _, combined = camera_velocity([.3,-.2,0], [0,0,1], f,u,r)
    parts = [camera_velocity(body,omega,f,u,r)[1] for body,omega in
             (([.3,0,0],[0,0,0]),([0,-.2,0],[0,0,0]),([0,0,0],[0,0,1]))]
    assert max(abs(combined[i]-sum(p[i] for p in parts)) for i in range(3)) < 1e-10
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        sdk_config = write_sdk_config(root/"sdk.json", .1)
        assert abs(sdk_config["velocimeters"][0]["noise_variance"]-.01) < 1e-15
        assert sdk_config["velocimeters"][0]["extrinsics"]["T"] == [0,0,0]
        session = uuid.uuid4().hex
        sink = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sink.bind("\0t265-wheel-"+session)
        sink.settimeout(.1)
        now = [1_020_000_000]
        bridge = WheelInputBridge(root, cfg, session, True, clock=lambda:now[0])
        def event(kind, sample): bridge.telemetry(dict(kind=kind, sample=sample))
        def gyro(t, **extra):
            return dict(angular_rate_capture_ns=t, receive_monotonic_ns=t+1_000_000,
                time_uncertainty_ns=1_000_000, capture_time_qualified=True,
                mcu_epoch="boot1", wire_valid=True, yaw_rate_deg_s=180/math.pi, **extra)
        wheel = dict(source="measured_wheel_feedback", v_body_mps=[0,0,0],
            interval_start_ns=1_000_000_000, interval_end_ns=1_010_000_000,
            receive_monotonic_ns=1_011_000_000, time_uncertainty_ns=1_000_000,
            capture_time_qualified=True, measurement_valid=True, mcu_epoch="boot1", sample_id=1)
        try:
            event("host_wheel_recomputed", dict(forward_mps=999, capture_monotonic_ns=None))
            event("imu", dict(wire_valid=True, yaw_rate_deg_s=999))
            bridge.poll()
            assert bridge.stats["sent_to_sdk_owner"] == 0
            event("imu",dict(gyro(1_000_000_000), yaw_rate_deg_s=0.))
            event("measured_body_velocity",wheel)
            bridge.poll()
            assert bridge.stats["sent_to_sdk_owner"] == 0  # Wait for gyro bracket, not latest value.
            event("imu",dict(gyro(1_010_000_000), yaw_rate_deg_s=360/math.pi))
            bridge.poll()
            packet = json.loads(sink.recv(16000))
            assert abs(packet["v_camera_mps"][0]-.069221) < 1e-8
            assert abs(packet["omega_body_rad_s"][2]-1) < 1e-12
            event("measured_body_velocity",wheel)
            bridge.poll()
            assert bridge.stats["last_reason"] == "duplicate_or_out_of_order_wheel_interval"
            event("measured_body_velocity",dict(wheel, source="command_target", sample_id=2))
            bridge.poll()
            assert bridge.stats["last_reason"] == "not_valid_measured_wheel_feedback"
            event("measured_body_velocity",dict(wheel, mcu_epoch="boot2", sample_id=3))
            bridge.poll()
            assert bridge.stats["last_reason"] == "epoch_changed_new_sdk_session_required"
            now[0] += 100_000_000
            event("measured_body_velocity",dict(wheel,sample_id=4))
            bridge.poll()
            assert bridge.stats["last_reason"] == "duplicate_or_out_of_order_wheel_interval"
            assert bridge.stats["sent_to_sdk_owner"] == 1  # No synthetic zero on missing data.
        finally:
            bridge.close(); sink.close()
        # Off is a distinct session; even qualified feedback is never sent.
        off = WheelInputBridge(root,cfg,uuid.uuid4().hex,False,clock=lambda:1_020_000_000)
        try:
            for t in (1_000_000_000,1_010_000_000): off.telemetry(dict(kind="imu",sample=gyro(t)))
            off.telemetry(dict(kind="measured_body_velocity",sample=wheel))
            off.poll()
            assert off.stats["last_reason"] == "injection_off" and off.stats["sent_to_sdk_owner"] == 0
        finally: off.close()
        # Missing identity and duplicate rates remain structural errors.
        cases = [
            (dict(wheel, mcu_epoch=None),
             [gyro(1_000_000_000),gyro(1_010_000_000)], "unknown_measurement_epoch"),
            (wheel, [gyro(1_000_000_000),gyro(1_000_000_000),gyro(1_010_000_000)],
             "nonmonotonic_gyro_capture_time"),
        ]
        for measured, gyros, expected in cases:
            rejected = WheelInputBridge(root,cfg,uuid.uuid4().hex,True,clock=lambda:1_040_000_000)
            try:
                for g in gyros: rejected.telemetry(dict(kind="imu",sample=g))
                rejected.telemetry(dict(kind="measured_body_velocity",sample=measured))
                rejected.poll()
                assert rejected.stats["last_reason"] == expected
                assert rejected.stats["sent_to_sdk_owner"] == 0
            finally: rejected.close()
        new_session = uuid.uuid4().hex
        sink = socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        sink.bind("\0t265-wheel-"+new_session); sink.settimeout(.1)
        zero = WheelInputBridge(root,cfg,new_session,True,clock=lambda:9_000_000_000)
        try:
            for t in (1_000_000_000,1_100_000_000):
                zero.telemetry(dict(kind="imu",sample=dict(gyro(t),yaw_rate_deg_s=0,capture_time_qualified=False,time_uncertainty_ns=None)))
            zero.telemetry(dict(kind="measured_body_velocity",sample=dict(wheel,interval_end_ns=1_100_000_000,
                capture_time_qualified=False,time_uncertainty_ns=600_000_000)))
            zero.poll()
            packet=json.loads(sink.recv(16000))
            assert packet["v_camera_mps"] == [0,0,0]
            assert packet['schema']==2 and packet['capture_time_qualified'] is False
            assert packet['time_uncertainty_ns'] is None and packet['gyro_max_gap_ns']==100_000_000
            assert packet['age_from_wheel_receive_ns']>50_000_000
            zero.poll()
            assert zero.stats["sent_to_sdk_owner"] == 1
        finally: zero.close(); sink.close()


def test_lidar_yaw_diagnostic():
    # Reference parser/timing + static geometry + the SAME chassis client. Only
    # synthetic rays, memory serial and local pseudo-terminals; no real hardware.
    import math
    import os
    import select
    import serial
    import numpy as np
    from unittest.mock import patch
    from lidar_yaw_map import LidarYawApp, DEFAULT_CONFIG as LIDAR_CONFIG, prune_completed_sessions as prune_lidar
    from lidar_yaw_debug.diagnostic import load_config, fixed_yaw_position, DiagnosticState, FieldYaw
    from lidar_yaw_debug.lidar_reader import LidarStream
    from lidar_yaw_debug.protocol import crc8, parse_frames, command_frame as lidar_command
    from f405_test_client import ChassisTestClient
    config=load_config(LIDAR_CONFIG)
    assert config["startup_fixture"]["field_heading_deg"]==135
    assert not any(config["qualification"].values())
    # Actual recorded short status bytes; one-byte feeds equal whole-buffer parse.
    raw=bytes.fromhex("54 a1 b7 7a 57 54 1f 00 00 00 41")
    expected=parse_frames(bytearray(raw),include_status=True)
    buffer=bytearray();events=[]
    for byte in raw:
        buffer.append(byte);events+=parse_frames(buffer,include_status=True)
    assert events==expected and [e[0] for e in events]==["status","status"]
    def scan_packet(index):
        first=(index*11)%360;last=(first+10)%360
        data=bytearray([0x54,0x2C])+(3600).to_bytes(2,"little")+round(first*100).to_bytes(2,"little")
        for _ in range(12):data+=(1000).to_bytes(2,"little")+bytes([30])
        data+=round(last*100).to_bytes(2,"little")+((65520+index*3)%65536).to_bytes(2,"little")
        data.append(crc8(data));return bytes(data)
    m=config["installation"]
    reader=LidarStream(dict(bearing_sign=1,bearing_zero_deg=0,physical_direction=1,
        center_raw_deg=m["center_raw_deg"],fov_deg=m["display_fov_deg"]),sensor_epoch="offline-timing")
    scans=[]
    for i in range(0,150,5):
        scans+=reader.feed(b"".join(scan_packet(j) for j in range(i,i+5)),1_000_000_000+i*3_000_000)
    assert len(scans)>=3 and reader.stats["crc_errors"]==0
    assert all(not s.timing_qualified for s in scans)  # Never qualify nominal 1ms ticks.
    assert all(all(b.time_ns>a.time_ns for a,b in zip(s.points,s.points[1:])) for s in scans)
    # Independent rectangle ray caster, actual inverted mounting and retained FOV.
    truth=np.array([-.9,.85]);heading=math.radians(135)
    ext=[*m["extrinsic"][:2],math.radians(m["extrinsic"][2])]
    def rotation(a):return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    laser_origin=truth+rotation(heading)@np.array(ext[:2])
    points=[]
    for angle in np.linspace(m["raw_start_deg"],m["raw_end_deg"],350):
        if any(lo<=angle<hi for lo,hi in m["excluded_raw_ranges_deg"]):continue
        local=np.array([math.cos(math.radians(angle)),math.sin(math.radians(angle))])
        ray=rotation(heading+ext[2])@local
        hits=[(wall-laser_origin[axis])/ray[axis] for axis in (0,1) for wall in (-1.5,1.5) if abs(ray[axis])>1e-10]
        distance=min(t for t in hits if t>0)
        hit=laser_origin+distance*ray
        # Separate exact wall segments from the extractor's 25mm corner support
        # overlap; this is a transform test, not a claim about real corner bias.
        if sum(abs(abs(hit)-1.5)<.05)==2:continue
        points.append(local*distance)
    fit=fixed_yaw_position(points,truth+[.03,-.02],heading,ext)
    assert fit["rank_xy"]==2,fit
    np.testing.assert_allclose(fit["candidate_center_xy_m"],truth,atol=1e-6)
    assert fit["yaw_rad"]==heading and not fit["qualified_pose"] and fit["covariance"] is None
    world=np.column_stack((np.full(80,-1.5),np.linspace(-.6,.6,80)))
    body=(world-truth)@rotation(heading)
    laser=(body-np.array(ext[:2]))@rotation(ext[2])
    fit=fixed_yaw_position(laser,truth+[.03,-.02],heading,ext)
    assert fit["rank_xy"]==1 and fit["position_observed_axes"]==["x"]
    assert abs(fit["candidate_center_xy_m"][1]-(truth[1]-.02))<1e-9
    state=DiagnosticState(config)
    def feedback(t,yaw=42.,rate=0.):
        state.yaw.ingest(dict(timestamp_monotonic_ns=t,wire_valid=True,age_ms=0,
            status=3,yaw_deg=yaw,yaw_rate_deg_s=rate))
        state.note_wheels(dict(timestamp_monotonic_ns=t,status=7,m1_count=123,m2_count=-456,m3_count=78))
    for t in range(1_000_000_000,1_710_000_000,10_000_000):feedback(t)
    state.operator_stationary=True
    ref=state.yaw.align(135,"operator zone1 physical startup fixture",1_700_000_000)
    assert abs(ref["offset_rad"]-math.radians(93))<1e-12  # 42 != boot zero, still maps to135.
    state.seed(*(truth+[.03,-.02]),"nearby measured branch")
    for t in range(1_710_000_000,2_310_000_000,100_000_000):
        # Keep the complete received static window, not repeated synthetic pose timestamps.
        for tt in range(t-90_000_000,t+1,10_000_000):
            if tt>state.yaw.history[-1]["timestamp_monotonic_ns"]:feedback(tt)
        result=state.observe(points,t,{"capture_time_qualified":False})
        assert result["rank_xy"]==2 and not result["qualified_pose"]
    now=t
    state.review_static("offline static checkpoint, not physical qualification",now)
    state.enter_rotation(now)
    rotating=state.observe(points,now,{})
    assert rotating["status"]=="ROTATION_OR_MOTION_CHAIN_ONLY" and rotating["rank_xy"]==0
    assert not rotating["translation_compensated"] and not rotating["rotation_compensated"]
    for i in range(1,101):feedback(now+i*10_000_000,yaw=42+i*.9,rate=90)
    assert abs(state.yaw.field_rad(now+1_000_000_000)-math.radians(225))<1e-10
    state.yaw.ingest(dict(wire_valid=False))
    assert state.yaw.offset_rad is None  # Recoverable reference invalidation; no chassis latch.
    # Ownership and no automatic commands on the REAL receive loops of two PTYs.
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);cm,cs=os.openpty();lm,ls=os.openpty()
        options=SimpleNamespace(config=LIDAR_CONFIG,lidar=os.ttyname(ls),chassis=os.ttyname(cs),
            output_root=root,http_port=0,open_browser=False,baud=230400,
            actual_lower_mode="NormalDebug",actual_lower_service="MOTIONDEBUG",actual_lower_firmware="unconfirmed")
        app=LidarYawApp(options)
        try:
            app.start()
            assert not select.select([cm,lm],[],[],.02)[0],"startup wrote motor or radar command"
            try:app.command("turn 90");assert False,"turn before static checkpoint"
            except ValueError:pass
            assert not select.select([cm,lm],[],[],.02)[0]
            app.command("scan_start")
            assert select.select([lm],[],[],1)[0]
            assert os.read(lm,16)==lidar_command(0xA0,0)+lidar_command(0xA2,3600)
            # Use the reviewed radar command helper, not a chassis opcode.
            # Drain only this PTY; it is a different physical stream from UART4.
            assert not select.select([cm],[],[],.02)[0]
            second=ChassisTestClient(os.ttyname(cs),115200,root/"second.json",root/"second.jsonl")
            try:second.start();assert False,"duplicate chassis owner"
            except serial.SerialException:pass
            app.command("stop")
            assert select.select([cm],[],[],1)[0]
            assert os.read(cm,15)[4]==0  # Shared client STOP opcode, no localization STOP lock.
            # Avoid waiting for absent test ACK on exit; this is not a real stopped proof.
            app.sent_motion=False
        finally:
            app.close(False)
            for fd in (cm,cs,lm,ls):os.close(fd)
        assert not list(root.glob("*/session_complete.json"))
        # Retention tests exclusively in temporary roots; current/user files protected.
        from lidar_yaw_map import OWNER
        sessions=[]
        for i in range(7):
            p=root/f"20261031_19000{i}_lidar_yaw";p.mkdir();sessions.append(p)
            (p/"metadata.json").write_text(json.dumps(dict(owner=OWNER)))
            (p/"session_complete.json").write_text(json.dumps(dict(owner=OWNER,complete=True,session_index=i)))
        (sessions[0]/"user-calibration.json").write_text("protected")
        removed=prune_lidar(root,sessions[-1])
        assert removed==[sessions[1].name] and sessions[-1].exists() and sessions[0].exists()
    with tempfile.TemporaryDirectory() as temporary:
        p=Path(temporary)/"config.json";bad=json.loads(json.dumps(config))
        bad["qualification"]["clock_calibrated"]=True;p.write_text(json.dumps(bad))
        try:load_config(p);assert False,"unearned qualification accepted"
        except ValueError:pass


def main() -> None:
    test_global_wall_initialization()
    test_dynamic_worker_process()
    # Wire-compatible fast CRCs: independent polynomial reference, including
    # every byte value and a multibyte sequence, not encoder/decoder roundtrip.
    from lidar_yaw_debug.protocol import crc8
    from f405_timed_protocol import crc16
    for data in [bytes([v]) for v in range(256)]+[bytes(range(256)),b'123456789']:
        c8=0;c16=65535
        for v in data:
            c8^=v;c16^=v
            for _ in range(8):
                c8=((c8<<1)^0x4d)&255 if c8&128 else (c8<<1)&255
                c16=(c16>>1)^0xa001 if c16&1 else c16>>1
        assert crc8(data)==c8 and crc16(data)==c16 and crc16_modbus(data)==c16
    test_dynamic_motion_compensation()
    test_synthetic_wheel_fixture()
    test_timed_v11_logging_recovery()
    test_timed_v13_feedback()
    test_timed_measurement_pipeline()
    test_timed_f405_client()
    test_lidar_yaw_diagnostic()
    test_official_wheel_input()
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
    # Pinned current 0x1F contract fixture, independent of legacy TURN sign.
    imu = bytes.fromhex("A3 B3 1F 07 FF F5 03 05 FB 50 01 03 31 0E C3")
    decoded = parse_imu(imu)
    assert decoded["yaw_deg"] == -720.123 and decoded["yaw_rate_deg_s"] == -120.0
    assert decoded["wire_valid"] and decoded["capture_monotonic_ns"] is None and decoded["mcu_epoch"] is None
    for yaw, rate in [(-2147483648,-32768),(2147483647,32767),(720123,1200)]:
        d = parse_imu(inbound(MSG_IMU,8,yaw.to_bytes(4,"big",signed=True)+rate.to_bytes(2,"big",signed=True)+b"\x01\x03"))
        assert d["yaw_mdeg"] == yaw and d["rate_ddeg_s"] == rate
    seen_imu=[]
    mixed=StreamParser(on_imu=seen_imu.append)
    mixed.feed(odom+imu[:8],receive_ns=1_000_000_000)
    mixed.feed(imu[8:]+status,receive_ns=1_010_000_000)
    assert mixed.stats["odom_frames"] == 1 and mixed.stats["imu_frames"] == 1
    assert mixed.stats["motion_status_frames"] == 1 and len(seen_imu) == 1
    assert seen_imu[0]["timestamp_monotonic_ns"] == 1_010_000_000
    mixed.feed(imu,receive_ns=1_020_000_000)
    assert len(seen_imu) == 1 and mixed.stats["imu_duplicates"] == 1
    assert imu_is_fresh(seen_imu[0],1_020_000_000)
    assert imu_is_fresh(seen_imu[0],1_159_000_000)  # Age is display-only.
    assert imu_is_fresh(seen_imu[0],1_009_000_000)
    for flag in (4,8,16,32,64,128):
        d=parse_imu(inbound(MSG_IMU,8,bytes(6)+bytes((1,3|flag))))
        assert not d["wire_valid"]
    assert parse_imu(inbound(MSG_IMU,8,bytes(6)+bytes((150,3))))["wire_valid"]
    wrapped=[]
    wrap_parser=StreamParser(on_imu=wrapped.append)
    for seq in (254,255,0,0,255,2):
        wrap_parser.feed(inbound(MSG_IMU,seq,bytes(6)+b"\x01\x03"))
    assert [x["sequence"] for x in wrapped] == [254,255,0,2]
    assert wrap_parser.stats["imu_sequence_gaps"] == 1
    mixed.feed(inbound(MSG_IMU,7,bytes(6)+b"\xff\x13"),receive_ns=1_030_000_000)
    assert not seen_imu[-1]["wire_valid"] and seen_imu[-1]["unique_sample"] is False
    from t265_f407_motion_map import MotionMapApp, DEBUG_COMMAND_LEFT_SIGN, DEBUG_COMMAND_TURN_SIGN
    assert DEBUG_COMMAND_LEFT_SIGN == 1 and DEBUG_COMMAND_TURN_SIGN == -1
    app = MotionMapApp.__new__(MotionMapApp)
    app.active_test = "turn_and_return"
    app.return_phase = "turn"
    app.return_distance_mm = 1000
    app.active_host_action_id = "host:1"
    app.active_command_sequence = 7
    app.active_command_opcode = MOTION_CMD_TURN_REL
    app.active_command_sent_ns = time.monotonic_ns()
    app.last_matching_status_ns = app.active_command_sent_ns
    app.record_event = lambda *args: None
    sent = []
    app.send_move = lambda *args: (sent.append(args), 8)[1]
    status = dict(command_sequence=7, command=MOTION_CMD_TURN_REL, state=2,
                  correlated=True, host_action_id="host:1", running_seen=True,
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
    status.update(correlated=False)
    status.update(state=2, timestamp_monotonic_ns=time.monotonic_ns())
    app.handle_motion_status(status)
    assert len(sent) == 1
    app.localization_json = None
    app.localization = {}
    app.wheel_reference_pose = None
    app.wheel_reference_valid = False
    app.latest_wheel_reference = None
    app.bridge = None
    app.update_data()
    assert len(sent) == 1  # An uncorrelated late terminal cannot start another move.
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
                                      actual_lower_service="unconfirmed",
                                      reference_description="test tape zero and ground angle line")
        assert not app.motion_identity_confirmed()
        app.options.actual_lower_mode = "NormalDebug"
        app.options.actual_lower_service = "MOTIONDEBUG"
        app.options.actual_lower_firmware = "14157e6"
        assert app.motion_identity_confirmed()
        app.options.actual_lower_firmware = "unconfirmed"
        assert app.motion_identity_confirmed()  # UART4 has no firmware-commit query.
        app.options.actual_lower_firmware = "a54ee465"
        assert not app.motion_identity_confirmed()  # Document commit is not firmware implementation.
        import shlex
        from unittest.mock import patch
        from t265_f407_motion_map import arguments
        entry=Path(__file__).with_name("t265-f407-debug.desktop").read_text()
        command=shlex.split(next(line[5:] for line in entry.splitlines() if line.startswith("Exec=")))
        with patch("sys.argv",["t265_f407_motion_map.py",*command[1:]]):
            app.options=arguments()
        assert app.motion_identity_confirmed() and app.options.actual_lower_firmware == "unconfirmed"
        app.options.reference_description="test tape zero and ground angle line"
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
        app.localization = dict(timestamp_monotonic_ns=time.monotonic_ns()-2_000_000_000,quality="GOOD",
                                t265={"tracker_confidence":3},pose={"x_m":.99,"y_m":.002,"yaw_deg":0})
        app.motion_status = {}
        app.settled_reference_mark = None
        try:
            app.save_ground_truth("988,2,?")
            assert False,"unmarked endpoint accepted"
        except ValueError:
            pass
        app.mark_settled_reference()
        app.localization["pose"]["x_m"] = 1.2  # Later drift/handling must not change the frozen endpoint.
        app.save_ground_truth("988,2,?")
        measurement = json.loads((root/"ground_truth.jsonl").read_text())
        assert measurement["forward_mm"] == 988 and measurement["turn_deg_unwrapped"] is None
        assert measurement["localization"]["pose"]["x_m"] == .99
        assert measurement["pose_age_at_mark_ms"]>=2000
        assert measurement["pose_sample_monotonic_ns"]==measurement["localization"]["timestamp_monotonic_ns"]
        try:
            app.save_ground_truth("nan,0,0")
            assert False, "nonfinite ground reference accepted"
        except ValueError:
            pass
        measurement.update(timestamp_monotonic_ns=300)
        measurement.pop("measurement_end_monotonic_ns")
        measurement.pop("endpoint_reference")
        measurement["localization"] = {}
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
        assert report["endpoint_reference"] == "legacy_input_time_no_settled_mark"
        measurement.update(timestamp_monotonic_ns=500,measurement_end_monotonic_ns=300,
                           endpoint_reference="operator_settled_M_mark",localization={"timestamp_monotonic_ns":300})
        rows.append(dict(timestamp_monotonic_ns="400",t265_x_m="1.2",t265_y_m=".002",
                         t265_yaw_deg="0",relative_yaw_deg="0",tracker_confidence="3"))
        (root/"ground_truth.jsonl").write_text(json.dumps(measurement)+"\n")
        marked=action_measurements(root/"localization_debug.csv",rows)[0]
        assert marked["measurement_entry_ns"] == 500 and marked["end_sample_ns"] == 300
        assert abs(marked["t265_minus_measurement"]["forward_mm"]-2)<1e-9
        # Recover existing M events without rewriting old user measurements.
        measurement.pop("measurement_end_monotonic_ns");measurement.pop("endpoint_reference")
        (root/"ground_truth.jsonl").write_text(json.dumps(measurement)+"\n")
        with (root/"events.jsonl").open("a") as stream:
            stream.write(json.dumps(dict(test_id=1,event="manual_settled_reference_mark",timestamp_monotonic_ns=300,
                                         localization={"timestamp_monotonic_ns":300}))+"\n")
        assert action_measurements(root/"localization_debug.csv",rows)[0]["end_sample_ns"] == 300
        sessions=[]
        for i in range(8):
            path=root/f"20261029_00000{i}_interactive"
            path.mkdir()
            (path/"metadata.json").write_text(json.dumps({"mode":"interactive_f405_normaldebug" if i == 7 else "interactive_f407_motion_debug"}))
            (path/"session_complete.json").write_text(json.dumps({"owner":"t265_f407_motion_map",
                                                                    "complete":True,"session_index":i}))
            sessions.append(path)
        (sessions[2]/"chassis_telemetry.jsonl").write_text("module log\n")
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
    test_f405_client()
    print("F405 test client / existing motion protocol checks PASS")


def test_f405_client():
    import math
    from unittest.mock import patch
    from f405_test_client import ChassisTestClient, WheelReference, UnsupportedCapability
    from f407_motion_protocol import move_body_frame, turn_ccw_frame
    fixtures=json.loads((Path(__file__).with_name("contracts")/"f405_normaldebug_examples_20261030.json").read_text())
    encoders={"move_body_forward_500mm_400mm_s":lambda:move_body_frame(0x31,500,0,400),
              "move_body_left_300mm_400mm_s":lambda:move_body_frame(0x32,0,300,400),
              "turn_ccw_90deg_60deg_s":lambda:turn_ccw_frame(0x33,90,60),
              "stop":lambda:motion_stop_frame(0x34),"reset_reference":lambda:motion_reset_frame(0x35)}
    for example in fixtures["examples"]:
        if example["name"] in encoders:
            assert encoders[example["name"]]()==bytes.fromhex(example["hex"])
    for operation in (lambda:move_body_frame(0,10000,10000),lambda:turn_ccw_frame(0,361),
                      lambda:turn_ccw_frame(0,90,361),lambda:turn_ccw_frame(0,0)):
        try: operation(); assert False,"invalid contract parameters accepted"
        except ValueError: pass
    def wheel(counts,seq,t,status=7):
        return dict(m1_count=counts[0]%65536,m2_count=counts[1]%65536,m3_count=counts[2]%65536,
                    sequence=seq,dt_ms=10,status=status,timestamp_monotonic_ns=t)
    for delta,axis in [((10,-10,0),"forward_mps"),((10,10,-20),"left_mps"),((10,10,10),"encoder_model_ccw_rad_s")]:
        w=WheelReference();w.update(wheel([0,0,0],1,1_000_000_000))
        result=w.update(wheel(delta,4,1_020_000_000))
        assert result["valid"] and result[axis]>0 and result["interval_s"] == .02
        assert result["lower_measured_forward_mps"] is None and result["capture_monotonic_ns"] is None
    w=WheelReference();w.update(wheel([65530,5,0],1,1_000_000_000))
    r=w.update(wheel([4,65531,0],2,1_010_000_000));assert r["raw_delta"]==[10,-10,0]
    delayed=w.update(wheel([4,65531,0],3,2_000_000_000))
    assert delayed['valid'] and delayed['interval_s']==.99
    assert delayed['capture_time_qualified'] is False  # Age alone no longer rejects a real count delta.
    class FakeSerial:
        def __init__(self): self.written=[]
        def write(self,packet): self.written.append(packet);return len(packet)
        def close(self): pass
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary)
        c=ChassisTestClient("NOT_OPENED",115200,root/"status.json",root/"status.jsonl",identity_confirmed=True)
        readonly=[]
        mutate=c.telemetry(lambda e:e['sample']['values'].append(99))
        observe=c.telemetry(readonly.append,readonly=True)
        sample={'values':[1,2]};c._publish('isolation_check',sample)
        assert sample['values']==[1,2] and readonly[0]['sample']['values']==[1,2]
        assert json.loads((root/'chassis_telemetry.jsonl').read_text())['sample']['values']==[1,2]
        mutate();observe()
        transport=FakeSerial();c.serial=transport
        clock=[10_000_000_000]
        def state(sequence,command_sequence,opcode,state):
            return inbound(MSG_MOTION_STATUS,sequence,bytes((command_sequence,state,0,opcode))+bytes(4))
        events=[];unsubscribe=c.telemetry(events.append)
        with patch("f405_test_client.time.monotonic_ns",side_effect=lambda:clock[0]):
            c.feed(state(1,10,0,0),clock[0])
            c.next_sequence=0x31
            move=c.move_body(500,0,400)
            assert transport.written[-1] == encoders["move_body_forward_500mm_400mm_s"]()
            assert move["mcu_epoch"] is None and not move["stale_reply_exclusion_guaranteed"]
            for method in (lambda:c.move_spin_angle(100,0,90,60),lambda:c.turn_ccw(90)):
                count=len(transport.written)
                try: method();assert False,"unsupported or concurrent request sent"
                except (UnsupportedCapability,RuntimeError):pass
                assert len(transport.written)==count
            clock[0]+=10_000_000
            c.feed(state(2,move["wire_sequence"],2,2),clock[0]);assert c.pending is not None
            c.feed(state(3,(move["wire_sequence"]+1)%256,3,2),clock[0]);assert c.pending is not None
            c.feed(state(4,move["wire_sequence"],3,1),clock[0])
            clock[0]+=10_000_000
            c.feed(state(5,move["wire_sequence"],3,2),clock[0]);assert c.pending is None
            result=c.drain_action_results()[-1]
            assert result["status"]["host_action_id"]==move["host_action_id"] and result["status"]["health"] is None
            c.next_sequence=0x33
            turn=c.turn_ccw(90,60);assert transport.written[-1]==encoders["turn_ccw_90deg_60deg_s"]()
            clock[0]+=10_000_000
            stop=c.stop();assert stop["wire_sequence"]!=turn["wire_sequence"]
            count=len(transport.written)
            try:c.move_body(10,0);assert False
            except RuntimeError:pass
            assert len(transport.written)==count
            clock[0]+=10_000_000
            c.feed(state(6,turn["wire_sequence"],2,2),clock[0]);assert c.pending["name"]=="stop"
            c.feed(state(7,stop["wire_sequence"],0,4),clock[0]);assert c.pending is None
            c.feed(bytes.fromhex("A3 B3 1F 08 00 01 5F 90 01 2C 01 03 7B B4 C3"),clock[0])
            count=len(transport.written)
            try:c.reset_reference();assert False
            except ValueError:pass
            assert len(transport.written)==count
            reset=c.reset_reference(stationary_confirmed=True)
            clock[0]+=10_000_000
            c.feed(state(8,reset["wire_sequence"],5,2),clock[0])
            assert c.host_reference_epoch==1 and c.imu_snapshot()["yaw_deg"]==90
            motion=c.move_body(100,0)
            clock[0]+=1_100_000_000;c.poll(clock[0])
            assert c.pending is not None  # Missing recent status alone is diagnostic.
            clock[0]=c.pending['deadline_ns']+1;c.poll(clock[0])
            assert c.pending is None and c.needs_stop_ack
            count=len(transport.written)
            c.feed(state(9,motion["wire_sequence"],3,2),clock[0])
            assert c.needs_stop_ack and len(transport.written)==count
            stop=c.stop();clock[0]+=10_000_000
            c.feed(state(10,stop["wire_sequence"],0,4),clock[0])
            assert not c.needs_stop_ack
            failed_stop=c.stop();clock[0]+=10_000_000
            c.feed(state(11,failed_stop["wire_sequence"],0,3),clock[0])
            assert c.pending is None and c.needs_stop_ack  # A failed STOP is not STOPPED.
            recovery_stop=c.stop();clock[0]+=10_000_000
            c.feed(state(12,recovery_stop["wire_sequence"],0,4),clock[0])
            assert not c.needs_stop_ack  # Explicit recovery, no permanent fault latch.
            # One serial read with multiple 0x15 frames is one receive interval.
            c.feed(inbound(MSG_ODOM,1,bytes(6)+b"\x0a\x07"),clock[0])
            clock[0]+=20_000_000
            c.feed(inbound(MSG_ODOM,2,bytes.fromhex("00 0A FF F6 00 00 0A 07"))+
                   inbound(MSG_ODOM,3,bytes.fromhex("00 14 FF EC 00 00 0A 07")),clock[0])
            r=c.drain_wheel_increments()[-1]
            assert r["coalesced_frames"]==1 and r["raw_delta"]==[20,-20,0] and r["valid"]
            assert c.capabilities()["move_spin_angle"] is False
            c.used_sequences={i:clock[0] for i in range(256)}
            count=len(transport.written)
            try:c.move_body(10,0);assert False,"short sequence reuse guard bypassed"
            except RuntimeError:pass
            assert len(transport.written)==count
            c.stop()  # Explicit STOP remains available even inside the reuse guard.
            count=len(transport.written);unsubscribe();c.close()
            assert len(transport.written)==count
        assert events and (root/"chassis_telemetry.jsonl").exists()
    # Exercise the real serial-owner loop on a local pseudo-terminal only.
    # No physical UART, camera, firmware or vehicle is involved.
    import os
    import select
    import serial
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary)
        master,slave=os.openpty()
        owner=ChassisTestClient(os.ttyname(slave),115200,root/"status.json",root/"status.jsonl",identity_confirmed=True)
        second=ChassisTestClient(os.ttyname(slave),115200,root/"second.json",root/"second.jsonl")
        try:
            owner.start()
            assert not select.select([master],[],[],.02)[0], "opening the client transmitted bytes"
            try: second.start();assert False,"second UART owner allowed"
            except serial.SerialException:pass
            os.write(master,state(1,50,0,0))
            deadline=time.monotonic()+1
            while owner.status_snapshot()[0] is None and time.monotonic()<deadline:time.sleep(.01)
            assert owner.status_snapshot()[0] is not None
            action=owner.turn_ccw(90,60)
            assert select.select([master],[],[],1)[0]
            assert os.read(master,15)==bytes.fromhex(action["packet_hex"])
            os.write(master,state(2,action["wire_sequence"],2,1)+state(3,action["wire_sequence"],2,2))
            deadline=time.monotonic()+1
            while owner.pending is not None and time.monotonic()<deadline:time.sleep(.01)
            assert owner.last_action["outcome"]=="reported_done"
            assert owner.last_action["status"]["running_seen"]
            owner.close()
            assert not select.select([master],[],[],.02)[0], "closing the client transmitted bytes"
        finally:
            owner.close();second.close();os.close(master);os.close(slave)


def test_timed_f405_client():
    """Audited v1 vectors + fragmented mixed input + fake/PTY device identity.
    No real board, motor, T265, subscription or time calibration in this test.
    """
    import struct
    from unittest.mock import patch
    import f405_timed_protocol as timed
    from f405_test_client import ChassisTestClient, TIMED_IMPLEMENTATION_COMMIT, TIMED_DOCUMENT_COMMIT
    fixtures=json.loads((Path(__file__).with_name("contracts")/"f405_timed_examples_20261031.json").read_text())
    for example in fixtures["examples"]:
        if "decoded" in example and "expected_rejection" not in example["decoded"]:
            assert timed.decode(bytes.fromhex(example["hex"]))==example["decoded"],example["name"]
    base=next(bytes.fromhex(e["hex"]) for e in fixtures["examples"] if e["name"]=="c_module_host_fixture_type_2")
    def identity_response(request, *, baud=230400, epoch=1, service=1, nonce_delta=0, sequence_delta=0, commit=TIMED_IMPLEMENTATION_COMMIT):
        req=timed.parse(request);p=bytearray(timed.parse(base)["payload"])
        p[:8]=(int.from_bytes(req["payload"][:8],"big")+nonce_delta).to_bytes(8,"big")
        p[10:26]=bytes([epoch])*16;p[41]=service
        p[48:52]=baud.to_bytes(4,"big");p[114:134]=bytes.fromhex(commit)
        # Payload magics must NOT become independent legacy/extended packets.
        p[60:64]=bytes.fromhex("a3b3d44d")
        return timed.packet(timed.ID_REPLY,(req["sequence"]+sequence_delta)%65536,p)
    legacy=inbound(MSG_ODOM,17,bytes.fromhex("d4 4d a3 b3 00 00 0a 07"))
    request=timed.request(timed.ID_REQ,123,9)
    extended=identity_response(request)
    seen=[];mux=timed.MixedStream(lambda raw,ns:seen.append(("legacy",raw)),lambda raw,ns:seen.append(("extended",raw)))
    stream=b"noise"+legacy+extended+legacy
    for i,b in enumerate(stream):mux.feed(bytes([b]),1_000_000_000+i*1000)
    assert seen==[("legacy",legacy),("extended",extended),("legacy",legacy)]
    bad=bytearray(extended);bad[-2]^=1
    mux.feed(bytes(bad)+legacy,1_001_000_000)
    assert seen[-1]==("legacy",legacy) and mux.stats["mixed_crc_errors"]>=1
    class MemorySerial:
        def __init__(self):self.written=[]
        def write(self,p):self.written.append(p);return len(p)
        def close(self):pass
    def status(seq,command_seq=0,opcode=0,state=0):
        return inbound(MSG_MOTION_STATUS,seq,bytes([command_seq,state,0,opcode])+bytes(4))
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);c=ChassisTestClient("MEMORY",230400,root/"status.json",root/"status.jsonl",
            identity_confirmed=True,identity_policy="timed")
        uart=MemorySerial();c.serial=uart;clock=[10_000_000_000];published=[];c.telemetry(published.append)
        with patch("f405_test_client.time.monotonic_ns",side_effect=lambda:clock[0]):
            c.feed(status(1),clock[0])
            req=uart.written[-1];assert timed.parse(req)["type"]==timed.ID_REQ
            try:c.turn_ccw(90);assert False,"operator profile bypassed device identity"
            except RuntimeError:pass
            assert all(p[:2]==timed.MAGIC for p in uart.written)
            clock[0]+=1_000_000
            c.feed(identity_response(req,nonce_delta=1),clock[0]);assert c.device_identity is None
            c.feed(identity_response(req,sequence_delta=1),clock[0]);assert c.device_identity is None
            c.feed(identity_response(req),clock[0]);assert c.control_identity_problem() is None
            assert c.connection_snapshot()["actual_lower_firmware"]==TIMED_IMPLEMENTATION_COMMIT
            assert not c.capabilities()["sdk_time_qualified"]
            turn=c.turn_ccw(90,60)
            assert uart.written[-1][6:8]==(-900).to_bytes(2,"big",signed=True)
            assert turn["observed_boot_epoch_at_send"]=="01"*16 and turn["mcu_epoch"] is None
            clock[0]+=10_000_000
            c.feed(status(2,turn["wire_sequence"],2,2),clock[0])
            assert c.pending is None
            gyro=next(e for e in fixtures["examples"] if e["name"]=="gyro_ccw")
            c.feed(bytes.fromhex(gyro["hex"]),clock[0])
            event=published[-1]
            assert event["kind"]=="timed_gyro_diagnostic" and event["sample"]["capture_time_qualified"] is False
            assert event["sample"]["rate_uncertainty_us"]==0xffffffff
            assert not any(e["kind"]=="measured_body_velocity" for e in published)
            # A changed boot cancels correlation, never automatically replays motion.
            turn=c.turn_ccw(-90,30)
            c._request_identity();req=uart.written[-1];clock[0]+=1_000_000
            c.feed(identity_response(req,epoch=2),clock[0])
            assert c.pending is None and c.needs_stop_ack
            assert c.latest_status is None and c.last_action["outcome"].startswith("unknown_device_restart")
            c.feed(status(1),clock[0])
            stop=c.stop();clock[0]+=1_000_000
            c.feed(status(2,stop["wire_sequence"],0,4),clock[0]);assert not c.needs_stop_ack
            clock[0]+=7_000_000_000
            assert c.control_identity_problem() is None  # Age alone no longer revokes a known compatible identity.
            c._request_identity();req=uart.written[-1];clock[0]+=1_000_000
            c.feed(identity_response(req,epoch=2,service=2),clock[0]);assert c.control_identity_problem() is not None
            c._request_identity();req=uart.written[-1];clock[0]+=1_000_000
            c.feed(identity_response(req,epoch=2),clock[0]);assert c.control_identity_problem() is None
            c._request_identity();req=uart.written[-1];clock[0]+=1_000_000
            c.feed(identity_response(req,epoch=2,commit=TIMED_DOCUMENT_COMMIT),clock[0])
            assert c.control_identity_problem() is None  # Reviewed doc-only rebuild, actually reported Git preserved.
            assert c.connection_snapshot()["actual_lower_firmware"]==TIMED_DOCUMENT_COMMIT
            c._request_identity();req=uart.written[-1];clock[0]+=1_000_000
            c.feed(identity_response(req,epoch=2,commit="ff"*20),clock[0])
            assert c.control_identity_problem() is not None  # Unknown firmware is not blindly permitted.
            before=len(uart.written);c.close();assert len(uart.written)==before
            assert all(timed.parse(p)["type"]==timed.ID_REQ for p in uart.written if p[:2]==timed.MAGIC)
    # Real serial thread and nonce handshake against one local PTY, not hardware.
    import os,select,threading,serial
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);master,slave=os.openpty();stop=threading.Event();received=[]
        def fake_board():
            buffer=bytearray();counter=0;last_status=0
            while not stop.is_set():
                if select.select([master],[],[],.01)[0]:
                    buffer.extend(os.read(master,4096))
                    while len(buffer)>=8:
                        size=int.from_bytes(buffer[4:6],"big")+11
                        if len(buffer)<size:break
                        raw=bytes(buffer[:size]);del buffer[:size];received.append(raw)
                        assert timed.parse(raw)["type"]==timed.ID_REQ
                        os.write(master,identity_response(raw))
                if time.monotonic()-last_status>.05:
                    counter=(counter+1)%256;os.write(master,status(counter));last_status=time.monotonic()
        worker=threading.Thread(target=fake_board,daemon=True);worker.start()
        client=ChassisTestClient(os.ttyname(slave),230400,root/"status.json",root/"status.jsonl",identity_policy="timed")
        try:
            client.start();assert client.control_identity_problem() is None
            assert client.connection_snapshot()["identity"]["uart_baud"]==230400
            assert client.status_snapshot()[0] is not None and received
            before=len(received);client.close();time.sleep(.03);assert len(received)==before
        finally:
            stop.set();worker.join(timeout=1);client.close();os.close(master);os.close(slave)
    # The new read-only check uses its own five-session retention, never T265
    # history. Exercise deletion only in an owned temporary root.
    from f405_connection_check import prune, OWNER as CHECK_OWNER
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);sessions=[]
        for i in range(8):
            p=root/f"20261101_00000{i}_check";p.mkdir();sessions.append(p)
            (p/"metadata.json").write_text(json.dumps(dict(owner=CHECK_OWNER)))
            (p/"session_complete.json").write_text(json.dumps(dict(owner=CHECK_OWNER,complete=True,session_index=i)))
        (sessions[0]/"用户测量.txt").write_text("preserve")
        (sessions[1]/"session_complete.json").unlink()
        assert prune(root,sessions[-1])==[sessions[2].name]
        assert sessions[0].exists() and sessions[1].exists() and sessions[-1].exists()


def test_global_wall_initialization():
    import math
    import numpy as np
    from lidar_yaw_debug.dynamic import DynamicFence,rotation
    from lidar_yaw_debug.diagnostic import load_config
    from lidar_yaw_debug.types import RawScan,TimedPoint
    mount=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')['installation']
    ext=np.array([*mount['extrinsic'][:2],math.radians(mount['extrinsic'][2])])
    origin=2_000_000_000
    def cloud(xy,theta,scan_number,one_wall=False):
        points=[];walls=[];laser=np.array(xy)+rotation(theta)@ext[:2]
        for i,angle in enumerate(np.linspace(143.25,288.25,320)):
            direction=np.array([math.cos(theta+ext[2]+math.radians(angle)),math.sin(theta+ext[2]+math.radians(angle))])
            hits=[]
            for axis in (0,1):
                for border in (-1.5,1.5):
                    if abs(direction[axis])<1e-10:continue
                    distance=(border-laser[axis])/direction[axis];hit=laser+distance*direction
                    if distance>0 and abs(hit[1-axis])<=1.5+1e-9:hits.append((distance,axis,border))
            distance,axis,border=min(hits)
            points.append(TimedPoint(f'{scan_number}:{i}',origin+scan_number*100_000_000+i*100_000,
                (distance*math.cos(math.radians(angle)),distance*math.sin(math.radians(angle))),raw_angle_deg=float(angle)))
            walls.append((axis,border))
        if one_wall:
            dominant=max(set(walls),key=walls.count);points=[p for p,w in zip(points,walls) if w==dominant]
        return RawScan(f'square:{scan_number}',points[-1].time_ns,tuple(points),'fixture','known_synthetic')
    for xy in ((0.,0.),(.4,-.3)):
        for quadrant in range(4):
            e=DynamicFence(mount);theta=math.radians(13+90*quadrant)
            for i in range(101):
                t=origin-200_000_000+i*5_000_000
                e.ingest_pose(dict(session_id='fixture',frame_number=i,timestamp_monotonic_ns=t,
                    receive_monotonic_ns=t,x_m=0.,y_m=0.,yaw_rad=0.,tracker_confidence=3))
                e.ingest_yaw(t,1287.,dict(source='synthetic_gyro'))
            e.request_initialization('offline arbitrary placement',physical_confirmed=False)
            for scan_number in range(3):
                sample=cloud(xy,theta,scan_number);result=e.try_initialize(sample)
                assert e.try_initialize(sample)['status']=='DUPLICATE_INITIAL_SCAN'
            assert e.reference is not None,result
            expected=rotation(-quadrant*math.pi/2)@xy
            # The reviewed 25mm inlier band includes a few points next to a
            # corner. Check a tight bounded fit, not exact arithmetic equality.
            assert np.linalg.norm(np.array(e.reference['start_center_xy_m'])-expected)<.001,(xy,quadrant,e.reference,expected)
            assert abs(e.reference['field_heading_deg']-13)<.1
            assert e.reference['square_absolute_side_identified'] is False
            scan=cloud(xy,theta,3);before=e.offset.copy();e.set_lidar_enabled(False)
            paused=e.observe(scan)
            assert paused['rank_xy']==0 and np.array_equal(e.offset,before)
            assert e.snapshot()['ready'] and not e.snapshot()['lidar_corrections_enabled']
            e.set_lidar_enabled(True);assert e.observe(scan)['rank_xy']==2
    e.request_initialization('single wall must not locate XY',physical_confirmed=False)
    for i in range(3):e.try_initialize(cloud((.4,-.3),theta,i,one_wall=True))
    assert e.reference is None


def test_dynamic_worker_process():
    # Existing regression entry; a PTY fixture, no physical UART or SDK.
    import os,pty
    from lidar_yaw_debug.worker import RadarWorker
    from lidar_yaw_debug.protocol import crc8
    from lidar_yaw_debug.diagnostic import load_config
    master,slave=pty.openpty();os.set_blocking(master,False)
    with tempfile.TemporaryDirectory() as tmp:
        root=Path(tmp);worker=RadarWorker()
        try:
            cfg=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')
            worker.start(cfg,os.ttyname(slave),230400,root)
            deadline=time.monotonic()+5;started=False
            while time.monotonic()<deadline and not started:
                started=any(m['kind']=='started' for m in worker.poll());time.sleep(.01)
            assert started and worker.process.pid!=os.getpid(),worker.state
            now=time.monotonic_ns()
            for i,t in enumerate((now-1_000_000_000,now)):
                worker.send('pose',dict(session_id='offline',frame_number=i,timestamp_monotonic_ns=t,
                    receive_monotonic_ns=t,x_m=-1.35,y_m=1.35,yaw_rad=2.356194490192345,tracker_confidence=3))
                worker.send('yaw',dict(time_ns=t,yaw_deg=135.,source={'source':'PTY fixture'}))
            worker.align('PTY math/transport fixture',physical_confirmed=False)
            raw=bytearray()
            for i in range(180):
                first=(i*600)%36000;last=(first+550)%36000
                packet=bytearray(b'\x54\x2c')+(3600).to_bytes(2,'little')+first.to_bytes(2,'little')
                packet += (b'\xe8\x03\x64')*12
                packet += last.to_bytes(2,'little')+(1000+2*i).to_bytes(2,'little');packet.append(crc8(packet));raw.extend(packet)
            deadline=time.monotonic()+4;written=0
            while time.monotonic()<deadline and (written<len(raw) or worker.state.get('scans_received',0)<2):
                try:written+=os.write(master,raw[written:]) if written<len(raw) else 0
                except BlockingIOError:pass
                worker.poll();time.sleep(.01)
            assert worker.state.get('scans_received',0)>=2 and worker.state.get('capture_chain_present')
        finally:
            worker.close();os.close(master);os.close(slave)
        assert worker.stopped_cleanly and not worker.process.is_alive()
        assert worker.state['logging_healthy'] and (root/'lidar_worker_summary.json').exists()
        assert not (root/'commands.jsonl').exists()


def test_dynamic_motion_compensation():
    import math
    import numpy as np
    from lidar_yaw_debug.dynamic import DynamicFence,rotation,deskew_points
    from lidar_yaw_debug.types import RawScan,TimedPoint
    from lidar_yaw_debug.diagnostic import load_config
    mount=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')['installation']
    engine=DynamicFence(mount);origin=1_000_000_000
    def pose(t):return np.array([-1.35+.7*t/math.sqrt(2),1.35-.7*t/math.sqrt(2),math.radians(135)+2*math.pi*t])
    for i in range(41):
        t=i*.005;p=pose(t);ns=origin+round(t*1e9)
        engine.ingest_pose(dict(session_id='offline',frame_number=i,timestamp_monotonic_ns=ns,
            receive_monotonic_ns=ns,x_m=p[0],y_m=p[1],yaw_rad=p[2],tracker_confidence=3))
        engine.ingest_yaw(ns,360*t,dict(source='independent_synthetic_motion'))
        if i==0:engine.align('offline known trajectory',physical_confirmed=False)
    ext=np.array([*mount['extrinsic'][:2],math.radians(mount['extrinsic'][2])]);points=[];walls=[]
    for i,(t,a) in enumerate(zip(np.linspace(.12,.16,400),np.linspace(143.25,288.25,400))):
        p=pose(t);angle=p[2]+ext[2]+math.radians(a);direction=np.array([math.cos(angle),math.sin(angle)])
        laser=p[:2]+rotation(p[2])@ext[:2];hits=[]
        for axis in (0,1):
            for border in (-1.5,1.5):
                if abs(direction[axis])<1e-9:continue
                d=(border-laser[axis])/direction[axis];hit=laser+d*direction
                if d>0 and abs(hit[1-axis])<=1.5+1e-9:hits.append((d,axis,border))
        d,axis,border=min(hits)
        points.append(TimedPoint(str(i),origin+round(t*1e9),(d*math.cos(math.radians(a)),d*math.sin(math.radians(a))),raw_angle_deg=a))
        walls.append((axis,border))
    scan=RawScan('moving',points[-1].time_ns,tuple(points),'offline','known_synthetic',False)
    ph,yh,ref,_,_=engine._frozen();query=engine.trajectory(ph,yh,ref)
    for start,end in ((origin-30_000_000,origin-5_000_000),
                      (origin+17_000_000,origin+23_000_000),
                      (origin+195_000_000,origin+215_000_000),
                      (origin+250_000_000,origin+270_000_000)):
        compact_p,compact_y,compact_ref,_,_=engine._frozen(start,end)
        ts=np.array([start,(start+end)//2,end],np.int64)
        assert np.allclose(engine.trajectory(compact_p,compact_y,compact_ref)(ts),query(ts),rtol=0,atol=1e-10)
    assert len(engine.poses)==41 and len(engine.yaws)==41  # Histories were not pruned.
    corrected,reference=deskew_points(np.array([p.point_lidar_m for p in points]),np.array([p.time_ns for p in points]),scan.reference_ns,query,ext)
    world=corrected@rotation(reference[2]).T+reference[:2]
    assert max(abs(world[i,axis]-border) for i,(axis,border) in enumerate(walls))<1e-8
    result=engine.observe(scan)
    assert result['rank_xy']>0 and result['deskew_applied'] and not result['deskew_qualified']
    assert np.linalg.norm(np.array(result['candidate_center_xy_m'])-pose(.16)[:2])<1e-6
    dominant=max(set(walls),key=walls.count);single=[p for p,w in zip(points,walls) if w==dominant]
    single_scan=RawScan('single',single[-1].time_ns,tuple(single),'offline','known_synthetic',False)
    one=engine.observe(single_scan);assert one['rank_xy']==1
    assert len(one['position_observed_axes'])==1 and len(one['inherited_axes'])==1
    old=engine.snapshot(origin+5_000_000_000)
    assert old['ready'] and old['motion_age_ms']>4000  # No age admission gate.
    engine.align('new physical reference',physical_confirmed=False)
    assert engine.snapshot()['source']=='VIO_FROM_INITIAL_REFERENCE'
    from unittest.mock import patch
    from lidar_t265_motion_map import DynamicMapApp,arguments
    with patch('sys.argv',['dynamic-map']),patch.object(DynamicMapApp,'detect_screen_size',return_value=(1024,768)):
        app=DynamicMapApp(arguments())
        try:
            image=app.render_screen();assert image.shape==(768,1024,3)
            assert {'start_dynamic','stop','speed_fast','backward'}<=set(app.hitboxes)
            app.measurement_buffer='-500,0,?'
            assert app.render_screen().shape==(768,1024,3)
        finally:app.stop_session()


def test_synthetic_wheel_fixture():
    from unittest.mock import patch
    import uuid
    from t265_wheel_input import SyntheticWheelInputBridge
    from t265_f407_motion_map import config_values,DEFAULT_CONFIG,MotionMapApp,arguments
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);sid=uuid.uuid4().hex
        b=SyntheticWheelInputBridge(root,config_values(DEFAULT_CONFIG),sid,True)
        try:
            p=b.packet_for('ccw_lever_only',[0,0,0],[0,0,.5],1_000_000_000)
            assert abs(p['v_camera_mps'][0]-.0346105)<1e-8
            assert p['source']=='synthetic_test_fixture' and p['measurement_valid'] is False
            assert p['mcu_epoch'] is None and p['fixture_session_id']==sid
            try:b.telemetry(dict(kind='imu',sample={}));assert False,'mixed real and synthetic source'
            except RuntimeError:pass
        finally:b.close()
        with patch('sys.argv',['map','--wheel-fixture','axes','--wheel-input','on','--output-root',str(root)]):options=arguments()
        assert options.sensor_check_seconds==30
        child=SimpleNamespace(pid=123,poll=lambda:0,returncode=0)
        with patch.object(MotionMapApp,'detect_screen_size',return_value=(1024,768)), \
             patch('t265_f407_motion_map.F407SerialBridge',side_effect=AssertionError('synthetic opened chassis UART')), \
             patch('t265_f407_motion_map.subprocess.Popen',return_value=child), \
             patch('t265_f407_motion_map.subprocess.check_output',return_value='offline'):
            app=MotionMapApp(options)
            try:
                app.start_session();m=json.loads((app.run_dir/'metadata.json').read_text())
                assert app.bridge is None and m['chassis_uart_opened'] is False
                assert m['actual_lower_firmware']=='not_opened'
                assert m['potential_input_dependency_sources']==['synthetic_test_fixture']
                assert '--debug-wheel-synthetic' in m['localizer_command']
            finally:app.stop_session()


def test_timed_v11_logging_recovery():
    """Only the reported version/logging/button failures; no real UART or USB."""
    import threading
    from unittest.mock import patch
    from async_debug_log import AsyncDebugLog
    from f405_test_client import ChassisTestClient,TIMED_V11_COMMITS
    from t265_f407_motion_map import MotionMapApp
    import f405_timed_protocol as wire
    examples=json.loads((Path(__file__).with_name('contracts')/'f405_timed_examples_20261031.json').read_text())['examples']
    payload=bytearray(wire.parse(bytes.fromhex(next(e['hex'] for e in examples if e['name']=='c_module_host_fixture_type_2')))['payload'])
    class Memory:
        def __init__(self):self.written=[]
        def write(self,p):self.written.append(p);return len(p)
        def close(self):pass
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);c=ChassisTestClient('FAKE',230400,root/'s.json',root/'s.jsonl',identity_policy='timed')
        c.serial=Memory();now=[10_000_000_000]
        with patch('f405_test_client.time.monotonic_ns',side_effect=lambda:now[0]):
            for sha in TIMED_V11_COMMITS:
                c._request_identity();req=wire.parse(c.serial.written[-1])
                p=bytearray(payload);p[:8]=req['payload'];p[43]=1;p[30:38]=(2047).to_bytes(8,'big');p[114:134]=bytes.fromhex(sha)
                now[0]+=1_000_000;c.feed(wire.packet(wire.ID_REPLY,req['sequence'],p),now[0])
                assert c.control_identity_problem() is None
                assert c.connection_snapshot()['actual_lower_firmware']==sha and not c.capabilities()['sdk_time_qualified']
            identity=dict(c.device_identity)
            assert c._identity_problem(dict(identity,protocol_minor=0))
            assert c._identity_problem(dict(identity,protocol_minor=2))
            assert c._identity_problem(dict(identity,git_commit='ff'*20))
            assert c._identity_problem(dict(identity,capabilities=1023))
            # A real clock-fault flag blocks source-time qualification, not the
            # independently compatible legacy control interface.
            c._request_identity();req=wire.parse(c.serial.written[-1]);p[:8]=req['payload'];p[8:10]=(2).to_bytes(2,'big')
            now[0]+=1_000_000;c.feed(wire.packet(wire.ID_REPLY,req['sequence'],p),now[0])
            assert not c.device_identity['epoch_valid'] and c.control_identity_problem() is None
            c.device_identity['protocol_minor']=2;c.identity_error=c._identity_problem(c.device_identity)
            app=MotionMapApp.__new__(MotionMapApp);app.bridge=c
            app.distance_mm=1000;app.speed_mm_s=300;app.turn_rate_deg_s=60;events=[]
            app.record_event=lambda event,details:events.append((event,details))
            before=len(c.serial.written)
            app.toggle_test('forward',lambda:(_ for _ in ()).throw(AssertionError('blocked action executed')))
            assert events[-1][0]=='chassis_request_not_sent' and '2' in events[-1][1]['reason']
            assert not events[-1][1]['uart_motion_write_attempted'] and len(c.serial.written)==before
            # Logging failure must not create an identity-request retry storm.
            c.log_error='fixture log failure';c.next_identity_query_ns=0
            for _ in range(1000):c.poll(now[0])
            assert c.next_identity_query_ns>now[0] and len(c.serial.written)==before
            c.close()
    # Preserve every raw record and timestamp; coalesce only latest-value UI
    # snapshots while a slow writer is held. Same bounded queue capacity.
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);gate=threading.Event();original=AsyncDebugLog._run
        def paused(logger):gate.wait(1);original(logger)
        with patch.object(AsyncDebugLog,'_run',paused):logger=AsyncDebugLog(root)
        for i in range(100):
            assert logger.submit('raw.jsonl',dict(timestamp_monotonic_ns=i))
            assert logger.submit('status.json',dict(timestamp_monotonic_ns=i),atomic=True)
        gate.set();logger.close()
        assert logger.healthy() and logger.statistics()['history_records_written']==100
        assert logger.statistics()['snapshots_coalesced']==99 and logger.statistics()['snapshot_writes']==1
        assert [json.loads(line)['timestamp_monotonic_ns'] for line in (root/'raw.jsonl').read_text().splitlines()]==list(range(100))
        assert json.loads((root/'status.json').read_text())['timestamp_monotonic_ns']==99


def test_timed_measurement_pipeline():
    import socket,uuid,threading,struct
    from unittest.mock import patch
    import f405_timed_protocol as wire
    from f405_timed_measurements import TimedMeasurements
    from t265_wheel_input import WheelInputBridge
    from t265_f407_motion_map import config_values,DEFAULT_CONFIG,prune_completed_sessions
    from async_debug_log import AsyncDebugLog
    examples=json.loads((Path(__file__).with_name("contracts")/"f405_timed_examples_20261031.json").read_text())["examples"]
    wheel=wire.decode(bytes.fromhex(next(x["hex"] for x in examples if x["name"]=="forward")))
    gyro=wire.decode(bytes.fromhex(next(x["hex"] for x in examples if x["name"]=="gyro_ccw")))
    identity=dict(boot_epoch=wheel["boot_epoch"],epoch_valid=True,drift_bound_ppm=0xffffffff)
    sync=dict(boot_epoch=wheel["boot_epoch"],epoch_valid=1,host_write_before_ns=900_000_000,
        receive_monotonic_ns=901_000_000,receive_event_us=1_000_000,response_prepare_us=1_000_100,
        tick_resolution_us=1,nonce=1)
    m=TimedMeasurements();m.set_identity(identity);assert m.sync(sync)
    observed=m.wheel(dict(wheel,receive_monotonic_ns=921_000_000),925_000_000)
    assert observed["measurement_valid"] and not observed["capture_time_qualified"]
    assert observed["time_uncertainty_ns"] is None
    assert "mcu_clock_drift_bound_unknown" in observed["qualification_reasons"]
    g=m.gyro(dict(gyro,receive_monotonic_ns=921_000_000),925_000_000)
    assert not g["capture_time_qualified"] and "IMU_acquisition_filter_link_to_ISR_delay_unbounded" in g["qualification_reasons"]
    assert m.gyro(dict(gyro,receive_monotonic_ns=922_000_000),926_000_000) is None
    assert m.stats["duplicate_rate"]==1
    # Actual timed wire shapes with unknown drift/source bounds and NO SYNC:
    # align in MCU event time, preserve null host mapping, and send once.
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);sid=uuid.uuid4().hex
        sink=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);sink.bind('\0t265-wheel-'+sid);sink.settimeout(.2)
        bridge=WheelInputBridge(root,config_values(DEFAULT_CONFIG),sid,True,clock=lambda:9_000_000_000)
        try:
            bridge.telemetry(dict(kind='timed_identity',sample=dict(identity,interface_compatible=True)))
            for i,t in enumerate([wheel['t0_us'],wheel['t1_us']]):
                bridge.telemetry(dict(kind='timed_gyro_diagnostic',sample=dict(gyro,rate_receive_us=t,
                    rate_source_id=1000+i,receive_monotonic_ns=1_500_000_000)))
            bridge.telemetry(dict(kind='timed_wheel_diagnostic',sample=dict(wheel,receive_monotonic_ns=1_500_000_000)))
            bridge.poll();p=json.loads(sink.recv(16000))
            assert p['alignment_clock']=='mcu_event' and p['alignment_start_ns']==wheel['t0_us']*1000
            assert p['interval_start_ns'] is None and p['time_uncertainty_ns'] is None and not p['capture_time_qualified']
            assert p['age_from_wheel_receive_ns']==7_500_000_000
            assert p['v_body_mps'][0]==wheel['forward_um_s']/1e6 and bridge.stats['sent_to_sdk_owner']==1
            bridge.telemetry(dict(kind='timed_wheel_diagnostic',sample=dict(wheel,receive_monotonic_ns=1_600_000_000)))
            bridge.poll();assert bridge.stats['sent_to_sdk_owner']==1
        finally:bridge.close();sink.close()
    # Synthetic QUALIFIED source only, not the actual f81 firmware calibration.
    m=TimedMeasurements();m.set_identity(dict(identity,drift_bound_ppm=10));assert m.sync(sync)
    qualified=m.wheel(dict(wheel,flags=131,source_uncertainty_us=100,receive_monotonic_ns=921_000_000),925_000_000)
    assert qualified["capture_time_qualified"] and 0<qualified["time_uncertainty_ns"]<=5_000_000
    assert abs(qualified["v_body_mps"][0]-.239272)<1e-9
    assert m.wheel(dict(wheel,measurement_id=101,forward_um_s=900000,receive_monotonic_ns=921_000_000),925_000_000)["measurement_valid"] is False
    m.set_identity(dict(identity,boot_epoch="aa"*16,drift_bound_ppm=10))
    assert not m.syncs and not m.clock_status(925_000_000)["qualified"]
    from f405_test_client import ChassisTestClient,TIMED_DOCUMENT_COMMIT
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary)
        class Memory:
            def __init__(self):self.written=[]
            def write(self,p):self.written.append(p);return len(p)
            def close(self):pass
        c=ChassisTestClient("FAKE",230400,root/"s.json",root/"s.jsonl",identity_policy="timed")
        c.serial=Memory();clock=[900_000_000]
        with patch("f405_test_client.time.monotonic_ns",side_effect=lambda:clock[0]):
            c._request_identity();req=wire.parse(c.serial.written[-1]);payload=bytearray(wire.parse(bytes.fromhex(next(e['hex'] for e in examples if e['name']=='c_module_host_fixture_type_2')))['payload'])
            payload[:8]=req['payload'];payload[114:134]=bytes.fromhex(TIMED_DOCUMENT_COMMIT)
            c.feed(wire.packet(wire.ID_REPLY,req['sequence'],payload),clock[0]+1000)
            c.start_timed_measurements();c.poll(clock[0]+2000)
            assert wire.parse(c.serial.written[-1])['type']==wire.CONFIG_REQ  # SYNC is diagnostic, not a subscription gate.
            req=next(wire.parse(p) for p in c.serial.written if wire.parse(p)['type']==wire.SYNC_REQ)
            wrong=bytearray(payload);wrong[:8]=req['payload']
            c.feed(wire.packet(wire.ID_REPLY,req['sequence'],wrong),clock[0]+3000)
            assert len(c.identity_queries)==2 and not c.have_sync_exchange  # Type must match too.
            nonce=int.from_bytes(req['payload'],'big');epoch=bytes.fromhex(c.device_identity['boot_epoch'])
            clock[0]+=1_000_000
            c.feed(wire.packet(wire.SYNC_REPLY,req['sequence'],struct.pack('>Q16sQQIH',nonce,epoch,1_000_000,1_000_100,1,1)),clock[0])
            c.poll(clock[0]+1000);req=wire.parse(c.serial.written[-1]);assert req['type']==wire.CONFIG_REQ and req['payload'][8:]==b'\x01\x01\x00\x00'
            nonce=int.from_bytes(req['payload'][:8],'big')
            c.feed(wire.packet(wire.CONFIG_REPLY,req['sequence'],struct.pack('>Q16sIHBBIIII',nonce,epoch,1,0,1,1,230400,20000,10000,230400)),clock[0]+2000)
            assert c.measurement_snapshot()['stream_active']
            c.timed_requested=False;c._request_timed(wire.CONFIG_REQ,enable=False);req=wire.parse(c.serial.written[-1])
            nonce=int.from_bytes(req['payload'][:8],'big')
            c.feed(wire.packet(wire.CONFIG_REPLY,req['sequence'],struct.pack('>Q16sIHBBIIII',nonce,epoch,1,0,1,0,230400,20000,10000,230400)),clock[0]+3000)
            assert not c.measurement_snapshot()['stream_active']
            assert all(p[:2]==wire.MAGIC for p in c.serial.written)  # No motion/STOP/reset.
            c.close()
    # A blocked GUI and blocked disk writer cannot gate wheel processing. This
    # is local IPC to a fake SDK owner, not a hardware/precision/fusion test.
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);sid=uuid.uuid4().hex;sink=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        sink.bind("\0t265-wheel-"+sid);sink.settimeout(.2)
        disk_gate=threading.Event();original=AsyncDebugLog._run
        def slow_disk(logger):disk_gate.wait(1);original(logger)
        with patch.object(AsyncDebugLog,"_run",slow_disk):
            bridge=WheelInputBridge(root,config_values(DEFAULT_CONFIG),sid,True)
        bridge.start()
        try:
            now=time.monotonic_ns()
            for capture in (now-20_000_000,now-10_000_000):
                bridge.telemetry(dict(kind="imu",sample=dict(angular_rate_capture_ns=capture,
                    receive_monotonic_ns=now,time_uncertainty_ns=1_000_000,capture_time_qualified=True,
                    mcu_epoch="test-boot",wire_valid=True,yaw_rate_deg_s=0)))
            bridge.telemetry(dict(kind="measured_body_velocity",sample=dict(source="measured_wheel_feedback",
                v_body_mps=[.1,0,0],interval_start_ns=now-20_000_000,interval_end_ns=now-10_000_000,
                receive_monotonic_ns=now,time_uncertainty_ns=1_000_000,capture_time_qualified=True,
                measurement_valid=True,mcu_epoch="test-boot",sample_id=1)))
            time.sleep(.10)  # No GUI update_data/poll call at all.
            packet=json.loads(sink.recv(16000));assert abs(packet["v_camera_mps"][0]-.08660254)<1e-8
            assert bridge.snapshot()["sdk_api_success_samples"]==0  # IPC success is NOT SDK success.
            assert bridge.snapshot(dict(session_id=sid,config_loaded=True))["state"]=="SDK_CONFIG_LOADED"
            assert bridge.snapshot(dict(session_id=sid,config_loaded=True,sdk_api_success_samples=1,last_sdk_success_ns=now))["state"]=="SDK_API_ACCEPTED"
            assert bridge.snapshot(dict(session_id=sid,sdk_api_rate_hz=100,status_monotonic_ns=now-2_000_000_000))["sdk_api_rate_hz"]==0
            assert bridge.snapshot(dict(session_id=sid,sdk_api_rate_hz=5,status_monotonic_ns=time.monotonic_ns()))["sdk_api_rate_hz"]==5
            assert bridge.snapshot(dict(session_id="old-session",sdk_api_success_samples=999))["sdk_api_success_samples"]==0
            assert bridge.snapshot()["processing_latency_ms_max"]<50
            # Same source interval is not sent twice, regardless of its age.
            bridge.telemetry(dict(kind="measured_body_velocity",sample=dict(source="measured_wheel_feedback",
                v_body_mps=[.1,0,0],interval_start_ns=now-20_000_000,interval_end_ns=now-10_000_000,
                receive_monotonic_ns=now,time_uncertainty_ns=1_000_000,capture_time_qualified=True,
                measurement_valid=True,mcu_epoch="test-boot",sample_id=2)))
            deadline=time.monotonic()+.2
            while bridge.snapshot()["last_input_rejection"]!="duplicate_or_out_of_order_wheel_interval" and time.monotonic()<deadline:time.sleep(.005)
            assert bridge.snapshot()["last_input_rejection"]=="duplicate_or_out_of_order_wheel_interval"
            assert bridge.stats["sent_to_sdk_owner"]==1
        finally:disk_gate.set();bridge.close();sink.close()
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);bridge=WheelInputBridge(root,config_values(DEFAULT_CONFIG),uuid.uuid4().hex,False)
        try:
            for _ in range(300):bridge.telemetry(dict(kind="host_wheel_recomputed",sample={}))
            bridge.poll()
            assert bridge.stats['subscriber_dropped']==44 and bridge.stats['overflow_discarded']==256
            assert bridge.stats['sent_to_sdk_owner']==0
        finally:bridge.close()
    # Only this module's manifest-owned images can be removed. Reproduce the
    # five-complete-session retention using temporary files, never real logs.
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);sessions=[]
        for i in range(8):
            path=root/f"20261101_00000{i}_interactive";path.mkdir();sessions.append(path)
            (path/'metadata.json').write_text(json.dumps(dict(mode='interactive_f405_normaldebug',sdk_session_id='test',fisheye_evidence_enabled=True)))
            (path/'session_complete.json').write_text(json.dumps(dict(owner='t265_f407_motion_map',complete=True,session_index=i)))
        (root/'protected_sessions.json').write_text(json.dumps(dict(owner='t265_f407_motion_map',sessions=[sessions[0].name])))
        (sessions[2]/'用户文件.bin').write_bytes(b'preserve')
        image=sessions[1]/'fisheye_b1_s1_f100.pgm';image.write_bytes(b'P5\n1 1\n255\n\0')
        manifest=sessions[1]/'fisheye_frames.csv'
        manifest.write_text('session_id,filename\nwrong,fisheye_b1_s1_f100.pgm\n')
        assert prune_completed_sessions(root,sessions[-1])==[]
        manifest.write_text('session_id,filename\ntest,fisheye_b1_s1_f100.pgm\n')
        assert prune_completed_sessions(root,sessions[-1])==[sessions[1].name]
        assert sessions[0].is_dir() and sessions[2].is_dir() and sessions[-1].is_dir()


def test_timed_v13_feedback():
    """1.3 identity -> recovery feedback -> existing IPC; no hardware/SDK."""
    import socket,uuid,queue
    from unittest.mock import patch
    import f405_timed_protocol as wire
    from f405_test_client import (ChassisTestClient, TIMED_V13_COMMITS,
        TIMED_V13_DOCUMENT_COMMIT, TIMED_V13_IMPLEMENTATION_COMMIT)
    from f405_timed_measurements import TimedMeasurements
    from t265_wheel_input import WheelInputBridge
    from t265_f407_motion_map import config_values,DEFAULT_CONFIG,MotionMapApp
    from lidar_t265_motion_map import DynamicMapApp
    examples=json.loads((Path(__file__).with_name('contracts')/'f405_timed_examples_20261031.json').read_text())['examples']
    def fixture(name):return wire.decode(bytes.fromhex(next(x['hex'] for x in examples if x['name']==name)))
    base=bytearray(wire.parse(bytes.fromhex(next(x['hex'] for x in examples if x['name']=='c_module_host_fixture_type_2')))['payload'])
    class Memory:
        def __init__(self):self.written=[]
        def write(self,p):self.written.append(p);return len(p)
        def close(self):pass
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);c=ChassisTestClient('FAKE',230400,root/'s.json',root/'s.jsonl',identity_policy='timed')
        c.serial=Memory();clock=[10_000_000_000]
        with patch('f405_test_client.time.monotonic_ns',side_effect=lambda:clock[0]):
            for sha in TIMED_V13_COMMITS:
                c._request_identity();req=wire.parse(c.serial.written[-1]);p=bytearray(base)
                p[:8]=req['payload'];p[42:44]=bytes((1,3));p[30:38]=(0x3df).to_bytes(8,'big');p[114:134]=bytes.fromhex(sha)
                clock[0]+=1_000_000;c.feed(wire.packet(wire.ID_REPLY,req['sequence'],p),clock[0])
                assert c.control_identity_problem() is None
                link=c.connection_snapshot();caps=c.capabilities()
                assert link['actual_lower_firmware']==sha and link['expected_lower_firmware']==TIMED_V13_IMPLEMENTATION_COMMIT
                assert link['contract_document_commit']==TIMED_V13_DOCUMENT_COMMIT
                assert not caps['quality_query_available'] and not caps['stats_query_available']
            identity=dict(c.device_identity)
            assert c._identity_problem(dict(identity,protocol_minor=1))
            assert c._identity_problem(dict(identity,git_commit='ff'*20))
            before=len(c.serial.written)
            recovery=inbound(MSG_IMU,2,(1234).to_bytes(4,'big',signed=True)+(10).to_bytes(2,'big',signed=True)+bytes((255,7)))
            c.feed(recovery,clock[0]);assert c.latest_imu['wire_valid'] and c.latest_imu['recovering']
            assert c.latest_imu['age_ms']==255 and c.latest_imu['yaw_deg']==1.234
            assert c.latest_imu['feedback_protocol']==(1,3) and len(c.serial.written)==before
            stamp=c.latest_imu['timestamp_monotonic_ns'];c.feed(recovery,clock[0]+1000)
            assert c.latest_imu['timestamp_monotonic_ns']==stamp and c.parser.stats['imu_duplicates']==1
            assert not parse_imu(recovery)['wire_valid']  # Unidentified/older semantics stay explicit.
            assert parse_imu(recovery,protocol_version=(1,3))['wire_valid']
            for flag in (0x20,0x40,0x80):
                bad=inbound(MSG_IMU,3,bytes(6)+bytes((255,7|flag)))
                assert not parse_imu(bad,protocol_version=(1,3))['wire_valid']
            damaged=bytearray(recovery);damaged[5]^=1;c.feed(damaged,clock[0]+2000)
            assert c.parser.stats['crc_errors']+c.mixed.stats['mixed_crc_errors']>0

        wheel=fixture('forward');gyro=fixture('gyro_ccw')
        identity.update(boot_epoch=wheel['boot_epoch'],epoch_valid=True,interface_compatible=True)
        now=20_000_000_000
        for flags,expected in ((0x81,True),(0x85,True),(0x181,True),(0x89,False),(0x91,False),(0xa1,False),(0x80,False)):
            m=TimedMeasurements();m.set_identity(identity)
            result=m.wheel(dict(wheel,flags=flags,receive_monotonic_ns=now),now)
            assert result['measurement_valid'] is expected,(flags,result)
            assert not result['capture_time_qualified']
        m=TimedMeasurements();m.set_identity(identity)
        result=m.gyro(dict(gyro,rate_flags=0x347,receive_monotonic_ns=now),now)
        assert result['wire_valid'] and not result['capture_time_qualified']
        assert m.gyro(dict(gyro,rate_flags=0x387,receive_monotonic_ns=now+1),now+1) is None
        assert not m.gyro_source_valid(dict(gyro,rate_flags=0x367))  # Clamped.
        assert m.gyro_source_valid(dict(gyro,rate_flags=0x345))  # Existing source before control READY.

        app=DynamicMapApp.__new__(DynamicMapApp);app.clock_map=TimedMeasurements();app.clock_map.set_identity(identity)
        app.chassis_inbox=queue.Queue();app.last_yaw_id=None;app.timed_yaw_seen=False;app.imu_packets=0
        sent=[];app.radar=SimpleNamespace(send=lambda kind,value:sent.append((kind,value)))
        g=dict(gyro,yaw_flags=0x347,receive_monotonic_ns=now)
        for _ in range(2):app.chassis_inbox.put(dict(kind='timed_gyro_diagnostic',sample=g))
        app.drain_yaw();assert app.imu_packets==1 and len(sent)==1 and app.timed_yaw_seen

        sid=uuid.uuid4().hex;sink=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        sink.bind('\0t265-wheel-'+sid);sink.settimeout(.3)
        bridge=WheelInputBridge(root,config_values(DEFAULT_CONFIG),sid,True,clock=lambda:now)
        try:
            bridge.telemetry(dict(kind='timed_identity',sample=identity))
            for i,t in enumerate((wheel['t0_us'],wheel['t1_us'])):
                bridge.telemetry(dict(kind='timed_gyro_diagnostic',sample=dict(gyro,rate_flags=0x347,
                    rate_source_id=100+i,rate_receive_us=t,receive_monotonic_ns=now-5_000_000_000)))
            event=dict(kind='timed_wheel_diagnostic',sample=dict(wheel,flags=0x85,receive_monotonic_ns=now-5_000_000_000))
            bridge.telemetry(event);bridge.poll();packet=json.loads(sink.recv(16000))
            assert packet['v_body_mps'][0]==wheel['forward_um_s']/1e6
            assert packet['alignment_clock']=='mcu_event' and not packet['capture_time_qualified']
            assert bridge.stats['sent_to_sdk_owner']==1
            bridge.telemetry(event);bridge.poll();assert bridge.stats['sent_to_sdk_owner']==1
        finally:bridge.close();sink.close()

    check=MotionMapApp.__new__(MotionMapApp);check.options=SimpleNamespace(wheel_input='on')
    check.localization={};check.wheel_sdk_status={'sdk_api_success_samples':10}
    try:check.validate_sensor_check();assert False,'SDK true without Pose cannot pass'
    except RuntimeError:pass
    check.localization={'pose':{'x_m':0,'y_m':0}};check.wheel_sdk_status={'config_loaded':True,'sdk_api_success_samples':0}
    try:check.validate_sensor_check();assert False,'config alone cannot pass'
    except RuntimeError:pass
    check.wheel_sdk_status['sdk_api_success_samples']=1;check.validate_sensor_check()


if __name__ == "__main__":
    main()
