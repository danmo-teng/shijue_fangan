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


def test_native_line_hypotheses():
    """Numeric parity and all-or-original fallback, without physical devices."""
    import ctypes
    import os
    import runpy
    import numpy as np
    from unittest.mock import patch
    from lidar_yaw_debug import native_geometry as native, static
    rng=np.random.default_rng(82)
    for n in (12,32,131,500):
        ordered=np.sort(rng.normal(size=n));indices=np.arange(n)
        assert np.array_equal([static._ordered_quantile(ordered,indices,q) for q in (.05,.95)],
                              np.percentile(ordered,[5,95]))
    if native.backend_status()["backend"]=="native_streaming_ransac":
        for n in (12,50,200,1000):
            for scale in (1.,1e6):
                points=rng.normal(size=(n,2))*scale
                pairs=rng.integers(n,size=(160,2),dtype=np.int64)
                p=points[pairs[:,0]];t=points[pairs[:,1]]-p
                lengths=np.linalg.norm(t,axis=1)
                normal=np.column_stack((-t[:,1],t[:,0]))/np.maximum(lengths[:,None],1e-12)
                delta=points[:,None,:]-p[None,:,:]
                masks=(abs(delta[:,:,0]*normal[None,:,0]+delta[:,:,1]*normal[None,:,1])<.025)&(lengths>=.15)
                expected=masks[:,int(np.argmax(masks.sum(axis=0)))]
                assert np.array_equal(native.best_hypothesis(points,pairs,.025),expected)

    def reference(points,**kwargs):
        # Bypass BOTH complete extraction and optional native scorer. Patching
        # only best_hypothesis on static.extract_lines would compare C++ to itself.
        with patch.object(static,"best_hypothesis",return_value=None):
            return static._extract_lines_python(points,**kwargs)

    def same_lines(actual,expected,label):
        assert len(actual)==len(expected),label
        for a,b in zip(actual,expected):
            assert np.array_equal(a["points"],b["points"]),label
            assert np.array_equal(a["normal"],b["normal"]),label
            assert abs(a["span"]-b["span"])<=5e-15,label

    x=np.linspace(-1.4,1.4,180)
    walls=np.vstack((np.column_stack((x,np.full_like(x,1.5))),
                     np.column_stack((np.full_like(x,1.5),x))))
    points=np.vstack((walls,rng.uniform(-1.4,1.4,(25,2))))
    expected=reference(points)
    # Optional-library behavior is checked even on a machine without a build.
    for env,missing in (({"RDK_LIDAR_NATIVE":"0"},False),
                        ({"RDK_LIDAR_NATIVE":"1","RDK_LIDAR_EXTRACT_NATIVE":"0"},False),
                        ({"RDK_LIDAR_NATIVE":"1"},True)):
        with patch.dict(os.environ,env):
            if missing:
                with patch.object(ctypes,"CDLL",side_effect=OSError("fixture missing library")):
                    optional=runpy.run_path(native.__file__)
            else:optional=runpy.run_path(native.__file__)
        assert optional["backend_status"]()["extraction_backend"]=="python"
        with patch.object(static,"_native_extract_lines",optional["extract_lines"]):
            same_lines(static.extract_lines(points),expected,"unavailable backend")
    if native.backend_status()["extraction_backend"]=="python":return

    cases=[("dual walls",points),("equal projections",np.repeat(walls,3,axis=0)),
           ("empty",np.empty((0,2))),
           ("nonfinite only",np.array([[np.nan,0],[0,np.inf],[-np.inf,1]])),
           ("nonfinite mixed",np.vstack((points,[[np.nan,0],[0,np.inf]])))]
    for delta in (-1e-12,1e-12):
        span=np.linspace(0,.35/.9+delta,120)
        cases.append(("span edge "+str(delta),np.column_stack((span,np.ones_like(span)))))
        left=np.linspace(-.6,0,90);right=np.linspace(.15+delta,.75+delta,90)
        both=np.r_[left,right]
        cases.append(("gap edge "+str(delta),np.column_stack((both,np.ones_like(both)))))
    for label,cloud in cases:
        for seed in (0,23):
            same_lines(static.extract_lines(cloud,seed=seed),reference(cloud,seed=seed),label)

    # A native error may have touched its private output. No partial native
    # line is permitted to escape; re-run the original seed from the beginning.
    def failed_extract(*args):
        out,_,count=args[-3:]
        out[0].count=12;out[0].span=999
        ctypes.cast(count,ctypes.POINTER(ctypes.c_size_t))[0]=1
        return -6
    before=native.backend_status()["extraction_fallbacks"]
    with patch.object(native,"_extract",side_effect=failed_extract):
        same_lines(static.extract_lines(points),expected,"native error has partial output")
    assert native.backend_status()["extraction_fallbacks"]==before+1

    original_order=np.argsort
    calls=[0]
    def fail_first_order(values,*args,**kwargs):
        calls[0]+=1
        if calls[0]==1:raise ValueError("fixture order callback")
        return original_order(values,*args,**kwargs)
    with patch.object(np,"argsort",side_effect=fail_first_order):
        same_lines(static.extract_lines(points),expected,"order callback fallback")
    assert "fixture order callback" in native.backend_status()["extraction_last_error"]

    # Pair callback errors also unwind through C++ before falling back. The
    # fresh Python run must reconstruct the same RNG, not continue consumed state.
    real_rng=np.random.default_rng
    bad_rng=SimpleNamespace(integers=lambda *a,**k: (_ for _ in ()).throw(ValueError("fixture pairs callback")))
    with patch.object(np.random,"default_rng",side_effect=[bad_rng,real_rng(0)]):
        same_lines(static.extract_lines(points),expected,"pairs callback fallback")
    assert "fixture pairs callback" in native.backend_status()["extraction_last_error"]

    for error in (KeyboardInterrupt("fixture cancel"),SystemExit(37)):
        with patch.object(np,"argsort",side_effect=error):
            try:native.extract_lines(points)
            except BaseException as actual:assert actual is error
            else:raise AssertionError("ctypes swallowed cancellation")


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
    # A complete scan must not wait for all later packets in the same read.
    # Compare exact physical rays/IDs/times, including wrap and CRC recovery.
    for corrupt in (False,True):
        cfg=dict(bearing_sign=1,bearing_zero_deg=0,physical_direction=1,
                 center_raw_deg=m['center_raw_deg'],fov_deg=m['display_fov_deg'])
        batch_reader=LidarStream(cfg,sensor_epoch='scan-delivery-parity')
        stream_reader=LidarStream(cfg,sensor_epoch='scan-delivery-parity')
        early=False
        for i in range(0,150,30):
            packets=[scan_packet(j) for j in range(i,i+30)]
            if corrupt and i==30:
                damaged=bytearray(packets[7]);damaged[9]^=1;packets[7]=bytes(damaged)
            data=b''.join(packets);arrival=2_000_000_000+i*3_000_000
            expected=batch_reader.feed(data,arrival);delivered=[];delivered_after_packets=[]
            def emit(scan):
                delivered.append(scan)
                delivered_after_packets.append(stream_reader.stats['scan_packets'])
            actual=stream_reader.feed(data,arrival,on_scan=emit)
            assert actual==expected and delivered==expected
            early=early or any(count<stream_reader.stats['scan_packets'] for count in delivered_after_packets)
        assert early and stream_reader.stats==batch_reader.stats
        assert stream_reader.stats['crc_errors']==int(corrupt)
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
    test_native_line_hypotheses()
    test_motion_log_storage()
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
    test_dynamic_motion_source()
    test_synthetic_wheel_fixture()
    test_timed_v11_logging_recovery()
    test_timed_v13_feedback()
    test_t265_usb_recovery()
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


def test_native_lidar_reader():
    # Byte-identical input, same source epoch and RX times; the existing Python
    # path is the oracle, including malformed input and segment/epoch recovery.
    from unittest.mock import patch
    from lidar_yaw_debug.lidar_reader import LidarStream
    from lidar_yaw_debug.lidar_timing import AffineClockMap, LidarTimingError
    from lidar_yaw_debug.native_lidar_reader import make_lidar_stream, NativeLidarStream, NativeReaderError
    from lidar_yaw_debug.protocol import crc8, command_frame

    def packet(index, direction=1, *, tick=None, count=12, first=None, last=None):
        first = (direction * index * 11) % 360 if first is None else first
        last = (first + direction * 10) % 360 if last is None else last
        data = bytearray((0x54, 0x20 | count)) + (3600).to_bytes(2, 'little')
        data += round(first * 100).to_bytes(2, 'little')
        for j in range(count):
            data += (0 if j == 3 else 1000 + j).to_bytes(2, 'little') + bytes((30 + j,))
        data += round(last * 100).to_bytes(2, 'little')
        data += ((65520 + index * 3) % 65536 if tick is None else tick).to_bytes(2, 'little')
        data.append(crc8(data))
        return bytes(data)

    base = dict(bearing_sign=1, bearing_zero_deg=0., physical_direction=1,
        center_raw_deg=215.75, fov_deg=147., max_batch_span_ns=2**62)

    def compare(records, cfg=None):
        import pickle
        from dataclasses import FrozenInstanceError
        cfg = base if cfg is None else cfg
        readers = [LidarStream(cfg, sensor_epoch='native-parity'),
                   make_lidar_stream(cfg, sensor_epoch='native-parity')]
        assert isinstance(readers[1], NativeLidarStream), readers[1].backend_error
        scans, early = 0, False
        copied_checked = False
        for data, arrival in records:
            outcomes = []
            for reader in readers:
                delivered, callback_counts = [], []
                def emit(scan):
                    delivered.append(scan)
                    callback_counts.append(reader.stats['scan_packets'])
                try:
                    result = reader.feed(data, arrival, on_scan=emit)
                    assert result == delivered
                    outcome = (delivered, None)
                except ValueError as error:
                    outcome = (delivered, (type(error), str(error)))
                outcomes.append(outcome)
                early |= any(n < reader.stats['scan_packets'] for n in callback_counts)
            assert outcomes[0] == outcomes[1], outcomes
            assert readers[0].stats == readers[1].stats
            assert readers[0].needs_new_epoch == readers[1].needs_new_epoch
            assert readers[0].last_error == readers[1].last_error
            if outcomes[1][0] and not copied_checked:
                scan = outcomes[1][0][0]
                restored = pickle.loads(pickle.dumps(scan))
                assert restored == scan and vars(restored) == vars(scan)
                try:
                    restored.points[0].time_ns = 0
                except FrozenInstanceError:
                    pass
                else:
                    raise AssertionError('native copy lost frozen point semantics')
                copied_checked = True
            scans += len(outcomes[0][0])
            if outcomes[0][1]:
                break
        readers[1].close()
        if copied_checked:
            with patch('lidar_yaw_debug.native_lidar_reader._load',
                       side_effect=AssertionError('transport must not load a device/native backend')):
                assert pickle.loads(pickle.dumps(restored)) == restored
            # Ordinary Python scans keep their previous independent pickle
            # format; they do not need a native payload or a compiled library.
            from lidar_yaw_debug.types import RawScan
            ordinary = RawScan(restored.scan_id, restored.reference_ns, restored.points,
                restored.sensor_epoch, restored.time_map_version, restored.timing_qualified)
            assert pickle.loads(pickle.dumps(ordinary)) == ordinary
        return scans, early, readers[0].stats

    for direction in (1, -1):
        cfg = dict(base, physical_direction=direction, bearing_sign=-direction,
                   bearing_zero_deg=17.5, center_raw_deg=0.)
        raw = b''.join(packet(i, direction) for i in range(200))
        for chunk_size in (1, 19, 47, 1410):
            records = [(raw[i:i+chunk_size], 3_000_000_000 + i * 100_000)
                       for i in range(0, len(raw), chunk_size)]
            scans, early, stats = compare(records, cfg)
            assert scans >= 5 and stats['timing_rejections'] == stats['sector_rejections'] == 0
            if chunk_size == 1410:
                assert early

    # Both observed status layouts, ordinary A1 length 0/1 and other ACKs;
    # corrupted lengths/CRCs must resynchronise one byte, then regain sectors.
    short = bytes.fromhex('54a1b77a57541f00000041')
    ack0 = bytes((0x54, 0xa1, 0)); ack0 += bytes((crc8(ack0),))
    ack1 = bytes((0x54, 0xa1, 1, 27)); ack1 += bytes((crc8(ack1),))
    packets = [packet(i) for i in range(200)]
    damaged = bytearray(packets[40]); damaged[9] ^= 1; packets[40] = bytes(damaged)
    packets[80] = b'junk\x54\x00' + short + ack0 + ack1 + command_frame(0xa2, 3600) + packets[80]
    raw = b''.join(packets)
    records = [(raw[i:i+173], 4_000_000_000 + i * 100_000) for i in range(0, len(raw), 173)]
    scans, _, stats = compare(records)
    assert scans >= 4 and stats['crc_errors'] >= 1
    assert stats['status_packets'] == 2 and stats['ack_packets'] == 3

    normal = [(packet(i), 5_000_000_000 + i*3_000_000) for i in range(100)]
    for change in (dict(sector_limits=dict(max_points=4)),
                   dict(sector_limits=dict(max_span_ns=1)),
                   dict(sector_limits=dict(max_point_gap_ns=1)),
                   dict(sector_limits=dict(max_angle_step_deg=.1))):
        assert compare(normal, dict(base, **change))[2]['sector_rejections'] > 0
    # Interpolation loss, malformed angle, repeated/reset counter, first-batch
    # ambiguity, RX reversal, buffer bound and unrepresentable mapped time.
    cases = [normal[:45] + [(packet(46), 5_138_000_000)] + normal[47:],
        normal[:45] + [(packet(45, first=400), 5_135_000_000)] + normal[46:],
        normal[:45] + [(packet(44), 5_135_000_000)],
        [(packet(0)+packet(0), 5_000_000_000)],
        normal[:10] + [(packet(10), 1)],
        [(bytes(65537), 5_000_000_000)],
        [(packet(0)+packet(1), 0)]]
    for records in cases:
        compare(records)
    compare([(packet(i, tick=i+1, count=1), 5_000_000_000+i*3_000_000) for i in range(80)])

    # The old reader remains selectable and is used if a build/ABI is absent or
    # a supplied calibration map needs its original full implementation.
    with patch.dict('os.environ', {'RDK_LIDAR_READER_NATIVE': '0'}):
        assert type(make_lidar_stream(base, sensor_epoch='off')) is LidarStream
    with patch('lidar_yaw_debug.native_lidar_reader._load', side_effect=NativeReaderError('old build')):
        fallback = make_lidar_stream(base, sensor_epoch='old-build')
        assert type(fallback) is LidarStream and fallback.backend_error == 'old build'
    clock = AffineClockMap(0, 10**9, 10**6, 'explicit-map')
    fallback = make_lidar_stream(base, sensor_epoch='calibrated-path', clock_map=clock)
    assert type(fallback) is LidarStream and fallback.adapter.clock_map is clock
    for cls in (LidarStream, make_lidar_stream):
        reader = cls(base, sensor_epoch='callback-failure')
        try:
            for data, arrival in normal:
                reader.feed(data, arrival, on_scan=lambda scan: (_ for _ in ()).throw(RuntimeError('consumer')))
        except RuntimeError as error:
            assert str(error) == 'consumer'
        else:
            raise AssertionError('callback exception was swallowed')


def test_native_lidar_receiver_recovery():
    # Real local PTYs only: a native failure must retain delivered sectors,
    # continue on Python, and survive a subsequent serial disconnect/reopen.
    import os
    import pty
    import queue
    import threading
    from unittest.mock import patch
    from lidar_yaw_debug import worker
    from lidar_yaw_debug.native_lidar_reader import NativeReaderError
    from lidar_yaw_debug.protocol import crc8

    class Channel(queue.Queue):
        def cancel_join_thread(self): pass

    def packets(start):
        output = bytearray()
        for i in range(start, start + 150):
            first = i * 11 % 360; last = (first + 10) % 360
            data = bytearray((0x54, 0x2c)) + (3600).to_bytes(2, 'little')
            data += (first * 100).to_bytes(2, 'little') + b'\xe8\x03\x64' * 12
            data += (last * 100).to_bytes(2, 'little') + ((i * 3) % 65536).to_bytes(2, 'little')
            data.append(crc8(data)); output.extend(data)
        return output

    original_make = worker.make_reader
    failed = []
    def make(config, *, native=True):
        reader = original_make(config, native=native)
        if native:
            assert reader.backend == 'cpp_bytes_timing_sector_v1'
            original_feed = reader.feed
            def feed(*args, **kwargs):
                result = original_feed(*args, **kwargs)
                if result and not failed:
                    failed.append(True)
                    raise NativeReaderError('fixture native failure after delivered sector')
                return result
            reader.feed = feed
        return reader

    scans, updates, stop = Channel(64), Channel(8), threading.Event()
    state = {}
    def wait_for(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try: state.update(updates.get(timeout=.02))
            except queue.Empty: pass
            if predicate(state): return
        raise AssertionError(state)

    config = json.loads((Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json').read_text())
    descriptors = []
    with tempfile.TemporaryDirectory() as tmp, patch.object(worker, 'make_reader', make):
        root = Path(tmp); port = root/'lidar'
        master, slave = pty.openpty(); descriptors.extend((master, slave)); port.symlink_to(os.ttyname(slave))
        thread = threading.Thread(target=worker.run_lidar_receiver,
            args=(config, str(port), 230400, root, scans, updates, stop))
        thread.start()
        try:
            wait_for(lambda s: s.get('lidar_connected'))
            os.write(master, packets(0))
            wait_for(lambda s: s.get('reader_backend') == 'python')
            assert failed and 'fixture native failure' in state['reader_backend_error']
            before = state['scans_received']
            os.write(master, packets(150))
            wait_for(lambda s: s.get('scans_received', 0) > before)
            os.close(master); descriptors.remove(master)
            wait_for(lambda s: not s.get('lidar_connected'))
            port.unlink()
            master, slave = pty.openpty(); descriptors.extend((master, slave)); port.symlink_to(os.ttyname(slave))
            wait_for(lambda s: s.get('lidar_connected') and s.get('lidar_open_attempts', 0) >= 2)
            before = state['scans_received']
            os.write(master, packets(300))
            wait_for(lambda s: s.get('scans_received', 0) > before)
        finally:
            stop.set(); thread.join(timeout=3)
            for descriptor in descriptors: os.close(descriptor)
        assert not thread.is_alive()
        events = [json.loads(line) for line in (root/'lidar_reader_events.jsonl').read_text().splitlines()]
        assert sum(e['event'] == 'lidar_reader_fallback' for e in events) == 1
        assert sum(e['event'] == 'lidar_connected' for e in events) == 2
        assert any(e['event'] == 'lidar_connection_error' for e in events)
        raw = [json.loads(line) for line in (root/'dynamic_scans.jsonl').read_text().splitlines()]
        assert len({s['scan_id'] for s in raw}) == len(raw) >= 3
        assert len({s['scan_id'].split(':')[0] for s in raw}) == 3
        assert all(not s['time_qualified'] for s in raw)
        summary = json.loads((root/'lidar_reader_summary.json').read_text())
        assert summary['reader_backend'] == 'python' and summary['reader_logging_healthy']
        assert not (root/'commands.jsonl').exists()


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
            queries=c.client_stats['identity_queries']
            c.poll(clock[0]+7_000_000_000)
            assert c.client_stats['identity_queries']==queries  # No periodic compatible-identity handshake.
            assert c.connection_snapshot()["actual_lower_firmware"]==TIMED_IMPLEMENTATION_COMMIT
            assert not c.capabilities()["sdk_time_qualified"]
            turn=c.turn_ccw(90,60)
            assert uart.written[-1][6:8]==(-900).to_bytes(2,"big",signed=True)
            assert turn["observed_boot_epoch_at_send"]=="01"*16 and turn["mcu_epoch"] is None
            clock[0]+=10_000_000
            c.feed(status(2,turn["wire_sequence"],2,2),clock[0])
            assert c.pending is None
            gyro=next(e for e in fixtures["examples"] if e["name"]=="gyro_ccw")
            g=timed.parse(bytes.fromhex(gyro['hex']));gp=bytearray(g['payload']);gp[:16]=bytes([1])*16
            c.feed(timed.packet(timed.GYRO,g['sequence'],gp),clock[0])
            event=published[-1]
            assert event["kind"]=="timed_gyro_diagnostic" and event["sample"]["capture_time_qualified"] is False
            assert event["sample"]["rate_uncertainty_us"]==0xffffffff
            assert not any(e["kind"]=="measured_body_velocity" for e in published)
            # A changed boot cancels correlation, never automatically replays motion.
            turn=c.turn_ccw(-90,30)
            gp[:16]=bytes([2])*16;c.feed(timed.packet(timed.GYRO,g['sequence'],gp),clock[0])
            req=uart.written[-1];assert timed.parse(req)['type']==timed.ID_REQ
            clock[0]+=1_000_000
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
            assert c.identity_error is None and c.connection_snapshot()['actual_lower_service']=='RESCUE_TASK'
            c.start_timed_measurements();c.poll(clock[0])
            assert any(timed.parse(p)['type']==timed.CONFIG_REQ for p in uart.written if p[:2]==timed.MAGIC)
            try:c.move_body(100,0);assert False,'TASK accepted MotionDebug control'
            except RuntimeError:pass
            c.timed_requested=False
            c._request_identity();req=uart.written[-1];clock[0]+=1_000_000
            c.feed(identity_response(req,epoch=2),clock[0]);assert c.control_identity_problem() is None
            c._request_identity();req=uart.written[-1];clock[0]+=1_000_000
            c.feed(identity_response(req,epoch=2,commit=TIMED_DOCUMENT_COMMIT),clock[0])
            assert c.control_identity_problem() is None  # Reviewed doc-only rebuild, actually reported Git preserved.
            assert c.connection_snapshot()["actual_lower_firmware"]==TIMED_DOCUMENT_COMMIT
            c._request_identity();req=uart.written[-1];clock[0]+=1_000_000
            c.feed(identity_response(req,epoch=2,commit="ff"*20),clock[0])
            assert c.control_identity_problem() is None  # Compatible layout, actual revision retained without a SHA gate.
            assert c.connection_snapshot()["actual_lower_firmware"]=="ff"*20
            before=len(uart.written);c.close();assert len(uart.written)==before+1
            assert timed.parse(uart.written[-1])['type']==timed.CONFIG_REQ  # Unsubscribe only; no motion.
            assert all(timed.parse(p)["type"] in (timed.ID_REQ,timed.SYNC_REQ,timed.CONFIG_REQ)
                       for p in uart.written if p[:2]==timed.MAGIC)
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
    # Legacy explicit helper only; normal connection checks must never call it.
    # Exercise its deletion boundary only in an owned temporary root.
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

    # Default check invocation (without the old --no-log-prune flag) preserves
    # every historical session. No serial port is opened by this fake client.
    import f405_connection_check as check
    fake=SimpleNamespace(log_error=None,timed_stream_active=False,timed_enable_writes=0,
        timed_disable_writes=0,start=lambda:None,close=lambda:None,poll=lambda:None,
        status_snapshot=lambda:({'timestamp_monotonic_ns':1_000_000_000},{}),
        connection_snapshot=lambda:{'motion_identity_ready':True},measurement_snapshot=lambda:{})
    options=SimpleNamespace(uart='fixture-unused',baud=921600,chassis_profile='timed',
        actual_lower_firmware='unconfirmed',actual_lower_mode='unconfirmed',actual_lower_service='unconfirmed')
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);old={}
        for i in range(8):
            directory=root/f'20261101_00000{i}_check';directory.mkdir()
            for name,data in (('metadata.json',dict(owner=CHECK_OWNER)),
                              ('session_complete.json',dict(owner=CHECK_OWNER,complete=True,session_index=i))):
                file=directory/name;file.write_text(json.dumps(data));old[file]=file.read_bytes()
        with patch.object(check,'ChassisTestClient',return_value=fake), \
                patch.object(check,'prune',side_effect=AssertionError('unexpected automatic deletion')) as prune_call, \
                patch.object(check.subprocess,'check_output',side_effect=['fixture-head\n','']), \
                patch.object(check.time,'monotonic',side_effect=[0.,2.]), \
                patch.object(check.time,'monotonic_ns',return_value=1_000_000_000),patch('builtins.print'):
            assert check.check_connection(options,root)==0
        prune_call.assert_not_called()
        assert all(file.read_bytes()==data for file,data in old.items())
        runs=[directory for directory in root.iterdir() if directory.name not in {file.parent.name for file in old}]
        assert len(runs)==1
        result=json.loads((runs[0]/'result.json').read_text())
        assert result['old_check_sessions_removed']==[]
        assert result['log_retention']['automatic_deletion'] is False
        marker=json.loads((runs[0]/'session_complete.json').read_text())
        assert marker['complete'] is True and marker['owner']==CHECK_OWNER


def test_921600_100hz_feedback():
    """Existing 1.3 frames, 100 distinct 10 ms intervals -> SDK-owner IPC.
    Memory UART and socket sink only; not a physical baud/rate or SDK test.
    """
    import math, socket, struct, uuid
    from unittest.mock import patch
    import f405_timed_protocol as wire
    from f405_test_client import ChassisTestClient
    from t265_f407_motion_map import DEFAULT_CONFIG, config_values
    from t265_wheel_input import WheelInputBridge

    examples=json.loads((Path(__file__).with_name('contracts')/'f405_timed_examples_20261031.json').read_text())['examples']
    fixture=bytes.fromhex(next(x['hex'] for x in examples if x['name']=='c_module_host_fixture_type_2'))
    class Memory:
        def __init__(self):self.written=[]
        def write(self,data):self.written.append(data);return len(data)
        def close(self):pass
    with tempfile.TemporaryDirectory() as tmp:
        root=Path(tmp);sid=uuid.uuid4().hex
        sink=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);sink.bind('\0t265-wheel-'+sid);sink.settimeout(.2)
        bridge=WheelInputBridge(root,config_values(DEFAULT_CONFIG),sid,True)
        client=ChassisTestClient('MEMORY',921600,root/'status.json',root/'status.jsonl',identity_policy='timed')
        client.serial=Memory();client.telemetry(bridge.telemetry,readonly=True)
        clock=[10_000_000_000]
        try:
            with patch('f405_test_client.time.monotonic_ns',side_effect=lambda:clock[0]):
                client._request_identity();req=wire.parse(client.serial.written[-1]);identity=bytearray(wire.parse(fixture)['payload'])
                identity[:8]=req['payload'];identity[42:44]=bytes((1,3));identity[48:52]=(921600).to_bytes(4,'big')
                identity[114:134]=bytes.fromhex('aa'*20);epoch=bytes(identity[10:26])
                clock[0]+=1000;client.feed(wire.packet(wire.ID_REPLY,req['sequence'],identity),clock[0])
                assert client.control_identity_problem() is None
                assert client.connection_snapshot()['actual_lower_firmware']=='aa'*20
                client.start_timed_measurements();client.poll(clock[0]+2000)
                config=next(wire.parse(p) for p in client.serial.written if wire.parse(p)['type']==wire.CONFIG_REQ)
                nonce=int.from_bytes(config['payload'][:8],'big')
                reply=struct.pack('>Q16sIHBBIIII',nonce,epoch,1,0,1,1,921600,10000,10000,921600)
                client.feed(wire.packet(wire.CONFIG_REPLY,config['sequence'],reply),clock[0]+3000)
                assert client.measurement_snapshot()['stream_active']
                speed=round(math.pi*.070/1061.268*20/(math.sqrt(3)*.01)*1e6)
                for i in range(101):
                    tick=1_000_000+i*10000
                    clock[0]+=10_000_000
                    gyro=struct.pack('>16sIIqQIIIiQIIIQ',epoch,1,i,0,tick,i+1,0x347,0xffffffff,
                                     0,tick,i+1,0x347,0xffffffff,tick)
                    client.feed(wire.packet(wire.GYRO,2*i,gyro),clock[0])
                    if i:
                        wheel=struct.pack('>16sIIQQiiIIIiiiIII',epoch,1,i,tick-10000,tick,speed,0,
                            i*10,(-i*10)&0xffffffff,0,10,-10,0,0x81,0xffffffff,1)
                        client.feed(wire.packet(wire.WHEEL,2*i+1,wheel),clock[0])
                        bridge.poll();packet=json.loads(sink.recv(16384))
                        assert packet['sample_id']==i and packet['alignment_end_ns']-packet['alignment_start_ns']==10_000_000
                        assert packet['v_body_mps']==[speed/1e6,0.,0.] and packet['omega_body_rad_s']==[0.,0.,0.]
                        assert packet['capture_time_qualified'] is False
                assert bridge.stats['sent_to_sdk_owner']==100
                # This is 100 distinct source intervals, not 100 repeats of one measurement.
                assert bridge.next_id==100 and not bridge.stats.get('duplicate_or_out_of_order_wheel_interval',0)
        finally:
            client.close();bridge.close();sink.close()


def test_global_wall_initialization():
    import math
    import numpy as np
    from lidar_yaw_debug.dynamic import DynamicFence,rotation
    from lidar_yaw_debug.diagnostic import load_config, fixed_yaw_position, reacquire_fixed_yaw_position
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
            # A displacement missing from wheel feedback can leave a remote XY
            # prior. Wall reacquisition keeps field yaw and does not need it.
            body=np.array([p.point_lidar_m for p in cloud(xy,theta,4).points])@rotation(ext[2]).T+ext[:2]
            local=fixed_yaw_position(body,np.array(xy)+[.35,.4],theta,[0,0,0])
            assert local['rank_xy']==0
            recovered=reacquire_fixed_yaw_position(body,theta)
            assert recovered['rank_xy']==2 and not recovered['yaw_reference_changed']
            assert np.linalg.norm(np.array(recovered['candidate_center_xy_m'])-xy)<.001
            # Geometrically incompatible orientation is not repaired by rotating
            # the fixed yaw; a single wall cannot fabricate the other axis.
            assert reacquire_fixed_yaw_position(body,theta+math.radians(25))['rank_xy']==0
            single=np.array([p.point_lidar_m for p in cloud(xy,theta,5,one_wall=True).points])@rotation(ext[2]).T+ext[:2]
            assert reacquire_fixed_yaw_position(single,theta)['rank_xy']==0
            # Exercise the runtime caller with an unmeasured physical push:
            # no new wheel/Pose translation, unchanged gyro reference.
            reference=dict(e.reference);shifted=np.array(xy)+[.35,.4]
            first=cloud(shifted,theta,6);waiting=e.observe(first)
            assert waiting['reacquisition_status']=='CONFIRMING_ENCLOSURE'
            assert not waiting['position_reacquired']
            duplicate=e.observe(first)
            assert duplicate['duplicate_scan'] and e.reacquisition_count==0
            recovered=e.observe(cloud(shifted,theta,7))
            assert recovered['position_reacquired'] and e.reacquisition_count==1
            assert e.reference==reference  # No yaw/90-degree branch reset.
            target=rotation(-quadrant*math.pi/2)@shifted
            # Startup itself estimates yaw to <0.1deg, so this end-to-end bound
            # includes its millimetre-scale geometric error at the new position.
            assert np.linalg.norm(np.array(e.snapshot()['center_xy_m'])-target)<.005
            assert np.linalg.norm(np.array(e.snapshot()['t265_center_xy_m'])-expected)<.001
            # Motion is removed before comparing candidate offsets. Two scan
            # centres may move >5cm while their map offset remains consistent.
            e.offset[:]=0;e.pending_reacquisition=None
            ref=np.array([0.,0.,theta]);offset=np.array(xy)+[.35,.4]
            first=e._fit_position(body,ref,offset,'moving-a')
            assert first['reacquisition_status']=='CONFIRMING_ENCLOSURE'
            moved=np.array([.12,0.]);body2=body-moved@rotation(theta)
            second=e._fit_position(body2,np.r_[moved,theta],offset,'moving-b')
            assert second['position_reacquired']
            e._fit_position(body,ref,offset,'conflict-a')
            conflict=e._fit_position(body2,ref,offset,'conflict-b')
            assert not conflict['position_reacquired']  # Incompatible offsets.
            e.set_lidar_enabled(False);assert e.pending_reacquisition is None
            e.set_lidar_enabled(True)
            assert not e._fit_position(body2,ref,offset,'after-obstacle')['position_reacquired']
    # The isolated numerical job has no pending confirmations or coordinate
    # branch of its own; the original engine must make identical decisions.
    from lidar_yaw_debug.scan_geometry import geometry_job
    plain=DynamicFence(mount);isolated=DynamicFence(mount,geometry=geometry_job)
    for target in (plain,isolated):
        for i in range(101):
            t=origin-200_000_000+i*5_000_000
            target.ingest_pose(dict(session_id='geometry-parity',frame_number=i,timestamp_monotonic_ns=t,
                receive_monotonic_ns=t,x_m=0.,y_m=0.,yaw_rad=0.,tracker_confidence=3))
            target.ingest_yaw(t,1287.,dict(source='synthetic_gyro'))
        target.request_initialization('same numerical fixture',physical_confirmed=False)
    timing={'started_ns','timestamp_monotonic_ns','compute_ms','compute_thread_cpu_ms',
            'snapshot_ms','deskew_ms','fit_ms','geometry_compute_cpu_ms','geometry_roundtrip_ms'}
    def numerical(value):
        if isinstance(value,dict):return {k:numerical(v) for k,v in value.items() if k not in timing}
        if isinstance(value,np.ndarray):return numerical(value.tolist())
        if isinstance(value,(list,tuple)):return [numerical(v) for v in value]
        return value
    angle=math.radians(13.)
    for i in range(3):
        scan=cloud((.4,-.3),angle,i)
        assert numerical(plain.try_initialize(scan))==numerical(isolated.try_initialize(scan))
    # The native high-rate owner supplies full RAW initialization history,
    # rather than its two-endpoint display view or selected motion's zero yaw.
    from lidar_yaw_debug.native_motion import NativeDynamicFence,NativeMotionRuntime
    runtime=NativeMotionRuntime(dict(feedback_fd=-1,pose_fd=-1))
    try:
        native=NativeDynamicFence(mount,runtime)
        for i in range(101):
            t=origin-200_000_000+i*5_000_000
            native.ingest_pose(dict(session_id='geometry-parity',frame_number=i,timestamp_monotonic_ns=t,
                receive_monotonic_ns=t,x_m=0.,y_m=0.,yaw_rad=0.,tracker_confidence=3))
            native.ingest_yaw(t,1287.,dict(source='synthetic_gyro'))
        native.request_initialization('same numerical fixture',physical_confirmed=False)
        assert len(native.poses)==2  # Low-rate facade view, not fitting support.
        with native.lock:
            raw_poses,raw_yaws=native._initial_support(cloud((.4,-.3),angle,0))
        assert len(raw_poses)==len(raw_yaws)==101
        assert np.array_equal(raw_poses,np.asarray(plain.poses))
        assert np.allclose(raw_yaws,np.asarray(plain.yaws),rtol=1e-12,atol=1e-10)
        for i in range(3):
            selected=native.try_initialize(cloud((.4,-.3),angle,i))
            assert selected['confirmations']==i+1
            assert selected['status']==('GLOBAL_SYMMETRIC_FIX' if i==2 else 'CONFIRMING_WALL_MATCH')
            if i<2:assert native.reference is None
        assert runtime.snapshot()['reference']==native.reference
        for key in ('pose_origin','vio_rotation','yaw_offset','start_center_xy_m','field_heading_deg'):
            assert np.allclose(native.reference[key],plain.reference[key],rtol=1e-12,atol=1e-10),key
        for key in ('source','branch_policy','reference_sample_ns','square_absolute_side_identified'):
            assert native.reference[key]==plain.reference[key],key
        assert native.has_anchor and runtime.snapshot()['has_anchor']
    finally:runtime.close()
    for scan in (cloud((.4,-.3),angle,3),cloud((.4,-.3),angle,4,one_wall=True),
                 cloud((.75,.1),angle,5),cloud((.75,.1),angle,6)):
        assert numerical(plain.observe(scan))==numerical(isolated.observe(scan))
        assert numerical(plain.snapshot(origin+5_000_000_000))==numerical(isolated.snapshot(origin+5_000_000_000))
    assert plain.reacquisition_count==isolated.reacquisition_count==1
    assert numerical(plain.observe(scan))==numerical(isolated.observe(scan))  # duplicate does not confirm again
    for target in (plain,isolated):target.set_lidar_enabled(False)
    scan=cloud((.75,.1),angle,7)
    assert numerical(plain.observe(scan))==numerical(isolated.observe(scan))
    # An insufficient cloud clears the real engine's pending reacquisition,
    # including the callback path; no numerical child can own that state.
    small=RawScan('too-few',scan.points[3].time_ns,scan.points[:4],'fixture','known_synthetic')
    for target in (plain,isolated):
        target.pending_reacquisition=dict(scan_id='pending',offset=np.ones(2))
        try:target.observe(small)
        except ValueError as error:assert str(error)=='可用雷达点不足'
        else:raise AssertionError('insufficient scan accepted')
        assert target.pending_reacquisition is None
    e.request_initialization('single wall must not locate XY',physical_confirmed=False)
    for i in range(3):e.try_initialize(cloud((.4,-.3),theta,i,one_wall=True))
    assert e.reference is None
    # Slow initialization must not block real motion ingestion or a new field
    # bind. Nor may its third confirmation overwrite that newly bound frame.
    import threading
    from unittest.mock import patch
    import lidar_yaw_debug.dynamic as dynamic
    e.request_initialization('concurrent initialization fixture',physical_confirmed=False)
    for i in range(2):e.try_initialize(cloud((.4,-.3),theta,20+i))
    assert e.initialization['confirmations']==2
    entered=threading.Event();release=threading.Event();updated=threading.Event()
    original_initialize=dynamic.initialize;errors=[];bound=[]
    def slow_initialize(*args,**kwargs):
        entered.set()
        assert release.wait(5),'fixture did not release initialization'
        return original_initialize(*args,**kwargs)
    def finish_initialization():
        try:e.try_initialize(cloud((.4,-.3),theta,22),expected_revision=e.revision)
        except Exception as exc:errors.append(exc)
    def update_motion_and_bind():
        try:
            t=origin+500_000_000
            e.ingest_pose(dict(session_id='fixture',frame_number=101,timestamp_monotonic_ns=t,
                receive_monotonic_ns=t,x_m=.01,y_m=.02,yaw_rad=0.,tracker_confidence=3))
            e.ingest_yaw(t,1287.,dict(source='synthetic_gyro'))
            bound.append(e.align('operator rebound while geometry runs',physical_confirmed=False,
                center_xy=(.2,-.4),heading_deg=42.))
            updated.set()
        except Exception as exc:errors.append(exc)
    with patch.object(dynamic,'initialize',slow_initialize):
        fitting=threading.Thread(target=finish_initialization);motion=None
        try:
            fitting.start();assert entered.wait(2),'initialization did not start'
            motion=threading.Thread(target=update_motion_and_bind);motion.start()
            assert updated.wait(2),'initialization still holds the motion lock'
            assert e.poses[-1][0]==origin+500_000_000
        finally:
            release.set();fitting.join(timeout=5)
            if motion is not None:motion.join(timeout=5)
    assert not fitting.is_alive() and motion is not None and not motion.is_alive()
    assert len(errors)==1 and isinstance(errors[0],ValueError) and 'reference changed' in str(errors[0]),errors
    assert e.reference==bound[0] and e.reference['field_heading_deg']==42.


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
            # PTY bytes sent before serial.open are flushed by pyserial. Wait
            # for the fixture receiver, not a hardware qualification exchange.
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and not worker.state.get('lidar_connected'):
                worker.poll();time.sleep(.01)
            assert worker.state.get('lidar_connected'),worker.state
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
            assert worker.state.get('scans_received',0)>=2 and worker.state.get('capture_chain_present'),worker.state
        finally:
            worker.close();os.close(master);os.close(slave)
        assert worker.stopped_cleanly and not worker.process.is_alive()
        assert worker.state['logging_healthy'] and (root/'lidar_worker_summary.json').exists()
        assert not (root/'commands.jsonl').exists()

    # Reuse this lifecycle fixture: a suspended numerical child cannot be
    # reaped by SIGTERM alone. The existing parent deadline must reclaim the
    # complete worker group without touching the caller's group or hardware.
    import signal
    master,slave=pty.openpty();worker=RadarWorker();child=None
    with tempfile.TemporaryDirectory() as tmp:
        try:
            worker.start(cfg,os.ttyname(slave),230400,Path(tmp))
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and not worker.state.get('geometry_pid'):
                worker.poll();time.sleep(.01)
            child=worker.state['geometry_pid']
            # This X5 kernel omits /proc/PID/task/PID/children. Use the
            # standard PPID field, and tolerate unrelated processes exiting.
            children=[]
            for path in Path('/proc').glob('[0-9]*/stat'):
                try:fields=path.read_text().split(') ',1)[1].split()
                except (OSError,IndexError):continue
                if int(fields[1])==worker.process.pid:children.append(int(path.parent.name))
            assert os.getpgid(worker.process.pid)==worker.process.pid
            assert os.getpgid(child)==worker.process.pid!=os.getpgrp()
            assert child in children and all(os.getpgid(pid)==worker.process.pid for pid in children)
            os.kill(child,signal.SIGSTOP)
            began=time.monotonic()
            worker.close()
            assert time.monotonic()-began<6.5
            assert not worker.process.is_alive() and not worker.stopped_cleanly
            assert worker.state['error']=='雷达进程未能正常退出'
            deadline=time.monotonic()+1
            def running(pid):
                try:return not Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].startswith('Z')
                except FileNotFoundError:return False
            while any(running(pid) for pid in children) and time.monotonic()<deadline:
                time.sleep(.01)
            assert not any(running(pid) for pid in children)
            assert not (Path(tmp)/'commands.jsonl').exists()
        finally:
            if worker.process is not None and worker.process.is_alive():worker.close()
            os.close(master);os.close(slave)


def test_dynamic_motion_compensation():
    import math
    import numpy as np
    from lidar_yaw_debug.dynamic import DynamicFence,rotation,deskew_points
    from lidar_yaw_debug.types import RawScan,TimedPoint
    from lidar_yaw_debug.diagnostic import load_config
    mount=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')['installation']
    # Preserve physical point identity and exact half-open bad-angle/FOV
    # boundaries when reducing six per-point interval scans to binary lookup.
    filter_engine=DynamicFence(dict(mount))
    actual_ranges=mount['excluded_raw_ranges_deg']
    changed_ranges=[[210.,215.],[150.,170.],[170.,180.],[160.,175.],
                    [220.,220.],[230.,225.],[215.,217.]]
    edges=[mount['center_raw_deg']-mount['fov_deg']/2,
           mount['center_raw_deg']+mount['fov_deg']/2]
    edges += [edge for ranges in (actual_ranges,changed_ranges) for pair in ranges for edge in pair]
    angles=[float(v) for v in np.linspace(-360.,720.,200)]
    angles += [edge+delta for edge in edges for delta in (-1e-12,0.,1e-12)]
    angles += [None,float('nan'),float('inf'),-float('inf')]
    filter_points=tuple(TimedPoint('filter-'+str(i),i+1,(1.,0.),raw_angle_deg=angle,
        valid_return=valid) for i,(angle,valid) in enumerate(
            [(angle,True) for angle in angles]+[(edges[0],False),(mount['center_raw_deg'],False)]))
    filter_scan=RawScan('filter-boundaries',filter_points[-1].time_ns,filter_points,
                        'offline','synthetic',False)
    # Mutate the same live mount between scans: an empty/changed mask must not
    # retain cached merged intervals. Reversed and zero-width intervals were
    # empty in the original predicate and remain empty.
    for ranges in (actual_ranges,[],changed_ranges,list(reversed(changed_ranges)),actual_ranges):
        filter_engine.mount['excluded_raw_ranges_deg']=ranges
        expected=[point for point in filter_points if point.valid_return and point.raw_angle_deg is not None
            and abs((point.raw_angle_deg-mount['center_raw_deg']+180)%360-180)<=mount['fov_deg']/2
            and not any(lo<=point.raw_angle_deg<hi for lo,hi in ranges)]
        selected=filter_engine.accepted_points(filter_scan)
        assert [point.raw_point_id for point in selected]==[point.raw_point_id for point in expected]
        assert all(a is b for a,b in zip(selected,expected))
    from lidar_yaw_debug.scan_geometry import geometry_job
    engine=DynamicFence(mount);isolated=DynamicFence(mount,geometry=geometry_job);origin=1_000_000_000
    def pose(t):return np.array([-1.35+.7*t/math.sqrt(2),1.35-.7*t/math.sqrt(2),math.radians(135)+2*math.pi*t])
    for i in range(41):
        t=i*.005;p=pose(t);ns=origin+round(t*1e9)
        for target in (engine,isolated):
            target.ingest_pose(dict(session_id='offline',frame_number=i,timestamp_monotonic_ns=ns,
                receive_monotonic_ns=ns,x_m=p[0],y_m=p[1],yaw_rad=p[2],tracker_confidence=3))
            target.ingest_yaw(ns,360*t,dict(source='independent_synthetic_motion'))
            if i==0:target.align('offline known trajectory',physical_confirmed=False)
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
    offloaded=isolated.observe(scan)
    for key in ('rank_xy','position_observed_axes','candidate_center_xy_m','used_points',
                'prior_center_xy_m','map_translation_offset_m','motion_selection','primary_yaw_field_rad'):
        assert offloaded[key]==result[key],key
    assert isolated.latest_points==engine.latest_points
    assert offloaded['geometry_compute_cpu_ms']>=0 and offloaded['geometry_roundtrip_ms']>=0
    assert result['rank_xy']>0 and result['deskew_applied'] and not result['deskew_qualified']
    assert np.linalg.norm(np.array(result['candidate_center_xy_m'])-pose(.16)[:2])<1e-6
    dominant=max(set(walls),key=walls.count);single=[p for p,w in zip(points,walls) if w==dominant]
    single_scan=RawScan('single',single[-1].time_ns,tuple(single),'offline','known_synthetic',False)
    one=engine.observe(single_scan);assert one['rank_xy']==1
    offloaded_single=isolated.observe(single_scan)
    assert offloaded_single['position_observed_axes']==one['position_observed_axes']
    assert offloaded_single['candidate_center_xy_m']==one['candidate_center_xy_m']
    assert np.array_equal(isolated.offset,engine.offset)
    # The native high-rate owner must also preserve this original geometric
    # consumer, including actual field bind, raw yaw and single-axis posterior.
    from lidar_yaw_debug.native_motion import NativeMotionRuntime,NativeDynamicFence
    native_runtime=NativeMotionRuntime(dict(feedback_fd=-1,pose_fd=-1))
    try:
        native=NativeDynamicFence(mount,native_runtime)
        for i in range(41):
            t=i*.005;p=pose(t);ns=origin+round(t*1e9)
            native.ingest_pose(dict(session_id='offline',frame_number=i,timestamp_monotonic_ns=ns,
                receive_monotonic_ns=ns,x_m=p[0],y_m=p[1],yaw_rad=p[2],tracker_confidence=3))
            native.ingest_yaw(ns,360*t,dict(source='independent_synthetic_motion'))
            if i==0:native.align('offline known trajectory',physical_confirmed=False)
        selected=native.observe(scan)
        for key in ('rank_xy','position_observed_axes','used_points','motion_source'):
            assert selected[key]==result[key],key
        for key in ('candidate_center_xy_m','prior_center_xy_m','map_translation_offset_m'):
            assert np.allclose(selected[key],result[key],rtol=1e-12,atol=1e-10),key
        native_single=native.observe(single_scan)
        assert native_single['position_observed_axes']==one['position_observed_axes']
        assert np.allclose(native.offset,engine.offset,rtol=1e-12,atol=1e-10)
        assert np.allclose(native.control_snapshot()['center_xy_m'],engine.control_snapshot()['center_xy_m'],
                           rtol=1e-12,atol=1e-10)
        # Identity invalidation must also discard the Python geometry-side
        # initialization; the native owner performs exactly one revision reset.
        revision=native.revision
        native.initialization=dict(status='CONFIRMING_WALL_MATCH',confirmations=2)
        native.pending_initial_pose=np.ones(3)
        native.pending_reacquisition=dict(confirmations=1)
        native.context(dict(kind='device_identity_changed',sample={}))
        assert native.reference is None and native.revision==revision+1
        assert native.initialization is None and native.pending_initial_pose is None
        assert native.pending_reacquisition is None and native.last_fit is None
        assert not native.has_anchor and not native.yaws
        assert native.try_initialize(scan)['status']=='INITIALIZATION_NOT_REQUESTED'
    finally:native_runtime.close()
    assert len(one['position_observed_axes'])==1 and len(one['inherited_axes'])==1
    old=engine.snapshot(origin+5_000_000_000)
    assert old['ready'] and old['motion_age_ms']>4000  # No age admission gate.
    compact=engine.control_snapshot(origin+5_000_000_000)
    expected={k:v for k,v in old.items() if k not in ('last_fit','points_field')}
    expected['motion_selection']=dict(expected['motion_selection'])
    expected['motion_selection'].pop('wall_motion_evidence')  # Display-only witness, not high-rate control data.
    assert compact==expected
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


def test_dynamic_motion_source():
    """Actual-interval math with fake feedback; no UART, SDK or ground accuracy claim."""
    import numpy as np
    from lidar_yaw_debug.motion_source import MotionSource, sample
    from collections import deque
    m=MotionSource();origin=1_000_000_000;yaws=deque()
    m.configure(dict(pose_origin=[0.,0.],vio_rotation=0.,yaw_offset=0.,reference_sample_ns=origin))
    def pose(i,x,confidence=3):
        m.pose(dict(timestamp_monotonic_ns=origin+i*20_000_000,x_m=x,y_m=0.,tracker_confidence=confidence))
    def step(i,velocity=.5,yaw=0.):
        t=origin+i*20_000_000;yaws.append((t,yaw))
        m.yaw(t,yaw,{'raw':{'yaw_receive_us':t//1000}})
        m.wheel(dict(source='measured_wheel_feedback',measurement_valid=True,mcu_epoch='boot',sample_id=i,
            raw_mcu_sample=dict(t0_us=(t-20_000_000)//1000,t1_us=t//1000),v_body_mps=[velocity,0.,0.],
            interval_start_ns=t-20_000_000,interval_end_ns=t,receive_monotonic_ns=t+9_000_000_000),yaws)
    pose(0,0.)
    for i in range(1,11):step(i);pose(i,i*.01)
    assert m.mode=='T265_WITH_WHEEL_TAIL' and abs(m.history[-1][1]-.1)<1e-10
    step(11);pose(11,2.11)
    assert m.mode=='WHEEL_FALLBACK' and abs(m.history[-1][1]-.11)<1e-10
    for i in range(12,80):
        step(i);pose(i,2+i*.01);m.observe_walls(str(i),['x'])
    assert m.mode=='WHEEL_FALLBACK'  # A single wall cannot authorize XY recovery.
    before=m.history[-1][1]
    m.observe_walls('both1',['x','y']);m.observe_walls('both2',['x','y'])
    step(80);pose(80,2.8)
    assert m.mode=='T265_WITH_WHEEL_TAIL' and abs(m.bias[0]+2)<1e-10
    assert abs(m.history[-1][1]-before-.01)<1e-10  # Recovery rebase, not a 2m snap.
    pose(81,2.81,0);step(81)
    assert m.mode=='WHEEL_FALLBACK'
    for i in range(82,90):step(i)  # No Pose, but genuine wheel intervals keep moving.
    assert abs(m.history[-1][1]-.89)<1e-10
    assert np.allclose(sample(m.history,10**15)[:2],[.89,0.])  # No unbounded last-pose slope.
    # Source switches never mix target velocity into wheel propagation.
    count=len(m.wheels);m.wheel(dict(source='command_target'),yaws);assert len(m.wheels)==count
    assert any(e['motion_source']=='WHEEL_FALLBACK' for e in m.events)
    # Nonzero yaw/field reference: B-left remains left, no second camera transform.
    other=MotionSource();other.configure(dict(pose_origin=[0.,0.],vio_rotation=.3,yaw_offset=2.1,reference_sample_ns=origin))
    other.pose(dict(timestamp_monotonic_ns=origin,x_m=0.,y_m=0.,tracker_confidence=3))
    for t in (origin,origin+20_000_000):other.yaw(t,.7,{'raw':{'yaw_receive_us':t//1000}})
    other.wheel(dict(source='measured_wheel_feedback',measurement_valid=True,mcu_epoch='boot',sample_id=1,
        raw_mcu_sample=dict(t0_us=origin//1000,t1_us=(origin+20_000_000)//1000),v_body_mps=[0.,1.,0.],
        interval_start_ns=origin,interval_end_ns=origin+20_000_000,receive_monotonic_ns=origin+20_000_000),deque([(origin,.7)]))
    assert np.allclose(other.history[-1][1:3],.02*np.array([-np.sin(2.5),np.cos(2.5)]))
    other.configure(dict(pose_origin=[0.,0.],vio_rotation=.3,yaw_offset=2.1,reference_sample_ns=origin))
    other.pose(dict(timestamp_monotonic_ns=origin+20_000_000,x_m=2.,y_m=0.,tracker_confidence=3))
    assert other.mode=='WHEEL_FALLBACK'  # First post-reference Pose cannot silently move the origin.


def test_historical_wall_correction_rebases_updated_endpoint():
    """Known offline posterior + a real late-Pose update during mocked fitting.

    Tests application of a geometric result, not sensor/ground accuracy.
    Cover delayed completion, later propagation, nonidentity field rotation
    and one-wall observability without changing the geometry/source policy.
    """
    import math
    from types import MethodType
    import numpy as np
    from lidar_yaw_debug.dynamic import DynamicFence,rotation
    from lidar_yaw_debug.types import RawScan,TimedPoint
    from lidar_yaw_debug.diagnostic import load_config
    mount=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')['installation']
    for angle in (0.,.7):
        for axes in (['x','y'],['x']):
            e=DynamicFence(mount);a=1_000_000_000;b=a+100_000_000
            p=dict(session_id='offline-history',frame_number=1,timestamp_monotonic_ns=a,
                receive_monotonic_ns=a,x_m=0.,y_m=0.,yaw_rad=0.,tracker_confidence=3)
            e.ingest_pose(p);e.ingest_yaw(a,0.,{});e.ingest_yaw(b,0.,{})
            e.align('offline known reference',physical_confirmed=False,center_xy=(.2,-.3),heading_deg=math.degrees(angle))
            def wheel(i,start,end):
                e.ingest_wheel(dict(source='measured_wheel_feedback',measurement_valid=True,
                    mcu_epoch='fixture',sample_id=i,v_body_mps=[.1,0.,0.],
                    raw_mcu_sample=dict(t0_us=start//1000,t1_us=end//1000),
                    interval_start_ns=start,interval_end_ns=end,receive_monotonic_ns=end))
            wheel(1,a,b)
            candidate=np.array([.2,-.3])+rotation(angle)@np.array([.01,0.])
            def fit(self,points,ref,offset,scan_id,revision=None):
                # Refine the latest history row at b with a Pose captured at
                # b-10ms. It is within the unchanged .05m conflict policy.
                self.ingest_pose(dict(p,frame_number=2,timestamp_monotonic_ns=b-10_000_000,
                    receive_monotonic_ns=b+1,x_m=.049,y_m=.02))
                # Then actual feedback advances beyond the scan reference.
                self.ingest_yaw(b+100_000_000,0.,{})
                wheel(2,b,b+100_000_000)
                return dict(rank_xy=len(axes),position_observed_axes=axes,
                    inherited_axes=[axis for axis in ('x','y') if axis not in axes],
                    candidate_center_xy_m=candidate.tolist(),position_reacquired=False,
                    reason='known_offline_wall_result')
            e._fit_position=MethodType(fit,e)
            scan=RawScan('history-rebase',b,tuple(TimedPoint(str(i),b-(7-i)*1_000_000,
                (1.,0.),raw_angle_deg=215.75) for i in range(8)),'offline','known_synthetic',False)
            result=e.observe(scan);state=e.snapshot()
            final=np.array(state['center_xy_m']);tail=rotation(angle)@np.array([.01,0.])
            corrected_at_scan=final-tail
            for axis in axes:
                i=('x','y').index(axis)
                assert abs(corrected_at_scan[i]-candidate[i])<1e-10
            if axes==['x']:assert e.offset[1]==0.  # A single wall never fixes Y.
            assert np.linalg.norm(result['motion_reference_update_m'])>.04
            assert state['sample_monotonic_ns']==b+100_000_000
            assert state['motion_selection']['mode']=='T265_WITH_WHEEL_TAIL'


def test_dynamic_consumer_starts_before_sdk_and_failed_spawn_closes_it():
    """Existing entry lifecycle only: no SDK, UART, radar or motion is opened."""
    import threading
    from unittest.mock import patch
    import t265_f407_motion_map as base
    import lidar_t265_motion_map as dynamic

    for fail_spawn in (False, True):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with patch('sys.argv', ['map', '--output-root', str(root), '--uart', 'fixture-chassis',
                                   '--lidar', 'fixture-lidar', '--windowed']):
                options = dynamic.arguments()
            order = []
            holder = []
            class FakeRadar:
                def __init__(self):
                    self.process = None
                    self.state = dict(logging_healthy=True)
                    self.input_drops = 0
                    self.stopped_cleanly = False
                def enable_native_feedback(self):
                    self.feedback_address='\0fixture-motion-feedback'
                def send(self, kind, payload):
                    assert kind=='feedback_context'
                    return True
                def start(self, config, port, baud, directory, *, sdk_policy, pose_input):
                    app = holder[0]
                    assert directory == app.run_dir and directory.is_dir()
                    assert (directory / 'localization.conf').is_file()
                    assert sdk_policy['session_id'] == app.wheel_input.session_id
                    assert sdk_policy['address'] == '\0t265-wheel-' + app.wheel_input.session_id
                    assert pose_input['socket'] is app.pose_socket
                    assert pose_input['session_id'] == app.wheel_input.session_id
                    assert app.localizer is None and app.bridge.serial is None
                    order.append('radar_spawn')
                    self.process = SimpleNamespace(pid=987)
                def close(self):
                    order.append('radar_close')
                    self.stopped_cleanly = True
            radar = FakeRadar()
            class FakeThread:
                def __init__(self, *, target, name, daemon):
                    self.name = name
                def start(self): order.append(self.name)
                def join(self, timeout=None): pass
                def is_alive(self): return False
            def spawn(command, **kwargs):
                app = holder[0]
                assert order == ['radar_spawn']
                assert '--debug-pose-socket' in command
                assert command[command.index('--debug-pose-socket') + 1] == '@' + app.pose_address[1:]
                assert command[command.index('--motion-feedback-socket') + 1] == '@fixture-motion-feedback'
                assert command[command.index('--debug-wheel-session') + 1] == app.wheel_input.session_id
                assert not (app.run_dir / 'metadata.json').exists()
                order.append('sdk_spawn')
                if fail_spawn:
                    raise OSError('fixture SDK process creation failure')
                return SimpleNamespace(pid=123, poll=lambda: 0, returncode=0)
            with patch.object(base.MotionMapApp, 'detect_screen_size', return_value=(1024, 768)), \
                 patch.object(dynamic.DynamicMapApp, 'render_screen'), \
                 patch.object(dynamic, 'RadarWorker', return_value=radar), \
                 patch.object(dynamic, 'threading', SimpleNamespace(Thread=FakeThread, Event=threading.Event)), \
                 patch.object(base.F407SerialBridge, 'start', side_effect=lambda: order.append('bridge_start')), \
                 patch.object(base.subprocess, 'Popen', side_effect=spawn), \
                 patch.object(base.subprocess, 'check_output', return_value='offline'), \
                 patch('low_latency_scheduling.prepare_realtime', return_value=dict(requested=False)), \
                 patch.object(base.cv2, 'destroyWindow'):
                app = dynamic.DynamicMapApp(options)
                holder.append(app)
                if fail_spawn:
                    try:
                        app.run()  # The original run/finally must close the early consumer.
                        assert False, 'SDK process creation failure was hidden'
                    except OSError as error:
                        assert str(error) == 'fixture SDK process creation failure'
                    assert order == ['radar_spawn', 'sdk_spawn', 'radar_close']
                    assert app.pose_thread is None and app.yaw_thread is None
                else:
                    try:
                        app.start_session()
                        assert order == ['radar_spawn', 'sdk_spawn', 'bridge_start',
                                         'dynamic-yaw-forwarder']
                        assert app.pose_thread is None  # The worker is the sole SDK Pose reader.
                        metadata = json.loads((app.run_dir / 'metadata.json').read_text())
                        assert metadata['lidar_worker_pid'] == 987
                        assert metadata['sdk_session_id'] == app.wheel_input.session_id
                        assert metadata['control_pose_published'] is False
                    finally:
                        app.stop_session()
                assert order[-1] == 'radar_close' and radar.stopped_cleanly
                assert app.localizer is None and app.localizer_log is None
                assert app.bridge is None and app.wheel_input is None
                assert app.dynamic_log.healthy() and not app.dynamic_log.thread.is_alive()
                assert app.pose_socket.fileno() == -1
                assert not (app.run_dir / 'commands.jsonl').exists()
                assert not (app.run_dir / 'chassis_actions.jsonl').exists()
                assert not (app.run_dir / 'session_complete.json').exists()


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
        b=SyntheticWheelInputBridge(root,config_values(DEFAULT_CONFIG),uuid.uuid4().hex,True,plan='pause-resume')
        try:
            pauses=[p for p in b.PHASES if p[2] is None]
            assert len(pauses)==3 and all(p[3] is None for p in pauses)
            assert any(p[0]=='resume_zero' and p[2]==[0.,0.,0.] for p in b.PHASES)
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
            assert c._identity_problem(dict(identity,git_commit='ff'*20)) is None
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
            # Logging failure must neither block identity refresh nor cause a
            # retry storm. A real reply clears the earlier incompatible identity.
            c.log_error='fixture log failure';c.next_identity_query_ns=0
            for _ in range(1000):c.poll(now[0])
            assert c.next_identity_query_ns>now[0] and len(c.serial.written)==before+1
            req=wire.parse(c.serial.written[-1]);p[:8]=req['payload'];p[8:10]=(1).to_bytes(2,'big')
            now[0]+=1_000_000;c.feed(wire.packet(wire.ID_REPLY,req['sequence'],p),now[0])
            assert c.control_identity_problem() is None and c.log_error
            c.latest_status=dict(state=0,command_sequence=250)
            action=c.move_body(500,0)
            assert c.serial.written[-1][:2]==b'\xa3\xb3' and action['name']=='move_body'
            stopped=c.stop()
            assert stopped['name']=='stop' and c.log_error
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
    from async_debug_log import ProcessDebugLog
    import multiprocessing as mp
    import os
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);logger=ProcessDebugLog(root)
        try:
            for i in range(1000):
                assert logger.submit('raw.jsonl',dict(timestamp_monotonic_ns=i))
                assert logger.submit('status.json',dict(timestamp_monotonic_ns=i),atomic=True)
        finally:logger.close()
        assert logger.process.pid!=os.getpid() and not logger.process.is_alive()
        assert logger.healthy() and logger.statistics()['final_receipt']
        assert logger.statistics()['history_records_written']==1000 and logger.dropped==0
        assert logger.statistics()['sender_stopped'] and not logger.thread.is_alive()
        assert [json.loads(line)['timestamp_monotonic_ns'] for line in (root/'raw.jsonl').read_text().splitlines()]==list(range(1000))
        assert json.loads((root/'status.json').read_text())['timestamp_monotonic_ns']==999
    with tempfile.TemporaryDirectory() as temporary:
        release=threading.Event();original_send=ProcessDebugLog._send
        def paused_send(logger):release.wait(2);original_send(logger)
        with patch.object(ProcessDebugLog,'_send',paused_send):
            logger=ProcessDebugLog(Path(temporary),capacity=1)
        try:
            assert logger.submit('full.jsonl',dict(value='accepted'))
            assert not logger.submit('full.jsonl',dict(value=1))
            assert logger.dropped==1 and not logger.healthy()
        finally:release.set();logger.close()
        assert not logger.process.is_alive() and logger.statistics()['transport_dropped']==1
        assert json.loads((Path(temporary)/'full.jsonl').read_text())['value']=='accepted'
    for failure in ('missing-directory','dev-full'):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            if failure=='missing-directory':root=root/'missing'
            else:(root/'raw.jsonl').symlink_to('/dev/full')
            logger=ProcessDebugLog(root)
            assert logger.submit('raw.jsonl',dict(value='x'*131072))
            logger.close()
            assert not logger.healthy() and logger.error
            assert logger.statistics()['final_receipt'] and not logger.process.is_alive()
    with tempfile.TemporaryDirectory() as temporary:
        logger=ProcessDebugLog(Path(temporary))
        logger.process.terminate();logger.process.join(timeout=2)
        logger.close()
        assert not logger.healthy() and logger.error and not logger.process.is_alive()
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary)
        with patch.object(mp.get_context('spawn').Process,'start',side_effect=OSError('fixture spawn failure')):
            logger=ProcessDebugLog(root)
        assert logger.submit('raw.jsonl',dict(value='thread fallback'))
        logger.close()
        assert logger.healthy() and logger.statistics()['execution']=='thread_fallback'
        assert json.loads((root/'raw.jsonl').read_text())['value']=='thread fallback'
        assert json.loads((root/'logging_events.jsonl').read_text())['event']=='process_log_start_failed_thread_fallback'


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
            # Reproduce a bounded history overflow while disk writing stalls.
            for i in range(bridge.log.queue.maxsize+1):
                bridge.log.submit('raw.jsonl',dict(index=i))
            assert not bridge.log.healthy()
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
            summary=bridge.snapshot(dict(session_id=sid,sdk_api_success_samples=1,logging_healthy=False))
            assert summary['state']=='SDK_API_ACCEPTED' and summary['sdk_logging_healthy'] is False
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
            assert c._identity_problem(dict(identity,git_commit='ff'*20)) is None
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
        app.chassis_inbox.put(dict(kind='timed_wheel_diagnostic',sample=dict(wheel,flags=0x85,receive_monotonic_ns=now)))
        app.drain_yaw();assert sent[-1][0]=='wheel'
        assert sent[-1][1]['source']=='measured_wheel_feedback' and sent[-1][1]['measurement_valid']

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


def test_t265_usb_recovery():
    """Fast startup/owner exclusion/failed upload; no real USB operation."""
    from unittest.mock import patch
    import t265_usb_recovery as usb
    devices=[dict(name='usb1',idVendor='1d6b',idProduct='0002',busnum='1',devnum='1'),
             dict(name='1-1',idVendor='05e3',idProduct='0610',busnum='1',devnum='2'),
             dict(name='1-1.1',idVendor='03e7',idProduct='2150',busnum='1',devnum='5'),
             dict(name='1-1.3',idVendor='1bcf',idProduct='28c4',busnum='1',devnum='4')]
    plan=usb.recovery_plan(devices)
    assert (plan['location'],plan['port'])==('1-1',1) and plan['force_known_rdk_ganged']
    for bad in (devices[:2],devices+[dict(devices[2],name='1-1.2')]):
        try:usb.recovery_plan(bad);assert False,'must not guess absent/ambiguous camera port'
        except RuntimeError:pass
    after=[dict(d,idVendor='8087',idProduct='0b37',devnum='6') if d['name']=='1-1.1' else d for d in devices]
    with tempfile.TemporaryDirectory() as temporary:
        commands=[]
        def fake_run(args,**kwargs):
            commands.append(args)
            return SimpleNamespace(returncode=1 if 'fuser' in args[0] else 0,stdout='fixture',stderr='')
        with patch.object(usb,'usb_inventory',side_effect=[devices,after]), \
             patch.object(usb.os,'geteuid',return_value=0),patch.object(usb.os,'access',return_value=True), \
             patch.object(usb.shutil,'which',side_effect=lambda name:'/fixture/'+name), \
             patch.object(usb.subprocess,'run',side_effect=fake_run):
            result=usb.recover_t265(Path(temporary)/'t265_usb_recovery.json')
        assert len(commands)==2 and commands[-1][-4:]==['--timeout-ms','1000','--chunk-kib','256']
        assert result['command_completed'] and result['runtime_enumerated']
        assert not result['forced_power_cycle'] and result['fixed_wait_seconds']==0
        assert not result['vbus_power_off_verified'] and not result['sdk_pose_verified']
        assert result['after'][-1]==devices[-1]
        commands.clear()
        with patch.object(usb,'usb_inventory',return_value=after), \
             patch.object(usb.os,'geteuid',return_value=0), \
             patch.object(usb.shutil,'which',side_effect=lambda name:'/fixture/'+name), \
             patch.object(usb.subprocess,'run',side_effect=fake_run):
            healthy=usb.recover_t265(Path(temporary)/'runtime.json')
        assert len(commands)==1 and healthy['action']=='runtime_pid_skip_reset' and not healthy['sdk_pose_verified']
        with patch.object(usb,'usb_inventory',return_value=devices), \
             patch.object(usb.os,'geteuid',return_value=0), \
             patch.object(usb.shutil,'which',side_effect=lambda name:'/fixture/'+name), \
             patch.object(usb.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout='1234',stderr='')) as busy:
            try:usb.recover_t265(Path(temporary)/'busy.json');assert False,'active SDK must not be reset'
            except RuntimeError:pass
            assert busy.call_count==1 and busy.call_args.args[0][0]=='/fixture/fuser'
        commands.clear()
        def failed_upload(args,**kwargs):
            commands.append(args)
            return SimpleNamespace(returncode=1 if 'fuser' in args[0] else 3,stdout='',stderr='0 bytes')
        with patch.object(usb,'usb_inventory',return_value=devices), \
             patch.object(usb.os,'geteuid',return_value=0),patch.object(usb.os,'access',return_value=True), \
             patch.object(usb.shutil,'which',side_effect=lambda name:'/fixture/'+name), \
             patch.object(usb.subprocess,'run',side_effect=failed_upload):
            try:usb.recover_t265(Path(temporary)/'failed.json');assert False,'failed upload must report failure'
            except RuntimeError:pass
        assert len(commands)==2 and all('uhubctl' not in c[0] for c in commands)


def test_motion_log_storage():
    """Deletion only in a temporary root; no UART/SDK or real-log cleanup."""
    from unittest.mock import patch
    from motion_log_storage import inspect_session, list_sessions, delete_sessions, OWNER, WHEEL_OWNER, WHEEL_SESSION_FILES
    from t265_f407_motion_map import SESSION_FILES, prune_completed_sessions, MotionMapApp, arguments
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        sessions = []
        for i in range(10):
            p = root / f"20261104_00000{i}_interactive"
            p.mkdir()
            (p/"metadata.json").write_text(json.dumps(dict(mode="interactive_f405_normaldebug", sdk_session_id=str(i))))
            (p/"session_complete.json").write_text(json.dumps(dict(owner=OWNER, complete=True, session_index=i)))
            (p/"t265_raw_pose.csv").write_text("fixture\n")
            sessions.append(p)
        current = sessions[-1]
        (root/"protected_sessions.json").write_text(json.dumps(dict(owner=OWNER, sessions=[sessions[0].name])))
        (sessions[1]/"session_complete.json").unlink()
        (sessions[1]/"events.jsonl").write_text(json.dumps(dict(event="session_stopped", localizer_returncode=1)))
        (sessions[2]/"用户笔记.txt").write_text("keep")
        (sessions[3]/"link").symlink_to(sessions[4]/"t265_raw_pose.csv")
        rows = list_sessions(root, current, SESSION_FILES)
        locked = {r["name"] for r in rows if r["reason"]}
        assert locked == {p.name for p in sessions[:4]+[current]}
        assert prune_completed_sessions(root, current) == [sessions[4].name]
        assert all(p.exists() for p in sessions[:4]+sessions[5:])
        # A newly protected, changed, or symlink-substituted selection must survive.
        stale = inspect_session(root, sessions[5].name, current, SESSION_FILES)
        (sessions[5]/"t265_raw_pose.csv").write_text("new data\n")
        assert delete_sessions(root, [stale], current, SESSION_FILES)[0] == []
        stale = inspect_session(root, sessions[6].name, current, SESSION_FILES)
        (root/"protected_sessions.json").write_text(json.dumps(dict(owner=OWNER, sessions=[sessions[0].name,sessions[6].name])))
        assert delete_sessions(root, [stale], current, SESSION_FILES)[0] == []
        stale = inspect_session(root, sessions[7].name, current, SESSION_FILES)
        sessions[7].rename(root/"user_backup")
        sessions[7].symlink_to(root/"user_backup", target_is_directory=True)
        assert delete_sessions(root, [stale], current, SESSION_FILES)[0] == []
        assert (root/"user_backup"/"t265_raw_pose.csv").exists()
        # Broken protection file fails closed for deletion, not for motion.
        (root/"protected_sessions.json").write_text("{")
        assert inspect_session(root, sessions[8].name, current, SESSION_FILES)["reason"]
        (root/"protected_sessions.json").write_text(json.dumps(dict(owner=OWNER,sessions=[sessions[0].name,sessions[6].name])))
        # Existing UI: manual selection, two-click confirmation, stop pass-through.
        with patch("sys.argv",["map","--output-root",str(root)]), \
                patch.object(MotionMapApp,"detect_screen_size",return_value=(1024,768)):
            app = MotionMapApp(arguments())
        app.run_dir = current
        panel = app.storage_panel
        try:
            with patch("motion_log_storage.disk_capacity",return_value=(64*2**30,60*2**30,4*2**30)) as capacity, \
                    patch("motion_log_storage.time.monotonic",return_value=100.):
                image = app.render_screen()
                panel.future.result(timeout=2)
                app.render_screen()
                for _ in range(3): app.render_screen()
                assert capacity.call_count == 1 and image.shape == (768,1024,3)
                app.select_at(780,1000)
                app.render_screen(); panel.future.result(timeout=2); app.render_screen()
                assert panel.open and panel.rows
                assert not panel.click(950,570,app)  # Original STOP remains reachable.
                assert panel.key(ord("v")) and not panel.key(ord("s")) and not panel.key(32)
                stops = []
                app.send_stop = lambda *args: stops.append(True)
                app.measurement_buffer = ""
                app.handle_key(ord("s"))
                assert stops == [True]  # STOP is not swallowed by measurement input.
                app.measurement_buffer = None
                panel.page = 0
                row = next(r for r in panel.rows if r["name"]==sessions[8].name)
                panel.selected = {row["name"]}
                app.render_screen()
                app.select_at(600,775)
                assert panel.confirm and sessions[8].exists()
                app.select_at(600,775)
                app.render_screen(); panel.future.result(timeout=2); app.render_screen()
                assert not sessions[8].exists() and current.exists()
                assert all(p.exists() for p in sessions[:4])
                app.handle_key(27)
                assert not panel.open  # ESC closes the panel, not the map.
                panel.next_refresh = 0
            with patch("motion_log_storage.disk_capacity",side_effect=OSError("unavailable")):
                if panel.future: panel.future.result(timeout=2)
                panel.poll(current)
                try: panel.future.result(timeout=2)
                except OSError: pass
                panel.poll(current)
                assert panel.capacity is None and "unavailable" in panel.error
        finally:
            panel.close()
        # Same entry can manage logs even when sensor startup would fail.
        with patch("sys.argv",["map","--logs-only","--windowed","--output-root",str(root)]), \
                patch.object(MotionMapApp,"detect_screen_size",return_value=(1024,768)):
            manager = MotionMapApp(arguments())
        before = sorted(p.name for p in root.iterdir())
        with patch.object(manager,"start_session",side_effect=AssertionError("hardware startup")), \
                patch.object(manager,"update_data",side_effect=AssertionError("sensor processing")), \
                patch("cv2.namedWindow"), patch("cv2.resizeWindow"), patch("cv2.imshow"), \
                patch("cv2.waitKey",return_value=-1), patch("cv2.setMouseCallback"), \
                patch("cv2.getWindowProperty",return_value=0), patch("cv2.destroyWindow"):
            assert manager.run() == 0
        assert before == sorted(p.name for p in root.iterdir())
    # Formal wheel sessions share deletion mechanics, never the test owner/root.
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);owner="competition_wheel_input"
        files={"metadata.json","session_complete.json"}
        names=[f"20261105_00000{i}_competition_wheel" for i in range(7)]
        for i,name in enumerate(names):
            p=root/name;p.mkdir()
            (p/'metadata.json').write_text(json.dumps(dict(owner=owner,mode='competition_t265_wheel_input',sdk_session_id=str(i))))
            (p/'session_complete.json').write_text(json.dumps(dict(owner=owner,complete=True,session_index=i)))
        current=root/names[-1]
        assert not list_sessions(root,current,files)  # Old test module cannot select these.
        rows=[r for r in list_sessions(root,current,files,owner=owner) if not r['reason']]
        rows.sort(key=lambda r:r['order'],reverse=True)
        removed,_=delete_sessions(root,rows[4:],current,files,owner=owner)
        assert set(removed)==set(names[:2]) and current.exists()
        assert len(list(root.iterdir()))==5
    print("motion log storage / retention / existing map UI PASS (temporary files only)")


if __name__ == "__main__":
    main()


def test_competition_feedback_adapter():
    """Formal Python is now an observer, never a second SDK packet producer.
    No actual SDK API or motor action; keep actual identities/measurements.
    """
    import socket, uuid, struct, threading
    import f405_timed_protocol as wire
    from competition_wheel_input import CompetitionWheelInput
    from t265_f407_motion_map import config_values, DEFAULT_CONFIG
    from t265_wheel_input import NativeWheelObserver
    examples=json.loads((Path(__file__).with_name('contracts')/'f405_timed_examples_20261031.json').read_text())['examples']
    def raw(name):return bytes.fromhex(next(x['hex'] for x in examples if x['name']==name))
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary); sid=uuid.uuid4().hex
        sink=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        sink.bind('\0t265-wheel-'+sid);sink.settimeout(.5)
        adapter=CompetitionWheelInput(root,config_values(DEFAULT_CONFIG),sid)
        source=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        observed=[];arrival=threading.Event()
        def observe(message):
            observed.append(message);arrival.set()
        adapter.telemetry_sink=observe
        try:
            assert type(adapter.bridge) is NativeWheelObserver
            adapter.start();source.connect(adapter.address)
            wheel=wire.decode(raw('forward'));g=wire.decode(raw('gyro_ccw'))
            p=bytearray(wire.parse(raw('c_module_host_fixture_type_2'))['payload'])
            p[:8]=adapter.identity_nonce.to_bytes(8,'big')
            p[8:10]=b'\0\0';p[10:26]=bytes.fromhex(wheel['boot_epoch'])
            p[41]=2  # Actual TASK service, not MotionDebug. No mode/SHA admission lock.
            p[42:44]=bytes((1,3));p[52:56]=(3).to_bytes(4,'big')
            p[56:60]=(0xffffffff).to_bytes(4,'big')
            received=time.monotonic_ns()-5_000_000_000
            def feed(frame):
                arrival.clear();source.send(struct.pack('>Q',received)+frame)
                assert arrival.wait(1),adapter.error
                assert observed[-1]['sample']['receive_monotonic_ns']==received
            feed(wire.packet(wire.ID_REPLY,0,p))
            for i,t in enumerate((wheel['t0_us'],wheel['t1_us'])):
                payload=struct.pack('>16sIIqQIIIiQIIIQ',bytes.fromhex(g['boot_epoch']),g['reference_epoch'],i,
                    g['yaw_mdeg'],t,100+i,0x347,0,0,t,100+i,0x347,0,t)
                feed(wire.packet(wire.GYRO,i,payload))
            feed(raw('forward'))
            sink.setblocking(False)
            try:sink.recv(16000);raise AssertionError('Python must not also inject')
            except BlockingIOError:pass
            assert adapter.identity['service']==2 and adapter.identity['epoch_valid']
            state=adapter.snapshot()
            assert state['stats']['sent_to_sdk_owner']==0 and state['sdk_api_success_samples']==0
            assert state['processing_owner']=='native_cpp_sdk_owner' and not state['python_sdk_sender']
            assert state['latest_measured']['v_body_mps']==[wheel['forward_um_s']/1e6,wheel['left_um_s']/1e6,0.]
            # The formal observer keeps its existing EOF/reaccept behaviour;
            # each accepted socket receives its own fixed packet buffer.
            source.close();source=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
            source.connect(adapter.address);feed(raw('forward'))
            assert adapter.error is None
        finally:
            source.close();adapter.close();sink.close()


def test_native_motion_feedback_transport():
    """Same real wire codec and original conversion; no UART/SDK or motor."""
    import math,struct
    from f405_timed_measurements import TimedMeasurements
    from lidar_yaw_debug.inputs import NativeFeedbackAdapter,forward_motion_event
    import f405_timed_protocol as wire
    old=[];new=[];logged=[]
    def collect(target):
        def send(kind,payload):target.append((kind,payload));return True
        return send
    reference=SimpleNamespace(clock_map=TimedMeasurements(),radar=SimpleNamespace(send=collect(old)),
        timed_yaw_seen=False,last_yaw_id=None,imu_packets=0,devent=lambda *a,**k:None)
    adapter=NativeFeedbackAdapter(collect(new),lambda *a,**k:None,
        SimpleNamespace(submit=lambda filename,value:logged.append((filename,value))))
    epoch=bytes.fromhex('11'*16)
    identity=dict(boot_epoch=epoch.hex(),epoch_valid=True,interface_compatible=True,
                  protocol_major=1,protocol_minor=3,drift_bound_ppm=0xffffffff)
    sync=dict(boot_epoch=epoch.hex(),epoch_valid=1,nonce=7,host_write_before_ns=1_000_000_000,
        receive_monotonic_ns=1_001_000_000,receive_event_us=1_000_100,
        response_prepare_us=1_000_200,tick_resolution_us=1)
    def context(kind,sample):
        event=dict(kind=kind,sample=sample)
        forward_motion_event(reference,event);adapter.context(event)
    context('timed_identity',identity);context('timed_clock_sync',sync)
    context('imu',dict(wire_valid=True,timestamp_monotonic_ns=1_002_000_000,yaw_deg=12.))
    scale=math.pi*.070/1061.268
    for index,sign in enumerate((1,-1),1):
        a=1_010_000+index*10_000;b=a+10_000
        delta=(10*sign,-10*sign,0)
        raw=wire.packet(wire.WHEEL,index,struct.pack('>16sIIQQiiIIIiiiIII',epoch,1,index,a,b,
            round(scale*(delta[0]-delta[1])/(math.sqrt(3)*.01)*1e6),0,1,2,3,*delta,129,0xffffffff,3))
        gyro=wire.packet(wire.GYRO,index,struct.pack('>16sIIqQIIIiQIIIQ',epoch,1,index,
            sign*90000,b,index,835,0xffffffff,sign*1000,b,index,835,0xffffffff,b+100))
        for packet,kind in ((raw,'timed_wheel_diagnostic'),(gyro,'timed_gyro_diagnostic')):
            receive=b*1000+1_000_000
            sample=wire.decode(packet)
            sample.update(receive_monotonic_ns=receive,capture_time_qualified=False,sdk_time_qualified=False)
            forward_motion_event(reference,dict(kind=kind,sample=sample))
            adapter.accept(struct.pack('>Q',receive)+packet)
            # Actual duplicate samples cannot become extra motion updates.
            forward_motion_event(reference,dict(kind=kind,sample=sample))
            adapter.accept(struct.pack('>Q',receive)+packet)
    context('imu',dict(wire_valid=True,timestamp_monotonic_ns=1_050_000_000,yaw_deg=99.))
    assert old==new and len(new)==5  # legacy before timed, 2 wheel + 2 timed yaw
    assert adapter.timed_yaw_seen and adapter.snapshot()['wheel_frames']==4
    assert len(logged)==8 and all(name=='motion_feedback.jsonl' for name,_ in logged)
    # An already matched clock snapshot remains an exact context copy, not a
    # reconstructed exchange or a duplicate-wheel baseline from the observer.
    bootstrap=NativeFeedbackAdapter(lambda *a:True,lambda *a,**k:None,SimpleNamespace(submit=lambda *a:None))
    bootstrap.context(dict(kind='clock_context',sample=dict(identity=identity,syncs=list(reference.clock_map.syncs))))
    assert bootstrap.clock_map.identity==reference.clock_map.identity
    assert list(bootstrap.clock_map.syncs)==list(reference.clock_map.syncs)
    assert bootstrap.clock_map.last_wheel is None
    context('device_identity_changed',dict(new_boot_epoch='22'*16))
    assert old==new and new[-1][0]=='invalidate' and not adapter.timed_yaw_seen


def test_competition_dynamic_transport():
    """Real worker/IPC through a PTY; no camera, chassis, motors or raw log edits."""
    import math, os, pty, socket, uuid
    from async_debug_log import AsyncDebugLog
    from competition_dynamic_localization import CompetitionDynamicLocalization
    from lidar_yaw_debug.dynamic import DynamicFence
    from lidar_yaw_debug.diagnostic import load_config
    cfg=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')
    # Selected competition branch is preserved at every start; delayed setup
    # must not subtract displacement already present in the SDK field pose.
    for x,y,yaw in ((-1.32,1.32,135.),(1.32,1.32,45.),(1.32,-1.32,225.),(-1.32,-1.32,315.)):
        e=DynamicFence(cfg['installation'])
        stamp=1_000_000_000
        e.ingest_pose(dict(session_id='fixture',frame_number=1,timestamp_monotonic_ns=stamp,
            receive_monotonic_ns=stamp,x_m=x+.06,y_m=y,yaw_rad=math.radians(yaw),tracker_confidence=3))
        e.ingest_yaw(stamp,1234.,{})
        # A received wheel-feedback interval brackets the first tracked Pose.
        # The earlier 6 cm field displacement must not become a false conflict
        # against zero travel in this stationary interval.
        e.ingest_wheel(dict(source='measured_wheel_feedback',measurement_valid=True,
            mcu_epoch='fixture-boot',sample_id=1,
            raw_mcu_sample=dict(t0_us=(stamp-10_000_000)//1000,t1_us=(stamp+10_000_000)//1000),
            v_body_mps=[0.,0.,0.],interval_start_ns=stamp-10_000_000,
            interval_end_ns=stamp+10_000_000,receive_monotonic_ns=stamp+10_000_000))
        e.align('formal fixture',physical_confirmed=False,center_xy=(x,y),heading_deg=yaw,first_sample=True)
        state=e.snapshot()
        assert abs(state['center_xy_m'][0]-x-.06)<1e-10
        assert state['center_xy_m']==state['t265_center_xy_m']
        assert state['motion_selection']['mode']=='T265_WITH_WHEEL_TAIL'
        assert state['motion_selection']['spatial_conflict_m']==0.
        assert e.reference['selected_start']==dict(center_xy_m=[x,y],yaw_deg=yaw)
        assert abs(state['yaw_deg']-yaw)<1e-9 and e.reference['vio_rotation']==0
    master,slave=pty.openpty()
    with tempfile.TemporaryDirectory() as tmp:
        root=Path(tmp);sid=uuid.uuid4().hex;log=AsyncDebugLog(root)
        port=root/'lidar-port'
        cfg['lidar']['port']=str(port)  # Initially missing; motion must still start.
        app=CompetitionDynamicLocalization(root,sid,cfg,dict(x_m=-1.32,y_m=1.32,yaw_deg=135),log)
        sink=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        sink.bind(app.control_address);sink.settimeout(4)
        source=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        def wait_packet(predicate):
            deadline=time.monotonic()+4
            while time.monotonic()<deadline:
                result=json.loads(sink.recv(32768))
                if predicate(result):return result
            raise AssertionError('dynamic output did not reach expected state')
        try:
            app.start();source.connect(app.pose_address)
            now=time.monotonic_ns()
            app.telemetry(dict(kind='imu',sample=dict(wire_valid=True,timestamp_monotonic_ns=now,yaw_deg=100.)))
            source.send(json.dumps(dict(session_id=sid,frame_number=1,timestamp_monotonic_ns=now,
                receive_monotonic_ns=now,x_m=-1.30,y_m=1.32,yaw_rad=math.radians(135),tracker_confidence=3)).encode())
            packet=json.loads(sink.recv(32768))
            assert packet['pose']==dict(x_m=-1.30,y_m=1.32,yaw_deg=135.)
            assert packet['timestamp_monotonic_ns']==now and packet['reference']['source']=='selected_competition_start'
            assert packet['qualified_pose'] is False and not packet['wheel']['independent_reference']
            assert not packet['lidar_connected']
            # Motion can publish before the independent receiver has attempted
            # serial.open; its initial unknown error is not a reason to wait.
            if not packet['lidar_error']:
                wait_packet(lambda p:not p['lidar_connected'] and p['lidar_error'])
            port.symlink_to(os.ttyname(slave))
            connected=wait_packet(lambda p:p['lidar_connected'])
            assert connected['pose']==packet['pose'] and connected['lidar_error'] is None
            # Availability changes are delivered without another SDK frame;
            # an idle timer must not manufacture a new sensor sample.
            assert connected['packet_id']>packet['packet_id']
            assert connected['timestamp_monotonic_ns']==now
            app.set_lidar_enabled(False)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                later=json.loads(sink.recv(32768))
                if not later['lidar_corrections_enabled']:break
            assert not later['lidar_corrections_enabled']
            assert later['pose']==packet['pose']
            # UART unplug cannot kill the motion engine or freeze its output.
            os.close(master);os.close(slave);master=slave=None
            down=wait_packet(lambda p:not p['lidar_connected'])
            assert down['current_wall_axes']==[] and down['lidar_error']
            def wheel(index,a,b,speed):
                return dict(source='measured_wheel_feedback',measurement_valid=True,
                    mcu_epoch='fixture-boot',sample_id=index,v_body_mps=[speed,0.,0.],
                    raw_mcu_sample=dict(t0_us=a//1000,t1_us=b//1000),
                    interval_start_ns=a,interval_end_ns=b,receive_monotonic_ns=b)
            app.radar.send('wheel',wheel(1,now-10_000_000,now,0.))
            wait_packet(lambda p:p['motion_source']=='T265_WITH_WHEEL_TAIL')
            source.send(json.dumps(dict(session_id=sid,frame_number=2,timestamp_monotonic_ns=now+1_000_000,
                receive_monotonic_ns=now+1_000_000,x_m=-1.30,y_m=1.32,
                yaw_rad=math.radians(135),tracker_confidence=0)).encode())
            wait_packet(lambda p:p['motion_source']=='WHEEL_FALLBACK')
            end=now+200_000_000
            app.radar.send('yaw',dict(time_ns=end,yaw_deg=100.,source={}))
            app.radar.send('wheel',wheel(2,now,end,.5))
            moved=wait_packet(lambda p:p['timestamp_monotonic_ns']==end)
            assert not moved['lidar_connected'] and moved['motion_source']=='WHEEL_FALLBACK'
            assert abs(moved['pose']['x_m']-(-1.30-.1/math.sqrt(2)))<1e-8
            assert abs(moved['pose']['y_m']-(1.32+.1/math.sqrt(2)))<1e-8
            # Reopen the port automatically without resetting XY/reference or
            # overriding the operator's existing pause of wall corrections.
            master,slave=pty.openpty();port.unlink();port.symlink_to(os.ttyname(slave))
            recovered=wait_packet(lambda p:p['lidar_connected'])
            assert recovered['pose']==moved['pose'] and recovered['reference']==packet['reference']
            assert not recovered['lidar_corrections_enabled']
            app.set_lidar_enabled(True)
            wait_packet(lambda p:p['lidar_corrections_enabled'])
            # The same formal supervisor passes its bound phase socket through
            # spawn. Exercise the original Python fallback owner too.
            with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as phase:
                def policy(suppressed):
                    phase.sendto(json.dumps(dict(schema=1,kind='competition_wheel_policy',session_id=sid,
                        suppressed=suppressed,reason='safe_zone_low_speed')).encode(),app.wheel_policy_address)
                policy(True)
                paused=wait_packet(lambda p:p['motion_source']=='T265_PHASE_NO_WHEEL')
                assert paused['pose']==moved['pose']
                app.radar.send('wheel',wheel(3,end,end+100_000_000,5.))
                source.send(json.dumps(dict(session_id=sid,frame_number=3,timestamp_monotonic_ns=end+100_000_000,
                    receive_monotonic_ns=end+100_000_000,x_m=-1.30,y_m=1.32,
                    yaw_rad=math.radians(135),tracker_confidence=3)).encode())
                still=wait_packet(lambda p:p['timestamp_monotonic_ns']==end+100_000_000)
                assert still['pose']==paused['pose'] and still['motion_source']=='T265_PHASE_NO_WHEEL'
                policy(False)
                resumed=wait_packet(lambda p:p['motion_source']=='T265_WITH_WHEEL_TAIL')
                assert resumed['pose']==still['pose']
        finally:
            source.close();sink.close();app.close();log.close()
            for fd in (master,slave):
                if fd is not None:os.close(fd)
        assert app.radar.stopped_cleanly and log.healthy() and app.error is None
        assert not (root/'commands.jsonl').exists()


def test_latest_control_snapshot_mailbox():
    import multiprocessing as mp
    import queue
    from lidar_yaw_debug.latest_snapshot import LatestSnapshot,MotionInbox
    inbox=MotionInbox()
    try:
        inbox.put_nowait(('wheel',dict(sample=1)))
        assert inbox.get(timeout=0)==('wheel',dict(sample=1))
        try:inbox.get_nowait()
        except queue.Empty:pass
        else:raise AssertionError('empty input must not manufacture a measurement')
        accepted=0
        for _ in range(1000):
            try:inbox.put_nowait(dict(data='x'*1024));accepted+=1
            except queue.Full:break
        else:raise AssertionError('motion input capacity must be bounded')
        for _ in range(accepted):assert inbox.get_nowait()==dict(data='x'*1024)
    finally:inbox.close()
    box=LatestSnapshot(mp.get_context('spawn'),capacity=1024)
    assert box.get(timeout=0) is None
    assert box.put_latest(dict(sample=1))==0
    assert box.put_latest(dict(sample=2))==1
    assert box.get(timeout=0)==dict(sample=2)
    assert box.get(timeout=0) is None
    box.wake()
    assert box.get(timeout=0) is None  # Operator/exit wake is not a new pose.
    try:box.put_latest(dict(data='x'*2048))
    except ValueError:pass
    else:raise AssertionError('mailbox capacity must be bounded')
    box.put_latest(dict(sample=3))
    assert box.get(timeout=0)==dict(sample=3)


def test_localization_realtime_permission_fallback():
    from unittest.mock import patch
    from types import SimpleNamespace
    import low_latency_scheduling as scheduling
    with patch.dict('os.environ',{'RDK_LOCALIZATION_RT':'1'}), \
            patch.object(scheduling.resource,'getrlimit',return_value=(0,0)), \
            patch.object(scheduling.subprocess,'run',return_value=SimpleNamespace(returncode=1,stderr='not permitted')):
        result=scheduling.prepare_realtime()
        assert result['rtprio_limit']==[0,0] and result['error']=='not permitted'
    with patch.dict('os.environ',{'RDK_LOCALIZATION_RT':'1'}), \
            patch.object(scheduling.os,'sched_setscheduler',side_effect=PermissionError('not permitted')):
        assert scheduling.set_realtime_thread()['error']=='not permitted'
    with patch.dict('os.environ',{'RDK_LOCALIZATION_RT':'0'}), \
            patch.object(scheduling.os,'sched_setscheduler') as apply:
        assert not scheduling.set_realtime_thread()['requested']
        apply.assert_not_called()


def test_competition_control_rate():
    """Paced 100 distinct feedback intervals through the existing worker/IPC.

    Records offline transport latency, not MCU/UART, SDK or physical accuracy.
    The same fixture can report the old 20Hz transport as a baseline by calling
    with require_all=False; no artificial repeated pose stamps are counted.
    """
    competition_control_rate_fixture()


class _ControlLatencyFixture:
    """Opt-in benchmark wrapper; no clocks/fields added to production IPC."""
    def __init__(self, mailbox):
        self.mailbox=mailbox;self.times={}

    def put_latest(self,value):
        return self.mailbox.put_latest(dict(value,_fixture_publish_ns=time.monotonic_ns()))

    def get(self,timeout=None):
        value=self.mailbox.get(timeout)
        if value is not None:
            received=time.monotonic_ns();published=value.pop('_fixture_publish_ns')
            stamp=value['last'].get('sample_monotonic_ns')
            if stamp is not None:self.times.setdefault(stamp,(published,received))
        return value

    def wake(self):
        self.mailbox.wake()


class _MotionLatencyFixture:
    """Record input ingress in shared storage in this benchmark only."""
    def __init__(self,inbox,context,capacity):
        self.inbox=inbox;self.received=context.RawArray('q',capacity)

    def put_nowait(self,value):self.inbox.put_nowait(value)

    def get(self,timeout=None):
        value=self.inbox.get(timeout)
        kind,payload=value
        if kind=='wheel' and 0<=payload['sample_id']<len(self.received):
            self.received[payload['sample_id']]=time.monotonic_ns()
        return value

    def get_nowait(self):return self.get(timeout=0)
    def close(self):self.inbox.close()
    def cancel_join_thread(self):self.inbox.cancel_join_thread()


def competition_control_rate_fixture(*,require_all=True,lidar_records=None,interval_count=100,measure_stages=False):
    import os,pty,socket,threading,uuid
    from async_debug_log import AsyncDebugLog
    from competition_dynamic_localization import CompetitionDynamicLocalization
    from lidar_yaw_debug.diagnostic import load_config
    import numpy as np
    cfg=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')
    with tempfile.TemporaryDirectory() as tmp:
        root=Path(tmp);sid=uuid.uuid4().hex;log=AsyncDebugLog(root)
        master=slave=None
        cfg['lidar']['port']=str(root/'absent-lidar')
        if lidar_records:
            master,slave=pty.openpty();cfg['lidar']['port']=os.ttyname(slave)
        app=CompetitionDynamicLocalization(root,sid,cfg,dict(x_m=0.,y_m=0.,yaw_deg=0.),log)
        if measure_stages:
            app.radar.control=_ControlLatencyFixture(app.radar.control)
            app.radar.inbox=_MotionLatencyFixture(app.radar.inbox,app.radar.context,interval_count+1)
        sink=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        sink.bind(app.control_address);sink.settimeout(5)
        source=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        sent={};scheduled={};received={};producer=lidar_producer=None
        def wheel(index,a,b):
            return dict(source='measured_wheel_feedback',measurement_valid=True,
                mcu_epoch='fixture-boot',sample_id=index,v_body_mps=[.1,0.,0.],
                raw_mcu_sample=dict(t0_us=a//1000,t1_us=b//1000),
                interval_start_ns=a,interval_end_ns=b,receive_monotonic_ns=b)
        try:
            app.start();source.connect(app.pose_address)
            now=time.monotonic_ns()
            app.telemetry(dict(kind='imu',sample=dict(wire_valid=True,timestamp_monotonic_ns=now,yaw_deg=0.)))
            source.send(json.dumps(dict(session_id=sid,frame_number=1,timestamp_monotonic_ns=now,
                receive_monotonic_ns=now,x_m=0.,y_m=0.,yaw_rad=0.,tracker_confidence=3)).encode())
            packet=json.loads(sink.recv(32768))
            assert packet['timestamp_monotonic_ns']==now
            if lidar_records:
                while not packet['lidar_connected']:packet=json.loads(sink.recv(32768))
                def replay():
                    origin=lidar_records[0]['timestamp_monotonic_ns'];start=time.monotonic_ns()
                    for row in lidar_records:
                        delay=(start+row['timestamp_monotonic_ns']-origin-time.monotonic_ns())/1e9
                        if delay>0:time.sleep(delay)
                        os.write(master,bytes.fromhex(row['data_hex']))
                lidar_producer=threading.Thread(target=replay);lidar_producer.start()
            def produce():
                start=time.monotonic_ns()
                for i in range(1,interval_count+1):
                    deadline=start+i*10_000_000
                    delay=(deadline-time.monotonic_ns())/1e9
                    if delay>0:time.sleep(delay)
                    t=now+i*10_000_000
                    sent[t]=time.monotonic_ns()
                    scheduled[t]=deadline
                    app.telemetry(dict(kind='imu',sample=dict(wire_valid=True,timestamp_monotonic_ns=t,yaw_deg=0.)))
                    app.radar.send('wheel',wheel(i,t-10_000_000,t))
                    # Two actual Pose samples/interval, the SDK's 200Hz shape.
                    for j in (1,0):
                        p=t-5_000_000*j
                        source.send(json.dumps(dict(session_id=sid,frame_number=2*i+1-j,
                            timestamp_monotonic_ns=p,receive_monotonic_ns=p,
                            x_m=.1*(p-now)/1e9,y_m=0.,yaw_rad=0.,tracker_confidence=3)).encode())
            producer=threading.Thread(target=produce);producer.start()
            while True:
                packet=json.loads(sink.recv(32768));stamp=packet['timestamp_monotonic_ns']
                if stamp in sent and stamp not in received:received[stamp]=time.monotonic_ns()
                if stamp==now+interval_count*10_000_000:break
            producer.join()
            if not lidar_records:assert abs(packet['pose']['x_m']-.001*interval_count)<1e-9
            assert not packet['wheel']['independent_reference']
            latency=[(received[t]-sent[t])/1e6 for t in received]
            metrics=dict(sent_distinct_intervals=len(sent),received_distinct_intervals=len(received),
                processing_ms_median=float(np.median(latency)),processing_ms_p95=float(np.percentile(latency,95)),
                processing_ms_max=max(latency),worker_input_drops=app.radar.input_drops,
                output_drops=app.output_drops)
            capture=[(received[t]-scheduled[t])/1e6 for t in received]
            lateness=[(sent[t]-scheduled[t])/1e6 for t in sent]
            metrics.update(synthetic_capture_to_output_ms_p95=float(np.percentile(capture,95)),
                synthetic_capture_to_output_ms_max=max(capture),
                producer_schedule_delay_ms_p95=float(np.percentile(lateness,95)),
                producer_schedule_delay_ms_max=max(lateness))
            if measure_stages:
                times=app.radar.control.times
                stages={name:[] for name in ('input_to_worker_publication','mailbox_delivery','control_to_sink',
                    'input_to_worker_ingress','worker_ingress_to_publication')}
                for stamp,end in received.items():
                    if stamp not in times:continue
                    publication,delivery=times[stamp]
                    for name,duration in zip(tuple(stages)[:3],(publication-sent[stamp],delivery-publication,end-delivery)):
                        stages[name].append(duration/1e6)
                    index=(stamp-now)//10_000_000
                    ingress=app.radar.inbox.received[index]
                    if ingress:
                        stages['input_to_worker_ingress'].append((ingress-sent[stamp])/1e6)
                        stages['worker_ingress_to_publication'].append((publication-ingress)/1e6)
                metrics['fixture_stage_ms']={name:dict(median=float(np.median(values)),
                    p95=float(np.percentile(values,95)),maximum=max(values),samples=len(values),
                    negative_samples=sum(v<0 for v in values)) for name,values in stages.items() if values}
        finally:
            if producer is not None:producer.join()
            if lidar_producer is not None:lidar_producer.join()
            source.close();sink.close();app.close();log.close()
            for fd in (master,slave):
                if fd is not None:os.close(fd)
        assert app.error is None and app.radar.stopped_cleanly and log.healthy()
        assert metrics['worker_input_drops']==metrics['output_drops']==0
        metrics.update(lidar_scans=app.radar.state['scans_received'],
            lidar_geometry_calls=app.radar.state['scans'],lidar_accepted_posteriors=app.radar.state['fits'],
            line_geometry=app.radar.state.get('line_geometry'),
            fit_processing_ms_p95=app.radar.state['scan_processing_ms_p95'],
            ingested_distinct_intervals=app.radar.state['measured_wheel_ingested'],
            motion_scheduling=app.radar.state.get('motion_scheduling'),
            worker_control_updates=app.radar.state.get('control_updates'),
            worker_control_coalesced=app.radar.state.get('control_updates_coalesced'),
            transport_scheduling=dict(app.scheduling) if hasattr(app,'scheduling') else None)
        fit_log=root/'dynamic_localization.jsonl'
        if fit_log.exists():
            fits=[json.loads(line) for line in fit_log.open()]
            metrics['fit_stage_ms']={key:dict(median=float(np.median(values)),p95=float(np.percentile(values,95)))
                for key in ('compute_ms','compute_thread_cpu_ms','snapshot_ms','deskew_ms','fit_ms')
                if (values:=[row[key] for row in fits if key in row])}
        # The bounded output deliberately coalesces if its consumer is delayed.
        # Check all real input intervals were processed; report actual distinct
        # output and latency instead of making OS scheduling a lossless test.
        if require_all:assert metrics['ingested_distinct_intervals']==len(sent),metrics
        print(json.dumps(dict(competition_control_rate=metrics)))
        return metrics


def test_native_test_uart_receive_transport():
    """Queued IPC keeps complete frames/native timestamps; no hardware/clock."""
    import ctypes, errno, socket, struct
    from unittest.mock import patch
    import native_test_uart as transport
    import local_packet_receiver as packets
    rx, peer = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    uart = transport.NativeTestUart.__new__(transport.NativeTestUart)
    uart.rx = rx
    uart.receive_monotonic_ns = None
    uart.dropped_rx_datagrams = 0
    rx.settimeout(.01)
    def frame(stamp, drops, data):
        return struct.pack('>QQ', stamp, drops) + data
    try:
        assert not packets.readable_nowait(rx.fileno())
        # Same maximum read/truncation semantics; each complete local packet is
        # returned once, even if serial.read's requested size is one byte.
        peer.send(frame(123, 7, b'abc'))
        peer.send(frame(124, 8, bytes(range(256)) * 16 + b'truncated'))
        peer.send(frame(125, 9, b'last'))
        assert packets.readable_nowait(rx.fileno())
        original_recv = packets._recv_nowait
        interrupted = [False]
        def interrupted_once(fd, buffer, size, flags):
            assert size == 4112 and flags == socket.MSG_DONTWAIT
            if not interrupted[0]:
                interrupted[0] = True
                ctypes.set_errno(errno.EINTR)
                return -1
            return original_recv(fd, buffer, size, flags)
        with patch.object(packets, '_recv_nowait', interrupted_once), \
             patch.object(packets.select, 'select', side_effect=AssertionError('queued RX waited')), \
             patch.object(transport.time, 'monotonic', side_effect=AssertionError('RX read a clock')):
            assert uart.read(1) == b'abc'
            assert (uart.receive_monotonic_ns, uart.dropped_rx_datagrams) == (123, 7)
            assert uart.read(1) == bytes(range(256)) * 16
            assert (uart.receive_monotonic_ns, uart.dropped_rx_datagrams) == (124, 8)
            assert uart.read(4096) == b'last'
            assert (uart.receive_monotonic_ns, uart.dropped_rx_datagrams) == (125, 9)
        assert not packets.readable_nowait(rx.fileno())
        assert uart.read(4096) == b''  # Existing socket timeout, no fake header.
        assert (uart.receive_monotonic_ns, uart.dropped_rx_datagrams) == (125, 9)
        try:
            uart._receiver.recv()
            assert False, 'packet helper must retain socket.timeout'
        except socket.timeout:
            pass
        def arrives_during_wait(readers, writers, errors, timeout):
            assert readers == [rx] and not writers and not errors and timeout == .01
            peer.send(frame(126, 10, b'after-empty'))
            return [rx], [], []
        with patch.object(packets.select, 'select', arrives_during_wait):
            assert uart.read(1) == b'after-empty'
        assert (uart.receive_monotonic_ns, uart.dropped_rx_datagrams) == (126, 10)
        rx.settimeout(None)
        def arrives_during_indefinite_wait(readers, writers, errors, timeout):
            assert readers == [rx] and not writers and not errors and timeout is None
            peer.send(frame(127, 11, b'blocking-socket'))
            return [rx], [], []
        with patch.object(packets.select, 'select', arrives_during_indefinite_wait):
            assert uart.read(1) == b'blocking-socket'
        rx.settimeout(0)
        try:
            uart.read(1)
            assert False, 'nonblocking empty socket must retain BlockingIOError'
        except BlockingIOError as error:
            assert error.errno in (errno.EAGAIN, errno.EWOULDBLOCK)
        rx.settimeout(.01)
        peer.send(b'short')
        try:
            uart.read(1)
            assert False, 'short container accepted'
        except OSError as error:
            assert str(error) == 'short native UART receive container'
        peer.close()
        assert packets.readable_nowait(rx.fileno())
        assert uart._receiver.recv() == b''  # EOF is data for the generic helper.
        try:
            uart.read(1)
            assert False, 'EOF must retain the original transport error'
        except OSError as error:
            assert str(error) == 'short native UART receive container'
        rx.close()
        try:
            uart.read(1)
            assert False, 'closed descriptor accepted'
        except OSError as error:
            assert error.errno == errno.EBADF
    finally:
        rx.close()
        peer.close()
    import uuid
    address='\0local-send-'+uuid.uuid4().hex
    with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as output, \
            socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as endpoint:
        output.setblocking(False);endpoint.setblocking(False);endpoint.bind(address)
        sender=packets.LocalDatagramSender(output,address)
        payload=b'unchanged\0local\xffpayload'
        assert output.sendto(payload,address)==len(payload)
        assert endpoint.recv(65536)==payload
        assert sender.send(payload)==len(payload)
        assert endpoint.recv(65536)==payload
        original_sendto=packets._sendto_nowait
        interrupted_send=[False]
        def interrupted_sendto_once(*args):
            if not interrupted_send[0]:
                interrupted_send[0]=True;ctypes.set_errno(errno.EINTR);return -1
            return original_sendto(*args)
        with patch.object(packets,'_sendto_nowait',interrupted_sendto_once):
            assert sender.send(payload)==len(payload)
        assert endpoint.recv(65536)==payload
        assert not packets.readable_nowait(endpoint.fileno())  # EINTR did not duplicate the datagram.
        with patch.object(packets,'_sendto_nowait',return_value=len(payload)-1):
            try:sender.send(payload)
            except OSError as error:assert error.errno==errno.EIO
            else:raise AssertionError('short datagram send claimed success')
        missing=address+'-unbound'
        failures=[]
        for send in (lambda:output.sendto(payload,missing),
                     lambda:packets.LocalDatagramSender(output,missing).send(payload)):
            try:send()
            except OSError as error:failures.append(error.errno)
            else:raise AssertionError('unbound destination accepted')
        assert len(failures)==2 and failures[0]==failures[1]
        for _ in range(1000):
            try:sender.send(payload)
            except BlockingIOError as error:
                assert error.errno in (errno.EAGAIN,errno.EWOULDBLOCK)
                break
        else:raise AssertionError('local datagram capacity must remain bounded')
        try:output.sendto(payload,address)
        except BlockingIOError as error:assert error.errno in (errno.EAGAIN,errno.EWOULDBLOCK)
        else:raise AssertionError('original sendto disagrees about full destination')
        for invalid in ('','\0','pathname','\0'+'x'*108):
            try:packets.LocalDatagramSender(output,invalid)
            except ValueError:pass
            else:raise AssertionError('invalid abstract destination accepted')
        try:sender.send(b'x'*65537)
        except ValueError:pass
        else:raise AssertionError('oversized local datagram accepted')


def test_native_cpp_wheel_input():
    """Cross-language wire→interval gyro→C velocity parity, no devices/motors.

    Reuses existing reference codecs/bridge and C++ regression executable;
    exercises forward/left signs, both spins, combined varying gyro, invalid
    values, duplicate sources, and coalescing without a timestamp/age gate.
    """
    import math, socket, struct, subprocess
    import f405_timed_protocol as wire
    from t265_f407_motion_map import config_values,DEFAULT_CONFIG,LOCALIZATION_ROOT
    from t265_wheel_input import WheelInputBridge
    examples=json.loads((Path(__file__).with_name('contracts')/'f405_timed_examples_20261031.json').read_text())['examples']
    raw=lambda name:bytes.fromhex(next(x['hex'] for x in examples if x['name']==name))
    epoch=bytes.fromhex(wire.decode(raw('forward'))['boot_epoch'])
    identity=bytearray(wire.parse(raw('c_module_host_fixture_type_2'))['payload'])
    identity[:8]=(7).to_bytes(8,'big');identity[8:10]=b'\0\0';identity[10:26]=epoch
    identity[42:44]=bytes((1,3));identity[52:56]=(3).to_bytes(4,'big');identity[56:60]=(0xffffffff).to_bytes(4,'big')
    events=[]
    def add(packet,*,request=False,drain=True):
        events.append(dict(raw_hex=packet.hex(),timestamp_ns=1_000_000_000+len(events)*100_000,request=request,drain=drain))
    add(wire.request(wire.ID_REQ,7,3),request=True);add(wire.packet(wire.ID_REPLY,3,identity))
    rate_id=0
    def gyro(t,rate,flags=0x341,drain=True,duplicate=False):
        nonlocal rate_id
        if not duplicate:rate_id+=1
        payload=struct.pack('>16sIIqQIIIiQIIIQ',epoch,1,rate_id,0,t,rate_id,0x341,0xffffffff,
                            rate,t,rate_id,flags,0xffffffff,t)
        add(wire.packet(wire.GYRO,rate_id&65535,payload),drain=drain)
    t=1_000_000
    gyro(t-5000,0);gyro(t,0)
    k=math.pi*.070/1061.268
    for i in range(1,49):
        if i==12:
            add(motion_move_frame(42,0,300,200),request=True)
        d1,d2,d3=((20,-20,0),(-20,20,0),(12,12,-24),(-12,-12,24),(25,-15,-10),(0,0,0))[i%6]
        vx=k*(d1-d2)/(math.sqrt(3)*.01);vy=k*(d1+d2-2*d3)/(.03)
        payload=struct.pack('>16sIIQQiiIIIiiiIII',epoch,1,i,t,t+10000,round(vx*1e6),round(vy*1e6),
                            i,i,i,d1,d2,d3,0x281,0xffffffff,4)
        # Waiting for the actual end bracket, with a nonconstant angular rate.
        add(wire.packet(wire.WHEEL,i,payload),drain=i%7!=0)
        gyro(t+5000,(-1 if i%2 else 1)*57296,drain=i%7!=0)
        gyro(t+10000,(-1 if i%3 else 1)*28648)
        if i%5==0:add(wire.packet(wire.WHEEL,i,payload))  # exact repeated measurement
        t+=10000
    # No fake zero: duplicate rate is not a new bracket; an invalid value
    # interrupts support instead of interpolating through a clipped gyro.
    gyro(t,0,duplicate=True);gyro(t+5000,0,flags=0x361);gyro(t+10000,0)
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);source=root/'wire.jsonl'
        source.write_text(''.join(json.dumps(e)+'\n' for e in events))
        native=[json.loads(line) for line in subprocess.check_output([
            str(LOCALIZATION_ROOT/'build/test_omni_localization'),'--wheel-replay',str(source),str(DEFAULT_CONFIG)],text=True).splitlines()]
        sink=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);sink.bind('\0t265-wheel-parity-check');sink.setblocking(False)
        bridge=WheelInputBridge(root,config_values(DEFAULT_CONFIG),'parity-check',True,clock=lambda:20_000_000_000)
        reference=[]
        try:
            for e in events:
                if not e['request']:
                    s=wire.decode(bytes.fromhex(e['raw_hex']));s['receive_monotonic_ns']=e['timestamp_ns']
                    if s['type']==wire.ID_REPLY:
                        s.update(epoch_valid=True,interface_compatible=True);kind='timed_identity'
                    else:kind='timed_wheel_diagnostic' if s['type']==wire.WHEEL else 'timed_gyro_diagnostic'
                    bridge.telemetry(dict(kind=kind,sample=s))
                if e['drain']:
                    bridge.poll()
                    while True:
                        try:reference.append(json.loads(sink.recv(16384)))
                        except BlockingIOError:break
        finally:bridge.close();sink.close()
        if len(native)!=len(reference):
            Path('/tmp/rdk_native_wheel_parity_failed.jsonl').write_text(source.read_text())
        assert len(native)==len(reference) and len(native)>=40,(len(native),len(reference))
        exact=('sample_id','alignment_clock','alignment_start_ns','alignment_end_ns','alignment_gyro_start_ns',
               'alignment_gyro_end_ns','receive_monotonic_ns','gyro_receive_ns','gyro_max_gap_ns','capture_time_qualified')
        max_error=0.
        for actual,expected in zip(native,reference):
            assert all(actual[k]==expected[k] for k in exact),(actual,expected)
            for key in ('v_body_mps','omega_body_rad_s','lever_body_mps','v_camera_mps'):
                error=max(abs(a-b) for a,b in zip(actual[key],expected[key]));max_error=max(max_error,error)
                assert error<1e-12,(key,actual[key],expected[key])
            if actual['sample_id']>=12:
                assert actual['motion_wire_request_at_receive']['wire_sequence']==42
        print(json.dumps(dict(native_cpp_wheel_parity_samples=len(native),max_vector_error=max_error,
                              unknown_time_bounds_preserved=True,actual_SDK=False,motor_commands=False)))


def test_wall_supported_source_arbitration():
    """Known independent synthetic wall displacements, not ground precision.

    Wheel-only slip, VIO-only error, both wrong, single-wall ambiguity, late
    capture-time correction, recovery, and a later confidence=3 jump.
    """
    import math,numpy as np
    from collections import deque
    from lidar_yaw_debug.motion_source import MotionSource,sample
    class Playback:
        def __init__(self,angle=0.):
            self.m=MotionSource();self.origin=1_000_000_000;self.raw=deque();self.yaws=deque();self.i=0
            self.angle=angle;self.start=np.array([.2,-.3]);c,s=math.cos(angle),math.sin(angle);self.r=np.array([[c,-s],[s,c]])
            self.m.configure(dict(pose_origin=[0.,0.],vio_rotation=angle,yaw_offset=angle,
                reference_sample_ns=self.origin,start_center_xy_m=self.start.tolist()))
            self.pose(0.,self.origin)
        def pose(self,x,t,confidence=3):
            self.m.pose(dict(timestamp_monotonic_ns=t,x_m=x,y_m=0.,tracker_confidence=confidence))
            if confidence>0:self.raw.append((t,x,0.,0.))
        def tick(self,v,x):
            self.i+=1;t=self.origin+self.i*10_000_000;self.yaws.append((t,0.))
            self.m.yaw(t,0.,dict(raw=dict(yaw_receive_us=t//1000)))
            self.m.wheel(dict(source='measured_wheel_feedback',measurement_valid=True,mcu_epoch='fixture',sample_id=self.i,
                raw_mcu_sample=dict(t0_us=(t-10_000_000)//1000,t1_us=t//1000),v_body_mps=[v,0.,0.],
                interval_start_ns=t-10_000_000,interval_end_ns=t,receive_monotonic_ns=t),self.yaws)
            self.pose(x,t)
            if self.i==1:self.wall(0.,index=0)
        def wall(self,x,axes=('x','y'),index=None,reacquired=False):
            i=self.i if index is None else index
            self.m.observe_walls('scan-'+str(i),list(axes),reference_ns=self.origin+i*10_000_000,
                center_xy=(self.start+self.r@np.array([x,0.])).tolist(),raw_poses=self.raw,reacquired=reacquired)
    slip=Playback()
    for i in range(1,41):
        slip.tick(1.,0.)
        if i%10==0:slip.wall(0.)
    assert slip.m.mode=='T265_PRIMARY_WHEEL_SUSPECT' and slip.m.wheel_suspect
    held=slip.m.history[-1][1]
    for i in range(41,51):slip.tick(1.,0.)
    assert abs(slip.m.history[-1][1]-held)<1e-12  # no slipping-wheel tail
    before=slip.m.history[-1]
    slip.pose(.8,before[0]+5_000_000)  # confidence=3 still cannot bypass old jump handling
    assert slip.m.mode=='WHEEL_FALLBACK' and slip.m.wheel_suspect
    assert slip.m.reason=='SDK_single_step_jump_while_wheel_suspect'
    assert abs(slip.m.history[-1][1]-before[1])<1e-12
    slip.tick(1.,.8)  # Pair the queued jump on the next actual wheel endpoint.
    assert slip.m.mode=='WHEEL_FALLBACK' and slip.m.wheel_suspect
    assert slip.m.reason=='SDK_single_step_jump_while_wheel_suspect'
    assert all(a[0]<b[0] for a,b in zip(slip.m.history,list(slip.m.history)[1:]))
    for which in ('vio_bad','both_bad','single_wall'):
        p=Playback()
        for i in range(1,51):
            raw=3*i*.01 if which=='vio_bad' else -i*.01 if which=='both_bad' else 0.
            p.tick(1.,raw)
            if i%10==0:p.wall(i*.01 if which=='vio_bad' else 0.,axes=('x',) if which=='single_wall' else ('x','y'))
        assert p.m.mode=='WHEEL_FALLBACK' and not p.m.wheel_suspect,which
    recovery=Playback()
    for i in range(1,111):
        x=0. if i<=40 else (i-40)*.01
        recovery.tick(1.,x)
        if i%10==0:recovery.wall(x)
    assert recovery.m.mode=='T265_WITH_WHEEL_TAIL' and not recovery.m.wheel_suspect,recovery.m.snapshot()
    # The fit arrives after more motion: correcting at the capture time must
    # preserve exactly the VIO motion AFTER that scan, in both field branches.
    for angle in (0.,.7):
        p=Playback(angle)
        for i in range(1,51):p.tick(2.,i*.01)
        p.wall(.3,index=30);before_scan=sample(p.m.history,p.origin+400_000_000)[:2]
        p.wall(.4,index=40)
        assert p.m.mode=='T265_PRIMARY_WHEEL_SUSPECT'
        selected=sample(p.m.history,p.origin+400_000_000)[:2]
        assert np.allclose(selected,before_scan)
        candidate=p.start+p.r@np.array([.4,0.]);offset=candidate-(p.start+p.r@selected)
        posterior=p.start+p.r@np.array(p.m.history[-1][1:3])+offset
        assert np.allclose(posterior,p.start+p.r@np.array([.5,0.]),atol=1e-12)
    print('PASS wall-supported arbitration / one-axis ambiguity / capture-time source rebase / confidence=3 jump')


def test_native_runtime_motion_source_parity():
    """Same production source handlers, offline; no UART, SDK or motion.

    Compare after every input, including source changes rather than merely the
    final position. These are known numerical fixtures, not sensor accuracy.
    """
    import math
    import numpy as np
    from collections import deque
    from lidar_yaw_debug.motion_source import MotionSource
    from lidar_yaw_debug.native_motion import NativeMotionRuntime

    def equivalent(expected, actual, path='motion_selection'):
        if isinstance(expected, dict):
            assert set(expected) == set(actual), (path, set(expected) ^ set(actual))
            for key, value in expected.items():equivalent(value, actual[key], path+'.'+key)
        elif isinstance(expected, (list, tuple)):
            assert len(expected) == len(actual), (path, expected, actual)
            for i, (a,b) in enumerate(zip(expected,actual)):equivalent(a,b,path+'['+str(i)+']')
        elif isinstance(expected, bool) or expected is None or isinstance(expected,str):
            assert actual == expected and (not isinstance(expected,bool) or isinstance(actual,bool)), (path,expected,actual)
        else:
            assert math.isclose(float(expected),float(actual),rel_tol=1e-12,abs_tol=1e-10), (path,expected,actual)

    class Playback:
        def __init__(self, angle=0., configure=True):
            self.native=NativeMotionRuntime(dict(feedback_fd=-1,pose_fd=-1))
            self.m=MotionSource();self.origin=1_000_000_000;self.raw=deque();self.yaws=deque();self.i=0
            self.start=np.array([.2,-.3]);c,s=math.cos(angle),math.sin(angle);self.r=np.array([[c,-s],[s,c]])
            self.reference=dict(pose_origin=[0.,0.],vio_rotation=angle,yaw_offset=angle,
                reference_sample_ns=self.origin,start_center_xy_m=self.start.tolist())
            if configure:self.m.configure(self.reference);self.native.command('configure',self.reference)
            self.check()
            self.pose(0.,self.origin)
        def check(self):
            state=self.native.snapshot(dict(kind='histories'))
            equivalent(self.m.snapshot(),state['motion_selection'])
            equivalent(list(self.m.events),state['events'],'source_events')
            for key,expected in (('history',self.m.history),('wheels',self.m.wheels),('native_yaws',self.m.native_yaws)):
                actual=state[key];expected=list(expected)
                assert len(actual)==len(expected),(key,len(actual),len(expected))
                if expected:
                    assert np.array_equal(np.asarray(actual)[:,0],np.asarray(expected)[:,0]),key
                    assert np.allclose(np.asarray(actual)[:,1:],np.asarray(expected)[:,1:],rtol=1e-12,atol=1e-10),(key,actual[-3:],expected[-3:])
            return state
        def pose(self,x,t,confidence=3):
            p=dict(session_id='native-parity',frame_number=t-self.origin,timestamp_monotonic_ns=t,
                receive_monotonic_ns=t,x_m=x,y_m=0.,yaw_rad=0.,tracker_confidence=confidence)
            self.m.pose(p);self.native.command('pose',p)
            if confidence>0:self.raw.append((t,x,0.,0.))
            self.check()
        def yaw(self,t,angle=0.):
            source=dict(raw=dict(yaw_receive_us=t//1000))
            if not self.yaws or t>=self.yaws[-1][0]:
                if self.yaws and t==self.yaws[-1][0]:self.yaws.pop()
                self.yaws.append((t,angle))
            self.m.yaw(t,angle,source)
            self.native.command('yaw',dict(time_ns=t,yaw_deg=math.degrees(angle),source=source));self.check()
        def wheel(self,p):
            self.m.wheel(p,self.yaws);self.native.command('wheel',p);self.check()
        def wheel_tick(self,v):
            self.i+=1;t=self.origin+self.i*10_000_000;self.yaw(t)
            self.wheel(dict(source='measured_wheel_feedback',measurement_valid=True,mcu_epoch='fixture',sample_id=self.i,
                raw_mcu_sample=dict(t0_us=(t-10_000_000)//1000,t1_us=t//1000),v_body_mps=[v,0.,0.],
                interval_start_ns=t-10_000_000,interval_end_ns=t,receive_monotonic_ns=t))
            return t
        def tick(self,v,x):
            t=self.wheel_tick(v)
            self.pose(x,t)
            if self.i==1:self.wall(0.,index=0)
        def suppress(self,value):
            reason='fixture_safe_zone' if value else 'fixture_phase_exit'
            self.m.set_wheel_suppressed(value,reason)
            self.native.command('wheel_suppression',dict(suppressed=value,reason=reason));self.check()
        def wall(self,x,axes=('x','y'),index=None,reacquired=False):
            i=self.i if index is None else index
            p=dict(scan_id='scan-'+str(i),axes=list(axes),reference_ns=self.origin+i*10_000_000,
                center_xy=(self.start+self.r@np.array([x,0.])).tolist(),raw_poses=list(self.raw),reacquired=reacquired)
            self.m.observe_walls(**p);self.native.command('walls',p);self.check()
        def close(self):self.native.close()

    for scenario in ('wheel_slip','wheel_then_vio_bad','vio_bad','both_bad','single_wall','recovery'):
        p=Playback()
        try:
            limit=110 if scenario=='recovery' else 100 if scenario=='wheel_then_vio_bad' else 50
            for i in range(1,limit+1):
                x=(3*i*.01 if scenario=='vio_bad' else -i*.01 if scenario=='both_bad' else
                   3*max(0,i-40)*.01 if scenario=='wheel_then_vio_bad' else
                   max(0,i-40)*.01 if scenario=='recovery' else 0.)
                p.tick(1.,x)
                if i%10==0:p.wall(i*.01 if scenario=='vio_bad' else x if scenario=='recovery' else
                    max(0,i-40)*.01 if scenario=='wheel_then_vio_bad' else 0.,
                    axes=('x',) if scenario=='single_wall' else ('x','y'))
            if scenario=='wheel_slip':
                assert p.m.mode=='T265_PRIMARY_WHEEL_SUSPECT'
                p.pose(.8,p.origin+505_000_000)  # confidence=3 still cannot bypass the jump policy.
                assert p.m.mode=='WHEEL_FALLBACK' and p.m.wheel_suspect
                p.tick(1.,.8)
                assert p.m.reason=='SDK_single_step_jump_while_wheel_suspect'
            elif scenario=='recovery':assert p.m.mode=='T265_WITH_WHEEL_TAIL' and not p.m.wheel_suspect
            else:assert p.m.mode=='WHEEL_FALLBACK' and not p.m.wheel_suspect
        finally:p.close()

    # A late two-wall verdict changes the source at scan capture, preserving
    # all subsequent actual Pose increments in either field rotation.
    for angle in (0.,.7):
        p=Playback(angle)
        try:
            for i in range(1,51):p.tick(2.,i*.01)
            p.wall(.3,index=30);p.wall(.4,index=40)
            assert p.m.mode=='T265_PRIMARY_WHEEL_SUSPECT'
        finally:p.close()

    # Float implementations need not be bit-identical; classifications on
    # either side of the unchanged50mm/25mm boundaries must still agree.
    for delta in (-1e-8,1e-8):
        p=Playback()
        try:
            p.tick(1.,.01+.05+delta)
            assert p.m.mode==('T265_WITH_WHEEL_TAIL' if delta<0 else 'WHEEL_FALLBACK')
        finally:p.close()
        p=Playback()
        try:
            for i in range(1,31):p.tick(1.,i/30*(.025+delta))
            p.wall(0.)
            assert p.m.wall_evidence['status']==('WHEEL_SUSPECT' if delta<0 else 'AMBIGUOUS')
        finally:p.close()

    # Actual wheel identity/order checks retain source state; no age gate or
    # fake epoch acceptance is introduced to make the native fixture pass.
    p=Playback(.7)
    try:
        p.tick(.5,.005)
        t=p.origin+10_000_000
        packet=dict(source='measured_wheel_feedback',measurement_valid=True,mcu_epoch='fixture',sample_id=1,
            raw_mcu_sample=dict(t0_us=p.origin//1000,t1_us=t//1000),v_body_mps=[.5,0.,0.],
            interval_start_ns=p.origin,interval_end_ns=t,receive_monotonic_ns=t)
        for altered in (packet,dict(packet,sample_id=0),dict(packet,mcu_epoch='another-boot',sample_id=2),
                        dict(packet,source='command_target'),dict(packet,measurement_valid=False)):
            p.wheel(altered)
        p.yaw(t-5_000_000,.2);p.yaw(t,.1)
        p.pose(.005,t)  # duplicate frame/sample does not create another update
        assert len(p.m.wheels)==2 and p.m.last_wheel==('fixture',1)
        p.pose(.005,t+5_000_000,confidence=0)
        assert p.m.mode=='WHEEL_FALLBACK'
    finally:p.close()

    # The phase excludes both arbitrarily bad wheel tails and source verdicts,
    # while retaining raw wheel integration. Enter/exit cover either sample order.
    for order in ('pose_ahead','wheel_ahead'):
        p=Playback(.7)
        try:
            for i in range(1,11):p.tick(1.,i*.01)
            if order=='pose_ahead':p.pose(.102,p.origin+105_000_000)
            else:p.wheel_tick(4.)
            before=p.m.history[-1][1];raw=p.m.last_tracked_xy[0]
            p.suppress(True)
            count=len(p.m.events);bias=p.m.bias.copy()
            p.suppress(True)  # Repeated business state must not re-anchor motion.
            assert len(p.m.events)==count and np.array_equal(p.m.bias,bias)
            for _ in range(20):
                raw+=.001;p.tick(4.,raw)
            assert p.m.mode=='T265_PHASE_NO_WHEEL' and abs(p.m.history[-1][1]-before-.02)<1e-10
            assert p.m.wheels[-1][1]>.8 and p.m.snapshot()['wheel_input_policy']['pause_requested']
            p.wall(3.);p.wall(4.);p.wall(5.)  # Cannot reinstate wheel through wall arbitration.
            assert p.m.mode=='T265_PHASE_NO_WHEEL' and p.m.evidence_confirmations==0
            held=list(p.m.history)
            p.pose(90.,p.origin+p.i*10_000_000+1,confidence=0)
            for _ in range(15):p.wheel_tick(4.)
            assert list(p.m.history)==held  # No fake stationary samples or confidence-zero motion.
            if order=='pose_ahead':
                raw+=.003;p.pose(raw,p.origin+p.i*10_000_000+5_000_000)
            exit_xy=np.array(p.m.history[-1][1:3]);exit_ns=max(p.m.history[-1][0],p.m.wheels[-1][0])
            p.suppress(False)
            assert np.allclose(p.m.history[-1][1:3],exit_xy,rtol=0,atol=1e-10)
            assert not p.m.snapshot()['wheel_input_policy']['pause_requested']
            end=p.wheel_tick(1.)
            expected=exit_xy+p.m._wheel_at(end)[:2]-p.m._wheel_at(exit_ns)[:2]
            assert np.allclose(p.m.history[-1][1:3],expected,rtol=0,atol=1e-10)
        finally:p.close()

    # A reference established during suppression must not resume injection;
    # with no reference/wheels there is no manufactured selected trajectory.
    p=Playback(configure=False)
    try:
        p.suppress(True)
        p.pose(.2,p.origin+5_000_000)
        assert not p.m.history
        p.m.configure(p.reference);p.native.command('configure',p.reference);p.check()
        assert p.m.mode=='T265_PHASE_NO_WHEEL' and p.m.snapshot()['wheel_input_policy']['pause_requested']
        assert len(p.m.history)==1 and p.m.history[-1][0]==p.origin+5_000_000
        p.pose(.5,p.origin+6_000_000)  # Phase adds no new jump rejection threshold.
        assert abs(p.m.history[-1][1]-.5)<1e-10
        p.suppress(False);p.wheel_tick(1.)
        assert np.linalg.norm(np.asarray(p.m.history[-1][1:3])-[.5,0.])<.01
    finally:p.close()

    # Exiting a business phase cannot clear an existing wall-supported slip.
    p=Playback()
    try:
        for i in range(1,51):
            p.tick(1.,0.)
            if i%10==0:p.wall(0.)
        assert p.m.wheel_suspect
        p.suppress(True)
        p.tick(4.,.001);before=p.m.history[-1][1]
        p.suppress(False)
        policy=p.m.snapshot()['wheel_input_policy']
        assert p.m.mode=='T265_PRIMARY_WHEEL_SUSPECT' and p.m.wheel_suspect
        assert policy['pause_requested'] and policy['reason']=='two_axis_wall_motion_wheel_suspect'
        assert p.m.history[-1][1]==before
        p.pose(.002,p.origin+p.i*10_000_000+5_000_000)
        assert abs(p.m.history[-1][1]-before-.001)<1e-10
    finally:p.close()


def test_native_runtime_historical_wall_parity():
    """Delayed posterior uses current history and preserves later motion."""
    import math
    import numpy as np
    from lidar_yaw_debug.dynamic import DynamicFence,rotation
    from lidar_yaw_debug.motion_source import sample
    from lidar_yaw_debug.native_motion import NativeMotionRuntime
    from lidar_yaw_debug.diagnostic import load_config
    mount=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')['installation']
    for angle in (0.,.7):
        for axes in (['x','y'],['x']):
            e=DynamicFence(mount);native=NativeMotionRuntime(dict(feedback_fd=-1,pose_fd=-1))
            a=1_000_000_000;b=a+100_000_000
            def pose(frame,t,x=0.,y=0.):
                p=dict(session_id='native-late-wall',frame_number=frame,timestamp_monotonic_ns=t,
                    receive_monotonic_ns=t,x_m=x,y_m=y,yaw_rad=0.,tracker_confidence=3)
                e.ingest_pose(p);native.command('pose',p)
            def yaw(t):
                p=dict(time_ns=t,yaw_deg=0.,source={})
                e.ingest_yaw(t,0.,{});native.command('yaw',p)
            def wheel(i,start,end):
                p=dict(source='measured_wheel_feedback',measurement_valid=True,mcu_epoch='fixture',sample_id=i,
                    v_body_mps=[.1,0.,0.],raw_mcu_sample=dict(t0_us=start//1000,t1_us=end//1000),
                    interval_start_ns=start,interval_end_ns=end,receive_monotonic_ns=end)
                e.ingest_wheel(p);native.command('wheel',p)
            try:
                pose(1,a);yaw(a);yaw(b)
                reference=e.align('offline known reference',physical_confirmed=False,center_xy=(.2,-.3),
                    heading_deg=math.degrees(angle))
                native.command('configure',reference)
                wheel(1,a,b)
                candidate=np.array([.2,-.3])+rotation(angle)@np.array([.01,0.])
                pose(2,b-10_000_000,.049,.02);yaw(b+100_000_000);wheel(2,b,b+100_000_000)
                state=native.snapshot(dict(kind='histories'))
                p=dict(revision=state['revision'],scan_id='known-late-wall',reference_ns=b,
                    candidate_center_xy_m=candidate.tolist(),position_observed_axes=axes,position_reacquired=False)
                native.command('apply_wall',p)
                # Exact production commit equations, independent of geometry.
                e.motion.observe_walls(p['scan_id'],axes,reference_ns=b,center_xy=candidate,raw_poses=e.poses)
                current=sample(e.motion.history or e.poses,b)
                current_xy=rotation(reference['vio_rotation'])@(current[:2]-reference['pose_origin'])+reference['start_center_xy_m']
                for axis in axes:
                    j=('x','y').index(axis);e.offset[j]=candidate[j]-current_xy[j]
                after=native.snapshot(dict(kind='histories'))
                # Geometry's narrow projection must preserve the complete
                # window's exact measured brackets/reference, including the
                # current posterior and the selected wheel-tail history.
                for first,last,latest in ((a-20_000_000,a-10_000_000,False),
                                          (a,b,False),(b-5_000_000,b+5_000_000,False),
                                          (b+200_000_000,b+300_000_000,False),
                                          (a,b,True)):
                    query=dict(kind='window',first_ns=first,last_ns=last,latest=latest)
                    full=native.snapshot(query)
                    compact=native.snapshot(dict(query,geometry_only=True))
                    assert set(compact)=={'poses','yaws','reference','offset','revision'}
                    assert compact=={key:full[key] for key in compact}
                assert np.allclose(after['offset'],e.offset,rtol=1e-12,atol=1e-10)
                assert np.allclose(after['history'],list(e.motion.history),rtol=1e-12,atol=1e-10)
                selected=np.asarray(after['history'][-1])
                final=rotation(reference['vio_rotation'])@(selected[1:3]-reference['pose_origin'])+reference['start_center_xy_m']+after['offset']
                corrected_at_scan=final-rotation(angle)@np.array([.01,0.])
                for axis in axes:
                    j=('x','y').index(axis);assert abs(corrected_at_scan[j]-candidate[j])<1e-10
                if axes==['x']:assert after['offset'][1]==0.
                assert int(selected[0])==b+100_000_000
            finally:native.close()


def test_native_runtime_competition_wheel_policy():
    """Local phase socket and real native owner; no UART, SDK or motors."""
    import socket
    import uuid
    from lidar_yaw_debug.native_motion import NativeMotionRuntime
    sid=uuid.uuid4().hex
    phase=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    sdk=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    sender=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    phase_address='\0test-competition-phase-'+sid
    sdk_address='\0test-competition-sdk-policy-'+sid
    phase.bind(phase_address);phase.setblocking(False)
    sdk.bind(sdk_address);sdk.settimeout(1.)
    runtime=None
    with tempfile.TemporaryDirectory() as temporary:
        try:
            runtime=NativeMotionRuntime(dict(session_id=sid,feedback_fd=-1,pose_fd=-1,
                wheel_policy_fd=phase.fileno(),sdk_policy_address='@'+sdk_address[1:],log_root=temporary))
            t=1_000_000_000;i=0
            reference=dict(pose_origin=[0.,0.],vio_rotation=0.,yaw_offset=0.,
                reference_sample_ns=t,start_center_xy_m=[0.,0.])

            def pose(x,confidence=3):
                runtime.command('pose',dict(session_id=sid,frame_number=t,
                    timestamp_monotonic_ns=t,receive_monotonic_ns=t,x_m=x,y_m=0.,yaw_rad=0.,
                    tracker_confidence=confidence,mapper_confidence=1))

            def wheel(v):
                nonlocal t,i
                t+=10_000_000;i+=1
                runtime.command('yaw',dict(time_ns=t,yaw_deg=0.,source=dict(raw=dict(yaw_receive_us=t//1000))))
                runtime.command('wheel',dict(source='measured_wheel_feedback',measurement_valid=True,
                    mcu_epoch='fixture',sample_id=i,v_body_mps=[v,0.,0.],
                    raw_mcu_sample=dict(t0_us=(t-10_000_000)//1000,t1_us=t//1000),
                    interval_start_ns=t-10_000_000,interval_end_ns=t,receive_monotonic_ns=t))

            def send(suppressed,sequence,session=sid):
                sender.sendto(json.dumps(dict(schema=1,kind='competition_wheel_policy',session_id=session,
                    sequence=sequence,suppressed=suppressed,reason='safe_zone_low_speed' if suppressed else 'normal_motion',
                    state='fixture',task_id=7,action_id=8)).encode(),phase_address)

            def wait(sequence):
                deadline=time.monotonic()+1.
                while time.monotonic()<deadline:
                    state=runtime.snapshot(dict(kind='histories'))
                    if (state.get('competition_wheel_policy') or {}).get('sequence')==sequence:return state
                    time.sleep(.001)
                raise AssertionError('local native phase input did not finish')

            runtime.command('yaw',dict(time_ns=t,yaw_deg=0.,source=dict(raw=dict(yaw_receive_us=t//1000))))
            pose(0.);runtime.command('field_reference',dict(reference=reference,revision=1))
            wheel(1.);pose(.01)
            send(True,0,session='old-session');send(False,1)
            assert not wait(1)['motion_selection']['phase_wheel_suppressed']
            sender.sendto(b'{malformed',phase_address)
            send(True,2)
            paused=wait(2)
            assert paused['motion_selection']['mode']=='T265_PHASE_NO_WHEEL'
            policy=json.loads(sdk.recv(4096))
            assert policy['pause'] is True and policy['reason']=='safe_zone_low_speed'
            assert policy['session_id']==sid and policy['dependency_persists_after_pause'] is True
            for _ in range(30):wheel(5.);pose(.01)
            state=runtime.snapshot(dict(kind='histories'))
            assert abs(state['last']['center_xy_m'][0]-.01)<1e-12
            assert state['wheels'][-1][1]>1.5  # Original wheel evidence remains.
            previous_sample=state['last']['sample_monotonic_ns']
            wheel(5.);pose(99.,confidence=0)
            held=runtime.snapshot()
            assert held['last']['sample_monotonic_ns']==previous_sample
            assert held['last']['center_xy_m']==state['last']['center_xy_m']
            # Accepted single-wall correction remains active with wheel disabled.
            runtime.command('apply_wall',dict(revision=1,scan_id='fixture-wall',reference_ns=previous_sample,
                candidate_center_xy_m=[.31,99.],position_observed_axes=['x']))
            corrected=runtime.snapshot()
            assert abs(corrected['last']['center_xy_m'][0]-.31)<1e-12
            assert corrected['last']['center_xy_m'][1]==0.
            assert corrected['motion_selection']['phase_wheel_suppressed']
            send(False,3);resumed=wait(3)
            assert abs(resumed['last']['center_xy_m'][0]-.31)<1e-12, (resumed['motion_selection'],resumed['history'][-3:],resumed['wheels'][-3:])
            assert json.loads(sdk.recv(4096))['pause'] is False
            wheel(.1);pose(.011)
            assert abs(runtime.snapshot()['last']['center_xy_m'][0]-.311)<1e-10
            # Resetting a source/reference cannot silently re-enable phase wheel input.
            send(True,4);wait(4);assert json.loads(sdk.recv(4096))['pause'] is True
            runtime.command('invalidate',dict(reason='fixture reference reset',revision=2))
            assert runtime.snapshot()['motion_selection']['phase_wheel_suppressed']
            runtime.command('field_reference',dict(reference=reference,revision=3))
            assert runtime.snapshot()['motion_selection']['wheel_input_policy']['pause_requested']
            assert runtime.snapshot()['error'] is None
            runtime.close()
            receipt=runtime.snapshot()
            assert all(not item['failed'] and not item['dropped'] for item in receipt['logging'].values())
            events=[json.loads(line) for line in (Path(temporary)/'native_motion_events.jsonl').read_text().splitlines()]
            assert any(e['event']=='competition_wheel_policy_error' for e in events)
            assert len([e for e in events if e['event']=='competition_wheel_policy_applied'])==4
        finally:
            if runtime is not None:runtime.close()
            sender.close();phase.close();sdk.close()


def test_native_runtime_socket_input_and_control_parity():
    """Real local sockets exercise production wire/JSON handlers, no devices."""
    import math
    import socket
    import struct
    import uuid
    from f405_timed_measurements import TimedMeasurements
    import f405_timed_protocol as wire
    from lidar_yaw_debug.control_packet import format_control_packet
    from lidar_yaw_debug.diagnostic import load_config
    from lidar_yaw_debug.dynamic import DynamicFence
    from lidar_yaw_debug.inputs import NativeFeedbackAdapter
    from lidar_yaw_debug.native_motion import NativeMotionRuntime

    def equivalent(expected,actual,path='packet'):
        if isinstance(expected,dict):
            assert set(expected)==set(actual),(path,set(expected)^set(actual))
            for key,value in expected.items():equivalent(value,actual[key],path+'.'+key)
        elif isinstance(expected,(list,tuple)):
            assert len(expected)==len(actual),(path,expected,actual)
            for i,(a,b) in enumerate(zip(expected,actual)):equivalent(a,b,path+'['+str(i)+']')
        elif isinstance(expected,bool) or expected is None or isinstance(expected,str):
            assert actual==expected and (not isinstance(expected,bool) or isinstance(actual,bool)),(path,expected,actual)
        else:assert math.isclose(float(expected),float(actual),rel_tol=1e-12,abs_tol=1e-10),(path,expected,actual)

    mount=load_config(Path(__file__).resolve().parents[1]/'config/lidar_yaw_debug.json')['installation']
    original=DynamicFence(mount);forwarded=[];logged=[]
    def forward(kind,payload):
        forwarded.append((kind,payload))
        if kind=='wheel':original.ingest_wheel(payload)
        elif kind=='yaw':original.ingest_yaw(payload['time_ns'],payload['yaw_deg'],payload['source'])
        elif kind=='invalidate':original.invalidate_reference(payload['reason'])
        else:raise AssertionError(kind)
        return True
    adapter=NativeFeedbackAdapter(forward,lambda *a,**k:None,
        SimpleNamespace(submit=lambda name,value:logged.append((name,value))))
    epoch=bytes.fromhex('12'*16)
    identity=dict(boot_epoch=epoch.hex(),epoch_valid=True,interface_compatible=True,
        protocol_major=1,protocol_minor=3,drift_bound_ppm=0xffffffff)
    clock=TimedMeasurements();clock.set_identity(identity)
    assert clock.sync(dict(boot_epoch=epoch.hex(),epoch_valid=1,nonce=7,
        host_write_before_ns=2_000_000_000,receive_monotonic_ns=2_001_000_000,
        receive_event_us=1_000_100,response_prepare_us=1_000_200,tick_resolution_us=1))
    context=dict(kind='clock_context',sample=dict(identity=identity,syncs=list(clock.syncs)))
    sid=uuid.uuid4().hex;addresses=['\0native-motion-test-'+sid+suffix for suffix in ('-wheel','-pose','-control')]
    owned=[];runtime=None
    with tempfile.TemporaryDirectory() as temporary:
        try:
            for address in addresses[:2]:
                server=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);owned.append(server)
                server.bind(address);server.listen(1)
            terminal=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);owned.append(terminal)
            terminal.bind(addresses[2]);terminal.settimeout(1.)
            runtime=NativeMotionRuntime(dict(feedback_fd=owned[0].fileno(),pose_fd=owned[1].fileno(),
                control_address='@'+addresses[2][1:],session_id=sid,pose_session_id=sid,log_root=temporary))
            clients=[]
            for address in addresses[:2]:
                client=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);owned.append(client)
                client.connect(address);clients.append(client)
            adapter.context(context);runtime.command('context',context)
            wire_count=0;pose_count=0;packets=[];latest_raw={};last_ready=None

            def wait(predicate):
                # A bounded fixture wait for local processing, not a production
                # freshness policy or an extra MCU request/handshake.
                deadline=time.monotonic()+2.
                while time.monotonic()<deadline:
                    state=runtime.snapshot(dict(kind='histories'))
                    if predicate(state):return state
                    time.sleep(.001)
                raise AssertionError('native local socket handler did not finish')
            def feedback(packet,received):
                nonlocal wire_count
                data=struct.pack('>Q',received)+packet
                adapter.accept(data);assert clients[0].send(data)==len(data);wire_count+=1
                return wait(lambda state:state['native_feedback']['frames']==wire_count)
            def pose(frame,t,x=0.,confidence=3,session=None):
                nonlocal pose_count,latest_raw
                p=dict(schema=1,session_id=session or sid,frame_number=frame,timestamp_monotonic_ns=t,
                    receive_monotonic_ns=t+1000,x_m=x,y_m=0.,yaw_rad=0.,tracker_confidence=confidence,
                    mapper_confidence=1,travel_from_start_m=abs(x))
                data=json.dumps(p,separators=(',',':')).encode();assert clients[1].send(data)==len(data)
                if session is not None and session!=sid:return p
                original.ingest_pose(p);latest_raw=p;pose_count+=1
                wait(lambda state:state['pose_ingested']==pose_count)
                return p
            def expected_packet():
                return format_control_packet(original.control_snapshot(),latest_raw,sid,len(packets),last_ready,
                    lidar_state=dict(lidar_connected=False,lidar_error=None))
            def receive():
                nonlocal last_ready
                packet=json.loads(terminal.recv(16384));expected=expected_packet()
                assert expected is not None;equivalent(expected,packet)
                assert packet['packet_id']==len(packets)
                packets.append(packet)
                if packet['quality']!='LOST':last_ready=packet
                return packet
            def no_packet():
                terminal.settimeout(.03)
                try:terminal.recv(16384);raise AssertionError('unchanged input duplicated a control packet')
                except socket.timeout:pass
                finally:terminal.settimeout(1.)
            def gyro(index,tick):
                return wire.packet(wire.GYRO,index,struct.pack('>16sIIqQIIIiQIIIQ',epoch,1,index,
                    12345,tick,index,835,0xffffffff,-1000,tick,index,835,0xffffffff,tick+100))
            def wheel(index,start,end):
                dt=(end-start)/1e6;v=math.pi*.070/1061.268*20/(math.sqrt(3)*dt)
                return wire.packet(wire.WHEEL,index,struct.pack('>16sIIQQiiIIIiiiIII',epoch,1,index,start,end,
                    round(v*1e6),0,10,20,30,10,-10,0,129,0xffffffff,3))
            tick0=1_010_000;tick1=tick0+10000
            host0=clock.map_time(tick0,2_020_000_000)[0]
            host1=clock.map_time(tick1,2_030_000_000)[0]
            feedback(gyro(1,tick0),2_020_000_000)
            pose(0,host0)
            no_packet()  # No field reference, no invented ready control pose.
            reference=original.align('socket fixture known reference',physical_confirmed=False,
                center_xy=(.2,-.3),heading_deg=math.degrees(.7))
            runtime.command('field_reference',dict(reference=reference,revision=original.revision,has_anchor=False))
            receive()
            packet=wheel(1,tick0,tick1)
            state=feedback(packet,2_030_000_000);receive()
            assert state['native_feedback']['wheel_frames']==1
            measured=next(p for kind,p in forwarded if kind=='wheel')
            assert measured['measurement_valid'] and not measured['capture_time_qualified']
            assert measured['mcu_epoch']==epoch.hex()  # Unknown drift is not an admission gate.
            # A real1mm position refinement changes the original publish key;
            # a Pose exactly matching the wheel tail need not send a packet.
            actual_pose_x=measured['v_body_mps'][0]*.01+.001
            pose(1,host1,x=actual_pose_x);receive()
            pose(2,host1+5_000_000,x=actual_pose_x,confidence=0)
            lost_tracking=receive()
            assert lost_tracking['motion_source']=='WHEEL_FALLBACK' and lost_tracking['t265']['tracker_confidence']==0
            feedback(packet,2_031_000_000);no_packet()  # Same real sample ID, not a new endpoint.
            pose(2,host1+5_000_000,x=actual_pose_x,confidence=0);no_packet()
            # A Pose from another device session cannot enter this configured
            # socket, while the active session stays usable.
            before=runtime.snapshot();pose(99,host1+6_000_000,session='another-session');no_packet()
            after=runtime.snapshot();assert after['pose_ingested']==before['pose_ingested']
            original.invalidate_reference('fixture explicit invalidation')
            runtime.command('invalidate',dict(reason='fixture explicit invalidation',revision=original.revision))
            stale=receive();assert stale['quality']=='LOST' and stale['pose']==last_ready['pose']
            runtime.command('context',context);no_packet()
            state=runtime.snapshot(dict(kind='histories'))
            assert state['native_feedback']['decode_errors']==0
            assert state['native_feedback']['frames']==wire_count
            assert state['direct_output']['packets_sent']==len(packets) and state['direct_output']['output_drops']==0
            assert [p['packet_id']for p in packets]==list(range(len(packets)))
            runtime.close()
            final=runtime.snapshot()
            assert all(v['dropped']==0 and not v['failed']for v in final['logging'].values())
            native_logged=[json.loads(line)for line in(Path(temporary)/'motion_feedback.jsonl').read_text().splitlines()]
            assert native_logged==[value for name,value in logged if name=='motion_feedback.jsonl']
            native_controls=[json.loads(line)for line in(Path(temporary)/'control_pose.jsonl').read_text().splitlines()]
            assert native_controls==packets  # Actual successful sends, not IPC intent.
        finally:
            if runtime is not None:runtime.close()
            for sock in owned:sock.close()


def test_motion_lab_stdio():
    """Only explicit local commands; no SDK/UART/GUI hardware in this check."""
    import io
    import os
    import socket
    import uuid
    from contextlib import redirect_stderr,redirect_stdout
    from unittest.mock import patch
    import t265_f407_motion_map as module
    from f405_test_client import ChassisTestClient
    def options(root,*extra):
        with patch('sys.argv',['map','--motion-lab-stdio','--output-root',str(root),*extra]):
            return module.arguments()
    for extra in (['--wheel-experiment'],['--motion-lab-chassis-only','--wheel-input','on'],
                  ['--sensor-check-seconds','1'],['--wheel-fixture','axes','--wheel-input','on']):
        with redirect_stderr(io.StringIO()):
            try:options(Path('/unused'),*extra);assert False,'ambiguous lab mode accepted'
            except SystemExit as error:assert error.code==2
    class FakeSerial:
        def __init__(self):self.written=[]
        def write(self,data):self.written.append(data);return len(data)
        def close(self):pass
    transports=[]
    def start_client(client):
        client.serial=FakeSerial();transports.append(client.serial)
    with tempfile.TemporaryDirectory() as temporary, \
         patch.object(module.MotionMapApp,'detect_screen_size',return_value=(1024,768)), \
         patch.object(module.subprocess,'check_output',return_value='offline'), \
         patch('low_latency_scheduling.prepare_realtime',return_value={'fixture_only':True}), \
         patch.object(ChassisTestClient,'start',start_client):
        root=Path(temporary)
        opt=options(root,'--motion-lab-chassis-only','--baud','115200','--chassis-profile','legacy',
                    '--actual-lower-service','MOTIONDEBUG','--reference-description','independent floor marks')
        app=module.MotionMapApp(opt)
        with patch.object(module.subprocess,'Popen',side_effect=AssertionError('chassis-only opened SDK')), \
             patch.object(module,'write_sdk_config',side_effect=AssertionError('chassis-only configured SDK')):
            app.start_session()
        transport=transports[-1]
        assert transport.written==[] and app.localizer is None and app.wheel_input is None
        assert app.motion_lab_status()['sdk'] is None and app.motion_lab_status()['current_pose'] is None
        metadata=json.loads((app.run_dir/'metadata.json').read_text())
        assert metadata['sdk_owner_started'] is False and metadata['sdk_session_id'] is None
        app.bridge.feed(inbound(MSG_MOTION_STATUS,0,bytes(8)),time.monotonic_ns())
        move=app.motion_lab_command(dict(op='move',forward_mm=301,left_mm=-43,speed_mm_s=200))
        assert move['submitted'] and move['lower_accepted'] is None
        packet=transport.written[-1]
        assert int.from_bytes(packet[6:8],'big',signed=True)==301
        assert int.from_bytes(packet[8:10],'big',signed=True)==-43
        written=len(transport.written)
        try:app.motion_lab_command(dict(op='move',forward_mm=10,left_mm=0));assert False
        except RuntimeError:pass
        assert len(transport.written)==written  # repeated command cannot become implicit STOP
        def finish(state=2):
            pending=app.bridge.pending
            app.bridge.feed(inbound(MSG_MOTION_STATUS,1,bytes((pending['wire_sequence'],state,0,pending['opcode']))+bytes(4)),
                            time.monotonic_ns()+1_000_000)
            app.update_data()
        finish()
        assert app.active_test is None and app.bridge.pending is None
        app.motion_lab_command(dict(op='mark'))
        app.motion_lab_command(dict(op='ground_truth',text='300,-40,?'))
        truth=json.loads((app.run_dir/'ground_truth.jsonl').read_text())
        assert truth['localization']=={} and truth['pose_sample_monotonic_ns'] is None
        assert truth['pose_age_at_mark_ms'] is None and truth['pose_source']=='not_started_chassis_only'
        turn=app.motion_lab_command(dict(op='turn',angle_deg=90,rate_deg_s=60))
        assert turn['submitted'] and int.from_bytes(transport.written[-1][6:8],'big',signed=True)==-900
        app.motion_lab_command(dict(op='stop'));finish(4)
        # Existing SDK experiment endpoint: exact bytes, no fabricated ACK or second input producer.
        sid=uuid.uuid4().hex;address='\0t265-wheel-'+sid
        app.sdk_session_id=sid;app.options.wheel_experiment=True
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as listener:
            listener.bind(address);listener.settimeout(.3)
            response=app.motion_lab_command(dict(op='experiment',phase_id='known_bias',mode='bias_body',
                duration_ns=500_000_000,bias_body_mps=[.1,0,0]))
            received=json.loads(listener.recv(65536))
            assert received==response['request'] and received['session_id']==sid
            assert response['sdk_applied'] is None and received['kind']=='wheel_input_experiment'
        app.options.wheel_experiment=False
        try:app.motion_lab_command(dict(op='experiment',mode='drop',phase_id='disabled',duration_ns=1));assert False
        except ValueError:pass
        with patch.object(module.time,'sleep'):
            app.stop_session()
        assert (app.run_dir/'session_complete.json').is_file()
        assert not (app.run_dir/'t265_raw_pose.csv').exists() and not (app.run_dir/'localization_debug.csv').exists()
        # The stdio loop survives malformed/unknown input without motions or GUI startup.
        read_fd,write_fd=os.pipe()
        os.write(write_fd,b'not-json\n{"op":"unknown"}\n{"op":"status","request_id":7}\n{"op":"close"}\n')
        os.close(write_fd)
        second=module.MotionMapApp(opt);output=io.StringIO()
        with os.fdopen(read_fd,'r') as incoming, patch.object(module.sys,'stdin',incoming), \
             redirect_stdout(output),patch.object(module.cv2,'namedWindow',side_effect=AssertionError('GUI started')):
            assert second.run()==0
        replies=[json.loads(row) for row in output.getvalue().splitlines()]
        assert [r['ok'] for r in replies]==[True,False,False,True,True]
        assert replies[3]['request_id']==7 and transports[-1].written==[]
        assert (second.run_dir/'session_complete.json').is_file()
        # A and F preserve all original SDK flags; only A loads real input/profile support.
        for wheel,experiment in [('off',False),('on',True)]:
            opt=options(root,'--wheel-input',wheel,*(['--wheel-experiment'] if experiment else []))
            current=module.MotionMapApp(opt)
            child=SimpleNamespace(pid=123,poll=lambda:0,returncode=0)
            with patch.object(module.subprocess,'Popen',return_value=child):
                current.start_session()
            metadata=json.loads((current.run_dir/'metadata.json').read_text());command=metadata['localizer_command']
            assert all(flag in command for flag in ['--enable-pose-jumping','--t265-only-output','--ignore-encoders','--debug-fresh-session'])
            assert ('--debug-wheel-socket' in command)==(wheel=='on')
            assert ('--debug-wheel-experiment' in command)==experiment
            assert metadata['wheel_input_requested']==(wheel=='on')
            current.stop_session()


def test_offline_wheel_gyro_residual():
    """Known raw packets: numerical/coverage checks, no slip labels or hardware."""
    import math
    from wheel_gyro_residual import analyze_records,analyze_session,export_report,METRES_PER_RAW_COUNT
    def wheel(i,a,b,d,epoch='a'):
        return dict(type=7,boot_epoch=epoch,reference_epoch=1,measurement_id=i,
            t0_us=a,t1_us=b,delta1=d[0],delta2=d[1],delta3=d[2],flags=129)
    def gyro(i,t,yaw=0,rate=0,epoch='a'):
        return dict(type=8,boot_epoch=epoch,reference_epoch=1,
            yaw_source_id=i,yaw_receive_us=t,yaw_mdeg=yaw,yaw_flags=835,
            rate_source_id=i,rate_receive_us=t,rate_mdps=rate,rate_flags=835)
    endpoints=[gyro(0,0),gyro(1,10000)]
    for sign in (-1,1):
        _,rows=analyze_records(endpoints+[wheel(1,0,10000,[sign]*3)])
        raw=rows[0]
        expected=math.degrees(sign*METRES_PER_RAW_COUNT/.13)/.01
        assert math.isclose(raw['residual_yaw_deg_s'],expected,abs_tol=1e-12)
        assert math.isclose(raw['residual_rate_deg_s'],expected,abs_tol=1e-12)
    _,rows=analyze_records(endpoints+[wheel(1,0,10000,[1,0,0])])
    assert math.isclose(rows[0]['encoder_omega_deg_s'],math.degrees(METRES_PER_RAW_COUNT/(3*.13))/.01)
    # A wall-blocked translational wheel pattern may have exactly zero yaw
    # residual. It cannot be certified as no-slip from this comparison.
    _,rows=analyze_records(endpoints+[wheel(1,0,10000,[10,-10,0])])
    assert rows[0]['residual_yaw_deg_s']==rows[0]['residual_rate_deg_s']==0 and rows[0]['vx_m_s']>0
    records=[gyro(0,0,0,0),gyro(1,20000,40,4000),wheel(1,5000,15000,[0,0,0])]
    report,rows=analyze_records(records+[records[0],records[-1]])
    assert math.isclose(rows[0]['yaw_delta_deg'],.02,abs_tol=1e-12)
    assert math.isclose(rows[0]['rate_integral_deg'],.02,abs_tol=1e-12)
    assert math.isclose(rows[0]['residual_yaw_deg_s'],-2.,abs_tol=1e-12)
    assert report['counts']['duplicate_wheel_id']==report['counts']['duplicate_rate_source_id']==1
    for packet in (wheel(1,-1,10000,[0,0,0]),wheel(1,0,30000,[0,0,0]),wheel(1,5000,15000,[0,0,0],epoch='b')):
        _,rows=analyze_records(records[:2]+[packet])
        assert rows[0]['yaw_delta_deg'] is None and rows[0]['rate_integral_deg'] is None
    packets=[gyro(i,i*10000) for i in range(13)]
    packets += [wheel(i if i<=6 else i+1,(i-1)*10000,i*10000,[0,0,0]) for i in range(1,13)]
    report,rows=analyze_records(packets)
    assert report['counts']['wheel_continuity_breaks']==1
    assert all(not (row['first_wheel_id']<=6 and row['last_wheel_id']>=8) for row in rows)
    assert sum(row['interval_count'] for row in rows if row['window']=='50ms')==12
    assert sum(row['interval_count'] for row in rows if row['window']=='100ms')==12
    timed_gap=[wheel(i,(i-1)*10000+(10000 if i>6 else 0),i*10000+(10000 if i>6 else 0),[0,0,0])
               for i in range(1,13)]
    report,rows=analyze_records(packets[:13]+timed_gap)
    assert report['counts']['wheel_continuity_breaks']==1
    assert all(not (row['t0_us']<70000 and row['t1_us']>60000) for row in rows)
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary);session=root/'session';session.mkdir()
        original='\n'.join(json.dumps(packet) for packet in records)+'\nnot-json\n'
        (session/'chassis_extended.jsonl').write_text(original)
        report,rows=analyze_session(session)
        assert report['counts']['bad_json_lines']==1 and report['source_file']=='chassis_extended.jsonl'
        for output in (session,session/'nested',root):
            try:export_report(session,output,report,rows)
            except ValueError:pass
            else:raise AssertionError('Original session/ancestor output was accepted')
        exported=export_report(session,root/'export',report,rows)
        assert all(path.is_file() for path in exported)
        assert (session/'chassis_extended.jsonl').read_text()==original
        # The existing entry must return from the explicit offline branch,
        # without reaching its live capture/device path.
        import io
        from contextlib import redirect_stdout
        from unittest.mock import patch
        import t265_f407_debug as debug
        output=io.StringIO()
        with patch('sys.argv',['debug','--wheel-gyro-residual',str(session),'--wheel-arm-m','.14']), \
                patch.object(debug,'run_capture',side_effect=AssertionError('live capture')), \
                redirect_stdout(output):
            assert debug.main()==0
        assert json.loads(output.getvalue())['model']['effective_wheel_arm_m']==.14


def test_offline_motion_source_candidates():
    """Tilted rigid-body cancellation, coverage and offline CLI isolation."""
    import math
    import io
    import numpy as np
    from contextlib import redirect_stdout
    from unittest.mock import patch
    from motion_source_review import _History, _integrated, _sdk_velocity, _review_scans, export_review
    import t265_f407_debug as debug
    f=np.array([math.sqrt(3)/2,0.,.5]);u=np.array([0.,-1.,0.])
    lever=-.0346105*f-.0599471445*np.cross(u,f)
    matrices=[np.eye(3),np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]]),
              np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])]
    quaternions=[(0.,0.,0.,1.),(0.,math.sqrt(.5),0.,math.sqrt(.5)),
                 (math.sqrt(.5),0.,0.,math.sqrt(.5))]
    raw=[]
    for rotation,q in zip(matrices,quaternions):
        omega=np.array([.2,1.1,-.4]);v=np.cross(omega,rotation@lever)+[1.,0.,0.]
        raw.append(dict(zip(('qx','qy','qz','qw'),q),
            **dict(zip(('wx_radps','wy_radps','wz_radps'),omega)),
            **dict(zip(('vx_mps','vy_mps','vz_mps'),v))))
    result=_sdk_velocity(raw,f,u,[-.0346105,-.0599471445],0.)
    assert np.allclose(result,[[math.sqrt(3)/2,.5]]*3,atol=1e-12)
    h=_integrated([0,100_000_000,300_000_000],[[0.,0.],[2.,0.],[4.,0.]])
    assert np.allclose(h.at([300_000_000]),[[.7,0.]])
    broken=_History([0,10,20],[[0.,0.],[1.,0.],[2.,0.]],[10])
    assert not broken.covers(0,20) and broken.covers(10,20)
    assert not broken.covers(-1,10) and not broken.covers(10,21)
    # An arbitrary SDK startup-heading difference must cancel from the
    # within-scan increment candidate, while the main field reference stays.
    from collections import Counter
    ticks=np.arange(33,dtype=np.int64)*1_250_000+1_000_000_000
    theta=np.linspace(0.,.1,33);zero=_History(ticks,np.zeros((33,2)))
    main=_History(ticks,theta);sdk_yaw=_History(ticks,theta+.4)
    points=[]
    for i,(t,yaw) in enumerate(zip(ticks,theta)):
        world=np.array([1.,-.6+i*.0375]);c,s=math.cos(yaw),math.sin(yaw)
        point=np.array([[c,s],[-s,c]])@world
        points.append(dict(time_ns=int(t),point_lidar_m=point.tolist(),valid_return=True,raw_angle_deg=0.))
    scans=[dict(scan_id='known-wall',reference_ns=int(ticks[-1]),points=points)]
    mount=dict(extrinsic=[0.,0.,0.],excluded_raw_ranges_deg=[],center_raw_deg=0.,fov_deg=360.)
    reference=dict(vio_rotation=0.,pose_origin=[0.,0.],start_center_xy_m=[0.,0.])
    metrics=_review_scans(scans,mount,zero,zero,zero,zero,main,sdk_yaw,0,0.,reference,Counter())
    by_model={r['model']:r for r in metrics}
    assert by_model['wheel_sdk_increment']['axis_rms_m']<1e-12
    assert by_model['wheel_sdk_increment_aligned']['axis_rms_m']<1e-12
    assert by_model['wheel_sdk_rotation']['axis_rms_m']>.05
    with tempfile.TemporaryDirectory() as directory:
        root=Path(directory);session=root/'source';session.mkdir()
        original=session/'keep';original.write_text('raw evidence')
        report=dict(runtime_connected=False,deployed=False)
        result=(report,[],[],[])
        for target in (session,session/'child',root):
            try:export_review(session,target,result)
            except ValueError:pass
            else:raise AssertionError('Source/ancestor accepted as export target')
        paths=export_review(session,root/'review',result)
        assert all(p.exists() for p in paths) and original.read_text()=='raw evidence'
        output=io.StringIO()
        with patch('sys.argv',['debug','--motion-source-review',str(session)]), \
             patch('motion_source_review.review_session',return_value=result), \
             patch.object(debug,'run_capture',side_effect=AssertionError('device capture')), \
             redirect_stdout(output):
            assert debug.main()==0
        assert json.loads(output.getvalue())==report


def test_motion_response_offline():
    """Independent known lag and rigid-body cases; no device access."""
    import math
    from wheel_gyro_residual import project_raw_pose, response_lag
    forward=[.8660254,0,.5];up=[0,-1,0];left=[-.5,0,.8660254]
    offset=[-.0346105,-.0599471445]
    lever=[offset[0]*forward[i]+offset[1]*left[i] for i in range(3)]
    omega=up
    camera_v=[omega[1]*lever[2]-omega[2]*lever[1],
              omega[2]*lever[0]-omega[0]*lever[2],
              omega[0]*lever[1]-omega[1]*lever[0]]
    raw=dict(qx=0,qy=0,qz=0,qw=1,wx_radps=omega[0],wy_radps=omega[1],wz_radps=omega[2],
             vx_mps=camera_v[0],vy_mps=camera_v[1],vz_mps=camera_v[2])
    projected=project_raw_pose(raw,forward,up,offset)
    assert abs(projected['yaw_rate_rad_s']-1)<1e-12
    assert max(abs(projected[k]) for k in ('forward_mps','left_mps','up_mps'))<1e-12
    assert response_lag([(i*.01,0,.01) for i in range(200)],[],minimum_excursion=.02)['identifiable'] is False
    def pulse(t):return math.exp(-((t-1)/.06)**2)+.55*math.exp(-((t-2.1)/.10)**2)
    def integral(t):return math.sqrt(math.pi)/2*(.06*math.erf((t-1)/.06)+.055*math.erf((t-2.1)/.10))
    reference=[(i*.01,(integral(i*.01)-integral(i*.01-.01))/.01,.01) for i in range(30,281)]
    sdk=[(i*.005,pulse(i*.005-.04)) for i in range(701)]
    result=response_lag(reference,[sdk],minimum_excursion=.02)
    assert result['identifiable'] and abs(result['peak']['lag_ms']-40)<=5,result
