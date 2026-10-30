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
    assert not imu_is_fresh(seen_imu[0],1_159_000_000)
    assert not imu_is_fresh(seen_imu[0],1_009_000_000)
    for flag in (4,8,16,32,64,128):
        d=parse_imu(inbound(MSG_IMU,8,bytes(6)+bytes((1,3|flag))))
        assert not d["wire_valid"]
    assert not parse_imu(inbound(MSG_IMU,8,bytes(6)+bytes((150,3))))["wire_valid"]
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
    assert not w.update(wheel([4,65531,0],3,2_000_000_000))["valid"]
    class FakeSerial:
        def __init__(self): self.written=[]
        def write(self,packet): self.written.append(packet);return len(packet)
        def close(self): pass
    with tempfile.TemporaryDirectory() as temporary:
        root=Path(temporary)
        c=ChassisTestClient("NOT_OPENED",115200,root/"status.json",root/"status.jsonl",identity_confirmed=True)
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
            for method in (c.query_capabilities,lambda:c.move_spin_angle(100,0,90,60),lambda:c.turn_ccw(90)):
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


if __name__ == "__main__":
    main()
