#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protocol import (  # noqa: E402
    CARGO_MIXED_MATERIAL,
    CMD_CARGO_AUDIT,
    CMD_VALID,
    CargoAuditPayload,
    FRAME_SIZE,
    crc16_modbus,
    mission_frame,
)


def test_bulldozer_step_contract_roundtrips_without_legacy_flag_aliases():
    from obstacle_route import ObstacleRoute
    from protocol import CMD_BULLDOZER_STEP
    for side in ('red', 'blue'):
        route = ObstacleRoute(side, (1,), ('green_supply',), 'material', (-.15, .54),
                              (-.15, .54), ('danger_cyan',), 'sweep')
        for index in range(len(route.steps)):
            route.index = index
            payload = route.payload(0, .5, pose_valid=True)
            flags = 1 | (8 if side == 'red' else 0) | (int(route.step) << 4)
            packet = mission_frame(7, CMD_BULLDOZER_STEP, flags, *payload)
            assert packet[4] == 0x14 and packet[5] == flags
            assert int.from_bytes(packet[6:8], 'big', signed=True) == payload[0]
            assert int.from_bytes(packet[8:10], 'big', signed=True) == payload[1]
            assert int.from_bytes(packet[10:12], 'big') == payload[2]
            assert int.from_bytes(packet[12:14], 'little') == crc16_modbus(packet[2:12])
    for flags, args in ((0xE9, (0,0,0)), (0x0B, (0,0,0)),
                        (0x19, (201,0,0)), (0x59, (10,0,9000))):
        try:
            mission_frame(1, CMD_BULLDOZER_STEP, flags, *args)
        except ValueError:
            continue
        raise AssertionError('invalid new step request accepted')


def main() -> None:
    from protocol import CMD_APPROACH_TARGET, CMD_APPROACH_RANGE, CMD_DISTANCE_VALID, CMD_RED_SIDE, CMD_CLUSTER_TARGET
    flags = CMD_VALID | CMD_APPROACH_RANGE | CMD_DISTANCE_VALID | CMD_RED_SIDE | CMD_CLUSTER_TARGET
    for distance in (0, 2500):
        request = mission_frame(3, CMD_APPROACH_TARGET, flags, distance, 10500, 0)
        assert int.from_bytes(request[6:8], "big") == distance
        assert int.from_bytes(request[8:10], "big") == 10500 and request[10:12] == b"\0\0"
    for distance, pitch, extra in ((-1,10500,0),(2501,12000,0),(100,14000,0),(100,10500,2)):
        try:
            mission_frame(3, CMD_APPROACH_TARGET, flags | extra, distance, pitch, 0)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid range request accepted")
    packet = mission_frame(7, CMD_CARGO_AUDIT, CMD_VALID, 1, 6, 0x1234)
    assert len(packet) == FRAME_SIZE
    assert packet[:4] == bytes((0xA3, 0xB3, 0x18, 7))
    assert packet[4] == CMD_CARGO_AUDIT
    expected = crc16_modbus(packet[2:12])
    assert packet[12:14] == expected.to_bytes(2, "little")
    assert packet[-1] == 0xC3

    audit = CargoAuditPayload(
        left_class="mixed_material",
        right_class="danger_cyan",
        left_count=2,
        right_count=1,
        total_count=3,
        danger_present=True,
        stable=True,
        audit_id=9,
    )
    audit_packet = audit.frame(8)
    assert audit_packet[4] == CMD_CARGO_AUDIT
    assert audit_packet[6] == CARGO_MIXED_MATERIAL
    assert audit_packet[7] == 4
    assert audit_packet[8] == 0x06  # left=2, right=1
    assert audit_packet[9] & (1 << 1)  # danger flag
    assert audit_packet[9] & (1 << 4)  # stable flag
    assert audit_packet[10] == 9
    assert audit_packet[11] == 3
    print("competition protocol PASS")


if __name__ == "__main__":
    main()


def test_empty_body_move_extension_keeps_field_translation_and_chassis_heading_separate():
    from protocol import CMD_BULLDOZER_STEP, command_context_frame
    for flags,u in ((0xD9,27000),(0xD1,9000),(0xD9,28200)):
        packet=mission_frame(9,CMD_BULLDOZER_STEP,flags,100,-9000,u)
        assert packet[4:6]==bytes((0x14,flags))
        assert int.from_bytes(packet[6:8],'big',signed=True)==100
        assert int.from_bytes(packet[8:10],'big',signed=True)==-9000
        assert int.from_bytes(packet[10:12],'big')==u
        assert packet[12:14]==crc16_modbus(packet[2:12]).to_bytes(2,'little')
        assert command_context_frame(9,11,12,0)[3]==packet[3]


def test_attack_face_opponent_extension_keeps_old_sweep_turns_fixed():
    from protocol import CMD_BULLDOZER_STEP
    for flags, heading in ((0xE9, 27000), (0xE1, 9000)):
        packet = mission_frame(9, CMD_BULLDOZER_STEP, flags, 0, 0, heading)
        assert int.from_bytes(packet[10:12], "big") == heading
        assert packet[12:14] == crc16_modbus(packet[2:12]).to_bytes(2, "little")
        for wrong in (0, 18000, (heading + 18000) % 36000):
            try:
                mission_frame(9, CMD_BULLDOZER_STEP, flags, 0, 0, wrong)
            except ValueError:
                pass
            else:
                raise AssertionError("wrong attack-facing heading accepted")
    # Existing step0/4 retain their original red/blue sweep headings.
    for flags, heading in ((0x09, 0), (0x01, 18000), (0x49, 9000), (0x41, 27000)):
        mission_frame(1, CMD_BULLDOZER_STEP, flags, 0, 0, heading)


def test_reused_post_grab_audit_flag_only_extends_stage_nav():
    from protocol import CMD_NAVIGATE_WAYPOINT, CMD_REUSE_POST_GRAB_AUDIT, CMD_STAGE_ONLY, CMD_DISTANCE_VALID
    for red in (0,8):
        flags = CMD_VALID | CMD_STAGE_ONLY | CMD_DISTANCE_VALID | CMD_REUSE_POST_GRAB_AUDIT | red
        for distance in (0,245):
            packet = mission_frame(9, CMD_NAVIGATE_WAYPOINT, flags, distance, 0, 27000)
            assert packet[5] == flags and packet[12:14] == crc16_modbus(packet[2:12]).to_bytes(2,'little')
        for bad_flags in (flags & ~CMD_STAGE_ONLY, flags & ~CMD_DISTANCE_VALID):
            try:
                mission_frame(9, CMD_NAVIGATE_WAYPOINT, bad_flags, 245, 0, 27000)
            except ValueError:
                pass
            else:
                raise AssertionError("audit reuse outside STAGE NAV accepted")


def test_near_claw_separation_flag_has_no_transport_or_motion_payload():
    from protocol import CMD_DISPERSE_PILE,CMD_DISPERSE_NEAR_KEEP,CMD_SIDE_VALID,CMD_TARGET_RIGHT
    for side in (0,CMD_TARGET_RIGHT):
        frame=mission_frame(4,CMD_DISPERSE_PILE,1|CMD_DISPERSE_NEAR_KEEP|CMD_SIDE_VALID|side)
        assert frame[4]==0x10 and frame[5]&0x10 and frame[6:12]==bytes(6)
    for flags,args in ((1|CMD_DISPERSE_NEAR_KEEP,(0,0,0)),
                       (1|CMD_DISPERSE_NEAR_KEEP|CMD_SIDE_VALID,(1,0,0))):
        try:mission_frame(4,CMD_DISPERSE_PILE,flags,*args)
        except ValueError:pass
        else:raise AssertionError('invalid near-claw separation payload')
