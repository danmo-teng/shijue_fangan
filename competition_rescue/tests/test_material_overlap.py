"""Recorded crowded green/core load; no camera, model execution or device IO."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from collections import Counter
import json
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'competition_rescue'),str(ROOT/'vision')]
import run_competition_rescue as runner
from capture_roi import CaptureRois
from state_machine import CompetitionMission,CompetitionSettings,CompetitionState,CargoBatch,TrackedCargo,StmSnapshot
from protocol import CMD_CARGO_AUDIT


def fixture():
    data=json.loads((ROOT/'docs/reports/competition_20261118_localization_audit/material_overlap_replay.json').read_text())
    rois=CaptureRois(*(tuple(map(tuple,data['capture_roi'][key])) for key in
        ['capture_polygon_px','capture_left_polygon_px','capture_right_polygon_px']))
    m=CompetitionMission(CompetitionSettings(side='blue',initial_stash_enabled=False))
    m.first_common_delivered=True
    m._select_batch(CargoBatch((270,),('green_supply',),'material'))
    return m,rois,data['frames']


def items_from(frame):
    return tuple(TrackedCargo(x['id'],x['class'],x['confidence'],tuple(x['bbox']),
        hits=x['hits'],misses=x['misses'],visible=x['visible']) for x in frame['tracks'])


def observe(m,rois,items,mode=23,flags=3,frame=10,now=1.,quality=''):
    with patch.object(runner,'tracked_cargo',return_value=tuple(items)):
        return runner.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),None,
            StmSnapshot(mode=mode,flags=flags,age_ms=0,camera_pitch_cdeg=14000),m,frame,
            capture_rois=rois,observed_s=now,capture_quality_reason=quality)


def test_actual_two_green_core_frames_reach_real_post_grab_and_transport_audit():
    for mode in (23,48):
        m,rois,frames=fixture()
        for f in frames:
            v=observe(m,rois,items_from(f),mode=mode,frame=f['frame_sequence'])
            assert v.capture_observation_valid, v.capture_observation_reason
            assert Counter(dict(v.capture_audit.class_counts))==Counter(green_supply=2,core_black=1)
            assert v.capture_audit.total_count==3 and m._audit_valid(v.capture_audit)
            assert not v.capture_audit.stable  # Real samples still need the original two-frame audit.
            assert m._audit_payload(v.capture_audit).total_count==3


def test_real_fourth_item_or_other_class_is_not_hidden_or_made_legal():
    m,rois,frames=fixture();items=items_from(frames[0])
    for mode in (23,48):
        fourth=TrackedCargo(900,'core_black',.9,(400,300,45,45),hits=5)
        v=observe(m,rois,items+(fourth,),mode=mode)
        assert v.capture_observation_valid and v.capture_audit.total_count==4
        assert not m._audit_valid(v.capture_audit)
        for category in ('danger_cyan','injured_orange'):
            extra=replace(fourth,class_name=category,bbox=(250,490,50,50))
            v=observe(m,rois,items+(extra,),mode=mode)
            assert v.capture_observation_valid and v.capture_audit.total_count==4
            assert not m._audit_valid(v.capture_audit)
    m.first_common_delivered=False
    v=observe(m,rois,items)
    assert v.capture_audit.total_count==3 and not m._audit_valid(v.capture_audit)
    # Missing black stays missing; this exception does not invent the prior third item.
    missing=observe(m,rois,tuple(x for x in items if x.track_id!=281))
    assert missing.capture_audit.total_count==2


def test_reaudit_exception_requires_closed_claws_distinct_established_material_boxes():
    m,rois,frames=fixture()
    items=tuple(x for x in items_from(frames[0]) if x.track_id in (273,281))
    for mode in (23,48):
        for changed in [
            (items[0],replace(items[1],hits=1)),
            (items[0],replace(items[1],track_id=items[0].track_id)),
            (items[0],replace(items[1],bbox=items[0].bbox)),
            (items[0],replace(items[1],class_name='injured_orange')),
            (items[0],replace(items[1],class_name='danger_cyan')),
        ]:
            v=observe(m,rois,changed,mode=mode)
            assert not v.capture_observation_valid and v.capture_audit is None
        assert observe(m,rois,items,mode=mode,flags=1).capture_audit is None
        assert observe(m,rois,items,mode=mode,quality='capture_image_dark').capture_audit is None
    for mode in (43,45,47):
        assert observe(m,rois,items,mode=mode).capture_audit is None
    # Existing grab-time multi-object handling remains available.
    assert observe(m,rois,items,mode=21,flags=1).capture_audit.total_count==2
    full=CaptureRois(((0,0),(1279,0),(1279,1023),(0,1023)),rois.left,rois.right)
    clipped=(replace(items[0],bbox=(0,200,200,200)),replace(items[1],bbox=(20,250,50,50)))
    assert observe(m,full,clipped).capture_observation_reason=='claw_target_truncated'


def test_overlap_repair_does_not_skip_the_existing_stable_sample_requirement():
    m,rois,frames=fixture();items=items_from(frames[0])
    status=StmSnapshot(mode=23,flags=3,age_ms=0,camera_pitch_cdeg=14000)
    m.state=CompetitionState.POST_GRAB_AUDIT;m.post_grab_audit_active=True
    first=observe(m,rois,items,frame=10,now=1.)
    out=m._audit_output(first,status,1.)
    assert out.command.opcode==CMD_CARGO_AUDIT and not out.command.audit.stable
    second=observe(m,rois,items,frame=11,now=1.13)
    out=m._audit_output(second,status,1.13)
    assert out.command.opcode==CMD_CARGO_AUDIT and not out.command.audit.stable
    third=observe(m,rois,items,frame=12,now=1.26)
    out=m._audit_output(third,status,1.26)
    assert out.command.opcode==CMD_CARGO_AUDIT and out.command.audit.stable
    assert out.command.audit.total_count==3
    assert m.state!=CompetitionState.NAVIGATE  # No MCU confirmation was supplied.
