"""Focused pre-grab motion handoff checks; no device IO."""
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / 'vision'))
from capture_grip_bands import GripBands
from state_machine import (CompetitionMission, CompetitionSettings, CompetitionState,
    CargoBatch, CargoAudit, TrackedCargo, VisionSnapshot, StmSnapshot, PoseSnapshot)
from protocol import (CMD_APPROACH_TARGET, CMD_APPROACH_RANGE, CMD_DISTANCE_VALID,
    CMD_VALID, CMD_CARGO_AUDIT, CMD_HOLD, CMD_GRAB_CONFIRMED, mission_frame)
import run_competition_rescue as runner


def capture_fixture(name='green_supply', mode=21, enabled=True):
    m = CompetitionMission(CompetitionSettings(side='blue', initial_stash_enabled=False,
                                               pre_grab_advance_enabled=enabled))
    m.initial_stash_done = True
    m._select_batch(CargoBatch((7,), (name,), 'injury' if name=='injured_orange' else 'material'))
    m.cluster_audit_active = mode != 21
    m._set_state(CompetitionState.CAPTURE_AUDIT, 1.)
    item = TrackedCargo(7, name, .9, (600, 100, 80, 120), hits=3)
    audit = CargoAudit(left_class=name, right_class='none', left_count=1, total_count=1,
                       left_green_count=int(name=='green_supply'))
    view = VisionSnapshot(frame_sequence=10, observed_monotonic_s=1., cargo=(item,),
        capture_audit=audit, pre_grab_configured_classes=(name,), pre_grab_regions=((7,'upper'),))
    status = StmSnapshot(mode=mode, flags=1, age_ms=0, camera_pitch_cdeg=14000,
                        context_valid=True, context_matches=False)
    return m, view, status


def test_grip_band_calibration_is_separate_and_per_class(tmp_path):
    p = tmp_path/'bands.json'
    assert GripBands.load(p).region('green_supply', 0) is None
    bands = GripBands((300,500), (180,420))
    bands.save(p)
    loaded = GripBands.load(p)
    assert loaded == bands
    assert loaded.region('green_supply', 220) == 'upper'
    assert loaded.region('injured_orange', 220) == 'middle'
    assert loaded.region('green_supply', -10) == 'upper'
    assert loaded.region('green_supply', 300) == 'middle'
    assert loaded.region('green_supply', 500) == 'lower'
    assert loaded.region('core_black', 100) is None
    with pytest.raises(ValueError): GripBands((400,300), None)
    with pytest.raises(ValueError): GripBands((300,300), None)


def test_pre_grab_motion_requires_owned_done_and_post_motion_picture():
    for name in ('green_supply','injured_orange'):
        for mode in (21,38,37):
            m, view, status = capture_fixture(name, mode)
            out = m.step(view, PoseSnapshot(valid=True), status, 1.)
            assert out.state == CompetitionState.PRE_GRAB_ADVANCE
            assert (out.command.opcode,out.command.arg_a,out.command.arg_b,out.command.aux)==(9,50,14000,2)
            command, epoch = out.command, m.range_epoch
            done = replace(status, context_matches=True, accepted_command=9, action_status=3)
            for wrong in (replace(done, mode=53), replace(done, context_matches=False),
                          replace(done, accepted_command=CMD_CARGO_AUDIT), replace(done, action_status=1)):
                assert m.step(replace(view,frame_sequence=11),PoseSnapshot(),wrong,1.1).command == command
                assert m.pre_grab_completed_frame is None and m.range_epoch == epoch
            out = m.step(replace(view,frame_sequence=12),PoseSnapshot(),done,1.2)
            assert out.reason == 'pre_grab_advance_reobserve'
            assert m.step(replace(view,frame_sequence=12),PoseSnapshot(),done,1.3).command == command
            missing = replace(view,frame_sequence=13,cargo=(),pre_grab_regions=(),capture_audit=None)
            out = m.step(missing,PoseSnapshot(),done,1.4)
            assert out.command.opcode == CMD_HOLD and m.range_epoch == epoch
            deeper = replace(view,frame_sequence=14,observed_monotonic_s=1.5,pre_grab_regions=((7,'middle'),))
            out = m.step(deeper,PoseSnapshot(),done,1.5)
            assert out.command.opcode == CMD_CARGO_AUDIT and not out.command.audit.stable
            assert m.state == CompetitionState.CAPTURE_AUDIT
            # Still high on a genuinely new image may request a new segment,
            # but no prior audit may trigger GRAB before that segment.
            out = m.step(replace(view,frame_sequence=15,observed_monotonic_s=1.6),PoseSnapshot(),done,1.6)
            assert out.command.opcode == CMD_APPROACH_TARGET and m.range_epoch == epoch+1
            recovery = m.step(view,PoseSnapshot(),replace(done,mode=24),1.7)
            assert recovery.command.opcode == CMD_HOLD and m.pre_grab_command is None


def test_pre_grab_checks_final_confirmation_but_not_closed_cargo_or_unmarked():
    m, view, status = capture_fixture()
    middle = replace(view,pre_grab_regions=((7,'middle'),))
    m._audit_output(middle,status,1.)
    m._audit_output(replace(middle,frame_sequence=11,observed_monotonic_s=1.2),status,1.2)
    assert m.state == CompetitionState.AUDIT_CONFIRM
    out = m._audit_confirmation_output(replace(view,frame_sequence=12),status,1.3)
    assert out.state == CompetitionState.PRE_GRAB_ADVANCE and out.command.opcode != CMD_GRAB_CONFIRMED
    for change in ('disabled','unmarked','closed','recheck','post_grab'):
        m, view, status = capture_fixture(enabled=change!='disabled')
        if change=='unmarked': view=replace(view,pre_grab_configured_classes=(),pre_grab_regions=())
        if change=='closed': status=replace(status,flags=3)
        if change=='recheck': m.cargo_recheck_pending=True
        if change=='post_grab': m.post_grab_audit_active=True
        assert m._pre_grab_depth_output(view,status,1.) is None


def test_runner_classifies_only_current_140_boxes_and_retains_action_identity(tmp_path):
    from capture_roi import CaptureRois
    m, view, status = capture_fixture()
    roi = CaptureRois(((0,0),(1279,0),(1279,1023),(0,1023)), (), ())
    items = (view.cargo[0], replace(view.cargo[0],track_id=8,class_name='injured_orange'))
    bands = GripBands((300,500),(120,420))
    with patch.object(runner,'tracked_cargo',return_value=items):
        snap = runner.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),None,status,m,10,
            capture_rois=roi,grip_bands=bands,observed_s=1.)
        assert snap.pre_grab_regions == ((7,'upper'),(8,'middle'))
        wrong_pitch = runner.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),None,
            replace(status,camera_pitch_cdeg=12000),m,11,capture_rois=roi,grip_bands=bands)
        assert wrong_pitch.pre_grab_regions == ()
        bad_image = runner.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),None,
            status,m,11,capture_rois=roi,grip_bands=bands,capture_quality_reason='bad_image')
        assert bad_image.pre_grab_regions == ()
    planner = runner.CompetitionPlanner(m,tmp_path/'pose',tmp_path/'stm',tmp_path/'command',
        tmp_path/'diag',runner.JsonlLog(tmp_path/'events'),50)
    out = planner._output_for_cycle(view,PoseSnapshot(valid=True),status,False,1.)
    planner._publish(out)
    ids=(planner.protocol_task,planner.protocol_action)
    acknowledged=replace(status,task_id=ids[0],action_id=ids[1],accepted_command=9,action_status=3)
    # Old action DONE cannot finish this movement.
    old=replace(acknowledged,action_id=(ids[1]-1)&65535)
    out=planner._output_for_cycle(replace(view,frame_sequence=11),PoseSnapshot(),old,False,1.1)
    planner._publish(out)
    assert (planner.protocol_task,planner.protocol_action)==ids and m.pre_grab_completed_frame is None
    out=planner._output_for_cycle(replace(view,frame_sequence=12),PoseSnapshot(),acknowledged,False,1.2)
    planner._publish(out)
    assert planner.protocol_action==ids[1]
    out=planner._output_for_cycle(replace(view,frame_sequence=13),PoseSnapshot(),acknowledged,False,1.3)
    planner._publish(out)
    assert planner.protocol_task==ids[0] and planner.protocol_action!=ids[1]


def test_pre_grab_protocol_only_allows_50mm_at_140_with_aux2():
    for side in (0,8):
        for cluster in (0,32):
            flags=CMD_VALID|CMD_DISTANCE_VALID|CMD_APPROACH_RANGE|side|cluster
            mission_frame(7,CMD_APPROACH_TARGET,flags,50,14000,2)
            for distance,pitch,aux in ((0,14000,2),(100,14000,2),(50,12000,2),(50,14000,1),(50,14000,3)):
                with pytest.raises(ValueError): mission_frame(7,CMD_APPROACH_TARGET,flags,distance,pitch,aux)
            mission_frame(7,CMD_APPROACH_TARGET,flags,460,12000,0)
            mission_frame(7,CMD_APPROACH_TARGET,flags,460,12000,1)


@pytest.mark.parametrize("selector", ("button", "key"))
def test_editor_saves_green_orange_lines_without_changing_roi(tmp_path, selector):
    import numpy as np
    from contextlib import ExitStack
    from unittest.mock import Mock
    import edit_capture_roi as editor
    from capture_roi import CaptureRois, save_capture_rois
    roi_path=tmp_path/'capture_roi.json'
    save_capture_rois(roi_path,CaptureRois(((200,200),(1000,200),(1000,800),(200,800)),(),()))
    original_roi=roi_path.read_bytes()
    args=SimpleNamespace(vision_fps=50.,model=None,labels=None,score_thres=.1,nms_thres=.45,
        priority=0,bpu_cores=[0],preprocess='cpu',decoder='software',device='unused',
        camera_fps=30,decode_fps=30.,output=roi_path,grip_bands=tmp_path/'bands.json')
    camera=Mock()
    camera.check_error.return_value=None
    camera.latest.return_value=SimpleNamespace(frame_id=1,pixel_format='bgr',
        image=np.zeros((1024,1280,3),dtype=np.uint8))
    detector=Mock();detector.infer.return_value=([],None)
    mouse={};frames=[];step=[0]
    def click(x,y):
        mouse['callback'](editor.cv2.EVENT_LBUTTONDOWN,x,y,0,None)
        mouse['callback'](editor.cv2.EVENT_LBUTTONUP,x,y,0,None)
    def key(_):
        n=step[0];step[0]+=1
        if n in (0,2):
            if selector=='key': return ord('g' if n==0 else 'o')
            click(300 if n==0 else 540,editor.DISPLAY_HEIGHT+90)
            return 255
        if n==1:
            click(500,225);click(500,375)  # Full-image y=300/500.
            mouse['callback'](editor.cv2.EVENT_LBUTTONDOWN,500,225,0,None)
            mouse['callback'](editor.cv2.EVENT_MOUSEMOVE,500,240,editor.cv2.EVENT_FLAG_LBUTTON,None)
            mouse['callback'](editor.cv2.EVENT_LBUTTONUP,500,240,0,None)
            return ord('s')
        if n==3:
            click(500,135);click(500,315)  # Full-image y=180/420.
            return ord('s')
        if n==4: return ord('p')
        return ord('q')
    with ExitStack() as stack:
        for name,value in (('arguments',lambda:args),('X5Yolo26',lambda *a:detector),
                           ('load_labels',lambda _:[]),('LatestFrameCamera',lambda *a,**k:camera),
                           ('resolve_camera_device',lambda _: 'unused')):
            stack.enter_context(patch.object(editor,name,value))
        stack.enter_context(patch.object(editor.signal,'signal'))
        for name in ('namedWindow','resizeWindow','destroyAllWindows'):
            stack.enter_context(patch.object(editor.cv2,name))
        stack.enter_context(patch.object(editor.cv2,'setMouseCallback',lambda _,cb:mouse.update(callback=cb)))
        stack.enter_context(patch.object(editor.cv2,'imshow',lambda _,image:frames.append(image.shape)))
        stack.enter_context(patch.object(editor.cv2,'waitKey',key))
        stack.enter_context(patch.object(editor.cv2,'getWindowProperty',return_value=1))
        assert editor.main()==0
    assert GripBands.load(args.grip_bands)==GripBands((320,500),(180,420))
    assert roi_path.read_bytes()==original_roi
    assert frames and camera.stop.call_count==1


def test_blue_audit_precedes_depth_with_target_still_in_upper_band():
    from run_competition_rescue import audit_from_cargo
    m,view,status=capture_fixture()
    green=view.cargo[0]
    blue=replace(green,track_id=8,class_name='danger_cyan')
    for items,sides in [((blue,),{8:'left'}),((green,blue),{7:'right',8:'left'}),
                        ((green,blue),{7:'left',8:'left'})]:
        audit=audit_from_cargo(items,side_by_track=sides)
        example=replace(view,capture_audit=audit)
        out=m._audit_output(example,status,1.)
        assert out.command.opcode==CMD_CARGO_AUDIT
        assert out.command.audit.total_count==len(items) and out.command.audit.danger_present
        if len(items)==1:
            assert out.reason=='capture_single_blue_approach'
            accepted=replace(status,context_matches=True,action_status=1,accepted_command=CMD_CARGO_AUDIT)
            next_view=replace(example,frame_sequence=example.frame_sequence+1,observed_monotonic_s=1.1)
            m._audit_output(next_view,accepted,1.1)
            assert m.audit_wire_mode_started_s==1.1
            m._audit_output(next_view,accepted,1.2)
            assert m.audit_wire_mode_started_s==1.1  # An old image cannot renew the handoff.



def test_orange88_b2_requests_remeasure_then_finishes_near():
    from rescue_vision.localizer import ApproachRangeCalibration
    m,v,s=capture_fixture('injured_orange')
    m.first_common_delivered=True
    m.settings=replace(m.settings,top_edge_range_reobserve_enabled=True)
    m.state=CompetitionState.APPROACH
    status=replace(s,mode=52,flags=0,camera_pitch_cdeg=10500,
                   context_matches=True,accepted_command=CMD_APPROACH_TARGET,action_status=1)
    calibration=ApproachRangeCalibration.load(ROOT.parent/'vision/config/approach_range.json')
    def snap(n,box):
        item=replace(v.cargo[0],bbox=box)
        with patch.object(runner,'tracked_cargo',return_value=(item,)):
            return runner.make_vision_snapshot([],[],SimpleNamespace(calibrated=False),None,status,m,n,
                observed_s=n*.1,range_calibration=calibration)
    box=(645,177,91,67)
    for n in (1,2):
        out=m._range_approach_output(snap(n,box),status,n*.1)
    assert (out.command.arg_a,out.command.arg_b,out.command.aux)==(500,10500,1)
    done=replace(status,action_status=3)
    assert m._range_approach_output(snap(3,box),done,.3).command.opcode==CMD_HOLD
    near=m._range_approach_output(snap(4,(600,430,80,90)),done,.4)
    assert (near.command.arg_a,near.command.arg_b,near.command.aux)==(260,10500,0)


def test_cluster_advance_can_finish_in_separation_ready(tmp_path):
    mission, view, status = capture_fixture(mode=38)
    planner = runner.CompetitionPlanner(mission, tmp_path/'pose', tmp_path/'stm',
        tmp_path/'command', tmp_path/'diag', runner.JsonlLog(tmp_path/'events'), 50)
    pose = PoseSnapshot(valid=True)
    out = planner._output_for_cycle(view, pose, status, False, 1.)
    planner._publish(out)
    assert (out.command.opcode, out.command.aux, out.command.arg_a) == (9, 2, 50)
    done = replace(status, mode=37, task_id=planner.protocol_task,
        action_id=planner.protocol_action, accepted_command=9, action_status=0x83)
    view = replace(view, pre_grab_regions=((7, 'middle'),), frame_sequence=11)
    for other in (replace(done, action_id=done.action_id+1), replace(done, task_id=done.task_id+1),
                  replace(done, action_status=0x81), replace(done, accepted_command=CMD_CARGO_AUDIT)):
        planner._output_for_cycle(view, pose, other, False, 1.1)
        assert mission.pre_grab_completed_frame is None
    assert planner._output_for_cycle(view, pose, done, False, 1.1).reason == 'pre_grab_advance_reobserve'
    reviewed = planner._output_for_cycle(replace(view, frame_sequence=12), pose, done, False, 1.2)
    assert reviewed.state == CompetitionState.CAPTURE_AUDIT and reviewed.command.opcode == CMD_CARGO_AUDIT
    normal, view, status = capture_fixture(mode=21)
    normal.step(view, pose, status, 1.)
    normal.step(view, pose, replace(done, context_matches=True), 1.1)
    assert normal.pre_grab_completed_frame is None
