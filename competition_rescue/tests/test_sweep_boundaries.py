from pathlib import Path
import sys
import json
from types import SimpleNamespace
from unittest.mock import patch
import pytest
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2]/'vision')]
from sweep_boundaries import validated_pair, validated_region, corridor_polygon, save_boundaries, load_boundaries
import run_competition_rescue as runner
from state_machine import TrackedCargo


def test_recorded_blue_4872_is_blocked_by_center_with_original_polygon():
    from dataclasses import replace
    from state_machine import CompetitionMission, CompetitionSettings, SweepPickupContext
    from sweep_boundaries import obstacle_reference_point
    evidence = json.loads((Path(__file__).resolve().parents[2] /
        'docs/reports/competition_20261118_0048/evidence.json').read_text())['blue_geometry']
    blue = TrackedCargo(234, 'danger_cyan', .9455, tuple(evidence['blue_bbox']), hits=3)
    polygon = tuple(map(tuple, evidence['saved_sweep_polygon']))
    safe = tuple(evidence['safe_bbox'])
    assert not evidence['in_region']  # Original bottom-midpoint result, before this fix.
    assert obstacle_reference_point(blue.class_name, blue.bbox) == (646., 453.5)
    with patch.object(runner, 'safe_zone_corridor_polygon', return_value=polygon):
        for pitch in (11500,12000):
            for first_done in (False,True):
                result = runner.safe_zone_push_corridor((blue,), safe, 'material', first_done,
                    carried_manifest=('green_supply',), carried_total_count=1,
                    claw_polygon=polygon, camera_pitch_cdeg=pitch)
                assert result[1] is blue and result[3] == 'BLOCKED'
            for category in ('green_supply', 'core_black', 'injured_orange'):
                other = replace(blue, class_name=category)
                assert obstacle_reference_point(category, other.bbox) == (646.,568.)
                assert runner.safe_zone_push_corridor((other,), safe, 'injury', True,
                    claw_polygon=polygon, camera_pitch_cdeg=pitch)[1] is None
        assert runner.safe_zone_push_corridor((replace(blue,inside_safe_zone=True),), safe,
            'material', True, claw_polygon=polygon)[1] is None
    m = CompetitionMission(CompetitionSettings(side='blue',initial_stash_enabled=False))
    m.safe_sweep_pickup = SweepPickupContext(target_id=blue.track_id,
        target_class=blue.class_name,target_bbox=blue.bbox,excluded_ids=frozenset())
    assert m.safe_sweep_target((blue,),polygon,safe) is blue
    outside = replace(blue, bbox=(0,339,100,100))
    assert m.safe_sweep_target((outside,),polygon,safe) is None


def test_curves_distinguish_angles_and_clip_safe_edge(tmp_path):
    pair=validated_pair({'left':[[200,0],[300,500],[200,900]],'right':[[800,0],[750,500],[1100,900]]})
    other=validated_pair({'left':[[100,0],[200,900]],'right':[[1100,0],[1200,900]]})
    path=tmp_path/'sweep_boundaries.json'
    save_boundaries(path,{'12000':pair,'11500':other})
    data=load_boundaries(path)
    assert data['12000']!=data['11500']
    poly=corridor_polygon(pair,100)
    assert min(y for x,y in poly)==100 and max(y for x,y in poly)==900
    block=TrackedCargo(162,'danger_cyan',.96,(819,285,247,229),hits=27)
    roi=((184,73),(1107,73),(1107,709),(184,709))
    with patch.object(runner,'runtime_boundaries',return_value=data):
        # Original rectangle admits the incident blue block; the human-defined
        # narrower curve below is a synthetic example, never a saved real ROI.
        assert runner.safe_zone_push_corridor((block,),(12,0,879,73),'injury',True,claw_polygon=roi,camera_pitch_cdeg=0)[1] is block
        assert runner.safe_zone_push_corridor((block,),(12,0,879,73),'injury',True,claw_polygon=roi,camera_pitch_cdeg=12000)[1] is None
        assert runner.safe_zone_push_corridor((block,),(12,0,879,73),'injury',True,claw_polygon=roi,camera_pitch_cdeg=11500)[1] is block
    with pytest.raises(ValueError):
        validated_pair({'left':[[900,0],[900,900]],'right':[[800,0],[1000,900]]})


def test_editor_saves_two_sweep_angles_without_changing_claw_roi(tmp_path):
    import numpy as np
    from contextlib import ExitStack
    from unittest.mock import Mock
    import edit_capture_roi as editor
    from capture_roi import CaptureRois, save_capture_rois
    roi=tmp_path/'capture_roi.json'
    save_capture_rois(roi,CaptureRois(((200,200),(1000,200),(1000,800),(200,800)),(),()))
    original=roi.read_bytes()
    args=SimpleNamespace(vision_fps=50.,model=None,labels=None,score_thres=.1,nms_thres=.45,priority=0,bpu_cores=[0],preprocess='cpu',decoder='software',device='unused',camera_fps=30,decode_fps=30.,output=roi,grip_bands=tmp_path/'bands.json')
    camera=Mock();camera.check_error.return_value=None
    camera.latest.return_value=SimpleNamespace(frame_id=1,pixel_format='bgr',image=np.zeros((1024,1280,3),np.uint8))
    detector=Mock();detector.infer.return_value=([],None)
    mouse={};step=[0]
    def click(x,y):mouse['cb'](editor.cv2.EVENT_LBUTTONDOWN,x,y,0,None)
    def key(_):
        n=step[0];step[0]+=1
        if n in (0,2):
            click(50 if n==0 else 250,editor.DISPLAY_HEIGHT+160)
            click(450,editor.DISPLAY_HEIGHT+160)  # Clear the current region only.
            click(150,80)
            mouse['cb'](editor.cv2.EVENT_MOUSEMOVE,200,600,editor.cv2.EVENT_FLAG_LBUTTON,None)
            mouse['cb'](editor.cv2.EVENT_LBUTTONUP,200,600,0,None)
            click(800,600);click(750 if n==0 else 700,80)
            return ord('s')
        if n==4:return ord('q')
        return 255
    with ExitStack() as stack:
        for name,value in [('arguments',lambda:args),('X5Yolo26',lambda *a:detector),('load_labels',lambda _:[]),('LatestFrameCamera',lambda *a,**k:camera),('resolve_camera_device',lambda _:'unused')]:stack.enter_context(patch.object(editor,name,value))
        stack.enter_context(patch.object(editor.signal,'signal'))
        for name in ['namedWindow','resizeWindow','destroyAllWindows','imshow']:stack.enter_context(patch.object(editor.cv2,name))
        stack.enter_context(patch.object(editor.cv2,'setMouseCallback',lambda _,cb:mouse.update(cb=cb)))
        stack.enter_context(patch.object(editor.cv2,'waitKey',key))
        stack.enter_context(patch.object(editor.cv2,'getWindowProperty',return_value=1))
        assert editor.main()==0
    loaded=load_boundaries(tmp_path/'sweep_boundaries.json')
    assert set(loaded)=={'11500','12000'} and loaded['11500']!=loaded['12000']
    assert roi.read_bytes()==original and not args.grip_bands.exists()



def test_closed_region_keeps_concavity_and_class_reference(tmp_path):
    # One loop, including a notch: bounding rectangle must not admit the notch.
    region=validated_region({'polygon':[[100,100],[900,100],[900,900],
        [600,900],[600,500],[400,500],[400,900],[100,900]]})
    path=tmp_path/'regions.json'
    old=validated_pair({'left':[[50,0],[100,900]],'right':[[1000,0],[1100,900]]})
    save_boundaries(path,{'11500':old})
    original=path.read_bytes()
    save_boundaries(path,{'11500':old,'12000':region})
    assert json.loads(path.read_text())['object_reference_overrides'] == {'danger_cyan':'bbox_center'}
    assert list(tmp_path.glob('regions.json.*.bak'))[0].read_bytes()==original
    data=load_boundaries(path)
    assert data['11500']==old and data['12000']==region
    assert corridor_polygon(region,350)==tuple(tuple(p) for p in region['polygon'])
    in_notch=TrackedCargo(1,'danger_cyan',.9,(450,600,100,100),hits=3)
    in_region=TrackedCargo(2,'danger_cyan',.9,(200,600,100,100),hits=3)
    # Its top lies outside the region, but the blue centre is on the boundary.
    tall=TrackedCargo(3,'danger_cyan',.9,(200,0,100,200),hits=3)
    roi=((100,100),(900,100),(900,900),(100,900))
    with patch.object(runner,'runtime_boundaries',return_value=data):
        for item,expected in [(in_notch,None),(in_region,in_region),(tall,tall)]:
            result=runner.safe_zone_push_corridor((item,),(950,0,200,50),'injury',True,
                claw_polygon=roi,camera_pitch_cdeg=12000)
            assert result[1] is expected
    for points in [[],[[1,1],[2,2],[3,3]],[[100,100],[900,800],[100,900],[900,100]]]:
        with pytest.raises(ValueError):validated_region({'polygon':points})
    previous=path.read_bytes()
    with pytest.raises(ValueError):save_boundaries(path,{'12000':{'polygon':[[0,0],[1,1]]}})
    assert path.read_bytes()==previous
