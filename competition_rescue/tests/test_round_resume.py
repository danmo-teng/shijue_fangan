import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import threading
import json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from round_resume import ProgressStore, match_progress, restore_progress, resume_config_frame
from state_machine import CompetitionMission, CompetitionSettings, CompetitionState, PoseSnapshot, StmSnapshot, VisionSnapshot, CargoBatch
from protocol import crc16_modbus, CMD_BULLDOZER_STEP


def test_stop_keeps_match_until_five_seconds_including_release_poll():
    from button_input import SysfsButtonInput
    # Both a held-key poll and the release fallback use the same 5s rule.
    for hold, reset in ((2.1, False), (4.9, False), (5.1, True)):
        levels = {key: 1 for key in range(1, 5)}
        buttons = SysfsButtonInput(read_level=lambda key: levels[key])
        buttons.poll(0.)
        levels[4] = 0
        buttons.poll(1.)
        assert [e['action'] for e in buttons.poll(1.04)] == ['stop']
        levels[4] = 1
        buttons.poll(1. + hold)
        events = buttons.poll(1.04 + hold)
        assert [e['action'] for e in events] == (['exit_session'] if reset else [])


def test_match_progress_three_reachable_cases_drop_load_and_return_to_source(tmp_path):
    for green,attack in ((False,False),(False,True),(True,True)):
        old=CompetitionMission(CompetitionSettings(side='red',initial_stash_enabled=True))
        old.initial_stash_done=attack;old.first_common_delivered=green
        old.search_point_number=3;old.delivery_count=3 if green else 0
        old.search_scan_direction=-1
        old.selected_batch=CargoBatch((91,),('injured_orange',),'injury')
        old.carried_manifest=('injured_orange',);old.disperse_attempts=2
        store=ProgressStore(tmp_path/'progress.json','this-match');store.save(old)
        saved=json.loads(store.path.read_text())
        assert saved['session_token']=='this-match'
        new=CompetitionMission(old.settings);restore_progress(new,saved['progress'])
        assert new.selected_batch is None and not new.carried_manifest and new.disperse_attempts==0
        assert new.first_common_delivered==green and new.initial_stash_done==attack
        assert new.search_scan_direction==-1
        legacy=dict(saved['progress']);legacy.pop('search_direction')
        restore_progress(new,legacy)
        assert new.search_scan_direction==1
        restore_progress(new,saved['progress'])
        p=PoseSnapshot(True,0,0,90,0);st=StmSnapshot(mode=3,age_ms=0,camera_pitch_cdeg=12000)
        out=new.step(VisionSnapshot(),p,st,1.)
        if attack:
            assert out.command.opcode==CMD_BULLDOZER_STEP
            assert new.search_transfer_route.target_point==3
        else:assert new.state==CompetitionState.INITIAL_OBSERVE


def test_resume_config_is_explicit_and_keeps_original_frame_crc():
    for side in ('red','blue'):
        for green,attack in ((False,False),(False,True),(True,True)):
            p=dict(first_green=green,attack_done=attack,delivery_count=3,stop_serial=7)
            for seq in range(3):
                f=resume_config_frame(seq,side,1,p)
                assert len(f)==15 and f[2:4]==bytes((0x11,seq))
                assert f[4:9]==bytes((0x11 if side=='red' else 0x12,1,0x52,int(green)|int(attack)<<1,3))
                assert f[9:12]==bytes((7,0,0))
                assert crc16_modbus(f[2:12])==int.from_bytes(f[12:14],'little')


def test_resume_reference_is_new_and_old_pose_cannot_release_config(tmp_path):
    from run_competition_rescue import operator_start_request
    control=tmp_path/'control';pose=tmp_path/'pose'
    control.write_text(json.dumps(dict(session_token='match',action='resume',reference_token='resume1',side='red',start_zone=1,stop_serial=7)))
    pose.write_text(json.dumps(dict(reference=dict(round_token='match'))))
    assert operator_start_request(control,pose,'match') is None
    pose.write_text(json.dumps(dict(reference=dict(round_token='resume1'))))
    assert operator_start_request(control,pose,'match')['action']=='resume'
    assert operator_start_request(control,pose,'different-match') is None


def test_resume_keeps_sdk_owner_and_new_stop_wins(tmp_path):
    import competition_map_app as formal
    from operator_panel import OperatorPanel
    app=formal.CompetitionMapApp.__new__(formal.CompetitionMapApp)
    app.prewarm_enabled=True;app.session_started=True;app.zone=1;app.side='red';app.opening_strategy='attack'
    app.corner_offset_m=.253;app.reset_session_token='match'
    app.dynamic=SimpleNamespace(request_start=Mock());owner=app.dynamic
    app.trajectory=Mock();app.odometry_trajectory=Mock();app.record_reset_event=Mock()
    panel=OperatorPanel.__new__(OperatorPanel)
    panel.app=app;panel.stopped=True;panel._stop_serial=7;panel._state_lock=threading.Lock()
    panel.control_path=tmp_path/'control';panel.silence=Mock();app.operator=panel
    app.start_session()
    request=json.loads(panel.control_path.read_text())
    assert request['action']=='resume' and request['session_token']=='match'
    assert request['reference_token']!='match' and not panel.stopped
    assert app.dynamic is owner and app.dynamic.request_start.call_count==1
    app.start_session() # repeated short3 cannot launch another resume
    assert app.dynamic.request_start.call_count==1
    panel.stopped=True;panel._stop_serial=8
    before=panel.control_path.read_text()
    assert not panel.release_resume(1,'red','attack','stale',7)
    assert panel.control_path.read_text()==before and panel.stopped
