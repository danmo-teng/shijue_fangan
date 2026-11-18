from pathlib import Path
import sys, json, socket, math
import numpy as np

root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'localization/tools'),str(root/'competition_rescue'),str(root/'vision')]
from lidar_yaw_debug.competition_yaw import reliable_delta, WallYawVote, start_word, propose
from lidar_yaw_debug.dynamic import rotation
from protocol import crc16_modbus
from run_competition_rescue import publish_start_config


def test_wall_vote_and_wire(tmp_path):
    # Synthetic orthogonal field walls seen with true +4 degrees, predicted 0.
    rot=rotation(math.radians(4))
    def line(points,normal):
        return dict(points=np.array(points)@rot,normal=np.array(normal)@rot)
    t=np.linspace(-1.,1.,24)
    walls=[line(np.c_[t*0+1.5,t],[1.,0.]),line(np.c_[t,t*0+1.5],[0.,1.])]
    assert reliable_delta(walls,(0.,0.),0.)==400
    assert reliable_delta(walls[:1],(0.,0.),0.) is None
    vote=WallYawVote()
    for idx,d in [(1,300),(2,350),(4,320),(5,None),(6,400),(7,505),(8,411),(9,399)]:
        assert vote.observe('e',idx,d) is None
    assert vote.observe('e',10,430)==411
    assert vote.observe('new_epoch',11,400) is None
    sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    with sock:
        sock.bind('\0yaw-test-'+str(id(sock)))
        assert propose('@yaw-test-'+str(id(sock)),123,-175)
        frame=sock.recv(16)
        assert frame[2:4]==bytes([0x21,0])
        assert frame[4:8]==bytes.fromhex('007bff51')
        assert crc16_modbus(frame[2:12])==int.from_bytes(frame[12:14],'little')
    for progress in [None,dict(first_green=True,attack_done=True,delivery_count=8,stop_serial=3)]:
        for d in [None,0,-175,320]:
            word=start_word(d,123)
            path=tmp_path/'cmd'
            assert publish_start_config(path,'blue',2,lambda:True,lambda:None,progress,word)
            f=path.read_bytes()
            assert int.from_bytes(f[10:12],'big')==word
            assert int.from_bytes(f[12:14],'little')==crc16_modbus(f[2:12])
            assert f[6]==(0x52 if progress else 0)


def test_native_yaw_pivot_and_round_ownership():
    from lidar_yaw_debug.native_motion import NativeMotionRuntime,NativeDynamicFence
    rt=NativeMotionRuntime(dict(feedback_fd=-1,pose_fd=-1))
    try:
        e=NativeDynamicFence({},rt)
        def sample(i,x,y):
            t=1000000000+i*5000000
            e.ingest_yaw(t,40.,dict(raw={}))
            e.ingest_pose(dict(session_id='fixture',frame_number=i,timestamp_monotonic_ns=t,
                receive_monotonic_ns=t,x_m=x,y_m=y,yaw_rad=math.radians(45),tracker_confidence=3))
        sample(1,1.,1.)
        e.align('start',center_xy=(1.,1.),heading_deg=45,round_token='r',wall_yaw_word=0x807b)
        sample(2,1.3,.9)
        before=rt.snapshot()['last']
        import pytest
        with pytest.raises(ValueError):rt.command('wall_yaw_commit',dict(round_token='old',delta_cdeg=400))
        rt.command('wall_yaw_commit',dict(round_token='r',delta_cdeg=400))
        after=rt.snapshot()['last']
        assert np.allclose(after['center_xy_m'],before['center_xy_m'])
        assert abs(after['yaw_deg']-before['yaw_deg']-4)<1e-7
        with pytest.raises(ValueError):rt.command('wall_yaw_commit',dict(round_token='r',delta_cdeg=400))
        sample(3,1.4,.9)
        assert np.allclose(np.subtract(rt.snapshot()['last']['center_xy_m'],after['center_xy_m']),
                           rotation(math.radians(4))@[.1,0])
    finally:rt.close()


def test_start_uses_measured_still_interval():
    from lidar_yaw_debug.native_motion import NativeMotionRuntime,NativeDynamicFence
    rt=NativeMotionRuntime(dict(feedback_fd=-1,pose_fd=-1))
    try:
        e=NativeDynamicFence({},rt)
        rt.command('lidar_state',dict(connected=True,enabled=True))
        for i in range(14):
            t=1_000_000_000+i*10_000_000
            e.ingest_yaw(t,20.+(1. if i==13 else 0.),dict(raw={}))
            e.ingest_pose(dict(session_id='stationary',frame_number=i,timestamp_monotonic_ns=t,
                receive_monotonic_ns=t,x_m=1.,y_m=1.,yaw_rad=.2,tracker_confidence=3))
            e.ingest_wheel(dict(source='measured_wheel_feedback',measurement_valid=True,mcu_epoch='boot',sample_id=i,
                raw_mcu_sample=dict(t0_us=(t-10000000)//1000,t1_us=t//1000),v_body_mps=[0.,0.,0.],
                interval_start_ns=t-10000000,interval_end_ns=t,receive_monotonic_ns=t))
            if i in (12,13):
                result=rt.command('wall_yaw_still',dict(first_ns=1020000000,reference_ns=1100000000))
                assert result['quiet']==(i==12)
    finally:rt.close()


def test_retry_wall_yaw_before_write_and_after_write_without_double_rotation():
    import pytest
    from unittest.mock import patch
    from lidar_yaw_debug.native_motion import NativeMotionRuntime, NativeDynamicFence
    from lidar_yaw_debug.competition_yaw import sync_applied_yaw
    rt = NativeMotionRuntime(dict(feedback_fd=-1, pose_fd=-1))
    try:
        engine = NativeDynamicFence({}, rt)
        t = 1_000_000_000
        engine.ingest_yaw(t, 40., dict(raw={}))
        engine.ingest_pose(dict(session_id='retry', frame_number=1, timestamp_monotonic_ns=t,
            receive_monotonic_ns=t, x_m=1., y_m=1., yaw_rad=.5, tracker_confidence=3))
        engine.align('start', center_xy=(1., 1.), heading_deg=45, round_token='round', wall_yaw_word=0x807b)
        initial = rt.snapshot()['last']['yaw_deg']
        with patch.object(rt, 'command', side_effect=RuntimeError('write failed')), pytest.raises(RuntimeError):
            sync_applied_yaw(engine, 'round', 350)
        assert 'wall_yaw_applied_cdeg' not in rt.snapshot()['reference']
        with patch.object(engine, 'refresh_native', side_effect=[engine.refresh_native(), RuntimeError('refresh failed')]), pytest.raises(RuntimeError):
            sync_applied_yaw(engine, 'round', 350)
        assert rt.snapshot()['reference']['wall_yaw_applied_cdeg'] == 350
        with patch.object(rt, 'command', side_effect=AssertionError('second write')):
            sync_applied_yaw(engine, 'round', 350)
            for token, delta in [('old',350), ('round',351)]:
                with pytest.raises(ValueError):
                    sync_applied_yaw(engine, token, delta)
        assert abs(rt.snapshot()['last']['yaw_deg'] - initial - 3.5) < 1e-7
    finally:
        rt.close()
