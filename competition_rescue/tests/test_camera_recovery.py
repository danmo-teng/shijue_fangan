from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_recovery import CameraRecovery


def trace():
    clock = SimpleNamespace(ns=0)
    records = []
    events = SimpleNamespace(write=lambda event, data: records.append((event, data)))
    recovery = CameraRecovery(events, lambda: clock.ns)
    assert not recovery.active and records == []
    recovery.begin('camera-index0', 'fixture disconnect')
    return recovery, clock, records


def test_node_reappearance_is_bounded_and_missing_polls_do_not_spam(tmp_path):
    r,c,events = trace()
    node=tmp_path/'video7'
    link=tmp_path/'camera-index0';link.symlink_to(node)
    for ns in range(0,210_000_000,10_000_000):
        c.ns=ns
        assert not r.ready(str(link),ns)
    assert [e for e,_ in events].count('camera_node_missing')==1
    assert r.last_missing_ns==200_000_000
    c.ns=210_000_000;node.touch()
    assert not r.ready(str(link),c.ns)
    c.ns=400_000_000
    assert r.ready(str(link),c.ns)
    present=events[-1][1]
    assert present['last_missing_monotonic_ns']==200_000_000
    assert present['node_observed_monotonic_ns']==400_000_000
    assert present['observation_gap_ms']==200
    assert r.open_attempts==0  # No constructor/rebuild on absent-node probes.


def test_real_open_failure_keeps_backoff_but_still_observes_node_flaps(tmp_path):
    r,c,events=trace();node=tmp_path/'video';node.touch()
    assert r.ready(str(node),0)
    start=r.open_started();c.ns=100_000_000
    r.open_finished(start,RuntimeError('pipeline failure'),retry_s=2)
    node.unlink();c.ns=200_000_000
    assert not r.ready(str(node),c.ns)
    node.touch();c.ns=400_000_000
    assert not r.ready(str(node),c.ns)  # Observed present, but do not hammer the pipeline.
    assert events[-1][0]=='camera_node_present'
    c.ns=2_100_000_000
    assert r.ready(str(node),c.ns)
    assert r.open_attempts==1


def test_open_frame_and_result_milestones_are_distinct_and_reuse_packet_time(tmp_path):
    r,c,events=trace();node=tmp_path/'video';node.touch()
    c.ns=200_000_000;assert r.ready(str(node),c.ns)
    start=r.open_started();c.ns=1_300_000_000;r.open_finished(start)
    packet=SimpleNamespace(frame_id=3,published_ns=1_320_000_000)
    c.ns=1_340_000_000;r.observe_frame(packet)
    r.observe_frame(SimpleNamespace(frame_id=4,published_ns=1_350_000_000))
    c.ns=1_400_000_000;details=r.finish(packet)
    assert details['node_present_observed_monotonic_ns']==200_000_000
    assert details['first_frame_id']==3  # Do not pretend runner necessarily saw frame1.
    assert details['first_frame_published_monotonic_ns']==1_320_000_000
    assert details['first_frame_observed_monotonic_ns']==1_340_000_000
    assert details['result_ready_monotonic_ns']==1_400_000_000
    assert details['total_recovery_ms']==1400
    assert [e for e,_ in events].count('camera_first_frame_observed')==1
    assert not r.active
    count=len(events);r.observe_frame(packet);assert len(events)==count


def test_unrestored_failure_keeps_incident_and_unknown_start_duration():
    r,c,events=trace();c.ns=1_000_000_000
    r.begin('camera-index0','another error before recovery')
    assert r.incident==1 and r.started_ns==0
    r.open_finished(None,RuntimeError('startup failed before trace existed'),retry_s=1.)
    assert events[-1][1]['open_duration_ms'] is None
    assert r.next_open_ns==2_000_000_000


def test_node_stat_error_is_not_fabricated_missing_evidence():
    from unittest.mock import patch
    r,c,events=trace()
    with patch.object(Path,'exists',side_effect=PermissionError('fixture')):
        assert r.ready('camera-index0',0)  # Delegate to original open error handling.
    assert r.present is None and r.last_missing_ns is None
    assert all(event!='camera_node_missing' for event,_ in events)
