"""Shared test/competition wheel and primary-yaw forwarding; no UART owner.

The owner supplies clock_map, radar, source counters and devent. This is the
original dynamic-map conversion, kept once so production cannot diverge.
"""
import struct
import threading
from types import SimpleNamespace

import f405_timed_protocol as wire
from f405_timed_measurements import TimedMeasurements


def forward_motion_event(self, event):
    s=event['sample'];kind=event['kind']
    if kind=='timed_identity':self.clock_map.set_identity(s)
    elif kind=='timed_clock_sync':self.clock_map.sync(s)
    elif kind=='device_identity_changed':
        self.radar.send('invalidate',dict(reason='device_identity_changed'));self.timed_yaw_seen=False;self.last_yaw_id=None
        self.devent('reference_invalidated',reason='device_identity_changed')
    elif kind=='timed_wheel_diagnostic':
        measured=self.clock_map.wheel(s,s['receive_monotonic_ns'])
        if measured is not None:self.radar.send('wheel',measured)
    elif kind=='timed_gyro_diagnostic':
        if not self.clock_map.identity or s['boot_epoch']!=self.clock_map.identity['boot_epoch']:return
        identity=(s['boot_epoch'],s['yaw_source_id'])
        if identity==self.last_yaw_id:return
        if not self.clock_map.gyro_source_valid(s,yaw=True):return
        estimate,uncertainty,reasons=self.clock_map.map_time(s['yaw_receive_us'],s['receive_monotonic_ns'])
        t=estimate if estimate is not None and estimate>=0 else s['receive_monotonic_ns']
        self.radar.send('yaw',dict(time_ns=t,yaw_deg=s['yaw_mdeg']/1000.,source=dict(source='versioned_primary_yaw',raw=s,
            time_qualified=False,time_uncertainty_ns=uncertainty,time_diagnostics=reasons)))
        self.last_yaw_id=identity;self.timed_yaw_seen=True;self.imu_packets+=1
    elif kind=='imu' and not self.timed_yaw_seen and s.get('wire_valid'):
        self.radar.send('yaw',dict(time_ns=s['timestamp_monotonic_ns'],yaw_deg=s['yaw_deg'],
            source=dict(source='legacy_primary_yaw_receive',raw=s,time_qualified=False)))
        self.imu_packets+=1


class NativeFeedbackAdapter:
    """The original conversion, next to motion propagation instead of the GUI.

    Only original 07/08 frames take this path. Identity and synchronization
    retain the original client's matched request context, never raw-reply guesses.
    No UART, SDK, age policy or alternate motion formula lives here.
    """
    def __init__(self, send, event, log):
        self.radar=SimpleNamespace(send=send)
        self.clock_map=TimedMeasurements()
        self.timed_yaw_seen=False;self.last_yaw_id=None;self.imu_packets=0
        self.devent=event;self.log=log;self.lock=threading.Lock()
        self.frames=0;self.wheels=0;self.gyros=0

    def context(self, event):
        with self.lock:
            if event['kind']=='clock_context':
                context=event['sample']
                if context.get('identity'):
                    self.clock_map.set_identity(context['identity'])
                    self.clock_map.syncs.clear()
                    self.clock_map.syncs.extend(context.get('syncs',()))
            else:
                forward_motion_event(self,event)

    def accept(self, data):
        if len(data)<8:raise ValueError('short native motion feedback')
        received_ns,=struct.unpack('>Q',data[:8])
        sample=wire.decode(data[8:])
        self.frames+=1
        kind=sample['type']
        if kind not in (wire.WHEEL,wire.GYRO):
            return  # Raw ID/SYNC cannot replace the client's matched context.
        sample.update(receive_monotonic_ns=received_ns,
                      capture_time_qualified=False,sdk_time_qualified=False)
        with self.lock:
            if kind==wire.WHEEL:self.wheels+=1
            else:self.gyros+=1
            self.log.submit('motion_feedback.jsonl',sample)
            forward_motion_event(self,dict(kind='timed_wheel_diagnostic' if kind==wire.WHEEL else
                'timed_gyro_diagnostic',sample=sample))

    def snapshot(self):
        return dict(frames=self.frames,wheel_frames=self.wheels,gyro_frames=self.gyros,
            timed_yaw_seen=self.timed_yaw_seen,conversion='original_forward_motion_event',
            identity_present=self.clock_map.identity is not None)
