"""Shared test/competition wheel and primary-yaw forwarding; no UART owner.

The owner supplies clock_map, radar, source counters and devent. This is the
original dynamic-map conversion, kept once so production cannot diverge.
"""

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
