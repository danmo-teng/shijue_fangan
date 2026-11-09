"""Unchanged formal control-packet formatting, without transport or estimation.

The caller owns the packet id and last ready packet. Only a successful send
advances its id; a missing reference copies the last ready pose with LOST.
"""


def format_control_packet(state,raw_pose,session_id,packet_id,last_control=None,*,lidar_state=None):
    if not state.get('ready'):
        if last_control is None:
            return None
        return dict(last_control,packet_id=packet_id,quality='LOST',
                    reference_invalid_reason=state.get('reason','reference unavailable'))
    x,y=state['center_xy_m']
    mode=state['motion_selection']['mode']
    raw=raw_pose
    lidar=lidar_state or {}
    return dict(schema_version=1,session_id=session_id,packet_id=packet_id,
        frame='field',timestamp_monotonic_ns=state['sample_monotonic_ns'],
        quality='DEGRADED',  # usable estimator output, not an accuracy certificate
        pose=dict(x_m=x,y_m=y,yaw_deg=state['yaw_deg']),
        pose_source=state['source'],motion_source=mode,
        t265=dict(tracker_confidence=raw.get('tracker_confidence',0),
            mapper_confidence=raw.get('mapper_confidence',0),
            travel_from_start_m=raw.get('travel_from_start_m',0.),
            translation_scale=1.,translation_scale_enabled=False,
            raw_center_xy_m=state['t265_center_xy_m'],
            sample_monotonic_ns=state['raw_t265_sample_ns']),
        wheel=dict(fusion_weight=0.,official_sdk_input=True,independent_reference=False),
        navigation=dict(distance_compensation_enabled=False,wheel_progress_m=None),
        qualified_pose=False,accuracy_verified=False,
        current_wall_axes=state['current_wall_axes'],
        lidar_corrections_enabled=state['lidar_corrections_enabled'],
        lidar_connected=lidar.get('lidar_connected',False),lidar_error=lidar.get('lidar_error'),
        position_reacquisitions=state['position_reacquisitions'],
        reference=state['reference'],motion_selection=state['motion_selection'])
