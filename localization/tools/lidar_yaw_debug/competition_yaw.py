"""One wall heading correction per formal start, in field CCW degrees."""
from collections import deque
import math
from statistics import median
import socket
import struct

from f407_motion_protocol import crc16_modbus
import numpy as np


def wall_axis_angles(lines, center, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array(((c,-s),(s,c)))
    evidence = [[], []]
    for line in lines:
        normal = rot @ line['normal']
        axis = int(abs(normal[1]) > abs(normal[0]))
        err = (90*axis-math.degrees(math.atan2(normal[1],normal[0]))+90)%180-90
        xy = line['points'] @ rot.T + center
        coord = xy[:,axis]
        wall = math.copysign(1.5, float(np.median(coord)))
        if (abs(err)<=12 and np.median(abs(coord-wall))<.20 and
                np.mean(abs(xy[:,1-axis])<=1.55)>.85):
            evidence[axis].append(err)
    if not all(evidence) or np.ptp(evidence[0]+evidence[1])>3.:
        return None
    return median(evidence[0]), median(evidence[1])


class WallYawVote:
    def __init__(self):
        self.samples = deque(maxlen=3)
        self.last = None

    def observe(self, epoch, index, delta):
        if self.last != (epoch,index-1) or delta is None:
            self.samples.clear()
        self.last = epoch,index
        if delta is not None:
            self.samples.append(delta)
        if len(self.samples)==3 and max(self.samples)-min(self.samples)<=100:
            return median(self.samples)
        return None


def start_word(delta, tag):
    return 0x8000 | tag if delta is None else 0x4000 | (1200+delta)


def reliable_delta(lines, center, yaw):
    from .diagnostic import fixed_yaw_position
    angles = wall_axis_angles(lines, center, yaw)
    if angles is None:
        return None
    delta = round(100 * sum(angles) / 2)
    # After rotating the geometry both directions must still fit real enclosure walls.
    fit = fixed_yaw_position([], center, yaw + math.radians(delta/100), [0,0,0],
                             _lines=lines, independent_axes=True, max_correction_m=float('inf'))
    return delta if fit['rank_xy']==2 else None


def sync_applied_yaw(engine, round_token, delta):
    """Finish the upper half of an acknowledged correction, including after a lost reply."""
    with engine.lock:
        reference = engine.refresh_native().get('reference') or {}
        if not round_token or reference.get('round_token') != round_token:
            raise ValueError('wall yaw belongs to a different round')
        if 'wall_yaw_applied_cdeg' not in reference:
            engine.runtime.command('wall_yaw_commit', dict(round_token=round_token, delta_cdeg=delta))
            reference = engine.refresh_native().get('reference') or {}
        if reference.get('round_token') != round_token or reference.get('wall_yaw_applied_cdeg') != delta:
            raise ValueError('native wall yaw does not match the acknowledged correction')
        engine.pending_reacquisition = None
        engine.latest_points = []
        engine.latest_matched_points = []


def propose(socket_name, tag, delta):
    body=struct.pack('>BBHhI',0x21,0,tag,delta,0)
    crc=crc16_modbus(body)
    wire=b'\xa3\xb3'+body+struct.pack('<H',crc)+b'\xc3'
    try:
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sock:
            sock.setblocking(False)
            return sock.sendto(wire,'\0'+socket_name[1:])==len(wire)
    except OSError:
        return False
