"""Small match checkpoint and explicit empty-load restart configuration."""
import json
from pathlib import Path
from protocol import FRAME_HEAD, FRAME_TAIL, crc16_modbus


def match_progress(mission):
    return dict(first_green=bool(mission.first_common_delivered),
                attack_done=bool(mission.initial_stash_done),
                search_point=int(mission.search_point_number),
                delivery_count=int(mission.delivery_count),
                search_direction=int(mission.search_scan_direction),
                orange_search_round=int(mission.orange_search_round),
                orange_search_turn_deg=float(mission.orange_search_turn_deg),
                stash_has_cargo=bool(mission.stash_has_cargo))


def restore_progress(mission, progress):
    mission.first_common_delivered = bool(progress['first_green'])
    mission.initial_stash_done = bool(progress['attack_done'])
    mission.search_point_number = mission.delivery_search_point_number = int(progress['search_point'])
    mission.delivery_count = int(progress['delivery_count'])
    mission.search_scan_direction = int(progress.get('search_direction', 1))
    mission.orange_search_round = int(progress.get('orange_search_round',
        min(4, mission.delivery_count) if mission.first_common_delivered else 0))
    mission.orange_search_turn_deg = float(progress.get('orange_search_turn_deg', 0.0))
    mission.stash_has_cargo = bool(progress['stash_has_cargo'])
    mission.resume_search_after_start = mission.initial_stash_done


class ProgressStore:
    def __init__(self, path, session):
        self.path, self.session, self.last = Path(path), session, None

    def save(self, mission):
        progress = match_progress(mission)
        # Retain the exact live angle for hot resume, but write only milestones
        # (round changes/full circle), never one disk update per yaw sample.
        milestone = dict(progress, orange_search_turn_deg=(
            360.0 if progress['orange_search_turn_deg'] >= 360.0 else 0.0))
        if milestone == self.last:
            return False
        # Only milestone changes write; not one disk operation per camera frame.
        value = dict(version=1, session_token=self.session, side=mission.settings.side,
                     start_zone=mission.settings.start_zone, progress=progress)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(value, ensure_ascii=False)+'\n')
        temporary.replace(self.path)
        self.last = milestone
        return True


def resume_config_frame(sequence, side, zone, progress):
    """TYPE11: old colour/zone + marker R + first-green/attack bits + count.

    Requires paired F405 support. Legacy zero-padding-only firmware rejects it.
    """
    flags = int(progress['first_green']) | (int(progress['attack_done']) << 1)
    payload = bytes((0x11 if side == 'red' else 0x12, zone, 0x52, flags,
                     int(progress['delivery_count']) & 255,
                     int(progress.get('stop_serial', 0)) & 255, 0, 0))
    body = bytes((0x11, sequence)) + payload
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, 'little') + bytes((FRAME_TAIL,))
