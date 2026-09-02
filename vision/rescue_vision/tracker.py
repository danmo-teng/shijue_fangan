from __future__ import annotations

import math
from typing import Any

from .models import Detection, Track, TrackState


class MultiFrameTracker:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.tracks: list[Track] = []
        self.next_id = 1
        reference_width, reference_height = config.get("threshold_reference_resolution", [640, 480])
        camera = config.get("camera", {})
        width = int(camera.get("width", reference_width))
        height = int(camera.get("height", reference_height))
        self.pixel_scale = math.sqrt((width * height) / max(reference_width * reference_height, 1))

    @staticmethod
    def _position(detection: Detection) -> tuple[float, float]:
        return detection.ground_xy_mm or detection.bottom_point

    def _settings(self, class_name: str) -> dict[str, Any]:
        return self.config["classes"][class_name].get("confirmation", {})

    def update(self, detections: list[Detection]) -> list[Track]:
        unmatched = set(range(len(detections)))
        used_tracks: set[int] = set()
        pairs: list[tuple[float, int, int]] = []
        for track_index, track in enumerate(self.tracks):
            settings = self._settings(track.class_name)
            gate = float(settings.get("match_distance", 100.0))
            if track.last_detection.ground_xy_mm is None:
                gate *= self.pixel_scale
            for detection_index, detection in enumerate(detections):
                if detection.class_name != track.class_name:
                    continue
                position = self._position(detection)
                distance = math.hypot(position[0] - track.position[0], position[1] - track.position[1])
                if distance <= gate:
                    pairs.append((distance, track_index, detection_index))

        for _, track_index, detection_index in sorted(pairs):
            if track_index in used_tracks or detection_index not in unmatched:
                continue
            track = self.tracks[track_index]
            detection = detections[detection_index]
            position = self._position(detection)
            alpha = float(self._settings(track.class_name).get("position_alpha", 0.55))
            track.position = (
                alpha * position[0] + (1.0 - alpha) * track.position[0],
                alpha * position[1] + (1.0 - alpha) * track.position[1],
            )
            track.confidence = 0.35 * detection.confidence + 0.65 * track.confidence
            track.hits += 1
            track.misses = 0
            track.age += 1
            track.last_detection = detection
            required_hits = int(self._settings(track.class_name).get("min_hits", 3))
            if track.hits >= required_hits:
                track.state = TrackState.CONFIRMED
            used_tracks.add(track_index)
            unmatched.remove(detection_index)

        survivors: list[Track] = []
        for index, track in enumerate(self.tracks):
            if index not in used_tracks:
                track.misses += 1
                track.age += 1
                max_misses = int(self._settings(track.class_name).get("max_misses", 5))
                if track.misses > 0 and track.state == TrackState.CONFIRMED:
                    track.state = TrackState.LOST
                if track.misses > max_misses:
                    continue
            survivors.append(track)
        self.tracks = survivors

        for detection_index in sorted(unmatched):
            detection = detections[detection_index]
            min_hits = int(self._settings(detection.class_name).get("min_hits", 3))
            state = TrackState.CONFIRMED if min_hits <= 1 else TrackState.TENTATIVE
            self.tracks.append(
                Track(
                    track_id=self.next_id,
                    class_name=detection.class_name,
                    position=self._position(detection),
                    confidence=detection.confidence,
                    state=state,
                    hits=1,
                    misses=0,
                    age=1,
                    last_detection=detection,
                )
            )
            self.next_id += 1

        return list(self.tracks)

    def confirmed(self) -> list[Track]:
        return [track for track in self.tracks if track.state == TrackState.CONFIRMED]
