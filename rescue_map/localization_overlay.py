"""Bounded display only; consumes existing results, never estimates position."""
from collections import deque
import numpy as np
import cv2


class LocalizationOverlay:
    def __init__(self):
        self.trail = deque(maxlen=4000)
        self.reference = None
        self.points = []
        self.axes = []
        self._view = None
        self._trail_pixels = np.empty((4000,2), dtype=np.int32)
        self._pixel_count = 0
        self._pixel_last = None
        self._scan_key = None
        self._dots_key = None
        self._dot_indices = None

    def reset(self):
        self.trail.clear()
        self.reference = None
        self.points = []
        self.axes = []
        self._view = None
        self._trail_pixels = np.empty((4000,2), dtype=np.int32)
        self._pixel_count = 0
        self._pixel_last = None
        self._scan_key = None
        self._dots_key = None
        self._dot_indices = None

    def update(self, state):
        reference = state.get('reference')
        if reference != self.reference:
            self.reset()
            self.reference = reference
        point = state.get('t265_center_xy_m')
        if state.get('ready') and point is not None and len(point) == 2 and np.isfinite(point).all():
            point = tuple(point)
            # Display decimation only. Preserve SDK jumps, never smooth them
            # into a fictitious good trajectory; raw logs remain full-rate.
            if not self.trail or np.linalg.norm(np.subtract(point, self.trail[-1])) >= .01:
                self.trail.append(point)
        fit = state.get('last_fit') or {}
        self.axes = fit.get('position_observed_axes', [])
        source=state.get('matched_points_field', [])
        key=(fit.get('scan_id',id(source)),tuple(self.axes),state.get('lidar_corrections_enabled',True))
        if key != self._scan_key:
            self._scan_key=key
            self.points=source[:256] if self.axes and key[2] else []

    def draw(self, canvas, left, top, size):
        # Clip rendering to the map, so even a bad SDK coordinate cannot paint
        # over the controls. This does not clamp or correct any position.
        region = canvas[top:top+size+1, left:left+size+1]
        def pixels(points):
            p = np.asarray(points, dtype=float)
            return np.rint(np.clip((p * [1, -1] + 1.5) * (size / 3.), -1e6, 1e6)).astype(np.int32)
        view=(left,top,size)
        n=len(self.trail)
        last=self.trail[-1] if n else None
        if view!=self._view or n!=self._pixel_count or last!=self._pixel_last:
            if (view==self._view and n>=2 and self.trail[-2]==self._pixel_last
                    and n in (self._pixel_count,self._pixel_count+1)):
                if n==self._pixel_count:self._trail_pixels[:n-1]=self._trail_pixels[1:n]
                self._trail_pixels[n-1]=pixels([last])[0]
            elif n:self._trail_pixels[:n]=pixels(self.trail)
            self._pixel_count=n;self._pixel_last=last
        if n>=2:cv2.polylines(region,[self._trail_pixels[:n]],False,(90,215,90),1,cv2.LINE_AA)
        dots_key=(view,self._scan_key)
        if dots_key!=self._dots_key:
            self._dot_indices=None
            if self.points:
                points=np.asarray(self.points,dtype=float)
                points=points[np.isfinite(points).all(axis=1)]
                if len(points):
                    p=pixels(points)
                    p=p[(p[:,0]>=0)&(p[:,0]<=size)&(p[:,1]>=0)&(p[:,1]<=size)]
                    if len(p):
                        dx=np.array([-1,0,1,-1,0,1,-1,0,1]);dy=np.repeat([-1,0,1],3)
                        self._dot_indices=(np.clip(p[:,1,None]+dy,0,size),np.clip(p[:,0,None]+dx,0,size))
            self._dots_key=dots_key
        if self._dot_indices is not None:region[self._dot_indices]=(240,215,40)
        self._view=view
