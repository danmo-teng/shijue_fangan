"""Bounded shared-frame YOLO26 worker; all mission/serial decisions stay in parent."""
from __future__ import annotations
import multiprocessing as mp
import queue
import time
import traceback
import numpy as np

WIDTH, HEIGHT = 1280, 1024
SLOT_BYTES = WIDTH * HEIGHT * 3


def _worker(buffers, free, requests, results, status, stop, model, labels, score, nms, priority, cores, preprocess):
    scaler = None
    try:
        import cv2
        from rescue_vision.yolo26 import X5Yolo26
        from rescue_vision.vse import VseScaler
        cv2.setNumThreads(1)
        detector = X5Yolo26(model, labels, score, nms, priority, cores)
        warning = None
        if preprocess != 'cpu':
            try:
                scaler = VseScaler(WIDTH, HEIGHT, 640, 512)
            except Exception as error:
                if preprocess == 'vse':
                    raise
                warning = str(error)
        status.put(('ready', {'shapes':detector.shapes, 'preprocess':'vse' if scaler else 'cpu', 'warning':warning}))
        while not stop.is_set():
            try:
                request = requests.get(timeout=.05)
            except queue.Empty:
                continue
            # Skip queued stale input; never build an unbounded camera backlog.
            while True:
                try:
                    newer = requests.get_nowait()
                except queue.Empty:
                    break
                free.put(request[0])
                request = newer
            slot, epoch, frame_id, captured_ns, pixel_format = request
            shape = (HEIGHT * 3 // 2, WIDTH) if pixel_format == 'nv12' else (HEIGHT, WIDTH, 3)
            image = np.frombuffer(buffers[slot], np.uint8, count=int(np.prod(shape))).reshape(shape)
            if pixel_format == 'nv12' and scaler:
                try:
                    detections, timing = detector.infer_nv12(image, WIDTH, HEIGHT, scaler)
                except RuntimeError as error:
                    if preprocess != 'auto' or 'VSE缩放失败' not in str(error):
                        raise
                    scaler.close()
                    scaler = None
                    status.put(('warning', str(error)))
                    detections, timing = detector.infer(cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12))
            else:
                detections, timing = detector.infer(cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12)
                                                    if pixel_format == 'nv12' else image)
            results.put((request, detections, timing))
    except Exception:
        status.put(('error', traceback.format_exc()))
    finally:
        if scaler:
            scaler.close()


class Yolo26Pipeline:
    input_width = input_height = 640

    def __init__(self, model, labels, score, nms, priority, cores, preprocess='auto'):
        context = mp.get_context('spawn')
        self.buffers = [context.RawArray('B', SLOT_BYTES) for _ in range(3)]
        self.free = context.Queue(3)
        self.requests, self.results, self.status = context.Queue(3), context.Queue(3), context.Queue()
        self.stop = context.Event()
        self.epoch = 0
        for slot in range(3):
            self.free.put(slot)
        self.process = context.Process(target=_worker, args=(self.buffers, self.free, self.requests,
            self.results, self.status, self.stop, str(model), labels, score, nms, priority, cores, preprocess), daemon=True)
        self.process.start()
        try:
            kind, info = self.status.get(timeout=30)
            if kind != 'ready':
                raise RuntimeError(str(info))
            self.shapes = info['shapes']
            self.preprocess = info['preprocess']
            if info['warning']:
                print('YOLO26 preprocessing fallback:', info['warning'], flush=True)
        except Exception:
            self.close()
            raise

    def reset(self):
        self.epoch += 1  # Discard results captured before a camera restart.

    def submit(self, packet):
        try:
            slot = self.free.get_nowait()
        except queue.Empty:
            return False
        image = packet.image
        expected = (HEIGHT * 3 // 2, WIDTH) if packet.pixel_format == "nv12" else (HEIGHT, WIDTH, 3)
        if image.dtype != np.uint8 or image.shape != expected or image.nbytes > SLOT_BYTES:
            self.free.put(slot)
            raise ValueError('Invalid camera frame for YOLO26 pipeline')
        np.frombuffer(self.buffers[slot], np.uint8, count=image.size)[:] = image.reshape(-1)
        self.requests.put_nowait((slot, self.epoch, packet.frame_id, packet.published_ns, packet.pixel_format))
        return True

    def receive(self):
        while True:
            try:
                kind, info = self.status.get_nowait()
            except queue.Empty:
                break
            if kind == 'error':
                raise RuntimeError(info)
            if kind == 'warning':
                self.preprocess = 'cpu'
                print('YOLO26 preprocessing fallback:', info, flush=True)
        if not self.process.is_alive():
            raise RuntimeError(f'YOLO26 worker exited: {self.process.exitcode}')
        chosen = None
        while True:
            try:
                item = self.results.get_nowait()
            except queue.Empty:
                break
            if chosen is not None:
                self.free.put(chosen[0][0])
            chosen = item
        if chosen is None:
            return None
        request, detections, timing = chosen
        slot, epoch, frame_id, captured_ns, pixel_format = request
        try:
            if epoch != self.epoch:
                return None
            from rescue_vision.camera import CameraFrame
            shape = (HEIGHT * 3 // 2, WIDTH) if pixel_format == 'nv12' else (HEIGHT, WIDTH, 3)
            image = np.frombuffer(self.buffers[slot], np.uint8, count=int(np.prod(shape))).reshape(shape).copy()
            return CameraFrame(frame_id, image, captured_ns, pixel_format), detections, timing
        finally:
            self.free.put(slot)

    def close(self):
        self.stop.set()
        self.process.join(3)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join()
        for pipe in (self.free, self.requests, self.results, self.status):
            pipe.cancel_join_thread()
            pipe.close()
