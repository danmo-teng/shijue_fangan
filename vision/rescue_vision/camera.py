from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from glob import glob
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except (ImportError, ValueError) as exc:
    raise RuntimeError(
        "缺少GStreamer Python绑定，请安装python3-gi和gir1.2-gstreamer-1.0"
    ) from exc


@dataclass(frozen=True)
class CameraFrame:
    frame_id: int
    image: np.ndarray
    published_ns: int
    pixel_format: str = "bgr"


def resolve_camera_device(requested: str) -> str:
    """Use a stable UVC video-index0 link if /dev/videoN was renumbered."""
    if requested == "auto":
        requested = ""
    if requested and Path(requested).exists():
        return requested
    candidates = sorted(glob("/dev/v4l/by-id/*-video-index0"))
    if len(candidates) == 1:
        print(f"摄像头节点 {requested or 'auto'} 不存在，自动使用 {candidates[0]}", file=sys.stderr)
        return candidates[0]
    if not candidates:
        raise RuntimeError(f"摄像头节点不存在：{requested or 'auto'}，且未找到video-index0设备")
    raise RuntimeError(
        f"摄像头节点不存在：{requested or 'auto'}；检测到多个摄像头，请明确指定：{candidates}")


class LatestFrameCamera:
    def __init__(self, device: str, width: int, height: int, fps: int,
                 decoder: str = "jpu", decode_fps: float = 60.0,
                 output_format: str = "bgr") -> None:
        Gst.init(None)
        self.width = width
        self.height = height
        self.decoder = decoder
        self.decode_fps = decode_fps
        self.output_format = output_format
        self._lock = threading.Lock()
        self._latest: Optional[CameraFrame] = None
        self._decoded = 0
        self._captured = 0
        self._compressed = None
        self._worker_error: Optional[str] = None
        self._running = False
        self._decode_thread: Optional[threading.Thread] = None
        self._jpu = None
        self._start_ns = time.monotonic_ns()
        if decoder not in ("jpu", "software"):
            raise ValueError("decoder must be jpu or software")
        if output_format not in ("bgr", "nv12"):
            raise ValueError("output_format must be bgr or nv12")
        if decoder != "jpu" and output_format != "bgr":
            raise ValueError("NV12 output currently requires the JPU decoder")
        source = (
            f"v4l2src device={device} io-mode=mmap do-timestamp=true ! "
            f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
            "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
        )
        if decoder == "jpu":
            description = source + "appsink name=vision_sink emit-signals=true max-buffers=1 drop=true sync=false"
        else:
            description = source + (
                "jpegparse ! jpegdec idct-method=ifast ! videoconvert ! video/x-raw,format=BGR ! "
                "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
                "appsink name=vision_sink emit-signals=true max-buffers=1 drop=true sync=false"
            )
        self.pipeline = Gst.parse_launch(description)
        self.sink = self.pipeline.get_by_name("vision_sink")
        self.sink.connect("new-sample", self._on_sample)
        self.bus = self.pipeline.get_bus()

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        ok, mapping = buffer.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            if self.decoder == "jpu":
                compressed = bytes(mapping.data)
                captured_ns = time.monotonic_ns()
                with self._lock:
                    self._captured += 1
                    self._compressed = (self._captured, compressed, captured_ns)
                return Gst.FlowReturn.OK
            expected = self.width * self.height * 3
            if mapping.size < expected:
                return Gst.FlowReturn.ERROR
            image = np.frombuffer(mapping.data, np.uint8, expected)
            image = image.reshape(self.height, self.width, 3).copy()
        finally:
            buffer.unmap(mapping)
        with self._lock:
            self._decoded += 1
            self._latest = CameraFrame(self._decoded, image, time.monotonic_ns(), "bgr")
        return Gst.FlowReturn.OK

    def _jpu_decode_loop(self) -> None:
        from .jpu import JpuDecodeTimeout

        last_captured = 0
        consecutive_timeouts = 0
        period = 1.0 / max(self.decode_fps, 1.0)
        next_decode = time.perf_counter()
        try:
            while self._running:
                now = time.perf_counter()
                if now < next_decode:
                    time.sleep(min(next_decode - now, 0.002))
                    continue
                next_decode = now + period
                with self._lock:
                    packet = self._compressed
                if packet is None or packet[0] == last_captured:
                    time.sleep(0.0005)
                    continue
                captured_id, jpeg, captured_ns = packet
                try:
                    image = (
                        self._jpu.decode_nv12(jpeg)
                        if self.output_format == "nv12"
                        else self._jpu.decode(jpeg)
                    )
                    consecutive_timeouts = 0
                except JpuDecodeTimeout:
                    consecutive_timeouts += 1
                    last_captured = captured_id
                    if consecutive_timeouts <= 5:
                        continue
                    raise RuntimeError("JPU连续5帧输出超时")
                last_captured = captured_id
                with self._lock:
                    self._decoded += 1
                    self._latest = CameraFrame(
                        self._decoded, image, captured_ns, self.output_format
                    )
        except Exception as error:
            with self._lock:
                self._worker_error = str(error)
            self._running = False

    def start(self) -> None:
        if self.decoder == "jpu":
            from .jpu import JpuDecoder
            self._jpu = JpuDecoder(self.width, self.height)
            self._running = True
            self._decode_thread = threading.Thread(
                target=self._jpu_decode_loop, name="jpu-latest-frame", daemon=True)
            self._decode_thread.start()
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            self.stop()
            raise RuntimeError("摄像头启动失败，设备可能被占用")
        result, _, _ = self.pipeline.get_state(5 * Gst.SECOND)
        if result == Gst.StateChangeReturn.FAILURE:
            self.stop()
            raise RuntimeError("摄像头未能在5秒内进入PLAYING状态")

    def latest(self) -> Optional[CameraFrame]:
        with self._lock:
            return self._latest

    def decoded_count(self) -> int:
        with self._lock:
            return self._decoded

    def captured_count(self) -> int:
        with self._lock:
            return self._captured if self.decoder == "jpu" else self._decoded

    def check_error(self) -> Optional[str]:
        with self._lock:
            if self._worker_error:
                return f"JPU错误：{self._worker_error}"
        message = self.bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if message is None:
            return None
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            return f"GStreamer错误：{error}; {debug}"
        return "摄像头流结束"

    def stop(self) -> None:
        self._running = False
        self.pipeline.set_state(Gst.State.NULL)
        self.pipeline.get_state(2 * Gst.SECOND)
        if self._decode_thread is not None:
            self._decode_thread.join(timeout=2.0)
            self._decode_thread = None
        if self._jpu is not None:
            self._jpu.close()
            self._jpu = None
