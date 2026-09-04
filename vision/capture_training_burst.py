#!/usr/bin/env python3
"""Capture lossless-from-camera MJPEG bursts for detector training."""
from __future__ import annotations

import argparse
import csv
import queue
import signal
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except (ImportError, ValueError) as exc:
    raise RuntimeError(
        "缺少GStreamer Python绑定，请安装python3-gi和gir1.2-gstreamer-1.0"
    ) from exc

from rescue_vision.camera import resolve_camera_device


NATIVE_WIDTH = 1280
NATIVE_HEIGHT = 1024
BUTTON_RECT = (20, 20, 285, 78)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="1280x1024训练图片连拍工具")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--width", type=int, default=NATIVE_WIDTH)
    parser.add_argument("--height", type=int, default=NATIVE_HEIGHT)
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--burst-count", type=int, default=20)
    parser.add_argument("--interval-ms", type=float, default=500.0)
    parser.add_argument("--preview-width", type=int, default=960)
    parser.add_argument("--preview-fps", type=float, default=30.0)
    parser.add_argument("--output", type=Path, default=root / "captures" / "training")
    return parser.parse_args()


class BurstCamera:
    def __init__(self, args: argparse.Namespace) -> None:
        Gst.init(None)
        self.args = args
        self.lock = threading.Lock()
        self.latest_jpeg: bytes | None = None
        self.latest_sequence = 0
        self.arrival_times: deque[int] = deque(maxlen=181)
        self.capture_remaining = 0
        self.capture_total = 0
        self.capture_queued = 0
        self.saved_count = 0
        self.next_capture_ns = 0
        self.last_selected_ns = 0
        self.current_directory: Path | None = None
        self.write_error: str | None = None
        self.write_queue: queue.Queue[tuple[Path, int, int, float, bytes] | None] = queue.Queue()
        self.writer = threading.Thread(target=self._write_loop, name="training-image-writer", daemon=True)
        self.writer.start()

        device = resolve_camera_device(args.device)
        description = (
            f"v4l2src device={device} io-mode=mmap do-timestamp=true ! "
            f"image/jpeg,width={args.width},height={args.height},framerate={args.camera_fps}/1 ! "
            "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
            "appsink name=capture_sink emit-signals=true max-buffers=1 drop=true sync=false"
        )
        self.pipeline = Gst.parse_launch(description)
        self.sink = self.pipeline.get_by_name("capture_sink")
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
            jpeg = bytes(mapping.data)
        finally:
            buffer.unmap(mapping)

        captured_ns = time.monotonic_ns()
        selected: tuple[Path, int, int, float, bytes] | None = None
        with self.lock:
            self.latest_jpeg = jpeg
            self.latest_sequence += 1
            self.arrival_times.append(captured_ns)
            if self.capture_remaining > 0 and captured_ns >= self.next_capture_ns:
                self.capture_queued += 1
                index = self.capture_queued
                delta_ms = (
                    0.0 if self.last_selected_ns == 0
                    else (captured_ns - self.last_selected_ns) / 1_000_000.0
                )
                self.last_selected_ns = captured_ns
                self.capture_remaining -= 1
                self.next_capture_ns = captured_ns + round(self.args.interval_ms * 1_000_000.0)
                selected = (self.current_directory, index, captured_ns, delta_ms, jpeg)
        if selected is not None:
            self.write_queue.put(selected)
        return Gst.FlowReturn.OK

    def _write_loop(self) -> None:
        while True:
            item = self.write_queue.get()
            if item is None:
                self.write_queue.task_done()
                return
            directory, index, captured_ns, delta_ms, jpeg = item
            filename = f"frame_{index:03d}_{captured_ns}.jpg"
            try:
                (directory / filename).write_bytes(jpeg)
                with (directory / "timestamps.csv").open("a", newline="", encoding="utf-8") as stream:
                    csv.writer(stream).writerow((index, captured_ns, f"{delta_ms:.3f}", filename))
                with self.lock:
                    self.saved_count += 1
            except OSError as exc:
                with self.lock:
                    self.write_error = str(exc)
            finally:
                self.write_queue.task_done()

    def start(self) -> None:
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("摄像头启动失败，设备可能被占用")
        result, _, _ = self.pipeline.get_state(5 * Gst.SECOND)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("摄像头未能进入PLAYING状态")

    def check_error(self) -> str | None:
        message = self.bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if message is None:
            return None
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            return f"GStreamer错误：{error}; {debug}"
        return "摄像头流结束"

    def trigger(self) -> Path | None:
        with self.lock:
            busy = self.capture_remaining > 0 or self.saved_count < self.capture_total
        if busy:
            return None
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        directory = self.args.output.resolve() / f"burst_{timestamp}"
        directory.mkdir(parents=True, exist_ok=False)
        with (directory / "timestamps.csv").open("w", newline="", encoding="utf-8") as stream:
            csv.writer(stream).writerow(("index", "monotonic_ns", "delta_ms", "filename"))
        with self.lock:
            self.current_directory = directory
            self.capture_total = self.args.burst_count
            self.capture_remaining = self.args.burst_count
            self.capture_queued = 0
            self.saved_count = 0
            self.next_capture_ns = 0
            self.last_selected_ns = 0
            self.write_error = None
        return directory

    def snapshot(self) -> tuple[bytes | None, int, str, float]:
        with self.lock:
            jpeg = self.latest_jpeg
            sequence = self.latest_sequence
            remaining = self.capture_remaining
            queued = self.capture_queued
            saved = self.saved_count
            total = self.capture_total
            directory = self.current_directory
            error = self.write_error
            arrivals = list(self.arrival_times)
        source_fps = 0.0
        if len(arrivals) >= 2 and arrivals[-1] > arrivals[0]:
            source_fps = (len(arrivals) - 1) * 1_000_000_000.0 / (arrivals[-1] - arrivals[0])
        if error:
            status = f"SAVE ERROR: {error}"
        elif remaining > 0:
            status = f"CAPTURING {queued}/{total}"
        elif total > 0 and saved < total:
            status = f"SAVING {saved}/{total}"
        elif total > 0:
            status = f"SAVED {saved}: {directory.name}"
        else:
            status = "READY"
        return jpeg, sequence, status, source_fps

    def stop(self) -> None:
        self.pipeline.set_state(Gst.State.NULL)
        self.pipeline.get_state(2 * Gst.SECOND)
        self.write_queue.put(None)
        self.write_queue.join()
        self.writer.join(timeout=2.0)


def fit_preview(image: np.ndarray, target_width: int) -> np.ndarray:
    scale = min(1.0, target_width / image.shape[1])
    size = (max(1, round(image.shape[1] * scale)), max(1, round(image.shape[0] * scale)))
    return cv2.resize(image, size, interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR)


def draw_overlay(image: np.ndarray, status: str, source_fps: float, count: int) -> None:
    x0, y0, x1, y1 = BUTTON_RECT
    cv2.rectangle(image, (x0, y0), (x1, y1), (40, 170, 40), -1)
    cv2.rectangle(image, (x0, y0), (x1, y1), (255, 255, 255), 2)
    cv2.putText(image, f"CAPTURE {count} FRAMES", (x0 + 15, y0 + 37),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.rectangle(image, (0, image.shape[0] - 68), (image.shape[1], image.shape[0]), (0, 0, 0), -1)
    cv2.putText(image, f"{status}  camera={source_fps:.1f}fps", (18, image.shape[0] - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "Click button or press Space/B; Q/Esc quits", (18, image.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1, cv2.LINE_AA)


def main() -> int:
    args = parse_args()
    if (args.width, args.height) != (NATIVE_WIDTH, NATIVE_HEIGHT):
        raise ValueError("训练采集固定使用1280x1024")
    if args.camera_fps <= 0 or args.burst_count <= 0 or args.interval_ms < 0:
        raise ValueError("camera-fps和burst-count必须为正数，interval-ms不能为负数")
    if args.preview_width <= 0 or args.preview_fps <= 0:
        raise ValueError("preview-width和preview-fps必须为正数")

    frame_period_ms = 1000.0 / args.camera_fps
    if args.interval_ms < frame_period_ms:
        print(
            f"提示：请求间隔{args.interval_ms:g}ms小于{args.camera_fps}FPS摄像头的"
            f"{frame_period_ms:.3f}ms帧周期；将保存每个新到达的真实帧，不生成重复帧。"
        )
    print(f"输出目录：{args.output.resolve()}")

    running = True
    camera = BurstCamera(args)

    def stop(_signal, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    window = "training burst capture"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

    def mouse(event, x, y, _flags, _parameter):
        if event == cv2.EVENT_LBUTTONUP:
            x0, y0, x1, y1 = BUTTON_RECT
            if x0 <= x <= x1 and y0 <= y <= y1:
                directory = camera.trigger()
                if directory:
                    print(f"开始连拍：{directory}")

    cv2.setMouseCallback(window, mouse)
    camera.start()
    last_sequence = -1
    latest_view: np.ndarray | None = None
    next_preview = 0.0
    try:
        while running:
            error = camera.check_error()
            if error:
                raise RuntimeError(error)
            jpeg, sequence, status, source_fps = camera.snapshot()
            now = time.perf_counter()
            if jpeg is not None and sequence != last_sequence and now >= next_preview:
                decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if decoded is not None:
                    latest_view = fit_preview(decoded, args.preview_width)
                last_sequence = sequence
                next_preview = now + 1.0 / args.preview_fps
            if latest_view is not None:
                shown = latest_view.copy()
                draw_overlay(shown, status, source_fps, args.burst_count)
                cv2.imshow(window, shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord(" "), ord("b"), ord("B")):
                directory = camera.trigger()
                if directory:
                    print(f"开始连拍：{directory}")
            time.sleep(0.001)
    finally:
        camera.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}")
        raise SystemExit(1)
