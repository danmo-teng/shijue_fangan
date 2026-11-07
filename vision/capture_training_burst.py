#!/usr/bin/env python3
"""Capture lossless-from-camera MJPEG bursts for detector training."""
from __future__ import annotations

import argparse
import csv
import math
import queue
import signal
import threading
import time
import tkinter as tk
import zipfile
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

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


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="1280x1024训练图片连拍工具")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--width", type=int, default=NATIVE_WIDTH)
    parser.add_argument("--height", type=int, default=NATIVE_HEIGHT)
    parser.add_argument("--camera-fps", type=int, default=180)
    parser.add_argument("--burst-count", type=int, default=20)
    parser.add_argument("--interval-ms", type=float, default=500.0)
    parser.add_argument("--start-number", type=int, default=1)
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
        self.finished_count = 0
        self.session_saved = 0
        self.next_number = args.start_number
        self.interval_ms = args.interval_ms
        self.next_capture_ns = 0
        self.last_selected_ns = 0
        self.current_directory: Path | None = None
        self.write_error: str | None = None
        self.write_queue: queue.Queue[tuple[Path, int, int, float, bytes] | None] = queue.Queue()

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
        self.writer = threading.Thread(target=self._write_loop, name="training-image-writer", daemon=True)
        self.writer.start()

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
        with self.lock:
            self.latest_jpeg = jpeg
            self.latest_sequence += 1
            self.arrival_times.append(captured_ns)
            if self.capture_remaining > 0 and captured_ns >= self.next_capture_ns:
                self.capture_queued += 1
                index = self.next_number
                self.next_number += 1
                delta_ms = (
                    0.0 if self.last_selected_ns == 0
                    else (captured_ns - self.last_selected_ns) / 1_000_000.0
                )
                self.last_selected_ns = captured_ns
                self.capture_remaining -= 1
                self.next_capture_ns = captured_ns + round(self.interval_ms * 1_000_000.0)
                self.write_queue.put((self.current_directory, index, captured_ns, delta_ms, jpeg))
        return Gst.FlowReturn.OK

    def _write_loop(self) -> None:
        while True:
            item = self.write_queue.get()
            if item is None:
                self.write_queue.task_done()
                return
            directory, index, captured_ns, delta_ms, jpeg = item
            filename = f"A{index}.jpg"
            try:
                with (directory / filename).open("xb") as stream:
                    stream.write(jpeg)
                with (directory / "timestamps.csv").open("a", newline="", encoding="utf-8") as stream:
                    csv.writer(stream).writerow((index, captured_ns, f"{delta_ms:.3f}", filename))
                with self.lock:
                    self.saved_count += 1
                    self.session_saved += 1
            except OSError as exc:
                with self.lock:
                    self.write_error = str(exc)
                    self.capture_remaining = 0
            finally:
                with self.lock:
                    self.finished_count += 1
                self.write_queue.task_done()

    def start(self) -> None:
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("摄像头启动失败，设备可能被占用")
        result, state, _ = self.pipeline.get_state(5 * Gst.SECOND)
        if result == Gst.StateChangeReturn.FAILURE or state != Gst.State.PLAYING:
            raise RuntimeError("摄像头未能进入PLAYING状态")

    def check_error(self) -> str | None:
        message = self.bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if message is None:
            return None
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            return f"GStreamer错误：{error}; {debug}"
        return "摄像头流结束"

    def trigger(self, count: int, interval_ms: float, start_number: int) -> Path:
        if count <= 0 or start_number < 0 or not math.isfinite(interval_ms) or interval_ms < 0:
            raise ValueError("张数须大于0，起始编号和间隔不能为负数，间隔须为有限数值")
        with self.lock:
            busy = self.capture_remaining > 0 or self.finished_count < self.capture_queued
        if busy:
            raise ValueError("本轮拍摄或写盘尚未完成")
        directory = self.current_directory
        if directory is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            directory = self.args.output.resolve() / f"session_{timestamp}"
            directory.mkdir(parents=True, exist_ok=False)
            with (directory / "timestamps.csv").open("x", newline="", encoding="utf-8") as stream:
                csv.writer(stream).writerow(("number", "monotonic_ns", "delta_ms", "filename"))
            self.current_directory = directory
        for path in directory.glob("A*.jpg"):
            number = path.stem[1:]
            if number.isdigit() and start_number <= int(number) < start_number + count:
                raise ValueError(f"文件已存在：{path.name}。请更换起始编号，原图片不会被覆盖。")
        with self.lock:
            self.capture_total = count
            self.capture_remaining = count
            self.capture_queued = 0
            self.saved_count = 0
            self.finished_count = 0
            self.next_number = start_number
            self.interval_ms = interval_ms
            self.next_capture_ns = 0
            self.last_selected_ns = 0
            self.write_error = None
        return directory

    def snapshot(self) -> dict:
        with self.lock:
            jpeg = self.latest_jpeg
            sequence = self.latest_sequence
            remaining = self.capture_remaining
            queued = self.capture_queued
            saved = self.saved_count
            finished = self.finished_count
            session_saved = self.session_saved
            next_number = self.next_number
            total = self.capture_total
            directory = self.current_directory
            error = self.write_error
            arrivals = list(self.arrival_times)
        source_fps = 0.0
        if len(arrivals) >= 2 and arrivals[-1] > arrivals[0]:
            source_fps = (len(arrivals) - 1) * 1_000_000_000.0 / (arrivals[-1] - arrivals[0])
        if error:
            status = f"保存失败：{error}"
        elif remaining > 0:
            status = f"拍摄中：已选取 {queued}/{total} 张，已保存 {saved} 张"
        elif finished < queued:
            status = f"写盘中：{saved}/{queued} 张"
        elif total > 0:
            status = f"本轮已保存 {saved} 张；本次启动累计 {session_saved} 张"
        else:
            status = "就绪：填写参数后点击开始拍摄"
        return dict(jpeg=jpeg, sequence=sequence, status=status, source_fps=source_fps,
                    busy=remaining > 0 or finished < queued, next_number=next_number,
                    directory=directory, session_saved=session_saved)

    def cancel_capture(self) -> None:
        with self.lock:
            self.capture_remaining = 0

    def stop(self) -> None:
        self.cancel_capture()
        self.pipeline.set_state(Gst.State.NULL)
        self.pipeline.get_state(2 * Gst.SECOND)
        self.write_queue.put(None)
        self.write_queue.join()
        self.writer.join(timeout=2.0)


def archive_session(directory: Path) -> Path:
    """Keep originals; replace the ZIP only after the entire new archive is written."""
    archive = directory.with_suffix(".zip")
    temporary = archive.with_suffix(".zip.tmp")
    try:
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=1) as output:
            for path in sorted(directory.rglob("*")):
                if path.is_file():
                    output.write(path, arcname=str(path.relative_to(directory.parent)))
        temporary.replace(archive)
    finally:
        temporary.unlink(missing_ok=True)
    return archive


class CaptureWindow:
    def __init__(self, root: tk.Tk, camera: BurstCamera, args: argparse.Namespace) -> None:
        self.root, self.camera, self.args = root, camera, args
        self.closing = False
        self.was_busy = False
        self.last_sequence = -1
        self.worker: threading.Thread | None = None
        self.worker_result: queue.Queue[tuple[str, str | None]] = queue.Queue()
        root.title("比赛摄像头批量拍摄")
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        self.preview = ttk.Label(root, text="正在等待摄像头画面…", anchor="center")
        self.preview.grid(row=0, column=0, sticky="nsew", padx=8, pady=6)
        controls = ttk.Frame(root, padding=8)
        controls.grid(row=1, column=0, sticky="ew")
        self.count = tk.StringVar(value=str(args.burst_count))
        self.interval = tk.StringVar(value=f"{args.interval_ms / 1000:g}")
        self.number = tk.StringVar(value=str(args.start_number))
        self.entries = []
        for column, (label, variable) in enumerate((("连续张数", self.count),
                                                  ("间隔（秒）", self.interval),
                                                  ("起始编号 A", self.number))):
            ttk.Label(controls, text=label).grid(row=0, column=column * 2, padx=(0, 4))
            entry = ttk.Entry(controls, textvariable=variable, width=9)
            entry.grid(row=0, column=column * 2 + 1, padx=(0, 12))
            self.entries.append(entry)
        self.capture_button = ttk.Button(controls, text="开始拍摄", command=self.capture)
        self.capture_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=8)
        self.cancel_button = ttk.Button(controls, text="停止本轮拍摄", command=camera.cancel_capture)
        self.cancel_button.grid(row=1, column=2, columnspan=2, sticky="ew", padx=6, pady=8)
        self.zip_button = ttk.Button(controls, text="压缩本次文件夹（ZIP）", command=self.compress)
        self.zip_button.grid(row=1, column=4, columnspan=2, sticky="ew", pady=8)
        self.zip_on_exit = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls, text="退出时压缩（保留原图片）", variable=self.zip_on_exit).grid(
            row=2, column=0, columnspan=4, sticky="w")
        ttk.Button(controls, text="退出", command=self.close).grid(row=2, column=4, columnspan=2, sticky="e")
        self.status = tk.StringVar()
        self.folder = tk.StringVar(value=f"保存位置：{args.output.resolve()}（首次拍摄时建立本次文件夹）")
        self.archive_status = tk.StringVar()
        for row, variable in enumerate((self.status, self.folder, self.archive_status), start=2):
            ttk.Label(root, textvariable=variable, wraplength=min(args.preview_width, root.winfo_screenwidth() - 40),
                      padding=(8, 2)).grid(row=row, column=0, sticky="w")
        ttk.Label(root, text="MJPEG 1280×1024；沿用当前曝光/焦距/白平衡；保存原始JPEG，不含预览文字。",
                  padding=(8, 4)).grid(row=5, column=0, sticky="w")
        root.geometry(f"{min(args.preview_width + 30, root.winfo_screenwidth() - 40)}x"
                      f"{min(900, root.winfo_screenheight() - 60)}")
        root.minsize(680, 520)
        root.after(0, self.update)

    def capture(self) -> None:
        if self.closing or (self.worker and self.worker.is_alive()):
            return
        try:
            count, number = int(self.count.get()), int(self.number.get())
            interval_ms = float(self.interval.get()) * 1000
            directory = self.camera.trigger(count, interval_ms, number)
        except (ValueError, OSError) as exc:
            messagebox.showerror("无法开始拍摄", str(exc), parent=self.root)
            return
        self.was_busy = True
        self.folder.set(f"本次文件夹：{directory}")
        self.archive_status.set("新增拍摄后可重新压缩，ZIP与图片文件夹保存在同一位置。")

    def compress(self) -> None:
        state = self.camera.snapshot()
        if self.closing or state["busy"] or (self.worker and self.worker.is_alive()):
            return
        if not state["directory"]:
            messagebox.showinfo("尚未拍摄", "请先拍摄图片，再压缩本次文件夹。", parent=self.root)
            return
        self.archive_status.set("正在后台压缩…")
        self.worker = threading.Thread(target=self._archive, args=(state["directory"],), daemon=True)
        self.worker.start()

    def _archive(self, directory: Path) -> None:
        try:
            self.worker_result.put((f"已压缩：{archive_session(directory)}", None))
        except Exception as exc:
            self.worker_result.put(("压缩失败，原图片保留。", str(exc)))

    def update(self) -> None:
        if self.closing:
            return
        state = self.camera.snapshot()
        if self.was_busy and not state["busy"]:
            self.number.set(str(state["next_number"]))
            self.was_busy = False
        worker_busy = self.worker is not None and self.worker.is_alive()
        editable = not state["busy"] and not worker_busy
        self.capture_button.configure(state="normal" if editable else "disabled")
        self.zip_button.configure(state="normal" if editable and state["directory"] else "disabled")
        self.cancel_button.configure(state="normal" if state["busy"] else "disabled")
        for entry in self.entries:
            entry.configure(state="normal" if editable else "disabled")
        self.status.set(f"{state['status']}；相机 {state['source_fps']:.1f} FPS")
        self._show_worker_result()
        error = self.camera.check_error()
        if error:
            messagebox.showerror("摄像头错误", error, parent=self.root)
            self.close()
            return
        if state["jpeg"] is not None and state["sequence"] != self.last_sequence:
            decoded = cv2.imdecode(np.frombuffer(state["jpeg"], dtype=np.uint8), cv2.IMREAD_COLOR)
            if decoded is not None:
                image = Image.fromarray(cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB))
                image.thumbnail((max(1, self.preview.winfo_width()), max(1, self.preview.winfo_height())))
                self.photo = ImageTk.PhotoImage(image)
                self.preview.configure(image=self.photo, text="")
            self.last_sequence = state["sequence"]
        self.root.after(max(1, round(1000 / self.args.preview_fps)), self.update)

    def _show_worker_result(self) -> None:
        while not self.worker_result.empty():
            text, error = self.worker_result.get_nowait()
            self.archive_status.set(text)
            if error:
                messagebox.showerror("文件处理失败", error, parent=self.root)

    def close(self) -> None:
        if self.closing:
            return
        self.closing = True
        self.status.set("正在结束拍摄并保存已采集图片…")
        self.capture_button.configure(state="disabled")
        self.cancel_button.configure(state="disabled")
        self.zip_button.configure(state="disabled")
        existing_worker = self.worker
        zip_on_exit = self.zip_on_exit.get()

        def finish() -> None:
            try:
                self.camera.stop()
                if existing_worker:
                    existing_worker.join()
                directory = self.camera.current_directory
                if zip_on_exit and directory:
                    self._archive(directory)
            except Exception as exc:
                self.worker_result.put(("退出时文件处理失败，原图片保留。", str(exc)))

        self.worker = threading.Thread(target=finish, name="capture-shutdown", daemon=True)
        self.worker.start()
        self.root.after(100, self._finish_close)

    def _finish_close(self) -> None:
        if self.worker.is_alive():
            self.root.after(100, self._finish_close)
            return
        self._show_worker_result()
        self.root.destroy()


def main() -> int:
    args = parse_args()
    if (args.width, args.height) != (NATIVE_WIDTH, NATIVE_HEIGHT):
        raise ValueError("训练采集固定使用1280x1024")
    if args.camera_fps <= 0 or args.burst_count <= 0 or not math.isfinite(args.interval_ms) or args.interval_ms < 0:
        raise ValueError("camera-fps和burst-count必须为正数，interval-ms不能为负数")
    if args.start_number < 0 or args.preview_width <= 0 or not math.isfinite(args.preview_fps) or args.preview_fps <= 0:
        raise ValueError("preview-width和preview-fps必须为正数")

    frame_period_ms = 1000.0 / args.camera_fps
    if args.interval_ms < frame_period_ms:
        print(
            f"提示：请求间隔{args.interval_ms:g}ms小于{args.camera_fps}FPS摄像头的"
            f"{frame_period_ms:.3f}ms帧周期；将保存每个新到达的真实帧，不生成重复帧。"
        )
    print(f"输出目录：{args.output.resolve()}")

    root = tk.Tk()
    root.withdraw()
    camera = None
    try:
        camera = BurstCamera(args)
        camera.start()
        window = CaptureWindow(root, camera, args)
    except Exception as exc:
        if camera is not None:
            camera.stop()
        messagebox.showerror("批量拍摄启动失败", f"{exc}\n请先关闭比赛或其他占用相机的程序。", parent=root)
        root.destroy()
        return 1
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    root.deiconify()
    root.mainloop()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"[ERROR] {error}")
        raise SystemExit(1)
