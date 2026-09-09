#!/usr/bin/env python3
"""Desktop gamepad teleoperation UI and F407 UART bridge.

The real F407 UART is opened only here. When the T265 map window is started,
the map builder receives a PTY path and all bytes from the F407 are mirrored to
that PTY. Bytes written back by the map builder are deliberately discarded so
the hand controller remains the only motion-command source.
"""

from __future__ import annotations

import argparse
import os
import pty
import queue
import select
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

try:  # Running as a package (unit tests / python -m).
    from .input_mapping import ArmGate, EvdevGamepad, command_from_pad
    from .protocol import (
        MSG_MOTION_STATUS,
        MotionStatus,
        StreamParser,
        decode_motion_status,
        state_name,
        stop_frame,
        teleop_frame,
    )
except ImportError:  # Running the file directly from the launcher script.
    from input_mapping import ArmGate, EvdevGamepad, command_from_pad
    from protocol import (
        MSG_MOTION_STATUS,
        MotionStatus,
        StreamParser,
        decode_motion_status,
        state_name,
        stop_frame,
        teleop_frame,
    )


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UART = "/dev/ttyS1"
DEFAULT_RATE = 50.0
DEFAULT_DEADZONE = 0.12
DEFAULT_PRECISION = 35


@dataclass(frozen=True)
class BridgeEvent:
    kind: str
    value: Any = None


class MapTunnel:
    """Own the PTY used by t265_map while the bridge owns the real UART."""

    def __init__(self, command: str, root: Path, emit: Callable[[BridgeEvent], None]) -> None:
        self.command = command
        self.root = Path(root)
        self.emit = emit
        self.master: int | None = None
        self.slave: int | None = None
        self.process: subprocess.Popen[bytes] | None = None
        self.log_file: Any = None

    @property
    def active(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> bool:
        if self.active:
            self.emit(BridgeEvent("info", "T265建图界面已经在运行"))
            return True
        if not self.command.strip():
            self.emit(BridgeEvent("error", "没有配置T265建图启动命令"))
            return False
        try:
            self.master, self.slave = pty.openpty()
            slave_name = os.ttyname(self.slave)
            argv = [part.replace("{pty}", slave_name) for part in shlex.split(self.command)]
            if not argv:
                raise RuntimeError("T265建图启动命令为空")
            log_dir = self.root / "runtime" / "gamepad_teleop"
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / "t265_map.log"
            self.log_file = log_path.open("ab", buffering=0)
            self.process = subprocess.Popen(
                argv,
                cwd=self.root,
                stdin=subprocess.DEVNULL,
                stdout=self.log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            os.set_blocking(self.master, False)
            self.emit(BridgeEvent("map", True))
            self.emit(BridgeEvent("info", f"已进入T265建图界面（PTY {slave_name}）"))
            return True
        except (OSError, ValueError, RuntimeError) as error:
            self.emit(BridgeEvent("error", f"启动T265建图失败：{error}"))
            self._close_fds()
            if self.log_file is not None:
                self.log_file.close()
                self.log_file = None
            return False

    def stop(self) -> None:
        process = self.process
        self.process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
                except (OSError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except OSError:
                        pass
        self._close_fds()
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        self.emit(BridgeEvent("map", False))

    def check_exit(self) -> None:
        if self.process is not None and self.process.poll() is not None:
            code = self.process.returncode
            self.process = None
            self._close_fds()
            if self.log_file is not None:
                self.log_file.close()
                self.log_file = None
            self.emit(BridgeEvent("map", False))
            self.emit(BridgeEvent("info", f"T265建图界面已退出（退出码 {code}）"))

    def forward_uart_bytes(self, data: bytes) -> None:
        if self.master is None or not data:
            return
        view = memoryview(data)
        while view:
            try:
                count = os.write(self.master, view)
            except (BlockingIOError, OSError):
                return
            if count <= 0:
                return
            view = view[count:]

    def discard_map_bytes(self) -> None:
        if self.master is None:
            return
        while True:
            try:
                if not os.read(self.master, 4096):
                    return
            except (BlockingIOError, OSError):
                return

    def _close_fds(self) -> None:
        for descriptor in (self.master, self.slave):
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
        self.master = None
        self.slave = None


class TeleopBridge:
    """Background input/UART loop used by the desktop window."""

    def __init__(self, options: argparse.Namespace) -> None:
        self.options = options
        self.events: queue.Queue[BridgeEvent] = queue.Queue()
        self.commands: queue.Queue[str] = queue.Queue()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.map_tunnel = MapTunnel(options.map_command, ROOT, self.events.put)

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="f407-gamepad", daemon=True)
        self.thread.start()

    def request_map_start(self) -> None:
        self.commands.put("map_start")

    def request_map_stop(self) -> None:
        self.commands.put("map_stop")

    def close(self) -> None:
        self.stop_event.set()
        self.commands.put("map_stop")
        if self.thread is not None:
            self.thread.join(timeout=4.0)
        self.thread = None

    def _run(self) -> None:
        pad = None
        uart = None
        parser = StreamParser()
        sequence = 0
        gate = ArmGate(self.options.deadzone)
        next_send = time.monotonic()
        try:
            try:
                import serial
            except ImportError as error:
                raise RuntimeError(
                    "缺少 pyserial，请安装：sudo apt install python3-serial"
                ) from error
            if not 20.0 <= self.options.rate <= 100.0:
                raise ValueError("发送频率必须在20..100 Hz")
            if not 0.05 <= self.options.deadzone <= 0.35:
                raise ValueError("摇杆死区必须在0.05..0.35")
            if not 10 <= self.options.precision_percent <= 100:
                raise ValueError("精细速度必须在10..100%")

            pad = EvdevGamepad(self.options.device)
            uart = serial.Serial(
                self.options.uart,
                self.options.baud,
                timeout=0,
                write_timeout=0.05,
            )
            self.events.put(BridgeEvent("ready", pad.name))
            self.events.put(BridgeEvent("info", f"UART已打开：{self.options.uart}@{self.options.baud}"))
            period = 1.0 / self.options.rate
            last_print = 0.0
            while not self.stop_event.is_set():
                now = time.monotonic()
                readable = [pad.fd]
                if self.map_tunnel.master is not None:
                    readable.append(self.map_tunnel.master)
                timeout = max(0.0, min(0.02, next_send - now))
                try:
                    select.select(readable, [], [], timeout)
                except (OSError, ValueError):
                    pass

                for command in self._drain_commands():
                    if command == "map_start":
                        self.map_tunnel.start()
                    elif command == "map_stop":
                        self.map_tunnel.stop()

                actions = pad.poll()
                if "map_start" in actions:
                    self.map_tunnel.start()
                if "map_stop" in actions:
                    self.map_tunnel.stop()

                waiting = uart.in_waiting
                if waiting:
                    received = uart.read(waiting)
                    if received:
                        self.map_tunnel.forward_uart_bytes(received)
                        for frame in parser.feed(received):
                            if frame.message_type == MSG_MOTION_STATUS:
                                try:
                                    status = decode_motion_status(frame)
                                except ValueError:
                                    continue
                                self.events.put(BridgeEvent("status", status))
                self.map_tunnel.discard_map_bytes()
                self.map_tunnel.check_exit()

                now = time.monotonic()
                if now < next_send:
                    continue
                next_send += period
                if now - next_send > period:
                    next_send = now + period

                forward, left, yaw, camera, buttons, speed, _ = command_from_pad(
                    pad.state, self.options.deadzone, self.options.precision_percent
                )
                enabled = gate.update(pad.state)
                if not enabled:
                    # Do not leave a nonzero command queued while RB is up.
                    forward = left = yaw = camera = buttons = 0
                uart.write(
                    teleop_frame(
                        sequence,
                        forward,
                        left,
                        yaw,
                        camera,
                        buttons,
                        speed,
                        enabled,
                    )
                )
                sequence = (sequence + 1) & 0xFF
                if now - last_print >= 1.0:
                    self.events.put(
                        BridgeEvent(
                            "command",
                            (enabled, forward, left, yaw, camera, buttons, speed),
                        )
                    )
                    last_print = now
        except (OSError, RuntimeError, ValueError) as error:
            self.events.put(BridgeEvent("error", str(error)))
        finally:
            if uart is not None:
                try:
                    # Three disabled frames cover a short USB/UART scheduling
                    # delay before the explicit STOP reaches the F407.
                    for _ in range(3):
                        uart.write(teleop_frame(sequence, 0, 0, 0, 0, 0, 0, False))
                        sequence = (sequence + 1) & 0xFF
                        time.sleep(0.02)
                    uart.write(stop_frame(sequence))
                    uart.flush()
                except (OSError, RuntimeError):
                    pass
                try:
                    uart.close()
                except OSError:
                    pass
            self.map_tunnel.stop()
            if pad is not None:
                pad.close()
            self.events.put(BridgeEvent("stopped"))

    def _drain_commands(self) -> list[str]:
        commands = []
        while True:
            try:
                commands.append(self.commands.get_nowait())
            except queue.Empty:
                return commands


class GamepadWindow:
    """Small Tk interface that explains the physical controller layout."""

    def __init__(self, options: argparse.Namespace) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.options = options
        self.root = tk.Tk()
        self.root.title("F407 手柄控制模式")
        self.root.geometry("1180x880")
        self.root.minsize(900, 680)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.status_var = tk.StringVar(value="正在启动手柄控制桥接…")
        self.detail_var = tk.StringVar(value="按住 RB 才允许小车和舵机动作")
        self.map_var = tk.StringVar(value="T265：未启动")
        self.f407_var = tk.StringVar(value="F407状态：等待回报")
        self.command_var = tk.StringVar(value="指令：F +0  L +0  Y +0  CAM +0  SPD 0%")

        top = ttk.Frame(self.root, padding=(12, 10, 12, 4))
        top.pack(fill="x")
        ttk.Label(top, textvariable=self.status_var, font=("Sans", 13, "bold")).pack(anchor="w")
        ttk.Label(top, textvariable=self.detail_var).pack(anchor="w")
        ttk.Label(top, textvariable=self.f407_var).pack(anchor="w")
        ttk.Label(top, textvariable=self.command_var, font=("Monospace", 10)).pack(anchor="w")

        buttons = ttk.Frame(top)
        buttons.pack(anchor="w", pady=(6, 0))
        ttk.Button(buttons, text="进入 T265 建图（X）", command=self.start_map).pack(
            side="left", padx=(0, 6)
        )
        ttk.Button(buttons, text="退出 T265 建图（Y）", command=self.stop_map).pack(side="left")
        ttk.Label(buttons, textvariable=self.map_var, padding=(12, 0, 0, 0)).pack(side="left")

        canvas_frame = ttk.Frame(self.root, padding=(12, 4, 12, 0))
        canvas_frame.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(canvas_frame, bg="#f5f7fb", highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(canvas_frame, orient="vertical", command=self.canvas.yview)
        scrollbar.pack(side="right", fill="y")
        self.canvas.configure(yscrollcommand=scrollbar.set, scrollregion=(0, 0, 1140, 930))
        self._draw_controller()

        footer = ttk.Label(
            self.root,
            text="X 启动建图；Y 退出建图。建图程序通过PTY接收里程计，运动指令仍由手柄控制。",
            padding=(12, 6),
        )
        footer.pack(fill="x")

        self.bridge = TeleopBridge(options)
        self.bridge.start()
        self.root.after(100, self._pump_events)

    def start_map(self) -> None:
        self.bridge.request_map_start()

    def stop_map(self) -> None:
        self.bridge.request_map_stop()

    def close(self) -> None:
        self.bridge.close()
        self.root.destroy()

    def _pump_events(self) -> None:
        try:
            while True:
                event = self.bridge.events.get_nowait()
                self._handle_event(event)
        except queue.Empty:
            pass
        if self.root.winfo_exists():
            self.root.after(100, self._pump_events)

    def _handle_event(self, event: BridgeEvent) -> None:
        if event.kind == "ready":
            self.status_var.set(f"手柄：{event.value}")
        elif event.kind == "info":
            self.detail_var.set(str(event.value))
        elif event.kind == "error":
            self.status_var.set(f"错误：{event.value}")
            self.detail_var.set("请检查手柄、evdev权限、/dev/ttyS1和依赖安装")
        elif event.kind == "map":
            self.map_var.set("T265：运行中" if event.value else "T265：未启动")
        elif event.kind == "status":
            status: MotionStatus = event.value
            suffix = f" fault={status.fault}" if status.fault else ""
            self.f407_var.set(
                f"F407状态：{state_name(status.state)} cmd=0x{status.command:02X}"
                f" ack={status.command_sequence}{suffix}"
            )
        elif event.kind == "command":
            enabled, forward, left, yaw, camera, buttons, speed = event.value
            self.command_var.set(
                f"指令：{'RB=ON' if enabled else 'RB=OFF'} "
                f"F {forward:+d}  L {left:+d}  Y {yaw:+d}  CAM {camera:+d}  "
                f"BTN 0x{buttons:02X}  SPD {speed}%"
            )
        elif event.kind == "stopped":
            self.status_var.set("手柄控制已停止，已发送安全停车帧")

    def _draw_controller(self) -> None:
        c = self.canvas
        c.delete("all")
        c.create_text(35, 24, anchor="w", text="手柄按键位置与功能", font=("Sans", 18, "bold"), fill="#14213d")
        self._draw_front(c, 35, 55)
        self._draw_back(c, 35, 480)
        self._draw_legend(c, 640, 55)

    @staticmethod
    def _label(canvas: Any, x: float, y: float, text: str, color: str = "#1f2937") -> None:
        canvas.create_text(x, y, text=text, fill=color, font=("Sans", 10), anchor="w")

    def _draw_front(self, c: Any, x: int, y: int) -> None:
        c.create_text(x, y - 15, anchor="w", text="正面按键", font=("Sans", 13, "bold"), fill="#334155")
        body = [
            x + 115,
            y + 35,
            x + 225,
            y + 12,
            x + 355,
            y + 12,
            x + 470,
            y + 35,
            x + 505,
            y + 245,
            x + 420,
            y + 315,
            x + 320,
            y + 270,
            x + 265,
            y + 270,
            x + 170,
            y + 315,
            x + 80,
            y + 245,
        ]
        c.create_polygon(body, fill="#e5e7eb", outline="#475569", width=2, smooth=True)
        c.create_oval(x + 145, y + 82, x + 205, y + 142, fill="#cbd5e1", outline="#334155", width=2)
        c.create_oval(x + 330, y + 128, x + 390, y + 188, fill="#cbd5e1", outline="#334155", width=2)
        c.create_text(x + 175, y + 112, text="左摇杆", font=("Sans", 9), fill="#334155")
        c.create_text(x + 360, y + 158, text="右摇杆", font=("Sans", 9), fill="#334155")
        c.create_rectangle(x + 205, y + 100, x + 245, y + 112, fill="#94a3b8", outline="#334155")
        c.create_rectangle(x + 219, y + 86, x + 231, y + 126, fill="#94a3b8", outline="#334155")
        c.create_text(x + 225, y + 145, text="转轴十字键", font=("Sans", 9), fill="#334155")
        for label, px, py, color in (
            ("Y", x + 400, y + 58, "#fef08a"),
            ("X", x + 375, y + 94, "#bfdbfe"),
            ("B", x + 425, y + 94, "#fecaca"),
            ("A", x + 400, y + 130, "#bbf7d0"),
        ):
            c.create_oval(px - 16, py - 16, px + 16, py + 16, fill=color, outline="#334155")
            c.create_text(px, py, text=label, font=("Sans", 10, "bold"), fill="#111827")
        c.create_text(x + 210, y + 42, text="SELECT", font=("Sans", 9, "bold"), fill="#0f766e")
        c.create_text(x + 325, y + 42, text="START", font=("Sans", 9, "bold"), fill="#0f766e")
        c.create_text(x + 270, y + 72, text="LOGO", font=("Sans", 9), fill="#64748b")
        c.create_text(x + 122, y + 220, text="Turbo", font=("Sans", 9), fill="#64748b")
        c.create_text(x + 430, y + 220, text="FN", font=("Sans", 9), fill="#64748b")

    def _draw_back(self, c: Any, x: int, y: int) -> None:
        c.create_text(x, y - 15, anchor="w", text="背面按键", font=("Sans", 13, "bold"), fill="#334155")
        body = [
            x + 120,
            y + 45,
            x + 200,
            y + 20,
            x + 400,
            y + 20,
            x + 485,
            y + 45,
            x + 510,
            y + 280,
            x + 410,
            y + 335,
            x + 170,
            y + 335,
            x + 70,
            y + 280,
        ]
        c.create_polygon(body, fill="#e5e7eb", outline="#475569", width=2, smooth=True)
        c.create_text(x + 160, y + 10, text="RB", font=("Sans", 10, "bold"), fill="#be123c")
        c.create_text(x + 360, y + 10, text="LB", font=("Sans", 10, "bold"), fill="#be123c")
        c.create_text(x + 85, y + 82, text="RT", font=("Sans", 9), fill="#475569")
        c.create_text(x + 440, y + 82, text="LT", font=("Sans", 9), fill="#475569")
        c.create_polygon(x + 130, y + 160, x + 160, y + 135, x + 190, y + 160, x + 175, y + 195, x + 145, y + 195, fill="#cbd5e1", outline="#334155")
        c.create_polygon(x + 365, y + 160, x + 395, y + 135, x + 425, y + 160, x + 410, y + 195, x + 380, y + 195, fill="#cbd5e1", outline="#334155")
        c.create_text(x + 155, y + 220, text="M1", font=("Sans", 10, "bold"), fill="#0f766e")
        c.create_text(x + 395, y + 220, text="M2", font=("Sans", 10, "bold"), fill="#0f766e")
        c.create_rectangle(x + 260, y + 150, x + 300, y + 172, fill="#94a3b8", outline="#334155")
        c.create_text(x + 280, y + 195, text="模式档位", font=("Sans", 9), fill="#64748b")

    def _draw_legend(self, c: Any, x: int, y: int) -> None:
        c.create_text(x, y, anchor="nw", text="控制说明", font=("Sans", 15, "bold"), fill="#14213d")
        lines = [
            ("右摇杆 ↑ / ↓", "前进 / 后退"),
            ("右摇杆 ← / →", "左移 / 右移"),
            ("左摇杆 ← / →", "原地逆时针 / 顺时针旋转"),
            ("左摇杆 ↑ / ↓", "摄像头舵机抬头 / 低头"),
            ("十字键 ← / →", "两侧夹爪闭合 / 张开"),
            ("十字键 ↑ / ↓", "大舵机抬起 / 放下"),
            ("RB（按住）", "运动与舵机总使能；松开立即停车"),
            ("LB（按住）", "精细速度，最高35%"),
            ("X", "进入 T265 建图界面"),
            ("Y", "退出 T265 建图界面"),
            ("A / B", "预留扩展按键"),
            ("START / SELECT / M1 / M2", "未分配，避免误触发"),
        ]
        row = y + 35
        for key, description in lines:
            c.create_text(x, row, anchor="nw", text=key, font=("Sans", 10, "bold"), fill="#0f766e")
            c.create_text(x + 145, row, anchor="nw", text=description, font=("Sans", 10), fill="#334155")
            row += 29
        c.create_text(
            x,
            row + 15,
            anchor="nw",
            text="建图采用PTY转发：真实UART由手柄程序独占，\nT265界面只接收里程计/状态，不会抢占手柄运动控制。",
            font=("Sans", 10),
            fill="#64748b",
        )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="F407 Gamepad teleoperation desktop bridge")
    parser.add_argument("--uart", default=DEFAULT_UART)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--device", help="explicit /dev/input/eventN")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--deadzone", type=float, default=DEFAULT_DEADZONE)
    parser.add_argument("--precision-percent", type=int, default=DEFAULT_PRECISION)
    default_map = str(ROOT / "t265_map" / "run_t265_map.sh") + " --enable-motion --uart {pty}"
    parser.add_argument("--map-command", default=default_map)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    options = parse_args(argv)
    try:
        window = GamepadWindow(options)
    except Exception as error:  # Tk/display and dependency errors need a clear terminal message.
        print(f"手柄控制启动失败：{error}", file=sys.stderr)
        return 2
    window.root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
