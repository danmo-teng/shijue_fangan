"""Storage controls shared by the existing test maps; no sensor/control ownership."""
from __future__ import annotations

import csv
import json
import os
import re
import shutil
import stat
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

OWNER = "t265_f407_motion_map"
WHEEL_OWNER = "competition_wheel_input"
SESSION_NAME = re.compile(r"\d{8}_\d{6}_interactive(?:_\d+)?")
SESSION_PATTERNS = {OWNER: SESSION_NAME,
    "competition_wheel_input": re.compile(r"\d{8}_\d{6}_competition_wheel(?:_\d+)?")}
DYNAMIC_SESSION_FILES = frozenset({"dynamic_events.jsonl", "dynamic_scans.jsonl", "dynamic_localization.jsonl",
    "dynamic_status.json", "dynamic_summary.json", "pose_motion.jsonl", "motion_feedback.jsonl", "lidar_rx.jsonl", "lidar_tx.jsonl",
    "lidar_worker_events.jsonl", "lidar_worker_summary.json", "dynamic_initialization.jsonl",
    "lidar_reader_events.jsonl", "lidar_reader_summary.json"})
WHEEL_SESSION_FILES = frozenset({"metadata.json", "localization.conf", "session_complete.json",
    "wheel_feedback_requests.bin", "t265_raw_pose.csv", "localization_debug.csv",
    "localization_events.jsonl", "wheel_sdk_config.json", "wheel_sdk.jsonl", "wheel_sdk_status.json",
    "wheel_input.jsonl", "wheel_measurements.jsonl", "chassis_identity.json",
    "chassis_extended.jsonl", "logging_events.jsonl", "final_status.json", "raw_t265_result.json",
    "control_pose.jsonl", "vision_replay_manifest.json", "uart_rx_native.jsonl"}) | DYNAMIC_SESSION_FILES


def read_json(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def inspect_session(root, name, current, allowed_files, *, owner=OWNER):
    """Snapshot only immediate children. Unknown/user files are never traversed."""
    path = root / name
    row = dict(name=name, size=0, reason="", fingerprint=None, order=(0, 0))
    try:
        pattern = SESSION_PATTERNS[owner]
        if not pattern.fullmatch(name) or path.is_symlink() or not path.is_dir():
            row["reason"] = "非本模块目录/符号链接"
            return row
        directory = path.stat()
        children = sorted((p.name, p.lstat()) for p in path.iterdir())
        row["size"] = sum(s.st_size for _, s in children if stat.S_ISREG(s.st_mode))
        row["fingerprint"] = (directory.st_dev, directory.st_ino, tuple(
            (n, s.st_ino, s.st_size, s.st_mtime_ns, s.st_mode) for n, s in children))
        protection_path = root / "protected_sessions.json"
        protection = read_json(protection_path)
        if current is not None and path.resolve() == current.resolve():
            row["reason"] = "当前会话"
        elif (protection_path.exists() or protection_path.is_symlink()) and (protection.get("owner") != owner or
                not isinstance(protection.get("sessions"), list)):
            row["reason"] = "保护清单不可读，暂不删除"
        elif name in protection.get("sessions", []):
            row["reason"] = "明确保留"
        elif any(not stat.S_ISREG(s.st_mode) for _, s in children):
            row["reason"] = "含子目录/链接，保留"
        if row["reason"]:
            return row
        metadata = read_json(path / "metadata.json")
        modes = {"interactive_f407_motion_debug", "interactive_f405_normaldebug"} if owner == OWNER else {"competition_t265_wheel_input"}
        if metadata.get("mode") not in modes or (owner != OWNER and metadata.get("owner") != owner):
            row["reason"] = "归属无法确认"
            return row
        image_names = set()
        manifest = path / "fisheye_frames.csv"
        if metadata.get("fisheye_evidence_enabled") and manifest.is_file():
            with manifest.open() as stream:
                for i, frame in enumerate(csv.DictReader(stream)):
                    filename = frame.get("filename", "")
                    if (i >= 32 or frame.get("session_id") != metadata.get("sdk_session_id") or
                            not re.fullmatch(r"fisheye_b[12]_s[12]_f\d+\.pgm", filename)):
                        row["reason"] = "鱼眼清单无法确认"
                        return row
                    image_names.add(filename)
        replay_manifest = path / "vision_replay_manifest.json"
        if replay_manifest.exists():
            replay = read_json(replay_manifest)
            if (owner != "competition_wheel_input" or replay.get("schema_version") != 1 or
                    replay.get("sdk_session_id") != metadata.get("sdk_session_id") or
                    replay.get("complete") is not True or not isinstance(replay.get("clips"), list)):
                row["reason"] = "回放未完整结束/归属不明"
                return row
            for index, clip in enumerate(replay['clips'], 1):
                stem = f"safe_zone_replay_{index:04d}"
                if (clip.get('video') != stem + '.avi' or clip.get('frames') != stem + '.jsonl'
                        or clip.get('complete') is not True):
                    row["reason"] = "回放文件清单无法确认"
                    return row
                image_names.update((clip['video'], clip['frames']))
        if any(n not in allowed_files and n not in image_names for n, _ in children):
            row["reason"] = "含未知/用户文件，保留"
            return row
        marker_path = path / "session_complete.json"
        marker = read_json(marker_path)
        if marker_path.exists():
            if (marker.get("owner") != owner or marker.get("complete") is not True or
                    not isinstance(marker.get("session_index"), int)):
                row["reason"] = "未完整结束"
            else:
                row["order"] = (1, marker["session_index"])
        else:
            # Compatibility with the existing terminal-event retention policy.
            if metadata.get("sdk_session_id"):
                row["reason"] = "未完整结束（缺完成标记）"
                return row
            with (path / "events.jsonl").open("rb") as stream:
                stream.seek(max(0, os.fstat(stream.fileno()).st_size - 65536))
                terminal = json.loads(stream.read().splitlines()[-1])
            if (terminal.get("event") != "session_stopped" or
                    terminal.get("localizer_returncode", 0) != 0):
                row["reason"] = "未完整结束"
            else:
                row["order"] = (0, (path / "events.jsonl").stat().st_mtime_ns)
    except (OSError, ValueError, TypeError, IndexError, AttributeError, csv.Error) as error:
        row["reason"] = "未完整结束/无法读取：" + type(error).__name__
    return row


def list_sessions(root, current, allowed_files, *, owner=OWNER):
    if not root.exists():
        return []
    return [inspect_session(root, p.name, current, allowed_files, owner=owner)
            for p in sorted(root.iterdir(), reverse=True) if SESSION_PATTERNS[owner].fullmatch(p.name)]


def delete_sessions(root, selected, current, allowed_files, *, owner=OWNER):
    """Delete exactly the reviewed completed snapshots, never recursive paths."""
    removed, skipped = [], []
    for original in selected:
        row = inspect_session(root, original["name"], current, allowed_files, owner=owner)
        if row["reason"] or row["fingerprint"] != original["fingerprint"]:
            skipped.append((row["name"], row["reason"] or "选择后内容已变化"))
            continue
        try:
            path = root / row["name"]
            # Bind unlink to the inspected directory, not a substituted symlink.
            fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
            try:
                info = os.fstat(fd)
                if (info.st_dev, info.st_ino) != row["fingerprint"][:2]:
                    raise OSError("目录已变化")
                for name, inode, size, mtime, mode in row["fingerprint"][2]:
                    info = os.stat(name, dir_fd=fd, follow_symlinks=False)
                    if (info.st_ino, info.st_size, info.st_mtime_ns, info.st_mode) != (inode, size, mtime, mode):
                        raise OSError("文件已变化")
                for name, *_ in row["fingerprint"][2]:
                    os.unlink(name, dir_fd=fd)
            finally:
                os.close(fd)
            path.rmdir()
            removed.append(row["name"])
        except OSError as error:
            skipped.append((row["name"], str(error)))
    return removed, skipped


def disk_capacity(root):
    path = root
    while not path.exists() and path != path.parent:
        path = path.parent
    return shutil.disk_usage(path)


def size_label(size):
    return f"{size / 2**30:.2f} GiB" if size >= 2**30 else f"{size / 2**20:.1f} MiB"


class StoragePanel:
    """One background I/O task at a time; rendering never scans/deletes logs."""
    PAGE_SIZE = 7

    def __init__(self, root, allowed_files, *, owner=OWNER):
        self.root, self.allowed_files = root, allowed_files
        self.owner=owner
        self.open = False
        self.rows, self.selected, self.boxes = [], set(), {}
        self.page, self.confirm = 0, False
        self.capacity, self.error, self.message = None, "", ""
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="log-storage")
        self.future, self.job, self.next_refresh = None, "", 0.
        self.scan_requested = False
        self.pending_delete = None
        self.render_key = None

    def close(self):
        self.executor.shutdown(wait=True)

    def poll(self, current):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                result = self.future.result()
                if self.job == "scan":
                    self.rows = result
                    self.selected.clear()
                    self.page = min(self.page, max(0, (len(result)-1)//self.PAGE_SIZE))
                elif self.job == "delete":
                    removed, skipped = result
                    self.message = f"已删除 {len(removed)} 次；保留/失败 {len(skipped)} 次"
                    if skipped:
                        self.message += "；" + skipped[0][1]
                    self.selected.clear()
                    self.scan_requested = True
                else:
                    self.capacity, self.error = result, ""
            except Exception as error:
                # A storage failure affects this panel, never motion/pose.
                self.error = str(error)
                if self.job == "capacity":
                    self.capacity = None
            self.future = None
        if self.pending_delete is not None:
            self.job = "delete"
            self.future = self.executor.submit(delete_sessions, self.root, self.pending_delete, current, self.allowed_files, owner=self.owner)
            self.pending_delete = None
        elif self.scan_requested:
            self.scan_requested = False
            self.job = "scan"
            self.future = self.executor.submit(list_sessions, self.root, current, self.allowed_files, owner=self.owner)
        elif time.monotonic() >= self.next_refresh:
            self.next_refresh = time.monotonic() + 1.
            self.job = "capacity"
            self.future = self.executor.submit(disk_capacity, self.root)

    def render(self, canvas, app, painter):
        import numpy as np
        self.poll(app.run_dir)
        key = (self.open, self.capacity, self.error, self.message, self.page, self.confirm,
               tuple(sorted(self.selected)), id(self.rows),
               self.job if self.future and self.job != "capacity" else None,
               self.pending_delete is not None, app.render_scale)
        if key != self.render_key:
            layer = np.zeros_like(canvas)
            text = painter(font_scale=1. / app.render_scale)
            self.draw(layer, text)
            self.layer = text.paint(layer)
            self.render_key = key
        canvas[984:1024,30:880] = self.layer[984:1024,30:880]
        if self.open:
            canvas[60:910,30:880] = self.layer[60:910,30:880]

    def draw(self, canvas, text):
        import cv2
        self.boxes = {}
        def button(name, label, box, selected=False):
            self.boxes[name] = box
            x0, y0, x1, y1 = box
            cv2.rectangle(canvas, (x0,y0), (x1,y1), (60,70,80) if selected else (45,45,48), -1)
            cv2.rectangle(canvas, (x0,y0), (x1,y1), (0,220,255) if selected else (110,110,115), 1)
            text.add(label, ((x0+x1)//2,(y0+y1)//2), 17, anchor="mm", max_width=x1-x0-12)
        cv2.rectangle(canvas, (30,984), (880,1023), (30,30,35), -1)
        label, color = "日志盘容量：读取中…", (220,220,220)
        if self.capacity:
            total, used, free = self.capacity
            label = f"日志盘剩余 {size_label(free)} / 总计 {size_label(total)}"
            if free < max(2**30, total*.1):
                color = (70,170,255)
                label += "  容量告警，请清理已分析日志"
            if free < 2**30:
                color = (60,70,255)
                label += "（低于1GiB）"
        elif self.error:
            label, color = "日志盘容量读取失败（不影响运动）", (70,170,255)
        text.add(label, (40,1003), 18, color, anchor="lm", max_width=660)
        button("open", "日志管理", (720,987,875,1020))
        if not self.open:
            return
        cv2.rectangle(canvas, (30,60), (880,910), (28,29,34), -1)
        text.add("比赛定位日志" if self.owner=="competition_wheel_input" else "测试定位日志", (50,78), 24, bold=True)
        button("close", "返回地图", (700,76,855,119))
        text.add("只删除本模块完整旧会话；当前 / 未完成 / 保护 / 用户文件均保留", (50,134), 15, max_width=800)
        text.add("列表按需刷新，不扫描模型/标定；STOP和S/空格仍有效", (50,166), 15, max_width=800)
        visible = self.rows[self.page*self.PAGE_SIZE:(self.page+1)*self.PAGE_SIZE]
        for index, row in enumerate(visible):
            y = 205 + index*71
            mark = "锁定" if row["reason"] else ("已选" if row["name"] in self.selected else "选择")
            button("row:"+row["name"], mark, (50,y,133,y+52), row["name"] in self.selected)
            text.add(row["name"], (150,y), 17, max_width=690)
            text.add(size_label(row["size"])+"  "+(row["reason"] or "完整旧会话，可删除"), (150,y+29), 15, max_width=690)
        if not visible:
            text.add("暂无已识别会话" if not self.future else "正在读取…", (55,225), 20)
        count = len(self.selected)
        size = sum(r["size"] for r in self.rows if r["name"] in self.selected)
        text.add(f"第{self.page+1}/{max(1,(len(self.rows)+self.PAGE_SIZE-1)//self.PAGE_SIZE)}页；已选{count}次，{size_label(size)}", (50,716), 18)
        button("prev", "上一页", (50,754,175,797))
        button("next", "下一页", (185,754,310,797))
        button("refresh", "刷新列表", (320,754,480,797))
        button("delete", f"确认永久删除{count}次" if self.confirm else "删除所选…", (550,754,855,797), self.confirm)
        status = "请核对勾选名称和容量，再次点击确认；返回/ESC取消" if self.confirm else self.message
        if self.pending_delete is not None:
            status = "正在提交删除所选旧会话…"
        elif self.future and self.job in {"scan","delete"}:
            status = "正在读取列表…" if self.job == "scan" else "正在删除所选旧会话…"
        text.add(status or "勾选需要清理的旧会话，再点击删除所选", (50,820), 16, max_width=800)
        if self.error:
            text.add("存储操作失败："+self.error, (50,858), 14, (70,170,255), max_width=800)

    def click(self, x, y, app):
        name = next((n for n,b in self.boxes.items() if b[0]<=x<=b[2] and b[1]<=y<=b[3]), None)
        if name is None:
            stop = app.hitboxes.get("stop")
            return self.open and not (stop and stop[0]<=x<=stop[2] and stop[1]<=y<=stop[3])
        if name == "close":
            self.open, self.confirm = False, False
        elif name == "open":
            self.open = True
            self.scan_requested = True
            self.confirm = False
        elif self.pending_delete is None and not (self.future and self.job in {"scan","delete"}):
            if name.startswith("row:"):
                row = next(r for r in self.rows if r["name"] == name[4:])
                if not row["reason"]:
                    self.selected.symmetric_difference_update({row["name"]})
                self.confirm = False
            elif name == "refresh":
                self.scan_requested, self.confirm = True, False
            elif name in {"prev", "next"}:
                self.page = max(0, min(max(0,(len(self.rows)-1)//self.PAGE_SIZE), self.page + (1 if name=="next" else -1)))
            elif name == "delete" and self.selected:
                if not self.confirm:
                    self.confirm = True
                elif not self.scan_requested:
                    self.pending_delete = [r for r in self.rows if r["name"] in self.selected]
                    self.confirm = False
        return True

    def key(self, key, app=None):
        if not self.open:
            return False
        if key & 255 in (27, ord("q"), ord("Q")):
            self.open, self.confirm = False, False
            return True
        if key & 255 in (32, ord("s"), ord("S")) and app is not None:
            stop = app.hitboxes.get("stop")
            if stop:
                app.select_at((stop[0]+stop[2])//2, (stop[1]+stop[3])//2)
            return True
        return key & 255 not in (32, ord("s"), ord("S"))
