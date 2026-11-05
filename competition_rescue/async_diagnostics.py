"""One bounded writer for competition diagnostics, separate from camera/control.

History records retain producer timestamps. Current JSON snapshots coalesce while
disk is busy. Neither full queues nor write errors become a motion interlock.
"""
from __future__ import annotations

import json
import atexit
import queue
import threading
import time
from pathlib import Path


class AsyncDiagnosticWriter:
    def __init__(self, status_path: Path, capacity: int = 1024):
        self.status_path = Path(status_path)
        self.queue = queue.Queue(maxsize=capacity)
        self.lock = threading.Lock()
        self.snapshots = {}
        self.stop = threading.Event()
        self.dropped = self.written = self.coalesced = self.snapshot_writes = 0
        self.error = None
        self.close_timed_out = False
        self.thread = threading.Thread(target=self._run, name="competition-diagnostics", daemon=True)
        atexit.register(self.close)
        self.thread.start()

    def _enqueue(self, kind, path, text):
        with self.lock:
            if self.stop.is_set() or self.error is not None:
                self.dropped += 1
                return False
            try:
                self.queue.put_nowait((kind, Path(path), text))
                return True
            except queue.Full:
                self.dropped += 1
                return False

    def reset(self, path):
        return self._enqueue("reset", path, "")

    def append(self, path, record):
        # Freeze mutable nested values before returning to the producer. Only
        # serialization happens here; open/write/flush/rename stay in _run.
        text = json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
        return self._enqueue("append", path, text)

    def snapshot(self, path, document):
        text = json.dumps(document, ensure_ascii=False, indent=2) + "\n"
        with self.lock:
            if self.stop.is_set() or self.error is not None:
                self.dropped += 1
                return False
            key = Path(path)
            if key in self.snapshots:
                self.coalesced += 1
            elif len(self.snapshots) >= 8:
                self.dropped += 1
                return False
            self.snapshots[key] = text
        return True

    def statistics(self):
        with self.lock:
            return dict(queue_capacity=self.queue.maxsize,
                        pending_records=self.queue.unfinished_tasks,
                        pending_snapshots=len(self.snapshots),
                        records_written=self.written, snapshots_written=self.snapshot_writes,
                        snapshots_coalesced=self.coalesced, dropped=self.dropped,
                        error=self.error, close_timed_out=self.close_timed_out,
                        incomplete=bool(self.dropped or self.error or self.close_timed_out))

    @staticmethod
    def _atomic(path, text):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(text, encoding="utf-8")
        temporary.replace(path)

    def _run(self):
        streams = {}
        last_flush = time.monotonic()
        try:
            while True:
                try:
                    kind, path, text = self.queue.get(timeout=.05)
                except queue.Empty:
                    pass
                else:
                    if path not in streams or kind == "reset":
                        previous = streams.pop(path, None)
                        if previous is not None:
                            previous.close()
                        path.parent.mkdir(parents=True, exist_ok=True)
                        streams[path] = path.open("w" if kind == "reset" else "a",
                                                  encoding="utf-8", buffering=65536)
                    if kind == "append":
                        streams[path].write(text)
                        self.written += 1
                    self.queue.task_done()
                if self.stop.is_set() or time.monotonic() - last_flush >= .1:
                    with self.lock:
                        snapshots, self.snapshots = self.snapshots, {}
                    for path, text in snapshots.items():
                        self._atomic(path, text)
                        self.snapshot_writes += 1
                    for stream in streams.values():
                        stream.flush()
                    last_flush = time.monotonic()
                if self.stop.is_set() and self.queue.empty():
                    with self.lock:
                        if not self.snapshots:
                            break
        except (OSError, ValueError, TypeError) as error:
            self.error = str(error)
        finally:
            for stream in streams.values():
                try:
                    stream.close()
                except OSError as error:
                    self.error = str(error)
            try:
                self._atomic(self.status_path, json.dumps(self.statistics(), ensure_ascii=False) + "\n")
            except OSError as error:
                self.error = str(error)

    def close(self, timeout=3.0):
        atexit.unregister(self.close)
        self.stop.set()
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.close_timed_out = True
        return self.statistics()
