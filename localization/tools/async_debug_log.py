"""Bounded diagnostic writer. Producers never serialize JSON or touch disk.
Overflow is explicit and makes this run incomplete; it cannot silently certify
an input or cause a vehicle fault latch. Close drains only at session shutdown.
"""
import json
import queue
import threading
import time


class AsyncDebugLog:
    def __init__(self, root, capacity=4096):
        self.root=root
        self.queue=queue.Queue(maxsize=capacity)
        self.stop=threading.Event()
        self.error=None
        self.dropped=0
        self.first_drop_ns=None
        self.last_drop_ns=None
        self.snapshots={}
        self.snapshot_lock=threading.Lock()
        self.queue_peak=0
        self.written=0
        self.snapshot_writes=0
        self.snapshots_coalesced=0
        self.thread=threading.Thread(target=self._run,name="debug-log-writer",daemon=True)
        self.thread.start()

    def submit(self, filename, value, *, atomic=False):
        if self.error:return False
        if atomic:
            # These are latest-value UI snapshots, never raw/history records.
            # Replacing a pending snapshot does not lose a telemetry sample.
            with self.snapshot_lock:
                if filename in self.snapshots:self.snapshots_coalesced+=1
                elif len(self.snapshots)>=16:return self._drop()
                self.snapshots[filename]=dict(value)
            return True
        try:
            self.queue.put_nowait((filename,dict(value),False))
            self.queue_peak=max(self.queue_peak,self.queue.qsize())
            return True
        except queue.Full:
            return self._drop()

    def _drop(self):
        now=time.monotonic_ns();self.dropped+=1
        if self.first_drop_ns is None:self.first_drop_ns=now
        self.last_drop_ns=now
        return False

    def statistics(self):
        return dict(queue_capacity=self.queue.maxsize,queue_pending=self.queue.qsize(),queue_peak=self.queue_peak,
                    history_records_written=self.written,snapshot_writes=self.snapshot_writes,
                    snapshots_coalesced=self.snapshots_coalesced,dropped=self.dropped,error=self.error,
                    flush_interval_ms=100)

    def _run(self):
        streams={}
        last_flush=time.monotonic()
        try:
            while True:
                self.stop.wait(.02)
                batch={};count=0
                for _ in range(512):
                    try:name,value,_=self.queue.get_nowait()
                    except queue.Empty:break
                    batch.setdefault(name,[]).append(json.dumps(value,ensure_ascii=False,allow_nan=False)+"\n")
                    count+=1
                for name,lines in batch.items():
                    if name not in streams:streams[name]=(self.root/name).open("a",encoding="utf-8",buffering=65536)
                    streams[name].write(''.join(lines))
                for _ in range(count):self.queue.task_done()
                self.written+=count
                if self.stop.is_set() or time.monotonic()-last_flush>=.1:
                    with self.snapshot_lock:
                        snapshots=self.snapshots;self.snapshots={}
                    for name,value in snapshots.items():
                        path=self.root/name;tmp=path.with_suffix(path.suffix+".tmp")
                        tmp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False)+"\n",encoding="utf-8")
                        tmp.replace(path);self.snapshot_writes+=1
                    for stream in streams.values():stream.flush()
                    last_flush=time.monotonic()
                if self.stop.is_set() and self.queue.empty():break
        except (OSError,ValueError,TypeError) as error:self.error=str(error)
        finally:
            for stream in streams.values():stream.close()
            if self.dropped or self.error:
                try:
                    with (self.root/"logging_events.jsonl").open("a",encoding="utf-8") as stream:
                        stream.write(json.dumps(dict(event="logging_incomplete",dropped=self.dropped,
                            first_drop_ns=self.first_drop_ns,last_drop_ns=self.last_drop_ns,error=self.error))+"\n")
                except OSError:pass

    def healthy(self):
        return self.error is None and self.dropped==0

    def close(self):
        self.stop.set();self.thread.join(timeout=5)
        if self.thread.is_alive():self.error="log writer did not drain before shutdown deadline"
