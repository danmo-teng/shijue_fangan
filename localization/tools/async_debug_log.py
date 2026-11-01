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
        self.thread=threading.Thread(target=self._run,name="debug-log-writer",daemon=True)
        self.thread.start()

    def submit(self, filename, value, *, atomic=False):
        if self.error:return False
        try:self.queue.put_nowait((filename,dict(value),atomic));return True
        except queue.Full:
            now=time.monotonic_ns();self.dropped+=1
            if self.first_drop_ns is None:self.first_drop_ns=now
            self.last_drop_ns=now
            return False

    def _run(self):
        streams={}
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:name,value,atomic=self.queue.get(timeout=.02)
                except queue.Empty:continue
                text=json.dumps(value,ensure_ascii=False,allow_nan=False)+"\n"
                path=self.root/name
                if atomic:
                    tmp=path.with_suffix(path.suffix+".tmp");tmp.write_text(text,encoding="utf-8");tmp.replace(path)
                else:
                    if name not in streams:streams[name]=path.open("a",encoding="utf-8",buffering=1)
                    streams[name].write(text)
                self.queue.task_done()
        except (OSError,ValueError) as error:self.error=str(error)
        finally:
            for stream in streams.values():stream.close()
            if self.dropped or self.error:
                try:
                    (self.root/"logging_events.jsonl").open("a",encoding="utf-8").write(json.dumps(dict(
                        event="logging_incomplete",dropped=self.dropped,first_drop_ns=self.first_drop_ns,
                        last_drop_ns=self.last_drop_ns,error=self.error))+"\n")
                except OSError:pass

    def healthy(self):
        return self.error is None and self.dropped==0

    def close(self):
        self.stop.set();self.thread.join(timeout=5)
        if self.thread.is_alive():self.error="log writer did not drain before shutdown deadline"
