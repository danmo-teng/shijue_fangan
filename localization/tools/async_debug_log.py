"""Bounded diagnostic writer. Producers never serialize JSON or touch disk.
Overflow is explicit and makes this run incomplete; it cannot silently certify
an input or cause a vehicle fault latch. Close drains only at session shutdown.
"""
import json
import multiprocessing as mp
import os
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
        self.thread=threading.Thread(target=self._profile_or_run,name="debug-log-writer",daemon=True)
        self.thread.start()

    def _profile_or_run(self):
        prefix=os.environ.get('RDK_DEBUG_LOG_PROFILE')
        if not prefix:return self._run()
        # Explicit latency experiment only; normal runs do not profile records.
        import cProfile
        profile=cProfile.Profile()
        try:profile.runcall(self._run)
        finally:profile.dump_stats(f'{prefix}_{os.getpid()}_{threading.get_native_id()}.prof')

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
                if self.stop.is_set() and self.queue.empty():
                    # close() may arrive between the flush condition above and
                    # this check; never lose a final queued atomic summary.
                    with self.snapshot_lock:
                        if not self.snapshots:break
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


def _process_debug_log(root, capacity, requests, reply, bad, dropped, error):
    """Keep the original JSON/file writer entirely outside the motion GIL."""
    logger=None;statistics={}
    def status():
        if logger is None:return
        dropped.value=logger.dropped
        if not logger.healthy():bad.value=1
        if logger.error:error.value=logger.error.encode('utf-8')[:1023]
    try:
        logger=AsyncDebugLog(root,capacity)
        while True:
            try:batch=requests.recv()
            except EOFError:break
            if batch is None:break
            for name,value,atomic in batch:
                logger.submit(name,value,atomic=atomic);status()
    except Exception as exc:
        bad.value=1;error.value=str(exc).encode('utf-8')[:1023]
    finally:
        if logger is not None:
            try:logger.close()
            except Exception as exc:bad.value=1;error.value=str(exc).encode('utf-8')[:1023]
            status();statistics=logger.statistics()
        try:reply.send(dict(statistics=statistics,healthy=not bool(bad.value)))
        except (OSError,EOFError):pass
        reply.close();requests.close()


class ProcessDebugLog:
    """Bounded logging transport; only the child executes AsyncDebugLog IO.

    A successful submit is IPC acceptance, not a persistence acknowledgement.
    Failures affect log completeness and never wait in a producer/control call.
    """
    def __init__(self, root, capacity=4096):
        context=mp.get_context('spawn')
        self.queue=queue.Queue(maxsize=capacity)
        requests,self._requests=context.Pipe(duplex=False)
        self._reply,reply=context.Pipe(duplex=False)
        self._bad=context.RawValue('b',0);self._writer_dropped=context.RawValue('Q',0)
        self._child_error=context.RawArray('c',1024)
        self._dropped=0;self._error=None;self._final=None;self._closed=False
        self._fallback=None;self._start_error=None
        self.process=context.Process(target=_process_debug_log,args=(root,capacity,requests,
            reply,self._bad,self._writer_dropped,self._child_error),name='motion-log-writer')
        self.thread=threading.Thread(target=self._send,name='motion-log-sender',daemon=True)
        try:
            self.process.start();requests.close();self.thread.start()
        except Exception as exc:
            self._start_error=str(exc)
            self._requests.close();self._reply.close()
            if self.process.pid is not None:
                if self.process.is_alive():self.process.terminate()
                self.process.join(timeout=1)
            self._fallback=AsyncDebugLog(root,capacity)
            self._fallback.submit('logging_events.jsonl',dict(event='process_log_start_failed_thread_fallback',
                error=self._start_error,latency_validated=False))
        finally:reply.close();requests.close()

    def _send(self):
        """Batch only transport/pickling; JSON and disk stay in the child."""
        try:
            finished=False
            while not finished:
                record=self.queue.get();batch=[];taken=1
                if record is None:finished=True
                else:batch.append(record)
                while not finished and len(batch)<128:
                    try:record=self.queue.get_nowait();taken+=1
                    except queue.Empty:break
                    if record is None:finished=True
                    else:batch.append(record)
                try:
                    if batch:self._requests.send(batch)
                    if finished:self._requests.send(None)
                finally:
                    for _ in range(taken):self.queue.task_done()
        except Exception as exc:self._fail('log sender: '+str(exc))
        finally:self._requests.close()

    def _fail(self, message):
        self._error=message;self._bad.value=1

    @property
    def error(self):
        if self._fallback is not None:return self._fallback.error
        return self._error or self._child_error.value.decode('utf-8',errors='replace') or None

    @property
    def dropped(self):
        if self._fallback is not None:return self._fallback.dropped
        return self._dropped+self._writer_dropped.value

    def submit(self, filename, value, *, atomic=False):
        if self._fallback is not None:return self._fallback.submit(filename,value,atomic=atomic)
        if self._closed or self.error:return False
        try:self.queue.put_nowait((filename,dict(value),atomic));return True
        except queue.Full:self._dropped+=1;self._bad.value=1;return False
        except (OSError,ValueError) as exc:self._fail(str(exc));return False

    def healthy(self):
        if self._fallback is not None:return self._fallback.healthy()
        return not self._bad.value and not self.dropped

    def statistics(self):
        if self._fallback is not None:
            return dict(self._fallback.statistics(),execution='thread_fallback',process_start_error=self._start_error)
        statistics=dict(self._final or {})
        statistics.update(execution='spawn_process_original_async_writer',writer_pid=self.process.pid,
            transport_queue_capacity=self.queue.maxsize,transport_queue_pending=self.queue.qsize(),
            transport_batch_limit=128,sender_stopped=not self.thread.is_alive(),
            transport_dropped=self._dropped,writer_dropped=self._writer_dropped.value,
            dropped=self.dropped,error=self.error,final_receipt=self._final is not None)
        return statistics

    def close(self):
        if self._closed:return
        self._closed=True
        if self._fallback is not None:self._fallback.close();return
        deadline=time.monotonic()+5
        def remaining():return max(0.,deadline-time.monotonic()-.2)
        try:
            if self.thread.is_alive():self.queue.put(None,timeout=remaining())
            self.thread.join(timeout=remaining())
            if self.thread.is_alive():
                self._fail('log sender did not drain before shutdown deadline')
                if self.process.is_alive():self.process.terminate()
                self.process.join(timeout=max(0.,deadline-time.monotonic()))
                self._requests.close()
            if self._reply.poll(remaining()):
                receipt=self._reply.recv();self._final=receipt['statistics']
                if not receipt['healthy']:self._bad.value=1
            else:self._fail('log process did not acknowledge complete drain before shutdown deadline')
        except (OSError,EOFError,queue.Full,ValueError) as exc:self._fail('log process close: '+str(exc))
        self.process.join(timeout=remaining())
        if self.process.is_alive():
            self._fail('log process did not exit before shutdown deadline')
            self.process.terminate();self.process.join(timeout=max(0.,deadline-time.monotonic()))
        if self.process.exitcode!=0:self._fail('log process exited '+str(self.process.exitcode))
        self.thread.join(timeout=max(0.,deadline-time.monotonic()))
        self._requests.close();self._reply.close()
