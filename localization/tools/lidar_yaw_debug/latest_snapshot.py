"""Bounded cross-process motion input and latest control snapshots.

This carries small control snapshots, not scans/history. No queue feeder,
display timer, sample clock, admission policy or hardware ownership lives here.
"""
import pickle
import queue
import select
import socket


class MotionInbox:
    """Atomic local datagrams, without multiprocessing.Queue's feeder hop.

    Multiple input threads send whole messages; only the motion owner receives.
    A full socket reports queue.Full, never waits or replays discarded inputs.
    """
    def __init__(self):
        self._reader,self._writer=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
        self._reader.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,262144)
        self._writer.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,262144)
        self._reader.setblocking(False);self._writer.setblocking(False)

    def put_nowait(self,value):
        payload=pickle.dumps(value,protocol=pickle.HIGHEST_PROTOCOL)
        if len(payload)>65536:raise queue.Full
        try:self._writer.send(payload)
        except BlockingIOError:raise queue.Full from None

    def get(self,timeout=None):
        if timeout!=0 and not select.select([self._reader],[],[],timeout)[0]:raise queue.Empty
        try:payload=self._reader.recv(65536)
        except BlockingIOError:raise queue.Empty from None
        return pickle.loads(payload)

    def get_nowait(self):
        return self.get(timeout=0)

    def cancel_join_thread(self):
        pass  # There is no feeder thread to drain/join.

    def close(self):
        self._reader.close();self._writer.close()


class LatestSnapshot:
    def __init__(self, context, capacity=65536):
        self._data = context.RawArray('B', capacity)
        self._size = context.RawValue('I', 0)
        self._lock = context.Lock()
        self._ready = context.Event()

    def put_latest(self, value):
        payload = pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)
        if len(payload) > len(self._data):
            raise ValueError('control snapshot exceeds bounded mailbox capacity')
        with self._lock:
            coalesced = self._size.value != 0
            memoryview(self._data).cast('B')[:len(payload)] = payload
            self._size.value = len(payload)
            self._ready.set()
        return int(coalesced)

    def get(self, timeout=None):
        if not self._ready.wait(timeout):
            return None
        with self._lock:
            size = self._size.value
            payload = bytes(memoryview(self._data).cast('B')[:size])
            self._size.value = 0
            self._ready.clear()
        return pickle.loads(payload) if size else None

    def wake(self):
        # Shutdown/local start request only; never manufacture a sensor sample.
        self._ready.set()
