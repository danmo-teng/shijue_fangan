"""Bounded cross-process motion input and latest control snapshots.

This carries small control snapshots, not scans/history. No queue feeder,
display timer, sample clock, admission policy or hardware ownership lives here.
"""
import pickle
import queue
import select
import socket
import ctypes
import errno
import os


# A NONBLOCKING socket read must not relinquish the GIL to a numerical/log
# thread just to find an empty queue. The kernel call is explicitly NOWAIT,
# with the same bounded packet size as socket.recv. No device/file I/O here.
_recv_nowait=ctypes.PyDLL(None,use_errno=True).recv
_recv_nowait.argtypes=[ctypes.c_int,ctypes.c_void_p,ctypes.c_size_t,ctypes.c_int]
_recv_nowait.restype=ctypes.c_ssize_t


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
        self._receive_buffer=None

    def __getstate__(self):
        state=dict(self.__dict__);state['_receive_buffer']=None
        return state  # Recreate the local scratch buffer in a spawned reader.

    def put_nowait(self,value):
        payload=pickle.dumps(value,protocol=pickle.HIGHEST_PROTOCOL)
        if len(payload)>65536:raise queue.Full
        try:self._writer.send(payload)
        except BlockingIOError:raise queue.Full from None

    def get(self,timeout=None):
        if timeout!=0 and not select.select([self._reader],[],[],timeout)[0]:raise queue.Empty
        if self._receive_buffer is None:self._receive_buffer=ctypes.create_string_buffer(65536)
        while True:
            n=_recv_nowait(self._reader.fileno(),self._receive_buffer,65536,socket.MSG_DONTWAIT)
            if n>=0:return pickle.loads(self._receive_buffer[:n])
            error=ctypes.get_errno()
            if error==errno.EINTR:continue
            if error in (errno.EAGAIN,errno.EWOULDBLOCK):raise queue.Empty
            raise OSError(error,os.strerror(error))

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
