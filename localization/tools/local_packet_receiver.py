"""Fixed-size local packet reads without yielding the GIL for queued data.

The caller owns the socket and its timeout/reconnection policy. This helper
does not open devices/sockets, alter their flags, read clocks or inspect data.
Only a genuinely empty queue waits via select, which releases the GIL.
"""
import ctypes
import errno
import os
import select
import socket


_recv_nowait = ctypes.PyDLL(None, use_errno=True).recv
_recv_nowait.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
_recv_nowait.restype = ctypes.c_ssize_t


class LocalPacketReceiver:
    """One reader per local SEQPACKET socket, with the original recv limit.

    recv() returns one real packet (b'' for EOF), raises socket.timeout when a
    finite wait expires, and retains BlockingIOError for timeout=0. timeout=None
    waits indefinitely. Truncation/discard of excess packet bytes matches recv.
    The returned bytes own their storage independently of this fixed buffer.
    """
    def __init__(self, connection, capacity):
        if not isinstance(capacity, int) or capacity <= 0:
            raise ValueError('positive local packet receive capacity required')
        self.connection = connection
        self.capacity = capacity
        self._buffer = ctypes.create_string_buffer(capacity)

    def _nowait(self):
        while True:
            size = _recv_nowait(self.connection.fileno(), self._buffer,
                                self.capacity, socket.MSG_DONTWAIT)
            if size >= 0:
                return self._buffer[:size]
            error = ctypes.get_errno()
            if error == errno.EINTR:
                continue
            if error in (errno.EAGAIN, errno.EWOULDBLOCK):
                return None
            raise OSError(error, os.strerror(error))

    def recv(self):
        data = self._nowait()
        if data is not None:
            return data
        timeout = self.connection.gettimeout()
        if timeout == 0:
            raise BlockingIOError(errno.EAGAIN, os.strerror(errno.EAGAIN))
        while True:
            if not select.select([self.connection], [], [], timeout)[0]:
                raise socket.timeout('timed out')
            data = self._nowait()
            if data is not None:
                return data
            # A sole-reader local socket cannot normally lose readiness. A
            # spurious wake must not manufacture bytes or extend a finite wait.
            if timeout is not None:
                raise socket.timeout('timed out')
