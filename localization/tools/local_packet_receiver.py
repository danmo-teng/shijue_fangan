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


class _PollFd(ctypes.Structure):
    _fields_ = [('fd', ctypes.c_int), ('events', ctypes.c_short),
                ('revents', ctypes.c_short)]


_poll_nowait = ctypes.PyDLL(None, use_errno=True).poll
_poll_nowait.argtypes = [ctypes.POINTER(_PollFd), ctypes.c_ulong, ctypes.c_int]
_poll_nowait.restype = ctypes.c_int


def readable_nowait(fd):
    """Probe one local descriptor without waiting or relinquishing the GIL."""
    probe = _PollFd(fd, select.POLLIN, 0)
    while True:
        result = _poll_nowait(ctypes.byref(probe), 1, 0)
        if result >= 0:
            if probe.revents & select.POLLNVAL:
                raise OSError(errno.EBADF, os.strerror(errno.EBADF))
            # HUP/ERR must reach the caller's original read/error handling.
            return bool(probe.revents & (select.POLLIN | select.POLLHUP | select.POLLERR))
        error = ctypes.get_errno()
        if error == errno.EINTR:
            continue
        raise OSError(error, os.strerror(error))


class _SockaddrUn(ctypes.Structure):
    _fields_ = [('family', ctypes.c_ushort), ('path', ctypes.c_ubyte * 108)]


_sendto_nowait = ctypes.PyDLL(None, use_errno=True).sendto
_sendto_nowait.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t,
                          ctypes.c_int, ctypes.POINTER(_SockaddrUn), ctypes.c_uint]
_sendto_nowait.restype = ctypes.c_ssize_t


class LocalDatagramSender:
    """One cached Linux abstract destination on a caller-owned local socket.

    Every call sends the original bytes once without waiting or releasing the
    GIL. The caller retains its drop/error policy and socket ownership.
    """
    def __init__(self, connection, address):
        if connection.family != socket.AF_UNIX or connection.type != socket.SOCK_DGRAM:
            raise ValueError('local AF_UNIX datagram socket required')
        if isinstance(address, str):
            address = os.fsencode(address)
        if not isinstance(address, bytes):
            raise TypeError('abstract local address must be str or bytes')
        if not 1 < len(address) <= 108 or address[0] != 0:
            raise ValueError('nonempty Linux abstract local address of at most 108 bytes required')
        self.connection = connection
        self._address = _SockaddrUn(socket.AF_UNIX)
        self._address.path[:len(address)] = address
        self._address_pointer = ctypes.pointer(self._address)
        self._address_size = _SockaddrUn.path.offset + len(address)

    def send(self, data):
        if not isinstance(data, bytes):
            view = memoryview(data)
            if not view.c_contiguous:
                raise BufferError('local datagram requires a C-contiguous buffer')
            data = bytes(view)
        if len(data) > 65536:
            raise ValueError('local datagram exceeds 65536 bytes')
        while True:
            sent = _sendto_nowait(self.connection.fileno(), data, len(data),
                socket.MSG_DONTWAIT | socket.MSG_NOSIGNAL,
                self._address_pointer, self._address_size)
            if sent >= 0:
                if sent != len(data):
                    raise OSError(errno.EIO, os.strerror(errno.EIO))
                return sent
            error = ctypes.get_errno()
            if error == errno.EINTR:
                continue
            raise OSError(error, os.strerror(error))


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
