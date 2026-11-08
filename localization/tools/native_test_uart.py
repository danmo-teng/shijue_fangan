"""Serial-compatible local transport; only the existing C++ process opens UART.

RX packets retain the UART owner's monotonic receive timestamp. TX returns the
actual serial.write result through local IPC, with no new F405 ACK/handshake.
No motion generation, retransmission, age gate or USB/SDK ownership here.
"""
import socket
import struct
import threading
import time


class NativeTestUart:
    def __init__(self, address, *, timeout=.05, write_timeout=.2, process=None):
        self.timeout, self.write_timeout = timeout, write_timeout
        self.rx = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.tx = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        self.rx.settimeout(timeout)
        self.tx.settimeout(write_timeout)
        self.write_lock = threading.Lock()
        self.receive_monotonic_ns = None
        self.receive_record_scope='native_CRC_valid_complete_frame'
        self.dropped_rx_datagrams = 0
        self.last_write_before_ns = self.last_write_after_ns = None
        # Sole owner startup, no MCU transaction or periodic probe. Bound only
        # local process creation; an SDK boot failure is reported as such.
        deadline=time.monotonic()+20
        try:
            for connection,name in ((self.rx,address+'-rx'),(self.tx,address)):
                while True:
                    try:
                        connection.connect('\0'+name.removeprefix('@'))
                        break
                    except (ConnectionRefusedError, FileNotFoundError):
                        if process is not None and process.poll() is not None:
                            raise OSError('C++ SDK/UART owner exited before the test transport opened')
                        if time.monotonic()>=deadline:raise OSError('C++ SDK/UART test transport did not open')
                        time.sleep(.01)
        except BaseException:
            self.close()
            raise

    @property
    def in_waiting(self):
        # Receive a complete native datagram, never turn a UART chunk into a
        # one-byte Python read just because a receive queue was initially empty.
        return 4096

    def read(self, _size):
        try:data=self.rx.recv(4112)
        except socket.timeout:return b''
        if len(data)<16:raise OSError('short native UART receive container')
        self.receive_monotonic_ns,self.dropped_rx_datagrams=struct.unpack('>QQ',data[:16])
        return data[16:]

    def write(self, data):
        with self.write_lock:
            self.tx.sendall(data)
            try:result=self.tx.recv(17)
            except OSError:
                # Actual transmission may have happened; do not reuse this
                # stream and mistake a delayed result for a later request.
                self.tx.close()
                raise
            if len(result)!=17:raise OSError('native UART write result missing; no replay')
            success,self.last_write_before_ns,self.last_write_after_ns=struct.unpack('>BQQ',result)
            if not success:raise OSError('C++ UART write failed or packet is outside the existing test protocol; no replay')
            return len(data)

    def close(self):
        self.rx.close()
        self.tx.close()


def serial_factory(address, process):
    def create(_device, _baud, *, timeout, write_timeout, exclusive):
        return NativeTestUart(address, timeout=timeout, write_timeout=write_timeout, process=process)
    return create
