"""Optional bytes/timing/sector implementation in the existing native library.

The Python reader remains the configuration validator and explicit fallback.
Only the unqualified arrival-clock path is accelerated; calibrated clock maps
keep their complete original implementation. No per-point JSON or device I/O.
"""
import ctypes as C
import os
from pathlib import Path

from .lidar_reader import LidarStream
from .lidar_timing import LidarTimingError
from .types import RawScan, TimedPoint, require_time_ns


_DOUBLE_FIELDS = ('bearing_sign', 'bearing_zero_rad', 'direction', 'entry', 'width',
    'nominal_tick_ns', 'max_raw_angle_gap_deg', 'max_interpacket_step_factor',
    'angle_gap_quantization_margin_deg', 'max_angle_step_deg')
_INTEGER_FIELDS = ('timestamp_modulus', 'max_counter_gap_ticks', 'max_packet_tick_delta',
    'max_receive_gap_ns', 'max_points_per_packet', 'max_rx_chunk_bytes',
    'max_buffer_bytes', 'max_batch_span_ns', 'max_points', 'max_span_ns',
    'max_point_gap_ns', 'max_stall_ns')
_COUNTERS = ('rx_batches', 'rx_bytes', 'scan_packets', 'status_packets', 'ack_packets',
    'crc_errors', 'timing_rejections', 'sector_rejections', 'scans_emitted')


class _Config(C.Structure):
    _fields_ = [(name, C.c_double) for name in _DOUBLE_FIELDS] + [
        (name, C.c_uint64) for name in _INTEGER_FIELDS]


class _Point(C.Structure):
    _fields_ = [('packet_offset', C.c_uint64), ('time_ns', C.c_int64),
        ('x', C.c_double), ('y', C.c_double), ('raw_angle_deg', C.c_double),
        ('intensity', C.c_double), ('point_index', C.c_uint32), ('valid_return', C.c_uint8)]


class _Status(C.Structure):
    _fields_ = [('counters', C.c_uint64 * 9), ('needs_new_epoch', C.c_uint8),
        ('last_error', C.c_char * 256)]


_Scan = C.CFUNCTYPE(C.c_int, C.c_void_p, C.POINTER(_Point), C.c_size_t)
_library = None
_load_error = None


class NativeReaderError(RuntimeError):
    """Native backend failure; receiver can continue using a new Python epoch."""


def _copy_native_scan(raw_points, count, epoch, version, payload=None):
    points = []
    previous_offset = None
    for index in range(count):
        point = raw_points[index]
        if point.packet_offset != previous_offset:
            previous_offset = point.packet_offset
            packet_id = f'byte:{previous_offset}'
            prefix = f'{epoch}:{packet_id}:'
        # C++ has already checked the scalars, unique byte/index identities and
        # sector time order/bounds. All values below become owned Python data.
        copied = object.__new__(TimedPoint)
        object.__setattr__(copied, '__dict__', dict(
            raw_point_id=prefix + str(point.point_index), time_ns=point.time_ns,
            point_lidar_m=(point.x, point.y), packet_id=packet_id,
            point_index=point.point_index, intensity=point.intensity,
            time_qualified=False, raw_angle_deg=point.raw_angle_deg,
            valid_return=bool(point.valid_return)))
        points.append(copied)
    uid = f'{epoch}:sector:{points[0].raw_point_id}:{points[-1].raw_point_id}'
    scan = object.__new__(RawScan)
    object.__setattr__(scan, '__dict__', dict(scan_id=uid,
        reference_ns=points[-1].time_ns, points=tuple(points),
        sensor_epoch=epoch, time_map_version=version, timing_qualified=False,
        _native_transport=(C.string_at(raw_points, count * C.sizeof(_Point))
                           if payload is None else payload)))
    return scan


def restore_native_scan(epoch, version, payload):
    # Same-host transient IPC only, not a persisted or cross-machine format.
    size = C.sizeof(_Point)
    if not payload or len(payload) % size:
        raise ValueError('invalid native scan transport size')
    count = len(payload) // size
    raw_points = (_Point * count).from_buffer_copy(payload)
    return _copy_native_scan(raw_points, count, epoch, version, payload)


def _load():
    global _library, _load_error
    if _library is not None:
        return _library
    if _load_error is not None:
        raise NativeReaderError(_load_error)
    try:
        library = C.CDLL(str(Path(__file__).resolve().parents[2] / 'build/liblidar_geometry.so'))
        abi = library.lidar_reader_abi_version
        abi.argtypes = []; abi.restype = C.c_uint32
        if abi() != 1:
            raise ValueError('unsupported lidar reader ABI')
        library.lidar_reader_create_v1.argtypes = [C.POINTER(_Config)]
        library.lidar_reader_create_v1.restype = C.c_void_p
        library.lidar_reader_destroy_v1.argtypes = [C.c_void_p]
        library.lidar_reader_destroy_v1.restype = None
        library.lidar_reader_feed_v1.argtypes = [C.c_void_p, C.c_char_p, C.c_size_t,
            C.c_int64, _Scan, C.c_void_p]
        library.lidar_reader_feed_v1.restype = C.c_int
        library.lidar_reader_get_status_v1.argtypes = [C.c_void_p, C.POINTER(_Status)]
        library.lidar_reader_get_status_v1.restype = C.c_int
        _library = library
        return library
    except (OSError, AttributeError, ValueError) as error:
        _load_error = str(error)
        raise NativeReaderError(_load_error) from error


class NativeLidarStream:
    """Same feed/scan objects and counters, with borrowed C data copied on emit."""
    backend = 'cpp_bytes_timing_sector_v1'
    backend_error = None

    def __init__(self, reference):
        if reference.receive_calibration.verified or reference.adapter.clock_map is not None:
            raise NativeReaderError('supplied/calibrated clock uses original Python reader')
        config = _Config()
        values = dict(bearing_sign=reference._bearing_sign,
            bearing_zero_rad=reference._bearing_zero, direction=reference.packet_config.direction,
            entry=reference.sector.entry, width=reference.sector.width)
        for name in _DOUBLE_FIELDS + _INTEGER_FIELDS:
            if name not in values:
                for source in (reference.packet_config, reference.sector, reference):
                    if hasattr(source, name):
                        values[name] = getattr(source, name)
                        break
            value = values[name]
            if name in _INTEGER_FIELDS and not 0 <= value <= 2**63 - 1:
                raise NativeReaderError('configuration integer outside native int64 profile')
            setattr(config, name, value)
        self._library = _load()
        self._handle = self._library.lidar_reader_create_v1(C.byref(config))
        if not self._handle:
            raise NativeReaderError('native lidar reader allocation/configuration failed')
        self.sensor_epoch = reference.sensor_epoch
        self.time_map_version = 'arrival-unverified:' + self.sensor_epoch
        self.packet_config = reference.packet_config
        self.stats = dict(reference.stats)
        self.needs_new_epoch = False
        self.last_error = ''

    def close(self):
        handle = getattr(self, '_handle', None)
        if handle:
            self._library.lidar_reader_destroy_v1(handle)
            self._handle = None

    def __del__(self):
        self.close()

    def _status(self):
        status = _Status()
        if not self._library.lidar_reader_get_status_v1(self._handle, C.byref(status)):
            raise NativeReaderError('native lidar reader status failed')
        self.stats.update(zip(_COUNTERS, status.counters))
        self.needs_new_epoch = bool(status.needs_new_epoch)
        self.last_error = status.last_error.decode('utf-8', errors='replace')

    def feed(self, data, arrival_ns, *, on_scan=None):
        arrival_ns = require_time_ns(arrival_ns, 'arrival_ns')
        if self.needs_new_epoch:
            raise LidarTimingError('reader requires explicit new source/clock epoch')
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError('bytes or bytearray required')
        if not self._handle:
            raise NativeReaderError('native lidar reader is closed')
        output, errors = [], []

        @_Scan
        def emit(_user, raw_points, count):
            try:
                scan = _copy_native_scan(raw_points, count, self.sensor_epoch, self.time_map_version)
                output.append(scan)
                if on_scan is not None:
                    self._status()
                    on_scan(scan)
                return 1
            except BaseException as error:
                errors.append(error)
                return 0

        result = self._library.lidar_reader_feed_v1(self._handle, bytes(data), len(data),
            arrival_ns, emit, None)
        self._status()
        if errors:
            raise errors[0]
        if result == -1:
            raise LidarTimingError(self.last_error)
        if result == -4:
            raise ValueError('time_ns must be a nonnegative int64 monotonic timestamp')
        if result:
            raise NativeReaderError(self.last_error or f'native lidar reader error {result}')
        return output


def make_lidar_stream(config, *, sensor_epoch, clock_map=None, native=True):
    reference = LidarStream(config, sensor_epoch=sensor_epoch, clock_map=clock_map)
    reference.backend = 'python'
    reference.backend_error = None
    if native and os.environ.get('RDK_LIDAR_READER_NATIVE', '1').lower() not in ('0', 'off', 'false'):
        try:
            return NativeLidarStream(reference)
        except NativeReaderError as error:
            reference.backend_error = str(error)
    return reference
