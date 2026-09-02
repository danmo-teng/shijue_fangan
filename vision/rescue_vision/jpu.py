from __future__ import annotations

import ctypes
from pathlib import Path

import cv2
import numpy as np


class JpuDecodeTimeout(RuntimeError):
    pass


class JpuDecoder:
    """Small Python wrapper around the X5 hb_media_codec JPEG decoder."""

    def __init__(self, width: int, height: int) -> None:
        library_path = Path(__file__).resolve().parents[1] / "native" / "librescue_jpu.so"
        if not library_path.exists():
            raise RuntimeError(f"JPU库不存在，请先运行 bash native/build_jpu.sh：{library_path}")
        self.width = width
        self.height = height
        self.library = ctypes.CDLL(str(library_path))
        self.library.rdk_jpu_create.argtypes = (ctypes.c_int, ctypes.c_int)
        self.library.rdk_jpu_create.restype = ctypes.c_void_p
        self.library.rdk_jpu_decode.argtypes = (
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32,
        )
        self.library.rdk_jpu_decode.restype = ctypes.c_int
        self.library.rdk_jpu_destroy.argtypes = (ctypes.c_void_p,)
        self.handle = self.library.rdk_jpu_create(width, height)
        if not self.handle:
            raise RuntimeError("JPU初始化失败，可能没有可用实例或媒体服务未就绪")
        self.nv12 = np.empty((height * 3 // 2, width), dtype=np.uint8)

    def decode(self, jpeg: bytes) -> np.ndarray:
        source = (ctypes.c_uint8 * len(jpeg)).from_buffer_copy(jpeg)
        destination = self.nv12.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        result = self.library.rdk_jpu_decode(
            self.handle, source, len(jpeg), destination, self.nv12.nbytes)
        if result == -20:
            raise JpuDecodeTimeout("JPU输出等待超时")
        if result < 0:
            raise RuntimeError(f"JPU JPEG解码失败，错误码{result}")
        return cv2.cvtColor(self.nv12, cv2.COLOR_YUV2BGR_NV12)

    def close(self) -> None:
        if self.handle:
            self.library.rdk_jpu_destroy(self.handle)
            self.handle = None

    def __enter__(self) -> "JpuDecoder":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
