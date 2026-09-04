from __future__ import annotations

import ctypes
from pathlib import Path

import numpy as np


class VseScaler:
    """Safe copied-buffer wrapper around the X5 VSE feedback scaler."""

    def __init__(self, input_width: int, input_height: int,
                 output_width: int, output_height: int) -> None:
        library_path = Path(__file__).resolve().parents[1] / "native" / "librescue_vse.so"
        if not library_path.exists():
            raise RuntimeError(f"VSE库不存在，请先运行 bash native/build_vse.sh：{library_path}")
        self.input_width = input_width
        self.input_height = input_height
        self.output_width = output_width
        self.output_height = output_height
        self.library = ctypes.CDLL(str(library_path))
        self.library.rescue_vse_create.argtypes = (
            ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
        )
        self.library.rescue_vse_create.restype = ctypes.c_void_p
        self.library.rescue_vse_scale.argtypes = (
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32,
            ctypes.c_int,
        )
        self.library.rescue_vse_scale.restype = ctypes.c_int
        self.library.rescue_vse_destroy.argtypes = (ctypes.c_void_p,)
        self.handle = self.library.rescue_vse_create(
            input_width, input_height, output_width, output_height
        )
        if not self.handle:
            raise RuntimeError("VSE初始化失败，硬件缩放通道可能不可用")
        self.output = np.empty((output_height * 3 // 2, output_width), dtype=np.uint8)

    def scale(self, nv12: np.ndarray, timeout_ms: int = 50) -> np.ndarray:
        expected = self.input_width * self.input_height * 3 // 2
        if nv12.dtype != np.uint8 or not nv12.flags.c_contiguous or nv12.size != expected:
            raise ValueError(
                f"VSE输入必须是连续uint8 NV12，大小{expected}，当前{nv12.dtype}/{nv12.size}"
            )
        source = nv12.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        destination = self.output.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        result = self.library.rescue_vse_scale(
            self.handle, source, nv12.nbytes,
            destination, self.output.nbytes, timeout_ms,
        )
        if result < 0:
            raise RuntimeError(f"VSE缩放失败，错误码{result}")
        return self.output.copy()

    def close(self) -> None:
        if self.handle:
            self.library.rescue_vse_destroy(self.handle)
            self.handle = None

    def __enter__(self) -> "VseScaler":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
