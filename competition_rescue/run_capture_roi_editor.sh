#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
vision_model="${project_dir}/vision/models/best_bayese_320x320_nv12.bin"

if [[ ! -f "${vision_model}" ]]; then
    echo "YOLO模型缺失：${vision_model}" >&2
    exit 1
fi
if ! python3 -c 'import cv2, numpy' >/dev/null 2>&1; then
    echo "缺少Python依赖：需要OpenCV和NumPy。" >&2
    exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
exec python3 "${script_dir}/edit_capture_roi.py" "$@"
