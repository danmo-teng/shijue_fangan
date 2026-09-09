#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
localizer="${project_dir}/localization/build/t265_omni_localizer"
vision_model="${project_dir}/vision/models/best_bayese_320x320_nv12.bin"

if [[ ! -x "${localizer}" || ! -f "${vision_model}" ]]; then
    echo "完整比赛流程依赖缺失：请确认定位程序和YOLO模型已部署。" >&2
    exit 1
fi
if ! python3 -c 'import cv2, numpy; from PIL import Image, ImageDraw, ImageFont' >/dev/null 2>&1; then
    echo "缺少Python依赖：需要OpenCV、NumPy和Pillow。" >&2
    exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
exec python3 "${script_dir}/competition_map_app.py" \
    --launch-localization \
    --launch-vision \
    --uart /dev/ttyS1 \
    --baud 115200 \
    --fullscreen \
    "$@"
