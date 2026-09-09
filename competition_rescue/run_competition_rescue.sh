#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
localizer="${project_dir}/localization/build/t265_omni_localizer"
vision_model="${project_dir}/vision/models/best_bayese_320x320_nv12.bin"

if ! python3 -c 'import cv2, numpy; from PIL import Image, ImageDraw, ImageFont' >/dev/null 2>&1; then
    echo "缺少Python依赖：需要OpenCV、NumPy和Pillow。" >&2
    exit 1
fi

if [[ ! -f "${vision_model}" ]]; then
    echo "YOLO模型缺失：${vision_model}" >&2
    exit 1
fi
if [[ ! -x "${localizer}" ]]; then
    echo "定位程序尚未编译：${localizer}" >&2
    exit 1
fi
if [[ ! -r /dev/ttyS1 || ! -w /dev/ttyS1 ]]; then
    echo "警告：当前用户不能读写/dev/ttyS1；完整比赛流程将等待串口转发。" >&2
fi

export DISPLAY="${DISPLAY:-:0}"
exec python3 "${script_dir}/run_competition_rescue.py" "$@"
