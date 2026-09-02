#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
localizer="${project_dir}/localization/build/t265_omni_localizer"

if ! python3 -c 'import cv2, numpy; from PIL import Image, ImageDraw, ImageFont' >/dev/null 2>&1; then
    echo "缺少Python依赖：需要OpenCV、NumPy和Pillow。" >&2
    exit 1
fi

if [[ ! -x "${localizer}" ]]; then
    cat >&2 <<EOF
定位程序尚未编译。请先执行：
  cmake -S "${project_dir}/localization" -B "${project_dir}/localization/build" \\
    -DCMAKE_BUILD_TYPE=Release -DREALSENSE_ROOT=/home/sunrise/文档/ChatGPT/T265
  cmake --build "${project_dir}/localization/build" -j4
EOF
    exit 1
fi

if [[ ! -r /dev/ttyS1 || ! -w /dev/ttyS1 ]]; then
    echo "警告：当前用户不能读写/dev/ttyS1；可先用--uart none只测试T265。" >&2
fi

export DISPLAY="${DISPLAY:-:0}"
exec python3 "${script_dir}/map_app.py" \
    --launch-localization \
    --uart /dev/ttyS1 \
    --baud 115200 \
    --fullscreen \
    "$@"
