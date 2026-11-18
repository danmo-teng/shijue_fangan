#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
formal_bundle="${project_dir}/localization/build/formal-wall-yaw"
if [[ -x "$formal_bundle/t265_omni_localizer" && -f "$formal_bundle/libmotion_runtime.so" && -f "$formal_bundle/liblidar_geometry.so" ]]; then
    export RDK_FORMAL_LOCALIZATION_BUILD="$formal_bundle"
fi
localizer="${RDK_FORMAL_LOCALIZATION_BUILD:-${project_dir}/localization/build}/t265_omni_localizer"
vision_model="${project_dir}/yolo26n/x5_output_640_1117/yolo26n_6cls_640_1117_bpu_bayese_640x640_nv12.bin"

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
    --localization-mode t265 \
    --official-wheel-input \
    --dynamic-localization \
    --gpio-buttons \
    --tx-rate 20 \
    --disable-t265-translation-scale \
    --t265-translation-scale 1.000 \
    --uart /dev/ttyS1 \
    --baud 921600 \
    --fullscreen \
    "$@"
