#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
# Match runs omit diagnostic recordings. --record enables field-test evidence.
export RDK_MATCH_RECORDING="${RDK_MATCH_RECORDING:-0}"
launcher_args=()
for arg in "$@"; do
    case "$arg" in
        --record) export RDK_MATCH_RECORDING=1 ;;
        --no-record) export RDK_MATCH_RECORDING=0 ;;
        *) launcher_args+=("$arg") ;;
    esac
done
case "$RDK_MATCH_RECORDING" in 0|1) ;; *) echo 'RDK_MATCH_RECORDING must be 0 or 1' >&2; exit 2 ;; esac
echo "比赛诊断记录 RDK_MATCH_RECORDING=$RDK_MATCH_RECORDING（0关闭，1开启）"
export RDK_FORMAL_LOCALIZATION_BUILD="${RDK_FORMAL_LOCALIZATION_BUILD:-${project_dir}/localization/build/formal-unload-guard}"
localizer="$RDK_FORMAL_LOCALIZATION_BUILD/t265_omni_localizer"
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
    "${launcher_args[@]}"
