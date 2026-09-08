#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"

if [[ ! -x "${script_dir}/build/t265_map_builder" ]]; then
    echo "Error: t265_map/build/t265_map_builder is missing." >&2
    echo "Build it with: cmake -S ${script_dir} -B ${script_dir}/build -DREALSENSE_ROOT=/home/sunrise/文档/ChatGPT/T265" >&2
    exit 1
fi

# Reuse the existing non-persistent 0.2.0.951 boot helper when the camera is
# enumerated in the official T265 boot state. No firmware file is changed.
boot_tool="${project_root}/t265_trajectory/build/t265_boot"
if command -v lsusb >/dev/null 2>&1 && lsusb -d 03e7:2150 >/dev/null 2>&1; then
    if [[ ! -x "${boot_tool}" ]]; then
        echo "Error: T265 is in bootloader state and ${boot_tool} is missing." >&2
        exit 2
    fi
    if command -v timeout >/dev/null 2>&1; then
        if ! timeout --signal=TERM --kill-after=3s 30s \
            "${boot_tool}" --chunk-kib 256; then
            echo "Error: T265 bootloader did not enter running state within 30 s." >&2
            exit 2
        fi
    else
        "${boot_tool}" --chunk-kib 256
    fi
fi

export DISPLAY="${DISPLAY:-:0}"
cd "${project_root}"
exec "${script_dir}/build/t265_map_builder" --fullscreen "$@"
