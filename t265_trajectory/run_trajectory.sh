#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -x "${script_dir}/build/t265_boot" ||
      ! -x "${script_dir}/build/t265_trajectory_debug" ]]; then
    echo "Error: build the T265 standalone project first." >&2
    exit 1
fi

if lsusb -d 03e7:2150 >/dev/null 2>&1; then
    if ! "${script_dir}/build/t265_boot" --chunk-kib 256; then
        echo "T265引导端点无响应。请拔掉T265本体5秒后重新插入，再运行本脚本。" >&2
        exit 2
    fi
fi

exec "${script_dir}/build/t265_trajectory_debug" --fullscreen "$@"
