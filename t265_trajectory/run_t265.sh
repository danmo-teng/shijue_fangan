#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -x "${script_dir}/build/t265_boot" || ! -x "${script_dir}/build/t265_debug" ]]; then
    echo "Error: build/t265_boot or build/t265_debug is missing; build the project first." >&2
    exit 1
fi

if lsusb -d 03e7:2150 >/dev/null 2>&1; then
    "${script_dir}/build/t265_boot" --chunk-kib 256
fi

exec "${script_dir}/build/t265_debug" "$@"
