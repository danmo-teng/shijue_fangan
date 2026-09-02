#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
boot_tool="${script_dir}/../t265_trajectory/build/t265_boot"

if [[ ! -x "${script_dir}/build/t265_omni_localizer" ]]; then
    echo "Error: build/t265_omni_localizer is missing; build the project first." >&2
    exit 1
fi
if lsusb -d 03e7:2150 >/dev/null 2>&1; then
    if [[ ! -x "${boot_tool}" ]]; then
        echo "Error: T265 is in boot state but ${boot_tool} is unavailable." >&2
        exit 1
    fi
    "${boot_tool}" --chunk-kib 256
fi

exec "${script_dir}/build/t265_omni_localizer" \
    --config "${script_dir}/config/localization.example.conf" \
    --output "${script_dir}/localization_result.json" \
    "$@"
