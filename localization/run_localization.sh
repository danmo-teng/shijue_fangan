#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
boot_tool="${script_dir}/../t265_trajectory/build/t265_boot"
experimental_robust=false
for argument in "$@"; do
    if [[ "${argument}" == "--experimental-robust-localization" ]]; then
        experimental_robust=true
    fi
done

if [[ ! -x "${script_dir}/build/t265_omni_localizer" ]]; then
    echo "Error: build/t265_omni_localizer is missing; build the project first." >&2
    exit 1
fi
if ${experimental_robust} && [[ "${RDK_T265_TEST_BOOT_ATTEMPTED:-0}" == "1" ]]; then
    echo "[T265 START] test entry already completed its single preparation attempt; continue sole UART/SDK owner." >&2
elif lsusb -d 03e7:2150 >/dev/null 2>&1; then
    if [[ ! -x "${boot_tool}" ]]; then
        echo "Error: T265 is in boot state but ${boot_tool} is unavailable." >&2
        if ! ${experimental_robust}; then exit 1; fi
        echo "[T265 UNAVAILABLE] experimental sole owner will retain UART without fabricated Pose." >&2
    elif "${boot_tool}" --chunk-kib 256; then
        :
    else
        boot_status=$?
        echo "[T265 START] single firmware preparation failed, status=${boot_status}." >&2
        if ! ${experimental_robust}; then exit "${boot_status}"; fi
        echo "[T265 UNAVAILABLE] continue experimental sole UART/SDK owner; no second upload attempt." >&2
    fi
fi

exec "${script_dir}/build/t265_omni_localizer" \
    --config "${script_dir}/config/localization.example.conf" \
    --output "${script_dir}/localization_result.json" \
    "$@"
