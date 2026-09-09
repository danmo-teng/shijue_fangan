#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "${script_dir}/.." && pwd)"
map_command="${GAMEPAD_T265_COMMAND:-${project_root}/t265_map/run_t265_map.sh --enable-motion --uart {pty}}"

export DISPLAY="${DISPLAY:-:0}"
exec python3 "${script_dir}/gamepad_control.py" \
    --uart "${GAMEPAD_UART:-/dev/ttyS1}" \
    --map-command "${map_command}" \
    "$@"
