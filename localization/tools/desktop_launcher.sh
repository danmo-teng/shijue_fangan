#!/usr/bin/env bash
set -euo pipefail

project_root="/home/sunrise/RDK_X5/shijue_fangan"
export DISPLAY="${DISPLAY:-:0}"
cd "${project_root}"
exec python3 "${project_root}/localization/tools/t265_f407_debug_map.py" --fullscreen "$@"
