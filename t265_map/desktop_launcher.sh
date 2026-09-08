#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export DISPLAY="${DISPLAY:-:0}"
cd "${script_dir}/.."
exec "${script_dir}/run_t265_map.sh" "$@"
