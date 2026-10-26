#!/usr/bin/env bash
set -uo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}" || exit 1
export DISPLAY="${DISPLAY:-:0}"
python3 "${script_dir}/benchmark_yolo26_x5.py" "$@"
status=$?
echo
echo "YOLO26测速结束（退出码 ${status}），日志在 vision/captures/yolo26_benchmark。"
read -r -p "按回车关闭终端..." _
exit "${status}"
