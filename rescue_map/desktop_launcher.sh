#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}" || exit 1

export DISPLAY="${DISPLAY:-:0}"
./run_rescue_map.sh
status=$?

if (( status != 0 )); then
    echo
    echo "一键启动失败，退出码：${status}"
    echo "请检查上面的T265、摄像头、模型、串口权限或START_POSE_MISMATCH提示。"
    read -r -p "按回车键关闭终端..." _
fi

exit "${status}"
