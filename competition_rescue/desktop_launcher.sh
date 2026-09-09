#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}" || exit 1
export DISPLAY="${DISPLAY:-:0}"
./run_competition_map.sh
status=$?

if (( status != 0 )); then
    echo
    echo "完整比赛流程启动失败，退出码：${status}"
    echo "请检查T265、摄像头、YOLO模型、串口权限和下位机完整比赛接口。"
    read -r -p "按回车键关闭终端..." _
fi
exit "${status}"
