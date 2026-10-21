#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}" || exit 1
export DISPLAY="${DISPLAY:-:0}"
python3 "${script_dir}/calibrate_range_view.py" --pitch 105 --bands 6 "$@"
status=$?
if (( status != 0 )); then
    echo
    echo "测距标定画面启动失败，退出码：${status}"
    echo "请确认比赛程序已关闭，并检查摄像头连接及上方错误信息。"
    read -r -p "按回车键关闭终端..." _
fi
exit "${status}"
