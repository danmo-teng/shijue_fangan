#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}" || exit 1
export DISPLAY="${DISPLAY:-:0}"
python3 "${script_dir}/capture_training_burst.py" "$@"
status=$?
if (( status != 0 )); then
    echo "批量拍摄启动失败，请检查相机连接及是否被比赛程序占用。"
    read -r -p "按回车键关闭终端..." _
fi
exit "${status}"
