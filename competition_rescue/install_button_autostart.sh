#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
target_dir="${XDG_CONFIG_HOME:-${HOME}/.config}/autostart"
mkdir -p "${target_dir}"
install -m 0644 "${script_dir}/button-autostart.desktop" "${target_dir}/rdk-rescue-buttons.desktop"
echo "已安装登录桌面后的实体按键待机；不会自动启动比赛。"
echo "当前未启动硬件。下次登录生效，或手动打开原比赛桌面入口。"
