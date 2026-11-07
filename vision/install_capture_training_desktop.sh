#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
desktop_dir="${desktop_dir:-/home/sunrise/桌面}"
mkdir -p "${desktop_dir}"
target="${desktop_dir}/比赛摄像头批量拍摄.desktop"
install -m 0755 "${script_dir}/capture_training.desktop" "${target}"
if command -v gio >/dev/null 2>&1; then
    gio set "${target}" metadata::trusted true >/dev/null 2>&1 || true
fi
echo "桌面快捷方式已安装：${target}"
