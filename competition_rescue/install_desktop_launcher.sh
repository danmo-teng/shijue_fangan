#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
if [[ -z "${desktop_dir}" ]]; then
    desktop_dir="/home/sunrise/桌面"
fi

mkdir -p "${desktop_dir}"
install -m 0755 "${script_dir}/complete-rescue.desktop" \
    "${desktop_dir}/智能救援完整比赛流程.desktop"
if command -v gio >/dev/null 2>&1; then
    gio set "${desktop_dir}/智能救援完整比赛流程.desktop" \
        metadata::trusted true >/dev/null 2>&1 || true
fi
echo "桌面快捷方式已安装：${desktop_dir}/智能救援完整比赛流程.desktop"
