#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
if [[ -z "${desktop_dir}" ]]; then
    desktop_dir="/home/sunrise/桌面"
fi

mkdir -p "${desktop_dir}"
for entry in "complete-rescue.desktop:智能救援完整比赛流程.desktop" \
             "capture-roi-editor.desktop:识别区域调整.desktop"; do
    source_name="${entry%%:*}"
    target_name="${entry#*:}"
    install -m 0755 "${script_dir}/${source_name}" "${desktop_dir}/${target_name}"
    if command -v gio >/dev/null 2>&1; then
        gio set "${desktop_dir}/${target_name}" metadata::trusted true >/dev/null 2>&1 || true
    fi
    echo "桌面快捷方式已安装：${desktop_dir}/${target_name}"
done
