#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
desktop_dir=""
if command -v xdg-user-dir >/dev/null 2>&1; then
    desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
if [[ -z "${desktop_dir}" ]]; then
    desktop_dir="/home/sunrise/桌面"
fi

mkdir -p "${desktop_dir}"
install -m 0755 "${script_dir}/desktop_launcher.sh" \
    "${desktop_dir}/t265-map-builder-launcher.sh"
install -m 0644 "${script_dir}/t265-map-builder.desktop" \
    "${desktop_dir}/T265-环境扫描建图.desktop"
chmod +x "${desktop_dir}/T265-环境扫描建图.desktop" 2>/dev/null || true
if command -v gio >/dev/null 2>&1; then
    gio set "${desktop_dir}/T265-环境扫描建图.desktop" \
        metadata::trusted true >/dev/null 2>&1 || true
fi
echo "桌面快捷方式已安装：${desktop_dir}/T265-环境扫描建图.desktop"
