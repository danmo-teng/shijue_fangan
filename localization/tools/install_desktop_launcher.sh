#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
desktop_dir=""
if command -v xdg-user-dir >/dev/null 2>&1; then
    desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
if [[ -z "${desktop_dir}" ]]; then
    desktop_dir="/home/sunrise/Desktop"
fi
mkdir -p "${desktop_dir}"
install -m 755 "${script_dir}/desktop_launcher.sh" "${desktop_dir}/t265-f407-debug-launcher.sh"
install -m 644 "${script_dir}/t265-f407-debug.desktop" "${desktop_dir}/T265-F407-调试地图.desktop"
chmod +x "${desktop_dir}/T265-F407-调试地图.desktop" 2>/dev/null || true
echo "已安装到：${desktop_dir}/T265-F407-调试地图.desktop"
