#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
desktop_dir=""
if command -v xdg-user-dir >/dev/null 2>&1; then
    desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
if [[ -z "${desktop_dir}" ]]; then
    if [[ -d "/home/sunrise/桌面" ]]; then
        desktop_dir="/home/sunrise/桌面"
    else
        desktop_dir="${HOME}/Desktop"
    fi
fi

mkdir -p "${desktop_dir}"
install -m 0755 "${script_dir}/desktop_launcher.sh" \
    "${desktop_dir}/f407-gamepad-control-launcher.sh"
install -m 0644 "${script_dir}/gamepad-control.desktop" \
    "${desktop_dir}/F407-手柄控制模式.desktop"
chmod +x "${desktop_dir}/F407-手柄控制模式.desktop" 2>/dev/null || true
if command -v gio >/dev/null 2>&1; then
    gio set "${desktop_dir}/F407-手柄控制模式.desktop" \
        metadata::trusted true >/dev/null 2>&1 || true
fi
echo "已安装桌面入口：${desktop_dir}/F407-手柄控制模式.desktop"
