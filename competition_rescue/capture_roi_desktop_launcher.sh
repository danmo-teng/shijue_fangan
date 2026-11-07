#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}" || exit 1

cat <<'HELP'
识别区域调整：复用原来的夹内ROI编辑器

请先退出占用摄像头的比赛/预热/其他视觉程序，并将摄像头摆在140°夹内观察姿态。
本工具显示摄像头和YOLO识别，不控制底盘或相机舵机。

调整方法：C 清空旧轮廓 → 鼠标左键沿新区域边缘依次点选至少3个顶点。
右键撤销最后一点；R 重新载入已保存区域；S 保存；Q / Esc 退出。
左右爪区域由大区域自动按中线分开。

保存文件：vision/config/capture_roi.json
保存后重启比赛/预热程序，正式流程读取新区域。
HELP

./run_capture_roi_editor.sh "$@"
status=$?
if (( status != 0 )); then
    echo
    echo "识别区域调整启动失败，退出码：${status}；请查看上方错误信息。"
    if [[ -t 0 ]]; then
        read -r -p "按回车键关闭终端..." _
    fi
fi
exit "${status}"
