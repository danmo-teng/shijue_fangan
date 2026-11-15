#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}" || exit 1

cat <<'HELP'
识别区域调整：复用原来的夹内ROI编辑器

请先退出占用摄像头的比赛/预热/其他视觉程序，并将摄像头摆在140°夹内观察姿态。
本工具显示摄像头和YOLO识别，不控制底盘或相机舵机。

延时摄影：先移开手和物块，点画面下方“开始／继续延时摄影”。
将物块沿爪子内侧或外侧移动一圈，经过的位置会留在同一张图中；点“暂停延时摄影”后照着图描ROI。
“清除叠影”只清图片，不清ROI；“看实时/看叠影”切换画面；“保存叠影图片”单独保存图片，不改ROI。
拍摄时固定相机和爪子，手和其他移动物也会留下影像。换一种物块前先保存图片，再清除叠影重拍。
图片保存位置：vision/config/capture_roi_trails/，文件名各自独立，不覆盖原图。

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
