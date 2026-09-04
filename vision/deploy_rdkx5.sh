#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  python3-opencv \
  python3-numpy \
  python3-serial \
  python3-gi \
  gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  v4l-utils

python3 -c 'import cv2, numpy, serial, gi; gi.require_version("Gst", "1.0"); from gi.repository import Gst; print("OpenCV", cv2.__version__, "deployment OK")'
bash "$(dirname "$0")/native/build_jpu.sh"
bash "$(dirname "$0")/native/build_vse.sh"

echo "Dependencies installed. Run:"
echo "  python3 web_editor.py --device /dev/video0 --decoder jpu --decode-fps 60"
echo "  python3 run_normal_supply_uart.py --uart /dev/ttyS1"
echo "  python3 run_yolo_x5.py --device /dev/video0 --preprocess auto"
