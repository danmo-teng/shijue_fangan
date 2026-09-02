#!/usr/bin/env bash
set -euo pipefail

device="${1:-/dev/video0}"
exposure="${EXPOSURE:-10}"
white_balance="${WHITE_BALANCE:-4500}"
focus="${FOCUS:-264}"

v4l2-ctl -d "$device" --set-ctrl=auto_exposure=1
v4l2-ctl -d "$device" --set-ctrl=exposure_time_absolute="$exposure"
v4l2-ctl -d "$device" --set-ctrl=power_line_frequency=0
v4l2-ctl -d "$device" --set-ctrl=backlight_compensation=0
v4l2-ctl -d "$device" --set-ctrl=white_balance_automatic=0
v4l2-ctl -d "$device" --set-ctrl=white_balance_temperature="$white_balance"
v4l2-ctl -d "$device" --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d "$device" --set-ctrl=focus_absolute="$focus"

v4l2-ctl -d "$device" --get-ctrl=auto_exposure,exposure_time_absolute,white_balance_automatic,white_balance_temperature,focus_automatic_continuous,focus_absolute
