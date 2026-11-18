#!/usr/bin/env bash
# Read the UVC camera while the independent S3 bench firmware cycles.
set -eu
if [[ ${1:-} == --help ]]; then
    echo "Usage: $0 [UVC device path] [output directory]; watch for 100 seconds"
    exit 0
fi
dev=${1:-/dev/v4l/by-id/usb-DHZJ_250303_XH_DHZJ_Camera_01.00.00-video-index0}
out=${2:-$(mktemp -d /tmp/rdk-camera-usb-XXXXXX)}
mkdir -p "$out"
log=$(mktemp "$out/stream-XXXXXX.log")
monitor=$(mktemp "$out/udev-XXXXXX.log")
for tool in gst-launch-1.0 udevadm timeout; do command -v "$tool" >/dev/null; done
if [[ ! -e $dev ]]; then echo "Camera unavailable: $dev" >&2; exit 1; fi
echo "Stream log: $log; USB/video events: $monitor"
echo "Start the bench cycle once streaming begins. Leave the RDK powered."
timeout 100s stdbuf -oL udevadm monitor --kernel --subsystem-match=usb --subsystem-match=video4linux >"$monitor" 2>&1 &
pid=$!
trap 'kill "$pid" 2>/dev/null || true' EXIT
end=$((SECONDS+100))
while (( SECONDS < end )); do
    if [[ ! -e $dev ]]; then sleep .2; continue; fi
    read -r uptime _ </proc/uptime
    echo "open at uptime=$uptime" >>"$log"
    result=0
    timeout -k 2s "$((end-SECONDS > 0 ? end-SECONDS : 1))s" gst-launch-1.0 -v v4l2src device="$dev" ! 'image/jpeg,width=1280,height=1024' ! fakesink sync=false >>"$log" 2>&1 || result=$?
    read -r uptime _ </proc/uptime
    echo "stream exit=$result uptime=$uptime" >>"$log"
    if (( result == 124 )); then break; fi
    sleep .2
done
