#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

gcc -O2 -Wall -Wextra -fPIC -shared \
  -I/usr/include \
  "$SCRIPT_DIR/jpu_decoder.c" \
  -L/usr/hobot/lib -Wl,-rpath,/usr/hobot/lib \
  -lmultimedia -lhbmem -lalog -lpthread -ldl \
  -o "$SCRIPT_DIR/librescue_jpu.so"

echo "Built $SCRIPT_DIR/librescue_jpu.so"
