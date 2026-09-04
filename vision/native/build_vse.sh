#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

gcc -O3 -Wall -Wextra -Werror -fPIC -shared \
  -I/usr/include \
  "$script_dir/vse_scaler.c" \
  -L/usr/hobot/lib -Wl,-rpath,/usr/hobot/lib \
  -lvpf -lhbmem -lalog -lpthread -ldl \
  -o "$script_dir/librescue_vse.so"

echo "Built $script_dir/librescue_vse.so"
