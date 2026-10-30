#!/usr/bin/env bash
# Rebuild usb_dac_2 and flash via picotool (USB 1200-baud BOOTSEL reset).
# Usage: ./rebuild_flash.sh
set -euo pipefail

cd "$(dirname "$0")"

# shellcheck source=/dev/null
. ./exports.sh
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"

TARGET=usb_dac_2
UF2="build/${TARGET}.uf2"

if [[ ! -d build ]]; then
  echo "build/ missing; configuring..."
  cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w
fi

echo "Building ${TARGET}..."
cmake --build build --target "${TARGET}"

if [[ ! -f "${UF2}" ]]; then
  echo "error: ${UF2} not found after build" >&2
  exit 1
fi

echo "Flashing ${UF2}..."
picotool load -f "${UF2}"

echo "Done."
