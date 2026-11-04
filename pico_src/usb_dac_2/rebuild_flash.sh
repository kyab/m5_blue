#!/usr/bin/env bash
# Rebuild usb_dac_2 and flash via picotool.
# This app enables PICO_ENABLE_USB_RESET_VIA_BAUD_RATE and disables the
# vendor Reset interface, so `picotool load -f` cannot enter BOOTSEL.
# Open the CDC port at 1200 once, then `picotool load -x`.
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

in_bootsel() {
  picotool info >/dev/null 2>&1
}

open_cdc_1200_once() {
  local port="$1"
  python3 - "$port" <<'PY'
import errno
import os
import sys
import termios

port = sys.argv[1]
if termios.B1200 != 1200:
    sys.exit(f"refusing: B1200 constant is {termios.B1200}")

def disconnected(exc: BaseException) -> bool:
    return isinstance(exc, (termios.error, OSError)) and getattr(exc, "errno", None) in (
        errno.ENXIO,
        errno.ENODEV,
    )

fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
    try:
        attrs = termios.tcgetattr(fd)
    except Exception as exc:
        # Open alone can already trip USB 1200-baud BOOTSEL; port is gone.
        if disconnected(exc):
            sys.exit(0)
        raise
    cc = attrs[6]
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
    try:
        termios.tcsetattr(
            fd,
            termios.TCSANOW,
            [0, 0, cflag, 0, termios.B1200, termios.B1200, cc],
        )
        got = termios.tcgetattr(fd)
    except Exception as exc:
        # Device often drops as soon as 1200 is applied; that is success.
        if disconnected(exc):
            sys.exit(0)
        raise
    if got[4] != 1200 or got[5] != 1200:
        sys.exit(f"refusing: baud stayed ispeed={got[4]} ospeed={got[5]}")
finally:
    try:
        os.close(fd)
    except OSError:
        pass
PY
}

wait_for_bootsel() {
  local i
  for i in $(seq 1 40); do
    if in_bootsel; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

wait_for_cdc() {
  local i
  for i in $(seq 1 40); do
    local found=0
    local port
    shopt -s nullglob
    for port in /dev/cu.usbmodem*; do
      found=1
      break
    done
    shopt -u nullglob
    if [[ "${found}" -eq 1 ]]; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

enter_bootsel() {
  if in_bootsel; then
    echo "Already in BOOTSEL."
    return 0
  fi

  shopt -s nullglob
  local ports=(/dev/cu.usbmodem*)
  shopt -u nullglob
  if [[ "${#ports[@]}" -ne 1 ]]; then
    echo "error: expected one CDC port at /dev/cu.usbmodem* (found ${#ports[@]})." >&2
    echo "Hold BOOTSEL once if the board is not running usb_dac_2." >&2
    if [[ "${#ports[@]}" -gt 0 ]]; then
      printf '  %s\n' "${ports[@]}" >&2
    fi
    exit 1
  fi

  echo "Opening ${ports[0]} at 1200 once to enter BOOTSEL..."
  open_cdc_1200_once "${ports[0]}"
  if ! wait_for_bootsel; then
    echo "error: BOOTSEL did not appear after one 1200 open. Hold BOOTSEL once." >&2
    exit 1
  fi
}

echo "Flashing ${UF2}..."
enter_bootsel
set +e
picotool load -x "${UF2}"
load_rc=$?
set -e
if ! wait_for_cdc; then
  echo "error: CDC did not return after picotool load -x (status ${load_rc})." >&2
  exit 1
fi

echo "Done. Application CDC is back. Do not open it at 1200."
