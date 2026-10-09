#!/usr/bin/env bash
# Rebuild a2dp_sink_demo, flash via picotool, then reconnect A2DP on macOS.
# Usage: ./rebuild_flash_reconnect.sh [bluetooth-address-or-name]
# Do not open /dev/cu.usbmodem* here: 1200 baud reboots into BOOTSEL.
set -euo pipefail

cd "$(dirname "$0")"

# shellcheck source=/dev/null
. ./exports.sh
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"

TARGET=a2dp_sink_demo
UF2="build/${TARGET}.uf2"

if [[ ! -d build ]]; then
  echo "build/ missing; configuring..."
  cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH="$PWD/pico-extras"
fi

echo "Building ${TARGET}..."
cmake --build build --target "${TARGET}"

if [[ ! -f "${UF2}" ]]; then
  echo "error: ${UF2} not found after build" >&2
  exit 1
fi

echo "Flashing ${UF2}..."
picotool load -f "${UF2}"
echo "Flash OK: ${UF2}"

echo "Waiting for CDC..."
sleep 2
cdc=(/dev/cu.usbmodem*)
if [[ ! -e "${cdc[0]}" ]]; then
  echo "error: CDC did not return. Hold BOOTSEL once if the board is not in application mode." >&2
  exit 1
fi
echo "CDC: ${cdc[*]}"

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "Reconnect skipped: blueutil works only on macOS."
  exit 0
fi

for brew_bin in /opt/homebrew/bin/blueutil /usr/local/bin/blueutil; do
  if [[ -x "${brew_bin}" ]]; then
    export PATH="$(dirname "${brew_bin}"):$PATH"
    break
  fi
done
if ! command -v blueutil >/dev/null 2>&1; then
  echo "error: blueutil is not installed. Install it with Homebrew (blueutil), then run this script again." >&2
  exit 1
fi

echo "Waiting for the board to become connectable..."
sleep 5

pick_device() {
  local want="${1:-}"
  if [[ -n "${want}" ]]; then
    printf '%s\n' "${want}"
    return
  fi

  local -a addrs=()
  local -a names=()
  local line addr name seen a
  while IFS= read -r line; do
    [[ "${line}" == address:* ]] || continue
    addr="${line#address: }"
    addr="${addr%%,*}"
    if [[ "${line}" =~ name:\ \"([^\"]*)\" ]]; then
      name="${BASH_REMATCH[1]}"
    else
      continue
    fi
    [[ "${name}" == "A2DP Sink Demo"* ]] || continue
    seen=0
    if [[ ${#addrs[@]} -gt 0 ]]; then
      for a in "${addrs[@]}"; do
        if [[ "${a}" == "${addr}" ]]; then
          seen=1
          break
        fi
      done
    fi
    [[ "${seen}" -eq 1 ]] && continue
    addrs+=("${addr}")
    names+=("${name}")
  done < <(blueutil --paired)

  if [[ ${#addrs[@]} -eq 0 ]]; then
    echo "error: no paired device whose name starts with \"A2DP Sink Demo\"." >&2
    exit 1
  fi
  if [[ ${#addrs[@]} -gt 1 ]]; then
    echo "error: several A2DP Sink Demo devices. Pass one address:" >&2
    local i
    for i in "${!addrs[@]}"; do
      echo "  ${addrs[$i]}  ${names[$i]}" >&2
    done
    exit 1
  fi
  printf '%s\n' "${addrs[0]}"
}

device="$(pick_device "${1:-}")"
echo "Connecting ${device}..."
if ! blueutil --connect "${device}"; then
  echo "Connect failed; retrying once..."
  sleep 5
  blueutil --connect "${device}"
fi

connected="$(blueutil --is-connected "${device}")"
info="$(blueutil --info "${device}")"
echo "${info}"
if [[ "${connected}" != "1" || "${info}" != *connected* ]]; then
  echo "error: not connected (is-connected=${connected})" >&2
  exit 1
fi

devname="${device}"
if [[ "${info}" =~ name:\ \"([^\"]*)\" ]]; then
  devname="${BASH_REMATCH[1]}"
fi

audio="$(system_profiler SPAudioDataType)"
# The device header is followed by a blank line, then indented properties.
# Stop at the next device, which is indented the same as the header.
audio_block="$(awk -v name="${devname}" '
  !p && index($0, name) {
    p = 1
    match($0, /^ */)
    header_indent = RLENGTH
    print
    next
  }
  p {
    if ($0 ~ /^$/) next
    match($0, /^ */)
    if (RLENGTH <= header_indent) exit
    print
  }
' <<<"${audio}")"
echo "${audio_block}"
if [[ "${audio_block}" != *"Transport: Bluetooth"* ]]; then
  echo "error: ${devname} is connected but is not a Bluetooth audio output." >&2
  exit 1
fi

echo "Reconnect OK: ${devname} (${device}), is-connected=1, Transport=Bluetooth"
