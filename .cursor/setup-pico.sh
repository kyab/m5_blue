#!/usr/bin/env bash
# Idempotent Cloud Agent bootstrap for Raspberry Pi Pico firmware (pico_src).
# Installs the Arm GCC toolchain, Pico SDK 2.3.1, and picotool 2.3.0.
set -euo pipefail

SDK_VERSION="2.3.1"
PICOTOOL_VERSION="2.3.0"
SDK_DIR="${PICO_SDK_INSTALL_DIR:-$HOME/.pico-sdk/sdk/${SDK_VERSION}}"
PICOTOOL_SRC_DIR="${PICOTOOL_SRC_DIR:-$HOME/.pico-sdk/picotool-src/${PICOTOOL_VERSION}}"
PROFILE_SNIPPET='export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"'

echo "[setup-pico] Installing apt packages..."
sudo DEBIAN_FRONTEND=noninteractive apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
  build-essential \
  cmake \
  ninja-build \
  git \
  python3 \
  pkg-config \
  gcc-arm-none-eabi \
  libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib \
  libusb-1.0-0-dev

echo "[setup-pico] Ensuring Pico SDK ${SDK_VERSION} at ${SDK_DIR}..."
if [[ -f "${SDK_DIR}/pico_sdk_init.cmake" ]]; then
  echo "[setup-pico] SDK already present."
  # Refresh submodules in case a previous clone was incomplete.
  git -C "${SDK_DIR}" submodule update --init --recursive
else
  mkdir -p "$(dirname "${SDK_DIR}")"
  rm -rf "${SDK_DIR}"
  git clone --branch "${SDK_VERSION}" --depth 1 --recurse-submodules \
    https://github.com/raspberrypi/pico-sdk.git "${SDK_DIR}"
fi

# Persist for login shells used by install/start and later agent sessions.
if [[ -f "${HOME}/.profile" ]] && ! grep -Fq 'PICO_SDK_PATH=' "${HOME}/.profile"; then
  {
    echo ""
    echo "# Pico SDK (Cloud Agent)"
    echo "${PROFILE_SNIPPET}"
  } >> "${HOME}/.profile"
fi
# Also write a system-wide drop-in so non-login shells and other users see it.
if command -v sudo >/dev/null 2>&1; then
  echo "${PROFILE_SNIPPET}" | sudo tee /etc/profile.d/pico-sdk.sh >/dev/null
  sudo chmod 644 /etc/profile.d/pico-sdk.sh
fi
export PICO_SDK_PATH="${SDK_DIR}"

echo "[setup-pico] Ensuring picotool ${PICOTOOL_VERSION}..."
if command -v picotool >/dev/null 2>&1 && picotool version 2>/dev/null | grep -q "${PICOTOOL_VERSION}"; then
  echo "[setup-pico] picotool already installed: $(command -v picotool)"
else
  mkdir -p "$(dirname "${PICOTOOL_SRC_DIR}")"
  if [[ ! -f "${PICOTOOL_SRC_DIR}/CMakeLists.txt" ]]; then
    rm -rf "${PICOTOOL_SRC_DIR}"
    git clone --branch "${PICOTOOL_VERSION}" --depth 1 --recurse-submodules \
      https://github.com/raspberrypi/picotool.git "${PICOTOOL_SRC_DIR}"
  else
    git -C "${PICOTOOL_SRC_DIR}" submodule update --init --recursive
  fi
  PICOTOOL_BUILD="${PICOTOOL_SRC_DIR}/build"
  cmake -S "${PICOTOOL_SRC_DIR}" -B "${PICOTOOL_BUILD}" -GNinja \
    -DPICO_SDK_PATH="${SDK_DIR}"
  cmake --build "${PICOTOOL_BUILD}"
  sudo cmake --install "${PICOTOOL_BUILD}"
fi

# Init the pico_src/first pico-extras submodule when this repo is checked out.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "${REPO_ROOT}/pico_src/first/.gitmodules" ]] || [[ -f "${REPO_ROOT}/.gitmodules" ]]; then
  if [[ -d "${REPO_ROOT}/pico_src/first" ]]; then
    echo "[setup-pico] Initializing pico_src/first pico-extras submodule..."
    git -C "${REPO_ROOT}" submodule update --init --recursive -- pico_src/first/pico-extras || true
  fi
fi

echo "[setup-pico] Verifying toolchain..."
arm-none-eabi-gcc --version | head -1
cmake --version | head -1
ninja --version
test -f "${SDK_DIR}/pico_sdk_init.cmake"
picotool version || picotool help >/dev/null
echo "[setup-pico] Done. PICO_SDK_PATH=${SDK_DIR}"
