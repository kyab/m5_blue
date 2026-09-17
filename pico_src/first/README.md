# Pico 2 W A2DP sink (`a2dp_sink_demo`)

Self-contained firmware based on raspberrypi/pico-examples
`bluetooth/btstack_examples/a2dp_sink_demo`, with I2S pins set for the
Pimoroni Pico Audio Pack (DATA=GP9, BCK=GP10, LRCK=GP11). MUTE (GP22) is
unused by this demo and is not wired in firmware.

`pico-extras` is a git submodule at tag `sdk-2.3.1` (same commit as `sdk-2.3.0`).

## Upstream source (pico-examples)

Vendored glue files come from [raspberrypi/pico-examples](https://github.com/raspberrypi/pico-examples)
at tag **`sdk-2.3.1`**
([`0d62f75bafc2c8120d3276c3343d1a9195e909e9`](https://github.com/raspberrypi/pico-examples/commit/0d62f75bafc2c8120d3276c3343d1a9195e909e9)).
The same file contents are also present at tag `sdk-2.3.0`.

| This tree | Upstream path | Notes |
| --- | --- | --- |
| `main.c` | `bluetooth/btstack_examples/main.c` | Locally trimmed (WiFi / iperf paths removed); `USING_I2C` audio-sink wiring kept |
| `btstack_audio_pico.c` | `bluetooth/btstack_examples/btstack_audio_pico.c` | Unchanged vs upstream |
| `btstack_config.h` | `bluetooth/config/btstack_config_common.h` | Inlined here (upstream `a2dp_sink_demo/btstack_config.h` only `#include`s the common header) |

The A2DP application body is still taken from the Pico SDK BTstack tree
(`$PICO_SDK_PATH/lib/btstack/example/a2dp_sink_demo.c`), not from pico-examples.

## Prerequisites

- Pico SDK **2.3.1** at `$HOME/.pico-sdk/sdk/2.3.1` (with btstack submodule)
- Arm GNU Toolchain (e.g. `15_2_Rel1`) on `PATH` or via `PICO_TOOLCHAIN_PATH`
- CMake + Ninja; host `g++` able to link (needed to build `pioasm` / `picotool`)

```sh
git submodule update --init --recursive -- pico_src/first/pico-extras
```

## Build

```sh
cd pico_src/first
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PICO_TOOLCHAIN_PATH="$HOME/.pico-sdk/toolchain/15_2_Rel1"
export PATH="$PICO_TOOLCHAIN_PATH/bin:$PATH"

cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w \
  -DPICO_EXTRAS_PATH="$PWD/pico-extras"
cmake --build build --target a2dp_sink_demo
```

Outputs: `build/a2dp_sink_demo.uf2` / `.elf`.
