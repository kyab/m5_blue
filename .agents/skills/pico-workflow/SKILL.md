---
name: pico-workflow
description: >-
  Build and flash Raspberry Pi Pico / Pico 2 W firmware under pico_src with
  CMake+Ninja and picotool. Prefers USB 1200-baud BOOTSEL reset
  (PICO_ENABLE_USB_RESET_VIA_BAUD_RATE) so flashing needs no BOOTSEL button.
  Use when the user works in pico_src (e.g. pico_src/first), or says things
  like "書き込んで", "焼いて", "flash", "upload", "ビルドして", "build" for a
  Pico CMake project — not PlatformIO / M5Stack (use pio-workflow for those).
---

# Pico Workflow

CMake + Ninja + `picotool` for first-party trees under `pico_src/`. Prefer this
over PlatformIO when the focused path or request is a Pico project.

## Pico SDK version

Always use **Pico SDK 2.3.1**:

```bash
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
```

- Do not use 2.3.0 or other SDK trees unless the user explicitly requests a different version.
- If `PICO_SDK_PATH` is already set to something else, override it to 2.3.1 for this workflow.
- Prefer app `CMakeLists.txt` / VS Code extension lines with `sdkVersion 2.3.1`. If an app still says `2.3.0`, still build with SDK 2.3.1 via `PICO_SDK_PATH` unless configure fails — then ask before changing the file.
- `pico-extras` for these apps should match the **sdk-2.3.1** tag when the project vendors it.
- **picotool 2.3.0** under `$HOME/.pico-sdk/picotool/2.3.0/` is the companion CLI (not the SDK version); keep using that path.

## When to use which skill

| Context | Skill |
|---|---|
| `platformio.ini` / M5Stack / ESP32 | `pio-workflow` |
| `pico_src/<app>/CMakeLists.txt` + Pico SDK | this skill |

If both could apply, pick from the focused file / cwd. Ask only when still ambiguous.

## Resolving Context

### 1. Project root

Nearest ancestor of the focused file / cwd that:

- contains `CMakeLists.txt`, and
- is under `pico_src/`, and
- is a **first-party app** (has its own `pico_sdk_import.cmake`, or is clearly the app the user named).

Default when unclear: `pico_src/first`.

Do **not** treat vendored trees as the app root unless the user names them:

- `pico_src/pico-examples`, `pico_src/pico-extras`, `pico_src/pico-playground`
- `pico_src/first/pico-extras`

Pass the app root to Shell as `working_directory`. Do not `cd` inside the command string.

### 2. Executable / UF2 name

1. User names a target → use it.
2. Else read `add_executable(<name> …)` from that app’s `CMakeLists.txt` (e.g. `a2dp_sink_demo`, `tusb_probe`, `blink`).
3. Else `ls build/*.uf2` and use the single match; if multiple, ask.

Artifacts: `build/<name>.uf2` / `build/<name>.elf`.

### 3. Board / mode

- Board: `-DPICO_BOARD=pico2_w` unless the user or `CMakeLists.txt` cache says otherwise.
- `PARTY_PICO_MODE` (`pico_src/first` only): use the user’s value if given; otherwise keep the existing CMake cache. Reconfigure only when the user changes mode or `build/` is missing.

## Environment

Before cmake / picotool, force SDK 2.3.1 and toolchain PATH:

```bash
# If present (also defaults PICO_SDK_PATH to 2.3.1):
. ./exports.sh
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"
```

If `exports.sh` is missing:

```bash
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"
```

USB flash needs host device access: Shell `required_permissions: ["all"]`.

## Trigger → Command Mapping

| User intent | Action |
|---|---|
| "ビルドして" / "build" | Configure (if needed) + `cmake --build build --target <name>` |
| "書き込んで" / "焼いて" / "flash" / "upload" | Ensure UF2 exists (build if missing/stale) + `picotool load -f build/<name>.uf2` |
| "ビルドして書き込んで" | Build then flash |
| "クリーンして" / "clean" | `cmake --build build --target clean` (or remove `build/` only if user asks for full clean) |

## Build

```bash
. ./exports.sh
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"

# Configure when build/ is missing or user changed -D options:
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w
# pico_src/first + pico-extras submodule:
# cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH="$PWD/pico-extras"
# Optional: -DPARTY_PICO_MODE=SAMPLER|SYNTH|DJ|SYNTH_WITH_BUTTON

cmake --build build --target <name>
```

`block_until_ms`: 300000 for first/full builds; 120000 is usually enough incremental.

Submodules: if configure fails on missing `pico-extras` / SDK pieces, run
`git submodule update --init --recursive -- <path>` for that app only, then retry.

## Flash (no BOOTSEL button)

Preferred path when firmware was built with `PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=1`
(and USB stdio enabled): `picotool load -f` opens CDC at magic baud **1200**,
reboots to BOOTSEL, loads UF2, reboots to app.

```bash
. ./exports.sh
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"
picotool load -f build/<name>.uf2
```

Rules:

1. **Do not** gate flash on `picotool info` — while the app is running, `info` fails and `&&` skips `load`.
2. Already in BOOTSEL → `picotool load build/<name>.uf2` ( `-f` still OK).
3. After load, `ERROR: … rebooting` is often benign if the progress bar reached 100%. Confirm CDC returns (`ls /dev/cu.usbmodem*`) after ~2s.
4. Optional verify (disruptive; reboots via `-f`): `picotool info -f` should show program `name: <name>`. Prefer CDC reappearance for routine checks.
5. No CDC and not in BOOTSEL → tell the user to hold BOOTSEL once, or use Option B below.

### Fallback — 1200 baud then load

```bash
ls /dev/cu.usbmodem*
stty -f /dev/cu.usbmodemXXXX 1200   # real device name
# wait for RPI-RP2 / RP2350 volume or BOOTSEL device, then:
picotool load build/<name>.uf2
```

## Execution Rules

1. `working_directory` = resolved Pico app root.
2. Flash / device probes: `required_permissions: ["all"]`.
3. After success: report UF2 path, flash OK, and whether CDC reappeared. Keep it short.
4. After build failure: first compiler error with `file:line`, not a full log dump.
5. Do not hardcode machine-specific absolute project paths in commands; `$HOME/.pico-sdk/...` toolchain paths from this skill are OK.
6. Always build against Pico SDK **2.3.1** (`PICO_SDK_PATH=$HOME/.pico-sdk/sdk/2.3.1`).

## Examples

**"pico_src/first を書き込んで"** (UF2 already built)

```bash
. ./exports.sh
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"
picotool load -f build/a2dp_sink_demo.uf2
```

**"SAMPLER でビルドして焼いて"**

```bash
. ./exports.sh
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH="$PWD/pico-extras" -DPARTY_PICO_MODE=SAMPLER
cmake --build build --target a2dp_sink_demo
picotool load -f build/a2dp_sink_demo.uf2
```

## Anti-Patterns

- Do not use `pio run` / `pio-workflow` for these Pico CMake apps.
- Do not build with Pico SDK 2.3.0 (or any non-2.3.1) unless the user asks.
- Do not require the physical BOOTSEL button when baud-reset firmware is already on the board.
- Do not `picotool info && picotool load -f …`.
- Do not flash vendored example trees by default.
- Do not invent serial port names; list `/dev/cu.usbmodem*` when needed.
