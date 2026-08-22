---
name: pio-workflow
description: Build, upload, monitor, and clean PlatformIO firmware through the `pio` CLI. After C/C++ edits, confirm with host syntax-only compile rather than a full firmware link. Use when the user says things like "確認", "コンパイル", "ビルドして", "書き込んで", "焼いて", "アップロードして", "モニタして", "クリーンして", "build", "flash", "upload", "monitor", or otherwise asks to compile/flash/observe firmware in a PlatformIO project.
---

# PlatformIO Workflow

Generic workflow for any PlatformIO project. The skill resolves the project root and the target environment from the repository instead of relying on hardcoded values.

## Host syntax-only check (default confirmation)

After C/C++ edits, do **not** run `pio run`. Confirm with a host `clang++` syntax-only compile of DSP headers (`DJFilter.hpp`, `RingBuffer.hpp`, and `Freezer.hpp` when present). This is ~1 s and does not link firmware or compile `main.cpp`.

```bash
STUB="$(mktemp -d)"
cat > "$STUB/esp_heap_caps.h" <<'EOF'
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#ifndef MALLOC_CAP_SPIRAM
#define MALLOC_CAP_SPIRAM (1 << 10)
#endif
inline void* heap_caps_malloc(size_t size, uint32_t) { return std::malloc(size); }
EOF
cat > "$STUB/esp_log.h" <<'EOF'
#pragma once
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGV(...) ((void)0)
EOF
clang++ -std=c++17 -fsyntax-only -Wall -Wextra -I src -I "$STUB" -x c++ - <<'EOF'
#include "DJFilter.hpp"
#include "RingBuffer.hpp"
#if __has_include("Freezer.hpp")
#include "Freezer.hpp"
#endif
EOF
rm -rf "$STUB"
```

`block_until_ms`: 15000.

Full `pio run -e ${ENV}` only when the user asks to build/flash, when Arduino/ESP-IDF APIs or `platformio.ini` change, or before a PR/flash. Do not run `pio check` unless asked. Format-only changes need no compile.

## Resolving Context

Before running any command, resolve two things:

### 1. Project root

The project root is the nearest ancestor directory (starting from the user's current working directory or the focused file) that contains a `platformio.ini`. Pass this path to the Shell tool via `working_directory`. Never `cd` inside the command string, and never hardcode an absolute path in the skill.

If `platformio.ini` cannot be found, stop and ask the user where the project lives instead of guessing.

### 2. Target environment (`-e <env>`)

PlatformIO `platformio.ini` can declare one or more `[env:<name>]` sections. Pick the environment as follows:

1. If the user names one explicitly ("core2環境でビルド", "build for esp32dev"), use that.
2. Otherwise read `platformio.ini` and list every `[env:<name>]` section.
   - If **exactly one** env exists → use it and pass `-e <name>`.
   - If **multiple** envs exist → ask the user which one, listing the options. Do not silently pick the first one.
3. If no `[env:*]` is defined, run `pio` without `-e` and let PlatformIO use its default.

Treat the chosen env as `${ENV}` in the command table below.

## Trigger → Command Mapping

| User intent (examples) | Command |
|---|---|
| "確認" / "コンパイル" / "syntax check" | Host syntax-only check above |
| "ビルドして" / "build" / "フルビルド" | `pio run -e ${ENV}` |
| "書き込んで" / "焼いて" / "flash" / "upload" / "アップロード" | `pio run -e ${ENV} -t upload` |
| "モニタして" / "シリアル見せて" / "monitor" | `pio device monitor -e ${ENV}` |
| "書き込んでモニタ" / "flash & monitor" | `pio run -e ${ENV} -t upload -t monitor` |
| "クリーンして" / "clean" | `pio run -e ${ENV} -t clean` |
| "フルクリーン" / "全部消して" / "fullclean" | `pio run -e ${ENV} -t fullclean` |

Omit `-e ${ENV}` only when no `[env:*]` sections exist in `platformio.ini`.

## Execution Rules

1. **Working directory**: Always invoke Shell with `working_directory` set to the resolved project root. Do not prefix the command with `cd`.
2. **CLI discovery**: Use `pio` from `PATH`. If it is missing, try `platformio`, then `~/.platformio/penv/bin/pio`. If none resolve, ask the user to install/activate PlatformIO instead of guessing a path.
3. **Timeouts (`block_until_ms`)**:
   - Clean / fullclean: 60000
   - Build (`pio run`): 300000 (large projects can exceed this on the first compile; if it times out, keep polling with the Await tool rather than killing it)
   - Upload: 300000 (includes build + flashing)
   - Monitor: run with `block_until_ms: 0` (background) because it never exits on its own
   - Upload + monitor: run with `block_until_ms: 0` (background) for the same reason
4. **Monitor handling**: `pio device monitor` is long-running. Start it in the background, then read the terminal file to show recent output. Tell the user how to stop it (`Ctrl+C`, or in the attached monitor `Ctrl+T` then `Q`). To reset the board while monitoring (e.g. to re-capture boot logs), use `Ctrl+T`, `Ctrl+R`, `Ctrl+T`, `Ctrl+R` (toggle RTS twice).
5. **Port**: Do not hardcode `--upload-port` / `--monitor-port`. Let PlatformIO auto-detect. Only add a port flag if the user explicitly names one or if auto-detection fails.
6. **After a build failure**: Surface the first compiler error with `file:line`, not the full log dump. Offer to fix it.
7. **After a successful build/upload**: Briefly report the RAM/Flash usage line if PlatformIO printed it, then ask whether to start the serial monitor (for upload).

## Examples

Assume the resolved project root is `<ROOT>` and the resolved env is `<ENV>`.

**Example 1 — "ビルドして"**

```bash
pio run -e <ENV>
```
Run via Shell with `working_directory: "<ROOT>"` and `block_until_ms: 300000`.

**Example 2 — "書き込んで"**

```bash
pio run -e <ENV> -t upload
```

**Example 3 — "書き込んでモニタも開いて"**

```bash
pio run -e <ENV> -t upload -t monitor
```
Start with `block_until_ms: 0` (background), then poll the terminal file with the Await tool to show the latest output.

**Example 4 — "クリーンしてビルド"**

Run sequentially in the same working directory:
```bash
pio run -e <ENV> -t clean && pio run -e <ENV>
```

**Example 5 — Multiple envs in `platformio.ini`**

If `platformio.ini` contains both `[env:esp32dev]` and `[env:m5stack-core2]`, and the user just says "ビルドして", ask:

> 複数の env が定義されています。どれでビルドしますか?
> - `esp32dev`
> - `m5stack-core2`

Then run the command with the chosen one.

## Anti-Patterns

- Do not `cd` inside the command string; use `working_directory`.
- Do not hardcode any absolute path (project root, `pio` binary path, etc.) in this skill or in commands.
- Do not assume a specific board/env. Resolve it from `platformio.ini` or ask.
- Do not invent ports; rely on auto-detection.
- Do not run the serial monitor in the foreground — it will hang the tool call.
- Do not run `pio run` or `pio check` as the default confirmation after an edit. Use the host syntax-only check.
