---
name: pio-workflow
description: Build, upload, monitor, and clean PlatformIO firmware through the `pio` CLI. Use when the user says things like "ビルドして", "書き込んで", "焼いて", "アップロードして", "モニタして", "クリーンして", "build", "flash", "upload", "monitor", or otherwise asks to compile/flash/observe firmware in a PlatformIO project.
---

# PlatformIO Workflow

Generic workflow for any PlatformIO project. The skill resolves the project root and the target environment from the repository instead of relying on hardcoded values.

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
| "ビルドして" / "build" / "コンパイル" | `pio run -e ${ENV}` |
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
4. **Monitor handling**: `pio device monitor` is long-running. Start it in the background, then read the terminal file to show recent output. Tell the user how to stop it (`Ctrl+C`, or in the attached monitor `Ctrl+T` then `Q`).
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
