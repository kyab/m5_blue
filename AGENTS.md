# AGENTS.md

Project-wide guidance for AI coding agents working in this repository.

## Going-Zero Reference

Whenever the user asks to reference, port, match, or debug code based on **Going-Zero** (including but not limited to the DJ filter, other audio DSP blocks, UI behaviors, or any other component), consult the Going-Zero source on GitHub directly instead of guessing or relying on memory.

Note: Going-Zero is a separate, independent repository — this project is not a fork of it. Treat it as an external reference.

- Reference repository: https://github.com/kyab/Going-Zero

How to reference:

- Prefer fetching the specific file from the Going-Zero repository on GitHub (raw URL via `WebFetch`, or the GitHub MCP) before writing or modifying related code in this repo.
- When porting, keep behavior aligned with the reference implementation (coefficients, formulas, constants, parameter mappings, state machines, fade/anti-click handling, etc.).

Triggers that should cause the agent to consult Going-Zero on GitHub:

- The user says "Going-Zero を参考にして" / "Going-Zero と同じにして" / "Going-Zero から移植して" or similar.
- Verifying that ported code (DJ filter, IIR filters, fader, etc.) matches the original.
- Adding a new effect or feature that already exists in Going-Zero.
- Debugging tonal, amplitude, or timing differences vs the original app.

## Coding Style

- Follow the project root `.clang-format` for C/C++ code (see also `.cursor/rules/clang-format-style.mdc`).
- Source code comments: English.
- Documentation (README.md etc.): English.
- Chat responses to the user: Japanese (per user preference).

## Git branches

When creating a new branch, use one of:

- `feat/<short-name>` — new feature
- `fix/<short-name>` — bug fix
- `docs/<short-name>` — documentation
- `chore/<short-name>` — other small updates

If the prefix is unclear, ask before creating the branch.

## Pull requests

When creating or updating pull requests for this repository (for example with `gh pr create` or `gh pr edit`):

- Write the **title and description/body in English**, consistent with README and other published documentation here.
- Keep the description as simple or detailed as the change requires; language stays English unless a maintainer explicitly chooses otherwise.

## Plots and figures

When adding or updating plots, charts, or other data graphics in this repository (for example in notebooks, scripts, or documentation figures):

- **Color**: **Prioritize** a **universal color palette** (universal-design / colorblind-friendly schemes) so series remain distinguishable for readers with color-vision deficiencies. Well-known references include [Paul Tol’s notes](https://personal.sron.nl/~pault/) and the Okabe–Ito palette; match the toolchain you use (e.g. Matplotlib, Plotly) to an explicitly CVD-safe set rather than default rainbow or highly saturated-only schemes.
- **More than color**: Do not rely on hue alone. Combine color with **line style** (e.g. solid, dashed, or dotted), **line width**, and **marker shape** (or distinct fill patterns where applicable) so each series or category stays identifiable in grayscale print and for low-vision readers.

## Build / Flash

- Use PlatformIO (`pio`) for building, uploading, and monitoring. See `.agents/skills/pio-workflow/SKILL.md`.
- After C/C++ edits, confirm with a **host syntax-only** compile (`clang++ -std=c++17 -fsyntax-only`), not `pio run`. Full firmware build only when the user asks to build/flash, when Arduino/ESP-IDF APIs or `platformio.ini` change, or before a PR/flash. Do not run `pio check` unless asked.

## Cursor Cloud Agent (environment setup)

This repo uses **git submodules** (`ESP32-A2DP`, `Module-Audio`). Cloud Agent VMs need them checked out before `pio run` can succeed (e.g. missing `audio_i2c.hpp` from Module-Audio).

- **Primary**: `.cursor/environment.json` runs `git submodule update --init --recursive` in the `install` step alongside PlatformIO setup. See [Cursor Cloud Agent setup](https://cursor.com/docs/cloud-agent/setup) for how `install` / Dockerfile-based environments work.
- **Fallback**: If submodules are still missing (e.g. shallow clone or interrupted install), run `git submodule update --init --recursive` from the repository root before building.

There is no separate product named "Cursor web env setup agent" in the docs; environment provisioning is described as **onboarding at [cursor.com/onboard](https://cursor.com/onboard)** (optional snapshot) **or** **repository-defined setup** via `.cursor/environment.json` (and optionally a Dockerfile). Prefer keeping submodule + toolchain steps in `environment.json` / `AGENTS.md` rather than ad-hoc chat instructions.

## Default PlatformIO Source Target

- Unless the user explicitly specifies another environment or file, treat the source scope as the default PlatformIO environment in `platformio.ini` (`[platformio] default_envs`).
- In this repository, the default environment is `m5stack-core2`, so prioritize edits and verification against that environment's effective source set (`build_src_filter = +<*> -<main_noise_test.cpp>`).
- Only switch to other environment-specific sources (for example `main_noise_test.cpp`) when the user explicitly requests that target.

## Cursor Cloud specific instructions

- **First build is slow (~95 s)**: PlatformIO downloads the ESP32 toolchain and compiles the entire Arduino framework on the first `pio run`. Subsequent incremental builds are fast (~5–10 s for source-only changes).
- **No upload/monitor in Cloud VMs**: `pio run -t upload` and `pio device monitor` require a physical M5Stack Core2 over USB. In Cloud Agent VMs the "hello world" validation is a successful `pio run` (compilation produces `firmware.bin`).
- **Static analysis**: `pio check -e m5stack-core2 --skip-packages` runs cppcheck on project source. One pre-existing "high" in a third-party library (`M5GFX`) is expected and not actionable.
- **Build commands** (see also `.agents/skills/pio-workflow/SKILL.md` and `README.md`):
  - Default target: `pio run` (or `pio run -e m5stack-core2`)
  - Noise-test target: `pio run -e m5stack-core2-noise-test`
  - Clean: `pio run -e m5stack-core2 -t clean`

## Known Hardware Notes

- **Module Audio (ES8388) left-channel hiss**: On the board investigated,
  audible hiss was tied to the TRRS MIC path coupling through the mixer while
  mixer bypass was enabled (a floating MIC line can inject noise). Firmware
  mitigates this by turning mixer bypass off: ES8388 `DACCONTROL17` and
  `DACCONTROL20` are programmed in `src/main.cpp` so LI2LO and RI2RO are
  cleared (no line-in to mixer bypass route). Residual LOUT1 hiss or L/R
  asymmetry vs ROUT1 may still occur on other Module Audio units or if that
  routing is changed; see `docs/Module-Audio-LOUT1-hiss-investigation.md`
  (Japanese) for the original evidence and measurements.
