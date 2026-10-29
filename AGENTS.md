# AGENTS.md

Project-wide guidance for AI coding agents working in this repository.

## ADR (Architecture Decision Record)
Refer docs/ADR.md
For coding, don't repeat ADR mentioned things in comments.

## Agent chat/conversation
### 質問と確認
**「不明な点」「曖昧な要件」「複数の選択肢」がある状態での推測によるコード生成や実装を禁止します。**
手戻りを防ぎ、正確な成果物を出すために、不確実な要素がある場合は必ず実装をストップし、ユーザーに質問・確認を行ってください。
確認の質問の際には、あなたにおまかせするといった選択肢も用意して。その場合は適時適切な推測や仮説をたててすすめて。

### 回答の規約
**回答の最後に必ず使用したモデルを次の形式で明記して**
回答 by : <モデル名>


## Coding Style

- Follow the project root `.clang-format` for C/C++ code (see also `.cursor/rules/clang-format-style.mdc`).
- Source code comments: English.
- Documentation (README.md etc.): English.
- Chat responses to the user: Japanese (per user preference).
- Japanese docs / plans / chat wording (and related vocabulary such as trigger vs gate): follow `.cursor/rules/japanese-wording.mdc`.

### Source code comments
コメントは、コードや名前から読み取れない設計上の理由・仕様・不変条件・制約・注意点があるときだけ、簡潔に書いてください。動作の言い換えや、変数名・関数名・型名・構造から明らかな内容は書かないでください（例: `y_per_zone` があるなら「zone あたりの Y 幅で割っている」は不要）。追加・変更時は、コメントとコードの内容が一致していることを確認してください。

## Git branches

When creating a new branch, use one of:

- `feat/<short-name>` — new feature
- `fix/<short-name>` — bug fix
- `docs/<short-name>` — documentation
- `chore/<short-name>` — other small updates
- `refactor/<short-name>` - refactoring

If the prefix is unclear, ask before creating the branch.

## Pull requests

When creating or updating pull requests for this repository (for example with `gh pr create` or `gh pr edit`):

- Write the **title and description/body in English**, consistent with README and other published documentation here.
- Keep the description as simple or detailed as the change requires; language stays English unless a maintainer explicitly chooses otherwise.

## Plots and figures

When adding or updating plots, charts, or other data graphics in this repository (for example in notebooks, scripts, or documentation figures):

- **Color**: **Prioritize** a **universal color palette** (universal-design / colorblind-friendly schemes) so series remain distinguishable for readers with color-vision deficiencies. Well-known references include [Paul Tol’s notes](https://personal.sron.nl/~pault/) and the Okabe–Ito palette; match the toolchain you use (e.g. Matplotlib, Plotly) to an explicitly CVD-safe set rather than default rainbow or highly saturated-only schemes.
- **More than color**: Do not rely on hue alone. Combine color with **line style** (e.g. solid, dashed, or dotted), **line width**, and **marker shape** (or distinct fill patterns where applicable) so each series or category stays identifiable in grayscale print and for low-vision readers.

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

## Build / Flash

- Use PlatformIO (`pio`) for building, uploading, and monitoring. See `.agents/skills/pio-workflow/SKILL.md`.
- For Pico / Pico 2 W apps under `pico_src/` (CMake + Ninja + `picotool`, including 1200-baud USB BOOTSEL reset), see `.agents/skills/pico-workflow/SKILL.md`.
- After C/C++ edits, confirm with a **host syntax-only** compile (`clang++ -std=c++17 -fsyntax-only`), not `pio run`. Full firmware build only when the user asks to build/flash, when Arduino/ESP-IDF APIs or `platformio.ini` change, or before a PR/flash. Do not run `pio check` unless asked.

## Cursor Cloud Agent (environment setup)

Cloud Agent VMs are configured for **Pico** firmware under `pico_src/` (not PlatformIO / M5Stack).

- **Primary**: `.cursor/environment.json` runs `.cursor/setup-pico.sh`, which installs Arm GCC, CMake/Ninja, Pico SDK **2.3.1**, and picotool **2.3.0**, and initializes `pico_src/first/pico-extras`. See [Cursor Cloud Agent setup](https://cursor.com/docs/cloud-agent/setup).
- **SDK path**: `PICO_SDK_PATH=$HOME/.pico-sdk/sdk/2.3.1` (also written to `/etc/profile.d/pico-sdk.sh`). Prefer `pico_src/first/exports.sh` when building that app.
- **Smoke check**: configure and build `pico_src/first` (`a2dp_sink_demo`) with `-DPICO_BOARD=pico2_w` and `-DPICO_EXTRAS_PATH=$PWD/pico-extras`. Cloud VMs cannot flash USB devices; a successful `.uf2` is enough.

There is no separate product named "Cursor web env setup agent" in the docs; environment provisioning is described as **onboarding at [cursor.com/onboard](https://cursor.com/onboard)** (optional snapshot) **or** **repository-defined setup** via `.cursor/environment.json` (and optionally a Dockerfile). Prefer keeping toolchain steps in `environment.json` / `AGENTS.md` rather than ad-hoc chat instructions.

## Default PlatformIO Source Target

- Unless the user explicitly specifies another environment or file, treat the source scope as the default PlatformIO environment in `platformio.ini` (`[platformio] default_envs`).
- In this repository, the default environment is `m5stack-core2`, so prioritize edits and verification against that environment's effective source set (`build_src_filter = +<*> -<main_noise_test.cpp>`).
- Only switch to other environment-specific sources (for example `main_noise_test.cpp`) when the user explicitly requests that target.

## Cursor Cloud specific instructions

- **First environment install is slow**: `setup-pico.sh` clones the Pico SDK (recursive) and builds picotool once into the snapshot. Later agent boots from a successful environment build skip `install`.
- **No USB flash in Cloud VMs**: `picotool load` needs a physical Pico. Validate with a successful CMake build that produces `build/<target>.uf2`.
- **Build commands** (see `.agents/skills/pico-workflow/SKILL.md`):
  - `cd pico_src/first && . ./exports.sh`
  - `cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH="$PWD/pico-extras"`
  - `cmake --build build --target a2dp_sink_demo`

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
