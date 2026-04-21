# AGENTS.md

Project-wide guidance for AI coding agents working in this repository.

## Going-Zero Reference

Whenever the user asks to reference, port, match, or debug code based on **Going-Zero** (including but not limited to the DJ filter, other audio DSP blocks, UI behaviors, or any other component), consult the Going-Zero source on GitHub directly instead of guessing or relying on memory.

Note: Going-Zero is a separate, independent repository — this project is not a fork of it. Treat it as an external reference.

- Reference repository: https://github.com/kyab/Going-Zero

How to reference:

- Prefer fetching the specific file from the Going-Zero repository on GitHub (raw URL via `WebFetch`, or the GitHub MCP) before writing or modifying related code in this repo.
- When porting, keep behavior aligned with the reference implementation (coefficients, formulas, constants, parameter mappings, state machines, fade/anti-click handling, etc.).
- If a deliberate divergence from Going-Zero is introduced (e.g. for M5Stack constraints), note the reason in a code comment near the change and in the commit message.

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

## Build / Flash

- Use PlatformIO (`pio`) for building, uploading, and monitoring. See `.agents/skills/pio-workflow/SKILL.md`.

## Known Hardware Notes

- **Module Audio (ES8388) left-channel hiss**: the LOUT1 output on the Module
  Audio unit currently in use produces audible hiss whenever the DAC is
  modulating (i.e. any sample is non-zero). ROUT1 is clean. Evidence and the
  ruling-out of software causes are documented in
  `docs/Module-Audio-LOUT1-hiss-investigation.md` (Japanese). Treat this as a
  hardware-side asymmetry of the specific board; do not spend further effort
  chasing it in firmware unless the investigation document is revisited.
