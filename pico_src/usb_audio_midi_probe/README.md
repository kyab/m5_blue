# USB Audio + MIDI + CDC probe

Trial firmware for **Pico 2 W** (`pico2_w`) that enumerates as three USB
functions at once:

1. **USB Audio** (UAC2 speaker, 16-bit stereo, 44.1 / 48 kHz) with explicit
   asynchronous feedback, played out over I2S.
2. **USB MIDI** that sends a **Note On / Note Off pair on C4 (note 60) every
   1 second**, and echoes any MIDI the host sends.
3. **USB CDC** that prints a status line about once a second (`probe_printf()`,
   equivalent to a periodic `printf()` on the serial console).

This is a bring-up sample. It is not wired into the M5Stack Core2 product
firmware under `src/`.

Verified connected and enumerating on **macOS** and **iOS**.

## Known issue: USB Audio glitches

USB Audio currently **glitches every few seconds** during playback (brief
dropouts / clicks).

## Hardware

- Raspberry Pi Pico 2 W
- [Pimoroni Pico Audio Pack](https://shop.pimoroni.com/products/pico-audio-pack)
  - I2S DATA = GP9
  - I2S BCK = GP10
  - I2S LRCK = GP11

## Build

Needs Pico SDK **2.3.1** and `pico-extras` (this repo uses
`pico_src/first/pico-extras`). `build/` is gitignored.

```sh
cd pico_src/usb_audio_midi_probe
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w
cmake --build build
```

Output: `build/tusb_probe.uf2`.

USB CDC is enabled, including **1200-baud BOOTSEL reset**. Copy the UF2 onto
the RP2350 mass-storage volume, or use `picotool load -f build/tusb_probe.uf2`
while the probe is already running.

## What you should see on the host

- A speaker named along the lines of **Pico USB Audio + MIDI + CDC** /
  **Pico Speaker**.
- A MIDI port **Pico MIDI**. A monitor should show C4 note on/off once a
  second even if you send nothing.
- A USB serial port. After opening it (DTR), a line like
  `mounted=1 cdc=1 alt=… rate=…` once a second.

`tools/probe_check.swift` can list CoreAudio / CoreMIDI devices on macOS.

This probe is built for **Apple hosts** (`CFG_PROBE_TARGET_APPLE=1`): Full-Speed
feedback is 3 bytes in 10.14 format. Windows typically wants 4-byte 16.16 and
may not enumerate this layout.

---

## Notes for agents

Investigation history and why the tree looks the way it does. Do not treat this
firmware as production-ready USB Audio.

### Why this sample exists

The product question was whether a Pico could present **USB Audio Class and USB
MIDI together**, with I2S output, while still offering a USB serial console for
logs. `pico_src/pico-playground/apps/usb_sound_card` (pico-extras `usb_device`)
is a speaker example but not TinyUSB and does not add MIDI + CDC. Working Pico
USB-audio projects (for example BambooMaster/pico_usb_i2s_speaker, TinyUSB
`uac2_speaker_fb`) use **TinyUSB UAC2**. This probe follows that stack.

### USB composite layout

TinyUSB’s audio driver and MIDI driver both sit under the Audio class. MIDI 1.0
must be a **separate IAD** with protocol `0x00`. UAC2 uses protocol `0x20`. If
those are merged into one Audio function, the host folds MIDI into the UAC2
streaming interface and one of the two fails.

Default interface map:

| Interfaces | Function |
| --- | --- |
| 0–1 | UAC2 speaker (control + streaming, alt 0 / alt 1) |
| 2–3 | USB MIDI 1.0 |
| 4–5 | CDC ACM (`pico_stdio_usb` + 1200-baud BOOTSEL) |

Audio OUT is ISO EP `0x01`, feedback IN is `0x81` (3 bytes on Apple FS), MIDI
is `0x02`/`0x82`, CDC is `0x83`/`0x04`/`0x84`. Custom descriptors are in
`src/usb_descriptors.c`; `PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS` is off
because `LIB_TINYUSB_DEVICE=1`. `tusb_init()` must run **before**
`stdio_init_all()`.

Clock feedback must track **I2S queue occupancy**, not the TinyUSB audio FIFO.
The FIFO is drained into `pico_audio_i2s` on each OUT completion, so it always
looks empty. `tud_audio_feedback_params_cb()` advertises
`AUDIO_FEEDBACK_METHOD_FREQUENCY_FIXED` only so the stack enables SOF;
`tud_audio_feedback_interval_isr()` then calls `tud_audio_fb_set()` from queue
depth.

### Panic that was fixed, glitch that remains

pico-sdk 2.3.1 ships TinyUSB **0.18**. On RP2040/RP2350, starting a new ISO
transfer while the previous buffer is still AVAILABLE panics:

```
*** PANIC ***
ep 01 was already available
```

That is TinyUSB issue [#2838](https://github.com/hathach/tinyusb/issues/2838) /
PR [#2937](https://github.com/hathach/tinyusb/pull/2937) (abort in-flight ISO
on SET_INTERFACE). This sample does not replace TinyUSB; it **linker-wraps**
`hw_endpoint_xfer_start` in `src/rp2040_hw_endpoint_fix.c`.

A second trigger was **nested `tud_task()`**: `pico_stdio_usb` calls `tud_task()`
from `printf()`. Status output therefore goes through `probe_printf()` /
`tud_cdc_write()` with LF→CRLF, not through `printf()` on the USB path.

After those changes, Audio + MIDI + CDC enumerate and run on macOS/iOS, but
**USB Audio still glitches every few seconds**. Do not assume the wrap or the
CDC path fixed underruns, feedback jitter, or ISO scheduling.

### Source map

| Path | Role |
| --- | --- |
| `CMakeLists.txt` | `pico2_w`, TinyUSB device, `pico_audio_i2s`, USB+UART stdio, `--wrap=hw_endpoint_xfer_start` |
| `src/main.c` | `tusb_init` / I2S / UAC2 callbacks / MIDI heartbeat / CDC status |
| `src/tusb_config.h` | UAC2 + MIDI + CDC; Apple 3-byte feedback |
| `src/usb_descriptors.c` / `.h` | Composite descriptors and IADs |
| `src/rp2040_hw_endpoint_fix.c` | TinyUSB 0.18 ISO AVAILABLE abort |
| `src/quirk_os_guessing.c` / `.h` | Optional host OS guess (default probe still pins Apple FB size) |
| `src/common_types.h` | Shared descriptor/debug types |
| `tools/probe_check.swift` | macOS CoreAudio / CoreMIDI helper |

Do not run PlatformIO (`pio`) on this tree. It is a Pico SDK CMake project.
Keep `pico_src/first/pico-extras` as the extras path unless CMake is pointed
elsewhere.
