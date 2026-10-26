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
| `main.cpp` | `bluetooth/btstack_examples/main.c` | C++ entry; Joystick2 + effects/synth; `USING_I2C` audio-sink wiring kept |
| `btstack_audio_pico.c` | `bluetooth/btstack_examples/btstack_audio_pico.c` | Volume + `apply_effects_before_i2s` hook |
| `btstack_config.h` | `bluetooth/config/btstack_config_common.h` | Inlined here (upstream `a2dp_sink_demo/btstack_config.h` only `#include`s the common header) |
| `Synth.hpp` | (local) | Monophonic sine voice (`SYNTH` / `SYNTH_WITH_BUTTON` / `SYNTH_SOFTPOT`). SoftPot owner and pitch latch live in `main.cpp` |


## Prerequisites

- Pico SDK **2.3.1** at `$HOME/.pico-sdk/sdk/2.3.1` (with btstack submodule)
- Arm GNU Toolchain (e.g. `15_2_Rel1`) on `PATH` or via `PICO_TOOLCHAIN_PATH`
- CMake + Ninja; host `g++` able to link (needed to build `pioasm` / `picotool`)

```sh
git submodule update --init --recursive -- pico_src/first/pico-extras
```

## Build
Assuming Pico SDK is installed alongside with [Raspberry Pi Pico Visual Studio Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico).


```sh
cd pico_src/first
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH="$PWD/pico-extras"
cmake --build build --target a2dp_sink_demo
```

Outputs: `build/a2dp_sink_demo.uf2` / `.elf`.

## Flash (no BOOTSEL button)

USB stdio is enabled, and `PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=1` is set in
`CMakeLists.txt` (Pico SDK **2.3.1** name; the older
`PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE` still works as an alias). Opening the
board’s USB serial at the magic baud (**1200**) resets into BOOTSEL so you can
load a UF2 without holding BOOTSEL.

### Option A — `picotool` (preferred)

With the board already running this firmware (USB CDC enumerated):

```sh
# Force reboot into BOOTSEL, then load and reboot into the image
picotool load -f build/a2dp_sink_demo.uf2
# or: picotool load -f build/a2dp_sink_demo.elf && picotool reboot
```

`picotool load -f` uses the stdio USB reset path when the device is running;
if the board is already in BOOTSEL, a plain `picotool load …` is enough.

### Option B — open serial at 1200 baud, then copy UF2

```sh
# macOS example: find the CDC device, then open at 1200 (device reboots to BOOTSEL)
ls /dev/cu.usbmodem*
# Either of these triggers the magic baud reset:
stty -f /dev/cu.usbmodem* 1200   # adjust glob to the real device name
# or:  minicom -D /dev/cu.usbmodemXXXX -b 1200
```

After reset, a `RPI-RP2` (or RP2350) mass-storage volume appears; copy
`build/a2dp_sink_demo.uf2` onto it, or run `picotool load build/a2dp_sink_demo.uf2`.

### Verify baud-reset once

1. Flash any build that includes the define above (BOOTSEL once if needed).
2. Confirm USB serial is present (`ls /dev/cu.usbmodem*`).
3. Open the port at 1200 baud (or `picotool reboot -f` / `picotool load -f …`).
4. Board should leave the CDC port and appear as a UF2 drive / `picotool info` BOOTSEL device.

### Build mode (`PARTY_PICO_MODE`)

- **`SYNTH_WITH_BUTTON` (default)**: Mixes Synth into A2DP.  
  - Joystick2 Y → pitch zones, X → flat/sharp (±1 semitone);  
  - Dual Button **Blue** triggers noteOn/noteOff.
- **`SYNTH`**: Mixes Synth into A2DP.
  - Jyostick2 Y → pitch zones, X → flat/sharp (±1 semitone), Z triggers noteOn/noteOff.
- **`DJ`**: Effects on A2DP playback.
  - Joystick2 X → DJ Filter, Y → Freezer grain, Z → Freezer gate.
- **`SAMPLER`**: Record/play sampler mixed into A2DP.
  - Dual Button **Red** held = record, **Blue** = noteOn/noteOff. 
  - Joystick2 Y/X set relative pitch.
- **`SYNTH_SOFTPOT`**: SoftPot on GP26 (ADC0) maps one octave, C through B. Joystick2 is not used and I2C is not initialized. `SOFTPOT_TRIGGER_MODE` selects the gate (default `DIRECT`).
  - **`DIRECT`**: `adc <= 40` is release (`noteOff`). A touch (`adc >= 41`) calls `noteOn` at that zone. A zone change while touched restarts the attack at the new pitch and keeps oscillator phase. Buttons are logged and ignored.
  - **`BUTTON`**: Pitch is latched with `setSemitone` only on the rising edge that starts the note. The button that started the note is the owner: only that button's release calls `noteOff`. The other button is ignored for that note. If red and blue both rise on the same poll while idle, red wins. The release threshold is not used.

```sh
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w                         # default = SYNTH_WITH_BUTTON
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPARTY_PICO_MODE=SYNTH_WITH_BUTTON
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPARTY_PICO_MODE=SYNTH
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPARTY_PICO_MODE=DJ
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPARTY_PICO_MODE=SAMPLER
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPARTY_PICO_MODE=SYNTH_SOFTPOT
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPARTY_PICO_MODE=SYNTH_SOFTPOT -DSOFTPOT_TRIGGER_MODE=BUTTON
```

`SYNTH_SOFTPOT` without `SOFTPOT_TRIGGER_MODE` uses `DIRECT`. Any other value fails configure.

The A2DP application body is still taken from the Pico SDK BTstack tree
(`$PICO_SDK_PATH/lib/btstack/example/a2dp_sink_demo.c`), not from pico-examples.


#### Control matrix

| Input | `SYNTH_WITH_BUTTON` | `SYNTH` | `DJ` | `SAMPLER` | `SYNTH_SOFTPOT` |
| --- | --- | --- | --- | --- | --- |
| Joystick2 Y | pitch 9 zones | pitch 9 zones | Freezer grain | relative pitch | — |
| Joystick2 X | ♭ / ♮ / ♯ | ♭ / ♮ / ♯ | DJ Filter | ♭ / ♮ / ♯ | — |
| Joystick2 Z | read-only | trigger | Freezer gate | read-only | — |
| Dual Button Blue (GP7) | trigger | — | — | noteOn / noteOff | `BUTTON`: noteOn if it starts the note; noteOff only while it owns the note. `DIRECT`: ignored |
| Dual Button Red (GP6) | read-only | — | — | record while held | `BUTTON`: same as Blue; red wins if both rise together. `DIRECT`: ignored |
| SoftPot wiper (GP26) | — | — | — | — | `BUTTON`: pitch latched at note-on (C–B). `DIRECT`: touch gate and zone retrigger |

#### Dual Button wiring (`SYNTH_WITH_BUTTON`, `SAMPLER`, `SYNTH_SOFTPOT`)

| Dual Button (Grove) | Pico 2W |
| --- | --- |
| Red (VCC) | **3V3** (not 5V) |
| Black (GND) | GND |
| Yellow (Red btn) | **GP6** (read-only in `SYNTH_WITH_BUTTON`; record while held in `SAMPLER`) |
| White (Blue btn) | **GP7** (trigger / sampler noteOn-Off) |

Active-low; unit onboard 10 kΩ pull-ups to VCC. Firmware uses `GPIO_IN` only (no Pico internal pull-up). In `SYNTH_SOFTPOT` `BUTTON` mode, either button can start a note, and only the owner button's release stops it. In `DIRECT` mode the buttons are still read for the log and do not gate the voice.

#### SoftPot wiring (`SYNTH_SOFTPOT`)

| SoftPot | Pico 2W |
| --- | --- |
| pin 1 (arrow, connector-side bus, high note B) | **3V3** direct |
| pin 3 (no arrow, far-end bus, low note C) | **GND** direct |
| pin 2 (wiper) | **GP26** (ADC0), with 10 kΩ in parallel to GND |

No series resistors. No Pico internal pull on GP26. `adc_read` once per 2 ms poll; `zone = (adc * 12) / 4096` (0 = C … 11 = B). Far end and no-touch both fall in zone C. `DIRECT` treats `adc <= 40` as release, so a sounding note starts at `adc >= 41` (still C through 341).

## Data Structure

- BTStack RingBuffer
  1. sbc_frame_ring_buffer: SBC frames.
  2. decoded_audio_ring_buffer : Decoded PCM datas.

- Pico Audio Pool
  1. Producer Pool 
  audio_new_producer_pool(..., 3, SAMPLES_PER_BUFFER=512). 
  
  Application side is responsible for take->fill->give.
  Currently implemented in btstack_audio_pico_sink_fill_buffers(), fired every 5ms by BTStack timer.

  2. Consumer Pool 
  connect_extra(..., buffer_count=2, samples=256).

  ProducerとConsumerは接続されているが、それぞれがBuffer Poolをサイズ x 個数もつ。
