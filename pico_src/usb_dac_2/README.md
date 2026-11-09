# usb_dac_2

Pico 2 W firmware that shows up on the host as a USB audio output (UAC1, 48 kHz / 16-bit stereo, headphones terminal), a USB MIDI output, and a CDC serial port. Audio is played through a Pimoroni Pico Audio Pack (I2S), mixed with a sine synth that runs independently of USB. Host USB audio can also be processed with the same DJ Filter and Freezer used by `pico_src/first` DJ mode.

## Hardware

| Part | Pins |
|---|---|
| Pimoroni Pico Audio Pack (I2S) | DATA = GP9, BCK = GP10, LRCK = GP11 (MUTE / GP22 is left alone) |
| M5Stack Dual Button unit | Red = GP6, Blue = GP7 (active low, external pull-up) |
| M5Stack Unit Joystick2 | Red = 3V3, Black = GND, Yellow (SCL) = GP5, White (SDA) = GP4 (I2C0, addr `0x63`, 100 kHz, Pico internal pull-ups) |

## Behavior

- **USB Audio**: the host stream goes Freezer → DJ Filter, then mixes with the Dual Button synth. Host volume and mute apply to the whole output (USB audio + synth). Range is -50 dB to 0 dB in 1 dB steps.
- **Joystick2** (same mapping as `pico_src/first` DJ mode): X = DJ Filter, Y = Freezer grain (1000–5000 samples), Z = Freezer gate. Core0 polls about every 2 ms; DSP runs on core1. If Joystick2 is missing, the filter stays at bypass and Freezer stays off.
- **USB MIDI** (device to host only): both buttons send C4 (note 60) note on / off on channel 1, velocity 100.
- **Synth**: the blue button triggers the `Synth` at C4 while it is held. It plays whether or not the host is streaming.
- **CDC**: `printf` output once per second (FIFO level, overflow / underrun counters, core1 loop rate), plus occasional Joystick2 status lines. Opening the port at 1200 baud reboots into BOOTSEL.

## Layout

```
src/main.cpp          core0 loop: tud_task, USB audio hand-off, MIDI, Joystick2 poll, status print
src/usb_audio.cpp     UAC1 callbacks, volume/mute, TinyUSB FIFO -> hand-off ring
src/audio_engine.cpp  core1: I2S, Freezer + DJ Filter on USB, synth, mix, master gain
src/usb_midi.cpp      button polling, note on/off
src/spsc_ring.h       lock-free ring between core0 and core1
src/Synth.hpp         copied from ../first with kSampleRateHz = 48000
src/DJFilter.hpp      copied from ../first with _fs = 48000
src/Freezer.hpp       copied from ../first (grain 1000–5000 samples)
src/RingBuffer.hpp    copied from ../first
src/joystick2.hpp     copied from ../first (I2C0 GP4/GP5)
lib/tinyusb           TinyUSB 0.21.0 (git submodule)
```

TinyUSB is only touched from core0; core1 reads the hand-off ring.

## Build and flash

Needs Pico SDK 2.3.1, `picotool` 2.3.0, and the submodules:

```bash
git submodule update --init pico_src/usb_dac_2/lib/tinyusb pico_src/first/pico-extras
cd pico_src/usb_dac_2
./rebuild_flash.sh
```

`rebuild_flash.sh` configures `build/` on first use, builds, then flashes.
This app sets `PICO_ENABLE_USB_RESET_VIA_BAUD_RATE` and turns
`PICO_ENABLE_USB_RESET_VIA_VENDOR_INTERFACE` off, so `picotool load -f` cannot
enter BOOTSEL (no Reset interface, VID `0xCafe`). The script opens the one
`/dev/cu.usbmodem*` port at 1200 baud once, waits for BOOTSEL, then runs
`picotool load -x`. If the board is already in BOOTSEL it skips the 1200 open.
After a successful load it waits until the CDC port is back. Do not leave that
port open at 1200.

## Host tools

macOS helpers live in `tools/`. Compile the Swift programs once:

```bash
swiftc -O -o play_sine tools/play_sine.swift
swiftc -O -o record tools/record.swift
```

`play_sine` and `record` run **at the same time**. Playback goes only to the output whose name contains `Pico USB DAC` (48 kHz stereo 1 kHz at -6 dBFS, same tone on both channels). Recording is the USB PnP input, written to a 48 kHz stereo 16-bit WAV while it captures. The shell waits on `record`; `play_sine` is in the background. Scan after both have finished.

```bash
./play_sine 1800 300 &
./record capture.wav 1800 300
python3 tools/scan_wav.py capture.wav glitches.txt
```

Open CDC at 115200 only. 1200 baud reboots into BOOTSEL.

### Arguments

`play_sine <seconds> [progress_sec]`

| Argument | Meaning |
|---|---|
| `seconds` | How long to play. `1800` is 30 minutes. |
| `progress_sec` | Print elapsed time every N seconds. Default is 60. `300` is every 5 minutes. |

`record <wav> <seconds> [progress_sec]`

| Argument | Meaning |
|---|---|
| `wav` | Output path. The file is rewritten from the start. |
| `seconds` | How long to record. |
| `progress_sec` | Print elapsed time and frames written every N seconds. Default is 60. |

`scan_wav.py <in.wav> <out.txt>`

| Argument | Meaning |
|---|---|
| `in.wav` | 48 kHz stereo 16-bit recording to check. |
| `out.txt` | Report path. The same text is also printed. |

The scan flags 5 ms silence more than 20 dB below the median, sample steps above 2.5× the steady 1 kHz slope, additive bursts left after removing that sine, and full-scale samples. The first and last 0.5 s are edges. Pass/fail uses the rest. Exit status is 0 on pass and 2 on fail.

## What to check on hardware

1. The device enumerates with Audio, MIDI, and Serial on macOS and iOS (Audio + MIDI), and on Windows.
2. The host volume slider and mute change the output level.
3. Play a 1 kHz sine for 30 minutes: `fifo[min,max]` stays near half of the 1176-byte FIFO, `full` and `underruns` stay at 0 after start-up.
4. Hold the blue button with the host idle: C4 (about 261.6 Hz) sounds, and keeps sounding across host play / stop.
5. A MIDI monitor shows C4 note on / off from both buttons.
6. `core1 loops/s` stays at about 1000.
7. With Joystick2 connected, host playback responds to X (filter), Y (grain), and Z (Freezer gate); Dual Button blue still triggers C4 with or without USB audio.
