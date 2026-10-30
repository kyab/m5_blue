# usb_dac_2

Pico 2 W firmware that shows up on the host as a USB audio output (UAC1, 48 kHz / 16-bit stereo, headphones terminal), a USB MIDI output, and a CDC serial port. Audio is played through a Pimoroni Pico Audio Pack (I2S), mixed with a sine synth that runs independently of USB.

## Hardware

| Part | Pins |
|---|---|
| Pimoroni Pico Audio Pack (I2S) | DATA = GP9, BCK = GP10, LRCK = GP11 (MUTE / GP22 is left alone) |
| M5Stack Dual Button unit | Red = GP6, Blue = GP7 (active low, external pull-up) |

## Behavior

- **USB Audio**: the host volume and mute apply to the whole output (USB audio + synth). Range is -50 dB to 0 dB in 1 dB steps.
- **USB MIDI** (device to host only): both buttons send C4 (note 60) note on / off on channel 1, velocity 100.
- **Synth**: the blue button triggers the `Synth` at C4 while it is held. It plays whether or not the host is streaming.
- **CDC**: `printf` output once per second (FIFO level, overflow / underrun counters, core1 loop rate). Opening the port at 1200 baud reboots into BOOTSEL.

## Layout

```
src/main.cpp          core0 loop: tud_task, USB audio hand-off, MIDI, status print
src/usb_audio.cpp     UAC1 callbacks, volume/mute, TinyUSB FIFO -> hand-off ring
src/audio_engine.cpp  core1: I2S, synth, mix, master gain
src/usb_midi.cpp      button polling, note on/off
src/spsc_ring.h       lock-free ring between core0 and core1
src/Synth.hpp         copied from ../first with kSampleRateHz = 48000
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

`rebuild_flash.sh` configures `build/` on first use, builds, and flashes with `picotool load -f` (1200-baud reset, no BOOTSEL button needed once the firmware is running).

## What to check on hardware

1. The device enumerates with Audio, MIDI, and Serial on macOS and iOS (Audio + MIDI), and on Windows.
2. The host volume slider and mute change the output level.
3. Play a 1 kHz sine for 30 minutes: `fifo[min,max]` stays near half of the 1176-byte FIFO, `full` and `underruns` stay at 0 after start-up.
4. Hold the blue button with the host idle: C4 (about 261.6 Hz) sounds, and keeps sounding across host play / stop.
5. A MIDI monitor shows C4 note on / off from both buttons.
6. `core1 loops/s` stays at about 1000.
