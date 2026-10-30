# USB DAC + MIDI + CDC (`pico_src/usb_dac`)

Pico 2 W firmware that presents:

1. **USB Audio (UAC1)** headphones, 48 kHz / 16-bit stereo, asynchronous ISO OUT
   with TinyUSB `AUDIO_FEEDBACK_METHOD_FIFO_COUNT`
2. **USB MIDI** device → host only: Dual Button red (GP6) and blue (GP7) each
   send C4 (note 60) note on/off, velocity 100, channel 1
3. **USB CDC** with `printf` via SDK `pico_stdio_usb`, including **1200-baud
   BOOTSEL reset** (`PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=1`)

Blue also triggers the local monophonic sine `Synth` (C4) on core1 so sound
continues when the host is not streaming. Host volume/mute applies to the mixed
output (−50…0 dB, 1 dB steps).

`pico_src/first` is reference-only and is not modified. TinyUSB **0.21.0** is
vendored as `lib/tinyusb`. I2S uses pico-extras `pico_audio_i2s`
(DATA=GP9, BCK=GP10, LRCK=GP11). MUTE (GP22) is left alone.

## Hardware

- Raspberry Pi Pico 2 W
- [Pimoroni Pico Audio Pack](https://shop.pimoroni.com/products/pico-audio-pack)
- Optional: M5Stack Unit Dual Button (Yellow/Red→GP6, White/Blue→GP7, VCC=3V3)

## Build

Needs Pico SDK **2.3.1**, Ninja, and `pico_src/first/pico-extras`.

```sh
cd pico_src/usb_dac
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$HOME/.pico-sdk/picotool/2.3.0/picotool:$PATH"
cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w
cmake --build build --target usb_dac
```

Output: `build/usb_dac.uf2`.

Flash (preferred, no BOOTSEL button when baud-reset firmware is already running):

```sh
picotool load -f build/usb_dac.uf2
```

## Host checks

- Audio device named like **Pico USB DAC + MIDI + CDC** / **Pico Headphones**,
  48 kHz only
- MIDI port **Pico MIDI**: red or blue → C4 on/off
- Serial: ~1 Hz status lines (`stream`, `fifo`, `loops`, underrun/overflow)
- Blue alone should produce C4 on the Audio Pack even with no host playback
