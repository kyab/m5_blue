https://note.com/leftbank/n/n6aadbf1f9b11


## Build and Run

### Default app (A2DP + DJ filter, from `src/main.cpp`)

```sh
pio run -e m5stack-core2
pio run -e m5stack-core2 -t upload
pio device monitor -e m5stack-core2
pio device monitor -e m5stack-core2 --rts 0 --dtr 0  # reset and monitor
```

### I2S noise-floor test (`src/main_noise_test.cpp`)

This env builds a minimal firmware that **disables BLE/A2DP/Wi-Fi** and plays
four 2-second segments on loop via pure I2S to the Module Audio ES8388:

1. `SINE_A`   — 1 kHz sine, amplitude 0.15
2. `ZERO`     — PCM all zero (DAC/HP path kept alive, no `setDACmute`)
3. `SINE_B`   — 1 kHz sine again
4. `DITHER`   — +-1 LSB dither (safety against any undocumented zero-detect mute)

It is designed for A/B comparison of the hiss level between:

- Core2 powered via **USB cable** (host PC / charger)
- Core2 powered by the **M5GO Bottom2 battery** (USB unplugged)

Each segment transition is logged to serial, along with `M5.Power` state
(`charging=`, battery level, battery voltage), and the current segment is
drawn on the LCD so you can flip USB on/off while recording audio.

```sh
pio run -e m5stack-core2-noise-test
pio run -e m5stack-core2-noise-test -t upload
pio device monitor -e m5stack-core2-noise-test
```

Notes:

- BT/Wi-Fi are disabled in `disable_radio()` via `esp_wifi_stop/deinit` and
  `esp_bt_controller_disable/deinit/mem_release(BTDM)` so RF activity does
  not contaminate the measurement.
- The ES8388 DAC mute bit is explicitly **left disabled** (`DACMute`=0) while
  the soft-ramp bit (`DACSoftRamp`=1) is set. This matches the intent of the
  test: observe the noise floor of a *live* DAC/HP output under "digital zero"
  input rather than a hardware-muted output.
