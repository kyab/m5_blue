https://note.com/leftbank/n/n6aadbf1f9b11


## Build and Run

### Default app (A2DP + DJ filter, from `src/main.cpp`)

DJ filter control defaults to **Unit Joystick2** on Core2 **PORT.A** (I2C G32/G33):
hold the Z button and move the Y axis (top = LPF, center = bypass, bottom = HPF).
To restore the older Rotation angle unit (U005) on PORT.B, switch the defines in
`src/main.cpp` (`DJ_FILTER_CTRL_ROTATION_ANGLE` instead of `DJ_FILTER_CTRL_JOYSTICK2`).

```sh
pio run
pio run -t upload
pio device monitor
pio device monitor --rts 0 --dtr 0  # reset and monitor
```

### I2S noise-floor test (`src/main_noise_test.cpp`)

This env builds a minimal firmware that **disables BLE/A2DP/Wi-Fi** and plays
four 2-second segments on loop via pure I2S to the Module Audio ES8388:

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
