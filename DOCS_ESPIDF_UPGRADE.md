# ESP-IDF version and upgrading to 5.x

## Why the current setup is "old"

- **platform** = `espressif32 @ 6.8.1` installs a specific **Arduino-ESP32** framework (e.g. 3.20017).
- That framework is built against **ESP-IDF 4.4.7** (see `esp_idf_version.h` in the framework package).
- PlatformIO’s espressif32 platform keeps the **Arduino** core tied to IDF 4.4.x in the default packages; even newer platform versions (e.g. 6.12) still ship Arduino on 4.4.7.
- So with the current `platform = espressif32 @ 6.8.1` and `framework = arduino`, you get **IDF 4.4.7** and **Legacy I2S** (`i2s_write`). That’s why it feels “old”.

## Can we use a newer ESP-IDF?

Yes. You have two main options.

### Option A: Arduino + ESP-IDF 5 via `platform_packages` (experimental)

Force a **newer Arduino-ESP32** that is built on IDF 5.x (e.g. 3.0.x from the official repo):

```ini
[env:m5stack-core2_idf5]
platform = espressif32 @ 6.8.1
board = m5stack-core2
framework = arduino
platform_packages =
    platformio/framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.0.7
```

- Replace `#3.0.7` with a tag or branch that matches an Arduino-ESP32 3.x release (IDF 5.x).
- **Risk**: M5Unified, Module-Audio, ESP32-A2DP, and audio-tools may assume the bundled SDK/API; version mismatches can cause build or runtime issues. You need to verify each library against Arduino-ESP32 3.x + IDF 5.
- After upgrading, AudioTools will see `ESP_IDF_VERSION >= 5` and use the **new I2S API** (`i2s_channel_write`) instead of Legacy.

### Option B: Use ESP-IDF framework directly (no Arduino)

Switch to the **ESP-IDF** framework, which can use IDF 5.5 on the same platform:

```ini
[env:m5stack-core2_espidf]
platform = espressif32 @ 6.8.1
board = m5stack-core2
framework = espidf
```

- Your app would no longer be Arduino; it would be plain ESP-IDF (C/C++) with a different project layout and build. M5Unified and other Arduino libraries would need ESP-IDF–style integration or replacements. This is a **large change** and only worth it if you plan to leave Arduino.

## M5Unified and Module-Audio with `framework = espidf`

### M5Unified — supported

- **library.json**: `"frameworks": ["arduino", "espidf", "*"]` — both Arduino and ESP-IDF are listed.
- **Espressif Component Registry**: [m5stack/m5unified](https://components.espressif.com/components/m5stack/m5unified) is published as an ESP-IDF component; the readme states “Arduino / ESP-IDF Library” and “Supported frameworks: ESP-IDF, Arduino IDE”.
- **Usage with espidf**: Use the component from the registry or the same repo with CMake/component.mk; project layout must follow ESP-IDF (e.g. `main/`, `CMakeLists.txt`). M5GFX is required as a dependency.

### Module-Audio — not supported for ESP-IDF

- **library.json**: `"frameworks": ["arduino"]` only — no `espidf` entry.
- **Source**: Depends on Arduino APIs:
  - **TwoWire** (`Wire`, `beginTransmission`, `write`, `requestFrom`, etc.) in `audio_i2c.cpp` and `es8388.cpp`.
  - **delay()** (Arduino/FreeRTOS).
  - **Serial** (only inside `#if AUDIO_I2C_DEBUG` / `#if ES8388_DEBUG`).
- **Implication**: Under pure `framework = espidf` there is no Arduino `TwoWire`; you would need to either:
  - Reimplement the same I2C and ES8388 logic using ESP-IDF’s `driver/i2c_master.h` (and optionally the codec register set from the existing sources), or
  - Depend on an Arduino-as-component / compatibility layer that provides `TwoWire` under ESP-IDF (if available and maintained).
- **Conclusion**: Module-Audio is **Arduino-only**. For an ESP-IDF-only project, the Audio Module (I2C + ES8388) must be driven by an ESP-IDF-native I2C driver and your own or ported codec code.

### Summary for this project (M5 + Module-Audio + A2DP + I2S)

If you switch to **`framework = espidf`** for this stack:

- **M5Unified** can be used as an ESP-IDF component (registry or repo + CMake). Display, buttons, and M5 device handling remain available.
- **Module-Audio** cannot be used as-is: it is Arduino-only (TwoWire, etc.). You must either:
  - Implement or port the I2C + ES8388 codec layer to ESP-IDF (`i2c_master` + register logic from the existing sources), or
  - Stay on Arduino (or Arduino + IDF 5 via Option A) so that Module-Audio keeps working without a rewrite.

So **Option B (pure espidf) is only viable if you are willing to replace or port the Module-Audio (AudioI2c + ES8388) part**; the rest (M5Unified, A2DP, I2S feed) can be adapted to ESP-IDF with the appropriate components and project layout.

## Risks of upgrading to ESP-IDF 5.x (with Arduino)

| Risk | Description |
|------|-------------|
| **I2S API change** | Legacy `i2s_write` is replaced by `i2s_channel_write` and different init/APIs. AudioTools handles this via `USE_LEGACY_I2S`; with IDF 5 it will use the new driver. Our I2S feed task only calls `i2s.write()` so no app code change in theory, but driver behaviour (blocking, buffer sizes) can differ. |
| **Bluetooth / A2DP** | ESP32-A2DP and the BT stack are part of the SDK. IDF 5.x can have different BT task priorities, buffer sizes, or API tweaks. Re-test connection, stream stability, and our ring-buffer + feed task under load. |
| **M5Unified / Module-Audio** | These may rely on specific Arduino-ESP32 or IDF behaviour. Check their release notes and issues for “ESP-IDF 5” or “Arduino 3.0” compatibility. |
| **Build and toolchain** | New framework/SDK can pull different toolchains or headers; first build may fail until dependencies are updated or patched. |
| **Regressions** | Newer SDK can introduce new bugs or different timing; run your usual tests (connect, play, track change, underrun behaviour) after upgrading. |

## Recommendation

- **Stay on current setup** if you want maximum stability and the current stack (M5 + Module-Audio + ESP32-A2DP + audio-tools) is working.
- **Try Option A** in a **separate env** (e.g. `m5stack-core2_idf5`) if you specifically want the new I2S API or other IDF 5 features; run full tests and be prepared to fix library or API issues.
- **Option B** is only if you are committed to moving the project to pure ESP-IDF.
