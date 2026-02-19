# ES8388 Reset Pop-Noise Safety Plan and Implementation

## Target scenario

This document focuses on the worst-case reset condition on M5Stack Core2:

- `DACPOWER` is ON (DAC + line outputs enabled),
- DAC output volume is high (application-level volume = 100),
- DAC is unmuted,
- then the **hardware reset button** is pressed.

For this case, we want to minimize audible pop noise both for end users and during development cycles.

## Important practical constraint

On hardware reset-button press, firmware cannot run a pre-shutdown sequence.  
So the mitigation strategy is:

1. Apply a **very early startup safety clamp** (as soon as possible after boot),
2. Keep DAC muted during codec re-initialization,
3. Unmute only after format/routing/volume are stable.

## Official datasheet basis

Source: ES8388 datasheet (Rev 5.0, July 2018)  
`https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1141/ES8388.pdf`

Relevant registers:

- **Reg.4 (`DACPOWER`, 0x04)**  
  - `0xC0` means both DAC channels power-down and output paths disabled (safe off state).
- **Reg.25 (`DACCONTROL3`, 0x19)**  
  - `DACMute` is **bit2** (`0x04`),
  - `DACSoftRamp` is bit5 (`0x20`),
  - `DACRampRate` is bits7:6 (`0xC0` for slowest = 0.5 dB / 128 LRCK).
- **Reg.28 (`DACCONTROL6`, 0x1C)**  
  - `ClickFree` is bit3 (`0x08`), used for click-free DAC power-up/down behavior.

## Network sample-code survey

### Espressif ESP-ADF driver

Reference: `components/audio_hal/driver/es8388/es8388.c`  
`https://github.com/espressif/esp-adf/blob/master/components/audio_hal/driver/es8388/es8388.c`

Observed patterns:

- Mute control is done by RMW on `DACCONTROL3` with mute bit at **bit2**.
- Init sequence starts by writing `DACCONTROL3 = 0x04` (muted).
- DAC output enable/disable is managed through `DACPOWER` (`0xC0` used to disable DAC/outs in init path).

### Apache NuttX ES8388 driver

Reference: `drivers/audio/es8388.c` / `es8388.h`  
`https://github.com/apache/nuttx/blob/master/drivers/audio/es8388.c`  
`https://github.com/apache/nuttx/blob/master/drivers/audio/es8388.h`

Observed patterns:

- `DACMUTE` mask is defined at shift 2 (`bit2`), consistent with datasheet.
- Reset/init path forces DAC mute first, then configures routing/volume/power.

### M5Module-Audio upstream

Reference: `src/es8388.cpp`  
`https://github.com/m5stack/M5Module-Audio/blob/master/src/es8388.cpp`

Observed issue in upstream baseline:

- `setDACmute()` reads `ADCCONTROL1` and writes `DACCONTROL3`, which is a wrong register source for DAC mute RMW.

## Safe register operation procedure

### Phase A: Emergency startup clamp (reset recovery)

Execute as early as possible on boot:

1. `DACCONTROL3` (Reg.25): set `DACMute=1`, `DACSoftRamp=1`, `DACRampRate=11` (slowest).
2. `DACCONTROL6` (Reg.28): set `ClickFree=1`.
3. Set output volume registers to minimum (`DACCONTROL24..27 = 0x00`).
4. Set digital DAC attenuation to minimum (`DACCONTROL4/5 = 0xC0`) as additional guard.
5. Set `DACPOWER = 0xC0` to disable DAC outputs.
6. Short settle delay (2 ms).

### Phase B: Normal init while muted

1. Configure clocks / serial format / mixer routing.
2. Keep `DACCONTROL3` in muted + soft-ramp state.
3. Re-apply safe output volume defaults (`DACCONTROL24..27`).
4. Enable `DACPOWER` for selected outputs.
5. Keep mute ON until stream configuration and startup volume are confirmed.

### Phase C: Controlled unmute

1. Set initial application volume low (0).
2. Clear only `DACMute` bit (Reg.25 bit2) via read-modify-write.
3. Keep SoftRamp/ramp-rate bits untouched.

## Implemented changes in this repository

- Reworked implementation to avoid fragile merge/conflict dependency on local codec library forks.
- `platformio.ini` uses the project submodule layout (`Module-Audio` + `ESP32-A2DP`) like `main` branch.
- `main.cpp` now owns the reset-safety sequence with direct I2C register operations:
  - early boot clamp (`DACMute=1`, `SoftRamp=1`, slowest ramp, minimum gains, `DACPOWER=0xC0`),
  - direct mute/unmute by RMW on `DACCONTROL3` bit2,
  - controlled unmute only after DAC format/sample-rate/volume setup.
- The approach keeps ES8388 safety logic explicit and local to app startup code, reducing future rebase conflicts.

## Why this is safer for reset-button development loops

- Even if hardware reset occurs while codec output is active and loud, next boot now pushes codec into a safe muted/off state quickly.
- Re-init no longer performs risky unmute transitions mid-configuration.
- Mute bit handling now matches official register definition and common driver implementations.
