# ES8388 DAC Soft Ramp – Register Summary and Code Reference

## Datasheet reference

ES8388 datasheet (e.g. https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1141/ES8388.pdf), Table 5 and Section 6.3.3. **DAC Soft Ramp** is in **Reg.25 (DAC Control 3)**, not in DAC Control 1.

---

## DAC Soft Ramp related registers (from datasheet Table 5)

| Datasheet | Address | Library name    | Bit fields (Table 5)     |
|-----------|---------|-----------------|--------------------------|
| Reg.23    | 0x17    | DACCONTROL1     | DACLRSWAP, DACLRP, DACWL, DACFORMAT |
| Reg.24    | 0x18    | DACCONTROL2     | DACFsMode, DACFsRatio    |
| **Reg.25**| **0x19**| **DACCONTROL3** | **DACRampRate, DACSoftRamp, DACLeR, DACMute** |

- **DACCONTROL3 (0x19) = Reg.25**  
  - Contains **DACSoftRamp** (one bit in the DACRampRate/DACSoftRamp/DACLeR/DACMute group).  
  - Typical use: set **DACSoftRamp (bit 5)** to 1 to enable DAC soft ramp; leave mute/other bits as-is or set per application.  
  - Module-Audio `init()` writes `DACCONTROL3 = 0x00`; we enable DACSoftRamp by read-modify-write with `reg25 |= 0x20`.

- **DACCONTROL1 (0x17)**  
  - Word length and format only; **no Soft Ramp bit** in the datasheet.

---

## Current state in this codebase (after fix)

- **`es8388.init()`**: writes `DACCONTROL3 = 0x00` (DACSoftRamp off).
- **`main.cpp`** (after `setDACOutput`, `setDACVolume`, `setBitsSample`, `setSampleRate`):  
  - Reads **DACCONTROL3** (0x19), sets **bit 5 (DACSoftRamp)**, writes back.  
  - Mute is still controlled by `setDACmute()` (same register); we do not clear its bits in this block.
- **DACCONTROL1** is no longer used for Soft Ramp; only `setBitsSample()` touches it (word length).

---

## Code (implemented)

In `main.cpp`, after ES8388 init and setDACVolume/setBitsSample/setSampleRate:

1. Read DACCONTROL3 via I2C (Wire).  
2. `reg25 |= 0x20` (DACSoftRamp).  
3. Write DACCONTROL3 back.  
4. Log written value for verification.

No DACCONTROL1 or DACCONTROL11 write for Soft Ramp; DACCONTROL3 only.
