# ES8388 Register 25 – DAC Control 3 (DACCONTROL3)

**Address:** 0x19  
**Datasheet:** ES8388 (Everest Semiconductor), Section 6.3.3, Table 5  
**Default value:** 0x22 (0010 0010)

---

## Bit layout

| Bit(s) | Name         | Value | Description |
|--------|--------------|-------|-------------|
| **7:6** | DACRampRate  | 00    | 0.5 dB per 4 LRCK (default) |
|        |              | 01    | 0.5 dB per 32 LRCK |
|        |              | 10    | 0.5 dB per 64 LRCK |
|        |              | 11    | 0.5 dB per 128 LRCK |
| **5**  | DACSoftRamp  | 0     | Digital volume control soft ramp disabled |
|        |              | 1     | Digital volume control soft ramp enabled (default) |
| **4**  | (reserved)   | —     | — |
| **3**  | DACLeR       | 0     | Normal (default) |
|        |              | 1     | Both channel gain set by DAC left gain control register |
| **2**  | DACMute      | 0     | Normal (default) |
|        |              | 1     | Mute analog outputs for both channels |
| **1:0**| (reserved)   | —     | — |

---

## Mask values (for software)

| Field        | Mask  | Enable / Set | Disable / Clear |
|-------------|-------|----------------|------------------|
| DACRampRate | 0xC0  | —             | 0x00 (default)    |
| DACSoftRamp | 0x20  | 0x20          | 0x00             |
| DACLeR      | 0x08  | 0x08          | 0x00 (default)   |
| DACMute     | 0x04  | 0x04 (mute)   | 0x00 (unmute)    |

---

## Note on DACMute bit

The datasheet defines **DACMute at bit 2** (mask 0x04). Some code (e.g. Module-Audio `setDACmute()`) uses **bit 1** (0x02); if mute does not behave as expected, use bit 2 (0x04) for mute and unmute.

---

## Example: SoftRamp on, unmute, default ramp rate

- DACRampRate = 00 (0x00)
- DACSoftRamp = 1 (0x20)
- DACLeR = 0 (0x00)
- DACMute = 0 (0x00)  
→ **Value = 0x20**
