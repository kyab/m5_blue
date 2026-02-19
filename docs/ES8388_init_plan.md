# ES8388 init() plan – DAC Volume 100 at reset + SoftRamp

## Requirements

1. **Before** main DAC register setup: consider DAC Volume was 100 at reset.
   - Enable SoftRamp at **slowest** setting as a precaution.
   - Then **gradually** set DAC Volume to 0 (step down with delay).

2. **During** main DAC register setup:
   - Set all DAC registers as usual.
   - Again enable SoftRamp at **slowest**.
   - **Explicitly unmute** (clear mute).
   - Set DAC Volume to 0 **once more** as a precaution.

---

## Plan

### Phase A – Before DAC register setup (reset-with-volume-100 handling)

| Step | Action |
|------|--------|
| A1 | Read DACCONTROL3; set **DACRampRate=11**, **SoftRamp=1** only; **do not change mute bit** (preserve bits 4:0). |
| A2 | **Ramp** DAC volume from 100 down to 0 in steps (e.g. step 5, delay 2 ms each). |

### Phase B – DAC register setup

| Step | Action |
|------|--------|
| B1 | DACCONTROL1, DACCONTROL2 (format, clock). |
| B2 | **DACCONTROL3 = 0xE0** (slowest ramp, SoftRamp on, **unmute**). |
| B3 | DACCONTROL4, 5; mixer DACCONTROL16–21. |
| B4 | **DACCONTROL24 = 0, DACCONTROL25 = 0** (volume 0 again). |
| B5 | DACCONTROL26, 27. |
| B6 | DACPOWER = 0x3F. |
| B7 | CHIPPOWER = 0x00 (DEM/STM up). |

No ramp after DACPOWER; volume is already 0 from Phase A and B4.
