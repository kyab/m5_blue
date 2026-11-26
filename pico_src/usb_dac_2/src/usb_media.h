#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void usb_media_init(void);
// Second and third Dual Buttons. Caller paces (~2 ms, core0, with the synth button).
void usb_media_poll_buttons(void);
// Current Consumer report: bit 0 = Play/Pause, bit 1 = Scan Next Track,
// bit 2 = Volume Increment, bit 3 = Volume Decrement.
uint8_t usb_media_buttons(void);

#ifdef __cplusplus
}
#endif
