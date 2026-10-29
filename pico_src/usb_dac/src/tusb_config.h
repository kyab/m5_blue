#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// Board / common
//--------------------------------------------------------------------+

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT 0
#endif

#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED OPT_MODE_DEFAULT_SPEED
#endif

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_PICO
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED BOARD_TUD_MAX_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//--------------------------------------------------------------------+
// Classes: UAC1 speaker + MIDI + CDC
//--------------------------------------------------------------------+

#define CFG_TUD_AUDIO 1
#define CFG_TUD_MIDI 1
#define CFG_TUD_CDC 1
#define CFG_TUD_HID 0
#define CFG_TUD_MSC 0
#define CFG_TUD_VENDOR 0

#define CFG_TUD_CDC_RX_BUFSIZE 64
#define CFG_TUD_CDC_TX_BUFSIZE 512
#define CFG_TUD_CDC_EP_BUFSIZE 64

#define CFG_TUD_MIDI_RX_BUFSIZE 64
#define CFG_TUD_MIDI_TX_BUFSIZE 64

//--------------------------------------------------------------------+
// Audio (UAC1 Full-Speed, 48 kHz stereo 16-bit, async + feedback)
//--------------------------------------------------------------------+

#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX 2
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX 2
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX 16

#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE 48000

// EP size for FS: ((48000+999)/1000 + 1) * 2 * 2 = 196 (49 frames max).
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX \
    TUD_AUDIO_EP_SIZE(false, CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)

// FIFO_COUNT needs >= 4 packet buffers.
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ (4 * CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX)

#define CFG_TUD_AUDIO_ENABLE_EP_OUT 1
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP 1

#define CFG_TUD_AUDIO_CTRL_BUF_SZ 64

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H_ */
