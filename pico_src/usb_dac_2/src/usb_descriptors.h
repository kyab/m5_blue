#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING,
    ITF_NUM_MIDI_CONTROL,
    ITF_NUM_MIDI_STREAMING,
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define USB_AUDIO_SAMPLE_RATE_HZ 48000

#define UAC1_ENTITY_INPUT_TERMINAL 0x01
#define UAC1_ENTITY_FEATURE_UNIT 0x02
#define UAC1_ENTITY_OUTPUT_TERMINAL 0x03

// USB-MIDI 1.0 is an Audio-class function; its own IAD (protocol 0) keeps hosts from folding it into the UAC1 function.
#define TUD_MIDI_DESC_IAD_LEN 8
#define TUD_MIDI_DESC_IAD(_firstitf, _nitfs, _stridx) \
    TUD_MIDI_DESC_IAD_LEN, TUSB_DESC_INTERFACE_ASSOCIATION, _firstitf, _nitfs, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING, AUDIO_FUNC_PROTOCOL_CODE_UNDEF, _stridx

#define TUD_AUDIO10_HEADPHONES_STEREO_FB_DESC_LEN (TUD_AUDIO10_DESC_STD_AC_LEN + TUD_AUDIO10_DESC_CS_AC_LEN(1) + TUD_AUDIO10_DESC_INPUT_TERM_LEN + TUD_AUDIO10_DESC_OUTPUT_TERM_LEN + TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2) + TUD_AUDIO10_DESC_STD_AS_LEN + TUD_AUDIO10_DESC_STD_AS_LEN + TUD_AUDIO10_DESC_CS_AS_INT_LEN + TUD_AUDIO10_DESC_TYPE_I_FORMAT_LEN(1) + TUD_AUDIO10_DESC_STD_AS_ISO_EP_LEN + TUD_AUDIO10_DESC_CS_AS_ISO_EP_LEN + TUD_AUDIO10_DESC_STD_AS_ISO_SYNC_EP_LEN)

// Feature Unit exposes mute and volume on the master channel only.
#define TUD_AUDIO10_HEADPHONES_STEREO_FB_DESCRIPTOR(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epout, _epoutsize, _epfb, _freq)                                                                                                                                                                                                                      \
    TUD_AUDIO10_DESC_STD_AC(_itfnum, 0x00, _stridx),                                                                                                                                                                                                                                                                                                          \
        TUD_AUDIO10_DESC_CS_AC(0x0100, (TUD_AUDIO10_DESC_INPUT_TERM_LEN + TUD_AUDIO10_DESC_OUTPUT_TERM_LEN + TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2)), ((_itfnum) + 1)),                                                                                                                                                                                            \
        TUD_AUDIO10_DESC_INPUT_TERM(UAC1_ENTITY_INPUT_TERMINAL, AUDIO_TERM_TYPE_USB_STREAMING, 0x00, 0x02, AUDIO10_CHANNEL_CONFIG_LEFT_FRONT | AUDIO10_CHANNEL_CONFIG_RIGHT_FRONT, 0x00, 0x00),                                                                                                                                                                 \
        TUD_AUDIO10_DESC_OUTPUT_TERM(UAC1_ENTITY_OUTPUT_TERMINAL, AUDIO_TERM_TYPE_OUT_HEADPHONES, 0x00, UAC1_ENTITY_FEATURE_UNIT, 0x00),                                                                                                                                                                                                                          \
        TUD_AUDIO10_DESC_FEATURE_UNIT(UAC1_ENTITY_FEATURE_UNIT, UAC1_ENTITY_INPUT_TERMINAL, 0x00, (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME), 0, 0),                                                                                                                                                                                             \
        TUD_AUDIO10_DESC_STD_AS_INT((uint8_t)((_itfnum) + 1), 0x00, 0x00, 0x00),                                                                                                                                                                                                                                                                                  \
        TUD_AUDIO10_DESC_STD_AS_INT((uint8_t)((_itfnum) + 1), 0x01, 0x02, 0x00),                                                                                                                                                                                                                                                                                  \
        TUD_AUDIO10_DESC_CS_AS_INT(UAC1_ENTITY_INPUT_TERMINAL, 0x00, AUDIO10_DATA_FORMAT_TYPE_I_PCM),                                                                                                                                                                                                                                                             \
        TUD_AUDIO10_DESC_TYPE_I_FORMAT(0x02, _nBytesPerSample, _nBitsUsedPerSample, _freq),                                                                                                                                                                                                                                                                       \
        TUD_AUDIO10_DESC_STD_AS_ISO_EP(_epout, (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS), _epoutsize, 0x01, _epfb),                                                                                                                                                                                                       \
        TUD_AUDIO10_DESC_CS_AS_ISO_EP(AUDIO10_CS_AS_ISO_DATA_EP_ATT_SAMPLING_FRQ, AUDIO10_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0x0000),                                                                                                                                                                                                                   \
        TUD_AUDIO10_DESC_STD_AS_ISO_SYNC_EP(_epfb, 0)

#endif
