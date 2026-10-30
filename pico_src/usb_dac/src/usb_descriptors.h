#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING,
    ITF_NUM_MIDI,
    ITF_NUM_MIDI_STREAMING,
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

// UAC1 entity IDs (match TUD_AUDIO10_SPEAKER_STEREO_FB_DESCRIPTOR layout).
#define UAC1_ENTITY_INPUT_TERMINAL 0x01
#define UAC1_ENTITY_FEATURE_UNIT 0x02
#define UAC1_ENTITY_OUTPUT_TERMINAL 0x03

// MIDI is a separate Audio-class function; give it its own IAD (protocol 0x00)
// so hosts do not fold it into the UAC1 Audio Control/Streaming pair.
#define TUD_MIDI_DESC_IAD_LEN 8
#define TUD_MIDI_DESC_IAD(_firstitf, _nitfs, _stridx) \
    TUD_MIDI_DESC_IAD_LEN, TUSB_DESC_INTERFACE_ASSOCIATION, _firstitf, _nitfs, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING, AUDIO_FUNC_PROTOCOL_CODE_UNDEF, _stridx

// Length matches TinyUSB example TUD_AUDIO10_SPEAKER_STEREO_FB_DESC_LEN.
#define TUD_AUDIO10_HEADPHONE_STEREO_FB_DESC_LEN(_nfreqs) ( \
    TUD_AUDIO10_DESC_STD_AC_LEN +                           \
    TUD_AUDIO10_DESC_CS_AC_LEN(1) +                         \
    TUD_AUDIO10_DESC_INPUT_TERM_LEN +                       \
    TUD_AUDIO10_DESC_OUTPUT_TERM_LEN +                      \
    TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2) +                  \
    TUD_AUDIO10_DESC_STD_AS_LEN +                           \
    TUD_AUDIO10_DESC_STD_AS_LEN +                           \
    TUD_AUDIO10_DESC_CS_AS_INT_LEN +                        \
    TUD_AUDIO10_DESC_TYPE_I_FORMAT_LEN(_nfreqs) +           \
    TUD_AUDIO10_DESC_STD_AS_ISO_EP_LEN +                    \
    TUD_AUDIO10_DESC_CS_AS_ISO_EP_LEN +                     \
    TUD_AUDIO10_DESC_STD_AS_ISO_SYNC_EP_LEN)

#define TUD_AUDIO10_HEADPHONE_STEREO_FB_DESCRIPTOR(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epout, _epoutsize, _epfb, ...)                                                                                                                                              \
    TUD_AUDIO10_DESC_STD_AC(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),                                                                                                                                                                                                    \
        TUD_AUDIO10_DESC_CS_AC(/*_bcdADC*/ 0x0100, /*_totallen*/ (TUD_AUDIO10_DESC_INPUT_TERM_LEN + TUD_AUDIO10_DESC_OUTPUT_TERM_LEN + TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2)), /*_itf*/ ((_itfnum) + 1)),                                                                                \
        TUD_AUDIO10_DESC_INPUT_TERM(/*_termid*/ 0x01, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_nchannels*/ 0x02, /*_channelcfg*/ AUDIO10_CHANNEL_CONFIG_LEFT_FRONT | AUDIO10_CHANNEL_CONFIG_RIGHT_FRONT, /*_idxchannelnames*/ 0x00, /*_stridx*/ 0x00),     \
        TUD_AUDIO10_DESC_OUTPUT_TERM(/*_termid*/ 0x03, /*_termtype*/ AUDIO_TERM_TYPE_OUT_HEADPHONES, /*_assocTerm*/ 0x00, /*_srcid*/ 0x02, /*_stridx*/ 0x00),                                                                                                                           \
        TUD_AUDIO10_DESC_FEATURE_UNIT(/*_unitid*/ 0x02, /*_srcid*/ 0x01, /*_stridx*/ 0x00, /*_ctrlmaster*/ (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME), /*_ctrlch1*/ (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME), /*_ctrlch2*/ (AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME)), \
        TUD_AUDIO10_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum) + 1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),                                                                                                                                                             \
        TUD_AUDIO10_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum) + 1), /*_altset*/ 0x01, /*_nEPs*/ 0x02, /*_stridx*/ 0x00),                                                                                                                                                             \
        TUD_AUDIO10_DESC_CS_AS_INT(/*_termid*/ 0x01, /*_delay*/ 0x00, /*_formattype*/ AUDIO10_DATA_FORMAT_TYPE_I_PCM),                                                                                                                                                                      \
        TUD_AUDIO10_DESC_TYPE_I_FORMAT(/*_nrchannels*/ 0x02, /*_subframesize*/ _nBytesPerSample, /*_bitresolution*/ _nBitsUsedPerSample, /*_freqs*/ __VA_ARGS__),                                                                                                                           \
        TUD_AUDIO10_DESC_STD_AS_ISO_EP(/*_ep*/ _epout, /*_attr*/ (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS), /*_maxEPsize*/ _epoutsize, /*_interval*/ 0x01, /*_sync_ep*/ _epfb),                                                                  \
        TUD_AUDIO10_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO10_CS_AS_ISO_DATA_EP_ATT_SAMPLING_FRQ, /*_lockdelayunits*/ AUDIO10_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000),                                                                                              \
        TUD_AUDIO10_DESC_STD_AS_ISO_SYNC_EP(/*_ep*/ _epfb, /*_bRefresh*/ 0)

#endif /* USB_DESCRIPTORS_H_ */
