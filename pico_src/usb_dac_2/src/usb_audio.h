#pragma once

#include <cstdint>

struct StereoFrame {
    int16_t l;
    int16_t r;
};

struct UsbAudioStats {
    bool streaming;
    bool primed;
    uint32_t fifo_min_bytes;
    uint32_t fifo_max_bytes;
    uint32_t rx_frames;
    uint32_t fifo_full_events;
    uint32_t underruns;
};

// core0 only.
void usb_audio_init();
void usb_audio_task();
// Returns the stats of the interval since the previous call and starts a new interval.
UsbAudioStats usb_audio_take_stats();

// core1 side. usb_audio_active() is true while a primed stream feeds the ring.
uint32_t usb_audio_pull(StereoFrame* dst, uint32_t frames);
bool usb_audio_active();
float usb_audio_master_gain();
