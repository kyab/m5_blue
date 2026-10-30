#pragma once

#include "spsc_ring.h"

#include <atomic>
#include <cstdint>

static constexpr uint32_t kSampleRateHz = 48000;
static constexpr size_t kUsbRingCapacityFrames = 144; // ~3 ms at 48 kHz

using UsbAudioRing = SpscStereoRing<kUsbRingCapacityFrames>;

struct AudioSharedState {
    UsbAudioRing ring;
    std::atomic<bool> host_mute{false};
    std::atomic<int16_t> host_volume_db{0}; // integer dB, -50..0
    std::atomic<bool> blue_pressed{false};
    std::atomic<bool> streaming{false};
    std::atomic<uint32_t> core1_loops{0};
    std::atomic<uint32_t> underrun_count{0};
    std::atomic<uint32_t> overflow_count{0};
    std::atomic<uint32_t> fifo_min{UINT32_MAX};
    std::atomic<uint32_t> fifo_max{0};
    std::atomic<uint32_t> last_feedback{0};
};

AudioSharedState &audio_shared();

void usb_audio_init();
void usb_audio_poll(); // core0: drain TinyUSB FIFO → ring at I2S pace
