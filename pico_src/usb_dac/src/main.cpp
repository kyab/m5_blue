#include "audio_engine.h"
#include "usb_audio.h"
#include "usb_midi.h"

#include "pico/stdlib.h"
#include "tusb.h"

#include <cstdio>

namespace {

void status_task() {
    static uint32_t next_ms = 0;
    uint32_t const now = to_ms_since_boot(get_absolute_time());
    if ((int32_t)(now - next_ms) < 0) return;
    next_ms = now + 1000;

    AudioSharedState &s = audio_shared();
    uint32_t const loops = s.core1_loops.exchange(0, std::memory_order_relaxed);
    uint32_t const underrun = s.underrun_count.exchange(0, std::memory_order_relaxed);
    uint32_t const overflow = s.overflow_count.exchange(0, std::memory_order_relaxed);
    uint32_t const fifo_min = s.fifo_min.exchange(UINT32_MAX, std::memory_order_relaxed);
    uint32_t const fifo_max = s.fifo_max.exchange(0, std::memory_order_relaxed);
    uint16_t const fifo_now = tud_audio_available();
    size_t const ring = s.ring.size();

    // printf only from core0 normal context (never from TinyUSB callbacks / core1).
    printf(
        "mounted=%u cdc=%u stream=%u rate=%lu ring=%u fifo=%u min=%lu max=%lu "
        "loops=%lu underrun=%lu overflow=%lu mute=%u vol=%d\n",
        tud_mounted() ? 1u : 0u,
        tud_cdc_connected() ? 1u : 0u,
        s.streaming.load(std::memory_order_relaxed) ? 1u : 0u,
        (unsigned long)kSampleRateHz,
        (unsigned)ring,
        (unsigned)fifo_now,
        (unsigned long)(fifo_min == UINT32_MAX ? 0 : fifo_min),
        (unsigned long)fifo_max,
        (unsigned long)loops,
        (unsigned long)underrun,
        (unsigned long)overflow,
        s.host_mute.load(std::memory_order_relaxed) ? 1u : 0u,
        (int)s.host_volume_db.load(std::memory_order_relaxed));
}

} // namespace

int main() {
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };
    // pico_stdio_usb requires tusb_init before stdio_init_all when LIB_TINYUSB_DEVICE=1.
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    stdio_init_all();

    usb_audio_init();
    usb_midi_init_gpio();
    audio_engine_start();

    printf("usb_dac ready (UAC1 headphones 48 kHz + MIDI + CDC)\n");

    while (true) {
        tud_task();
        usb_audio_poll();
        usb_midi_poll();
        status_task();
    }
}
