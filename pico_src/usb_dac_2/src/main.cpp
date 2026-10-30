#include <cstdio>

#include "audio_engine.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "usb_audio.h"
#include "usb_midi.h"

namespace {

constexpr uint32_t kStatusIntervalMs = 1000;

void print_status() {
    static uint32_t last_loops = 0;

    const UsbAudioStats usb = usb_audio_take_stats();
    const AudioEngineStats engine = audio_engine_stats();
    printf("stream=%d primed=%d fifo[min=%lu max=%lu] rx_frames=%lu full=%lu underruns=%lu | core1 loops/s=%lu usb_short=%lu\n",
           usb.streaming, usb.primed,
           static_cast<unsigned long>(usb.fifo_min_bytes), static_cast<unsigned long>(usb.fifo_max_bytes),
           static_cast<unsigned long>(usb.rx_frames), static_cast<unsigned long>(usb.fifo_full_events),
           static_cast<unsigned long>(usb.underruns),
           static_cast<unsigned long>(engine.loops - last_loops), static_cast<unsigned long>(engine.usb_short_frames));
    last_loops = engine.loops;
}

}  // namespace

int main() {
    tusb_rhport_init_t dev_init;
    dev_init.role = TUSB_ROLE_DEVICE;
    dev_init.speed = TUSB_SPEED_AUTO;
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    stdio_init_all();

    usb_audio_init();
    usb_midi_init();
    audio_engine_start();

    uint32_t last_status_ms = to_ms_since_boot(get_absolute_time());
    for (;;) {
        tud_task();
        usb_audio_task();
        usb_midi_task();

        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_status_ms >= kStatusIntervalMs) {
            last_status_ms = now_ms;
            if (tud_cdc_connected()) print_status();
        }
    }
}
