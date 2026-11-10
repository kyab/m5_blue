#include <cstdio>

#include "audio_engine.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "usb_audio.h"
#include "usb_midi.h"

namespace {

constexpr uint32_t kStatusIntervalMs = 1000;
constexpr uint32_t kButtonPollMs = 2;

void print_status() {
    static uint32_t last_loops = 0;

    const UsbAudioStats usb = usb_audio_take_stats();
    const AudioEngineStats engine = audio_engine_stats();
    printf("stream=%d primed=%d fifo[min=%lu max=%lu] rx_frames=%lu full=%lu underruns=%lu | core1 loops/s=%lu usb_short=%lu\n",
           usb.streaming, usb.primed, static_cast<unsigned long>(usb.fifo_min_bytes), static_cast<unsigned long>(usb.fifo_max_bytes),
           static_cast<unsigned long>(usb.rx_frames), static_cast<unsigned long>(usb.fifo_full_events),
           static_cast<unsigned long>(usb.underruns), static_cast<unsigned long>(engine.loops - last_loops),
           static_cast<unsigned long>(engine.usb_short_frames));
    const JoystickProbeLog joy = audio_engine_joystick_log();
    // w/r: byte count, -1 NACK or no device, -2 timeout, -99 read not started. Repeated so a late CDC open still shows the boot probe.
    printf("joy=%s fw=%u baud=%lu attempts=%lu ok_ms=%lu last_ms=%lu w=%d r=%d sda=%u scl=%u boot_fails=%lu boot_w=%d boot_r=%d\n",
           joy.ok ? "ok" : "fail", joy.fw, static_cast<unsigned long>(joy.baud), static_cast<unsigned long>(joy.attempts),
           static_cast<unsigned long>(joy.ok_at_ms), static_cast<unsigned long>(joy.last_try_ms), joy.last_write, joy.last_read, joy.sda,
           joy.scl, static_cast<unsigned long>(joy.boot_fails), joy.boot_write, joy.boot_read);
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
    audio_engine_init();
    audio_engine_start();

    uint32_t last_status_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_button_ms = last_status_ms;
    for (;;) {
        tud_task();
        usb_audio_task();
        usb_midi_task();

        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_button_ms >= kButtonPollMs) {
            last_button_ms = now_ms;
            usb_midi_poll_buttons();
        }
        if (now_ms - last_status_ms >= kStatusIntervalMs) {
            last_status_ms = now_ms;
            if (tud_cdc_connected()) print_status();
        }
    }
}
