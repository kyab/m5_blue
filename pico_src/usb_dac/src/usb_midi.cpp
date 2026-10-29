#include "usb_midi.h"

#include "usb_audio.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "tusb.h"

namespace {

constexpr uint kDualButtonRedGpio = 6;
constexpr uint kDualButtonBlueGpio = 7;
constexpr uint32_t kPollMs = 2;
constexpr uint8_t kMidiNoteC4 = 60;
constexpr uint8_t kMidiVelocity = 100;
constexpr uint8_t kMidiChannel = 0; // ch 1

uint32_t last_poll_ms = 0;
bool red_was = false;
bool blue_was = false;

void send_note(bool on) {
    if (!tud_midi_mounted()) return;
    uint8_t packet[4] = {
        0x09,
        (uint8_t)((on ? 0x90 : 0x80) | kMidiChannel),
        kMidiNoteC4,
        on ? kMidiVelocity : (uint8_t)0,
    };
    tud_midi_packet_write(packet);
}

void discard_host_midi() {
    uint8_t packet[4];
    while (tud_midi_available()) {
        if (!tud_midi_packet_read(packet)) break;
    }
}

} // namespace

void usb_midi_init_gpio() {
    // INPUT only; Dual Button onboard 10k pull-ups need VCC=3V3 (no Pico internal pull-up).
    gpio_init(kDualButtonRedGpio);
    gpio_set_dir(kDualButtonRedGpio, GPIO_IN);
    gpio_init(kDualButtonBlueGpio);
    gpio_set_dir(kDualButtonBlueGpio, GPIO_IN);
}

void usb_midi_poll() {
    discard_host_midi();

    uint32_t const now = to_ms_since_boot(get_absolute_time());
    if ((now - last_poll_ms) < kPollMs) return;
    last_poll_ms = now;

    // Active-low, no software debounce (same as first).
    const bool red = (gpio_get(kDualButtonRedGpio) == 0);
    const bool blue = (gpio_get(kDualButtonBlueGpio) == 0);

    audio_shared().blue_pressed.store(blue, std::memory_order_relaxed);

    if (red && !red_was) send_note(true);
    else if (!red && red_was) send_note(false);

    if (blue && !blue_was) send_note(true);
    else if (!blue && blue_was) send_note(false);

    red_was = red;
    blue_was = blue;
}
