#include "usb_midi.h"

#include "audio_engine.h"
#include "pico/stdlib.h"
#include "tusb.h"

namespace {

constexpr uint kDualButtonRedGpio = 6;
constexpr uint kDualButtonBlueGpio = 7;

constexpr uint8_t kChannel = 0;
constexpr uint8_t kNoteC4 = 60;
constexpr uint8_t kVelocity = 100;
constexpr uint8_t kCableNumber = 0;

bool g_red_pressed = false;
bool g_blue_pressed = false;

void send_note(bool on) {
    const uint8_t msg[3] = {static_cast<uint8_t>((on ? 0x90 : 0x80) | kChannel), kNoteC4, on ? kVelocity : static_cast<uint8_t>(0)};
    tud_midi_stream_write(kCableNumber, msg, sizeof(msg));
}

}  // namespace

void usb_midi_init() {
    gpio_init(kDualButtonRedGpio);
    gpio_set_dir(kDualButtonRedGpio, GPIO_IN);
    gpio_init(kDualButtonBlueGpio);
    gpio_set_dir(kDualButtonBlueGpio, GPIO_IN);

    g_red_pressed = (gpio_get(kDualButtonRedGpio) == 0);
    g_blue_pressed = (gpio_get(kDualButtonBlueGpio) == 0);
    audio_engine_set_synth_gate(g_blue_pressed);
}

void usb_midi_task() {
    uint8_t discard[32];
    while (tud_midi_available()) tud_midi_stream_read(discard, sizeof(discard));
}

void usb_midi_poll_buttons() {
    const bool red = (gpio_get(kDualButtonRedGpio) == 0);
    const bool blue = (gpio_get(kDualButtonBlueGpio) == 0);
    audio_engine_set_synth_gate(blue);

    const bool mounted = tud_midi_mounted();
    if (red != g_red_pressed) {
        g_red_pressed = red;
        if (mounted) send_note(red);
    }
    if (blue != g_blue_pressed) {
        g_blue_pressed = blue;
        if (mounted) send_note(blue);
    }
}
