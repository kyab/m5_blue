#include "usb_media.h"

#include "pico/stdlib.h"
#include "tusb.h"

namespace {

constexpr uint kRedGpio = 12;
constexpr uint kBlueGpio = 13;

constexpr uint8_t kPlayPauseBit = 1u << 0;
constexpr uint8_t kNextTrackBit = 1u << 1;

uint8_t g_buttons = 0;
uint8_t g_sent = 0;

uint8_t read_buttons() {
    const bool red = (gpio_get(kRedGpio) == 0);
    const bool blue = (gpio_get(kBlueGpio) == 0);
    return static_cast<uint8_t>((blue ? kPlayPauseBit : 0) | (red ? kNextTrackBit : 0));
}

}  // namespace

void usb_media_init(void) {
    gpio_init(kRedGpio);
    gpio_set_dir(kRedGpio, GPIO_IN);
    gpio_init(kBlueGpio);
    gpio_set_dir(kBlueGpio, GPIO_IN);

    g_buttons = read_buttons();
    g_sent = g_buttons;
}

void usb_media_poll_buttons(void) {
    g_buttons = read_buttons();
    if (g_buttons == g_sent) return;
    if (!tud_hid_ready()) return;
    if (tud_hid_report(0, &g_buttons, sizeof(g_buttons))) g_sent = g_buttons;
}

uint8_t usb_media_buttons(void) {
    return g_buttons;
}
