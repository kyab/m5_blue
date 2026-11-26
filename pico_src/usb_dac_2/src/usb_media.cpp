#include "usb_media.h"

#include "pico/stdlib.h"
#include "tusb.h"

namespace {

constexpr uint kRedGpio = 12;
constexpr uint kBlueGpio = 13;
constexpr uint kVolumeDownGpio = 14;
constexpr uint kVolumeUpGpio = 15;

constexpr uint8_t kPlayPauseBit = 1u << 0;
constexpr uint8_t kNextTrackBit = 1u << 1;
constexpr uint8_t kVolumeUpBit = 1u << 2;
constexpr uint8_t kVolumeDownBit = 1u << 3;

uint8_t g_buttons = 0;
uint8_t g_sent = 0;

void init_input(uint gpio) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
}

uint8_t read_buttons() {
    const bool red = (gpio_get(kRedGpio) == 0);
    const bool blue = (gpio_get(kBlueGpio) == 0);
    const bool volume_down = (gpio_get(kVolumeDownGpio) == 0);
    const bool volume_up = (gpio_get(kVolumeUpGpio) == 0);
    return static_cast<uint8_t>((blue ? kPlayPauseBit : 0) | (red ? kNextTrackBit : 0) | (volume_up ? kVolumeUpBit : 0) | (volume_down ? kVolumeDownBit : 0));
}

}  // namespace

void usb_media_init(void) {
    init_input(kRedGpio);
    init_input(kBlueGpio);
    init_input(kVolumeDownGpio);
    init_input(kVolumeUpGpio);

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
