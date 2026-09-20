/**
 * Copyright (c) 2026 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdio>
#include <cstdint>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

#include "joystick2.hpp"

// Implemented in pico-sdk/lib/btstack/example/a2dp_sink_demo.c
extern "C" int btstack_main(int argc, const char* argv[]);

#if USING_I2C
extern "C" const btstack_audio_sink_t* btstack_audio_pico_sink_get_instance(void);
#endif

static constexpr uint32_t kJoystick2PollMs = 100;

static Joystick2 g_joystick2;

// Used by some BTstack examples to toggle the board LED.
extern "C" void hal_led_toggle(void) {
    static int led_state;
    led_state = 1 - led_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
}

static void joystick2_poll_once(void) {
    int16_t x = 0;
    int16_t y = 0;
    uint8_t button = 0xFF;
    const bool axes_ok = g_joystick2.readAxesOffset(&x, &y);
    const bool button_ok = g_joystick2.readButton(&button);

    if (axes_ok && button_ok) {
        printf("joy x=%d y=%d btn=%u\n", (int)x, (int)y, (unsigned)button);
    } else if (axes_ok) {
        printf("joy x=%d y=%d btn=?\n", (int)x, (int)y);
    } else {
        printf("joy read failed\n");
    }
}

// Pico SDK multicore: joystick I2C polling runs on core 1.
static void core1_entry(void) {
    while (true) {
        joystick2_poll_once();
        sleep_ms(kJoystick2PollMs);
    }
}

int main() {
    stdio_init_all();

    // sleep for boot messages to be observed by user.
    sleep_ms(3000);

    printf("main started\n");

    if (cyw43_arch_init() != PICO_OK) {
        panic("failed to cyw43");
    }

    g_joystick2.init();

#if USING_I2C
    btstack_audio_sink_set_instance(btstack_audio_pico_sink_get_instance());
#endif

    btstack_main(0, nullptr);

    if (g_joystick2.ok()) {
        multicore_launch_core1(core1_entry);
    }

    btstack_run_loop_execute();

    cyw43_arch_deinit();
    return 0;
}
