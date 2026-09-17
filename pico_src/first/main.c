/**
 * Copyright (c) 2026 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

// Implemented in pico-sdk/lib/btstack/example/a2dp_sink_demo.c
int btstack_main(int argc, const char *argv[]);

#if USING_I2C
const btstack_audio_sink_t *btstack_audio_pico_sink_get_instance(void);
#endif

// Used by some BTstack examples to toggle the board LED.
void hal_led_toggle(void) {
    static int led_state;
    led_state = 1 - led_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init() != PICO_OK) {
        panic("failed to cyw43");
    }

#if USING_I2C
    btstack_audio_sink_set_instance(btstack_audio_pico_sink_get_instance());
#endif

    btstack_main(0, NULL);
    btstack_run_loop_execute();

    cyw43_arch_deinit();
    return 0;
}
