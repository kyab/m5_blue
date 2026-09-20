/**
 * Copyright (c) 2026 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cmath>
#include <cstdio>
#include <cstdint>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

#include "joystick2.hpp"
#include "RingBuffer.hpp"
#include "Freezer.hpp"
#include "DJFilter.hpp"
#include "effects.h"

// Implemented in pico-sdk/lib/btstack/example/a2dp_sink_demo.c
extern "C" int btstack_main(int argc, const char* argv[]);

#if USING_I2C
extern "C" const btstack_audio_sink_t* btstack_audio_pico_sink_get_instance(void);
#endif

// Match M5 loop delay(2) so joystick debounce constants keep the same wall-clock timing.
static constexpr uint32_t kJoystick2PollMs = 2;
static constexpr uint32_t kJoystick2PrintMs = 200;
static constexpr uint32_t kDjProcessFrames = 512; // matches SAMPLES_PER_BUFFER in btstack_audio_pico.c

static Joystick2 g_joystick2;

static RingBufferInterleaved* g_ring = nullptr;
static Freezer g_freezer(nullptr);
static DJFilter g_dj_filter;
static volatile float g_dj_filter_target_value = 0.0f;
static float g_dj_left[kDjProcessFrames];
static float g_dj_right[kDjProcessFrames];

static const float kDjFilterBypassDeadzone = 0.03f;

static const int16_t kJoystick2AxisOffsetFullScale = 4096;
static const int16_t kJoystick2XChangeThreshold = 200;
static const int16_t kJoystick2XDeadbandPos = 30;
static const int16_t kJoystick2XDeadbandNeg = -30;
static const float kJoystick2FilterVHPFMin = 0.55f;
static const float kJoystick2FilterVHPFMax = 0.99f;
static const float kJoystick2FilterVLPFMin = -0.10f;
static const float kJoystick2FilterVLPFMax = -0.86f;

static float apply_dj_filter_target_value(float v) {
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    if (fabsf(v) < kDjFilterBypassDeadzone) v = 0.0f;
    g_dj_filter_target_value = v;
    return v;
}

static float map_joystick2_linear(int16_t y, int16_t y_start, int16_t y_end, float v_start, float v_end) {
    const int32_t span = (int32_t)y_end - (int32_t)y_start;
    if (span == 0) {
        return v_end;
    }
    float t = (float)((int32_t)y - (int32_t)y_start) / (float)span;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    return v_start + t * (v_end - v_start);
}

static float map_joystick2_x_offset_to_filter_v(int16_t x_offset) {
    if (x_offset > kJoystick2XDeadbandPos) {
        const int16_t x_start = (int16_t)(kJoystick2XDeadbandPos + 1);
        return map_joystick2_linear(x_offset, x_start, kJoystick2AxisOffsetFullScale, kJoystick2FilterVHPFMin, kJoystick2FilterVHPFMax);
    }
    if (x_offset < kJoystick2XDeadbandNeg) {
        const int16_t x_start = (int16_t)(kJoystick2XDeadbandNeg - 1);
        return map_joystick2_linear(x_offset, x_start, (int16_t)(-kJoystick2AxisOffsetFullScale), kJoystick2FilterVLPFMin, kJoystick2FilterVLPFMax);
    }
    return 0.0f;
}

static uint32_t map_joystick2_y_offset_to_freezer_grain(int16_t y_offset) {
    int32_t y = y_offset;
    if (y < -kJoystick2AxisOffsetFullScale) {
        y = -kJoystick2AxisOffsetFullScale;
    } else if (y > kJoystick2AxisOffsetFullScale) {
        y = kJoystick2AxisOffsetFullScale;
    }

    const int32_t inputSpan = static_cast<int32_t>(kJoystick2AxisOffsetFullScale) * 2;
    const int32_t outputSpan = static_cast<int32_t>(Freezer::kMaxGrainSamples - Freezer::kMinGrainSamples);
    const int32_t offset = kJoystick2AxisOffsetFullScale - y;
    return Freezer::kMinGrainSamples + static_cast<uint32_t>((offset * outputSpan + inputSpan / 2) / inputSpan);
}

static void init_effects() {
    static RingBufferInterleaved ring;
    g_ring = &ring;
    g_freezer.setRingBuffer(g_ring);
}

extern "C" void apply_effects_before_i2s(int16_t* data, uint32_t frame_count) {
    static float s_applied_dj_filter_value = 0.0f;

    if (data == nullptr || frame_count == 0) return;

    g_freezer.process(data, frame_count);

    float target_v = g_dj_filter_target_value;
    if (target_v != s_applied_dj_filter_value) {
        g_dj_filter.setFilterValue(target_v);
        s_applied_dj_filter_value = target_v;
    }

    uint32_t offset = 0;
    while (offset < frame_count) {
        uint32_t n = frame_count - offset;
        if (n > kDjProcessFrames) n = kDjProcessFrames;
        for (uint32_t i = 0; i < n; i++) {
            const uint32_t src = (offset + i) * 2;
            g_dj_left[i] = static_cast<float>(data[src]) / 32768.0f;
            g_dj_right[i] = static_cast<float>(data[src + 1]) / 32768.0f;
        }
        g_dj_filter.process(g_dj_left, g_dj_right, n);
        for (uint32_t i = 0; i < n; i++) {
            float l = g_dj_left[i] * 32768.0f;
            float r = g_dj_right[i] * 32768.0f;
            if (l > 32767.0f)
                l = 32767.0f;
            else if (l < -32768.0f)
                l = -32768.0f;
            if (r > 32767.0f)
                r = 32767.0f;
            else if (r < -32768.0f)
                r = -32768.0f;
            const uint32_t dst = (offset + i) * 2;
            data[dst] = static_cast<int16_t>(l);
            data[dst + 1] = static_cast<int16_t>(r);
        }
        offset += n;
    }
}

// X drives the DJ filter, Y sets the Freezer grain, and Z activates Freezer while held.
static void update_effects_from_joystick2() {
    float v = 0.0f;
    static uint8_t s_button = 1; // 1 = released
    static uint8_t s_button_read_failures = 0;
    static bool s_z_latched = false;
    static uint8_t s_z_release_count = 0;
    static int16_t s_x_held = 0;
    static uint8_t s_x_zero_confirm = 0;
    static int16_t s_y_held = 0;
    static uint8_t s_y_zero_confirm = 0;
    static const uint8_t kZReleaseConfirm = 12; // ~24 ms at loop delay(2)
    static const uint8_t kButtonReadFailureRelease = 12;
    static const int16_t kAxisGlitchAbsPrev = 500;
    static const uint8_t kAxisZeroConfirm = 4;

    int16_t x_raw = 0;
    int16_t y_raw = 0;
    const bool joystick_ok = g_joystick2.ok();

    if (joystick_ok) {
        uint8_t button_raw = s_button;

        const bool button_read_ok = g_joystick2.readButton(&button_raw);
        const bool axes_read_ok = g_joystick2.readAxesOffset(&x_raw, &y_raw);

        if (button_read_ok) {
            s_button = button_raw;
            s_button_read_failures = 0;
        } else {
            if (s_button_read_failures < 255) s_button_read_failures++;
            if (s_button_read_failures >= kButtonReadFailureRelease) {
                s_button = 1;
                s_z_latched = false;
                s_z_release_count = 0;
            }
        }
        const bool z_raw = (s_button == 0);
        if (z_raw) {
            s_z_release_count = 0;
            s_z_latched = true;
        } else if (s_z_latched) {
            if (s_z_release_count < 255) s_z_release_count++;
            if (s_z_release_count >= kZReleaseConfirm) {
                s_z_latched = false;
                s_z_release_count = 0;
            }
        } else {
            s_z_release_count = 0;
        }

        if (axes_read_ok) {
            const bool x_sudden_zero =
                x_raw == 0 && (s_x_held > kAxisGlitchAbsPrev || s_x_held < -kAxisGlitchAbsPrev);
            if (x_sudden_zero) {
                if (s_x_zero_confirm < 255) s_x_zero_confirm++;
                if (s_x_zero_confirm >= kAxisZeroConfirm) {
                    s_x_held = 0;
                    s_x_zero_confirm = 0;
                }
            } else if (x_raw <= kJoystick2XChangeThreshold && x_raw >= -kJoystick2XChangeThreshold) {
                s_x_held = 0;
                s_x_zero_confirm = 0;
            } else {
                const int32_t x_delta = static_cast<int32_t>(x_raw) - static_cast<int32_t>(s_x_held);
                const int32_t x_delta_abs = x_delta < 0 ? -x_delta : x_delta;
                if (x_delta_abs >= kJoystick2XChangeThreshold) {
                    s_x_held = x_raw;
                }
                s_x_zero_confirm = 0;
            }

            const bool y_sudden_zero =
                y_raw == 0 && (s_y_held > kAxisGlitchAbsPrev || s_y_held < -kAxisGlitchAbsPrev);
            if (y_sudden_zero) {
                if (s_y_zero_confirm < 255) s_y_zero_confirm++;
                if (s_y_zero_confirm >= kAxisZeroConfirm) {
                    s_y_held = 0;
                    s_y_zero_confirm = 0;
                }
            } else {
                s_y_held = y_raw;
                s_y_zero_confirm = 0;
            }
        }
    }

    const int16_t x_used = s_x_held;
    const int16_t y_used = s_y_held;
    const uint32_t grainSamples = map_joystick2_y_offset_to_freezer_grain(y_used);
    v = map_joystick2_x_offset_to_filter_v(x_used);
    v = apply_dj_filter_target_value(v);

    g_freezer.setGrainSize(grainSamples);
    g_freezer.setActive(joystick_ok && s_z_latched);

    static uint32_t s_last_print_ms = 0;
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - s_last_print_ms >= kJoystick2PrintMs) {
        s_last_print_ms = now_ms;
        printf("joy x=%d y=%d z=%u v=%+.2f g=%lu\n",
               (int)x_raw, (int)y_raw, s_z_latched ? 1u : 0u, (double)v, (unsigned long)grainSamples);
    }
}

// Used by some BTstack examples to toggle the board LED.
extern "C" void hal_led_toggle(void) {
    static int led_state;
    led_state = 1 - led_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
}

// Pico SDK multicore: joystick I2C polling runs on core 1.
static void core1_entry(void) {
    while (true) {
        update_effects_from_joystick2();
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

    init_effects();
    g_joystick2.init();

#if USING_I2C
    btstack_audio_sink_set_instance(btstack_audio_pico_sink_get_instance());
#endif

    btstack_main(0, nullptr);

    if (g_joystick2.ok()) {
        multicore_launch_core1(core1_entry);
    } else {
        g_dj_filter_target_value = 0.0f;
    }

    btstack_run_loop_execute();

    cyw43_arch_deinit();
    return 0;
}
