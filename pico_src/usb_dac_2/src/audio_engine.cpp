#include "audio_engine.h"

#include <algorithm>
#include <atomic>
#include <cmath>

#include "DJFilter.hpp"
#include "Freezer.hpp"
#include "RingBuffer.hpp"
#include "Synth.hpp"
#include "hardware/dma.h"
#include "joystick2.hpp"
#include "pico/audio_i2s.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "usb_audio.h"

namespace {

constexpr uint32_t kI2sBufferCount = 4;
constexpr int kSynthSemitoneC4 = 12;
constexpr float kUsbFadeStep = 1.0f / static_cast<float>(kAudioFramesPerBuffer);
constexpr float kMasterGainSmoothing = 0.002f;

// Match pico_src/first DJ mode joystick mapping and debounce.
constexpr int16_t kJoystick2AxisOffsetFullScale = 4096;
constexpr int16_t kJoystick2XChangeThreshold = 200;
constexpr int16_t kAxisGlitchAbsPrev = 500;
constexpr uint8_t kAxisZeroConfirm = 1;  // ~8 ms at 8 ms polling (was 4 counts at 2 ms)
constexpr uint8_t kZReleaseConfirm = 3;   // ~24 ms at 8 ms polling (was 12 counts at 2 ms)
constexpr uint8_t kButtonReadFailureRelease = 3;
constexpr float kDjFilterBypassDeadzone = 0.03f;
constexpr int16_t kJoystick2XDeadbandPos = 30;
constexpr int16_t kJoystick2XDeadbandNeg = -30;
constexpr float kJoystick2FilterVHPFMin = 0.55f;
constexpr float kJoystick2FilterVHPFMax = 0.99f;
constexpr float kJoystick2FilterVLPFMin = -0.10f;
constexpr float kJoystick2FilterVLPFMax = -0.86f;
constexpr uint32_t kJoystick2PollMs = 8;
// Cold USB insert: probes in the first ~250 ms got NACK (-1). The probe at 501 ms read fw=1.
constexpr uint32_t kJoystick2ReadyMs = 500;
constexpr uint32_t kJoystick2RetryMs = 200;

static_assert(Synth::kSampleRateHz == static_cast<float>(kAudioSampleRateHz), "Synth must run at the I2S sample rate");

Synth g_synth;
std::atomic<bool> g_synth_gate{false};
std::atomic<uint32_t> g_loops{0};
std::atomic<uint32_t> g_usb_short_frames{0};

Joystick2 g_joystick2;
RingBufferInterleaved* g_ring = nullptr;
Freezer g_freezer(nullptr);
DJFilter g_dj_filter;
std::atomic<float> g_dj_filter_target_value{0.0f};

uint32_t g_core1_stack[1024];

float map_joystick2_linear(int16_t y, int16_t y_start, int16_t y_end, float v_start, float v_end) {
    const int32_t span = static_cast<int32_t>(y_end) - static_cast<int32_t>(y_start);
    if (span == 0) {
        return v_end;
    }
    float t = static_cast<float>(static_cast<int32_t>(y) - static_cast<int32_t>(y_start)) / static_cast<float>(span);
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    return v_start + t * (v_end - v_start);
}

float map_joystick2_x_offset_to_filter_v(int16_t x_offset) {
    if (x_offset > kJoystick2XDeadbandPos) {
        const int16_t x_start = static_cast<int16_t>(kJoystick2XDeadbandPos + 1);
        return map_joystick2_linear(x_offset, x_start, kJoystick2AxisOffsetFullScale, kJoystick2FilterVHPFMin, kJoystick2FilterVHPFMax);
    }
    if (x_offset < kJoystick2XDeadbandNeg) {
        const int16_t x_start = static_cast<int16_t>(kJoystick2XDeadbandNeg - 1);
        return map_joystick2_linear(x_offset, x_start, static_cast<int16_t>(-kJoystick2AxisOffsetFullScale), kJoystick2FilterVLPFMin, kJoystick2FilterVLPFMax);
    }
    return 0.0f;
}

uint32_t map_joystick2_y_offset_to_freezer_grain(int16_t y_offset) {
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

float apply_dj_filter_target_value(float v) {
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    if (fabsf(v) < kDjFilterBypassDeadzone) v = 0.0f;
    g_dj_filter_target_value.store(v, std::memory_order_relaxed);
    return v;
}

audio_buffer_pool_t* setup_i2s() {
    static audio_format_t audio_format;
    audio_format.format = AUDIO_BUFFER_FORMAT_PCM_S16;
    audio_format.sample_freq = kAudioSampleRateHz;
    audio_format.channel_count = 2;

    static audio_buffer_format_t producer_format;
    producer_format.format = &audio_format;
    producer_format.sample_stride = sizeof(StereoFrame);

    audio_buffer_pool_t* pool = audio_new_producer_pool(&producer_format, kI2sBufferCount, kAudioFramesPerBuffer);

    audio_i2s_config_t config;
    config.data_pin = PICO_AUDIO_I2S_DATA_PIN;
    config.clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE;
    config.dma_channel = static_cast<uint8_t>(dma_claim_unused_channel(true));
    config.pio_sm = 0;
    // audio_i2s_setup() claims the channel again (pico-extras issue #48).
    dma_channel_unclaim(config.dma_channel);

    if (!audio_i2s_setup(&audio_format, &config)) panic("I2S setup failed");
    if (!audio_i2s_connect(pool)) panic("I2S connect failed");
    audio_i2s_set_enabled(true);
    return pool;
}

void render(float* lr, uint32_t frames) {
    g_synth.gen(lr, frames);
}

void update_synth_trigger() {
    static bool pressed = false;
    const bool gate = g_synth_gate.load(std::memory_order_relaxed);
    if (gate == pressed) return;
    pressed = gate;
    if (pressed) {
        g_synth.setSemitone(kSynthSemitoneC4);
        g_synth.noteOn();
    } else {
        g_synth.noteOff();
    }
}

void apply_usb_effects(StereoFrame* usb, float* usb_lr, uint32_t frames) {
    static float s_applied_dj_filter_value = 0.0f;
    static float s_dj_left[kAudioFramesPerBuffer];
    static float s_dj_right[kAudioFramesPerBuffer];

    // Freezer expects interleaved int16 PCM.
    g_freezer.process(reinterpret_cast<int16_t*>(usb), frames);

    for (uint32_t i = 0; i < frames; ++i) {
        usb_lr[i * 2] = static_cast<float>(usb[i].l) * (1.0f / 32768.0f);
        usb_lr[i * 2 + 1] = static_cast<float>(usb[i].r) * (1.0f / 32768.0f);
    }

    const float target_v = g_dj_filter_target_value.load(std::memory_order_relaxed);
    if (target_v != s_applied_dj_filter_value) {
        g_dj_filter.setFilterValue(target_v);
        s_applied_dj_filter_value = target_v;
    }

    for (uint32_t i = 0; i < frames; ++i) {
        s_dj_left[i] = usb_lr[i * 2];
        s_dj_right[i] = usb_lr[i * 2 + 1];
    }
    g_dj_filter.process(s_dj_left, s_dj_right, frames);
    for (uint32_t i = 0; i < frames; ++i) {
        usb_lr[i * 2] = s_dj_left[i];
        usb_lr[i * 2 + 1] = s_dj_right[i];
    }
}

void mix_to_i2s(const float* synth, const float* usb_lr, uint32_t frames, StereoFrame* out) {
    static float usb_gain = 0.0f;
    static float master_gain = usb_audio_master_gain();

    const float usb_target = usb_audio_active() ? 1.0f : 0.0f;
    const float master_target = usb_audio_master_gain();

    for (uint32_t i = 0; i < frames; ++i) {
        if (usb_gain < usb_target) {
            usb_gain = std::min(usb_target, usb_gain + kUsbFadeStep);
        } else if (usb_gain > usb_target) {
            usb_gain = std::max(usb_target, usb_gain - kUsbFadeStep);
        }
        master_gain += (master_target - master_gain) * kMasterGainSmoothing;

        const float l = (synth[i * 2] + usb_lr[i * 2] * usb_gain) * master_gain;
        const float r = (synth[i * 2 + 1] + usb_lr[i * 2 + 1] * usb_gain) * master_gain;
        out[i].l = static_cast<int16_t>(std::clamp(l, -1.0f, 1.0f) * 32767.0f);
        out[i].r = static_cast<int16_t>(std::clamp(r, -1.0f, 1.0f) * 32767.0f);
    }
}

void core1_main() {
    audio_buffer_pool_t* pool = setup_i2s();

    float synth[kAudioFramesPerBuffer * 2];
    float usb_lr[kAudioFramesPerBuffer * 2];
    StereoFrame usb[kAudioFramesPerBuffer];

    uint32_t last_joystick_ms = to_ms_since_boot(get_absolute_time());
    for (;;) {
        // I2C stays off core0. A blocking read on that core was starving usb_audio_task.
        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_joystick_ms >= kJoystick2PollMs) {
            audio_engine_poll_joystick(now_ms);
            last_joystick_ms = to_ms_since_boot(get_absolute_time());
        }

        audio_buffer_t* buffer = take_audio_buffer(pool, true);
        const uint32_t frames = buffer->max_sample_count;

        update_synth_trigger();
        render(synth, frames);

        const uint32_t got = usb_audio_pull(usb, frames);
        if (got < frames) {
            for (uint32_t i = got; i < frames; ++i) usb[i] = StereoFrame{0, 0};
            if (usb_audio_active()) g_usb_short_frames.fetch_add(frames - got, std::memory_order_relaxed);
        }

        apply_usb_effects(usb, usb_lr, frames);
        mix_to_i2s(synth, usb_lr, frames, reinterpret_cast<StereoFrame*>(buffer->buffer->bytes));

        buffer->sample_count = frames;
        give_audio_buffer(pool, buffer);
        g_loops.fetch_add(1, std::memory_order_relaxed);
    }
}

}  // namespace

void audio_engine_init() {
    static RingBufferInterleaved ring;
    g_ring = &ring;
    g_freezer.setRingBuffer(g_ring);

    g_joystick2.init();
    if (!g_joystick2.ok()) {
        g_dj_filter_target_value.store(0.0f, std::memory_order_relaxed);
        g_freezer.setActive(false);
    }
}

void audio_engine_start() {
    multicore_launch_core1_with_stack(core1_main, g_core1_stack, sizeof(g_core1_stack));
}

void audio_engine_poll_joystick(uint32_t now_ms) {
    if (!g_joystick2.ok() && now_ms >= kJoystick2ReadyMs) {
        static uint32_t last_probe_ms = 0;
        if (last_probe_ms == 0 || now_ms - last_probe_ms >= kJoystick2RetryMs) {
            last_probe_ms = now_ms;
            g_joystick2.probe();
        }
    }

    static uint8_t s_button = 1;  // 1 = released
    static uint8_t s_button_read_failures = 0;
    static bool s_z_latched = false;
    static uint8_t s_z_release_count = 0;
    static int16_t s_x_held = 0;
    static uint8_t s_x_zero_confirm = 0;
    static int16_t s_y_held = 0;
    static uint8_t s_y_zero_confirm = 0;

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
            const bool x_sudden_zero = x_raw == 0 && (s_x_held > kAxisGlitchAbsPrev || s_x_held < -kAxisGlitchAbsPrev);
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

            const bool y_sudden_zero = y_raw == 0 && (s_y_held > kAxisGlitchAbsPrev || s_y_held < -kAxisGlitchAbsPrev);
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
    float v = map_joystick2_x_offset_to_filter_v(x_used);
    v = apply_dj_filter_target_value(v);

    g_freezer.setGrainSize(grainSamples);
    g_freezer.setActive(joystick_ok && s_z_latched);
}

void audio_engine_set_synth_gate(bool pressed) {
    g_synth_gate.store(pressed, std::memory_order_relaxed);
}

AudioEngineStats audio_engine_stats() {
    return AudioEngineStats{g_loops.load(std::memory_order_relaxed), g_usb_short_frames.load(std::memory_order_relaxed)};
}
