#include "audio_engine.h"

#include "Synth.hpp"
#include "usb_audio.h"

#include "pico/audio_i2s.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr uint32_t kI2sFrames = 48;
constexpr int kI2sBufferCount = 4;
constexpr int kSynthSemitoneC4 = 12; // Synth base is C3
constexpr float kVolumeSmooth = 0.08f;
constexpr float kUsbFadeStep = 1.0f / 48.0f; // ~1 ms fade

Synth g_synth;
struct audio_buffer_pool *g_pool = nullptr;

float g_gain_smoothed = 1.0f;
float g_usb_fade = 0.0f;
bool g_blue_was = false;

float g_synth_buf[kI2sFrames * 2];
int16_t g_usb_buf[kI2sFrames * 2];

float host_gain_target() {
    AudioSharedState &s = audio_shared();
    if (s.host_mute.load(std::memory_order_relaxed)) return 0.0f;
    int16_t db = s.host_volume_db.load(std::memory_order_relaxed);
    if (db >= 0) return 1.0f;
    if (db <= -50) return 0.0f;
    return powf(10.0f, (float)db / 20.0f);
}

void render(float *lr, uint32_t frames) {
    AudioSharedState &s = audio_shared();
    const bool blue = s.blue_pressed.load(std::memory_order_relaxed);
    if (blue && !g_blue_was) {
        g_synth.setSemitone(kSynthSemitoneC4);
        g_synth.noteOn();
    } else if (!blue && g_blue_was) {
        g_synth.noteOff();
    }
    g_blue_was = blue;

    g_synth.gen(lr, frames);
}

void core1_entry() {
    while (true) {
        audio_buffer_t *buf = take_audio_buffer(g_pool, true);
        if (buf == nullptr) continue;

        AudioSharedState &s = audio_shared();
        const uint32_t frames = std::min<uint32_t>(buf->max_sample_count, kI2sFrames);

        size_t const got = s.ring.pop(g_usb_buf, frames);
        if (got < frames) {
            s.underrun_count.fetch_add(1, std::memory_order_relaxed);
        }

        const bool streaming = s.streaming.load(std::memory_order_relaxed);
        if (streaming && got > 0) {
            g_usb_fade = 1.0f;
        } else if (g_usb_fade > 0.0f) {
            g_usb_fade -= kUsbFadeStep;
            if (g_usb_fade < 0.0f) g_usb_fade = 0.0f;
        }

        render(g_synth_buf, frames);

        const float target = host_gain_target();
        g_gain_smoothed += (target - g_gain_smoothed) * kVolumeSmooth;

        int16_t *out = (int16_t *)buf->buffer->bytes;
        for (uint32_t i = 0; i < frames; ++i) {
            float l = g_synth_buf[i * 2];
            float r = g_synth_buf[i * 2 + 1];
            if (g_usb_fade > 0.0f) {
                l += ((float)g_usb_buf[i * 2] / 32768.0f) * g_usb_fade;
                r += ((float)g_usb_buf[i * 2 + 1] / 32768.0f) * g_usb_fade;
            }
            l *= g_gain_smoothed;
            r *= g_gain_smoothed;
            if (l > 1.0f) l = 1.0f;
            if (l < -1.0f) l = -1.0f;
            if (r > 1.0f) r = 1.0f;
            if (r < -1.0f) r = -1.0f;
            out[i * 2] = (int16_t)(l * 32767.0f);
            out[i * 2 + 1] = (int16_t)(r * 32767.0f);
        }

        buf->sample_count = frames;
        give_audio_buffer(g_pool, buf);
        s.core1_loops.fetch_add(1, std::memory_order_relaxed);
    }
}

} // namespace

void audio_engine_start() {
    static audio_format fmt = {
        .sample_freq = kSampleRateHz,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };
    static audio_buffer_format producer_format = {
        .format = &fmt,
        .sample_stride = 4,
    };

    g_pool = audio_new_producer_pool(&producer_format, kI2sBufferCount, kI2sFrames);

    audio_i2s_config config = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
        .dma_channel = 0,
        .pio_sm = 0,
    };
    if (!audio_i2s_setup(&fmt, &config)) {
        panic("i2s setup failed");
    }
    if (!audio_i2s_connect_extra(g_pool, false, 3, kI2sFrames, NULL)) {
        panic("i2s connect failed");
    }
    // GP22 MUTE left untouched (same as first).
    audio_i2s_set_enabled(true);

    multicore_launch_core1(core1_entry);
}
