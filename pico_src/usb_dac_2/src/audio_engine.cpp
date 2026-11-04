#include "audio_engine.h"

#include <algorithm>
#include <atomic>

#include "hardware/dma.h"
#include "pico/audio_i2s.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "Synth.hpp"
#include "usb_audio.h"

namespace {

constexpr uint32_t kI2sBufferCount = 4;
constexpr int kSynthSemitoneC4 = 12;
constexpr float kUsbFadeStep = 1.0f / static_cast<float>(kAudioFramesPerBuffer);
constexpr float kMasterGainSmoothing = 0.002f;

static_assert(Synth::kSampleRateHz == static_cast<float>(kAudioSampleRateHz), "Synth must run at the I2S sample rate");

Synth g_synth;
std::atomic<bool> g_synth_gate{false};
std::atomic<uint32_t> g_loops{0};
std::atomic<uint32_t> g_usb_short_frames{0};

uint32_t g_core1_stack[1024];

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

void mix_to_i2s(const float* synth, const StereoFrame* usb, uint32_t frames, StereoFrame* out) {
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

        const float l = (synth[i * 2] + static_cast<float>(usb[i].l) * (1.0f / 32768.0f) * usb_gain) * master_gain;
        const float r = (synth[i * 2 + 1] + static_cast<float>(usb[i].r) * (1.0f / 32768.0f) * usb_gain) * master_gain;
        out[i].l = static_cast<int16_t>(std::clamp(l, -1.0f, 1.0f) * 32767.0f);
        out[i].r = static_cast<int16_t>(std::clamp(r, -1.0f, 1.0f) * 32767.0f);
    }
}

void core1_main() {
    audio_buffer_pool_t* pool = setup_i2s();

    float synth[kAudioFramesPerBuffer * 2];
    StereoFrame usb[kAudioFramesPerBuffer];

    for (;;) {
        audio_buffer_t* buffer = take_audio_buffer(pool, true);
        const uint32_t frames = buffer->max_sample_count;

        update_synth_trigger();
        render(synth, frames);

        const uint32_t got = usb_audio_pull(usb, frames);
        if (got < frames) {
            for (uint32_t i = got; i < frames; ++i) usb[i] = StereoFrame{0, 0};
            if (usb_audio_active()) g_usb_short_frames.fetch_add(frames - got, std::memory_order_relaxed);
        }

        mix_to_i2s(synth, usb, frames, reinterpret_cast<StereoFrame*>(buffer->buffer->bytes));

        buffer->sample_count = frames;
        give_audio_buffer(pool, buffer);
        g_loops.fetch_add(1, std::memory_order_relaxed);
    }
}

}  // namespace

void audio_engine_start() {
    multicore_launch_core1_with_stack(core1_main, g_core1_stack, sizeof(g_core1_stack));
}

void audio_engine_set_synth_gate(bool pressed) {
    g_synth_gate.store(pressed, std::memory_order_relaxed);
}

AudioEngineStats audio_engine_stats() {
    return AudioEngineStats{g_loops.load(std::memory_order_relaxed), g_usb_short_frames.load(std::memory_order_relaxed)};
}
