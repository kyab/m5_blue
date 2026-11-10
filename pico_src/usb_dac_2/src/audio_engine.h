#pragma once

#include <cstdint>

constexpr uint32_t kAudioSampleRateHz = 48000;
constexpr uint32_t kAudioFramesPerBuffer = 48;

struct AudioEngineStats {
    uint32_t loops;
    uint32_t usb_short_frames;
};

// Init Joystick2 / Freezer / DJ Filter on core0 before launching core1.
void audio_engine_init();
// Launches the audio loop on core1.
void audio_engine_start();
// Called from core1, paced at ~8 ms. Core0 must not touch I2C after init.
// now_ms is to_ms_since_boot. The firmware probe runs here, after Joystick2 can ACK.
void audio_engine_poll_joystick(uint32_t now_ms);
// Safe to call from core0.
void audio_engine_set_synth_gate(bool pressed);
AudioEngineStats audio_engine_stats();
