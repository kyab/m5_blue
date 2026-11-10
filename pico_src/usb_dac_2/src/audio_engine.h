#pragma once

#include <cstdint>

constexpr uint32_t kAudioSampleRateHz = 48000;
constexpr uint32_t kAudioFramesPerBuffer = 48;

struct AudioEngineStats {
    uint32_t loops;
    uint32_t usb_short_frames;
};

// Last Joystick2 probe. Printed every second so a late CDC open still shows boot failures.
// write/read codes: byte count, -1 generic (NACK or no device), -2 timeout, -99 read not started.
struct JoystickProbeLog {
    bool ok;
    uint8_t fw;
    int last_write;
    int last_read;
    uint32_t attempts;
    uint32_t last_try_ms;
    uint32_t ok_at_ms;
    uint32_t boot_fails;
    int boot_write;
    int boot_read;
    uint8_t sda;
    uint8_t scl;
    uint32_t baud;
};

// Init Joystick2 / Freezer / DJ Filter on core0 before launching core1.
void audio_engine_init();
// Launches the audio loop on core1.
void audio_engine_start();
// Called from core1, paced at ~8 ms. Core0 must not touch I2C after init.
// now_ms is to_ms_since_boot. A failed boot probe is retried from here.
void audio_engine_poll_joystick(uint32_t now_ms);
// Safe to call from core0.
void audio_engine_set_synth_gate(bool pressed);
AudioEngineStats audio_engine_stats();
JoystickProbeLog audio_engine_joystick_log();
