// Phase 3: Bluetooth A2DP + Module Audio + Custom Audio Processing
// Experiment: I2S feed task + ring buffer; silence on underrun for noise-free output

#include <M5Unified.h>
#include "audio_i2c.hpp"
#include "es8388.hpp"
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include "RingBuffer.hpp"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include <cmath>

// MCLK output configuration (required for ES8388)
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"

// Module Audio I2C pins
#define SYS_I2C_SDA_PIN 21
#define SYS_I2C_SCL_PIN 22

// Module Audio I2S pins
#define SYS_I2S_MCLK_PIN 0
#define SYS_I2S_SCLK_PIN 19
#define SYS_I2S_LRCK_PIN 27
#define SYS_I2S_DOUT_PIN 2
#define SYS_I2S_DIN_PIN 34

// Dual button connected to Port.A
#define DUAL_BUTTON_BLUE 33
#define DUAL_BUTTON_RED 32

// Audio I2C device (for RGB LED, HP detect, etc.)
AudioI2c device;

// ES8388 codec
ES8388 es8388(&Wire, SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN);

namespace {
constexpr uint8_t kDacMuteBit = 0x04;               // DACCONTROL3 bit2
constexpr uint8_t kDacSoftRampBit = 0x20;           // DACCONTROL3 bit5
constexpr uint8_t kDacRampRateMask = 0xC0;          // DACCONTROL3 bits7:6
constexpr uint8_t kDacRampRateSlowest = 0xC0;       // 0.5dB / 128 LRCK
constexpr uint8_t kDacClickFreeBit = 0x08;          // DACCONTROL6 bit3
constexpr uint8_t kDacPowerAllOff = 0xC0;           // DACPOWER: both DACs off, outputs disabled
constexpr uint8_t kDacDigitalMute = 0xC0;           // DACCONTROL4/5: -96dB
constexpr uint32_t kCodecClampDelayMs = 2;

bool es8388_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ES8388_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool es8388_read_reg(uint8_t reg, uint8_t* value) {
    Wire.beginTransmission(ES8388_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)ES8388_ADDR, (uint8_t)1) != 1) return false;
    *value = Wire.read();
    return true;
}

// Force a known-safe DAC state immediately after reset:
// mute on, soft-ramp slowest, minimum output gain, and DAC outputs disabled.
bool es8388_apply_reset_safety_clamp() {
    bool ok = true;
    uint8_t reg = 0;
    if (es8388_read_reg(ES8388_DACCONTROL3, &reg)) {
        reg = static_cast<uint8_t>((reg & ~(kDacRampRateMask | kDacSoftRampBit | kDacMuteBit))
              | kDacRampRateSlowest | kDacSoftRampBit | kDacMuteBit);
        ok &= es8388_write_reg(ES8388_DACCONTROL3, reg);
    } else {
        ok &= es8388_write_reg(ES8388_DACCONTROL3, static_cast<uint8_t>(kDacRampRateSlowest | kDacSoftRampBit | kDacMuteBit));
    }

    if (es8388_read_reg(ES8388_DACCONTROL6, &reg)) {
        reg |= kDacClickFreeBit;
        ok &= es8388_write_reg(ES8388_DACCONTROL6, reg);
    } else {
        ok &= es8388_write_reg(ES8388_DACCONTROL6, kDacClickFreeBit);
    }

    ok &= es8388_write_reg(ES8388_DACCONTROL24, 0x00);
    ok &= es8388_write_reg(ES8388_DACCONTROL25, 0x00);
    ok &= es8388_write_reg(ES8388_DACCONTROL26, 0x00);
    ok &= es8388_write_reg(ES8388_DACCONTROL27, 0x00);
    ok &= es8388_write_reg(ES8388_DACCONTROL4, kDacDigitalMute);
    ok &= es8388_write_reg(ES8388_DACCONTROL5, kDacDigitalMute);
    ok &= es8388_write_reg(ES8388_DACPOWER, kDacPowerAllOff);
    delay(kCodecClampDelayMs);
    return ok;
}

bool es8388_set_dac_mute_direct(bool mute) {
    uint8_t reg = 0;
    if (!es8388_read_reg(ES8388_DACCONTROL3, &reg)) return false;
    reg = static_cast<uint8_t>((reg & ~kDacRampRateMask) | kDacRampRateSlowest | kDacSoftRampBit);
    if (mute) {
        reg |= kDacMuteBit;
    } else {
        reg = static_cast<uint8_t>(reg & ~kDacMuteBit);
    }
    return es8388_write_reg(ES8388_DACCONTROL3, reg);
}
}  // namespace

// I2SStream for audio output (managed by A2DP library)
I2SStream i2s;

// Bluetooth A2DP Sink with I2SStream
BluetoothA2DPSink a2dp_sink(i2s);

// Ring buffer for delay effect
RingBufferInterleaved* g_ring = nullptr;

// Effect flags
bool g_effect_blue = false; // Volume reduction
bool g_effect_red = false;  // Delay effect

// I2S feed: app-owned ring buffer so we can output silence when callback doesn't run.
//
// Log note: "A2DP audio state: Suspended" + set_i2s_active(0/1) appear when the remote
// (e.g. MacBook) suspends/resumes the stream (track skip, pause, or focus change). While
// suspended the callback may stop, so we output silence; on resume the 0→audio step caused
// pops — we mitigate with a short resume fade-in in the feed task (kResumeRampSamples).
//
// I2S write pace and "how much has been played":
// - Consumption rate is fixed by hardware: sample_rate * channels * bytes_per_sample
//   = 44100 * 2 * 2 = 176400 bytes/sec. We must supply data at least at this rate on average.
// - AudioTools I2SStream.write() typically blocks until the data is accepted into the driver's
//   DMA buffer; when that buffer is full, write() blocks. So we are paced by the hardware:
//   we cannot write faster than consumption for long, and we must keep writing to avoid underrun.
// - The I2S driver does NOT expose "how many bytes have been played" or playback position.
//   We only know "how much we have written". Management is done by: (1) feeding at the right
//   average rate (our task + ring buffer), (2) on underrun outputting silence so the DAC never
//   sees garbage or repeated samples.
//
static const uint32_t kSampleRate = 44100;
static const size_t kFeedRingBufBytes = (kSampleRate * 2 * 2) / 5; // 200 ms stereo 16-bit
static const size_t kSilenceChunkBytes = 1024;                     // ~5.8 ms at 44.1k stereo; write at least this often
// After underrun we output silence; when callback resumes, 0→audio step causes a pop.
// Apply a short fade-in over the first N samples of real audio (in the feed task).
static const uint32_t kResumeRampSamples = 512; // ~11.6 ms at 44.1k stereo
// When pausing, transition from data to silence causes a pop. Fade out over first N samples of silence.
static const uint32_t kPauseFadeOutSamples = 1024; // ~23 ms (spread over multiple chunks)
// When source sends silence (zeros), don't forward zeros to I2S (causes pop); replace with dither after short fade
static const int kSilenceDataThreshold = 1;       // max abs sample to treat chunk as silence
static const uint32_t kSilenceDataFadeSamples = 128;  // fade from last sample to dither over this many samples
static RingbufHandle_t g_feed_ring = nullptr;
static TaskHandle_t g_i2s_feed_task_handle = nullptr;

// Anti-pop: mute/ramp so DAC never sees discontinuities
static const uint32_t kRampUpSamples = (kSampleRate * 100) / 1000;         // 100 ms fade-in
static const uint32_t kTrackChangeMuteSamples = (kSampleRate * 35) / 1000; // 35 ms mute
volatile bool g_bt_connected = false;
volatile uint32_t g_ramp_up_samples_left = 0;
volatile uint32_t g_track_change_mute_left = 0;

// Feed task sets g_dac_volume_target (0 or 100); loop() ramps via service_dac_volume_ramp().
// When kPreBTTestCycles > 0 we skip DAC ramp so the test observes raw hardware (no mute/volume change).

// DAC volume ramp: target set by feed task; loop() ramps current toward target via setDACVolume() to reduce pops.
// Start at 0; feed task sets target 100 when first real audio arrives to avoid startup pop.
volatile int g_dac_volume_target = 0;
static int g_dac_volume_current = 0;

// Global LFSR for ±1 LSB dither (pre-BT and runtime). 32-bit Galois, polynomial 0x80200003, max period 2^32-1
static uint32_t g_lfsr_state = 0xACE1u;
static inline int16_t dither_lsb() {
    g_lfsr_state = (g_lfsr_state >> 1) ^ (0x80200003u & (uint32_t)(-(int32_t)(g_lfsr_state & 1)));
    return (g_lfsr_state & 1u) ? (int16_t)1 : (int16_t)-1;
}

// Pre-BT test: sine wave for a few sec, then silence for a few sec, repeat N cycles.
// Set kPreBTTestCycles > 0 to run; 0 to skip and start BT immediately.
// When running this test, DAC ramp is skipped (service_dac_volume_ramp + feed target=0) so we observe raw hardware.
//
// ES8388 soft mute is DACCONTROL3 (Reg.25) bit2 (DACMute).
// We use direct I2C RMW here so mute handling is independent from library implementation details.
//
// kPreBTTestUseGainRamp: 0 = switch sine/silence buffers (old). 1 = same sine, 2s gain 1->0->0->1 (no buffer switch).
// When 1, i2s_feed_task logs immediately if ring buffer underrun (feed task starved).
volatile bool g_pre_bt_gain_ramp_test_active = false;
static const int kPreBTTestCycles = 0;
static const int kPreBTTestSineSec = 2;
static const int kPreBTTestSilenceSec = 2;
static const int kPreBTTestUseGainRamp = 1;  // 1 = gain-ramp test (2s full, 2s fade-out, 2s zero, 2s fade-in) x5

// Forward declarations
void i2s_feed_task(void* arg);

// Ramps g_dac_volume_current toward g_dac_volume_target; call from loop(). Skipped when kPreBTTestCycles > 0.
static const int kDacVolumeRampStep = 20;
void service_dac_volume_ramp() {
    if (kPreBTTestCycles > 0) return;
    int target = g_dac_volume_target;
    if (g_dac_volume_current < target) {
        g_dac_volume_current += kDacVolumeRampStep;
        if (g_dac_volume_current > target) g_dac_volume_current = target;
        es8388.setDACVolume(g_dac_volume_current);
    } else if (g_dac_volume_current > target) {
        g_dac_volume_current -= kDacVolumeRampStep;
        if (g_dac_volume_current < target) g_dac_volume_current = target;
        es8388.setDACVolume(g_dac_volume_current);
    }
}

// #region agent log
static void debug_log(const char* hid, const char* msg, const char* data, int line) {
    Serial.printf("{\"ts\":%lu,\"hypothesisId\":\"%s\",\"message\":\"%s\",\"data\":%s,\"location\":\"main.cpp:%d\"}\n",
                  (unsigned long)millis(), hid, msg, data ? data : "{}", line);
}
#define DEBUG_LOG(hid, msg, data) debug_log(hid, msg, data, __LINE__)

// Startup pop debugging: log step then delay so user can hear when pop occurs (step id in log).
static const int kStartupStepDelayMs = 1000;
static void startup_step(const char* step_id, const char* step_name) {
    Serial.printf("{\"ts\":%lu,\"startup_step\":\"%s\",\"name\":\"%s\"}\n",
                  (unsigned long)millis(), step_id, step_name);
    delay(kStartupStepDelayMs);
}
// #endregion

// Audio callback - process in-place then push to feed ring; library does NOT write to I2S
void audio_callback(int16_t* data, uint32_t sample_num) {
    static bool first_call = true;
    if (first_call) {
        ESP_LOGI("audio", "*** audio_callback FIRST CALL *** samples=%lu", sample_num);
        first_call = false;
    }

    // Anti-pop gain: per-sample ramps; when effectively mute use ±1 LSB dither instead of zero
    if (!g_bt_connected) {
        for (uint32_t i = 0; i < sample_num; i++) {
            int16_t d = dither_lsb();
            data[i * 2] = d;
            data[i * 2 + 1] = d;
        }
    } else {
        const uint32_t ramp_start = (g_ramp_up_samples_left < kRampUpSamples)
            ? (kRampUpSamples - g_ramp_up_samples_left) : 0u;
        for (uint32_t i = 0; i < sample_num; i++) {
            float gain;
            if (g_track_change_mute_left > 0) {
                gain = 0.0f;
                g_track_change_mute_left--;
            } else if (g_ramp_up_samples_left > 0) {
                uint32_t global_index = ramp_start + i;
                gain = (global_index >= kRampUpSamples) ? 1.0f
                    : (static_cast<float>(global_index) / static_cast<float>(kRampUpSamples));
                g_ramp_up_samples_left--;
            } else {
                gain = 1.0f;
            }
            int16_t l = data[i * 2];
            int16_t r = data[i * 2 + 1];
            if (gain != 1.0f) {
                l = static_cast<int16_t>(static_cast<float>(l) * gain);
                r = static_cast<int16_t>(static_cast<float>(r) * gain);
                if (gain <= 0.001f) {
                    l = dither_lsb();
                    r = dither_lsb();
                }
            }
            data[i * 2] = l;
            data[i * 2 + 1] = r;
        }
    }

    // Apply volume effect
    if (g_effect_blue) {
        for (uint32_t i = 0; i < sample_num; i++) {
            int16_t* left = &data[i * 2];
            int16_t* right = &data[i * 2 + 1];
            float leftf = *left;
            float rightf = *right;
            leftf *= 0.3f;
            rightf *= 0.3f;
            *left = static_cast<int16_t>(leftf);
            *right = static_cast<int16_t>(rightf);
        }
    }

    // Apply delay effect using ring buffer
    if (g_ring != nullptr) {
        if (g_effect_red) {
            g_ring->storeSamples(data, sample_num);
            g_ring->readSamplesTo(data, sample_num);
        } else {
            g_ring->storeSamples(data, sample_num);
        }
    }

    // Push to I2S feed ring buffer (non-blocking; drop if full to avoid blocking BT stack)
    if (g_feed_ring != nullptr) {
        size_t len = sample_num * 4;
        BaseType_t ok = xRingbufferSend(g_feed_ring, data, len, 0);
        if (ok != pdTRUE) {
            // #region agent log
            static char s_dbg[72];
            snprintf(s_dbg, sizeof(s_dbg), "{\"drop_len\":%u,\"sample_num\":%lu}", (unsigned)len, (unsigned long)sample_num);
            DEBUG_LOG("H4", "ring_drop", s_dbg);
            // #endregion
        }
    }

    static uint32_t processed_samples = 0;
    processed_samples += sample_num;
    if (processed_samples >= 44100) {
        processed_samples = 0;
        ESP_LOGI("audio", "audio_callback");
    }
}

// Connection state callback
void connection_state_callback(esp_a2d_connection_state_t state, void* ptr) {
    const char* state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
    ESP_LOGI("a2dp", "Connection state: %s", state_str[state]);

    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        g_bt_connected = true;
        g_ramp_up_samples_left = kRampUpSamples;
    } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        g_bt_connected = false;
    }

    // Update display
    M5.Display.fillRect(0, 180, 320, 60, BLACK);
    M5.Display.setCursor(0, 180);
    M5.Display.setTextColor(state == ESP_A2D_CONNECTION_STATE_CONNECTED ? GREEN : YELLOW);
    M5.Display.printf("BT: %s", state_str[state]);
}

// AVRCP track change: brief mute to avoid click between tracks
void track_change_callback(uint8_t* id) {
    (void)id;
    g_track_change_mute_left = kTrackChangeMuteSamples;
}

// #region agent log
void audio_state_callback_debug(esp_a2d_audio_state_t state, void* obj) {
    (void)obj;
    static char s_dbg[48];
    snprintf(s_dbg, sizeof(s_dbg), "{\"state\":%d}", (int)state);
    DEBUG_LOG("H5", "a2dp_audio_state", s_dbg);
}
// #endregion

// I2S feed task: read from ring buffer and write to I2S; output silence on underrun.
// When the remote suspends (track skip, pause, focus change), callback stops and we output
// silence; on resume, applying a short ramp here avoids a 0→full-amplitude pop.
void i2s_feed_task(void* arg) {
    (void)arg;
    static uint8_t silence[kSilenceChunkBytes] = {0};
    static uint8_t fadeout_buf[kSilenceChunkBytes]; // ramp from last sample to 0 on pause
    size_t item_size = 0;
    static uint32_t s_silence_writes = 0;
    static uint32_t s_resume_ramp_left = 0;
    static int16_t s_last_l = 0, s_last_r = 0; // last sample written for pause fade-out
    static uint32_t s_pause_ramp_pos = 0;      // current sample index in pause fade-out (0 = just started)
    static bool s_had_audio_once = false;      // true after first data chunk (so we only mute after pause, not at startup)
    static bool s_pre_bt_underrun_logged = false;  // log underrun only once per burst when pre-BT gain ramp test active
    static bool s_source_silence_logged = false;  // log source silence once per run

    while (true) {
        uint8_t* data = (uint8_t*)xRingbufferReceiveUpTo(g_feed_ring, &item_size, pdMS_TO_TICKS(3), kSilenceChunkBytes);
        if (data != nullptr && item_size > 0) {
            if (g_pre_bt_gain_ramp_test_active) {
                s_pre_bt_underrun_logged = false;  // reset so next underrun is logged
            }
            uint32_t samples_in_chunk = item_size / 4;
            int16_t* p = reinterpret_cast<int16_t*>(data);

            // If source sent silence (zeros), do not forward zeros to I2S (causes pop); replace with fade-to-dither then dither
            int max_abs = 0;
            for (uint32_t i = 0; i < samples_in_chunk; i++) {
                int a = (int)p[i * 2];
                if (a < 0) a = -a;
                if (a > max_abs) max_abs = a;
                a = (int)p[i * 2 + 1];
                if (a < 0) a = -a;
                if (a > max_abs) max_abs = a;
            }
            if (max_abs <= kSilenceDataThreshold) {
                g_dac_volume_target = 0;  // mute quickly so dither avoids pop and no hiss
                if (!s_source_silence_logged) {
                    ESP_LOGI("main", "Source silence detected (max_abs=%d), replacing with fade-to-dither, DAC target=0", max_abs);
                    s_source_silence_logged = true;
                }
                uint32_t fade_len = (samples_in_chunk < kSilenceDataFadeSamples) ? samples_in_chunk : kSilenceDataFadeSamples;
                for (uint32_t i = 0; i < samples_in_chunk; i++) {
                    float t;
                    if (i >= fade_len || fade_len <= 1) {
                        t = 1.0f;
                    } else {
                        t = static_cast<float>(i) / static_cast<float>(fade_len - 1);
                        t = t * t * (3.0f - 2.0f * t);  // smoothstep
                    }
                    int16_t d = dither_lsb();
                    p[i * 2] = (int16_t)((float)s_last_l * (1.0f - t) + (float)d * t);
                    p[i * 2 + 1] = (int16_t)((float)s_last_r * (1.0f - t) + (float)d * t);
                }
                if (samples_in_chunk > 0) {
                    s_last_l = p[(samples_in_chunk - 1) * 2];
                    s_last_r = p[(samples_in_chunk - 1) * 2 + 1];
                }
            }

            // #region agent log
            if (s_silence_writes >= 2) {
                static char s_dbg[80];
                snprintf(s_dbg, sizeof(s_dbg), "{\"silence_writes\":%lu,\"ramp_samples\":%lu}",
                         (unsigned long)s_silence_writes, (unsigned long)kResumeRampSamples);
                DEBUG_LOG("H1", "resume_start", s_dbg);
            }
            // #endregion
            if (s_silence_writes >= 2) {
                s_resume_ramp_left = kResumeRampSamples;
            }
            s_silence_writes = 0;

            if (s_resume_ramp_left > 0) {
                uint32_t ramp_len = (samples_in_chunk < s_resume_ramp_left) ? samples_in_chunk : s_resume_ramp_left;
                for (uint32_t i = 0; i < ramp_len; i++) {
                    float g;
                    if (ramp_len <= 1) {
                        g = 1.0f;
                    } else {
                        float t = static_cast<float>(i) / static_cast<float>(ramp_len - 1);
                        g = t * t * (3.0f - 2.0f * t);  // smoothstep: 0->1 S-curve
                    }
                    p[i * 2] = static_cast<int16_t>(p[i * 2] * g);
                    p[i * 2 + 1] = static_cast<int16_t>(p[i * 2 + 1] * g);
                }
                s_resume_ramp_left -= ramp_len;
                // #region agent log
                if (s_resume_ramp_left == 0) {
                    DEBUG_LOG("H2", "ramp_done", "{}");
                }
                // #endregion
            }

            if (samples_in_chunk > 0 && max_abs > kSilenceDataThreshold) {
                s_last_l = p[(samples_in_chunk - 1) * 2];
                s_last_r = p[(samples_in_chunk - 1) * 2 + 1];
                s_source_silence_logged = false;  // reset so next source silence is logged once
            }
            s_had_audio_once = true;
            if (max_abs > kSilenceDataThreshold) {
                g_dac_volume_target = 100;
            }
            i2s.write(data, item_size);
            vRingbufferReturnItem(g_feed_ring, data);
        } else {
            // No data from ring buffer (underrun / feed task starved)
            if (g_pre_bt_gain_ramp_test_active && !s_pre_bt_underrun_logged) {
                ESP_LOGW("main", "Pre-BT gain ramp test UNDERRUN: feed task starved (ring buffer empty)");
                s_pre_bt_underrun_logged = true;
            }
            const uint32_t samples_per_chunk = kSilenceChunkBytes / 4;
            if (s_silence_writes == 0) {
                s_pause_ramp_pos = 0;
            }
            if (s_pause_ramp_pos < kPauseFadeOutSamples) {
                // Pause fade-out: ramp from last sample to 0 over kPauseFadeOutSamples (multiple chunks)
                if (kPreBTTestCycles == 0 && s_pause_ramp_pos == 0 && s_had_audio_once) {
                    // #region agent log
                    DEBUG_LOG("H6", "first_silence_after_data", "{}");
                    // #endregion
                    g_dac_volume_target = 0;
                }
                int16_t* out = reinterpret_cast<int16_t*>(fadeout_buf);
                for (uint32_t i = 0; i < samples_per_chunk; i++) {
                    uint32_t pos = s_pause_ramp_pos + i;
                    float g;
                    if (pos >= kPauseFadeOutSamples) {
                        g = 0.0f;
                    } else if (kPauseFadeOutSamples <= 1) {
                        g = 0.0f;
                    } else {
                        float t = static_cast<float>(pos) / static_cast<float>(kPauseFadeOutSamples - 1);
                        float sm = t * t * (3.0f - 2.0f * t);  // smoothstep(t)
                        g = 1.0f - sm;  // 1->0 S-curve
                    }
                    int16_t l = static_cast<int16_t>(s_last_l * g);
                    int16_t r = static_cast<int16_t>(s_last_r * g);
                    if (g <= 0.001f) {
                        l = dither_lsb();
                        r = dither_lsb();
                    }
                    out[i * 2] = l;
                    out[i * 2 + 1] = r;
                }
                s_pause_ramp_pos += samples_per_chunk;
                if (s_pause_ramp_pos > kPauseFadeOutSamples) {
                    s_pause_ramp_pos = kPauseFadeOutSamples;
                }
                i2s.write(fadeout_buf, kSilenceChunkBytes);
            } else {
                if (s_had_audio_once) {
                    g_dac_volume_target = 0;
                }
                // Steady silence: ±1 LSB dither so DAC doesn't sit at exact zero
                int16_t* s = reinterpret_cast<int16_t*>(silence);
                for (uint32_t i = 0; i < samples_per_chunk; i++) {
                    int16_t d = dither_lsb();
                    s[i * 2] = d;
                    s[i * 2 + 1] = d;
                }
                i2s.write(silence, kSilenceChunkBytes);
            }
            s_silence_writes++;
        }
    }
}

void setup() {
    // Initialize M5Unified
    auto cfg = M5.config();
    M5.begin(cfg);

    // Run reset-recovery clamp as early as possible.
    // This handles the worst case where reset button is pressed while DAC is loud and unmuted.
    Wire.begin(SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN, 400000L);
    if (!es8388_apply_reset_safety_clamp()) {
        ESP_LOGW("main", "Early ES8388 safety clamp failed");
    }

    startup_step("S01", "M5.begin");

    Serial.begin(115200);
    delay(1000);
    startup_step("S02", "Serial.begin");

    // Reduce log level to avoid performance issues
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("BT_AV", ESP_LOG_WARN); // Suppress frequent BT logs

    ESP_LOGI("main", "=== Phase 3: BT + Module Audio (I2SStream approach) ===");
    ESP_LOGI("main", "Available Heap: %zu", esp_get_free_heap_size());

    // Display setup
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.println("\nM5Blue - Phase 3");
    M5.Display.println("I2S feed task + silence on underrun");
    M5.Display.println("");

    // Initialize dual button pins
    pinMode(DUAL_BUTTON_BLUE, INPUT);
    pinMode(DUAL_BUTTON_RED, INPUT);

    // Scan I2C bus
    ESP_LOGI("main", "Scanning I2C bus...");
    Wire.begin(SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN, 400000L);
    for (int addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            ESP_LOGI("main", "Found I2C device at address 0x%02X", addr);
        }
    }
    startup_step("S03", "I2C_scan");

    // Initialize Module Audio I2C device
    ESP_LOGI("main", "Initializing AudioI2c device...");
    if (!device.begin(&Wire, SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN)) {
        ESP_LOGW("main", "AudioI2c device not found");
        M5.Display.setTextColor(YELLOW);
        M5.Display.println("AudioI2c: N/A");
    } else {
        ESP_LOGI("main", "AudioI2c OK");
        M5.Display.setTextColor(GREEN);
        M5.Display.println("AudioI2c: OK");
        device.setHPMode(AUDIO_HPMODE_NATIONAL);
        device.setRGBBrightness(50);
        device.setRGBLED(0, 0x0000FF); // Blue - waiting
    }
    startup_step("S04", "AudioI2c");

    // Initialize codec after early clamp. Keep volume at minimum until all stream settings are stable.
    ESP_LOGI("main", "Initializing ES8388 codec...");
    if (!es8388.init()) {
        ESP_LOGE("main", "Failed to initialize ES8388!");
        M5.Display.setTextColor(RED);
        M5.Display.println("ES8388: FAILED!");
    } else {
        ESP_LOGI("main", "ES8388 OK");
        M5.Display.setTextColor(GREEN);
        M5.Display.println("ES8388: OK");
    }
    startup_step("S05", "ES8388_init");

    // Keep output at minimum until all stream settings are stable.
    es8388.setDACOutput(DAC_OUTPUT_OUT1);
    es8388.setDACVolume(0);
    es8388.setBitsSample(ES_MODULE_DAC, BIT_LENGTH_16BITS);
    es8388.setSampleRate(SAMPLE_RATE_44K);
    startup_step("S06", "DAC_config_volume0");

    // Clear DAC mute via direct RMW on Reg.25 bit2, preserving SoftRamp + slow ramp-rate.
    if (!es8388_set_dac_mute_direct(false)) {
        ESP_LOGW("main", "Failed to unmute ES8388 DAC (direct)");
    }
    startup_step("S07", "DAC_unmute");

    // Configure MCLK output on GPIO0 (required for ES8388)
    ESP_LOGI("main", "Configuring MCLK output on GPIO0...");
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
    startup_step("S08", "MCLK_GPIO0");

    // Initialize ring buffer for delay effect
    ESP_LOGI("main", "Initializing ring buffer...");
    g_ring = new RingBufferInterleaved();
    ESP_LOGI("main", "Ring buffer OK");
    startup_step("S09", "ring_buffer");

    // Disable internal speaker (use Module Audio instead)
    M5.Speaker.end();
    startup_step("S10", "M5.Speaker.end");

    // Configure I2SStream for Module Audio
    // Must call end() first to clear default config, then reconfigure
    ESP_LOGI("main", "Configuring I2SStream for Module Audio...");
    i2s.end(); // Clear any default config
    startup_step("S11", "i2s.end");
    auto i2s_cfg = i2s.defaultConfig();
    i2s_cfg.pin_bck = SYS_I2S_SCLK_PIN;  // GPIO 19
    i2s_cfg.pin_ws = SYS_I2S_LRCK_PIN;   // GPIO 27
    i2s_cfg.pin_data = SYS_I2S_DOUT_PIN; // GPIO 2
    i2s.begin(i2s_cfg);
    ESP_LOGI("main", "I2SStream configured: BCK=%d, WS=%d, DATA=%d",
             SYS_I2S_SCLK_PIN, SYS_I2S_LRCK_PIN, SYS_I2S_DOUT_PIN);
    startup_step("S12", "i2s.begin");

    // I2S feed: we own the only path to I2S; library must not write
    a2dp_sink.set_stream_reader(nullptr, false);

    // Ring buffer for PCM feed (producer = audio_callback, consumer = i2s_feed_task)
    g_feed_ring = xRingbufferCreate(kFeedRingBufBytes, RINGBUF_TYPE_BYTEBUF);
    if (g_feed_ring == nullptr) {
        ESP_LOGE("main", "Feed ring buffer create failed");
    } else {
        ESP_LOGI("main", "Feed ring buffer: %u bytes", (unsigned)kFeedRingBufBytes);
        BaseType_t ok = xTaskCreatePinnedToCore(i2s_feed_task, "i2s_feed", 4096, nullptr, 5, &g_i2s_feed_task_handle, 1);
        if (ok != pdPASS) {
            ESP_LOGE("main", "I2S feed task create failed");
        } else {
            ESP_LOGI("main", "I2S feed task started (silence on underrun)");
        }
    }
    startup_step("S13", "feed_ring_and_task");

    // Pre-BT test: sine -> silence cycles to locate hiss (same I2S path as A2DP)
    if (kPreBTTestCycles > 0 && g_feed_ring != nullptr) {
        const size_t chunks_per_sec = (size_t)((kSampleRate * 2 * 2) / kSilenceChunkBytes); // ~172
        static const float kPreBTTestSineHz = 440.0f;
        static const float kPreBTTestSineAmplitude = 0.15f;
        static uint8_t sine_chunk[kSilenceChunkBytes];
        static uint32_t phase_sample = 0;

        if (kPreBTTestUseGainRamp != 0) {
            // Gain-ramp test: always output sine, only gain changes (no buffer switch). If pop still
            // occurs at envelope edges, cause is likely analog (codec/amp transient), not digital.
            // Per cycle: 2s gain=1, 2s gain 1->0 (per-sample), 2s gain=0, 2s gain 0->1 (per-sample); repeat.
            const uint32_t samples_per_phase = (uint32_t)(2 * kSampleRate);  // stereo samples in 2s = 88200
            const size_t samples_per_chunk = kSilenceChunkBytes / 4;         // 256
            // Ensure we output exactly samples_per_phase samples per phase (last chunk may be partial)
            const size_t chunks_2s = (samples_per_phase + samples_per_chunk - 1) / samples_per_chunk;  // 345
            ESP_LOGI("main", "Pre-BT test: gain ramp x%d (2s full, 2s fade-out, 2s zero, 2s fade-in), DAC mute OFF", kPreBTTestCycles);
            M5.Display.setTextColor(YELLOW);
            M5.Display.printf("\nTest: gain ramp x%d", kPreBTTestCycles);
            g_pre_bt_gain_ramp_test_active = true;

            for (int c = 0; c < kPreBTTestCycles; c++) {
                ESP_LOGI("main", "Pre-BT test cycle %d/%d", c + 1, kPreBTTestCycles);
                for (size_t p = 0; p < 4; p++) {  // phase: 0=full, 1=fade-out, 2=zero, 3=fade-in
                    uint32_t phase_gain_sample = 0;  // sample index within this phase (0 .. samples_per_phase-1)
                    for (size_t i = 0; i < chunks_2s; i++) {
                        uint32_t remaining = samples_per_phase - phase_gain_sample;
                        size_t samples_this_chunk = (remaining < (uint32_t)samples_per_chunk)
                            ? (size_t)remaining : samples_per_chunk;
                        for (size_t j = 0; j < samples_this_chunk; j++) {
                            uint32_t s = phase_gain_sample + (uint32_t)j;
                            float gain;
                            if (p == 0) {
                                gain = 1.0f;
                            } else if (p == 1) {
                                // Fade-out: gain 1 -> 0 over 2s, S-curve (smoothstep) per-sample
                                if (s >= samples_per_phase) {
                                    gain = 0.0f;
                                } else if (samples_per_phase <= 1u) {
                                    gain = 0.0f;
                                } else {
                                    float t = static_cast<float>(s) / static_cast<float>(samples_per_phase - 1u);
                                    float sm = t * t * (3.0f - 2.0f * t);  // smoothstep(t)
                                    gain = 1.0f - sm;
                                }
                            } else if (p == 2) {
                                gain = 0.0f;
                            } else {
                                // Fade-in: gain 0 -> 1 over 2s, S-curve (smoothstep) per-sample
                                if (s >= samples_per_phase) {
                                    gain = 1.0f;
                                } else if (samples_per_phase <= 1u) {
                                    gain = 1.0f;
                                } else {
                                    float t = static_cast<float>(s) / static_cast<float>(samples_per_phase - 1u);
                                    gain = t * t * (3.0f - 2.0f * t);  // smoothstep(t)
                                }
                            }
                            uint32_t n = phase_sample + (uint32_t)j;
                            float v = gain * kPreBTTestSineAmplitude * 32767.0f
                                      * sinf(2.0f * 3.14159265f * kPreBTTestSineHz * (float)n / (float)kSampleRate);
                            int16_t samp = (int16_t)v;
                            // Zero phase only: ±1 LSB dither (shared LFSR) to spread spectrum
                            if (p == 2) {
                                samp = dither_lsb();
                            }
                            size_t off = j * 4;
                            sine_chunk[off] = (uint8_t)(samp & 0xff);
                            sine_chunk[off + 1] = (uint8_t)(samp >> 8);
                            sine_chunk[off + 2] = (uint8_t)(samp & 0xff);
                            sine_chunk[off + 3] = (uint8_t)(samp >> 8);
                        }
                        phase_sample += (uint32_t)samples_this_chunk;
                        phase_gain_sample += (uint32_t)samples_this_chunk;
                        xRingbufferSend(g_feed_ring, sine_chunk, samples_this_chunk * 4, portMAX_DELAY);
                    }
                }
            }
            g_pre_bt_gain_ramp_test_active = false;
        } else {
            // Original test: switch between sine buffer and silence buffer (short fades at boundaries)
            ESP_LOGI("main", "Pre-BT test: %d x (sine %ds + silence %ds), DAC mute OFF", kPreBTTestCycles, kPreBTTestSineSec, kPreBTTestSilenceSec);
            M5.Display.setTextColor(YELLOW);
            M5.Display.printf("\nTest: sine/silence x%d (DAC mute OFF)", kPreBTTestCycles);
            static const size_t kPreBTTestFadeInChunks = 3;
            static const size_t kPreBTTestFadeOutChunks = 3;
            static uint8_t silence_chunk[kSilenceChunkBytes] = {0};
            const size_t sine_chunks = (size_t)kPreBTTestSineSec * chunks_per_sec;
            const size_t silence_chunks = (size_t)kPreBTTestSilenceSec * chunks_per_sec;

            for (int c = 0; c < kPreBTTestCycles; c++) {
                ESP_LOGI("main", "Pre-BT test cycle %d: sine %ds", c + 1, kPreBTTestSineSec);
                for (size_t i = 0; i < sine_chunks; i++) {
                    float gain = 1.0f;
                    if (i < kPreBTTestFadeInChunks && sine_chunks > kPreBTTestFadeInChunks) {
                        gain = (float)(i + 1) / (float)kPreBTTestFadeInChunks;
                    } else if (i >= sine_chunks - kPreBTTestFadeOutChunks && sine_chunks > kPreBTTestFadeOutChunks) {
                        size_t fade_idx = i - (sine_chunks - kPreBTTestFadeOutChunks);
                        gain = 1.0f - (float)(fade_idx + 1) / (float)kPreBTTestFadeOutChunks;
                    }
                    for (size_t j = 0; j < kSilenceChunkBytes / 4; j++) {
                        uint32_t n = phase_sample + (uint32_t)j;
                        float v = gain * kPreBTTestSineAmplitude * 32767.0f
                                  * sinf(2.0f * 3.14159265f * kPreBTTestSineHz * (float)n / (float)kSampleRate);
                        int16_t s = (int16_t)v;
                        size_t off = j * 4;
                        sine_chunk[off] = (uint8_t)(s & 0xff);
                        sine_chunk[off + 1] = (uint8_t)(s >> 8);
                        sine_chunk[off + 2] = (uint8_t)(s & 0xff);
                        sine_chunk[off + 3] = (uint8_t)(s >> 8);
                    }
                    phase_sample += (uint32_t)(kSilenceChunkBytes / 4);
                    xRingbufferSend(g_feed_ring, sine_chunk, kSilenceChunkBytes, portMAX_DELAY);
                }
                ESP_LOGI("main", "Pre-BT test cycle %d: silence %ds", c + 1, kPreBTTestSilenceSec);
                for (size_t i = 0; i < silence_chunks; i++) {
                    xRingbufferSend(g_feed_ring, silence_chunk, kSilenceChunkBytes, portMAX_DELAY);
                }
            }
        }
        ESP_LOGI("main", "Pre-BT test done.");
        M5.Display.setTextColor(WHITE);
    }

    // Setup A2DP callbacks
    a2dp_sink.set_on_connection_state_changed(connection_state_callback);
    a2dp_sink.set_on_audio_state_changed(audio_state_callback_debug, nullptr);
    a2dp_sink.set_avrc_rn_track_change_callback(track_change_callback);

    // Set raw stream reader/writer: process audio and push to feed ring
    a2dp_sink.set_raw_stream_reader_writer(audio_callback);

    // Start A2DP sink
    ESP_LOGI("main", "Starting Bluetooth A2DP Sink...");
    M5.Display.setTextColor(CYAN);
    M5.Display.println("\nStarting BT...");
    startup_step("S14", "before_a2dp.start");
    a2dp_sink.start("M5Blue");
    startup_step("S15", "after_a2dp.start");

    ESP_LOGI("main", "Setup complete. Waiting for Bluetooth connection...");
    M5.Display.setTextColor(WHITE);
    M5.Display.println("Name: M5Blue");

    // Set LED to indicate ready
    device.setRGBLED(0, 0x00FF00); // Green - ready
    device.setRGBLED(1, 0x00FF00);
    device.setRGBLED(2, 0x00FF00);
}

void loop() {
    M5.update();

    // DAC volume target is set by i2s_feed_task (0 or 100); ramp is applied here
    service_dac_volume_ramp();

    // Read dual button states
    bool blue_pressed = (digitalRead(DUAL_BUTTON_BLUE) == LOW);
    bool red_pressed = (digitalRead(DUAL_BUTTON_RED) == LOW);

    // Effect Blue: Volume reduction
    if (blue_pressed && !g_effect_blue) {
        ESP_LOGI("main", "Effect Blue ON (Volume 30%%)");
        g_effect_blue = true;
        device.setRGBLED(0, 0x0000FF); // Blue LED
    } else if (!blue_pressed && g_effect_blue) {
        ESP_LOGI("main", "Effect Blue OFF");
        g_effect_blue = false;
        device.setRGBLED(0, 0x00FF00); // Green LED
    }

    // Effect Red: Delay
    if (red_pressed && !g_effect_red) {
        ESP_LOGI("main", "Effect Red ON (Delay)");
        g_effect_red = true;
        if (g_ring) {
            g_ring->syncPositon();
            g_ring->advanceReadPosition(-44100); // ~1 second delay
        }
        device.setRGBLED(2, 0xFF0000); // Red LED
    } else if (!red_pressed && g_effect_red) {
        ESP_LOGI("main", "Effect Red OFF");
        g_effect_red = false;
        device.setRGBLED(2, 0x00FF00); // Green LED
    }

    // Small delay to prevent tight loop
    delay(10);
}
