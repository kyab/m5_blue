// Phase 3: Bluetooth A2DP + Module Audio + queued I2S processing

#include <M5Unified.h>
#include "audio_i2c.hpp"
#include "es8388.hpp"
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include "RingBuffer.hpp"
#include "DJFilter.hpp"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// MCLK output configuration (required for ES8388)
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"

// Pin assignment for: M5Stack Core2 + Module Audio + 2x Bottom stack.
// References: docs.m5stack.com/en/core/core2, docs.m5stack.com/en/module/Module-Audio,
//             docs.m5stack.com/en/base/m5go_bottom2 (Port B/C on Bottom).
//
// Core2 has only one Grove on the side: PORT.A (G32/G33). PORT.B (G26/G36) and PORT.C (G13/G14)
// are brought out by the Bottom module(s), not on Core2 body.
// Core2 internal: G1/G3 (USB serial), G21/G22 (I2C: Touch/RTC/IMU), G0/G2/G34 (NS4168/mic —
//   we release with M5.Speaker.end() and use G0/G2/G34 for Module Audio I2S).
// Core2 M-Bus: G36=pin4, G32/G33=pin19/20, G27/G19=pin21/22, G2/G0=pin23/24, G34=pin26.
// Module Audio uses M-Bus: I2C 17/18 (G21/G22), I2S on 21,22,23,24,26 (G27,G19,G2,G0,G34). Does not use G36.
// Bottom (e.g. M5GO Bottom2) exposes PORT.B (G26 DAC, G36 ADC) and PORT.C (G13/G14 UART).
#define SYS_I2C_SDA_PIN 21
#define SYS_I2C_SCL_PIN 22

#define SYS_I2S_MCLK_PIN 0
#define SYS_I2S_SCLK_PIN 19
#define SYS_I2S_LRCK_PIN 27
#define SYS_I2S_DOUT_PIN 2
#define SYS_I2S_DIN_PIN 34

// Dual button: Core2 side PORT.A only (G32/G33)
#define DUAL_BUTTON_BLUE 33
#define DUAL_BUTTON_RED 32

// Rotation angle unit (M5STACK-U005): connect to PORT.B (black, analog) on the Bottom module; G36 = ADC
#define ROTATION_ANGLE_GPIO 36

// Audio I2C device (for RGB LED, HP detect, etc.)
AudioI2c device;

// ES8388 codec
ES8388 es8388(&Wire, SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN);

static const uint32_t kSampleRate = 44100;
static const uint32_t kStereoChannels = 2;

// I2SStream for audio output (fed by a single writer task)
I2SStream i2s;

static const uint32_t kI2SWriterFrames = 256;       // ~5.8 ms at 44.1 kHz; caps resume latency
static const size_t kI2SWriterBytes = kI2SWriterFrames * kStereoChannels * sizeof(int16_t);
static const size_t kBtPcmRingBytes = 32768;        // ~186 ms stereo 16-bit queue; absorbs A2DP burst jitter
static const size_t kBtPcmPrebufferBytes = 24576;   // ~139 ms: resume into the stable high-water region
static const uint32_t kBtPcmPushWaitMs = 20;        // Restore bounded back-pressure before dropping PCM
static uint8_t s_bt_pcm_ring[kBtPcmRingBytes];
static size_t s_bt_pcm_head = 0;
static size_t s_bt_pcm_tail = 0;
static size_t s_bt_pcm_used = 0;
static portMUX_TYPE s_bt_pcm_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_i2s_writer_bytes[kI2SWriterBytes];
static TaskHandle_t s_i2s_writer_task = nullptr;
static_assert(sizeof(s_i2s_writer_bytes) == kI2SWriterBytes, "I2S writer buffer size mismatch");

extern volatile esp_a2d_audio_state_t g_a2dp_audio_state;
static size_t bt_pcm_ring_used();

static inline size_t frame_align_bytes(size_t bytes) {
    return bytes & ~(size_t)(2 * sizeof(int16_t) - 1);
}

struct AudioStats {
    volatile uint32_t callback_count = 0;
    volatile uint32_t callback_samples_min = UINT32_MAX;
    volatile uint32_t callback_samples_max = 0;
    volatile uint32_t callback_gap_us_min = UINT32_MAX;
    volatile uint32_t callback_gap_us_max = 0;
    volatile uint32_t ring_used_min = UINT32_MAX;
    volatile uint32_t ring_used_max = 0;
    volatile uint32_t ring_drop_count = 0;
    volatile uint32_t ring_drop_bytes = 0;
    volatile uint32_t writer_pcm_blocks = 0;
    volatile uint32_t writer_dither_blocks = 0;
    volatile uint32_t writer_rebuffer_count = 0;
    volatile uint32_t writer_write_count = 0;
    volatile uint32_t writer_write_us_min = UINT32_MAX;
    volatile uint32_t writer_write_us_max = 0;
    volatile uint32_t writer_write_us_sum = 0;
};

static AudioStats s_audio_stats;
static portMUX_TYPE s_audio_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_last_callback_us = 0;

static inline void stats_minmax_u32(volatile uint32_t& min_v, volatile uint32_t& max_v, uint32_t value) {
    if (value < min_v) min_v = value;
    if (value > max_v) max_v = value;
}

static void stats_note_callback(uint32_t sample_num) {
    uint32_t now_us = (uint32_t)micros();
    portENTER_CRITICAL(&s_audio_stats_mux);
    s_audio_stats.callback_count++;
    stats_minmax_u32(s_audio_stats.callback_samples_min, s_audio_stats.callback_samples_max, sample_num);
    if (s_last_callback_us != 0) {
        stats_minmax_u32(s_audio_stats.callback_gap_us_min, s_audio_stats.callback_gap_us_max, now_us - s_last_callback_us);
    }
    s_last_callback_us = now_us;
    portEXIT_CRITICAL(&s_audio_stats_mux);
}

static void stats_reset_callback_gap() {
    portENTER_CRITICAL(&s_audio_stats_mux);
    s_last_callback_us = 0;
    portEXIT_CRITICAL(&s_audio_stats_mux);
}

static void stats_note_ring_used(size_t used) {
    uint32_t used_u32 = (used > UINT32_MAX) ? UINT32_MAX : (uint32_t)used;
    portENTER_CRITICAL(&s_audio_stats_mux);
    stats_minmax_u32(s_audio_stats.ring_used_min, s_audio_stats.ring_used_max, used_u32);
    portEXIT_CRITICAL(&s_audio_stats_mux);
}

static void stats_note_ring_drop(size_t bytes) {
    uint32_t bytes_u32 = (bytes > UINT32_MAX) ? UINT32_MAX : (uint32_t)bytes;
    portENTER_CRITICAL(&s_audio_stats_mux);
    s_audio_stats.ring_drop_count++;
    s_audio_stats.ring_drop_bytes += bytes_u32;
    portEXIT_CRITICAL(&s_audio_stats_mux);
}

static void stats_note_writer(bool pcm_block, bool rebuffered, uint32_t write_us) {
    portENTER_CRITICAL(&s_audio_stats_mux);
    if (pcm_block) s_audio_stats.writer_pcm_blocks++;
    else s_audio_stats.writer_dither_blocks++;
    if (rebuffered) s_audio_stats.writer_rebuffer_count++;
    s_audio_stats.writer_write_count++;
    s_audio_stats.writer_write_us_sum += write_us;
    stats_minmax_u32(s_audio_stats.writer_write_us_min, s_audio_stats.writer_write_us_max, write_us);
    portEXIT_CRITICAL(&s_audio_stats_mux);
}

static void stats_note_rebuffer() {
    portENTER_CRITICAL(&s_audio_stats_mux);
    s_audio_stats.writer_rebuffer_count++;
    portEXIT_CRITICAL(&s_audio_stats_mux);
}

struct ControlLoopStats {
    uint32_t loop_count = 0;
    uint32_t loop_gap_us_min = UINT32_MAX;
    uint32_t loop_gap_us_max = 0;
    uint32_t loop_gap_us_sum = 0;
    uint32_t loop_body_us_min = UINT32_MAX;
    uint32_t loop_body_us_max = 0;
    uint32_t loop_body_us_sum = 0;
    uint32_t m5_update_us_min = UINT32_MAX;
    uint32_t m5_update_us_max = 0;
    uint32_t m5_update_us_sum = 0;
    uint32_t button_us_min = UINT32_MAX;
    uint32_t button_us_max = 0;
    uint32_t button_us_sum = 0;
    uint32_t adc_us_min = UINT32_MAX;
    uint32_t adc_us_max = 0;
    uint32_t adc_us_sum = 0;
    uint32_t display_us_min = UINT32_MAX;
    uint32_t display_us_max = 0;
    uint32_t display_us_sum = 0;
    uint32_t display_update_count = 0;
    uint32_t audio_stats_us_min = UINT32_MAX;
    uint32_t audio_stats_us_max = 0;
    uint32_t audio_stats_us_sum = 0;
};

static ControlLoopStats s_control_stats;
static uint32_t s_last_loop_start_us = 0;

static inline void control_stats_note_us(uint32_t& min_v, uint32_t& max_v, uint32_t& sum_v, uint32_t value) {
    if (value < min_v) min_v = value;
    if (value > max_v) max_v = value;
    sum_v += value;
}

static void dump_control_stats_if_due() {
    static uint32_t s_last_dump_ms = 0;
    uint32_t now_ms = (uint32_t)millis();
    if (now_ms - s_last_dump_ms < 1000) return;
    s_last_dump_ms = now_ms;

    ControlLoopStats stats = s_control_stats;
    s_control_stats = ControlLoopStats();
    if (stats.loop_count == 0) return;

    uint32_t loop_gap_min = (stats.loop_gap_us_min == UINT32_MAX) ? 0 : stats.loop_gap_us_min;
    uint32_t loop_body_min = (stats.loop_body_us_min == UINT32_MAX) ? 0 : stats.loop_body_us_min;
    uint32_t m5_update_min = (stats.m5_update_us_min == UINT32_MAX) ? 0 : stats.m5_update_us_min;
    uint32_t button_min = (stats.button_us_min == UINT32_MAX) ? 0 : stats.button_us_min;
    uint32_t adc_min = (stats.adc_us_min == UINT32_MAX) ? 0 : stats.adc_us_min;
    uint32_t display_min = (stats.display_us_min == UINT32_MAX) ? 0 : stats.display_us_min;
    uint32_t audio_stats_min = (stats.audio_stats_us_min == UINT32_MAX) ? 0 : stats.audio_stats_us_min;
    uint32_t display_avg = (stats.display_update_count == 0) ? 0 : stats.display_us_sum / stats.display_update_count;

    ESP_LOGI("control_stats",
             "loops=%lu gap_us=%lu/%lu/%lu body_us=%lu/%lu/%lu m5_us=%lu/%lu/%lu button_us=%lu/%lu/%lu adc_us=%lu/%lu/%lu display_us=%lu/%lu/%lu display_n=%lu audio_stats_us=%lu/%lu/%lu",
             (unsigned long)stats.loop_count,
             (unsigned long)loop_gap_min,
             (unsigned long)(stats.loop_gap_us_sum / stats.loop_count),
             (unsigned long)stats.loop_gap_us_max,
             (unsigned long)loop_body_min,
             (unsigned long)(stats.loop_body_us_sum / stats.loop_count),
             (unsigned long)stats.loop_body_us_max,
             (unsigned long)m5_update_min,
             (unsigned long)(stats.m5_update_us_sum / stats.loop_count),
             (unsigned long)stats.m5_update_us_max,
             (unsigned long)button_min,
             (unsigned long)(stats.button_us_sum / stats.loop_count),
             (unsigned long)stats.button_us_max,
             (unsigned long)adc_min,
             (unsigned long)(stats.adc_us_sum / stats.loop_count),
             (unsigned long)stats.adc_us_max,
             (unsigned long)display_min,
             (unsigned long)display_avg,
             (unsigned long)stats.display_us_max,
             (unsigned long)stats.display_update_count,
             (unsigned long)audio_stats_min,
             (unsigned long)(stats.audio_stats_us_sum / stats.loop_count),
             (unsigned long)stats.audio_stats_us_max);
}

static void dump_audio_stats_if_due() {
    static uint32_t s_last_dump_ms = 0;
    uint32_t now_ms = (uint32_t)millis();
    if (now_ms - s_last_dump_ms < 1000) return;
    s_last_dump_ms = now_ms;

    AudioStats stats;
    portENTER_CRITICAL(&s_audio_stats_mux);
    stats = s_audio_stats;
    s_audio_stats = AudioStats();
    portEXIT_CRITICAL(&s_audio_stats_mux);

    uint32_t cb_min = (stats.callback_samples_min == UINT32_MAX) ? 0 : stats.callback_samples_min;
    uint32_t cb_gap_min = (stats.callback_gap_us_min == UINT32_MAX) ? 0 : stats.callback_gap_us_min;
    uint32_t ring_min = (stats.ring_used_min == UINT32_MAX) ? 0 : stats.ring_used_min;
    uint32_t write_min = (stats.writer_write_us_min == UINT32_MAX) ? 0 : stats.writer_write_us_min;
    uint32_t write_avg = (stats.writer_write_count == 0) ? 0 : stats.writer_write_us_sum / stats.writer_write_count;
    size_t ring_now = bt_pcm_ring_used();

    ESP_LOGI("audio_stats",
             "cb=%lu samples=%lu..%lu gap_us=%lu..%lu ring_now=%u ring=%lu..%lu drop=%lu/%luB writer_pcm=%lu dither=%lu rebuf=%lu write_us=%lu/%lu/%lu state=%d",
             (unsigned long)stats.callback_count,
             (unsigned long)cb_min,
             (unsigned long)stats.callback_samples_max,
             (unsigned long)cb_gap_min,
             (unsigned long)stats.callback_gap_us_max,
             (unsigned)ring_now,
             (unsigned long)ring_min,
             (unsigned long)stats.ring_used_max,
             (unsigned long)stats.ring_drop_count,
             (unsigned long)stats.ring_drop_bytes,
             (unsigned long)stats.writer_pcm_blocks,
             (unsigned long)stats.writer_dither_blocks,
             (unsigned long)stats.writer_rebuffer_count,
             (unsigned long)write_min,
             (unsigned long)write_avg,
             (unsigned long)stats.writer_write_us_max,
             (int)g_a2dp_audio_state);
}

static void bt_pcm_ring_drop_locked(size_t bytes) {
    bytes = frame_align_bytes(bytes);
    if (bytes > s_bt_pcm_used) bytes = s_bt_pcm_used;
    s_bt_pcm_tail = (s_bt_pcm_tail + bytes) % kBtPcmRingBytes;
    s_bt_pcm_used -= bytes;
}

static size_t bt_pcm_ring_push(const uint8_t* data, size_t len) {
    len = frame_align_bytes(len);
    if (len == 0) return 0;
    if (len > kBtPcmRingBytes) {
        data += len - kBtPcmRingBytes;
        len = kBtPcmRingBytes;
    }

    uint32_t wait_start_ms = (uint32_t)millis();
    for (;;) {
        portENTER_CRITICAL(&s_bt_pcm_mux);
        size_t free_bytes = kBtPcmRingBytes - s_bt_pcm_used;
        portEXIT_CRITICAL(&s_bt_pcm_mux);
        if (free_bytes >= len) break;
        if ((uint32_t)millis() - wait_start_ms >= kBtPcmPushWaitMs) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    portENTER_CRITICAL(&s_bt_pcm_mux);
    size_t free_bytes = kBtPcmRingBytes - s_bt_pcm_used;
    if (free_bytes < len) {
        // Last resort: keep latency bounded if Bluetooth still outruns I2S after bounded back-pressure.
        stats_note_ring_drop(len - free_bytes);
        bt_pcm_ring_drop_locked(len - free_bytes);
    }

    size_t first = std::min(len, kBtPcmRingBytes - s_bt_pcm_head);
    memcpy(s_bt_pcm_ring + s_bt_pcm_head, data, first);
    if (len > first) {
        memcpy(s_bt_pcm_ring, data + first, len - first);
    }
    s_bt_pcm_head = (s_bt_pcm_head + len) % kBtPcmRingBytes;
    s_bt_pcm_used += len;
    stats_note_ring_used(s_bt_pcm_used);
    portEXIT_CRITICAL(&s_bt_pcm_mux);
    return len;
}

static size_t bt_pcm_ring_pop(uint8_t* data, size_t max_len) {
    max_len = frame_align_bytes(max_len);
    portENTER_CRITICAL(&s_bt_pcm_mux);
    size_t len = std::min(max_len, s_bt_pcm_used);
    len = frame_align_bytes(len);
    if (len > 0) {
        size_t first = std::min(len, kBtPcmRingBytes - s_bt_pcm_tail);
        memcpy(data, s_bt_pcm_ring + s_bt_pcm_tail, first);
        if (len > first) {
            memcpy(data + first, s_bt_pcm_ring, len - first);
        }
        s_bt_pcm_tail = (s_bt_pcm_tail + len) % kBtPcmRingBytes;
        s_bt_pcm_used -= len;
    }
    portEXIT_CRITICAL(&s_bt_pcm_mux);
    return len;
}

static size_t bt_pcm_ring_used() {
    portENTER_CRITICAL(&s_bt_pcm_mux);
    size_t used = s_bt_pcm_used;
    portEXIT_CRITICAL(&s_bt_pcm_mux);
    return used;
}

static void bt_pcm_ring_clear() {
    portENTER_CRITICAL(&s_bt_pcm_mux);
    s_bt_pcm_head = 0;
    s_bt_pcm_tail = 0;
    s_bt_pcm_used = 0;
    portEXIT_CRITICAL(&s_bt_pcm_mux);
}

class BluetoothA2DPOutputQueuedI2S : public BluetoothA2DPOutput {
public:
    bool begin() override { return true; }
    void end() override {}
    void set_sample_rate(int rate) override {
        if (rate != (int)kSampleRate) {
            ESP_LOGW("a2dp", "A2DP sample rate %d differs from fixed I2S rate %lu", rate, (unsigned long)kSampleRate);
        }
    }
    void set_output_active(bool active) override {
        ESP_LOGI("a2dp", "queued output active=%d (I2S writer stays running)", active);
    }
    size_t write(const uint8_t* data, size_t len) override {
        bt_pcm_ring_push(data, len);
        return len; // Do not block the A2DP callback on I2S back-pressure.
    }
};

static BluetoothA2DPOutputQueuedI2S a2dp_output;

class BluetoothA2DPSinkKeepI2S : public BluetoothA2DPSink {
public:
    using BluetoothA2DPSink::BluetoothA2DPSink;

protected:
    void set_i2s_active(bool active) override {
        if (active) {
            BluetoothA2DPSink::set_i2s_active(true);
        } else {
            // Keep I2S running on remote pause/stop to avoid DAC stop/start pops.
            ESP_LOGI("a2dp", "ignore set_i2s_active(false): keep I2S active");
        }
    }
};

// Bluetooth A2DP Sink with I2SStream
BluetoothA2DPSinkKeepI2S a2dp_sink(a2dp_output);

// Ring buffer for delay effect
RingBufferInterleaved* g_ring = nullptr;

// Effect control targets written by loop() and consumed by the I2S writer task.
volatile bool g_effect_blue = false; // Volume reduction
volatile bool g_effect_red = false;  // Delay effect

// DJ Filter (Going-Zero port): knob-driven LPF/HPF crossover
static DJFilter g_dj_filter;
static volatile float g_dj_filter_target_value = 0.0f;
// Scratch buffers for int16<->float deinterleave in the I2S writer task.
static float s_dj_left[kI2SWriterFrames];
static float s_dj_right[kI2SWriterFrames];

// When the A2DP source sends long runs of digital silence (pause, track gap, app mute),
// some DAC paths treat "all zero" PCM as an idle state and create audible clicks at
// resume. If a block's peak is below this threshold, mix in sub-audible TPDF dither
// so the I2S stream never stays stuck at exact zero.
static const int32_t kSilenceDitherPeakThreshold = 16;  // |int16| peak per block
static const int32_t kSilenceDitherLsbScale = 2;        // ~2 LSB RMS; >>15 after TPDF

volatile esp_a2d_connection_state_t g_bt_connection_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
volatile esp_a2d_audio_state_t g_a2dp_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
volatile bool g_bt_state_display_dirty = true;

static void update_bt_status_display() {
    if (!g_bt_state_display_dirty) return;
    g_bt_state_display_dirty = false;
    const char* state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
    esp_a2d_connection_state_t state = g_bt_connection_state;
    M5.Display.fillRect(0, 180, 320, 20, BLACK);
    M5.Display.setCursor(0, 180);
    M5.Display.setTextColor(state == ESP_A2D_CONNECTION_STATE_CONNECTED ? GREEN : YELLOW);
    M5.Display.printf("BT: %s", state_str[state]);
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

static inline int16_t make_tpdf_dither_sample() {
    uint32_t w = esp_random();
    int32_t d = (int32_t)(w & 0x7FFFu) - (int32_t)((w >> 15) & 0x7FFFu);
    return static_cast<int16_t>((d * kSilenceDitherLsbScale) >> 15);
}

static void fill_dither_bytes(uint8_t* data, size_t bytes) {
    int16_t* samples = reinterpret_cast<int16_t*>(data);
    size_t sample_count = bytes / sizeof(int16_t);
    for (size_t i = 0; i < sample_count; i++) {
        samples[i] = make_tpdf_dither_sample();
    }
}

static void add_silence_dither_if_needed(int16_t* data, uint32_t frame_count) {
    int32_t peak = 0;
    const uint32_t total = frame_count * kStereoChannels;
    for (uint32_t i = 0; i < total; i++) {
        int32_t s = data[i];
        if (s < 0) s = -s;
        if (s > peak) peak = s;
    }
    if (peak > kSilenceDitherPeakThreshold) return;

    for (uint32_t i = 0; i < frame_count; i++) {
        int32_t nl = (int32_t)data[i * 2] + make_tpdf_dither_sample();
        int32_t nr = (int32_t)data[i * 2 + 1] + make_tpdf_dither_sample();
        if (nl > 32767) nl = 32767;
        else if (nl < -32768) nl = -32768;
        if (nr > 32767) nr = 32767;
        else if (nr < -32768) nr = -32768;
        data[i * 2] = static_cast<int16_t>(nl);
        data[i * 2 + 1] = static_cast<int16_t>(nr);
    }
}

static void apply_effects_before_i2s(int16_t* data, uint32_t frame_count, bool from_bt_pcm) {
    static bool s_delay_effect_active = false;
    static float s_applied_dj_filter_value = 0.0f;

    bool blue_enabled = g_effect_blue;
    bool red_enabled = g_effect_red;

    if (from_bt_pcm) {
        if (g_ring != nullptr && red_enabled && !s_delay_effect_active) {
            g_ring->syncPositon();
            g_ring->advanceReadPosition(-44100); // ~1 second delay
        }
        s_delay_effect_active = red_enabled;

        // Apply volume effect
        if (blue_enabled) {
            for (uint32_t i = 0; i < frame_count; i++) {
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
            if (red_enabled) {
                g_ring->storeSamples(data, frame_count);
                g_ring->readSamplesTo(data, frame_count);
            } else {
                g_ring->storeSamples(data, frame_count);
            }
        }

        float target_v = g_dj_filter_target_value;
        if (target_v != s_applied_dj_filter_value) {
            g_dj_filter.setFilterValue(target_v);
            s_applied_dj_filter_value = target_v;
        }

        // DJ Filter (Going-Zero): deinterleave int16 -> float, process, reinterleave back with clamp.
        uint32_t n = (frame_count > kI2SWriterFrames) ? kI2SWriterFrames : frame_count;
        for (uint32_t i = 0; i < n; i++) {
            s_dj_left[i] = static_cast<float>(data[i * 2]) / 32768.0f;
            s_dj_right[i] = static_cast<float>(data[i * 2 + 1]) / 32768.0f;
        }
        g_dj_filter.process(s_dj_left, s_dj_right, n);
        for (uint32_t i = 0; i < n; i++) {
            float l = s_dj_left[i] * 32768.0f;
            float r = s_dj_right[i] * 32768.0f;
            if (l > 32767.0f) l = 32767.0f;
            else if (l < -32768.0f) l = -32768.0f;
            if (r > 32767.0f) r = 32767.0f;
            else if (r < -32768.0f) r = -32768.0f;
            data[i * 2] = static_cast<int16_t>(l);
            data[i * 2 + 1] = static_cast<int16_t>(r);
        }
    }

    if (from_bt_pcm) {
        // Idle-zero prevention: add uncorrelated TPDF dither only for (near-)silent blocks.
        add_silence_dither_if_needed(data, frame_count);
    } else if (!red_enabled) {
        s_delay_effect_active = false;
    }
}

static void i2s_writer_task(void* arg) {
    (void)arg;
    const size_t block_bytes = sizeof(s_i2s_writer_bytes);
    const uint32_t block_frames = block_bytes / (kStereoChannels * sizeof(int16_t));
    bool draining_bt_pcm = false;
    ESP_LOGI("i2s_writer", "task started: block=%u bytes frames=%lu", (unsigned)block_bytes, (unsigned long)block_frames);
    for (;;) {
        bool write_bt_pcm = false;
        bool rebuffered = false;
        if (g_a2dp_audio_state == ESP_A2D_AUDIO_STATE_STARTED) {
            size_t queued = bt_pcm_ring_used();
            if (!draining_bt_pcm) {
                if (queued >= kBtPcmPrebufferBytes) {
                    draining_bt_pcm = true;
                }
            }
            if (draining_bt_pcm) {
                if (queued >= block_bytes) {
                    write_bt_pcm = true;
                } else {
                    // Keep draining mode once playback is established; a short underrun should
                    // produce at most one dither block, not a full prebuffer-sized gap.
                    rebuffered = true;
                }
            }
        } else {
            draining_bt_pcm = false;
        }

        if (write_bt_pcm) {
            size_t got = bt_pcm_ring_pop(s_i2s_writer_bytes, block_bytes);
            if (got != block_bytes) {
                draining_bt_pcm = false;
                write_bt_pcm = false;
                rebuffered = true;
                fill_dither_bytes(s_i2s_writer_bytes, block_bytes);
            }
        } else {
            fill_dither_bytes(s_i2s_writer_bytes, block_bytes);
        }
        apply_effects_before_i2s(reinterpret_cast<int16_t*>(s_i2s_writer_bytes), block_frames, write_bt_pcm);
        uint32_t write_start_us = (uint32_t)micros();
        size_t written = i2s.write(s_i2s_writer_bytes, block_bytes);
        uint32_t write_us = (uint32_t)micros() - write_start_us;
        stats_note_writer(write_bt_pcm && written == block_bytes, rebuffered, write_us);
    }
}

// Audio callback - keep Bluetooth ingress lightweight; effects run immediately before I2S writes.
void audio_callback(int16_t* data, uint32_t sample_num) {
    (void)data;
    stats_note_callback(sample_num);

    static bool first_call = true;
    if (first_call) {
        ESP_LOGI("audio", "*** audio_callback FIRST CALL *** samples=%lu", sample_num);
        first_call = false;
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
    g_bt_connection_state = state;
    g_bt_state_display_dirty = true;

}

// #region agent log
void audio_state_callback_debug(esp_a2d_audio_state_t state, void* obj) {
    (void)obj;
    g_a2dp_audio_state = state;
    stats_reset_callback_gap();
    if (state != ESP_A2D_AUDIO_STATE_STARTED) {
        bt_pcm_ring_clear();
    }
    static char s_dbg[48];
    snprintf(s_dbg, sizeof(s_dbg), "{\"state\":%d}", (int)state);
    DEBUG_LOG("H5", "a2dp_audio_state", s_dbg);
}
// #endregion

void setup() {
    // Initialize M5Unified
    auto cfg = M5.config();
    M5.begin(cfg);
    startup_step("S01", "M5.begin");

    Serial.begin(115200);
    delay(1000);
    startup_step("S02", "Serial.begin");

    // Reduce log level to avoid performance issues
    esp_log_level_set("*", ESP_LOG_INFO);
    // AVRCP absolute-volume logging is demoted to ESP_LOGD in the ESP32-A2DP submodule; BT_AV WARN is an extra guard.
    esp_log_level_set("BT_AV", ESP_LOG_WARN);

    ESP_LOGI("main", "=== Phase 3: BT + Module Audio (I2SStream approach) ===");
    ESP_LOGI("main", "Available Heap: %zu", esp_get_free_heap_size());

    // Display setup
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.println("\nM5Blue - Phase 3");
    M5.Display.println("A2DP queued I2S processing");
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

    // ES8388 init: Pops are avoided by using a local patched Module-Audio (lib/Module-Audio):
    // init() leaves DAC muted (DACCONTROL3=0x02) and Lout/Rout volume at 0; we unmute after SoftRamp in S07.
    // Upstream init() does DACPOWER=0x3F then DACCONTROL3=0x00 (unmute), which causes two pops.
    // Upstream setDACmute() is broken (reads ADCCONTROL1, writes DACCONTROL3) — we use direct I2C for DACCONTROL3.
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

    // Library bug: Module-Audio setDACmute() reads ES8388_ADCCONTROL1 (0x09) but writes
    // ES8388_DACCONTROL3 (0x19), corrupting DACCONTROL3. Do not use setDACmute(); use direct I2C on DACCONTROL3.

    // Configure the DAC path once. Avoid runtime mute/volume switching in the audio path.
    es8388.setDACOutput(DAC_OUTPUT_OUT1);
    es8388.setDACVolume(70);
    es8388.setBitsSample(ES_MODULE_DAC, BIT_LENGTH_16BITS);
    es8388.setSampleRate(SAMPLE_RATE_44K);
    startup_step("S06", "DAC_config");

    // DACCONTROL3 (0x19): bit5 = DACSoftRamp, bit1 = DACMute. Set SoftRamp, clear Mute (unmute).
    {
        uint8_t reg25 = 0x00;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL3);
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)ES8388_ADDR, (uint8_t)1) == 1) {
            reg25 = Wire.read();
        }
        reg25 = (reg25 & ~0x02u) | 0x20u;  // clear Mute (bit1), set SoftRamp (bit5)
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL3);
        Wire.write(reg25);
        if (Wire.endTransmission() == 0) {
            ESP_LOGI("main", "ES8388 DACCONTROL3=0x%02X (SoftRamp on, unmute)", reg25);
        }
    }
    startup_step("S07", "DAC_SoftRamp");

    // Disable LI2LO / RI2RO analog bypass in the ES8388 output mixer. The Module-Audio
    // driver init() leaves these enabled (DACCONTROL17/20 = 0xD0 => LD2LO=1, LI2LO=1),
    // which routes LINPUT/RINPUT directly into the HP output mixer. On Module Audio,
    // LIN1 is wired (via Q1 + R8/R9) to the TRRS jack MIC contact; with a plain TRRS
    // earphone (no mic) that pin is floating and picks up digital noise, producing a
    // left-only hiss. Plugging a TRS plug shorts MIC to GND and the hiss disappears.
    // Clearing bit6 (LI2LO / RI2RO) on both registers removes the leak path entirely.
    // Verified on the noise-test build; see src/main_noise_test.cpp.
    {
        uint8_t reg27 = 0x00;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL17);
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)ES8388_ADDR, (uint8_t)1) == 1) {
            reg27 = Wire.read();
        }
        uint8_t reg27_new = reg27 & ~0x40u;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL17);
        Wire.write(reg27_new);
        Wire.endTransmission();

        uint8_t reg2a = 0x00;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL20);
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)ES8388_ADDR, (uint8_t)1) == 1) {
            reg2a = Wire.read();
        }
        uint8_t reg2a_new = reg2a & ~0x40u;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL20);
        Wire.write(reg2a_new);
        Wire.endTransmission();

        ESP_LOGI("main", "ES8388 DACCONTROL17: 0x%02X -> 0x%02X (LI2LO disabled)", reg27, reg27_new);
        ESP_LOGI("main", "ES8388 DACCONTROL20: 0x%02X -> 0x%02X (RI2RO disabled)", reg2a, reg2a_new);
    }
    startup_step("S07b", "DAC_Mixer_Bypass_Off");

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
    i2s_cfg.sample_rate = kSampleRate;
    i2s_cfg.channels = 2;
    i2s_cfg.bits_per_sample = 16;
    i2s_cfg.pin_bck = SYS_I2S_SCLK_PIN;  // GPIO 19
    i2s_cfg.pin_ws = SYS_I2S_LRCK_PIN;   // GPIO 27
    i2s_cfg.pin_data = SYS_I2S_DOUT_PIN; // GPIO 2
    i2s.begin(i2s_cfg);
    ESP_LOGI("main", "I2SStream configured: BCK=%d, WS=%d, DATA=%d",
             SYS_I2S_SCLK_PIN, SYS_I2S_LRCK_PIN, SYS_I2S_DOUT_PIN);
    startup_step("S12", "i2s.begin");

    if (s_i2s_writer_task == nullptr) {
        BaseType_t task_ok = xTaskCreatePinnedToCore(i2s_writer_task, "I2SWriter", 4096, nullptr, 2, &s_i2s_writer_task, 1);
        if (task_ok != pdPASS) {
            ESP_LOGE("main", "Failed to start I2S writer task");
        }
    }

    // Setup A2DP callbacks
    a2dp_sink.set_on_connection_state_changed(connection_state_callback);
    a2dp_sink.set_on_audio_state_changed(audio_state_callback_debug, nullptr);

    // Keep Bluetooth ingress lightweight; effects are applied in the I2S writer task.
    a2dp_sink.set_raw_stream_reader_writer(audio_callback);

    // Host auto-connect (Mac/PC reconnect when this device powers on):
    // A2DP/AVDTP do not define "who must initiate" the ACL; the stack may page
    // a bonded peer or accept an incoming page. macOS/Windows often auto-connect
    // to known speakers when the remote is connectable, but that behavior is OS-
    // dependent. ESP32-A2DP enables outbound paging to the last paired source
    // (NVS `connected_bda` / `last_bda`) on stack-up plus retries after drop,
    // which matches typical headphone/speaker behavior after initial pairing.
    a2dp_sink.set_auto_reconnect(true);
    // Default reconnect_delay is 1000 ms; shorten slightly so paging starts sooner after boot.
    a2dp_sink.set_reconnect_delay(500);

    // Start A2DP sink
    ESP_LOGI("main", "Starting Bluetooth A2DP Sink...");
    M5.Display.setTextColor(CYAN);
    M5.Display.println("\nStarting BT...");
    startup_step("S14", "before_a2dp.start");
    a2dp_sink.start("M5Blue");
    startup_step("S15", "after_a2dp.start");
    // Reinforce after BT stack init (ESP32-A2DP fork uses ESP_LOGD for AVRCP volume paths).
    esp_log_level_set("BT_AV", ESP_LOG_WARN);

    ESP_LOGI("main", "Setup complete. Waiting for Bluetooth connection...");
    M5.Display.setTextColor(WHITE);
    M5.Display.println("Name: M5Blue");

    // Set LED to indicate ready
    device.setRGBLED(0, 0x00FF00); // Green - ready
    device.setRGBLED(1, 0x00FF00);
    device.setRGBLED(2, 0x00FF00);
}

void loop() {
    uint32_t loop_start_us = (uint32_t)micros();
    if (s_last_loop_start_us != 0) {
        control_stats_note_us(s_control_stats.loop_gap_us_min, s_control_stats.loop_gap_us_max, s_control_stats.loop_gap_us_sum, loop_start_us - s_last_loop_start_us);
    }
    s_last_loop_start_us = loop_start_us;

    uint32_t section_start_us = (uint32_t)micros();
    M5.update();
    control_stats_note_us(s_control_stats.m5_update_us_min, s_control_stats.m5_update_us_max, s_control_stats.m5_update_us_sum, (uint32_t)micros() - section_start_us);
    update_bt_status_display();

    section_start_us = (uint32_t)micros();
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
        device.setRGBLED(2, 0xFF0000); // Red LED
    } else if (!red_pressed && g_effect_red) {
        ESP_LOGI("main", "Effect Red OFF");
        g_effect_red = false;
        device.setRGBLED(2, 0x00FF00); // Green LED
    }
    control_stats_note_us(s_control_stats.button_us_min, s_control_stats.button_us_max, s_control_stats.button_us_sum, (uint32_t)micros() - section_start_us);

    // Rotation angle unit (Grove B): drive DJ filter every loop, log/display at 5 Hz.
    // ESP32 ADC on GPIO36 is inherently noisy; apply (A) 16x oversampling + (B) EMA low-pass
    // before mapping to v. Knob asymmetric around center, so use piecewise linear mapping.
    // Going-Zero GUI horizontal slider equivalence:
    //   knob full left  (mV=ROT_MV_LEFT  =3145) -> v=-1 (LPF heavy)
    //   knob center     (mV=ROT_MV_CENTER=2100) -> v= 0 (bypass)
    //   knob full right (mV=ROT_MV_RIGHT = 142) -> v=+1 (HPF heavy)
    {
        const int ROT_MV_LEFT = 3145;
        const int ROT_MV_CENTER = 2100;
        const int ROT_MV_RIGHT = 142;

        // (A) Oversampling: read N times in a burst and average.
        const int kOversampleN = 8;
        int mV_sum = 0;
        section_start_us = (uint32_t)micros();
        for (int i = 0; i < kOversampleN; i++) {
            mV_sum += analogReadMilliVolts(ROTATION_ANGLE_GPIO);
        }
        control_stats_note_us(s_control_stats.adc_us_min, s_control_stats.adc_us_max, s_control_stats.adc_us_sum, (uint32_t)micros() - section_start_us);
        int mV_avg = mV_sum / kOversampleN;

        // (B) EMA low-pass filter: mV_filt = alpha * mV_avg + (1 - alpha) * mV_filt
        // Higher alpha keeps ADC noise smoothing but reduces knob-to-filter lag.
        static float s_mV_filt = 0.0f;
        static bool s_ema_init = false;
        const float kEmaAlpha = 0.65f;
        if (!s_ema_init) {
            s_mV_filt = (float)mV_avg;
            s_ema_init = true;
        } else {
            s_mV_filt = kEmaAlpha * (float)mV_avg + (1.0f - kEmaAlpha) * s_mV_filt;
        }
        int mV = (int)s_mV_filt;

        // Track observed min/max of the smoothed readings (calibration aid, serial log)
        static int s_mV_min = 99999;
        static int s_mV_max = -1;
        if (mV < s_mV_min) s_mV_min = mV;
        if (mV > s_mV_max) s_mV_max = mV;

        // Piecewise linear: separate slopes for the LPF (mV >= center) and HPF (mV < center) halves.
        float v;
        if (mV >= ROT_MV_CENTER) {
            v = -(float)(mV - ROT_MV_CENTER) / (float)(ROT_MV_LEFT - ROT_MV_CENTER);
        } else {
            v = (float)(ROT_MV_CENTER - mV) / (float)(ROT_MV_CENTER - ROT_MV_RIGHT);
        }
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        // Small deadzone so a physical knob near the center reliably hits exact bypass (reset+fade-in path)
        if (fabsf(v) < 0.03f) v = 0.0f;
        g_dj_filter_target_value = v;

        static uint32_t last_rotation_dump_ms = 0;
        uint32_t now_ms = (uint32_t)millis();
        if (now_ms - last_rotation_dump_ms >= 200) {
            last_rotation_dump_ms = now_ms;
            section_start_us = (uint32_t)micros();
            // mV + filter value line (row 200, yellow)
            M5.Display.fillRect(0, 200, 320, 20, BLACK);
            M5.Display.setCursor(0, 200);
            M5.Display.setTextColor(YELLOW);
            M5.Display.printf("mV=%4d  v=%+.2f", mV, (double)v);
            control_stats_note_us(s_control_stats.display_us_min, s_control_stats.display_us_max, s_control_stats.display_us_sum, (uint32_t)micros() - section_start_us);
            s_control_stats.display_update_count++;
        }
    }

    section_start_us = (uint32_t)micros();
    dump_audio_stats_if_due();
    control_stats_note_us(s_control_stats.audio_stats_us_min, s_control_stats.audio_stats_us_max, s_control_stats.audio_stats_us_sum, (uint32_t)micros() - section_start_us);
    control_stats_note_us(s_control_stats.loop_body_us_min, s_control_stats.loop_body_us_max, s_control_stats.loop_body_us_sum, (uint32_t)micros() - loop_start_us);
    s_control_stats.loop_count++;
    dump_control_stats_if_due();

    // Keep control latency low while still yielding to other tasks.
    delay(2);
}
