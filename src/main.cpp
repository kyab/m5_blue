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

// Dual button: Core2 side PORT.A only (G32/G33). Mutually exclusive with Joystick2 on PORT.A.
#define DUAL_BUTTON_BLUE 33
#define DUAL_BUTTON_RED 32

// PORT.A Grove I2C (Core2 side): G32=SDA, G33=SCL
#define PORT_A_I2C_SDA_PIN 32
#define PORT_A_I2C_SCL_PIN 33

// DJ filter control source — enable exactly one.
//   DJ_FILTER_CTRL_JOYSTICK2      : Unit Joystick2 on PORT.A (I2C @0x63). Hold Z and move Y.
//   DJ_FILTER_CTRL_ROTATION_ANGLE : Rotation angle unit (M5STACK-U005) on PORT.B (G36 ADC).
#define DJ_FILTER_CTRL_JOYSTICK2 1
// #define DJ_FILTER_CTRL_ROTATION_ANGLE 1

#if defined(DJ_FILTER_CTRL_JOYSTICK2) && defined(DJ_FILTER_CTRL_ROTATION_ANGLE)
#error "Enable only one of DJ_FILTER_CTRL_JOYSTICK2 or DJ_FILTER_CTRL_ROTATION_ANGLE"
#endif
#if !defined(DJ_FILTER_CTRL_JOYSTICK2) && !defined(DJ_FILTER_CTRL_ROTATION_ANGLE)
#error "Define DJ_FILTER_CTRL_JOYSTICK2 or DJ_FILTER_CTRL_ROTATION_ANGLE"
#endif

#if defined(DJ_FILTER_CTRL_ROTATION_ANGLE)
// Rotation angle unit (M5STACK-U005): PORT.B (black, analog) on the Bottom module; G36 = ADC
#define ROTATION_ANGLE_GPIO 36
// Dual button may use PORT.A only when Joystick2 is not occupying it.
#define USE_DUAL_BUTTON_PORT_A 1
#endif

#if defined(DJ_FILTER_CTRL_JOYSTICK2)
#include "m5_unit_joystick2.hpp"
#endif

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
    uint32_t adc_button_us_min = UINT32_MAX;
    uint32_t adc_button_us_max = 0;
    uint32_t adc_button_us_sum = 0;
    uint32_t joystick_read_us_min = UINT32_MAX;
    uint32_t joystick_read_us_max = 0;
    uint32_t joystick_read_us_sum = 0;
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
    if (now_ms - s_last_dump_ms < 10000) return;
    s_last_dump_ms = now_ms;

    ControlLoopStats stats = s_control_stats;
    s_control_stats = ControlLoopStats();
    if (stats.loop_count == 0) return;

    uint32_t loop_gap_min = (stats.loop_gap_us_min == UINT32_MAX) ? 0 : stats.loop_gap_us_min;
    uint32_t loop_body_min = (stats.loop_body_us_min == UINT32_MAX) ? 0 : stats.loop_body_us_min;
    uint32_t m5_update_min = (stats.m5_update_us_min == UINT32_MAX) ? 0 : stats.m5_update_us_min;
    uint32_t button_min = (stats.button_us_min == UINT32_MAX) ? 0 : stats.button_us_min;
    uint32_t adc_button_min = (stats.adc_button_us_min == UINT32_MAX) ? 0 : stats.adc_button_us_min;
    uint32_t joystick_read_min = (stats.joystick_read_us_min == UINT32_MAX) ? 0 : stats.joystick_read_us_min;
    uint32_t display_min = (stats.display_us_min == UINT32_MAX) ? 0 : stats.display_us_min;
    uint32_t audio_stats_min = (stats.audio_stats_us_min == UINT32_MAX) ? 0 : stats.audio_stats_us_min;
    uint32_t display_avg = (stats.display_update_count == 0) ? 0 : stats.display_us_sum / stats.display_update_count;

    ESP_LOGI("control_stats",
             "loops=%lu gap_us=%lu/%lu/%lu body_us=%lu/%lu/%lu m5_us=%lu/%lu/%lu button_us=%lu/%lu/%lu adc_button_us=%lu/%lu/%lu joystick_read_us=%lu/%lu/%lu display_us=%lu/%lu/%lu display_n=%lu audio_stats_us=%lu/%lu/%lu",
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
             (unsigned long)adc_button_min,
             (unsigned long)(stats.adc_button_us_sum / stats.loop_count),
             (unsigned long)stats.adc_button_us_max,
             (unsigned long)joystick_read_min,
             (unsigned long)(stats.joystick_read_us_sum / stats.loop_count),
             (unsigned long)stats.joystick_read_us_max,
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
    if (now_ms - s_last_dump_ms < 10000) return;
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

// Ring buffer: delay (red) disabled for now; used by Going-Zero-style Freezer (blue) on g_ring.
RingBufferInterleaved* g_ring = nullptr;

// Effect control targets written by dual_button_poll_task and consumed by the I2S writer task.
volatile bool g_effect_blue = false; // Freezer (Going-Zero Freeze / Freezer.m)
volatile bool g_effect_red = false;  // Delay effect (disabled — shares g_ring with Freezer)

// DJ Filter (Going-Zero port): controller-driven LPF/HPF crossover
static DJFilter g_dj_filter;
static volatile float g_dj_filter_target_value = 0.0f;
// Scratch buffers for int16<->float deinterleave in the I2S writer task.
static float s_dj_left[kI2SWriterFrames];
static float s_dj_right[kI2SWriterFrames];

// Same bypass deadzone as the U005 path so near-center / near-zero hits exact bypass.
static const float kDjFilterBypassDeadzone = 0.03f;

// Clamp to [-1, +1], apply bypass deadzone, and publish to the I2S writer task.
static float apply_dj_filter_target_value(float v) {
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    if (fabsf(v) < kDjFilterBypassDeadzone) v = 0.0f;
    g_dj_filter_target_value = v;
    return v;
}

#if defined(DJ_FILTER_CTRL_ROTATION_ANGLE)
// Piecewise-linear U005 calibration (mV) -> DJFilter v in [-1, +1].
//   full left  (3145 mV) -> -1 (LPF heavy)
//   center     (2100 mV) ->  0 (bypass)
//   full right ( 142 mV) -> +1 (HPF heavy)
static float map_rotation_angle_mv_to_filter_v(int mV) {
    const int ROT_MV_LEFT = 3145;
    const int ROT_MV_CENTER = 2100;
    const int ROT_MV_RIGHT = 142;

    float v;
    if (mV >= ROT_MV_CENTER) {
        v = -(float)(mV - ROT_MV_CENTER) / (float)(ROT_MV_LEFT - ROT_MV_CENTER);
    } else {
        v = (float)(ROT_MV_CENTER - mV) / (float)(ROT_MV_CENTER - ROT_MV_RIGHT);
    }
    return v;
}

// Read U005 on PORT.B, smooth, map to filter v, and refresh the on-screen readout.
static void update_dj_filter_from_rotation_angle() {
    // ESP32 ADC on GPIO36 is inherently noisy; apply (A) oversampling + (B) EMA low-pass
    // before mapping to v. Knob is asymmetric around center, so mapping is piecewise linear.
    const int kOversampleN = 8;
    int mV_sum = 0;
    uint32_t section_start_us = (uint32_t)micros();
    for (int i = 0; i < kOversampleN; i++) {
        mV_sum += analogReadMilliVolts(ROTATION_ANGLE_GPIO);
    }
    control_stats_note_us(s_control_stats.adc_button_us_min, s_control_stats.adc_button_us_max, s_control_stats.adc_button_us_sum,
                          (uint32_t)micros() - section_start_us);
    int mV_avg = mV_sum / kOversampleN;

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

    static int s_mV_min = 99999;
    static int s_mV_max = -1;
    if (mV < s_mV_min) s_mV_min = mV;
    if (mV > s_mV_max) s_mV_max = mV;

    float v = apply_dj_filter_target_value(map_rotation_angle_mv_to_filter_v(mV));

    static uint32_t last_rotation_dump_ms = 0;
    uint32_t now_ms = (uint32_t)millis();
    if (now_ms - last_rotation_dump_ms >= 200) {
        last_rotation_dump_ms = now_ms;
        section_start_us = (uint32_t)micros();
        M5.Display.fillRect(0, 200, 320, 20, BLACK);
        M5.Display.setCursor(0, 200);
        M5.Display.setTextColor(YELLOW);
        M5.Display.printf("mV=%4d  v=%+.2f", mV, (double)v);
        control_stats_note_us(s_control_stats.display_us_min, s_control_stats.display_us_max, s_control_stats.display_us_sum,
                              (uint32_t)micros() - section_start_us);
        s_control_stats.display_update_count++;
    }
}
#endif  // DJ_FILTER_CTRL_ROTATION_ANGLE

#if defined(DJ_FILTER_CTRL_JOYSTICK2)
static M5UnitJoystick2 g_joystick2;
static bool g_joystick2_ok = false;

// 12-bit Y offset full-scale used to reach |v|=1 at physical extremes (hall units ~±2k).
static const int16_t kJoystick2YOffsetFullScale = 2000;

// Positive Y offset = stick toward top (UiFlow convention) -> LPF (negative v).
// Negative Y offset = stick toward bottom -> HPF (positive v).
static float map_joystick2_y_offset_to_filter_v(int16_t y_offset) {
    return -(float)y_offset / (float)kJoystick2YOffsetFullScale;
}

static bool init_joystick2_porta() {
    g_joystick2_ok = g_joystick2.begin(&Wire1, JOYSTICK2_ADDR, PORT_A_I2C_SDA_PIN, PORT_A_I2C_SCL_PIN, 400000UL);
    if (g_joystick2_ok) {
        ESP_LOGI("main", "Joystick2 OK on PORT.A addr=0x%02X", JOYSTICK2_ADDR);
        g_joystick2.set_rgb_color(0x001000);
        M5.Display.setTextColor(GREEN);
        M5.Display.println("Joystick2: OK");
    } else {
        ESP_LOGW("main", "Joystick2 not found on PORT.A (SDA=%d SCL=%d)", PORT_A_I2C_SDA_PIN, PORT_A_I2C_SCL_PIN);
        M5.Display.setTextColor(YELLOW);
        M5.Display.println("Joystick2: N/A");
        g_dj_filter_target_value = 0.0f;
    }
    return g_joystick2_ok;
}

// Hold Z (button) and move Y to drive the DJ filter; otherwise force bypass.
static void update_dj_filter_from_joystick2() {
    float v = 0.0f;
    uint8_t button = 1;
    int16_t y_off = 0;

    if (g_joystick2_ok) {
        uint32_t section_start_us = (uint32_t)micros();
        // Z button: 0 = pressed (ON), 1 = released
        button = g_joystick2.get_button_value();
        y_off = g_joystick2.get_joy_adc_12bits_offset_value_y();
        control_stats_note_us(s_control_stats.joystick_read_us_min, s_control_stats.joystick_read_us_max,
                              s_control_stats.joystick_read_us_sum, (uint32_t)micros() - section_start_us);

        const bool z_on = (button == 0);
        if (z_on) {
            v = map_joystick2_y_offset_to_filter_v(y_off);
        }
    }

    v = apply_dj_filter_target_value(v);

    static uint32_t last_joy_dump_ms = 0;
    uint32_t now_ms = (uint32_t)millis();
    if (now_ms - last_joy_dump_ms >= 200) {
        last_joy_dump_ms = now_ms;
        uint32_t section_start_us = (uint32_t)micros();
        M5.Display.fillRect(0, 200, 320, 20, BLACK);
        M5.Display.setCursor(0, 200);
        M5.Display.setTextColor(YELLOW);
        M5.Display.printf("Z=%d Y=%+5d v=%+.2f", (button == 0) ? 1 : 0, (int)y_off, (double)v);
        control_stats_note_us(s_control_stats.display_us_min, s_control_stats.display_us_max, s_control_stats.display_us_sum,
                              (uint32_t)micros() - section_start_us);
        s_control_stats.display_update_count++;
    }
}
#endif  // DJ_FILTER_CTRL_JOYSTICK2

// When the A2DP source sends long runs of digital silence (pause, track gap, app mute),
// some DAC paths treat "all zero" PCM as an idle state and create audible clicks at
// resume. If a block's peak is below this threshold, mix in sub-audible TPDF dither
// so the I2S stream never stays stuck at exact zero.
static const int32_t kSilenceDitherPeakThreshold = 16;  // |int16| peak per block
static const int32_t kSilenceDitherLsbScale = 2;        // ~2 LSB RMS; >>15 after TPDF

volatile esp_a2d_connection_state_t g_bt_connection_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
volatile esp_a2d_audio_state_t g_a2dp_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
volatile bool g_bt_state_display_dirty = true;
volatile bool g_a2dp_connected = false;
volatile int g_host_volume_127 = 127;
volatile bool g_host_volume_dirty = true;
volatile bool g_host_volume_received = false;
volatile bool g_force_silent_output = false;

enum class DacVolumeCurve {
    Linear,
    Gamma16,
    ExpK3,
};

static DacVolumeCurve s_dac_volume_curve = DacVolumeCurve::Linear;
static int s_last_applied_dac_volume = -1;

// Deferred Module-Audio RGB LED (I2C off the hot path).
#if defined(USE_DUAL_BUTTON_PORT_A)
static const uint32_t kDualButtonPollMs = 5;
#endif
static const uint32_t kModuleRgbLedDeferMs = 10;
static TaskHandle_t s_module_led_task = nullptr;
static volatile uint32_t s_module_led_rgb0 = 0x00FF00;
static volatile uint32_t s_module_led_rgb2 = 0x00FF00;

static void module_rgb_led_task(void* arg) {
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(kModuleRgbLedDeferMs));
        uint32_t c0 = s_module_led_rgb0;
        uint32_t c2 = s_module_led_rgb2;
        device.setRGBLED(0, c0);
        device.setRGBLED(2, c2);
    }
}

#if defined(USE_DUAL_BUTTON_PORT_A)
static void dual_button_poll_task(void* arg) {
    (void)arg;
    bool prev_blue = (digitalRead(DUAL_BUTTON_BLUE) == LOW);
    g_effect_blue = prev_blue;
    s_module_led_rgb0 = prev_blue ? 0x0000FF : 0x00FF00;
    if (s_module_led_task != nullptr) xTaskNotifyGive(s_module_led_task);

    for (;;) {
        uint32_t section_start_us = (uint32_t)micros();

        bool blue = (digitalRead(DUAL_BUTTON_BLUE) == LOW);
        bool red = (digitalRead(DUAL_BUTTON_RED) == LOW);

        if (blue != prev_blue) {
            prev_blue = blue;
            g_effect_blue = blue;
            if (blue) {
                ESP_LOGI("main", "Effect Blue ON (Freezer)");
                s_module_led_rgb0 = 0x0000FF;
            } else {
                ESP_LOGI("main", "Effect Blue OFF");
                s_module_led_rgb0 = 0x00FF00;
            }
            if (s_module_led_task != nullptr) xTaskNotifyGive(s_module_led_task);
        }

#if 0
        // Red delay shares g_ring with Freezer; disabled until split-buffer exists.
        if (red != prev_red) {
            prev_red = red;
            g_effect_red = red;
            if (red) {
                ESP_LOGI("main", "Effect Red ON (Delay)");
                s_module_led_rgb2 = 0xFF0000;
            } else {
                ESP_LOGI("main", "Effect Red OFF");
                s_module_led_rgb2 = 0x00FF00;
            }
            if (s_module_led_task != nullptr) xTaskNotifyGive(s_module_led_task);
        }
#else
        (void)red;
#endif

        control_stats_note_us(s_control_stats.button_us_min, s_control_stats.button_us_max, s_control_stats.button_us_sum,
                              (uint32_t)micros() - section_start_us);
        vTaskDelay(pdMS_TO_TICKS(kDualButtonPollMs));
    }
}
#endif  // USE_DUAL_BUTTON_PORT_A

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

// Startup step trace: default no delay (-DSTARTUP_STEP_DELAY_MS to override).
#ifndef STARTUP_STEP_DELAY_MS
#define STARTUP_STEP_DELAY_MS 0
#endif
static const int kStartupStepDelayMs = STARTUP_STEP_DELAY_MS;
static uint32_t s_prev_startup_step_us = 0;
static void startup_step(const char* step_id, const char* step_name) {
    uint32_t now_us = (uint32_t)micros();
    uint32_t delta_us = (s_prev_startup_step_us == 0) ? 0 : (uint32_t)(now_us - s_prev_startup_step_us);
    s_prev_startup_step_us = now_us;
    Serial.printf("\n--- SETUP %s | %s | millis=%lu ts_us=%lu dt_us=%lu ---\n", step_id, step_name,
                  (unsigned long)millis(), (unsigned long)now_us, (unsigned long)delta_us);
    Serial.printf("{\"ts\":%lu,\"ts_us\":%lu,\"dt_us\":%lu,\"startup_step\":\"%s\",\"name\":\"%s\"}\n",
                  (unsigned long)millis(), (unsigned long)now_us, (unsigned long)delta_us, step_id, step_name);
    ESP_LOGI("SETUP_STEP", "%s  %s", step_id, step_name);
    delay(kStartupStepDelayMs);
}

static void pin_module_audio_mclk_gpio0() {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
}

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

// Going-Zero Freezer.m / MiniFader.h: FADE_SAMPLE_NUM (~1 ms at 44.1 kHz). Going-Zero default grain is 3000; M5_blue uses 2000 for shorter loop windows.
static const uint32_t kFreezerFadeSamples = 50;
static const uint32_t kFreezerDefaultGrainSamples = 2000;

static int16_t fz_scale_i16(int16_t s, float rate) {
    int32_t v = (int32_t)lroundf((float)s * rate);
    if (v > 32767) v = 32767;
    else if (v < -32768) v = -32768;
    return static_cast<int16_t>(v);
}

// Freezer effect + ring capture (Going-Zero Freezer.m). Call only when g_ring is non-null and input is in `data`.
static void apply_going_zero_freezer(int16_t* data, uint32_t frame_count) {
    static bool s_active = false;
    static bool s_is_fading_out = false;
    static bool s_is_fading_in = false;
    static bool s_target_active = false;
    static uint32_t s_fade_out_ctr = 0;
    static uint32_t s_fade_in_ctr = 0;
    static uint32_t s_grain_size = kFreezerDefaultGrainSamples;
    static uint32_t s_grain_sample_index = 0;
    static size_t s_grain_start_frame = 0;
    static uint32_t s_grain_mini_out = 0;
    static uint32_t s_grain_mini_in = 0;

    const size_t buf_frames = g_ring->getBufferSize();
    if (buf_frames == 0) return;

    for (uint32_t i = 0; i < frame_count; i++) {
        uint32_t grain_sz = s_grain_size;
        if (grain_sz > buf_frames) grain_sz = (uint32_t)buf_frames;
        if (grain_sz == 0) grain_sz = 1;

        int16_t* sl = &data[i * 2];
        int16_t* sr = &data[i * 2 + 1];
        int16_t L = *sl;
        int16_t R = *sr;

        const bool want = g_effect_blue;
        if (want != s_active && !s_is_fading_out) {
            s_target_active = want;
            s_is_fading_out = true;
            s_fade_out_ctr = kFreezerFadeSamples;
        }

        const bool was_fading_out = s_is_fading_out;
        if (was_fading_out) {
            if (s_active) {
                size_t ri = (s_grain_start_frame + s_grain_sample_index) % buf_frames;
                g_ring->readFrameModulo(ri, &L, &R);
                uint32_t d = s_grain_sample_index + 1;
                if (grain_sz - d == kFreezerFadeSamples) s_grain_mini_out = kFreezerFadeSamples;
                if (grain_sz - d < kFreezerFadeSamples) {
                    if (s_grain_mini_out > 0) {
                        float rate = s_grain_mini_out / (float)kFreezerFadeSamples;
                        L = fz_scale_i16(L, rate);
                        R = fz_scale_i16(R, rate);
                        s_grain_mini_out--;
                    }
                }
                s_grain_sample_index++;
                if (s_grain_sample_index >= grain_sz) {
                    s_grain_sample_index = 0;
                    s_grain_mini_in = 0;
                }
                if (s_grain_sample_index < kFreezerFadeSamples) {
                    if (s_grain_mini_in < kFreezerFadeSamples) {
                        float rate = s_grain_mini_in / (float)kFreezerFadeSamples;
                        L = fz_scale_i16(L, rate);
                        R = fz_scale_i16(R, rate);
                        s_grain_mini_in++;
                    }
                }
            }
            if (s_fade_out_ctr > 0) {
                float rate = s_fade_out_ctr / (float)kFreezerFadeSamples;
                L = fz_scale_i16(L, rate);
                R = fz_scale_i16(R, rate);
                s_fade_out_ctr--;
            }
            if (s_fade_out_ctr == 0) {
                s_active = s_target_active;
                if (s_active) {
                    uint32_t gsz = s_grain_size;
                    if (gsz > buf_frames) gsz = (uint32_t)buf_frames;
                    if (gsz == 0) gsz = 1;
                    const size_t wp = g_ring->getWritePosition();
                    s_grain_start_frame = (wp + buf_frames - gsz) % buf_frames;
                    s_grain_sample_index = 0;
                    s_grain_mini_in = 0;
                    s_grain_mini_out = 0;
                }
                s_is_fading_out = false;
                s_is_fading_in = true;
                s_fade_in_ctr = 0;
            }
        }

        if (!was_fading_out) {
            if (s_active) {
                size_t ri = (s_grain_start_frame + s_grain_sample_index) % buf_frames;
                g_ring->readFrameModulo(ri, &L, &R);
                uint32_t d = s_grain_sample_index + 1;
                if (grain_sz - d == kFreezerFadeSamples) s_grain_mini_out = kFreezerFadeSamples;
                if (grain_sz - d < kFreezerFadeSamples) {
                    if (s_grain_mini_out > 0) {
                        float rate = s_grain_mini_out / (float)kFreezerFadeSamples;
                        L = fz_scale_i16(L, rate);
                        R = fz_scale_i16(R, rate);
                        s_grain_mini_out--;
                    }
                }
                s_grain_sample_index++;
                if (s_grain_sample_index >= grain_sz) {
                    s_grain_sample_index = 0;
                    s_grain_mini_in = 0;
                }
                if (s_grain_sample_index < kFreezerFadeSamples) {
                    if (s_grain_mini_in < kFreezerFadeSamples) {
                        float rate = s_grain_mini_in / (float)kFreezerFadeSamples;
                        L = fz_scale_i16(L, rate);
                        R = fz_scale_i16(R, rate);
                        s_grain_mini_in++;
                    }
                }
                if (s_is_fading_in) {
                    if (s_fade_in_ctr < kFreezerFadeSamples) {
                        float rate = s_fade_in_ctr / (float)kFreezerFadeSamples;
                        L = fz_scale_i16(L, rate);
                        R = fz_scale_i16(R, rate);
                        s_fade_in_ctr++;
                        if (s_fade_in_ctr >= kFreezerFadeSamples) s_is_fading_in = false;
                    }
                }
            } else {
                if (s_is_fading_in) {
                    if (s_fade_in_ctr < kFreezerFadeSamples) {
                        float rate = s_fade_in_ctr / (float)kFreezerFadeSamples;
                        L = fz_scale_i16(L, rate);
                        R = fz_scale_i16(R, rate);
                        s_fade_in_ctr++;
                        if (s_fade_in_ctr >= kFreezerFadeSamples) s_is_fading_in = false;
                    }
                }
            }
        }

        *sl = L;
        *sr = R;
    }
}

static void apply_effects_before_i2s(int16_t* data, uint32_t frame_count, bool from_bt_pcm) {
    static float s_applied_dj_filter_value = 0.0f;

    bool red_enabled = g_effect_red;

    if (from_bt_pcm) {
        if (g_force_silent_output) {
            memset(data, 0, frame_count * kStereoChannels * sizeof(int16_t));
            // Keep dither injection alive even in forced-silent mode to avoid hard zero-idle transitions.
            add_silence_dither_if_needed(data, frame_count);
            return;
        }
        // DUAL_BUTTON_RED delay shares g_ring with Freezer; delay path disabled until split-buffer or mux exists.
#if 0
        static bool s_delay_effect_active = false;
        if (g_ring != nullptr && red_enabled && !s_delay_effect_active) {
            g_ring->syncPositon();
            g_ring->advanceReadPosition(-44100); // ~1 second delay
        }
        s_delay_effect_active = red_enabled;
#endif
        (void)red_enabled;

        if (g_ring != nullptr) {
            g_ring->storeSamples(data, frame_count);
            apply_going_zero_freezer(data, frame_count);
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
    }
}

static void i2s_writer_task(void* arg) {
    (void)arg;
    const size_t block_bytes = sizeof(s_i2s_writer_bytes);
    const uint32_t block_frames = block_bytes / (kStereoChannels * sizeof(int16_t));
    bool draining_bt_pcm = false;
    static bool s_bt_boundary_valid = false;
    static int16_t s_bt_prev_tail_L = 0;
    static int16_t s_bt_prev_tail_R = 0;
    // Adjacent PCM at I2S block splits should be continuous; a large jump often indicates a host
    // stream discontinuity (e.g. seek). Loud HF content can also produce large steps; threshold
    // is set conservatively high to balance false positives vs. audible clicks from Safari/macOS.
    static const int32_t kBtBoundaryDeltaThreshold = 28000;
    static const uint32_t kBtBoundaryRepairFrames = 48;
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
                s_bt_boundary_valid = false;
                fill_dither_bytes(s_i2s_writer_bytes, block_bytes);
            } else {
                int16_t* pcm = reinterpret_cast<int16_t*>(s_i2s_writer_bytes);
                const uint32_t last_frame_idx = (block_frames - 1U) * kStereoChannels;
                if (s_bt_boundary_valid) {
                    int32_t dL = (int32_t)pcm[0] - (int32_t)s_bt_prev_tail_L;
                    int32_t dR = (int32_t)pcm[1] - (int32_t)s_bt_prev_tail_R;
                    if (dL < 0) dL = -dL;
                    if (dR < 0) dR = -dR;
                    const int32_t mx = (dL > dR) ? dL : dR;
                    if (mx >= kBtBoundaryDeltaThreshold) {
                        const uint32_t nrep =
                            (block_frames < kBtBoundaryRepairFrames) ? block_frames : kBtBoundaryRepairFrames;
                        for (uint32_t fi = 0; fi < nrep; fi++) {
                            float t = (float)(fi + 1U) / (float)nrep;
                            float fl = (1.0f - t) * (float)s_bt_prev_tail_L + t * (float)pcm[fi * kStereoChannels];
                            float fr =
                                (1.0f - t) * (float)s_bt_prev_tail_R + t * (float)pcm[fi * kStereoChannels + 1U];
                            int32_t il = (int32_t)lroundf(fl);
                            int32_t ir = (int32_t)lroundf(fr);
                            if (il > 32767) il = 32767;
                            else if (il < -32768) il = -32768;
                            if (ir > 32767) ir = 32767;
                            else if (ir < -32768) ir = -32768;
                            pcm[fi * kStereoChannels] = static_cast<int16_t>(il);
                            pcm[fi * kStereoChannels + 1U] = static_cast<int16_t>(ir);
                        }
                    }
                }
                s_bt_prev_tail_L = pcm[last_frame_idx];
                s_bt_prev_tail_R = pcm[last_frame_idx + 1U];
                s_bt_boundary_valid = true;
            }
        } else {
            s_bt_boundary_valid = false;
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
    g_a2dp_connected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
    if (!g_a2dp_connected) {
        g_force_silent_output = true;
        if (s_last_applied_dac_volume != 0) {
            if (es8388.setDACVolume(0)) {
                s_last_applied_dac_volume = 0;
                ESP_LOGI("main", "A2DP disconnected -> DAC volume=0");
            } else {
                ESP_LOGW("main", "A2DP disconnected but failed to set DAC volume=0");
            }
        }
    } else if (g_host_volume_received) {
        g_host_volume_dirty = true;
    } else {
        ESP_LOGI("main", "A2DP connected: waiting for host volume event before raising DAC volume");
    }

}

void a2dp_audio_state_callback(esp_a2d_audio_state_t state, void* obj) {
    (void)obj;
    g_a2dp_audio_state = state;
    stats_reset_callback_gap();
    if (state != ESP_A2D_AUDIO_STATE_STARTED) {
        bt_pcm_ring_clear();
    }
}

static int map_host_volume_to_dac(int host_volume_127) {
    if (host_volume_127 <= 0) return 0;
    if (host_volume_127 >= 127) return 100;

    float x = (float)host_volume_127 / 127.0f;
    float mapped = 0.0f;
    switch (s_dac_volume_curve) {
        case DacVolumeCurve::Linear:
            mapped = x;
            break;
        case DacVolumeCurve::Gamma16:
            mapped = powf(x, 1.6f);
            break;
        case DacVolumeCurve::ExpK3:
            mapped = (expf(3.0f * x) - 1.0f) / (expf(3.0f) - 1.0f);
            break;
        default:
            mapped = x;
            break;
    }
    int dac_volume = (int)lroundf(100.0f * mapped);
    if (dac_volume < 0) dac_volume = 0;
    if (dac_volume > 100) dac_volume = 100;
    return dac_volume;
}

static void handle_host_volume_change(int host_volume_127) {
    if (host_volume_127 < 0) host_volume_127 = 0;
    if (host_volume_127 > 127) host_volume_127 = 127;
    g_host_volume_received = true;
    g_host_volume_127 = host_volume_127;
    g_host_volume_dirty = g_a2dp_connected;
}

static void apply_host_volume_to_dac_if_needed() {
    if (!g_a2dp_connected) return;
    if (!g_host_volume_received) return;
    if (!g_host_volume_dirty) return;
    int host_volume = g_host_volume_127;
    int dac_volume = map_host_volume_to_dac(host_volume);
    g_force_silent_output = (host_volume == 0);
    if (dac_volume == s_last_applied_dac_volume) {
        ESP_LOGD("main", "skip DAC volume write: host=%d dac=%d (same)", host_volume, dac_volume);
        g_host_volume_dirty = false;
        return;
    }
    if (es8388.setDACVolume((uint8_t)dac_volume)) {
        s_last_applied_dac_volume = dac_volume;
        ESP_LOGI("main", "host volume=%d -> dac volume=%d (curve=%d)", host_volume, dac_volume, (int)s_dac_volume_curve);
    } else {
        ESP_LOGW("main", "failed DAC volume update: host=%d dac=%d", host_volume, dac_volume);
    }
    g_host_volume_dirty = false;
}

void setup() {
    // Initialize M5Unified
    auto cfg = M5.config();
    M5.begin(cfg);
    startup_step("S01", "M5.begin");

    Serial.begin(115200);
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

#if defined(USE_DUAL_BUTTON_PORT_A)
    // Initialize dual button pins (PORT.A GPIO). Not used when Joystick2 owns PORT.A.
    pinMode(DUAL_BUTTON_BLUE, INPUT);
    pinMode(DUAL_BUTTON_RED, INPUT);
#endif

    // Scan system I2C bus (Module Audio / ES8388 on G21/G22)
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

#if defined(DJ_FILTER_CTRL_JOYSTICK2)
    init_joystick2_porta();
    startup_step("S03b", "Joystick2_PORTA");
#endif

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

    // Button sampling at fixed interval; RGB LED updates deferred on a lower-priority task (I2C).
    if (xTaskCreatePinnedToCore(module_rgb_led_task, "ModRgbLed", 3072, nullptr, 3, &s_module_led_task, 0) != pdPASS) {
        ESP_LOGE("main", "Failed to create ModRgbLed task");
    }
#if defined(USE_DUAL_BUTTON_PORT_A)
    if (xTaskCreatePinnedToCore(dual_button_poll_task, "DualBtn", 3072, nullptr, 6, nullptr, 0) != pdPASS) {
        ESP_LOGE("main", "Failed to create DualBtn task");
    }
#endif

    // Core2 onboard speaker/analog pins overlap Module Audio MCLK/I2S; release driver first.
    M5.Speaker.end();
    startup_step("S05", "M5.Speaker.end");

    // Codec needs MCLK from the host before I2C initialization sequence completes reliably.
    pin_module_audio_mclk_gpio0();
    startup_step("S06", "MCLK_GPIO0");

    ESP_LOGI("main", "Initializing ES8388 codec...");
    // Patched Module-Audio: init() asserts DAC mute + volume 0. Do not use setDACmute() (broken RMW).
    bool es8388_inited = es8388.init();
    if (!es8388_inited) {
        es8388_inited = es8388.init();
    }
    if (!es8388_inited) {
        ESP_LOGE("main", "Failed to initialize ES8388!");
        M5.Display.setTextColor(RED);
        M5.Display.println("ES8388: FAILED!");
    } else {
        ESP_LOGI("main", "ES8388 OK");
        M5.Display.setTextColor(GREEN);
        M5.Display.println("ES8388: OK");
    }
    startup_step("S07", "ES8388_init");

    // Library bug: Module-Audio setDACmute() reads ES8388_ADCCONTROL1 but writes DACCONTROL3 — unused.

    es8388.setDACOutput(DAC_OUTPUT_OUT1);
    es8388.setDACVolume(0);
    s_last_applied_dac_volume = 0;
    es8388.setBitsSample(ES_MODULE_DAC, BIT_LENGTH_16BITS);
    es8388.setSampleRate(SAMPLE_RATE_44K);

    // DACCONTROL3 (0x19): bit5 = DACSoftRamp, bit1 = DACMute. Hold muted until I2S feeds PCM.
    {
        uint8_t reg25 = 0x00;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL3);
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)ES8388_ADDR, (uint8_t)1) == 1) {
            reg25 = Wire.read();
        }
        reg25 = (reg25 | 0x20u) | 0x02u;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL3);
        Wire.write(reg25);
        if (Wire.endTransmission() == 0) {
            ESP_LOGI("main", "ES8388 DACCONTROL3=0x%02X (SoftRamp + digital mute)", reg25);
        }
    }
    startup_step("S08", "DAC_config");

    // Disable LI2LO / RI2RO analog bypass in the ES8388 output mixer. The Module-Audio driver init()
    // leaves mixer paths that leak noise from LIN/RIN into the headphone mix on Module Audio wiring.
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

        ESP_LOGI("main", "ES8388 DACCONTROL17: 0x%02X -> 0x%02X (LI2LO off)", reg27, reg27_new);
        ESP_LOGI("main", "ES8388 DACCONTROL20: 0x%02X -> 0x%02X (RI2RO off)", reg2a, reg2a_new);
    }
    startup_step("S09", "DAC_Mixer_Bypass_Off");

    ESP_LOGI("main", "Initializing ring buffer...");
    g_ring = new RingBufferInterleaved();
    ESP_LOGI("main", "Ring buffer OK");
    startup_step("S10", "ring_buffer");

    ESP_LOGI("main", "Configuring I2SStream for Module Audio...");
    i2s.end();
    startup_step("S11", "i2s.end");
    auto i2s_cfg = i2s.defaultConfig();
    i2s_cfg.sample_rate = kSampleRate;
    i2s_cfg.channels = 2;
    i2s_cfg.bits_per_sample = 16;
    i2s_cfg.pin_bck = SYS_I2S_SCLK_PIN;
    i2s_cfg.pin_ws = SYS_I2S_LRCK_PIN;
    i2s_cfg.pin_data = SYS_I2S_DOUT_PIN;
    i2s.begin(i2s_cfg);
    ESP_LOGI("main", "I2SStream pins: BCK=%d WS=%d DATA=%d", SYS_I2S_SCLK_PIN, SYS_I2S_LRCK_PIN, SYS_I2S_DOUT_PIN);

    if (s_i2s_writer_task == nullptr) {
        BaseType_t task_ok = xTaskCreatePinnedToCore(i2s_writer_task, "I2SWriter", 4096, nullptr, 2, &s_i2s_writer_task, 1);
        if (task_ok != pdPASS) {
            ESP_LOGE("main", "Failed to start I2S writer task");
        }
    }
    startup_step("S12", "i2s.begin");

    {
        uint8_t reg25 = 0x00;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL3);
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)ES8388_ADDR, (uint8_t)1) == 1) {
            reg25 = Wire.read();
        }
        reg25 = (reg25 & ~0x02u) | 0x20u;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL3);
        Wire.write(reg25);
        if (Wire.endTransmission() == 0) {
            ESP_LOGI("main", "ES8388 DACCONTROL3=0x%02X (SoftRamp, unmuted)", reg25);
        }
    }
    startup_step("S13", "DAC_SoftRamp");

    // Setup A2DP callbacks
    a2dp_sink.set_on_connection_state_changed(connection_state_callback);
    a2dp_sink.set_on_audio_state_changed(a2dp_audio_state_callback, nullptr);
    a2dp_sink.set_digital_volume_control(false);
    a2dp_sink.set_on_volumechange(handle_host_volume_change);

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
    apply_host_volume_to_dac_if_needed();

#if defined(DJ_FILTER_CTRL_JOYSTICK2)
    update_dj_filter_from_joystick2();
#elif defined(DJ_FILTER_CTRL_ROTATION_ANGLE)
    update_dj_filter_from_rotation_angle();
#endif

    section_start_us = (uint32_t)micros();
    dump_audio_stats_if_due();
    control_stats_note_us(s_control_stats.audio_stats_us_min, s_control_stats.audio_stats_us_max, s_control_stats.audio_stats_us_sum, (uint32_t)micros() - section_start_us);
    control_stats_note_us(s_control_stats.loop_body_us_min, s_control_stats.loop_body_us_max, s_control_stats.loop_body_us_sum, (uint32_t)micros() - loop_start_us);
    s_control_stats.loop_count++;
    dump_control_stats_if_due();

    // Keep control latency low while still yielding to other tasks.
    delay(2);
}
