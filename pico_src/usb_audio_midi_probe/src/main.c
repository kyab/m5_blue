/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/audio_i2s.h"
#include "pico/stdio_uart.h"
#include "pico/stdio_usb.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "common_types.h"

#ifdef CFG_QUIRK_OS_GUESSING
#include "quirk_os_guessing.h"
#endif

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

// List of supported sample rates
const uint32_t sample_rates[] = {44100, 48000};

uint32_t current_sample_rate = 44100;

#define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)

/* Blink pattern
 * - 25 ms   : streaming data
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
    BLINK_STREAMING = 25,
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

enum {
    VOLUME_CTRL_0_DB = 0,
    VOLUME_CTRL_10_DB = 2560,
    VOLUME_CTRL_20_DB = 5120,
    VOLUME_CTRL_30_DB = 7680,
    VOLUME_CTRL_40_DB = 10240,
    VOLUME_CTRL_50_DB = 12800,
    VOLUME_CTRL_60_DB = 15360,
    VOLUME_CTRL_70_DB = 17920,
    VOLUME_CTRL_80_DB = 20480,
    VOLUME_CTRL_90_DB = 23040,
    VOLUME_CTRL_100_DB = 25600,
    VOLUME_CTRL_SILENCE = 0x8000,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];    // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1]; // +1 for master channel 0

// Buffer for speaker data
static struct audio_buffer_pool *producer_pool;

// Master gain in Q15, recomputed whenever the host changes mute or volume.
static volatile uint16_t gain_q15 = 0x7fff;

// Observability for the console. Written by ISRs, read by the status task.
static volatile uint32_t last_feedback;
static volatile uint32_t last_queued;
static volatile uint32_t overflow_count;
static volatile uint32_t midi_rx_count;
static volatile uint8_t streaming_alt;

// UAC2 reports volume in 1/256 dB steps; convert once here instead of per sample.
static void update_gain(void) {
    if (mute[0]) {
        gain_q15 = 0;
        return;
    }

    float lin = powf(10.0f, ((float)volume[0] / 256.0f) / 20.0f);
    if (lin > 1.0f) lin = 1.0f;
    gain_q15 = (uint16_t)(lin * 32767.0f);
}

static void midi_task(void);
static void status_task(void);
static void probe_printf(const char *fmt, ...);
static uint32_t board_millis(void) { return to_ms_since_boot(get_absolute_time()); }
static void board_led_write(bool on) { (void)on; }

void led_blinking_task(void);

#if CFG_AUDIO_DEBUG
void audio_debug_task(void);
uint8_t current_alt_settings;
uint16_t fifo_count;
uint32_t fifo_count_avg;
#endif

/*------------- MAIN -------------*/
// The host picked a new rate; pico_audio_i2s reprograms the PIO clock divider from the pool format.
static void audio_reconfigure(void) {
    ((struct audio_format *)producer_pool->format)->sample_freq = current_sample_rate;
}

// One buffer per USB frame keeps the queue measurement (used for feedback) at
// 1 ms granularity; 16 of them is ~16 ms of headroom at 48 kHz.
#define POOL_BUFFER_COUNT 16
#define POOL_BUFFER_SAMPLES 64

static void i2s_setup(void) {
    static struct audio_format fmt = {
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .sample_freq = 48000,
        .channel_count = 2,
    };
    static struct audio_buffer_format producer_format = {
        .format = &fmt,
        .sample_stride = 4,
    };
    producer_pool = audio_new_producer_pool(&producer_format, POOL_BUFFER_COUNT, POOL_BUFFER_SAMPLES);

    struct audio_i2s_config config = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
        .dma_channel = 0,
        .pio_sm = 0,
    };
    if (!audio_i2s_setup(&fmt, &config)) panic("i2s setup failed");
    if (!audio_i2s_connect_extra(producer_pool, false, 3, POOL_BUFFER_SAMPLES, NULL)) panic("i2s connect failed");
    audio_i2s_set_enabled(true);
}

void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    stdio_usb_call_chars_available_callback();
}

int main(void) {
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    // pico_stdio_usb uses our descriptors (LIB_TINYUSB_DEVICE) and requires tusb_init first.
    stdio_init_all();
    i2s_setup();
    probe_printf("USB Audio + MIDI + CDC probe ready\n");

    while (1) {
        tud_task();
        led_blinking_task();
#if CFG_AUDIO_DEBUG
        audio_debug_task();
#endif
        midi_task();
        status_task();
    }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request) {
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO_CS_REQ_CUR) {
            TU_LOG1("Clock get current freq %lu\r\n", current_sample_rate);

            audio_control_cur_4_t curf = {(int32_t)tu_htole32(current_sample_rate)};
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
        } else if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_4_n_t(N_SAMPLE_RATES) rangef =
                {
                    .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)};
            TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++) {
                rangef.subrange[i].bMin = (int32_t)sample_rates[i];
                rangef.subrange[i].bMax = (int32_t)sample_rates[i];
                rangef.subrange[i].bRes = 0;
                TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
            }

            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
        }
    } else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
               request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t cur_valid = {.bCur = 1};
        TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
    }
    TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf) {
    (void)rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

        current_sample_rate = (uint32_t)((audio_control_cur_4_t const *)buf)->bCur;
        audio_reconfigure();

        TU_LOG1("Clock set current freq: %ld\r\n", current_sample_rate);

        return true;
    } else {
        TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request) {
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t mute1 = {.bCur = mute[request->bChannelNumber]};
        TU_LOG1("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
    } else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0] = {.bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256)}};
            TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
                    range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
        } else if (request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_2_t cur_vol = {.bCur = tu_htole16(volume[request->bChannelNumber])};
            TU_LOG1("Get channel %u volume %d dB\r\n", request->bChannelNumber, cur_vol.bCur / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
        }
    }
    TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf) {
    (void)rhport;

    TU_ASSERT(request->bEntityID == UAC2_ENTITY_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

        mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;
        update_gain();

        TU_LOG1("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

        return true;
    } else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

        volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;
        update_gain();

        TU_LOG1("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

        return true;
    } else {
        TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return tud_audio_clock_get_request(rhport, request);
    if (request->bEntityID == UAC2_ENTITY_FEATURE_UNIT)
        return tud_audio_feature_unit_get_request(rhport, request);
    else {
        TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
                request->bEntityID, request->bControlSelector, request->bRequest);
    }
    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_FEATURE_UNIT)
        return tud_audio_feature_unit_set_request(rhport, request, buf);
    if (request->bEntityID == UAC2_ENTITY_CLOCK)
        return tud_audio_clock_set_request(rhport, request, buf);
    TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    if (ITF_NUM_AUDIO_STREAMING == itf && alt == 0) {
        streaming_alt = 0;
        blink_interval_ms = BLINK_MOUNTED;
    }

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
    if (ITF_NUM_AUDIO_STREAMING == itf) {
        streaming_alt = alt;
        if (alt != 0) {
            blink_interval_ms = BLINK_STREAMING;
            // SOF ISR owns tud_audio_fb_set; calling it here races SET_INTERFACE's
            // first usbd_edpt_xfer on the ISO endpoints.
            last_feedback = (uint32_t)(((uint64_t)current_sample_rate << 16) / 1000u);
        }
    }

#if CFG_AUDIO_DEBUG
    current_alt_settings = alt;
#endif

    return true;
}

// Declaring a frequency method is what makes the audio driver enable the SOF
// interrupt and call tud_audio_feedback_interval_isr(); the value itself is then
// produced there from the I2S queue depth, which is what actually has to be
// regulated. The driver's own FIFO_COUNT method cannot be used because the USB
// FIFO is drained immediately into the I2S pool and so always reads as empty.
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param) {
    (void)func_id;
    (void)alt_itf;

    feedback_param->method = AUDIO_FEEDBACK_METHOD_FREQUENCY_FIXED;
    feedback_param->sample_freq = current_sample_rate;
    feedback_param->frequency.mclk_freq = clock_get_hz(clk_sys);
}

// Samples handed to pico_audio_i2s but not yet copied into the DMA chain.
// pico_audio exposes no accessor, so walk the prepared list under its own lock.
TU_ATTR_FAST_FUNC static uint32_t queued_samples(void) {
    uint32_t total = 0;

    uint32_t save = spin_lock_blocking(producer_pool->prepared_list_spin_lock);
    for (audio_buffer_t *b = producer_pool->prepared_list; b != NULL; b = b->next) {
        total += b->sample_count;
    }
    spin_unlock(producer_pool->prepared_list_spin_lock, save);

    return total;
}

TU_ATTR_FAST_FUNC void tud_audio_feedback_interval_isr(uint8_t func_id, uint32_t frame_number, uint8_t interval_shift) {
    (void)func_id;
    (void)frame_number;
    (void)interval_shift;

    // Nominal samples per frame in 16.16, and the +/- one sample window the spec allows.
    uint32_t const nominal = (uint32_t)(((uint64_t)current_sample_rate << 16) / 1000u);
    uint32_t const fb_max = (current_sample_rate / 1000 + 1) << 16;
    uint32_t const fb_min = ((current_sample_rate - 1) / 1000) << 16;

    // Steer towards 3 ms of buffered audio: deep enough to ride out host jitter,
    // shallow enough to keep latency low.
    uint32_t const target = current_sample_rate * 3 / 1000;
    uint32_t const level = queued_samples();
    last_queued = level;

    uint32_t feedback;
    if (level < target) {
        feedback = nominal + (uint32_t)(((uint64_t)(target - level) * (fb_max - nominal)) / target);
    } else {
        uint32_t excess = level - target;
        if (excess > target) excess = target;
        feedback = nominal - (uint32_t)(((uint64_t)excess * (nominal - fb_min)) / target);
    }

    if (feedback > fb_max) feedback = fb_max;
    if (feedback < fb_min) feedback = fb_min;

    last_feedback = feedback;
    tud_audio_fb_set(feedback);
}

TU_ATTR_FAST_FUNC static void apply_gain(int16_t *samples, uint32_t count) {
    uint16_t const g = gain_q15;
    if (g == 0x7fff) return;

    for (uint32_t i = 0; i < count; i++) {
        samples[i] = (int16_t)(((int32_t)samples[i] * g) >> 15);
    }
}

// Called from the USB ISR once the driver has moved a packet into its FIFO.
// Doing the copy here rather than in the main loop keeps the queue depth that
// feeds the feedback calculation free of main-loop scheduling jitter.
TU_ATTR_FAST_FUNC bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
    (void)rhport;
    (void)n_bytes_received;
    (void)func_id;
    (void)ep_out;
    (void)cur_alt_setting;

    uint16_t const frame_bytes = CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX;

    while (tud_audio_available() >= frame_bytes) {
        audio_buffer_t *buffer = take_audio_buffer(producer_pool, false);
        if (buffer == NULL) {
            // Queue is full; leave the rest in the USB FIFO for the next round.
            overflow_count++;
            break;
        }

        uint32_t want = tud_audio_available();
        uint32_t const capacity = buffer->max_sample_count * frame_bytes;
        if (want > capacity) want = capacity;
        want -= want % frame_bytes;

        uint16_t const got = tud_audio_read(buffer->buffer->bytes, (uint16_t)want);
        buffer->sample_count = got / frame_bytes;
        apply_gain((int16_t *)buffer->buffer->bytes, buffer->sample_count * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX);
        give_audio_buffer(producer_pool, buffer);
    }

    return true;
}

bool tud_audio_feedback_format_correction_cb(uint8_t func_id) {
    (void)func_id;

#if CFG_PROBE_TARGET_APPLE
    return tud_speed_get() == TUSB_SPEED_FULL;
#elif CFG_QUIRK_OS_GUESSING
    return tud_speed_get() == TUSB_SPEED_FULL && quirk_os_guessing_get() == QUIRK_OS_GUESSING_OSX;
#else
    return false;
#endif
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

static void midi_task(void) {
    uint8_t packet[4];
    while (tud_midi_available()) {
        if (!tud_midi_packet_read(packet)) break;
        midi_rx_count++;
        // Echo back so the host can see traffic in both directions.
        tud_midi_packet_write(packet);
    }

    // Heartbeat note so a MIDI monitor on the host shows incoming events even
    // when nothing is sent to us.
    static uint32_t next_note_ms = 0;
    static bool note_on = false;
    uint32_t const now = board_millis();
    if ((int32_t)(now - next_note_ms) < 0) return;
    next_note_ms = now + 1000;

    uint8_t const note[4] = {0x09, note_on ? 0x90 : 0x80, 60, note_on ? 100 : 0};
    tud_midi_packet_write(note);
    note_on = !note_on;
}

// pico_stdio_usb calls tud_task() from printf, so USB console output goes
// through TinyUSB directly. Expand LF to CRLF the same way pico_stdio does.
static void cdc_write_chars(const char *s, int len) {
    if (!tud_cdc_connected() || len <= 0) return;

    char out[208];
    int o = 0;
    char prev = 0;
    for (int i = 0; i < len && o < (int)sizeof(out) - 2; i++) {
        char const c = s[i];
        if (c == '\n' && prev != '\r') {
            out[o++] = '\r';
            out[o++] = '\n';
        } else {
            out[o++] = c;
        }
        prev = c;
    }

    if ((uint32_t)o > tud_cdc_write_available()) return;
    tud_cdc_write(out, (uint32_t)o);
    tud_cdc_write_flush();
}

static void probe_printf(const char *fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;

    cdc_write_chars(buf, n);
    stdio_filter_driver(&stdio_uart);
    stdio_put_string(buf, n, false, true);
    stdio_filter_driver(NULL);
}

static void status_task(void) {
    static uint32_t next_ms = 0;
    uint32_t const now = board_millis();
    if ((int32_t)(now - next_ms) < 0) return;
    next_ms = now + 1000;

    uint32_t const fb = last_feedback;
    probe_printf("mounted=%d cdc=%d alt=%u rate=%lu queued=%lu target=%lu fb=%lu.%02lu smp/frame gain=%u drops=%lu midi_rx=%lu\n",
                 tud_mounted(), stdio_usb_connected(), streaming_alt, (unsigned long)current_sample_rate,
                 (unsigned long)last_queued, (unsigned long)(current_sample_rate * 3 / 1000),
                 (unsigned long)(fb >> 16), (unsigned long)(((fb & 0xffff) * 100) >> 16),
                 gain_q15, (unsigned long)overflow_count, (unsigned long)midi_rx_count);
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
    static uint32_t start_ms = 0;
    static bool led_state = false;

    // Blink every interval ms
    if (board_millis() - start_ms < blink_interval_ms) return;
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state;
}

#if CFG_AUDIO_DEBUG
//--------------------------------------------------------------------+
// HID interface for audio debug
//--------------------------------------------------------------------+
// Every 1ms, we will sent 1 debug information report
void audio_debug_task(void) {
    static uint32_t start_ms = 0;
    uint32_t curr_ms = board_millis();
    if (start_ms == curr_ms) return; // not enough time
    start_ms = curr_ms;

    audio_debug_info_t debug_info;
    debug_info.sample_rate = current_sample_rate;
    debug_info.alt_settings = current_alt_settings;
    debug_info.fifo_size = CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ;
    debug_info.fifo_count = fifo_count;
    debug_info.fifo_count_avg = (uint16_t)(fifo_count_avg >> 16);
    for (int i = 0; i < CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1; i++) {
        debug_info.mute[i] = mute[i];
        debug_info.volume[i] = volume[i];
    }

    if (tud_hid_ready())
        tud_hid_report(0, &debug_info, sizeof(debug_info));
}

// Invoked when received GET_REPORT control request
// Unused here
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // TODO not Implemented
    (void)itf;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// Unused here
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // This example doesn't use multiple report and report ID
    (void)itf;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}

#endif
