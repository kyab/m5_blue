// Phase 3: Bluetooth A2DP + Module Audio + in-place audio processing

#include <M5Unified.h>
#include "audio_i2c.hpp"
#include "es8388.hpp"
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include "RingBuffer.hpp"
#include "DJFilter.hpp"
#include "esp_random.h"
#include <cmath>

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

// I2SStream for audio output (managed by A2DP library)
I2SStream i2s;

// Bluetooth A2DP Sink with I2SStream
BluetoothA2DPSink a2dp_sink(i2s);

// Ring buffer for delay effect
RingBufferInterleaved* g_ring = nullptr;

// Effect flags
bool g_effect_blue = false; // Volume reduction
bool g_effect_red = false;  // Delay effect

// DJ Filter (Going-Zero port): knob-driven LPF/HPF crossover
static DJFilter g_dj_filter;
// Scratch buffers for int16<->float deinterleave in audio_callback (non-reentrant: BT callback is single-threaded)
static const uint32_t kMaxCallbackSamples = 1024;
static float s_dj_left[kMaxCallbackSamples];
static float s_dj_right[kMaxCallbackSamples];

static const uint32_t kSampleRate = 44100;

// When the A2DP source sends long runs of digital silence (pause, track gap, app mute),
// some DAC paths treat "all zero" PCM as an idle state and create audible clicks at
// resume. If a block's peak is below this threshold, mix in sub-audible TPDF dither
// so the I2S stream never stays stuck at exact zero.
static const int32_t kSilenceDitherPeakThreshold = 16;  // |int16| peak per block
static const int32_t kSilenceDitherLsbScale = 2;        // ~2 LSB RMS; >>15 after TPDF

volatile esp_a2d_connection_state_t g_bt_connection_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
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

// Audio callback - process in-place; ESP32-A2DP writes the modified buffer to I2S.
void audio_callback(int16_t* data, uint32_t sample_num) {
    static bool first_call = true;
    if (first_call) {
        ESP_LOGI("audio", "*** audio_callback FIRST CALL *** samples=%lu", sample_num);
        first_call = false;
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

    // DJ Filter (Going-Zero): deinterleave int16 -> float, process, reinterleave back with clamp.
    // Cap at kMaxCallbackSamples; leftover tail (if any) bypasses DJ filter.
    {
        uint32_t n = (sample_num > kMaxCallbackSamples) ? kMaxCallbackSamples : sample_num;
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

    // Idle-zero prevention: add uncorrelated TPDF dither only for (near-)silent blocks.
    {
        int32_t peak = 0;
        const uint32_t total = sample_num * 2u;
        for (uint32_t i = 0; i < total; i++) {
            int32_t s = data[i];
            if (s < 0) s = -s;
            if (s > peak) peak = s;
        }
        if (peak <= kSilenceDitherPeakThreshold) {
            for (uint32_t i = 0; i < sample_num; i++) {
                uint32_t wL = esp_random();
                uint32_t wR = esp_random();
                int32_t dL = (int32_t)(wL & 0x7FFFu) - (int32_t)((wL >> 15) & 0x7FFFu);
                int32_t dR = (int32_t)(wR & 0x7FFFu) - (int32_t)((wR >> 15) & 0x7FFFu);
                int32_t addL = (dL * kSilenceDitherLsbScale) >> 15;
                int32_t addR = (dR * kSilenceDitherLsbScale) >> 15;
                int32_t nl = (int32_t)data[i * 2] + addL;
                int32_t nr = (int32_t)data[i * 2 + 1] + addR;
                if (nl > 32767) nl = 32767;
                else if (nl < -32768) nl = -32768;
                if (nr > 32767) nr = 32767;
                else if (nr < -32768) nr = -32768;
                data[i * 2] = static_cast<int16_t>(nl);
                data[i * 2 + 1] = static_cast<int16_t>(nr);
            }
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
    g_bt_connection_state = state;
    g_bt_state_display_dirty = true;

}

// #region agent log
void audio_state_callback_debug(esp_a2d_audio_state_t state, void* obj) {
    (void)obj;
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
    esp_log_level_set("BT_AV", ESP_LOG_WARN); // Suppress frequent BT logs

    ESP_LOGI("main", "=== Phase 3: BT + Module Audio (I2SStream approach) ===");
    ESP_LOGI("main", "Available Heap: %zu", esp_get_free_heap_size());

    // Display setup
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.println("\nM5Blue - Phase 3");
    M5.Display.println("A2DP in-place processing");
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

    // Setup A2DP callbacks
    a2dp_sink.set_on_connection_state_changed(connection_state_callback);
    a2dp_sink.set_on_audio_state_changed(audio_state_callback_debug, nullptr);

    // Process audio in-place; ESP32-A2DP writes the modified buffer to I2S.
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
    update_bt_status_display();

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

    // Rotation angle unit (Grove B): drive DJ filter every loop (~100 Hz), log/display at 5 Hz.
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

        // (A) Oversampling: read N times in a burst and average (~1ms total on GPIO36)
        const int kOversampleN = 16;
        int mV_sum = 0;
        int raw_sum = 0;
        for (int i = 0; i < kOversampleN; i++) {
            mV_sum += analogReadMilliVolts(ROTATION_ANGLE_GPIO);
            raw_sum += analogRead(ROTATION_ANGLE_GPIO);
        }
        int mV_avg = mV_sum / kOversampleN;
        int raw_avg = raw_sum / kOversampleN;

        // (B) EMA low-pass filter: mV_filt = alpha * mV_avg + (1 - alpha) * mV_filt
        // alpha=0.2 at ~100Hz loop => time constant ~50ms (enough smoothing, still responsive)
        static float s_mV_filt = 0.0f;
        static float s_raw_filt = 0.0f;
        static bool s_ema_init = false;
        const float kEmaAlpha = 0.2f;
        if (!s_ema_init) {
            s_mV_filt = (float)mV_avg;
            s_raw_filt = (float)raw_avg;
            s_ema_init = true;
        } else {
            s_mV_filt = kEmaAlpha * (float)mV_avg + (1.0f - kEmaAlpha) * s_mV_filt;
            s_raw_filt = kEmaAlpha * (float)raw_avg + (1.0f - kEmaAlpha) * s_raw_filt;
        }
        int mV = (int)s_mV_filt;
        int raw = (int)s_raw_filt;

        // Track observed min/max of the smoothed readings (calibration aid, serial log)
        static int s_mV_min = 99999;
        static int s_mV_max = -1;
        static int s_raw_min = 99999;
        static int s_raw_max = -1;
        if (mV < s_mV_min) s_mV_min = mV;
        if (mV > s_mV_max) s_mV_max = mV;
        if (raw < s_raw_min) s_raw_min = raw;
        if (raw > s_raw_max) s_raw_max = raw;

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
        g_dj_filter.setFilterValue(v);

        static uint32_t last_rotation_dump_ms = 0;
        uint32_t now_ms = (uint32_t)millis();
        if (now_ms - last_rotation_dump_ms >= 200) {
            last_rotation_dump_ms = now_ms;
            const char* mode = (v == 0.0f) ? "BYPASS" : (v < 0.0f ? "LPF" : "HPF");
            ESP_LOGI("main", "Rotation mV=%d (min=%d max=%d) raw=%d (min=%d max=%d) v=%.3f mode=%s",
                     mV, s_mV_min, s_mV_max, raw, s_raw_min, s_raw_max, (double)v, mode);
            // mV + filter value line (row 200, yellow)
            M5.Display.fillRect(0, 200, 320, 20, BLACK);
            M5.Display.setCursor(0, 200);
            M5.Display.setTextColor(YELLOW);
            M5.Display.printf("mV=%4d  v=%+.2f", mV, (double)v);
            // raw line (row 220, cyan)
            M5.Display.fillRect(0, 220, 320, 20, BLACK);
            M5.Display.setCursor(0, 220);
            M5.Display.setTextColor(CYAN);
            M5.Display.printf("raw=%4d", raw);
        }
    }

    // Small delay to prevent tight loop
    delay(10);
}
