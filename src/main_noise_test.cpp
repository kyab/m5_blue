// Noise floor observation test.
//
// Purpose:
//   Compare audible hiss/noise between USB-powered and battery-powered operation
//   on M5Stack Core2 + Module Audio (ES8388) + M5GO Bottom2.
//
// Design (per ChatGPT discussion, share/69e63dce-34dc-83ab-bdd7-f8aa55aae4d3):
//   - Disable BLE/A2DP/Wi-Fi so their RF activity cannot contribute.
//   - Keep DAC / HP output analog path alive (do NOT use ES8388 DAC mute).
//   - Drive a pure I2S stream with three alternating segments:
//       1) 1 kHz sine (reference tone)
//       2) PCM all zero (observe "DAC alive but zero input" hiss)
//       3) 1 kHz sine (reference again, for A/B repeat)
//       4) +-1 LSB dither (avoid any possible zero-detect auto-mute; per ChatGPT)
//   - Repeat forever so the user can flip between USB and battery in situ.
//
// Build: enable via the dedicated PlatformIO env `m5stack-core2-noise-test`.
// The default env (`m5stack-core2`) continues to build the full A2DP/DJ app
// from main.cpp.

#ifdef NOISE_TEST_BUILD

#include <M5Unified.h>
#include <Wire.h>
#include "audio_i2c.hpp"
#include "es8388.hpp"
#include "AudioTools.h"

// Explicitly disable BLE/Classic BT controller and Wi-Fi so they cannot
// inject RF / digital noise during the measurement.
#include "esp_bt.h"
#include "esp_wifi.h"

// MCLK output configuration (required for ES8388)
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"

#include <cmath>

// Pin assignment: Core2 + Module Audio on M-Bus (same mapping as main.cpp).
#define SYS_I2C_SDA_PIN 21
#define SYS_I2C_SCL_PIN 22

#define SYS_I2S_MCLK_PIN 0
#define SYS_I2S_SCLK_PIN 19
#define SYS_I2S_LRCK_PIN 27
#define SYS_I2S_DOUT_PIN 2
#define SYS_I2S_DIN_PIN 34

// Test parameters
static const uint32_t kSampleRate = 44100;
static const float kToneFreqHz = 1000.0f;        // 1 kHz reference
static const float kToneAmplitude = 0.15f;       // small: typical observation level, room for headroom
static const uint32_t kSegmentSec = 2;           // length of each segment
static const uint32_t kBlockSamples = 256;       // frames per I2S write (stereo 16-bit -> 1024 B)

// Dither LFSR (Galois, 32-bit) for +/-1 LSB dither segment.
static uint32_t g_lfsr = 0xACE1u;
static inline int16_t dither_lsb() {
    g_lfsr = (g_lfsr >> 1) ^ (0x80200003u & (uint32_t)(-(int32_t)(g_lfsr & 1)));
    return (g_lfsr & 1u) ? (int16_t)1 : (int16_t)-1;
}

// Segment kinds
enum Segment {
    SEG_SINE_A = 0,
    SEG_ZERO   = 1,
    SEG_SINE_B = 2,
    SEG_DITHER = 3,
    SEG_COUNT  = 4,
};

static const char* segment_name(Segment s) {
    switch (s) {
    case SEG_SINE_A: return "SINE_A";
    case SEG_ZERO:   return "ZERO  ";
    case SEG_SINE_B: return "SINE_B";
    case SEG_DITHER: return "DITHER";
    case SEG_COUNT:  break;
    }
    return "?";
}

// Globals
AudioI2c g_device;
ES8388 g_es8388(&Wire, SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN);
I2SStream g_i2s;

static int16_t g_block[kBlockSamples * 2];  // stereo
static double g_phase = 0.0;                // carry sine phase across blocks to avoid clicks

static void fill_sine_block() {
    const double inc = 2.0 * M_PI * (double)kToneFreqHz / (double)kSampleRate;
    const float amp = kToneAmplitude * 32767.0f;
    for (uint32_t i = 0; i < kBlockSamples; i++) {
        float v = amp * (float)sin(g_phase);
        g_phase += inc;
        if (g_phase >= 2.0 * M_PI) g_phase -= 2.0 * M_PI;
        int16_t s = (int16_t)v;
        g_block[i * 2 + 0] = s;
        g_block[i * 2 + 1] = s;
    }
}

static void fill_zero_block() {
    memset(g_block, 0, sizeof(g_block));
}

static void fill_dither_block() {
    for (uint32_t i = 0; i < kBlockSamples; i++) {
        int16_t d = dither_lsb();
        g_block[i * 2 + 0] = d;
        g_block[i * 2 + 1] = d;
    }
}

static void disable_radio() {
    // Wi-Fi off (idempotent; safe to call even if never started)
    esp_err_t e1 = esp_wifi_stop();
    esp_err_t e2 = esp_wifi_deinit();
    // BT controller off (Arduino core may leave it initialized in some configs)
    esp_err_t e3 = esp_bt_controller_disable();
    esp_err_t e4 = esp_bt_controller_deinit();
    // Release BT memory so nothing is broadcast/scanning
    esp_err_t e5 = esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    Serial.printf("[radio] wifi_stop=%d wifi_deinit=%d bt_disable=%d bt_deinit=%d bt_mem=%d\n",
                  (int)e1, (int)e2, (int)e3, (int)e4, (int)e5);
}

static void log_power_state() {
    // Report USB vs battery state via M5Unified power API so the serial log
    // records the actual power situation during each phase of the test.
    bool charging = M5.Power.isCharging();
    int level = M5.Power.getBatteryLevel();
    float volt = M5.Power.getBatteryVoltage() / 1000.0f;  // mV -> V
    Serial.printf("[power] charging=%d batt_level=%d%% batt_volt=%.2fV\n",
                  (int)charging, level, (double)volt);
}

static void draw_header() {
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(0, 0);
    M5.Display.println("I2S Noise Test");
    M5.Display.setTextColor(CYAN);
    M5.Display.printf("fs=%u  tone=%.0fHz\n", (unsigned)kSampleRate, (double)kToneFreqHz);
    M5.Display.printf("amp=%.2f (sine/0/sine/dither)\n", (double)kToneAmplitude);
    M5.Display.setTextColor(YELLOW);
    M5.Display.println("BT: OFF  WiFi: OFF");
}

static void draw_segment(Segment seg, uint32_t cycle) {
    M5.Display.fillRect(0, 100, 320, 140, BLACK);
    M5.Display.setCursor(0, 100);
    M5.Display.setTextSize(3);
    uint16_t color = WHITE;
    switch (seg) {
    case SEG_SINE_A: color = GREEN;  break;
    case SEG_ZERO:   color = RED;    break;
    case SEG_SINE_B: color = GREEN;  break;
    case SEG_DITHER: color = MAGENTA;break;
    case SEG_COUNT:  break;
    }
    M5.Display.setTextColor(color);
    M5.Display.printf("%s\n", segment_name(seg));
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.printf("cycle=%u\n", (unsigned)cycle);

    // Power state on the bottom rows
    bool charging = M5.Power.isCharging();
    int level = M5.Power.getBatteryLevel();
    float volt = M5.Power.getBatteryVoltage() / 1000.0f;
    M5.Display.printf("USB: %s\n", charging ? "CHARGING" : "(battery)");
    M5.Display.printf("batt: %d%%  %.2fV\n", level, (double)volt);
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== I2S Noise Test (BLE/A2DP/Wi-Fi OFF) ===");
    esp_log_level_set("*", ESP_LOG_WARN);

    disable_radio();
    log_power_state();

    // Display
    M5.Display.setRotation(1);
    draw_header();

    // I2C + Module Audio I2C device (for HP routing config; does NOT touch DAC mute)
    Wire.begin(SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN, 400000L);
    if (g_device.begin(&Wire, SYS_I2C_SDA_PIN, SYS_I2C_SCL_PIN)) {
        Serial.println("[i2c] AudioI2c OK");
        g_device.setHPMode(AUDIO_HPMODE_NATIONAL);
        g_device.setRGBBrightness(20);
        g_device.setRGBLED(0, 0x000040);
        g_device.setRGBLED(1, 0x000040);
        g_device.setRGBLED(2, 0x000040);
    } else {
        Serial.println("[i2c] AudioI2c NOT FOUND (continuing)");
    }

    // ES8388 init
    if (!g_es8388.init()) {
        Serial.println("[es8388] init FAILED");
    } else {
        Serial.println("[es8388] init OK");
    }
    g_es8388.setDACOutput(DAC_OUTPUT_OUT1);
    // Moderate volume; do NOT call setDACmute() on purpose for this test.
    // Keep the DAC/HP path fully "alive" while we inject sine/zero/dither.
    g_es8388.setDACVolume(80);
    g_es8388.setBitsSample(ES_MODULE_DAC, BIT_LENGTH_16BITS);
    g_es8388.setSampleRate(SAMPLE_RATE_44K);

    // Clear DAC Mute bit explicitly (DACCONTROL3 bit 1) and enable DAC Soft Ramp (bit 5).
    {
        uint8_t reg25 = 0;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL3);
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)ES8388_ADDR, (uint8_t)1) == 1) reg25 = Wire.read();
        reg25 = (reg25 & ~0x02u) | 0x20u;
        Wire.beginTransmission(ES8388_ADDR);
        Wire.write(ES8388_DACCONTROL3);
        Wire.write(reg25);
        Wire.endTransmission();
        Serial.printf("[es8388] DACCONTROL3=0x%02X (unmute, SoftRamp on)\n", reg25);
    }

    // MCLK on GPIO0 (required for ES8388)
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFF0);

    // Release M5 internal speaker (which shares NS4168 path) so I2S0 is free.
    M5.Speaker.end();

    // I2S setup
    g_i2s.end();
    auto i2s_cfg = g_i2s.defaultConfig();
    i2s_cfg.sample_rate = kSampleRate;
    i2s_cfg.channels = 2;
    i2s_cfg.bits_per_sample = 16;
    i2s_cfg.pin_bck = SYS_I2S_SCLK_PIN;
    i2s_cfg.pin_ws = SYS_I2S_LRCK_PIN;
    i2s_cfg.pin_data = SYS_I2S_DOUT_PIN;
    g_i2s.begin(i2s_cfg);
    Serial.printf("[i2s] started: fs=%u BCK=%d WS=%d DATA=%d\n",
                  (unsigned)kSampleRate, SYS_I2S_SCLK_PIN, SYS_I2S_LRCK_PIN, SYS_I2S_DOUT_PIN);

    Serial.println("[test] looping: SINE_A / ZERO / SINE_B / DITHER  (2s each)");
}

void loop() {
    static uint32_t cycle = 0;

    for (int s = 0; s < SEG_COUNT; s++) {
        Segment seg = (Segment)s;
        draw_segment(seg, cycle);
        Serial.printf("[seg] cycle=%u seg=%s\n", (unsigned)cycle, segment_name(seg));
        log_power_state();

        const uint32_t total_blocks = (kSampleRate * kSegmentSec) / kBlockSamples;
        for (uint32_t b = 0; b < total_blocks; b++) {
            switch (seg) {
            case SEG_SINE_A:
            case SEG_SINE_B:
                fill_sine_block();
                break;
            case SEG_ZERO:
                fill_zero_block();
                break;
            case SEG_DITHER:
                fill_dither_block();
                break;
            case SEG_COUNT:
                break;
            }
            g_i2s.write(reinterpret_cast<const uint8_t*>(g_block), sizeof(g_block));
        }

        // Let M5Unified service buttons/display during long-running writes.
        M5.update();
    }
    cycle++;
}

#endif  // NOISE_TEST_BUILD
