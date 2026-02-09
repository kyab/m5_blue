// Phase 3: Bluetooth A2DP + Module Audio + Custom Audio Processing
// Using I2SStream and set_raw_stream_reader_writer (original approach)
// Audio processing happens in-place within the A2DP callback, no separate task needed

#include <M5Unified.h>
#include "audio_i2c.hpp"
#include "es8388.hpp"
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include "RingBuffer.hpp"

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

// I2SStream for audio output (managed by A2DP library)
I2SStream i2s;

// Bluetooth A2DP Sink with I2SStream
BluetoothA2DPSink a2dp_sink(i2s);

// Ring buffer for delay effect
RingBufferInterleaved *g_ring = nullptr;

// Effect flags
bool g_effect_blue = false; // Volume reduction
bool g_effect_red = false;  // Delay effect

// Audio callback - modify audio data in-place
// This is called by the A2DP library, and the modified data is then written to I2S by the library
void audio_callback(int16_t *data, uint32_t sample_num) {
    // DEBUG: Log first call
    static bool first_call = true;
    if (first_call) {
        ESP_LOGI("audio", "*** audio_callback FIRST CALL *** samples=%lu", sample_num);
        first_call = false;
    }

    // Apply volume effect
    if (g_effect_blue) {
        for (uint32_t i = 0; i < sample_num; i++) {
            int16_t *left = &data[i * 2];
            int16_t *right = &data[i * 2 + 1];
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

    // Periodic log
    static uint32_t processed_samples = 0;
    processed_samples += sample_num;
    if (processed_samples >= 44100) {
        processed_samples = 0;
        // ESP_LOGI("audio", "audio_callback (HP: %s)",
        //          device.getHPInsertStatus() ? "Yes" : "No");
        ESP_LOGI("audio", "audio_callback");
    }
}

// Connection state callback
void connection_state_callback(esp_a2d_connection_state_t state, void *ptr) {
    const char *state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
    ESP_LOGI("a2dp", "Connection state: %s", state_str[state]);

    // Update display
    M5.Display.fillRect(0, 180, 320, 60, BLACK);
    M5.Display.setCursor(0, 180);
    M5.Display.setTextColor(state == ESP_A2D_CONNECTION_STATE_CONNECTED ? GREEN : YELLOW);
    M5.Display.printf("BT: %s", state_str[state]);
}

void setup() {
    // Initialize M5Unified
    auto cfg = M5.config();
    M5.begin(cfg);

    Serial.begin(115200);
    delay(1000);

    // Reduce log level to avoid performance issues
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("BT_AV", ESP_LOG_WARN); // Suppress frequent BT logs

    ESP_LOGI("main", "=== Phase 3: BT + Module Audio (I2SStream approach) ===");
    ESP_LOGI("main", "Available Heap: %zu", esp_get_free_heap_size());

    // Display setup
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(WHITE);
    M5.Display.println("\nM5Blue - Phase 3");
    M5.Display.println("I2SStream approach");
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

    // Initialize ES8388 codec
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

    // Configure ES8388 for DAC output
    es8388.setDACOutput(DAC_OUTPUT_OUT1);
    es8388.setDACVolume(100);
    es8388.setBitsSample(ES_MODULE_DAC, BIT_LENGTH_16BITS);
    es8388.setSampleRate(SAMPLE_RATE_44K);

    // Configure MCLK output on GPIO0 (required for ES8388)
    ESP_LOGI("main", "Configuring MCLK output on GPIO0...");
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFF0);

    // Initialize ring buffer for delay effect
    ESP_LOGI("main", "Initializing ring buffer...");
    g_ring = new RingBufferInterleaved();
    ESP_LOGI("main", "Ring buffer OK");

    // Disable internal speaker (use Module Audio instead)
    M5.Speaker.end();

    // Configure I2SStream for Module Audio
    // Must call end() first to clear default config, then reconfigure
    ESP_LOGI("main", "Configuring I2SStream for Module Audio...");
    i2s.end(); // Clear any default config
    auto i2s_cfg = i2s.defaultConfig();
    i2s_cfg.pin_bck = SYS_I2S_SCLK_PIN;  // GPIO 19
    i2s_cfg.pin_ws = SYS_I2S_LRCK_PIN;   // GPIO 27
    i2s_cfg.pin_data = SYS_I2S_DOUT_PIN; // GPIO 2
    i2s.begin(i2s_cfg);
    ESP_LOGI("main", "I2SStream configured: BCK=%d, WS=%d, DATA=%d",
             SYS_I2S_SCLK_PIN, SYS_I2S_LRCK_PIN, SYS_I2S_DOUT_PIN);

    // Setup A2DP callbacks
    a2dp_sink.set_on_connection_state_changed(connection_state_callback);

    // Set raw stream reader/writer callback
    // This allows modifying audio data in-place before I2S output
    a2dp_sink.set_raw_stream_reader_writer(audio_callback);

    // Start A2DP sink
    ESP_LOGI("main", "Starting Bluetooth A2DP Sink...");
    M5.Display.setTextColor(CYAN);
    M5.Display.println("\nStarting BT...");
    a2dp_sink.start("M5Blue");

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
