#pragma once

#include "BluetoothA2DPSink.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"

// BT PCM ring buffer push (implemented in main.cpp).
size_t bt_pcm_ring_push(const uint8_t* data, size_t len);

constexpr uint32_t kA2dpExpectedSampleRate = 44100;

class BluetoothA2DPOutputQueuedI2S : public BluetoothA2DPOutput {
  public:
    bool begin() override { return true; }
    void end() override {}
    void set_sample_rate(int rate) override {
        if (rate != (int)kA2dpExpectedSampleRate) {
            ESP_LOGW("a2dp", "A2DP sample rate %d differs from fixed I2S rate %lu", rate, (unsigned long)kA2dpExpectedSampleRate);
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

    // Change CoD minor after A2DP init so the host shows a headphone icon.
    void av_hdl_a2d_evt(uint16_t event, void* p_param) override {
        BluetoothA2DPSink::av_hdl_a2d_evt(event, p_param);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
        if (event != ESP_A2D_PROF_STATE_EVT || p_param == nullptr) {
            return;
        }
        const auto* a2d = static_cast<const esp_a2d_cb_param_t*>(p_param);
        if (a2d->a2d_prof_stat.init_state != ESP_A2D_INIT_SUCCESS) {
            return;
        }
        esp_bt_cod_t cod = {};
        cod.major = ESP_BT_COD_MAJOR_DEV_AV;
        cod.minor = 0x06; // Headphones
        cod.service = ESP_BT_COD_SRVC_AUDIO | ESP_BT_COD_SRVC_RENDERING;
        esp_err_t err = esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
        if (err != ESP_OK) {
            ESP_LOGW("a2dp", "esp_bt_gap_set_cod failed: %s", esp_err_to_name(err));
        }
#endif
    }
};
