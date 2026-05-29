#include "streaming_bpm_analyzer.hpp"
#include <Arduino.h>
#include <algorithm>
#include <cstring>
#include <esp_heap_caps.h>
#include <esp_log.h>

namespace bpm {

static float median3(float a, float b, float c) {
    float x = a, y = b, z = c;
    if (x > y) {
        float t = x;
        x = y;
        y = t;
    }
    if (y > z) {
        float t = y;
        y = z;
        z = t;
    }
    if (x > y) {
        float t = x;
        x = y;
        y = t;
    }
    return y;
}

StreamingBpmAnalyzer::StreamingBpmAnalyzer()
    : branch_complex_(BpmConfig::odfSampleRateX2()),
      branch_rms_(BpmConfig::odfSampleRateX2()),
      branch_mel_(BpmConfig::odfSampleRateX2()) {
    fifo_buf_ = static_cast<uint8_t*>(heap_caps_malloc(kFifoBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (fifo_buf_ == nullptr) {
        ESP_LOGE("bpm", "FIFO alloc failed (PSRAM required; %u bytes)", (unsigned)kFifoBytes);
    }
}

StreamingBpmAnalyzer::~StreamingBpmAnalyzer() {
    if (fifo_buf_ != nullptr) {
        heap_caps_free(fifo_buf_);
        fifo_buf_ = nullptr;
    }
}

void StreamingBpmAnalyzer::reset() {
    head_ = tail_ = used_ = 0;
    display_bpm_ = 0.0f;
    stft_.reset();
    branch_complex_.reset();
    branch_rms_.reset();
    branch_mel_.reset();
}

float StreamingBpmAnalyzer::beatDurationSec() const {
    if (display_bpm_ < 1.0f) return 0.0f;
    return 60.0f / display_bpm_;
}

void StreamingBpmAnalyzer::fuseMedianEwma() {
    float vals[3];
    int n = 0;
    if (branch_complex_.hasEstimate()) vals[n++] = branch_complex_.instantBpm();
    if (branch_rms_.hasEstimate()) vals[n++] = branch_rms_.instantBpm();
    if (branch_mel_.hasEstimate()) vals[n++] = branch_mel_.instantBpm();
    if (n == 0) return;

    float fused = vals[0];
    if (n == 2) fused = 0.5f * (vals[0] + vals[1]);
    else if (n == 3) fused = median3(vals[0], vals[1], vals[2]);

    if (display_bpm_ < 0.5f) {
        display_bpm_ = fused;
    } else {
        display_bpm_ = 0.88f * display_bpm_ + 0.12f * fused;
    }
}

void StreamingBpmAnalyzer::enqueueStereoInterleaved(const int16_t* pcm, size_t total_int16_values) {
    if (fifo_buf_ == nullptr || pcm == nullptr || total_int16_values < 2 || (total_int16_values & 1u)) return;
    size_t nbytes = total_int16_values * sizeof(int16_t);

    portENTER_CRITICAL(&fifo_mux_);
    while (used_ + nbytes > kFifoBytes) {
        size_t need_drop = used_ + nbytes - kFifoBytes;
        need_drop = (need_drop + 3u) & ~size_t(3);
        if (need_drop > used_) need_drop = used_;
        tail_ = (tail_ + need_drop) % kFifoBytes;
        used_ -= need_drop;
    }
    const uint8_t* src = reinterpret_cast<const uint8_t*>(pcm);
    size_t part = std::min(nbytes, kFifoBytes - head_);
    memcpy(fifo_buf_ + head_, src, part);
    if (nbytes > part) memcpy(fifo_buf_, src + part, nbytes - part);
    head_ = (head_ + nbytes) % kFifoBytes;
    used_ += nbytes;
    portEXIT_CRITICAL(&fifo_mux_);
}

void StreamingBpmAnalyzer::service() {
    if (fifo_buf_ == nullptr) return;
    for (;;) {
        int16_t L = 0;
        int16_t R = 0;
        bool got = false;
        portENTER_CRITICAL(&fifo_mux_);
        if (used_ >= sizeof(int16_t) * 2) {
            uint8_t tmp[4];
            for (size_t i = 0; i < 4; i++) {
                tmp[i] = fifo_buf_[(tail_ + i) % kFifoBytes];
            }
            L = static_cast<int16_t>(tmp[0] | (tmp[1] << 8));
            R = static_cast<int16_t>(tmp[2] | (tmp[3] << 8));
            tail_ = (tail_ + 4) % kFifoBytes;
            used_ -= 4;
            got = true;
        }
        portEXIT_CRITICAL(&fifo_mux_);
        if (!got) break;

        float m = 0.5f * static_cast<float>(L + R) * (1.0f / 32768.0f);
        stft_.pushMonoFloat(m);
        OdfTriplet tri{};
        while (stft_.takeOdfIfNew(tri)) {
            branch_complex_.pushStftHop(tri.complex_odf);
            branch_rms_.pushStftHop(tri.rms_odf);
            branch_mel_.pushStftHop(tri.melflux_odf);
            fuseMedianEwma();
        }
    }
}

namespace {

StreamingBpmAnalyzer* s_bpm_for_task = nullptr;

void bpm_worker_task(void* arg) {
    (void)arg;
    if (s_bpm_for_task == nullptr) {
        vTaskDelete(nullptr);
        return;
    }
    for (;;) {
        s_bpm_for_task->service();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

} // namespace

void start_bpm_worker_task(StreamingBpmAnalyzer* analyzer) {
    if (analyzer == nullptr) return;
    s_bpm_for_task = analyzer;
    // FreeRTOS stack depth is in StackType_t words (see Arduino xTaskCreatePinnedToCore); 4096 matches I2S writer scale.
    constexpr uint32_t kBpmWorkerStackWords = 4096;
    BaseType_t ok = xTaskCreatePinnedToCore(bpm_worker_task, "bpm_work", kBpmWorkerStackWords, nullptr, 1, nullptr, 0);
    if (ok != pdPASS) {
        ESP_LOGE("bpm", "failed to create bpm_work task");
    }
}

} // namespace bpm
