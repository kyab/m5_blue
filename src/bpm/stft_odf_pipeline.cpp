#include "stft_odf_pipeline.hpp"
#include <Arduino.h>
#include <cmath>
#include <complex>
#include <cstring>
#include <esp_heap_caps.h>

namespace bpm {

static constexpr float kSilenceCutoff = 1e-10f;
static constexpr float kDbSilenceCutoff = -200.0f;

float StftOdfPipeline::amp2dbEssentia(float a) {
    if (a < kSilenceCutoff) return kDbSilenceCutoff;
    return 20.0f * log10f(a);
}

bool StftOdfPipeline::allocBuffers() {
    const size_t ring_bytes = BpmConfig::kMonoRingSize * sizeof(float);
    const size_t fft_bytes = static_cast<size_t>(BpmConfig::kFftSize) * sizeof(double);
    const size_t spec_bytes = static_cast<size_t>(BpmConfig::kSpectrumBins) * sizeof(float);
    const size_t mel_bytes = static_cast<size_t>(BpmConfig::kMelBands) * sizeof(float);

    uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    ring_ = static_cast<float*>(heap_caps_malloc(ring_bytes, caps));
    hann_ = static_cast<float*>(heap_caps_malloc(static_cast<size_t>(BpmConfig::kFftSize) * sizeof(float), caps));
    fft_real_ = static_cast<double*>(heap_caps_malloc(fft_bytes, caps));
    fft_imag_ = static_cast<double*>(heap_caps_malloc(fft_bytes, caps));
    mag_ = static_cast<float*>(heap_caps_malloc(spec_bytes, caps));
    phase_ = static_cast<float*>(heap_caps_malloc(spec_bytes, caps));
    phase_1_ = static_cast<float*>(heap_caps_malloc(spec_bytes, caps));
    phase_2_ = static_cast<float*>(heap_caps_malloc(spec_bytes, caps));
    spectrum_1_ = static_cast<float*>(heap_caps_malloc(spec_bytes, caps));
    mel_linear_ = static_cast<float*>(heap_caps_malloc(mel_bytes, caps));
    prev_mel_db_ = static_cast<float*>(heap_caps_malloc(mel_bytes, caps));

    // Avoid mirroring FFT/STFT allocations into internal DRAM: they are large (~100KiB+) and starve FreeRTOS /
    // Arduino boot-time task stacks (startup assert in esp_startup_start_app_common).
    if (!ring_ || !hann_ || !fft_real_ || !fft_imag_ || !mag_ || !phase_ || !phase_1_ || !phase_2_ || !spectrum_1_ || !mel_linear_
        || !prev_mel_db_) {
        freeBuffers();
        return false;
    }

    return true;
}

void StftOdfPipeline::freeBuffers() {
    heap_caps_free(ring_);
    heap_caps_free(hann_);
    heap_caps_free(fft_real_);
    heap_caps_free(fft_imag_);
    heap_caps_free(mag_);
    heap_caps_free(phase_);
    heap_caps_free(phase_1_);
    heap_caps_free(phase_2_);
    heap_caps_free(spectrum_1_);
    heap_caps_free(mel_linear_);
    heap_caps_free(prev_mel_db_);
    ring_ = nullptr;
    hann_ = nullptr;
    fft_real_ = nullptr;
    fft_imag_ = nullptr;
    mag_ = nullptr;
    phase_ = nullptr;
    phase_1_ = nullptr;
    phase_2_ = nullptr;
    spectrum_1_ = nullptr;
    mel_linear_ = nullptr;
    prev_mel_db_ = nullptr;
}

StftOdfPipeline::StftOdfPipeline() {
    if (!allocBuffers()) {
        Serial.println("bpm: StftOdfPipeline alloc failed");
        return;
    }
    fft_engine_ = new arduinoFFT(fft_real_, fft_imag_, BpmConfig::kFftSize, static_cast<double>(BpmConfig::kSampleRate));
    const int N = BpmConfig::kFftSize;
    for (int n = 0; n < N; n++) {
        hann_[n] = 0.5f * (1.0f - cosf((2.0f * static_cast<float>(M_PI) * n) / static_cast<float>(N - 1)));
    }
    reset();
}

StftOdfPipeline::~StftOdfPipeline() {
    delete fft_engine_;
    fft_engine_ = nullptr;
    freeBuffers();
}

void StftOdfPipeline::reset() {
    if (!ring_) return;
    memset(ring_, 0, BpmConfig::kMonoRingSize * sizeof(float));
    memset(fft_real_, 0, static_cast<size_t>(BpmConfig::kFftSize) * sizeof(double));
    memset(fft_imag_, 0, static_cast<size_t>(BpmConfig::kFftSize) * sizeof(double));
    memset(mag_, 0, static_cast<size_t>(BpmConfig::kSpectrumBins) * sizeof(float));
    memset(phase_, 0, static_cast<size_t>(BpmConfig::kSpectrumBins) * sizeof(float));
    memset(phase_1_, 0, static_cast<size_t>(BpmConfig::kSpectrumBins) * sizeof(float));
    memset(phase_2_, 0, static_cast<size_t>(BpmConfig::kSpectrumBins) * sizeof(float));
    memset(spectrum_1_, 0, static_cast<size_t>(BpmConfig::kSpectrumBins) * sizeof(float));
    memset(prev_mel_db_, 0, static_cast<size_t>(BpmConfig::kMelBands) * sizeof(float));
    rms_old_ = 0.0;
    first_melflux_ = true;
    first_rms_ = true;
    mono_total_ = 0;
    fresh_odf_ = false;
}

void StftOdfPipeline::pushMonoFloat(float x) {
    if (!ring_ || !fft_engine_) return;
    const uint32_t mask = static_cast<uint32_t>(BpmConfig::kMonoRingMask);
    ring_[mono_total_ & mask] = x;
    mono_total_++;
    if (mono_total_ < static_cast<uint32_t>(BpmConfig::kFftSize)) return;
    if (((mono_total_ - static_cast<uint32_t>(BpmConfig::kFftSize)) % static_cast<uint32_t>(BpmConfig::kHopSize)) != 0) return;
    uint32_t end = (mono_total_ - 1) & mask;
    processFrame(end);
}

void StftOdfPipeline::processFrame(uint32_t end_idx_masked) {
    const uint32_t mask = static_cast<uint32_t>(BpmConfig::kMonoRingMask);
    for (int i = 0; i < BpmConfig::kFftSize; i++) {
        int32_t pos = static_cast<int32_t>(end_idx_masked) - (BpmConfig::kFftSize - 1) + i;
        pos &= static_cast<int32_t>(mask);
        float s = ring_[static_cast<uint32_t>(pos)];
        fft_real_[i] = static_cast<double>(s * hann_[i]);
        fft_imag_[i] = 0.0;
    }
    fft_engine_->Compute(FFT_FORWARD);
    magnitudePhaseFromFft();
    last_odf_.complex_odf = onsetComplex(mag_, phase_);
    last_odf_.rms_odf = onsetRms(mag_);
    last_odf_.melflux_odf = onsetMelFlux(mag_);
    fresh_odf_ = true;
}

void StftOdfPipeline::magnitudePhaseFromFft() {
    const int half = BpmConfig::kFftSize / 2;
    for (int k = 0; k <= half; k++) {
        double re = fft_real_[k];
        double im = fft_imag_[k];
        mag_[k] = static_cast<float>(sqrt(re * re + im * im));
        phase_[k] = static_cast<float>(atan2(im, re));
    }
}

float StftOdfPipeline::onsetComplex(const float* mag, const float* ph) {
    float sum = 0.0f;
    const int n = BpmConfig::kSpectrumBins;
    for (int i = 0; i < n; i++) {
        float target_phase = 2.0f * phase_1_[i] - phase_2_[i];
        target_phase = fmodf(target_phase + float(M_PI), -2.0f * float(M_PI)) + float(M_PI);
        std::complex<float> pred(spectrum_1_[i], 0.0f);
        std::complex<float> meas = std::polar(mag[i], ph[i] - target_phase);
        float d = std::abs(pred - meas);
        sum += d;
    }
    memcpy(phase_2_, phase_1_, static_cast<size_t>(n) * sizeof(float));
    memcpy(phase_1_, ph, static_cast<size_t>(n) * sizeof(float));
    memcpy(spectrum_1_, mag, static_cast<size_t>(n) * sizeof(float));
    return sum;
}

float StftOdfPipeline::onsetRms(const float* mag) {
    double energy = 0.0;
    const int n = BpmConfig::kSpectrumBins;
    for (int i = 0; i < n; i++) {
        double m = mag[i];
        energy += m * m;
    }
    double rms = sqrt(energy) / static_cast<double>(n);
    if (first_rms_) {
        first_rms_ = false;
        rms_old_ = rms;
        return 0.0f;
    }
    float o = static_cast<float>(rms - rms_old_);
    if (o < 0.0f) o = 0.0f;
    rms_old_ = rms;
    return o;
}

float StftOdfPipeline::onsetMelFlux(const float* mag) {
    mel_.applyMagnitude(mag, mel_linear_);
    float mel_db[BpmConfig::kMelBands];
    for (int b = 0; b < BpmConfig::kMelBands; b++) {
        mel_db[b] = amp2dbEssentia(mel_linear_[b]);
    }
    if (first_melflux_) {
        first_melflux_ = false;
        memcpy(prev_mel_db_, mel_db, sizeof(mel_db));
        return 0.0f;
    }
    float flux = 0.0f;
    for (int b = 0; b < BpmConfig::kMelBands; b++) {
        float diff = mel_db[b] - prev_mel_db_[b];
        if (diff > 0.0f) flux += diff;
        prev_mel_db_[b] = mel_db[b];
    }
    return flux;
}

bool StftOdfPipeline::takeOdfIfNew(OdfTriplet& out) {
    if (!fresh_odf_) return false;
    fresh_odf_ = false;
    out = last_odf_;
    return true;
}

} // namespace bpm
