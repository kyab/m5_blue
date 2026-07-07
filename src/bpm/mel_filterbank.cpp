#include "mel_filterbank.hpp"
#include <algorithm>
#include <cmath>

namespace bpm {

static float hz_to_mel_htk(float hz) {
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

static float mel_to_hz_htk(float mel) {
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

MelFilterbank::MelFilterbank() : fft_bins_hz_(static_cast<float>(BpmConfig::kSampleRate) / static_cast<float>(BpmConfig::kFftSize)) {
    const float nyquist = static_cast<float>(BpmConfig::kSampleRate) * 0.5f;
    float high = std::min(BpmConfig::kMelHighHz, nyquist - 1.0f);
    float low = BpmConfig::kMelLowHz;
    if (high <= low + 1.0f) high = low + 1.0f;

    float mel_lo = hz_to_mel_htk(low);
    float mel_hi = hz_to_mel_htk(high);
    float dmel = (mel_hi - mel_lo) / static_cast<float>(kBands + 1);
    float mel = mel_lo;
    for (int i = 0; i < kBands + 2; i++) {
        edge_hz_[i] = mel_to_hz_htk(mel);
        mel += dmel;
    }
}

void MelFilterbank::applyMagnitude(const float* mag_spectrum, float* out_bands) const {
    const int kMaxBin = BpmConfig::kSpectrumBins - 1;
    for (int b = 0; b < kBands; b++) {
        float f_l = edge_hz_[b];
        float f_c = edge_hz_[b + 1];
        float f_r = edge_hz_[b + 2];
        int k0 = static_cast<int>(floorf(f_l / fft_bins_hz_));
        int k1 = static_cast<int>(ceilf(f_r / fft_bins_hz_));
        if (k0 < 0) k0 = 0;
        if (k1 > kMaxBin) k1 = kMaxBin;
        if (k1 < k0) k1 = k0;
        float num = 0.0f;
        float den = 0.0f;
        for (int k = k0; k <= k1; k++) {
            float f = static_cast<float>(k) * fft_bins_hz_;
            float w = 0.0f;
            if (f <= f_c && f_c > f_l + 1e-6f && f >= f_l) {
                w = (f - f_l) / (f_c - f_l);
            } else if (f > f_c && f_r > f_c + 1e-6f && f <= f_r) {
                w = (f_r - f) / (f_r - f_c);
            }
            if (w > 0.0f) {
                num += w * mag_spectrum[k];
                den += w;
            }
        }
        out_bands[b] = (den > 1e-12f) ? (num / den) : 0.0f;
    }
}

} // namespace bpm
