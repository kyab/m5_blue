#include "davies_tempo_estimator.hpp"
#include <cmath>
#include <cstring>

namespace bpm {

void OdfAdaptiveThreshold::reset() {
    memset(buf_, 0, sizeof(buf_));
    idx_ = 0;
    filled_ = 0;
    sum_ = 0.0f;
}

float OdfAdaptiveThreshold::push(float x) {
    if (filled_ < kWin) {
        buf_[idx_] = x;
        sum_ += x;
        idx_ = (idx_ + 1) % kWin;
        filled_++;
        return 0.0f;
    }
    float old = buf_[idx_];
    buf_[idx_] = x;
    sum_ = sum_ - old + x;
    idx_ = (idx_ + 1) % kWin;
    float mean = sum_ / static_cast<float>(kWin);
    float o = x - mean;
    return (o > 0.0f) ? o : 0.0f;
}

DaviesStyleTempoEstimator::DaviesStyleTempoEstimator(float odf_sample_rate_x2) : sr_odf_(odf_sample_rate_x2) {
    float tau120 = 60.0f * sr_odf_ / 120.0f;
    float ray2 = tau120 * tau120;

    min_lag_ = static_cast<int>(floorf(60.0f / static_cast<float>(BpmConfig::kMaxTempo) * sr_odf_));
    max_lag_ = static_cast<int>(ceilf(60.0f / static_cast<float>(BpmConfig::kMinTempo) * sr_odf_));
    if (min_lag_ < 3) min_lag_ = 3;
    if (max_lag_ > kBuffer - 1) max_lag_ = kBuffer - 1;
    if (max_lag_ <= min_lag_ + 2) max_lag_ = min_lag_ + 3;

    memset(rayleigh_weight_, 0, sizeof(rayleigh_weight_));
    float sumw = 0.0f;
    for (int L = min_lag_; L <= max_lag_; L++) {
        float tau = static_cast<float>(L);
        float w = (tau / ray2) * expf(-0.5f * tau * tau / ray2);
        rayleigh_weight_[L] = w;
        sumw += w;
    }
    if (sumw > 1e-12f) {
        for (int L = min_lag_; L <= max_lag_; L++) rayleigh_weight_[L] /= sumw;
    }
    reset();
}

void DaviesStyleTempoEstimator::reset() {
    memset(odf_ring_, 0, sizeof(odf_ring_));
    ring_w_ = 0;
    ring_count_ = 0;
    push_counter_ = 0;
    last_bpm_ = 0.0f;
    has_estimate_ = false;
}

void DaviesStyleTempoEstimator::linearizeSnapshot(float* dst) const {
    int start = (ring_w_ - ring_count_ + kBuffer) % kBuffer;
    for (int i = 0; i < ring_count_; i++) {
        dst[i] = odf_ring_[(start + i) % kBuffer];
    }
}

float DaviesStyleTempoEstimator::estimateBpmFromBuffer(const float* x, int n) const {
    float best_score = -1.0f;
    int best_lag = min_lag_;
    for (int L = min_lag_; L <= max_lag_; L++) {
        float ac = 0.0f;
        for (int t = L; t < n; t++) {
            ac += x[t] * x[t - L];
        }
        float denom = static_cast<float>(n - L);
        if (denom > 0.0f) ac /= denom;
        float score = ac * rayleigh_weight_[L];
        if (score > best_score) {
            best_score = score;
            best_lag = L;
        }
    }
    if (best_lag <= 0) return 0.0f;
    float bpm = 60.0f * sr_odf_ / static_cast<float>(best_lag);
    if (bpm < static_cast<float>(BpmConfig::kMinTempo)) bpm = static_cast<float>(BpmConfig::kMinTempo);
    if (bpm > static_cast<float>(BpmConfig::kMaxTempo)) bpm = static_cast<float>(BpmConfig::kMaxTempo);
    return bpm;
}

void DaviesStyleTempoEstimator::pushOdfSample(float adapt_thresholded) {
    odf_ring_[ring_w_] = adapt_thresholded;
    ring_w_ = (ring_w_ + 1) % kBuffer;
    if (ring_count_ < kBuffer) ring_count_++;

    push_counter_++;
    if (push_counter_ < 16) return;
    push_counter_ = 0;
    if (ring_count_ < 256) return;

    float linear[kBuffer];
    linearizeSnapshot(linear);
    float energy = 0.0f;
    for (int i = 0; i < ring_count_; i++) energy += linear[i] * linear[i];
    if (energy < 1e-8f) return;

    float bpm = estimateBpmFromBuffer(linear, ring_count_);
    if (bpm > 0.0f && std::isfinite(bpm)) {
        last_bpm_ = bpm;
        has_estimate_ = true;
    }
}

EssentiaStyleOdfBranch::EssentiaStyleOdfBranch(float odf_sample_rate_x2) : tempo_(odf_sample_rate_x2) {
    reset();
}

void EssentiaStyleOdfBranch::reset() {
    adapt_.reset();
    tempo_.reset();
    has_prev_raw_ = false;
    prev_raw_odf_ = 0.0f;
}

void EssentiaStyleOdfBranch::pushOneDoubledSample(float s) {
    float y = adapt_.push(s);
    tempo_.pushOdfSample(y);
}

void EssentiaStyleOdfBranch::pushStftHop(float raw_odf) {
    if (!has_prev_raw_) {
        has_prev_raw_ = true;
        prev_raw_odf_ = raw_odf;
        pushOneDoubledSample(raw_odf);
        return;
    }
    pushOneDoubledSample(0.5f * (prev_raw_odf_ + raw_odf));
    pushOneDoubledSample(raw_odf);
    prev_raw_odf_ = raw_odf;
}

} // namespace bpm
