// Davies-style tempo estimation: adaptive-threshold ODF (streaming), x2 rate (per Essentia),
// Rayleigh-weighted autocorrelation over a sliding buffer. Full comb-filterbank + Viterbi (TempoTapDeg)
// is omitted on ESP32; see MTG/essentia tempotapdegara.cpp.

#pragma once

#include "bpm_config.hpp"
#include <cstdint>

namespace bpm {

class OdfAdaptiveThreshold {
public:
    void reset();
    float push(float x);

private:
    static constexpr int kWin = 17;
    float buf_[kWin]{};
    int idx_ = 0;
    int filled_ = 0;
    float sum_ = 0.0f;
};

class DaviesStyleTempoEstimator {
public:
    explicit DaviesStyleTempoEstimator(float odf_sample_rate_x2);
    void reset();
    void pushOdfSample(float adapt_thresholded);
    float instantBpm() const { return last_bpm_; }
    bool hasEstimate() const { return has_estimate_; }

private:
    float estimateBpmFromBuffer(const float* linear_odf, int n) const;
    void linearizeSnapshot(float* dst) const;

    float sr_odf_;
    float rayleigh_weight_[256]{};

    static constexpr int kBuffer = 512;
    float odf_ring_[kBuffer]{};
    int ring_w_ = 0;
    int ring_count_ = 0;

    int min_lag_ = 1;
    int max_lag_ = 1;

    int push_counter_ = 0;
    float last_bpm_ = 0.0f;
    bool has_estimate_ = false;
};

class EssentiaStyleOdfBranch {
public:
    explicit EssentiaStyleOdfBranch(float odf_sample_rate_x2);
    void reset();
    void pushStftHop(float raw_odf);

    float instantBpm() const { return tempo_.instantBpm(); }
    bool hasEstimate() const { return tempo_.hasEstimate(); }

private:
    void pushOneDoubledSample(float s);

    OdfAdaptiveThreshold adapt_;
    DaviesStyleTempoEstimator tempo_;
    bool has_prev_raw_ = false;
    float prev_raw_odf_ = 0.0f;
};

} // namespace bpm
