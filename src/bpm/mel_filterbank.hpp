// 40-band Mel filterbank (HTK mel warp), 0..4000 Hz — matches Essentia OnsetConfiguration defaults
// for melflux in onsetdetection.cpp (numberBands 40, bounds 0..4000).

#pragma once

#include "bpm_config.hpp"
#include <cstdint>

namespace bpm {

class MelFilterbank {
public:
    MelFilterbank();
    void applyMagnitude(const float* mag_spectrum, float* out_bands) const;

private:
    static constexpr int kBands = BpmConfig::kMelBands;
    float edge_hz_[kBands + 2]{};
    float fft_bins_hz_;
};

} // namespace bpm
