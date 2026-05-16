// BPM analyzer configuration aligned with Essentia BeatTrackerMultiFeature spectral branch:
// STFT 2048 / 1024, Hann; ODF x2 upsample before TempoTapDegara-style tempo estimation.
// Reference: https://github.com/MTG/essentia/blob/master/src/algorithms/rhythm/beattrackermultifeature.cpp

#pragma once

#include <cstddef>
#include <cstdint>

namespace bpm {

struct BpmConfig {
    static constexpr int kSampleRate = 44100;
    static constexpr int kFftSize = 2048;
    static constexpr int kHopSize = 1024;
    static constexpr int kSpectrumBins = kFftSize / 2 + 1;
    static constexpr int kMelBands = 40;
    static constexpr float kMelLowHz = 0.0f;
    static constexpr float kMelHighHz = 4000.0f;
    static constexpr int kMinTempo = 40;
    static constexpr int kMaxTempo = 208;
    static constexpr int kMonoRingMask = 4095;
    static constexpr size_t kMonoRingSize = 4096;

    static constexpr float odfSampleRateBase() {
        return static_cast<float>(kSampleRate) / static_cast<float>(kHopSize);
    }
    static constexpr float odfSampleRateX2() {
        return odfSampleRateBase() * 2.0f;
    }
};

} // namespace bpm
