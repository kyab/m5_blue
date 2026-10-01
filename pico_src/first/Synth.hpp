#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Monophonic sine synth. gen() writes a fresh stereo buffer; mixing is done by the caller.
// Control thread publishes pitch targets; gen() applies them and advances phase (audio thread).
class Synth {
  public:
    // Low ド frequency. Swap to 130.81f (C3) or 523.25f (C5) as needed.
    static constexpr float kBaseFrequencyHz = 261.63f; // C4
    static constexpr float kSampleRateHz = 44100.0f;
    static constexpr float kLevel = 0.7f; // fraction of full scale before AVRCP volume
    static constexpr int kInitialSemitone = 7; // ソ

    Synth() {
        _requestedSemitone.store(kInitialSemitone, std::memory_order_relaxed);
        _appliedSemitone = kInitialSemitone;
        _phase = 0.0f;
        updatePhaseIncrement(kInitialSemitone);
    }

    // 0 = low ド, 12 = octave ド. Mid-note changes keep phase continuous.
    void setSemitone(int semitone) {
        _requestedSemitone.store(semitone, std::memory_order_relaxed);
    }

    // Reset pitch (to initial ソ) and phase. Call on Z note-off (not from gen).
    void reset() {
        _requestedSemitone.store(kInitialSemitone, std::memory_order_relaxed);
        _appliedSemitone = kInitialSemitone;
        _phase = 0.0f;
        updatePhaseIncrement(kInitialSemitone);
    }

    // Fill interleaved stereo with mono sine (L=R). Does not mix; caller adds onto A2DP.
    void gen(int16_t* interleavedStereo, uint32_t frameCount) {
        if (interleavedStereo == nullptr || frameCount == 0) return;

        const int target = _requestedSemitone.load(std::memory_order_relaxed);
        if (target != _appliedSemitone) {
            _appliedSemitone = target;
            updatePhaseIncrement(_appliedSemitone);
            // Keep _phase for click-free pitch changes.
        }

        const float amp = kLevel * 32767.0f;
        constexpr float kTwoPi = static_cast<float>(2.0 * M_PI);

        for (uint32_t i = 0; i < frameCount; ++i) {
            const int16_t sample = static_cast<int16_t>(sinf(_phase) * amp);
            interleavedStereo[i * 2] = sample;
            interleavedStereo[i * 2 + 1] = sample;
            _phase += _phaseInc;
        }
        while (_phase >= kTwoPi) {
            _phase -= kTwoPi;
        }
    }

  private:
    void updatePhaseIncrement(int semitone) {
        const float freq = kBaseFrequencyHz * powf(2.0f, static_cast<float>(semitone) / 12.0f);
        _phaseInc = static_cast<float>(2.0 * M_PI) * freq / kSampleRateHz;
    }

    std::atomic<int> _requestedSemitone;
    int _appliedSemitone;
    float _phase;
    float _phaseInc;
};
