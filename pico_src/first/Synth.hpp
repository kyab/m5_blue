#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Monophonic sine synth mixed into interleaved stereo PCM (A2DP path).
// Control thread publishes targets; gen() owns phase / applied pitch (audio thread).
class Synth {
  public:
    // Low ド frequency. Swap to 130.81f (C3) or 523.25f (C5) as needed.
    static constexpr float kBaseFrequencyHz = 261.63f; // C4
    static constexpr float kSampleRateHz = 44100.0f;
    static constexpr float kLevel = 0.7f; // fraction of full scale before AVRCP volume
    static constexpr int kInitialSemitone = 7; // ソ

    Synth() {
        _requestedSemitone.store(kInitialSemitone, std::memory_order_relaxed);
        _resetRequested.store(true, std::memory_order_relaxed);
        _appliedSemitone = kInitialSemitone;
        _phase = 0.0f;
        updatePhaseIncrement(kInitialSemitone);
    }

    // 0 = low ド, 12 = octave ド. Mid-note changes keep phase continuous.
    void setSemitone(int semitone) {
        _requestedSemitone.store(semitone, std::memory_order_relaxed);
    }

    // Reset pitch (to initial ソ) and phase. Intended on Z note-off.
    void reset() {
        _requestedSemitone.store(kInitialSemitone, std::memory_order_relaxed);
        _resetRequested.store(true, std::memory_order_relaxed);
    }

    // Add mono sine into L and R. Caller gates on Z; do not call when silent.
    void gen(int16_t* interleavedStereo, uint32_t frameCount) {
        if (interleavedStereo == nullptr || frameCount == 0) return;

        if (_resetRequested.exchange(false, std::memory_order_relaxed)) {
            _phase = 0.0f;
            _appliedSemitone = kInitialSemitone;
            updatePhaseIncrement(_appliedSemitone);
        }

        const int target = _requestedSemitone.load(std::memory_order_relaxed);
        if (target != _appliedSemitone) {
            _appliedSemitone = target;
            updatePhaseIncrement(_appliedSemitone);
            // Keep _phase for click-free pitch changes.
        }

        const float amp = kLevel * 32767.0f;
        constexpr float kTwoPi = static_cast<float>(2.0 * M_PI);

        for (uint32_t i = 0; i < frameCount; ++i) {
            const float s = sinf(_phase) * amp;
            const int32_t add = static_cast<int32_t>(s);

            int32_t l = static_cast<int32_t>(interleavedStereo[i * 2]) + add;
            int32_t r = static_cast<int32_t>(interleavedStereo[i * 2 + 1]) + add;
            if (l > 32767) l = 32767;
            else if (l < -32768) l = -32768;
            if (r > 32767) r = 32767;
            else if (r < -32768) r = -32768;
            interleavedStereo[i * 2] = static_cast<int16_t>(l);
            interleavedStereo[i * 2 + 1] = static_cast<int16_t>(r);

            _phase += _phaseInc;
            if (_phase >= kTwoPi) {
                _phase -= kTwoPi;
            }
        }
    }

  private:
    void updatePhaseIncrement(int semitone) {
        const float freq = kBaseFrequencyHz * powf(2.0f, static_cast<float>(semitone) / 12.0f);
        _phaseInc = static_cast<float>(2.0 * M_PI) * freq / kSampleRateHz;
    }

    std::atomic<int> _requestedSemitone;
    std::atomic<bool> _resetRequested;
    int _appliedSemitone;
    float _phase;
    float _phaseInc;
};
