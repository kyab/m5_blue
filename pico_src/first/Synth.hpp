#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Monophonic sine synth. gen() writes a fresh stereo buffer; mixing is done by the caller.
// Voice state: Off / On / Tail (50 ms fade-out after noteOff).
class Synth {
  public:
    // Low ド frequency. Swap to 130.81f (C3) or 523.25f (C5) as needed.
    static constexpr float kBaseFrequencyHz = 261.63f; // C4
    static constexpr float kSampleRateHz = 44100.0f;
    static constexpr float kLevel = 0.8f; // fraction of full scale before AVRCP volume
    static constexpr int kInitialSemitone = 7; // ソ
    static constexpr uint32_t kFadeOutSamples =
        static_cast<uint32_t>(0.05f * kSampleRateHz); // 50 ms

    Synth() {
        _wantedOn.store(false, std::memory_order_relaxed);
        _requestedSemitone.store(kInitialSemitone, std::memory_order_relaxed);
        _state = State::Off;
        _appliedSemitone = kInitialSemitone;
        _phase = 0.0f;
        _fadeRemaining = 0;
        updatePhaseIncrement(kInitialSemitone);
    }

    // 0 = low ド, 12 = octave ド. Mid-note changes keep phase continuous.
    void setSemitone(int semitone) {
        _requestedSemitone.store(semitone, std::memory_order_relaxed);
    }

    void noteOn() { _wantedOn.store(true, std::memory_order_relaxed); }

    // Request Tail (50 ms fade-out). After fade completes, voice becomes Off.
    void noteOff() { _wantedOn.store(false, std::memory_order_relaxed); }

    // Fill interleaved stereo with mono sine (L=R). Does not mix; caller adds onto A2DP.
    // Off: write silence. On: full level. Tail: linear fade-out then Off.
    void gen(int16_t* interleavedStereo, uint32_t frameCount) {
        if (interleavedStereo == nullptr || frameCount == 0) return;

        syncVoiceRequest();

        if (_state == State::Off) {
            std::memset(interleavedStereo, 0, static_cast<size_t>(frameCount) * 2u * sizeof(int16_t));
            return;
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
            float gain = 1.0f;
            if (_state == State::Tail) {
                if (_fadeRemaining == 0) {
                    enterOff();
                    std::memset(&interleavedStereo[i * 2], 0,
                                static_cast<size_t>(frameCount - i) * 2u * sizeof(int16_t));
                    break;
                }
                gain = static_cast<float>(_fadeRemaining) / static_cast<float>(kFadeOutSamples);
                --_fadeRemaining;
            }

            const int16_t sample = static_cast<int16_t>(sinf(_phase) * amp * gain);
            interleavedStereo[i * 2] = sample;
            interleavedStereo[i * 2 + 1] = sample;
            _phase += _phaseInc;

            if (_state == State::Tail && _fadeRemaining == 0) {
                enterOff();
                if (i + 1 < frameCount) {
                    std::memset(&interleavedStereo[(i + 1) * 2], 0,
                                static_cast<size_t>(frameCount - i - 1) * 2u * sizeof(int16_t));
                }
                break;
            }
        }
        while (_phase >= kTwoPi) {
            _phase -= kTwoPi;
        }
    }

  private:
    enum class State : uint8_t { Off, On, Tail };

    void updatePhaseIncrement(int semitone) {
        const float freq = kBaseFrequencyHz * powf(2.0f, static_cast<float>(semitone) / 12.0f);
        _phaseInc = static_cast<float>(2.0 * M_PI) * freq / kSampleRateHz;
    }

    void enterOff() {
        _state = State::Off;
        _fadeRemaining = 0;
        _appliedSemitone = kInitialSemitone;
        _requestedSemitone.store(kInitialSemitone, std::memory_order_relaxed);
        _phase = 0.0f;
        updatePhaseIncrement(kInitialSemitone);
    }

    void syncVoiceRequest() {
        const bool wantOn = _wantedOn.load(std::memory_order_relaxed);
        if (wantOn) {
            if (_state != State::On) {
                if (_state == State::Off) {
                    _phase = 0.0f; // start at amplitude 0
                }
                _state = State::On;
                _fadeRemaining = 0;
            }
        } else if (_state == State::On) {
            _state = State::Tail;
            _fadeRemaining = kFadeOutSamples;
        }
    }

    std::atomic<bool> _wantedOn;
    std::atomic<int> _requestedSemitone;
    State _state;
    int _appliedSemitone;
    float _phase;
    float _phaseInc;
    uint32_t _fadeRemaining;
};
