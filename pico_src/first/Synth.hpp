#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Monophonic sine synth.
// Voice state: Off / Head (50 ms fade-in after noteOn) / On / Tail (50 ms fade-out after noteOff).
class Synth {
  public:
    // Low C note frequency. Change to 130.81f (C3) or 261.63f (C4) or 523.25f (C5).. as needed.
    static constexpr float kBaseFrequencyHz = 130.81f; // C3

    static constexpr float kSampleRateHz = 44100.0f;
    static constexpr float kLevel = 0.6f;
    static constexpr int kInitialSemitone = 7; // G note
    static constexpr uint32_t kFadeInSamples =
        static_cast<uint32_t>(0.05f * kSampleRateHz); // Attack time
    static constexpr uint32_t kFadeOutSamples =
        static_cast<uint32_t>(0.70f * kSampleRateHz); // Release time
    static constexpr float kFadeInStep = 1.0f / static_cast<float>(kFadeInSamples);
    static constexpr float kFadeOutStep = 1.0f / static_cast<float>(kFadeOutSamples);

    Synth() {
        _wantedOn = false;
        _semitone = kInitialSemitone;
        _state = State::Off;
        _phase = 0.0f;
        _fadeGain = 0.0f;
    }

    // 0 = low C note, 12 = octave C note. Mid-note changes keep phase continuous.
    void setSemitone(int semitone) { _semitone = semitone; }

    void noteOn() { _wantedOn = true; }

    void noteOff() { _wantedOn = false; }

    // Generate and fill interleaved stereo float buffer (nominal peak ±kLevel).
    void gen(float* interleavedStereo, uint32_t frameCount) {
        if (interleavedStereo == nullptr || frameCount == 0) return;

        syncVoiceRequest();

        if (_state == State::Off) {
            std::memset(interleavedStereo, 0, frameCount * 2 * sizeof(float));
            return;
        }

        constexpr float kTwoPi = static_cast<float>(2.0 * M_PI);
        const float freq = kBaseFrequencyHz * powf(2.0f, static_cast<float>(_semitone) / 12.0f);
        const float phaseInc = kTwoPi * freq / kSampleRateHz;

        uint32_t i = 0;
        for (; i < frameCount; ++i) {
            if (_state == State::Head) {
                _fadeGain += kFadeInStep;
                if (_fadeGain > 1.0f) {
                    _fadeGain = 1.0f;
                    _state = State::On;
                }
            } else if (_state == State::Tail) {
                _fadeGain -= kFadeOutStep;
                if (_fadeGain <= 0.0f) {
                    _fadeGain = 0.0f;
                }
            }

            const float sample = sinf(_phase) * kLevel * _fadeGain;
            interleavedStereo[i * 2] = sample;
            interleavedStereo[i * 2 + 1] = sample;
            _phase += phaseInc;

            if (_state == State::Tail && _fadeGain <= 0.0f) {
                releaseVoice();
                ++i;
                break;
            }
        }

        if (i < frameCount) {
            std::memset(&interleavedStereo[i * 2], 0, (frameCount - i) * 2 * sizeof(float));
        }
        if (_state != State::Off) {
            while (_phase >= kTwoPi) {
                _phase -= kTwoPi;
            }
        }
    }

  private:
    enum class State : uint8_t { Off,
                                 Head,
                                 On,
                                 Tail };

    void syncVoiceRequest() {
        if (_wantedOn) {
            if (_state == State::Off) {
                _phase = 0.0f;
                _fadeGain = 0.0f;
                _state = State::Head;
            } else if (_state == State::Tail) {
                _state = State::Head;
            }
        } else if (_state == State::On) {
            _fadeGain = 1.0f;
            _state = State::Tail;
        } else if (_state == State::Head) {
            _state = State::Tail;
        }
    }

    void releaseVoice() {
        _state = State::Off;
        _fadeGain = 0.0f;
        _semitone = kInitialSemitone;
        _phase = 0.0f;
    }

    std::atomic<bool> _wantedOn;
    std::atomic<int> _semitone;
    State _state;
    float _phase;
    float _fadeGain;
};
