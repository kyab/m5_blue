#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

// A2DP sample recorder / one-shot player.
// States: Off / Head (fade-in) / On / Tail (fade-out) / Rec.
// feedSamples() and gen() are always called from the audio task; each decides
// accumulate / discard / play / silence from the current state.
// Relative pitch is stored until Off/Tail -> Head, then held for that playback.
class Sampler {
  public:
    static constexpr float kSampleRateHz = 44100.0f;
    static constexpr uint32_t kMaxFrames = 88200; // 2 s at kSampleRateHz
    static constexpr uint32_t kFadeInSamples =
        static_cast<uint32_t>(0.05f * kSampleRateHz);
    static constexpr uint32_t kFadeOutSamples =
        static_cast<uint32_t>(0.10f * kSampleRateHz);
    static constexpr float kFadeInStep = 1.0f / static_cast<float>(kFadeInSamples);
    static constexpr float kFadeOutStep = 1.0f / static_cast<float>(kFadeOutSamples);

    Sampler() {
        _wantedOn = false;
        _wantedRec = false;
        _pendingSemitone = 0;
        _state = State::Off;
        _fadeGain = 0.0f;
        _playSpeed = 1.0;
        _lengthFrames = 0;
        _playPos = 0.0;
    }

    void startRecord() { _wantedRec = true; }
    void stopRecord() { _wantedRec = false; }

    void noteOn() { _wantedOn = true; }
    void noteOff() { _wantedOn = false; }

    // Applied on the next Off/Tail -> Head. Ignored until then.
    void setRelativeSemitone(int semitone) { _pendingSemitone = semitone; }

    void feedSamples(const float* interleavedStereo, uint32_t frameCount) {
        if (interleavedStereo == nullptr || frameCount == 0) return;

        syncRequests();

        if (_state != State::Rec) return;

        for (uint32_t i = 0; i < frameCount; ++i) {
            if (_lengthFrames >= kMaxFrames) break;
            const uint32_t src = i * 2;
            _buf[_lengthFrames * 2] = floatToInt16(interleavedStereo[src]);
            _buf[_lengthFrames * 2 + 1] = floatToInt16(interleavedStereo[src + 1]);
            ++_lengthFrames;
        }
    }

    // Fill interleaved stereo float. Silence when Off/Rec or past recorded length.
    void gen(float* interleavedStereo, uint32_t frameCount) {
        if (interleavedStereo == nullptr || frameCount == 0) return;

        syncRequests();

        if (_state == State::Off || _state == State::Rec) {
            std::memset(interleavedStereo, 0, frameCount * 2 * sizeof(float));
            return;
        }

        uint32_t i = 0;
        for (; i < frameCount; ++i) {
            if (_playPos >= static_cast<double>(_lengthFrames)) {
                releaseVoice();
                break;
            }

            // Auto-enter Tail when remaining output frames fit in the fade-out.
            if (_state == State::On && remainingSourceFrames() <= fadeOutSourceFrames()) {
                _fadeGain = 1.0f;
                _wantedOn = false;
                _state = State::Tail;
            }

            if (_state == State::Head) {
                _fadeGain += kFadeInStep;
                if (_fadeGain > 1.0f) {
                    _fadeGain = 1.0f;
                    _state = State::On;
                    if (remainingSourceFrames() <= fadeOutSourceFrames()) {
                        _wantedOn = false;
                        _state = State::Tail;
                    }
                }
            } else if (_state == State::Tail) {
                _fadeGain -= kFadeOutStep;
                if (_fadeGain <= 0.0f) {
                    _fadeGain = 0.0f;
                }
            }

            const int32_t base = static_cast<int32_t>(_playPos);
            const double mu = _playPos - static_cast<double>(base);
            const float l = cubicAt(base, 0, mu) * _fadeGain;
            const float r = cubicAt(base, 1, mu) * _fadeGain;
            interleavedStereo[i * 2] = l;
            interleavedStereo[i * 2 + 1] = r;
            _playPos += _playSpeed;

            if (_state == State::Tail && _fadeGain <= 0.0f) {
                releaseVoice();
                ++i;
                break;
            }
        }

        if (i < frameCount) {
            std::memset(&interleavedStereo[i * 2], 0, (frameCount - i) * 2 * sizeof(float));
        }
    }

  private:
    enum class State : uint8_t { Off,
                                 Head,
                                 On,
                                 Tail,
                                 Rec };

    static int16_t floatToInt16(float s) {
        float v = s * 32768.0f;
        if (v > 32767.0f)
            v = 32767.0f;
        else if (v < -32768.0f)
            v = -32768.0f;
        return static_cast<int16_t>(v);
    }

    // Catmull-Rom cubic from Scratch-Now TurnTable::cubicInterpolate. mu == 0 returns y1.
    static float cubicInterpolate(float y0, float y1, float y2, float y3, double mu) {
        const double mu2 = mu * mu;
        const double a0 = static_cast<double>(y3) - static_cast<double>(y2) - static_cast<double>(y0) + static_cast<double>(y1);
        const double a1 = static_cast<double>(y0) - static_cast<double>(y1) - a0;
        const double a2 = static_cast<double>(y2) - static_cast<double>(y0);
        const double a3 = static_cast<double>(y1);
        return static_cast<float>((mu * mu2 * a0) + (mu2 * a1) + (mu * a2) + a3);
    }

    float sampleAt(int32_t index, int channel) const {
        if (index < 0 || static_cast<uint32_t>(index) >= _lengthFrames) return 0.0f;
        return static_cast<float>(_buf[static_cast<uint32_t>(index) * 2 + channel]) / 32768.0f;
    }

    float cubicAt(int32_t base, int channel, double mu) const {
        return cubicInterpolate(sampleAt(base - 1, channel), sampleAt(base, channel), sampleAt(base + 1, channel), sampleAt(base + 2, channel), mu);
    }

    double remainingSourceFrames() const {
        return static_cast<double>(_lengthFrames) - _playPos;
    }

    double fadeOutSourceFrames() const {
        return static_cast<double>(kFadeOutSamples) * _playSpeed;
    }

    void triggerPlayback(bool resetFade) {
        const int semitone = _pendingSemitone.load(std::memory_order_relaxed);
        _playSpeed = std::pow(2.0, static_cast<double>(semitone) / 12.0);
        _playPos = 0.0;
        if (resetFade) _fadeGain = 0.0f;
        _state = State::Head;
    }

    void syncRequests() {
        const bool wantRec = _wantedRec.load(std::memory_order_relaxed);

        if (wantRec) {
            if (_state == State::Off) {
                _lengthFrames = 0;
                _playPos = 0.0;
                _fadeGain = 0.0f;
                _wantedOn = false;
                _state = State::Rec;
            } else if (_state == State::Head || _state == State::On || _state == State::Tail) {
                // Reject record while playing.
                _wantedRec = false;
            }
        } else if (_state == State::Rec) {
            _playPos = 0.0;
            _fadeGain = 0.0f;
            // Reject play while recording.
            _wantedOn = false;
            _state = State::Off;
        }

        if (_state == State::Rec) return;

        const bool wantOn = _wantedOn.load(std::memory_order_relaxed);
        if (wantOn) {
            if (_state == State::Off) {
                triggerPlayback(true);
            } else if (_state == State::Tail) {
                // Retrigger from the start at the latest pitch, keeping the fade level.
                triggerPlayback(false);
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
        _playPos = 0.0;
        _wantedOn = false;
    }

    std::atomic<bool> _wantedOn;
    std::atomic<bool> _wantedRec;
    std::atomic<int> _pendingSemitone;
    State _state;
    float _fadeGain;
    double _playSpeed;
    uint32_t _lengthFrames;
    double _playPos;
    int16_t _buf[kMaxFrames * 2];
};
