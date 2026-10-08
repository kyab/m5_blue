#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

// A2DP sample recorder / one-shot player.
// States: Off / Head (fade-in) / On / Tail (fade-out) / Rec.
// feedSamples() and gen() are always called from the audio task; each decides
// accumulate / discard / play / silence from the current state.
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
        _state = State::Off;
        _fadeGain = 0.0f;
        _lengthFrames = 0;
        _playPos = 0;
    }

    void startRecord() { _wantedRec = true; }
    void stopRecord() { _wantedRec = false; }

    void noteOn() { _wantedOn = true; }
    void noteOff() { _wantedOn = false; }

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
            if (_playPos >= _lengthFrames) {
                releaseVoice();
                break;
            }

            // Auto-enter Tail near end
            if (_state == State::On && (_lengthFrames - _playPos) <= kFadeOutSamples) {
                _fadeGain = 1.0f;
                _wantedOn = false;
                _state = State::Tail;
            }

            if (_state == State::Head) {
                _fadeGain += kFadeInStep;
                if (_fadeGain > 1.0f) {
                    _fadeGain = 1.0f;
                    _state = State::On;
                    if ((_lengthFrames - _playPos) <= kFadeOutSamples) {
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

            const float l = static_cast<float>(_buf[_playPos * 2]) / 32768.0f * _fadeGain;
            const float r = static_cast<float>(_buf[_playPos * 2 + 1]) / 32768.0f * _fadeGain;
            interleavedStereo[i * 2] = l;
            interleavedStereo[i * 2 + 1] = r;
            ++_playPos;

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

    void syncRequests() {
        const bool wantRec = _wantedRec.load(std::memory_order_relaxed);

        if (wantRec) {
            if (_state == State::Off) {
                _lengthFrames = 0;
                _playPos = 0;
                _fadeGain = 0.0f;
                _wantedOn = false;
                _state = State::Rec;
            } else if (_state == State::Head || _state == State::On || _state == State::Tail) {
                // Reject record while playing.
                _wantedRec = false;
            }
        } else if (_state == State::Rec) {
            _playPos = 0;
            _fadeGain = 0.0f;
            // Reject play while recording.
            _wantedOn = false;
            _state = State::Off;
        }

        if (_state == State::Rec) return;

        const bool wantOn = _wantedOn.load(std::memory_order_relaxed);
        if (wantOn) {
            if (_state == State::Off) {
                _playPos = 0;
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
        _playPos = 0;
        _wantedOn = false;
    }

    std::atomic<bool> _wantedOn;
    std::atomic<bool> _wantedRec;
    State _state;
    float _fadeGain;
    uint32_t _lengthFrames;
    uint32_t _playPos;
    int16_t _buf[kMaxFrames * 2];
};
