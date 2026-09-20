#pragma once

#include "RingBuffer.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Stereo grain freezer based directly on Going-Zero's Freezer and MiniFader.
// Control setters only publish targets; all DSP state is owned by the audio thread.
class Freezer {
  public:
    static constexpr uint32_t kMinGrainSamples = 1000;
    static constexpr uint32_t kDefaultGrainSamples = 3000;
    static constexpr uint32_t kMaxGrainSamples = 5000;
    static constexpr uint32_t kFadeSamples = 50;

    explicit Freezer(RingBufferInterleaved* ring)
        : _ring(ring),
          _requestedActive(false),
          _requestedGrainSamples(kDefaultGrainSamples),
          _active(false),
          _targetActive(false),
          _isFadingOut(false),
          _isFadingIn(false),
          _fadeOutCounter(0),
          _fadeInCounter(0),
          _grainSamples(kDefaultGrainSamples),
          _targetGrainSamples(kDefaultGrainSamples),
          _pendingGrainChange(false),
          _grainStartFrame(0),
          _grainSampleIndex(0),
          _grainFadeOutCounter(0),
          _grainFadeInCounter(kFadeSamples),
          _freezeStoreLimited(false),
          _freezeStoreRemaining(0) {}

    void setRingBuffer(RingBufferInterleaved* ring) { _ring = ring; }

    void setActive(bool active) { _requestedActive.store(active, std::memory_order_relaxed); }

    void setGrainSize(uint32_t samples) {
        if (samples < kMinGrainSamples) {
            samples = kMinGrainSamples;
        } else if (samples > kMaxGrainSamples) {
            samples = kMaxGrainSamples;
        }
        _requestedGrainSamples.store(samples, std::memory_order_relaxed);
    }

    void process(int16_t* interleavedStereo, uint32_t frameCount) {
        if (_ring == nullptr || interleavedStereo == nullptr || frameCount == 0) return;

        const size_t bufferFrames = _ring->getBufferSize();
        if (bufferFrames == 0) return;

        // While frozen, store at most kMaxGrainSamples more frames so the loop window is not overwritten.
        if (!_freezeStoreLimited) {
            _ring->storeSamples(interleavedStereo, frameCount);
        } else if (_freezeStoreRemaining > 0) {
            const uint32_t n = (frameCount < _freezeStoreRemaining) ? frameCount : _freezeStoreRemaining;
            _ring->storeSamples(interleavedStereo, n);
            _freezeStoreRemaining -= n;
        }

        for (uint32_t i = 0; i < frameCount; ++i) {
            syncGrainTarget();

            int16_t* left = &interleavedStereo[i * 2];
            int16_t* right = &interleavedStereo[i * 2 + 1];
            int16_t outputLeft = *left;
            int16_t outputRight = *right;

            const bool requestedActive = _requestedActive.load(std::memory_order_relaxed);
            if (requestedActive != _active && !_isFadingOut) {
                _targetActive = requestedActive;
                _isFadingOut = true;
                _fadeOutCounter = kFadeSamples;
            }

            if (_isFadingOut) {
                processCurrentState(outputLeft, outputRight, bufferFrames);
                applyFadeOut(outputLeft, outputRight, _fadeOutCounter);
                if (_fadeOutCounter == 0) {
                    changeStateAfterFadeOut(bufferFrames);
                }
            } else {
                processCurrentState(outputLeft, outputRight, bufferFrames);
                if (_isFadingIn) {
                    applyFadeIn(outputLeft, outputRight, _fadeInCounter);
                    if (_fadeInCounter >= kFadeSamples) {
                        _isFadingIn = false;
                    }
                }
            }

            *left = outputLeft;
            *right = outputRight;
        }
    }

  private:
    static int16_t scaleSample(int16_t sample, float gain) {
        int32_t scaled = static_cast<int32_t>(lroundf(static_cast<float>(sample) * gain));
        if (scaled > 32767) {
            scaled = 32767;
        } else if (scaled < -32768) {
            scaled = -32768;
        }
        return static_cast<int16_t>(scaled);
    }

    static void applyFadeOut(int16_t& left, int16_t& right, uint32_t& counter) {
        if (counter == 0) return;
        const float gain = static_cast<float>(counter) / static_cast<float>(kFadeSamples);
        left = scaleSample(left, gain);
        right = scaleSample(right, gain);
        --counter;
    }

    static void applyFadeIn(int16_t& left, int16_t& right, uint32_t& counter) {
        if (counter >= kFadeSamples) return;
        const float gain = static_cast<float>(counter) / static_cast<float>(kFadeSamples);
        left = scaleSample(left, gain);
        right = scaleSample(right, gain);
        ++counter;
    }

    void syncGrainTarget() {
        const uint32_t requested = _requestedGrainSamples.load(std::memory_order_relaxed);
        if (requested == _targetGrainSamples) return;

        _targetGrainSamples = requested;
        if (_active) {
            _pendingGrainChange = true;
        } else {
            _grainSamples = requested;
            _pendingGrainChange = false;
        }
    }

    void processCurrentState(int16_t& left, int16_t& right, size_t bufferFrames) {
        if (!_active) return;
        processGrainSample(left, right, bufferFrames);
    }

    void processGrainSample(int16_t& left, int16_t& right, size_t bufferFrames) {
        const size_t readFrame = (_grainStartFrame + _grainSampleIndex) % bufferFrames;
        _ring->readFrameModulo(readFrame, &left, &right);

        const uint32_t remaining = _grainSamples - _grainSampleIndex;
        if (remaining == kFadeSamples) {
            _grainFadeOutCounter = kFadeSamples;
        }
        if (remaining <= kFadeSamples) {
            applyFadeOut(left, right, _grainFadeOutCounter);
        }

        ++_grainSampleIndex;
        if (_grainSampleIndex >= _grainSamples) {
            if (_pendingGrainChange) {
                _grainSamples = _targetGrainSamples;
                _pendingGrainChange = false;
            }
            _grainSampleIndex = 0;
            _grainFadeInCounter = 0;
        }

        if (_grainSampleIndex < kFadeSamples) {
            applyFadeIn(left, right, _grainFadeInCounter);
        }
    }

    void changeStateAfterFadeOut(size_t bufferFrames) {
        _active = _targetActive;
        if (_active) {
            if (_pendingGrainChange) {
                _grainSamples = _targetGrainSamples;
                _pendingGrainChange = false;
            }
            if (_grainSamples > bufferFrames) {
                _grainSamples = static_cast<uint32_t>(bufferFrames);
            }
            const size_t writeFrame = _ring->getWritePosition();
            _grainStartFrame = (writeFrame + bufferFrames - _grainSamples) % bufferFrames;
            _grainSampleIndex = 0;
            _grainFadeInCounter = 0;
            _grainFadeOutCounter = 0;
            _freezeStoreLimited = true;
            _freezeStoreRemaining = kMaxGrainSamples;
        } else {
            _grainSamples = _targetGrainSamples;
            _pendingGrainChange = false;
            _freezeStoreLimited = false;
            _freezeStoreRemaining = 0;
        }

        _isFadingOut = false;
        _isFadingIn = true;
        _fadeInCounter = 0;
    }

    RingBufferInterleaved* _ring;
    std::atomic<bool> _requestedActive;
    std::atomic<uint32_t> _requestedGrainSamples;

    bool _active;
    bool _targetActive;
    bool _isFadingOut;
    bool _isFadingIn;
    uint32_t _fadeOutCounter;
    uint32_t _fadeInCounter;

    uint32_t _grainSamples;
    uint32_t _targetGrainSamples;
    bool _pendingGrainChange;
    size_t _grainStartFrame;
    uint32_t _grainSampleIndex;
    uint32_t _grainFadeOutCounter;
    uint32_t _grainFadeInCounter;

    bool _freezeStoreLimited;
    uint32_t _freezeStoreRemaining;
};
