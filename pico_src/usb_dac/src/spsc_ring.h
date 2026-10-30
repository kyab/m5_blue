#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

// Single-producer (core0) / single-consumer (core1) lock-free ring of stereo int16 frames.
template <size_t CapacityFrames>
class SpscStereoRing {
  public:
    static constexpr size_t kCapacity = CapacityFrames;

    void reset() {
        write_.store(0, std::memory_order_relaxed);
        read_.store(0, std::memory_order_relaxed);
    }

    size_t size() const {
        const size_t w = write_.load(std::memory_order_acquire);
        const size_t r = read_.load(std::memory_order_relaxed);
        return w - r;
    }

    size_t free_space() const { return kCapacity - size(); }

    // Push up to `frames` stereo frames. Returns frames written.
    size_t push(const int16_t *interleaved, size_t frames) {
        size_t w = write_.load(std::memory_order_relaxed);
        size_t r = read_.load(std::memory_order_acquire);
        size_t avail = kCapacity - (w - r);
        if (frames > avail) frames = avail;
        for (size_t i = 0; i < frames; ++i) {
            const size_t slot = (w + i) % kCapacity;
            buf_[slot][0] = interleaved[i * 2];
            buf_[slot][1] = interleaved[i * 2 + 1];
        }
        std::atomic_thread_fence(std::memory_order_release);
        write_.store(w + frames, std::memory_order_release);
        return frames;
    }

    // Pop up to `frames`. Missing frames are zero-filled. Returns frames taken from the ring.
    size_t pop(int16_t *interleaved, size_t frames) {
        size_t r = read_.load(std::memory_order_relaxed);
        size_t w = write_.load(std::memory_order_acquire);
        size_t avail = w - r;
        size_t got = frames < avail ? frames : avail;
        for (size_t i = 0; i < got; ++i) {
            const size_t slot = (r + i) % kCapacity;
            interleaved[i * 2] = buf_[slot][0];
            interleaved[i * 2 + 1] = buf_[slot][1];
        }
        for (size_t i = got; i < frames; ++i) {
            interleaved[i * 2] = 0;
            interleaved[i * 2 + 1] = 0;
        }
        std::atomic_thread_fence(std::memory_order_release);
        read_.store(r + got, std::memory_order_release);
        return got;
    }

  private:
    int16_t buf_[CapacityFrames][2]{};
    std::atomic<size_t> write_{0};
    std::atomic<size_t> read_{0};
};
