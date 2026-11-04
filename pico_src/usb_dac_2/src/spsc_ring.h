#pragma once

#include <atomic>
#include <cstdint>

// Lock-free single-producer / single-consumer ring. Capacity must be a power of two.
template <typename T, uint32_t Capacity>
class SpscRing {
    static_assert(Capacity != 0 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

  public:
    // Producer side.
    uint32_t write(const T* src, uint32_t count) {
        const uint32_t head = _head.load(std::memory_order_relaxed);
        const uint32_t tail = _tail.load(std::memory_order_acquire);
        const uint32_t space = Capacity - (head - tail);
        if (count > space) count = space;
        for (uint32_t i = 0; i < count; ++i) {
            _data[(head + i) & kMask] = src[i];
        }
        _head.store(head + count, std::memory_order_release);
        return count;
    }

    // Consumer side.
    uint32_t read(T* dst, uint32_t count) {
        const uint32_t tail = _tail.load(std::memory_order_relaxed);
        const uint32_t head = _head.load(std::memory_order_acquire);
        const uint32_t avail = head - tail;
        if (count > avail) count = avail;
        for (uint32_t i = 0; i < count; ++i) {
            dst[i] = _data[(tail + i) & kMask];
        }
        _tail.store(tail + count, std::memory_order_release);
        return count;
    }

    // Upper bound when called from the producer, lower bound when called from the consumer.
    uint32_t size() const {
        return _head.load(std::memory_order_acquire) - _tail.load(std::memory_order_acquire);
    }

  private:
    static constexpr uint32_t kMask = Capacity - 1;

    T _data[Capacity];
    std::atomic<uint32_t> _head{0};
    std::atomic<uint32_t> _tail{0};
};
