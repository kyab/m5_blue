#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

constexpr size_t RING_BUFFER_SAMPLE_NUM = 5000 + 5000;
constexpr size_t RING_BUFFER_SIZE = RING_BUFFER_SAMPLE_NUM * 2; // Stereo (L+R)

class RingBufferInterleaved {
  public:
    RingBufferInterleaved() {
        _buffer = (int16_t *)malloc(RING_BUFFER_SIZE * sizeof(int16_t));
        if (_buffer == nullptr) {
            printf("RingBuffer: alloc failed (%u samples)\n", (unsigned)RING_BUFFER_SAMPLE_NUM);
        } else {
            _buffer_size = RING_BUFFER_SAMPLE_NUM;
            printf("RingBuffer allocated: %u samples\n", (unsigned)_buffer_size);
        }
    }

    ~RingBufferInterleaved() {
        if (_buffer != nullptr) {
            free(_buffer);
        }
    }

    void storeSamples(const int16_t *buffer, size_t sample_num) {
        if (_buffer == nullptr) return;
        for (size_t i = 0; i < sample_num; i++) {
            _buffer[_write_pos * 2] = buffer[i * 2];
            _buffer[_write_pos * 2 + 1] = buffer[i * 2 + 1];
            _write_pos++;
            if (_write_pos >= _buffer_size) {
                _write_pos = 0;
            }
        }
    }

    void readSamplesTo(int16_t *buffer, size_t sample_num) {
        if (_buffer == nullptr) return;
        for (size_t i = 0; i < sample_num; i++) {
            buffer[i * 2] = _buffer[_read_pos * 2];
            buffer[i * 2 + 1] = _buffer[_read_pos * 2 + 1];
            _read_pos++;
            if (_read_pos >= _buffer_size) {
                _read_pos = 0;
            }
        }
    }

    void syncPositon() { _read_pos = _write_pos; }

    void advanceReadPosition(int32_t sample_num) {
        int32_t new_read_pos = static_cast<int32_t>(_read_pos) + sample_num;
        if (new_read_pos < 0) {
            new_read_pos += _buffer_size;
        } else if (new_read_pos >= static_cast<int32_t>(_buffer_size)) {
            new_read_pos -= _buffer_size;
        }
        _read_pos = static_cast<size_t>(new_read_pos);
    }

    size_t getBufferSize() const { return _buffer_size; }

    size_t getWritePosition() const { return _write_pos; }

    // Read one stereo frame at frame_index modulo capacity (for Freezer-style grain playback).
    void readFrameModulo(size_t frame_index, int16_t *out_l, int16_t *out_r) const {
        if (_buffer == nullptr || _buffer_size == 0) {
            *out_l = 0;
            *out_r = 0;
            return;
        }
        size_t idx = frame_index % _buffer_size;
        *out_l = _buffer[idx * 2];
        *out_r = _buffer[idx * 2 + 1];
    }

  private:
    int16_t *_buffer = nullptr;
    size_t _buffer_size = 0;
    size_t _write_pos = 0;
    size_t _read_pos = 0;
};
