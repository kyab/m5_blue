#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <esp_heap_caps.h>
#include <esp_log.h>

// 2 seconds of stereo audio at 44.1kHz
constexpr size_t RING_BUFFER_SAMPLE_NUM = 44100 * 2;
constexpr size_t RING_BUFFER_SIZE = RING_BUFFER_SAMPLE_NUM * 2; // Stereo (L+R)

class RingBufferInterleaved {
  public:
    RingBufferInterleaved() {
        // Allocate buffer in PSRAM (external memory)
        _buffer = (int16_t *)heap_caps_malloc(RING_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
        if (_buffer == nullptr) {
            ESP_LOGE("RingBuffer", "PSRAM alloc failed");
        } else {
            _buffer_size = RING_BUFFER_SAMPLE_NUM;
            ESP_LOGI("RingBuffer", "RingBuffer allocated in PSRAM: %d samples", _buffer_size);
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