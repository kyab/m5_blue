#include <cstddef>
#include <cstdint>
#include <esp_heap_caps.h>

// 10 seconds of stereo audio at 44.1kHz
constexpr size_t RING_BUFFER_SAMPLE_NUM = 44100 * 10;
constexpr size_t RING_BUFFER_SIZE = RING_BUFFER_SAMPLE_NUM * 2;  // Stereo (L+R)

class RingBufferInterleaved {
public:
  RingBufferInterleaved() {
    // Allocate buffer in PSRAM (external memory) to avoid internal RAM exhaustion
    _buffer = (int16_t*)heap_caps_malloc(RING_BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (_buffer == nullptr) {
      // Fallback to internal RAM with smaller size if PSRAM not available
      Serial.println("PSRAM alloc failed, using smaller internal buffer");
      _buffer_size = 44100 * 2;  // 2 seconds fallback
      _buffer = (int16_t*)malloc(_buffer_size * 2 * sizeof(int16_t));
    } else {
      _buffer_size = RING_BUFFER_SAMPLE_NUM;
      Serial.printf("RingBuffer allocated in PSRAM: %d samples\n", _buffer_size);
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

private:
  int16_t *_buffer = nullptr;
  size_t _buffer_size = 0;
  size_t _write_pos = 0;
  size_t _read_pos = 0;
};