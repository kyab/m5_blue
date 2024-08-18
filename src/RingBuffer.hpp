#include <cstddef>
#include <cstdint>

constexpr size_t RING_BUFFER_SAMPLE_NUM = 44100 * 10;
constexpr size_t RING_BUFFER_SIZE = RING_BUFFER_SAMPLE_NUM * 2;

class RingBufferInterleaved {
public:
  RingBufferInterleaved() = default;
  ~RingBufferInterleaved() = default;

  void storeSamples(const int16_t *buffer, size_t sample_num) {
    for (size_t i = 0; i < sample_num; i++) {
      _buffer[_write_pos * 2] = buffer[i * 2];
      _buffer[_write_pos * 2 + 1] = buffer[i * 2 + 1];
      _write_pos++;
      if (_write_pos >= RING_BUFFER_SAMPLE_NUM) {
        _write_pos = 0;
      }
    }
  }

  void readSamplesTo(int16_t *buffer, size_t sample_num) {
    for (size_t i = 0; i < sample_num; i++) {
      buffer[i * 2] = _buffer[_read_pos * 2];
      buffer[i * 2 + 1] = _buffer[_read_pos * 2 + 1];
      _read_pos++;
      if (_read_pos >= RING_BUFFER_SAMPLE_NUM) {
        _read_pos = 0;
      }
    }
  }

  void syncPositon() { _read_pos = _write_pos; }

  void advanceReadPosition(int32_t sample_num) {
    int32_t new_read_pos = static_cast<int32_t>(_read_pos) + sample_num;
    if (new_read_pos < 0) {
      new_read_pos += RING_BUFFER_SAMPLE_NUM;
    } else if (new_read_pos >= RING_BUFFER_SAMPLE_NUM) {
      new_read_pos -= RING_BUFFER_SAMPLE_NUM;
    }

    _read_pos = static_cast<size_t>(new_read_pos);
  }

private:
  int16_t _buffer[RING_BUFFER_SIZE];
  size_t _write_pos = 0;
  size_t _read_pos = 0;
};