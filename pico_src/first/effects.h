#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Stereo interleaved float PCM (nominal full scale ±1). frame_count is frames (L+R pairs).
void apply_effects_before_i2s(float* data, uint32_t frame_count);

#ifdef __cplusplus
}
#endif
