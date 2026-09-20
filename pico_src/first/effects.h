#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Stereo interleaved PCM. frame_count is frames (L+R pairs), not int16 samples.
void apply_effects_before_i2s(int16_t* data, uint32_t frame_count);

#ifdef __cplusplus
}
#endif
