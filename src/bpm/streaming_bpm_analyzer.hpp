// Facade: PCM ring -> STFT -> Essentia-like ODF trio (complex, rms, melflux) -> per-branch
// Davies-style tempo -> median fusion + display smoothing. Independent of application main flow.

#pragma once

#include "davies_tempo_estimator.hpp"
#include "stft_odf_pipeline.hpp"
#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

namespace bpm {

class StreamingBpmAnalyzer {
public:
    StreamingBpmAnalyzer();
    ~StreamingBpmAnalyzer();

    StreamingBpmAnalyzer(const StreamingBpmAnalyzer&) = delete;
    StreamingBpmAnalyzer& operator=(const StreamingBpmAnalyzer&) = delete;

    void reset();

    // Bluetooth / audio path may call; non-blocking, drops oldest PCM on overflow.
    void enqueueStereoInterleaved(const int16_t* pcm, size_t total_int16_values);

    // Worker task: drain PCM, update STFT/ODF/tempo estimates.
    void service();

    float bpm() const { return display_bpm_; }
    float beatDurationSec() const;

private:
    void fuseMedianEwma();

    static constexpr size_t kFifoBytes = 200 * 1024;
    uint8_t* fifo_buf_ = nullptr;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t used_ = 0;
    portMUX_TYPE fifo_mux_ = portMUX_INITIALIZER_UNLOCKED;

    StftOdfPipeline stft_;
    EssentiaStyleOdfBranch branch_complex_;
    EssentiaStyleOdfBranch branch_rms_;
    EssentiaStyleOdfBranch branch_mel_;

    float display_bpm_ = 0.0f;
};

void start_bpm_worker_task(StreamingBpmAnalyzer* analyzer);

} // namespace bpm
