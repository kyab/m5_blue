// STFT 2048 / 1024 + Hann; Essentia OnsetDetection methods complex, rms, melflux (same spectrum chain).
// Large FFT / ring buffers are allocated in PSRAM when available (DRAM is tight with BT stack).

#pragma once

#include "bpm_config.hpp"
#include "mel_filterbank.hpp"
#include <arduinoFFT.h>
#include <cstdint>

namespace bpm {

struct OdfTriplet {
    float complex_odf = 0.0f;
    float rms_odf = 0.0f;
    float melflux_odf = 0.0f;
};

class StftOdfPipeline {
public:
    StftOdfPipeline();
    ~StftOdfPipeline();

    StftOdfPipeline(const StftOdfPipeline&) = delete;
    StftOdfPipeline& operator=(const StftOdfPipeline&) = delete;

    void reset();

    void pushMonoFloat(float x);
    bool takeOdfIfNew(OdfTriplet& out);

private:
    void processFrame(uint32_t end_idx_masked);
    void magnitudePhaseFromFft();

    float onsetComplex(const float* mag, const float* ph);
    float onsetRms(const float* mag);
    float onsetMelFlux(const float* mag);

    static float amp2dbEssentia(float a);

    bool allocBuffers();
    void freeBuffers();

    float* ring_ = nullptr;
    float* hann_ = nullptr;
    double* fft_real_ = nullptr;
    double* fft_imag_ = nullptr;
    arduinoFFT* fft_engine_ = nullptr;
    MelFilterbank mel_;

    float* mag_ = nullptr;
    float* phase_ = nullptr;
    float* phase_1_ = nullptr;
    float* phase_2_ = nullptr;
    float* spectrum_1_ = nullptr;
    float* mel_linear_ = nullptr;
    float* prev_mel_db_ = nullptr;

    double rms_old_ = 0.0;
    bool first_melflux_ = true;
    bool first_rms_ = true;

    uint32_t mono_total_ = 0;
    OdfTriplet last_odf_{};
    bool fresh_odf_ = false;
};

} // namespace bpm
