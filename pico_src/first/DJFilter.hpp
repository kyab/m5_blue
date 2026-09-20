// DJFilter: real-time LPF/HPF crossover filter controlled by a single parameter v in [-1, +1].
// C++ port of Going-Zero (https://github.com/kyab/Going-Zero) DJFilter / LPF_IIR / HPF_IIR.
// Mac-side 30s ring buffers are unnecessary: a 2nd-order direct-form biquad with 2 input + 2 output
// state samples per channel is mathematically equivalent and fits on an MCU.
//
// =============================================================================
// Deliberate divergence from Going-Zero (reasons documented inline in this header).
//
// Symptom (M5Stack Core2 / rotary ADC + EMA + small deadzone): around v=+0.01..+0.20
// the filter output became unstable and lingered for seconds as sub-bass rumble
// or near-zero "near-mute" tail before recovering.
//
// Root cause (numerical analysis):
//   Going-Zero's HPF mapping at small positive v drives fc into single-digit Hz
//   with constant Q=5. The resulting biquad has poles at radius r ~ 1 - O(1e-5),
//   giving time constants of 0.5..2 seconds. Any state perturbation (coefficient
//   move from sub-block ramp, knob wobble, DC creep in the source) needs O(seconds)
//   to settle. Going-Zero hides this on Mac by (a) UI not nudging v in that band
//   and (b) reset()+FaderIn on the v==0 exact transition - the same mitigation
//   actually produces a 1..2s "wrong-state" transient itself, which the FaderIn
//   masks the leading edge of.
//
// Fix strategy (in this port):
//   1. Clamp HPF fc to [20 Hz, 20 kHz] and LPF fc to [20 Hz, 20 kHz]. Below 20 Hz
//      the HPF effect is inaudible to humans; clamping eliminates the marginal-
//      stability regime entirely. The mapping is otherwise preserved, so the
//      audible region (HPF fc >= ~20 Hz at v > 0.30, LPF fc <= 20 kHz at v < -0.017)
//      is bit-equivalent to Going-Zero.
//   2. Smooth Q taper: Q = 5 for fc >= kQTaperFcHigh (300 Hz, fully resonant DJ
//      character, matches Going-Zero), Q = 0.707 for fc <= kQTaperFcLow (30 Hz,
//      Butterworth, no resonance), log-linear interpolation between. Avoids the
//      Q=5 corner peak from appearing as a +14 dB sub-bass spike right at the
//      clamp boundary.
//   3. No periodic state reset. Resetting state on a near-bypass IIR with poles
//      near z=1 creates a long transient identical in spirit to the original bug.
//      With (1) the poles are well inside the unit circle so the IIR is self-
//      stabilising and does not need help.
//   4. Sub-block (32 samples ~= 0.73 ms) coefficient updates remain, so coefficient
//      moves are dense enough to avoid stepwise zipper noise during knob sweeps.
//   5. Polarity flip bridge (LPF -> bypass -> HPF) kept: smoothly hands off
//      between which filter is "active" on fast L<->R polarity slams.
// =============================================================================

#pragma once

#include <cmath>
#include <cstdint>

#ifndef DJ_FILTER_DIAG
#define DJ_FILTER_DIAG 0
#endif

#if DJ_FILTER_DIAG
#include "esp_log.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class BiquadIIR {
public:
    enum Type { LPF, HPF };

    BiquadIIR() { reset(); }

    void reset() {
        _lx1 = _lx2 = _ly1 = _ly2 = 0.0f;
        _rx1 = _rx2 = _ry1 = _ry2 = 0.0f;
    }

    // Bilinear-transform biquad matching Going-Zero's IIR_LPF / IIR_HPF, but Q is now
    // a parameter (Going-Zero hard-codes Q=5; we vary Q with fc to keep the filter well
    // away from the unit circle at low fc - see DJFilter design notes at the top of file).
    void setCutoff(Type type, float fc_hz, float fs_hz, float Q) {
        if (Q < 0.1f) Q = 0.1f;
        float fc = fc_hz / fs_hz;
        // Prevent tanf() blowup near Nyquist.
        if (fc > 0.499f) fc = 0.499f;
        if (fc < 1.0e-6f) fc = 1.0e-6f;
        float pre = tanf((float)M_PI * fc) / (2.0f * (float)M_PI);
        float pre2 = pre * pre;
        float a0 = 1.0f + 2.0f * (float)M_PI * pre / Q + 4.0f * (float)M_PI * (float)M_PI * pre2;
        float a1 = (8.0f * (float)M_PI * (float)M_PI * pre2 - 2.0f) / a0;
        float a2 = (1.0f - 2.0f * (float)M_PI * pre / Q + 4.0f * (float)M_PI * (float)M_PI * pre2) / a0;

        float b0, b1, b2;
        if (type == LPF) {
            b0 = 4.0f * (float)M_PI * (float)M_PI * pre2 / a0;
            b1 = 8.0f * (float)M_PI * (float)M_PI * pre2 / a0;
            b2 = 4.0f * (float)M_PI * (float)M_PI * pre2 / a0;
        } else {
            b0 = 1.0f / a0;
            b1 = -2.0f / a0;
            b2 = 1.0f / a0;
        }
        _b0 = b0;
        _b1 = b1;
        _b2 = b2;
        _a1 = a1;
        _a2 = a2;
    }

    // Direct-form-I biquad: y[n] = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2
    void processStereo(float* left, float* right, uint32_t n) {
        float b0 = _b0, b1 = _b1, b2 = _b2, a1 = _a1, a2 = _a2;
        float lx1 = _lx1, lx2 = _lx2, ly1 = _ly1, ly2 = _ly2;
        float rx1 = _rx1, rx2 = _rx2, ry1 = _ry1, ry2 = _ry2;
        for (uint32_t i = 0; i < n; i++) {
            float xl = left[i];
            float yl = b0 * xl + b1 * lx1 + b2 * lx2 - a1 * ly1 - a2 * ly2;
            lx2 = lx1;
            lx1 = xl;
            ly2 = ly1;
            ly1 = yl;
            left[i] = yl;

            float xr = right[i];
            float yr = b0 * xr + b1 * rx1 + b2 * rx2 - a1 * ry1 - a2 * ry2;
            rx2 = rx1;
            rx1 = xr;
            ry2 = ry1;
            ry1 = yr;
            right[i] = yr;
        }
        _lx1 = lx1;
        _lx2 = lx2;
        _ly1 = ly1;
        _ly2 = ly2;
        _rx1 = rx1;
        _rx2 = rx2;
        _ry1 = ry1;
        _ry2 = ry2;
    }

private:
    float _b0 = 0.0f, _b1 = 0.0f, _b2 = 0.0f, _a1 = 0.0f, _a2 = 0.0f;
    float _lx1, _lx2, _ly1, _ly2;
    float _rx1, _rx2, _ry1, _ry2;
};

class DJFilter {
public:
    DJFilter() : _vTarget(0.0f), _vSmooth(0.0f), _flipBridgeActive(false), _flipBridgeToPositive(false) {
        applyCoefficients(0.0f);
    }

    float getFilterValue() const { return _vTarget; }

    // v in [-1, +1]. v == 0 -> bypass; v < 0 -> LPF mode; v > 0 -> HPF mode.
    //
    // Fast LPF<->HPF flips can click because biquad coefficient families effectively
    // switch polarity. Instead of reset+mute, we bridge through bypass internally:
    //
    //   LPF target -> (ramp to 0) -> (ramp to HPF target), or vice versa.
    //
    // This keeps coefficient changes continuous and avoids hard IIR state discontinuities.
    void setFilterValue(float v) {
        int oldSign = (_vTarget < 0.0f) ? -1 : ((_vTarget > 0.0f) ? 1 : 0);
        int newSign = (v < 0.0f) ? -1 : ((v > 0.0f) ? 1 : 0);
        bool polarityFlip = (oldSign != 0) && (newSign != 0) && (oldSign != newSign);
        if (polarityFlip) {
            _flipBridgeActive = true;
            _flipBridgeToPositive = (newSign > 0);
        }
        _vTarget = v;
    }

    // Real-time stereo processing. Coefficients are interpolated per sub-block so fc
    // moves smoothly even when the knob is spun much faster than the audio block rate.
    void process(float* left, float* right, uint32_t n) {
        if (n == 0) return;

        // Snap to exact target when both target and smoothed values are micro-deviation
        // from zero: removes float dither that could otherwise flip the coefficient branch
        // sample-to-sample. This is purely a numerical hygiene step, not a state reset.
        if (!_flipBridgeActive && stableBypassHold()) {
            _vSmooth = _vTarget;
        }

        // Linear ramp of _vSmooth from its current value to _vTarget across this block.
        //   kSubBlock = 32 samples (~0.73 ms at 44.1 kHz). Coefficients are recomputed
        //   at every sub-block, so user-visible reaction time is one block (~5.8 ms at
        //   kI2SWriterFrames=256), and _vSmooth lands float-equal to _vTarget at the
        //   end of the block (no asymptotic drift).
        const uint32_t kSubBlock = 32;

        uint32_t offset = 0;
        uint32_t remainingSubs = (n + kSubBlock - 1) / kSubBlock;
        while (offset < n) {
            uint32_t sub = (n - offset > kSubBlock) ? kSubBlock : (n - offset);
            float effectiveTarget = _vTarget;
            if (_flipBridgeActive) {
                if (_flipBridgeToPositive) {
                    if (_vSmooth < 0.0f) {
                        effectiveTarget = 0.0f;
                    } else {
                        effectiveTarget = _vTarget;
                        if (_vTarget <= 0.0f) {
                            _flipBridgeActive = false;
                        }
                    }
                } else {
                    if (_vSmooth > 0.0f) {
                        effectiveTarget = 0.0f;
                    } else {
                        effectiveTarget = _vTarget;
                        if (_vTarget >= 0.0f) {
                            _flipBridgeActive = false;
                        }
                    }
                }
                if (fabsf(_vSmooth) <= kBridgeEpsilon && effectiveTarget == 0.0f) {
                    _vSmooth = 0.0f;
                    effectiveTarget = _vTarget;
                }
                int smoothSign = (_vSmooth < 0.0f) ? -1 : ((_vSmooth > 0.0f) ? 1 : 0);
                int targetSign = (_vTarget < 0.0f) ? -1 : ((_vTarget > 0.0f) ? 1 : 0);
                if (smoothSign == targetSign || targetSign == 0) {
                    _flipBridgeActive = false;
                    effectiveTarget = _vTarget;
                }
            }
            float vStep = (effectiveTarget - _vSmooth) / (float)remainingSubs;
            _vSmooth += vStep;
            applyCoefficients(mapVForCoeffs(_vSmooth));
            _hpf.processStereo(left + offset, right + offset, sub);
            _lpf.processStereo(left + offset, right + offset, sub);
            sanitizeFinite(left + offset, right + offset, sub);
            offset += sub;
            if (remainingSubs > 0) {
                remainingSubs--;
            }
        }
        if (!_flipBridgeActive) {
            _vSmooth = _vTarget;
        }

#if DJ_FILTER_DIAG
        diagPeakAfterProcess(left, right, n);
#endif
    }

    void reset() {
        _lpf.reset();
        _hpf.reset();
    }

private:
    bool stableBypassHold() const {
        return fabsf(_vTarget) <= kStableBypassEps && fabsf(_vSmooth) <= kStableBypassEps;
    }

    // Float noise around 0 must not alternate LPF-vs-HPF coefficient families sample-to-sample.
    static float mapVForCoeffs(float v) {
        if (fabsf(v) <= kCoeffBranchNoiseEps) {
            return 0.0f;
        }
        return v;
    }

    static void sanitizeFinite(float* left, float* right, uint32_t n) {
        for (uint32_t i = 0; i < n; i++) {
            if (!std::isfinite(left[i]) || !std::isfinite(right[i])) {
                left[i] = 0.0f;
                right[i] = 0.0f;
            }
        }
    }

    // Q taper: keep Going-Zero's Q=5 character in the audible-resonance band but
    // gently roll Q down toward Butterworth (0.707) below 30 Hz so the clamp boundary
    // (fc=20 Hz at near-bypass) does not present a +14 dB resonance peak.
    static float qForFc(float fc_hz) {
        if (fc_hz <= kQTaperFcLow) return kQLow;
        if (fc_hz >= kQTaperFcHigh) return kQHigh;
        float t = (log10f(fc_hz) - kQTaperLogLow) / (kQTaperLogHigh - kQTaperLogLow);
        return kQLow + t * (kQHigh - kQLow);
    }

#if DJ_FILTER_DIAG
    void diagPeakAfterProcess(float* left, float* right, uint32_t n) const {
        float peak = 0.0f;
        for (uint32_t i = 0; i < n; i++) {
            float a = fabsf(left[i]);
            float b = fabsf(right[i]);
            if (a > peak) peak = a;
            if (b > peak) peak = b;
        }
        if (peak <= kDiagPeakThreshold && std::isfinite(peak)) {
            return;
        }
        ESP_LOGW("DJFilter_DIAG",
                 "peak=%.5f |vTarget|=%.5f |vSmooth|=%.5f flip=%d toPos=%d n=%u",
                 (double)peak,
                 (double)_vTarget,
                 (double)_vSmooth,
                 (int)_flipBridgeActive,
                 (int)_flipBridgeToPositive,
                 (unsigned)n);
    }
#endif

    // Translate a smoothed v in [-1, +1] into LPF/HPF cutoffs (Going-Zero mapping,
    // with fc clamped to [kHpfFcMin, kHpfFcMax] / [kLpfFcMin, kLpfFcMax]).
    void applyCoefficients(float v) {
        const float kLog2_22000 = 14.425215f; // log2f(22000.0f)
        float vs = v / 1.3f;
        float fcLpf, fcHpf;
        if (vs < 0.0f) {
            fcLpf = powf(2.0f + vs, kLog2_22000);
            fcHpf = 1.0f;
        } else {
            fcLpf = 22000.0f;
            fcHpf = powf(1.0f + vs, kLog2_22000);
        }
        if (fcLpf > kLpfFcMax) fcLpf = kLpfFcMax;
        if (fcLpf < kLpfFcMin) fcLpf = kLpfFcMin;
        if (fcHpf > kHpfFcMax) fcHpf = kHpfFcMax;
        if (fcHpf < kHpfFcMin) fcHpf = kHpfFcMin;
        _lpf.setCutoff(BiquadIIR::LPF, fcLpf, _fs, qForFc(fcLpf));
        _hpf.setCutoff(BiquadIIR::HPF, fcHpf, _fs, qForFc(fcHpf));
    }

    static constexpr float _fs = 44100.0f;

    // Branching / bypass thresholds.
    static constexpr float kBridgeEpsilon = 0.02f;
    static constexpr float kStableBypassEps = 0.032f; // matches main.cpp UI deadzone 0.03 + small margin
    static constexpr float kCoeffBranchNoiseEps = 5e-5f;

    // Filter fc clamps - core of the deliberate Going-Zero divergence.
    static constexpr float kHpfFcMin = 20.0f;
    static constexpr float kHpfFcMax = 20000.0f;
    static constexpr float kLpfFcMin = 20.0f;
    static constexpr float kLpfFcMax = 20000.0f;

    // Q taper (log-linear). Going-Zero hard-coded Q = 5.
    static constexpr float kQLow = 0.707f;
    static constexpr float kQHigh = 5.0f;
    static constexpr float kQTaperFcLow = 30.0f;
    static constexpr float kQTaperFcHigh = 300.0f;
    // log10(30) and log10(300) precomputed (constexpr cannot call log10f portably).
    static constexpr float kQTaperLogLow = 1.4771213f;  // log10(30)
    static constexpr float kQTaperLogHigh = 2.4771213f; // log10(300)

#if DJ_FILTER_DIAG
    static constexpr float kDiagPeakThreshold = 4.0f; // normalized linear; sine at 1.0 -> ~1 peak
#endif

    BiquadIIR _lpf;
    BiquadIIR _hpf;
    volatile float _vTarget; // target set by setFilterValue() (audio thread; volatile for paranoia)
    float _vSmooth;          // actual parameter applied to coefficients (audio thread only)
    bool _flipBridgeActive;
    bool _flipBridgeToPositive;
};
