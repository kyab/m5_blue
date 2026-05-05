// DJFilter: real-time LPF/HPF crossover filter controlled by a single parameter v in [-1, +1].
// C++ port of Going-Zero (https://github.com/kyab/Going-Zero) DJFilter / LPF_IIR / HPF_IIR / MiniFader.
// Mac-side 30s ring buffers are unnecessary: a 2nd-order direct-form biquad with 2 input + 2 output
// state samples per channel is mathematically equivalent and fits on an MCU.

#pragma once

#include <cmath>
#include <cstdint>

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

    // Compute biquad coefficients. Matches Going-Zero IIR_LPF / IIR_HPF (bilinear transform, Q=5.0).
    void setCutoff(Type type, float fc_hz, float fs_hz) {
        const float Q = 5.0f;
        float fc = fc_hz / fs_hz;
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

    // Direct-form biquad: y[n] = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2
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

        // Linear ramp of _vSmooth from its current value to _vTarget across this block.
        //   kSubBlock = 32 samples (~0.73 ms at 44.1 kHz). Coefficients are recomputed
        //   at every sub-block, so the user-visible reaction time to a knob change is
        //   one block (~5.8 ms at the default kI2SWriterFrames=256), independent of
        //   block size, and _vSmooth is float-equal to _vTarget at the end of the block
        //   (no asymptotic drift, so the "filter strength" the user perceives matches
        //   exactly what the parameter mapping prescribes).
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
            applyCoefficients(_vSmooth);
            _hpf.processStereo(left + offset, right + offset, sub);
            _lpf.processStereo(left + offset, right + offset, sub);
            offset += sub;
            if (remainingSubs > 0) {
                remainingSubs--;
            }
        }
        // Snap to exact target to remove any accumulated float rounding error.
        if (!_flipBridgeActive) {
            _vSmooth = _vTarget;
        }
    }

    void reset() {
        _lpf.reset();
        _hpf.reset();
    }

private:
    // Translate a smoothed v in [-1, +1] into LPF/HPF cutoffs (Going-Zero mapping).
    void applyCoefficients(float v) {
        float vs = v / 1.3f;
        const float kLog2_22000 = 14.425215f; // log2f(22000.0f)
        if (vs < 0.0f) {
            float fc = powf(2.0f + vs, kLog2_22000);
            _lpf.setCutoff(BiquadIIR::LPF, fc, _fs);
            _hpf.setCutoff(BiquadIIR::HPF, 1.0f, _fs);
        } else {
            _lpf.setCutoff(BiquadIIR::LPF, 22000.0f, _fs);
            float fc = powf(1.0f + vs, kLog2_22000);
            _hpf.setCutoff(BiquadIIR::HPF, fc, _fs);
        }
    }

    static constexpr float _fs = 44100.0f;
    static constexpr float kBridgeEpsilon = 0.02f;
    BiquadIIR _lpf;
    BiquadIIR _hpf;
    volatile float _vTarget; // target set by setFilterValue() (control thread)
    float _vSmooth;          // actual parameter applied to coefficients (audio thread only)
    bool _flipBridgeActive;
    bool _flipBridgeToPositive;
};
