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

// Post-polarity-flip mute ramp: only armed on LPF<->HPF (see DJFilter). Q=5 biquads
// have a visibly longer transient after a coef reset than a short linear mute can hide:
// lengthening (~6 ms here) beats obvious clicks. Uses smoothstep for zero slope at the
// mute tail so the ramp does not reopen the input into a ringing filter with another edge.
class MiniFaderIn {
public:
    // ~264 samples @ 44.1 kHz (~6 ms). Only used after polarityFlip; harmless when idle.
    static constexpr uint32_t FADE_SAMPLE_NUM = 264;

    MiniFaderIn() : _count(FADE_SAMPLE_NUM) {}

    void startFadeIn() { _count = 0; }

    void processStereo(float* left, float* right, uint32_t n) {
        for (uint32_t i = 0; i < n; i++) {
            if (_count < FADE_SAMPLE_NUM) {
                float u = ((float)(_count + 1U)) / (float)FADE_SAMPLE_NUM;
                if (u > 1.0f) {
                    u = 1.0f;
                }
                // smoothstep(u) / smoothstep(1)==1 gives u*u*(3-2*u), du/dt ~ 0 at u=1.
                float rate = u * u * (3.0f - 2.0f * u);
                left[i] *= rate;
                right[i] *= rate;
                _count++;
            }
        }
    }

private:
    uint32_t _count;
};

class DJFilter {
public:
    DJFilter() : _vTarget(0.0f), _vSmooth(0.0f), _resetPending(false) {
        applyCoefficients(0.0f);
    }

    float getFilterValue() const { return _vTarget; }

    // v in [-1, +1]. v == 0 -> bypass; v < 0 -> LPF mode; v > 0 -> HPF mode.
    //
    // Click-guard policy (refined after listening tests):
    //
    //   * Only LPF<->HPF *polarity flips* (oldSign and newSign both nonzero, opposite
    //     signs) are click-protected with reset() + MiniFaderIn + _vSmooth snap. This
    //     happens when a very fast knob spin crosses the bypass deadzone in a single
    //     update. The biquad b coefficients change sign here, so reusing IIR state
    //     would produce a large transient.
    //
    //   * LPF<->bypass and bypass<->HPF are *not* click-protected. Bypass is implemented
    //     as LPF=22 kHz + HPF=1 Hz, which lives in the *same* coefficient family as the
    //     active mode on either side. The per-block linear ramp of _vSmooth toward
    //     _vTarget (see process()) moves fc continuously through that family, so IIR
    //     state stays consistent and no audible discontinuity is produced. An earlier
    //     version reset+faded these too; the snap-then-fade caused a large coefficient
    //     step that the 1 ms fade could not fully mask under Q=5, leaving an audible
    //     pop at every bypass crossing.
    //
    //   * Same-mode parameter changes are not fader-gated either: per-sub-block
    //     coefficient interpolation in process() keeps fc moving smoothly under fast
    //     spins.
    void setFilterValue(float v) {
        int oldSign = (_vTarget < 0.0f) ? -1 : ((_vTarget > 0.0f) ? 1 : 0);
        int newSign = (v < 0.0f) ? -1 : ((v > 0.0f) ? 1 : 0);
        bool polarityFlip = (oldSign != 0) && (newSign != 0) && (oldSign != newSign);
        if (polarityFlip) {
            _resetPending = true;
            _faderIn.startFadeIn();
            // Snap _vSmooth: interpolating across a polarity flip is meaningless because
            // we just reset() the IIR state and the b-coefficient sign changes discretely.
            _vSmooth = v;
        }
        _vTarget = v;
        // Coefficients are not (re)computed here; process() does that per sub-block.
    }

    // Real-time stereo processing. Coefficients are interpolated per sub-block so fc
    // moves smoothly even when the knob is spun much faster than the audio block rate.
    void process(float* left, float* right, uint32_t n) {
        if (_resetPending) {
            _lpf.reset();
            _hpf.reset();
            _resetPending = false;
        }
        if (n == 0) return;

        // Linear ramp of _vSmooth from its current value to _vTarget across this block.
        //   kSubBlock = 32 samples (~0.73 ms at 44.1 kHz). Coefficients are recomputed
        //   at every sub-block, so the user-visible reaction time to a knob change is
        //   one block (~5.8 ms at the default kI2SWriterFrames=256), independent of
        //   block size, and _vSmooth is float-equal to _vTarget at the end of the block
        //   (no asymptotic drift, so the "filter strength" the user perceives matches
        //   exactly what the parameter mapping prescribes).
        const uint32_t kSubBlock = 32;
        uint32_t numSubs = (n + kSubBlock - 1) / kSubBlock;
        float vStep = (_vTarget - _vSmooth) / (float)numSubs;

        uint32_t offset = 0;
        while (offset < n) {
            uint32_t sub = (n - offset > kSubBlock) ? kSubBlock : (n - offset);
            _vSmooth += vStep;
            applyCoefficients(_vSmooth);
            _faderIn.processStereo(left + offset, right + offset, sub);
            _hpf.processStereo(left + offset, right + offset, sub);
            _lpf.processStereo(left + offset, right + offset, sub);
            offset += sub;
        }
        // Snap to exact target to remove any accumulated float rounding error.
        _vSmooth = _vTarget;
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
    BiquadIIR _lpf;
    BiquadIIR _hpf;
    MiniFaderIn _faderIn;
    volatile float _vTarget; // target set by setFilterValue() (control thread)
    float _vSmooth;          // actual parameter applied to coefficients (audio thread only)
    bool _resetPending;
};
