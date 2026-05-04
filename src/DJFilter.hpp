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

// ~1 ms linear fade-in on demand; used to mask filter transients on every mode-boundary
// transition (LPF<->bypass, bypass<->HPF, and the LPF<->HPF polarity flip), and on large
// same-mode jumps (very fast knob spins).
class MiniFaderIn {
public:
    static constexpr uint32_t FADE_SAMPLE_NUM = 50;

    MiniFaderIn() : _count(FADE_SAMPLE_NUM) {}

    void startFadeIn() { _count = 0; }

    void processStereo(float* left, float* right, uint32_t n) {
        for (uint32_t i = 0; i < n; i++) {
            if (_count < FADE_SAMPLE_NUM) {
                float rate = (float)_count / (float)FADE_SAMPLE_NUM;
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
    DJFilter() : _v(0.0f), _resetPending(false) {
        setFilterValue(0.0f);
    }

    float getFilterValue() const { return _v; }

    // v in [-1, +1]. v == 0 -> bypass; v < 0 -> LPF mode; v > 0 -> HPF mode.
    //
    // Pop-noise click guard on mode-boundary crossings.
    // The 2nd-order biquad keeps x/y history that is meaningful only for the filter type that
    // produced it. Reusing that history with a different filter type, or with a very different
    // cutoff, can produce a large transient -> audible click. We classify the parameter into
    // three "modes" by the sign of v (LPF / bypass / HPF) and arm a click guard whenever the
    // mode changes:
    //   * _resetPending: clear IIR state on the next process() call so the new coefficients see
    //     a clean state (avoids LPF-state-with-HPF-coefs explosion in particular).
    //   * MiniFaderIn:   ~1 ms linear input ramp 0->1 on the next process() call so the audible
    //     transient at the start of the new filter's response is masked.
    // Going-Zero divergence: the original DJFilter.m only triggers fadeIn on the
    // active->bypass transition. We additionally guard bypass->active and the direct LPF<->HPF
    // polarity flip (which can happen when a fast knob spin skips the bypass deadzone). The
    // extra reset on polarity flip is the most important addition because biquad b-coefficient
    // signs invert between LPF and HPF, so reusing state across that boundary is the dominant
    // pop-noise source observed when sweeping the knob quickly.
    void setFilterValue(float v) {
        int oldSign = (_v < 0.0f) ? -1 : ((_v > 0.0f) ? 1 : 0);
        int newSign = (v < 0.0f) ? -1 : ((v > 0.0f) ? 1 : 0);
        if (oldSign != newSign) {
            // Mode boundary crossing (LPF<->bypass, bypass<->HPF, LPF<->HPF on a fast sweep that
            // skipped the deadzone). Reset IIR state so old-mode history does not feed new-mode
            // coefficients, and ramp the input to mask the residual transient.
            _resetPending = true;
            _faderIn.startFadeIn();
        } else if (fabsf(v - _v) >= kBigJumpThreshold) {
            // Same-mode large jump (a very fast knob spin can move v by >0.1 between blocks even
            // after the EMA pre-smoothing in loop()). The IIR state is still consistent with the
            // current mode, so we do *not* reset it; just fade the input briefly to soften the
            // coefficient-step transient. Going-Zero has this trigger as commented-out code in
            // DJFilter.m; we re-enable it as defense in depth.
            _faderIn.startFadeIn();
        }
        _v = v;
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

    // Real-time stereo processing.
    //
    // Going-Zero match: regardless of mode, the audio always flows through the same
    // chain (faderIn -> HPF -> LPF). In bypass (v == 0) the cutoffs are LPF=22 kHz and
    // HPF=1 Hz, so the chain is effectively passthrough but the MiniFaderIn ramp can still
    // be applied at mode boundaries. The original DJFilter.m additionally calls reset()
    // every block while v == 0; we replicate that so a long bypass period leaves a clean
    // state when the knob next moves.
    //
    // Earlier versions of this port early-returned in bypass. That avoided two biquads
    // per block (CPU win) but introduced an audible step at active->bypass: the previous
    // block's output was the filtered signal, the next block's output was the raw input,
    // and the 1-sample jump between them was the click the user reported. Running through
    // the near-passthrough chain with the input ramp turns that jump into a 1 ms fade.
    void process(float* left, float* right, uint32_t n) {
        if (_resetPending) {
            _lpf.reset();
            _hpf.reset();
            _resetPending = false;
        }
        if (_v == 0.0f) {
            _lpf.reset();
            _hpf.reset();
        }
        _faderIn.processStereo(left, right, n);
        _hpf.processStereo(left, right, n);
        _lpf.processStereo(left, right, n);
    }

    void reset() {
        _lpf.reset();
        _hpf.reset();
    }

private:
    static constexpr float _fs = 44100.0f;
    static constexpr float kBigJumpThreshold = 0.1f; // |delta v| within the same mode that warrants a fade
    BiquadIIR _lpf;
    BiquadIIR _hpf;
    MiniFaderIn _faderIn;
    volatile float _v;
    bool _resetPending;
};
