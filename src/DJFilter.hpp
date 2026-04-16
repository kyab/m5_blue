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

// ~1ms linear fade-in on demand; used to suppress clicks when entering bypass.
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
    DJFilter() : _v(0.0f) {
        setFilterValue(0.0f);
    }

    float getFilterValue() const { return _v; }

    // v in [-1, +1]. v == 0 -> bypass; v < 0 -> LPF mode; v > 0 -> HPF mode.
    // Fires MiniFaderIn when transitioning from nonzero back to 0 (click guard).
    void setFilterValue(float v) {
        if (_v != 0.0f && v == 0.0f) {
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

    // Real-time stereo processing. Bypass + reset when v == 0 exactly.
    void process(float* left, float* right, uint32_t n) {
        if (_v == 0.0f) {
            _lpf.reset();
            _hpf.reset();
            return;
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
    BiquadIIR _lpf;
    BiquadIIR _hpf;
    MiniFaderIn _faderIn;
    volatile float _v;
};
