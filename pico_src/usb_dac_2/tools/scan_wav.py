#!/usr/bin/env python3
"""Scan a 48 kHz stereo int16 WAV.

Silence: 5 ms RMS more than 20 dB below that channel's median.
Steps: sample delta above 2.5x the max slope of a 1 kHz sine at the
steady amplitude (median RMS * sqrt(2)). The file's own peak is not
used, so a spike cannot raise the threshold that should catch it.
Additive bursts: in each 10 ms block, subtract a fitted 1 kHz sine
plus DC. A residual peak above 0.12 is the kind of added transient
heard at 1:47 and 1:58 in the 30 min capture. Those bursts do not
move the sine's phase.
Clips: int16 samples at -32768 or 32767.

The first and last 0.5 s are reported as edges. Pass/fail uses the body.
"""

import math
import struct
import sys
import wave
from array import array

RATE = 48000
SILENCE_MS = 5.0
SILENCE_DB = 20.0
STEP_MULTIPLE = 2.5
ADDITIVE_PEAK = 0.12
BLOCK = 480  # 10 ms


def db(lin):
    if lin <= 1e-8:
        return -160.0
    return 20.0 * math.log10(lin)


def median(xs):
    if not xs:
        return 0.0
    ys = sorted(xs)
    return ys[len(ys) // 2]


def open_wav(path):
    w = wave.open(path)
    if w.getnchannels() != 2 or w.getframerate() != RATE or w.getsampwidth() != 2:
        sys.exit(
            f"unexpected wav ch={w.getnchannels()} rate={w.getframerate()} width={w.getsampwidth()}"
        )
    return w


def read_block(w, frames):
    raw = w.readframes(frames)
    n = len(raw) // 4
    if n == 0:
        return array("h")
    samples = array("h")
    samples.frombytes(raw[: n * 4])
    return samples


def fit_residual_peak(xs):
    """Peak |sample - fitted 1 kHz sine - DC| over one short block."""
    w = 2.0 * math.pi * 1000.0 / RATE
    sss = scc = ssc = ss = sc = s1 = sxs = sxc = sx = 0.0
    n = len(xs)
    sines = [0.0] * n
    coses = [0.0] * n
    for i, x in enumerate(xs):
        si = math.sin(w * i)
        ci = math.cos(w * i)
        sines[i] = si
        coses[i] = ci
        sss += si * si
        scc += ci * ci
        ssc += si * ci
        ss += si
        sc += ci
        s1 += 1.0
        sxs += x * si
        sxc += x * ci
        sx += x
    m = [[sss, ssc, ss], [ssc, scc, sc], [ss, sc, s1]]
    b = [sxs, sxc, sx]
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(m[r][i]))
        m[i], m[piv] = m[piv], m[i]
        b[i], b[piv] = b[piv], b[i]
        d = m[i][i]
        if d == 0.0:
            return 0.0
        for j in range(i, 3):
            m[i][j] /= d
        b[i] /= d
        for r in range(3):
            if r == i:
                continue
            f = m[r][i]
            for j in range(i, 3):
                m[r][j] -= f * m[i][j]
            b[r] -= f * b[i]
    aa, bb, dc = b
    peak = 0.0
    for i, x in enumerate(xs):
        err = abs(x - (aa * sines[i] + bb * coses[i] + dc))
        if err > peak:
            peak = err
    return peak


def where(t, duration):
    if t < 0.5 or t > duration - 0.5:
        return "edge"
    return "body"


def merge_hits(hits, gap):
    by_ch = {}
    for t, ch, value in sorted(hits, key=lambda item: (item[1], item[0])):
        events = by_ch.setdefault(ch, [])
        if events and t - events[-1]["t1"] <= gap:
            events[-1]["t1"] = t
            events[-1]["value"] = max(events[-1]["value"], value)
            events[-1]["n"] += 1
        else:
            events.append({"t0": t, "t1": t, "ch": ch, "value": value, "n": 1})
    merged = [ev for events in by_ch.values() for ev in events]
    merged.sort(key=lambda ev: (ev["t0"], ev["ch"]))
    return merged


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: scan_wav.py in.wav out.txt")
    wav_path, out_path = sys.argv[1], sys.argv[2]
    win = int(round(RATE * SILENCE_MS / 1000.0))

    rms = [[], []]
    frames = 0
    with open_wav(wav_path) as w:
        total = w.getnframes()
        while frames < total:
            samples = read_block(w, 48000)
            if not samples:
                break
            nfr = len(samples) // 2
            base = 0
            while base + win <= nfr:
                for ch in (0, 1):
                    acc = 0.0
                    for i in range(win):
                        s = samples[(base + i) * 2 + ch] / 32768.0
                        acc += s * s
                    rms[ch].append(math.sqrt(acc / win))
                base += win
            frames += nfr
    duration = frames / RATE
    med = [median(rms[0]), median(rms[1])]
    silence = []
    for wi in range(min(len(rms[0]), len(rms[1]))):
        hit = []
        levels = []
        for ch in (0, 1):
            if med[ch] > 0 and rms[ch][wi] < med[ch] * 10 ** (-SILENCE_DB / 20.0):
                hit.append(ch)
                levels.append(db(rms[ch][wi]))
        if hit:
            t = wi * win / RATE
            silence.append((t, where(t, duration), hit, levels))

    steady_amp = [m * math.sqrt(2.0) for m in med]
    slope = [2.0 * math.pi * 1000.0 / RATE * a for a in steady_amp]
    step_lin = [STEP_MULTIPLE * s for s in slope]

    step_hits = []
    add_hits = []
    clips = []
    frame_index = 0
    prev = [0, 0]
    have_prev = False
    with open_wav(wav_path) as w:
        remain = w.getnframes()
        while remain > 0:
            take = min(BLOCK, remain)
            samples = read_block(w, take)
            nfr = len(samples) // 2
            if nfr == 0:
                break
            for ch in (0, 1):
                xs = [samples[i * 2 + ch] / 32768.0 for i in range(nfr)]
                if have_prev:
                    jump = abs(xs[0] - prev[ch])
                    if jump > step_lin[ch]:
                        t = frame_index / RATE
                        step_hits.append((t, ch, jump))
                for i in range(1, nfr):
                    jump = abs(xs[i] - xs[i - 1])
                    if jump > step_lin[ch]:
                        t = (frame_index + i) / RATE
                        step_hits.append((t, ch, jump))
                prev[ch] = xs[-1]
                peak = fit_residual_peak(xs)
                if peak > ADDITIVE_PEAK:
                    t = frame_index / RATE
                    add_hits.append((t, ch, peak))
            for i in range(nfr):
                for ch in (0, 1):
                    s = samples[i * 2 + ch]
                    if s <= -32768 or s >= 32767:
                        t = (frame_index + i) / RATE
                        clips.append((t, ch, s, where(t, duration)))
                        if len(clips) > 20:
                            break
                if len(clips) > 20:
                    break
            have_prev = True
            frame_index += nfr
            remain -= nfr

    steps = merge_hits(step_hits, 0.05)
    additive = merge_hits(add_hits, 0.05)
    # A burst is common when both channels fire inside the same 50 ms.
    paired = []
    used = set()
    for i, ev in enumerate(additive):
        if i in used:
            continue
        mate = None
        mate_i = None
        for j, other in enumerate(additive):
            if j in used or other["ch"] == ev["ch"]:
                continue
            if other["t0"] <= ev["t1"] + 0.05 and ev["t0"] <= other["t1"] + 0.05:
                mate = other
                mate_i = j
                break
        used.add(i)
        if mate_i is not None:
            used.add(mate_i)
        t0 = ev["t0"] if mate is None else min(ev["t0"], mate["t0"])
        t1 = ev["t1"] if mate is None else max(ev["t1"], mate["t1"])
        peak = ev["value"] if mate is None else max(ev["value"], mate["value"])
        paired.append([t0, t1, peak, 1 if mate is None else 2])
    common = []
    for t0, t1, peak, channels in paired:
        if common and t0 <= common[-1]["t1"] + 0.05:
            common[-1]["t1"] = max(common[-1]["t1"], t1)
            common[-1]["peak"] = max(common[-1]["peak"], peak)
            common[-1]["channels"] = max(common[-1]["channels"], channels)
        else:
            common.append(
                {
                    "t0": t0,
                    "t1": t1,
                    "peak": peak,
                    "channels": channels,
                    "where": where(t0, duration),
                }
            )

    def body(item_where):
        return item_where == "body"

    body_silence = [s for s in silence if s[1] == "body"]
    body_steps = [s for s in steps if where(s["t0"], duration) == "body"]
    body_add = [s for s in common if s["where"] == "body"]
    body_clips = [c for c in clips if c[3] == "body"]
    ok = not body_silence and not body_steps and not body_add and not body_clips

    lines = []
    lines.append(f"wav: {wav_path}")
    lines.append(f"duration_s: {duration:.3f}")
    lines.append(f"frames: {frames}")
    lines.append(
        f"median_5ms_rms_dbfs: L={db(med[0]):.2f} R={db(med[1]):.2f}"
    )
    lines.append(
        f"steady_amp_dbfs: L={db(steady_amp[0]):.2f} R={db(steady_amp[1]):.2f}"
    )
    lines.append(
        f"silence_rule: {SILENCE_MS:g} ms RMS more than {SILENCE_DB:g} dB below that channel's median"
    )
    lines.append(
        "step_rule: |sample delta| > "
        f"{STEP_MULTIPLE:g}x max slope of 1 kHz at the steady amplitude "
        f"(L>{step_lin[0]:.4f} R>{step_lin[1]:.4f})"
    )
    lines.append(
        "additive_rule: 10 ms residual peak after removing a fitted 1 kHz sine and DC "
        f"> {ADDITIVE_PEAK:.2f}"
    )
    lines.append("clip_rule: int16 at -32768 or 32767")
    lines.append(
        f"counts: silence={len(silence)} (body {len(body_silence)}) "
        f"steps={len(steps)} (body {len(body_steps)}) "
        f"additive={len(common)} (body {len(body_add)}) "
        f"clips_listed={len(clips)} (body {len(body_clips)})"
    )
    lines.append("silence:")
    if not silence:
        lines.append("  none")
    else:
        for t, edge, hit, levels in silence[:40]:
            chs = ",".join(f"{c}:{lv:.1f}dBFS" for c, lv in zip(hit, levels))
            lines.append(f"  {t:.3f}s {edge} {chs}")
        if len(silence) > 40:
            lines.append(f"  ... {len(silence) - 40} more")
    lines.append("steps:")
    if not steps:
        lines.append("  none")
    else:
        for ev in steps[:40]:
            lines.append(
                f"  {ev['t0']:.3f}-{ev['t1']:.3f}s ch{ev['ch']} "
                f"{where(ev['t0'], duration)} max_jump={ev['value']:.3f} n={ev['n']}"
            )
        if len(steps) > 40:
            lines.append(f"  ... {len(steps) - 40} more")
    lines.append("additive:")
    if not common:
        lines.append("  none")
    else:
        for ev in common[:40]:
            lines.append(
                f"  {ev['t0']:.3f}-{ev['t1']:.3f}s {ev['where']} "
                f"channels={ev['channels']} residual_peak={ev['peak']:.3f}"
            )
        if len(common) > 40:
            lines.append(f"  ... {len(common) - 40} more")
    lines.append("clips:")
    if not clips:
        lines.append("  none")
    else:
        for t, ch, s, edge in clips[:20]:
            lines.append(f"  {t:.3f}s ch{ch} {edge} sample={s}")
    lines.append(f"body_glitch: {'PASS' if ok else 'FAIL'}")
    text = "\n".join(lines) + "\n"
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(text)
    print(text, end="")
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
