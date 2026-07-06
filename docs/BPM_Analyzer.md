# BPM Analyzer (ESP32 port) — design, divergence from Going-Zero, and debugging brief

> **Purpose of this document.**
> This file is meant to be loaded by an AI coding agent in a *separate* debugging chat.
> The on-device BPM display currently shows values that clearly differ from the reference
> app (Going-Zero). This document describes, in enough detail to debug without re-reading
> the whole tree:
> 1. What the reference (Going-Zero) actually does.
> 2. What this branch implemented, algorithm by algorithm, and **which parts were dropped or
>    simplified** (how far the port is "degraded").
> 3. The **buffers** used for analysis (type / size / seconds), and how they differ from
>    Going-Zero (e.g. shorter history because of limited RAM).
> 4. Concrete hypotheses for *why the BPM is wrong*, and a prioritized list of what to port
>    next.
>
> If you are that debugging agent: start from **§6 (Likely causes of wrong BPM)** and
> **§4 (Buffer comparison)**. The reference algorithm is Essentia `BeatTrackerMultiFeature`;
> per `AGENTS.md`, consult the Going-Zero source on GitHub directly when in doubt
> (`https://github.com/kyab/Going-Zero`, file `Going Zero/filter/BeatTracker.mm`).

---

## 1. File map (this branch)

All BPM code is isolated under `src/bpm/` and is independent of the audio output path.

| File | Role |
|------|------|
| `src/bpm/bpm_config.hpp` | Compile-time constants (sample rate, FFT/hop, mel bands, tempo range, buffer sizes). |
| `src/bpm/mel_filterbank.{hpp,cpp}` | 40-band HTK mel filterbank, 0–4000 Hz (for the melflux ODF). |
| `src/bpm/stft_odf_pipeline.{hpp,cpp}` | Mono ring → Hann STFT (arduinoFFT) → magnitude/phase → **3 onset detection functions** (complex, rms, melflux). |
| `src/bpm/davies_tempo_estimator.{hpp,cpp}` | Per-ODF adaptive threshold + **Rayleigh-weighted autocorrelation** tempo estimate (`DaviesStyleTempoEstimator`), wrapped by `EssentiaStyleOdfBranch`. |
| `src/bpm/streaming_bpm_analyzer.{hpp,cpp}` | Facade: PCM FIFO, worker task, calls STFT/ODF/tempo, **median-of-3 + EWMA fusion**, exposes `bpm()`. |
| `src/main.cpp` | Constructs the analyzer after `M5.begin()`, feeds PCM from `audio_callback`, starts the worker task, draws BPM on screen. |

Integration points in `src/main.cpp`:
- `static bpm::StreamingBpmAnalyzer* g_streaming_bpm = nullptr;` (heap, constructed in `setup()` after `M5.begin()`).
- `audio_callback()` → `g_streaming_bpm->enqueueStereoInterleaved(data, sample_num)`.
- `a2dp_audio_state_callback()` → `g_streaming_bpm->reset()` on stop.
- `bpm::start_bpm_worker_task(g_streaming_bpm)` (worker pinned to **core 1**).
- `loop()` draws `g_streaming_bpm->bpm()` at ~5 Hz.

Data flow:

```
BT A2DP PCM (int16 stereo, 44.1 kHz)
  → enqueueStereoInterleaved  → 200 KiB PSRAM byte FIFO
  → [worker task, core 1] service()
      → mono = 0.5*(L+R)/32768         (float, -1..1)
      → StftOdfPipeline.pushMonoFloat  (4096 mono ring, 2048/1024 Hann STFT)
          → complex_odf, rms_odf, melflux_odf   (one triplet per 1024-sample hop)
      → 3× EssentiaStyleOdfBranch.pushStftHop(odf)
          → x2 linear upsample → adaptive threshold → Davies ACF tempo
      → fuseMedianEwma()  → display_bpm_
  → loop() renders display_bpm_
```

---

## 2. Reference: what Going-Zero actually does

Going-Zero's `BeatTracker.mm` is a thin wrapper around **Essentia's streaming
`BeatTrackerMultiFeature`** algorithm running on macOS (full desktop CPU, `float`/`Real`):

```objc
_intBeatTracker = factory.create("BeatTrackerMultiFeature");
_vecInput >> _intBeatTracker->input("signal");
_intBeatTracker->output("ticks") >> PC(_pool, "rhythm.ticks");
```

Key reference behavior (from `BeatTracker.mm`):

- **Input**: mono `(L+R)/2`, `float`, 44.1 kHz.
- **Windowing over time** (rolling analysis, not whole-file):
  - Warm-up: first dispatch at **3 s**, then **5 s**.
  - Steady state: process when the buffer reaches **11 s**
    (`_audioFragment.size() >= 11 * 44100`); keep only ticks in the **[5 s, 10 s]**
    slice of each run; then **shift left by 5 s** and keep a **6 s overlap**
    (`shift_left(...,44100*5)` + `erase(begin+44100*6, end)`), then append newly
    buffered audio. So it is an **11 s analysis window with a 5 s hop and 6 s overlap**.
  - `_audioFragment.reserve(44100*20)` → up to **20 s** of mono float reserved.
- **Output**: **beat tick positions** (`rhythm.ticks`, seconds), not a tempo scalar.
- **Tempo/BPM derivation** (`-(void)update`): `beatDuration = mean of the last 8
  inter-tick intervals`, then `BPM = 60 / beatDuration`. It also does off-beat / phase
  handling (`pastBeatRelativeSec`, `estimatedNextBeatRelativeSec`, `flipOffBeat`).

### What `BeatTrackerMultiFeature` does internally (Essentia)

`BeatTrackerMultiFeature` estimates beats by:

1. Computing **five onset detection functions (ODFs)** and estimating beats from each with
   `TempoTapDegara`, then selecting the most consistent estimate with
   `TempoTapMaxAgreement`. The five ODFs are:
   - **complex** spectral difference (`OnsetDetection` method `complex`)
   - **energy flux** (`OnsetDetection` method `rms` / energy)
   - **spectral flux in mel bands** (`OnsetDetection` method `melflux`)
   - **beat emphasis** (`OnsetDetectionGlobal` method `beat_emphasis`)
   - **modified information gain** (`OnsetDetectionGlobal` method `infogain`)
2. `TempoTapDegara` = **Degara et al. (2012)** probabilistic beat tracking: it turns an ODF
   into beat *ticks* using a period-salience stage (resonator/comb-filter-like) plus a
   **hidden-Markov / Viterbi** decoding of beat period *and phase*. Input ODF rate is
   `44100/512 ≈ 86.13 Hz`.
3. `TempoTapMaxAgreement` picks, among the per-ODF tick sequences, the one with maximum
   mutual agreement (robust voting across features).

So the reference is a **full multi-feature beat tracker with phase**, not just a tempo
estimator.

---

## 3. What this branch ported vs. dropped (algorithm by algorithm)

| Reference stage (Essentia `BeatTrackerMultiFeature`) | This branch | Status |
|---|---|---|
| Input: mono `(L+R)/2`, float, 44.1 kHz | `0.5*(L+R)/32768` float | **Ported** (equivalent). |
| STFT frame/hop | 2048 / 1024, Hann (symmetric `N-1`) | **Ported** (frame/hop match the complex/melflux path). |
| ODF: **complex** spectral difference | `onsetComplex()` (target-phase prediction, `|pred-meas|`) | **Ported** (simplified normalization). |
| ODF: **energy/rms flux** | `onsetRms()` (`sqrt(Σmag²)/N`, half-wave diff) | **Ported** (scale/normalization not matched to Essentia). |
| ODF: **melflux** (40 bands, 0–4000 Hz, dB) | `onsetMelFlux()` + `MelFilterbank` (HTK), `amp2dbEssentia` | **Ported** (mel warp/bounds match defaults). |
| ODF: **beat_emphasis** (`OnsetDetectionGlobal`) | — | **DROPPED** (global/whole-signal; hard to stream). |
| ODF: **infogain** (`OnsetDetectionGlobal`) | — | **DROPPED** (global/whole-signal). |
| ODF → 2× upsample to ~86.13 Hz for tempo stage | `pushStftHop()` linear x2 (`0.5*(prev+cur)`, `cur`) | **Ported** (rate matches TempoTapDegara input). |
| Onset post-processing (Essentia normalization / moving average) | `OdfAdaptiveThreshold` (17-tap moving-average subtract + half-wave rectify) | **Approximated** (window/params not matched). |
| Tempo/beat estimation: **TempoTapDegara** (comb/resonator + HMM/Viterbi, ticks + phase) | `DaviesStyleTempoEstimator`: **Rayleigh-weighted plain autocorrelation**, argmax lag → BPM | **HEAVILY SIMPLIFIED** — tempo scalar only, **no comb filterbank, no HMM/Viterbi, no beat phase/ticks**. |
| Multi-feature selection: **TempoTapMaxAgreement** (agreement over tick sequences) | `fuseMedianEwma()`: median of ≤3 instantaneous BPMs + EWMA (`0.88/0.12`) | **HEAVILY SIMPLIFIED** — scalar median, no sequence agreement. |
| BPM = `60 / mean(last 8 inter-tick intervals)` | `60*sr_odf/best_lag`, clamped 40–208 | **Different derivation** (from lag, not from tracked beats). |
| Beat phase / off-beat / next-beat prediction | — | **DROPPED** (tempo only; no phase). |

### Degradation summary (one paragraph)

The port keeps the **front end** close to Essentia (mono mix, 2048/1024 Hann STFT, and 3 of
the 5 onset detection functions: complex, rms, melflux). Everything **behind the ODFs is
replaced by a much simpler tempo estimator**: instead of `TempoTapDegara`
(resonator/comb + Viterbi beat tracking producing *ticks* and *phase*) it uses a single
**Rayleigh-weighted autocorrelation argmax** over a short ODF history; instead of
`TempoTapMaxAgreement` it uses **median + EWMA** of the three per-ODF instantaneous tempi.
It outputs a **tempo scalar only** — there is no beat tracking, no phase, no off-beat, and
**no octave/harmonic disambiguation**. Two of five ODFs (`beat_emphasis`, `infogain`) are
omitted. This is the most likely reason the displayed BPM diverges from Going-Zero
(see §6).

---

## 4. Buffers used for analysis: type / size / seconds, and vs. Going-Zero

Rates: STFT hop rate = `44100/1024 = 43.07 Hz`; ODF tempo-stage rate (x2) =
`86.13 Hz` (matches Essentia `TempoTapDegara` input `44100/512`).

### 4a. This branch

| Buffer | Location | Type | Length | Bytes | Time span | Notes |
|---|---|---|---|---|---|---|
| PCM transport FIFO (`fifo_buf_`) | PSRAM | `uint8_t` (int16 stereo) | `200 KiB` = 51 200 frames | 204 800 | **~1.16 s** | Only a transport queue BT→worker, **not** the analysis history. Drops oldest on overflow. |
| Mono STFT ring (`ring_`) | PSRAM | `float` | `kMonoRingSize = 4096` | 16 384 | **~92.8 ms** | Just enough to frame 2048-pt FFTs with 1024 hop. |
| Hann window (`hann_`) | PSRAM | `float` | 2048 | 8 192 | — | |
| FFT real/imag (`fft_real_`, `fft_imag_`) | PSRAM | **`double`** | 2048 each | 16 384 each | — | **arduinoFFT uses `double`**; Essentia uses `float`. |
| Magnitude/phase (`mag_`,`phase_`,`phase_1_`,`phase_2_`,`spectrum_1_`) | PSRAM | `float` | `kSpectrumBins = 1025` each | 4 100 each | — | For complex ODF phase history. |
| Mel scratch (`mel_linear_`,`prev_mel_db_`) | PSRAM | `float` | `kMelBands = 40` each | 160 each | — | |
| ODF history per branch (`odf_ring_`) | analyzer heap (internal DRAM) | `float` | `kBuffer = 512` | 2 048 | **~5.94 s** | **This is the actual tempo-analysis window.** Estimation starts at `ring_count_ >= 256` (**~3.0 s**). |
| Rayleigh weights (`rayleigh_weight_`) | analyzer heap | `float` | 256 | 1 024 | — | Peak near lag for 120 BPM. |
| Adaptive-threshold ring (`buf_`) | analyzer heap | `float` | `kWin = 17` | 68 | ~0.2 s | Moving-average baseline. |

There are **3** ODF branches (complex, rms, melflux), so the tempo-stage arrays are ×3.

**Effective analysis memory of past audio ≈ 5.94 s**, held as a **downsampled ODF**
(86.13 Hz), *not* as raw audio.

### 4b. Going-Zero

| Buffer | Type | Length | Time span | Notes |
|---|---|---|---|---|
| `_audioFragment` (reserve) | `float`/`Real` mono | `44100*20` | up to **20 s** reserved | Raw audio, full precision. |
| Steady-state analysis window | `float` | `11 * 44100` | **11 s** processed per run | 5 s hop, 6 s overlap. |
| Ticks slice kept per run | seconds | ticks in **[5 s, 10 s]** | — | |
| Overflow while async busy | `_audioPool` | unbounded (`push_back`) | — | Buffers incoming audio during background run. |

### 4c. Key buffer differences

- **History length**: reference tempo/beat tracking sees **11 s of raw audio** (with 20 s
  reserved). This branch sees **~5.9 s of ODF** (min 3 s) — roughly **half the temporal
  context**, and downsampled. Shorter history → coarser autocorrelation lag resolution and
  fewer beat periods to average.
- **Lag quantization**: at 86.13 Hz, lag for 120 BPM ≈ 43 samples; a ±1-sample lag error is
  **±~3 BPM at 120**, and worse at high tempi (integer-lag argmax has no interpolation).
- **What is stored**: reference keeps **raw audio** and re-runs the full pipeline; this
  branch keeps only a **rectified ODF ring** — it cannot recover spectral detail lost in the
  ODF.
- **Numeric type**: FFT here is **`double`** (arduinoFFT), Essentia is **`float`** — not a
  correctness problem, but a memory/CPU cost note.
- **Transport FIFO** (200 KiB, ~1.16 s) is unrelated to analysis span; if the worker stalls,
  it drops oldest PCM (analysis sees gaps), while Going-Zero buffers into `_audioPool`
  without dropping.

### 4d. Config constants (`src/bpm/bpm_config.hpp`)

```
kSampleRate   = 44100
kFftSize      = 2048     kHopSize   = 1024     kSpectrumBins = 1025
kMelBands     = 40       kMelLowHz  = 0        kMelHighHz    = 4000
kMinTempo     = 40       kMaxTempo  = 208
kMonoRingSize = 4096     kMonoRingMask = 4095
odfSampleRateBase = 44100/1024 = 43.07 Hz     odfSampleRateX2 = 86.13 Hz
```
Tempo estimator (`davies_tempo_estimator.hpp`): `kBuffer = 512`, `rayleigh_weight_[256]`,
`OdfAdaptiveThreshold::kWin = 17`. Fusion (`streaming_bpm_analyzer.cpp`): EWMA
`display_bpm_ = 0.88*display_bpm_ + 0.12*fused`.

---

## 5. Threading / runtime notes (context for reproducing the bug)

- Worker `bpm_work` is pinned to **core 1** (stack 4096 words), separate from BT (`BTC_TASK`)
  / IDLE0 on core 0. This was required to stop a **Task WDT abort** (`IDLE0 (CPU 0)` starved)
  that previously crashed after ~1 min of playback.
- `service()` processes at most `kServiceMonoSampleBudget = 2048` mono samples per call, then
  `vTaskDelay(1 ms)`, so it never monopolizes the CPU.
- All large STFT buffers and the FIFO are **PSRAM only** (no internal-DRAM fallback) to avoid
  a boot-time `esp_startup_start_app_common` assert from DRAM exhaustion.
- Analyzer object is constructed with `new` **after `M5.begin()`** (not as a static global),
  so PSRAM is initialized before large allocations.
- Button effects (Freezer / delay) are currently **disabled** via `kButtonEffectsEnabled =
  false` in `main.cpp` (unrelated to BPM, but it means `g_ring` is not allocated).

---

## 6. Likely causes of wrong BPM (debugging hypotheses, ranked)

Start here. Most probable first.

1. **Octave / harmonic errors from plain autocorrelation (highest suspicion).**
   `estimateBpmFromBuffer()` takes the **argmax of a Rayleigh-weighted ACF**. An ACF has
   peaks at the true period *and its integer multiples/divisors*, so the global max easily
   lands on **half or double** the real tempo (e.g. shows 75 or 150 for a true 150/75). There
   is **no octave disambiguation**. Essentia's `TempoTapDegara` avoids this with a resonator
   comb + HMM. → Check whether the displayed value is ~½ or ~2× the true BPM.

2. **No comb filterbank (Davies is only half-implemented).**
   The real Davies (2007) tempo stage uses a **shift-invariant comb filterbank** over the
   ACF, which reinforces a period *and its harmonics together*; plain ACF does not. This
   changes which lag wins.

3. **Rayleigh weighting centered at 120 BPM biases results.**
   `tau120 = 60*sr_odf/120`; the weight peaks at 120 BPM. Off-center tempi are penalized,
   pulling estimates toward 120 and interacting badly with hypothesis (1).

4. **Coarse lag resolution + short window** (see §4c). Integer-lag argmax with no parabolic
   interpolation gives quantized BPM (±~3 BPM near 120, worse higher). The ~5.9 s ODF window
   (vs 11 s) also gives fewer periods to stabilize.

5. **Only 3 of 5 ODFs, and weak fusion.** Missing `beat_emphasis` (which is specifically
   tempo-oriented) and `infogain`. `fuseMedianEwma()` medians *instantaneous scalar tempi*,
   which is far weaker than `TempoTapMaxAgreement` over tick sequences; one branch stuck on a
   harmonic can drag the median.

6. **Onset post-processing mismatch.** `OdfAdaptiveThreshold` (17-tap MA subtract + rectify)
   is not Essentia's onset normalization; a different ODF shape changes ACF peak locations.
   Also `onsetRms()` divides energy by `N` (1025), producing very small values — check scale
   vs the other ODFs before fusion.

7. **Hann window convention.** Symmetric `0.5*(1-cos(2π n/(N-1)))` vs Essentia's normalized
   periodic Hann — minor spectral-leakage difference, unlikely to be the main error but worth
   ruling out.

**Suggested verification method:** feed the *same* audio to Going-Zero
(`BeatTrackerMultiFeature`, mono 44.1 kHz) and to this device; log per-branch
`instantBpm()` (complex/rms/melflux) and the fused value. If per-branch values cluster at
½/2× the reference, hypotheses (1)–(3) are confirmed.

---

## 7. What to port next (prioritized proposals)

Ordered by expected accuracy gain per unit of effort/memory on ESP32.

1. **Octave/harmonic correction on top of the current ACF (cheap, high impact).**
   After finding `best_lag`, evaluate candidate tempi `{T, 2T, T/2, 3T, T/3}` and pick using
   a **comb sum** (sum ACF at multiples of the lag) plus the Rayleigh prior. This directly
   attacks the most likely bug (§6.1) with almost no extra memory.

2. **Shift-invariant comb filterbank (proper Davies tempo salience).**
   Replace plain ACF argmax with a comb-filter salience over candidate periods. Memory is
   small (a salience curve over ~130 lags). This is the "correct" Davies front half and
   should markedly improve tempo selection.

3. **Longer ODF history.** Raise `kBuffer` 512 → ~1024 (**~12 s**, +2 KiB × 3 branches) to
   match Going-Zero's 11 s window and improve lag resolution/stability. Add **parabolic
   interpolation** around the ACF/comb peak for sub-lag BPM precision.

4. **Better multi-feature fusion (toward `TempoTapMaxAgreement`).**
   Instead of median of scalars, build a **tempo histogram** from all branches including
   octave candidates and vote; pick the bin with max agreement. More robust than EWMA of one
   scalar.

5. **Add the `beat_emphasis` ODF.** It is the most tempo-relevant of the two dropped ODFs;
   even a streaming approximation should help. `infogain` is lower priority.

6. **Match Essentia onset normalization** for complex/rms/melflux (moving-average window
   sizes, per-ODF scaling) so branch outputs are comparable before fusion.

7. **(Only if beat *phase* is needed)** Add a lightweight beat-phase / dynamic-programming
   tracker to produce ticks like Going-Zero (`pastBeatRelativeSec`,
   `estimatedNextBeatRelativeSec`, off-beat). Not needed if only the BPM number matters.

> Note: fully replicating `TempoTapDegara` (HMM/Viterbi beat tracking) on ESP32 is likely
> overkill; items 1–4 should get the **BPM scalar** close to Going-Zero without it.

---

## 8. Reference pointers

- Going-Zero beat tracker wrapper: `Going Zero/filter/BeatTracker.mm` (uses
  `BeatTrackerMultiFeature`, 11 s rolling window, BPM = 60 / mean of last 8 inter-tick
  intervals). Fed from `BeatTrackerController.m`.
- Essentia: `BeatTrackerMultiFeature`, `OnsetDetection` (complex/rms/melflux),
  `OnsetDetectionGlobal` (beat_emphasis/infogain), `TempoTapDegara`,
  `TempoTapMaxAgreement`. See `src/algorithms/rhythm/` in MTG/essentia.
- Davies & Plumbley, "Context-Dependent Beat Tracking of Musical Audio" (2007) — ACF + comb
  filterbank + Rayleigh weighting (the tempo half we partially implement).
- Degara et al., "Reliability-Informed Beat Tracking of Musical Signals" (2012) — the model
  behind `TempoTapDegara`.
