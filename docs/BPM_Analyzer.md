# BPM アナライザ（ESP32 移植版）— 設計・Going-Zero との差分・デバッグ向けブリーフ

> **このドキュメントの目的**
> このファイルは *別の* デバッグ用チャットで AI コーディングエージェントに読み込ませることを想定している。
> 現在、実機ディスプレイに表示される BPM 値は参照アプリ（Going-Zero）と明らかに異なる。
> ツリー全体を読み直さなくてもデバッグできるよう、以下を十分な粒度で記載する:
> 1. 参照実装（Going-Zero）が実際に何をしているか。
> 2. 本ブランチが何を実装したか、アルゴリズムごとに、そして **どの部分を省略・簡略化したか**
>    （移植がどの程度「縮退」しているか）。
> 3. 解析に使う **バッファ**（型 / サイズ / 秒数）と、Going-Zero との違い
>    （例: RAM 制約で履歴秒数が短い、など）。
> 4. **なぜ BPM がずれるのか** の具体的な仮説と、次に移植すべきものの優先順位付きリスト。
>
> あなたがそのデバッグ用エージェントなら: **§6（BPM がずれる原因の仮説）** と
> **§4（バッファ比較）** から読み始めること。参照アルゴリズムは Essentia の
> `BeatTrackerMultiFeature`。`AGENTS.md` の方針に従い、迷ったら Going-Zero のソースを
> GitHub で直接参照すること
> （`https://github.com/kyab/Going-Zero`、ファイル `Going Zero/filter/BeatTracker.mm`）。

---

## 1. ファイル構成（本ブランチ）

BPM 関連コードはすべて `src/bpm/` に隔離されており、音声出力パスからは独立している。

| ファイル | 役割 |
|------|------|
| `src/bpm/bpm_config.hpp` | コンパイル時定数（サンプルレート、FFT/ホップ、メルバンド数、テンポ範囲、バッファサイズ）。 |
| `src/bpm/mel_filterbank.{hpp,cpp}` | 40 バンドの HTK メルフィルタバンク、0–4000 Hz（melflux ODF 用）。 |
| `src/bpm/stft_odf_pipeline.{hpp,cpp}` | モノラルリング → Hann STFT（arduinoFFT）→ 振幅/位相 → **3 つのオンセット検出関数**（complex, rms, melflux）。 |
| `src/bpm/davies_tempo_estimator.{hpp,cpp}` | ODF ごとの適応しきい値 + **Rayleigh 重み付き自己相関**によるテンポ推定（`DaviesStyleTempoEstimator`）。`EssentiaStyleOdfBranch` がラップ。 |
| `src/bpm/streaming_bpm_analyzer.{hpp,cpp}` | ファサード: PCM FIFO、ワーカータスク、STFT/ODF/テンポ呼び出し、**3 値のメディアン + EWMA 融合**、`bpm()` を公開。 |
| `src/main.cpp` | `M5.begin()` 後にアナライザを生成、`audio_callback` から PCM を供給、ワーカータスク起動、画面に BPM 描画。 |

`src/main.cpp` の連携ポイント:
- `static bpm::StreamingBpmAnalyzer* g_streaming_bpm = nullptr;`（ヒープ確保、`setup()` 内で `M5.begin()` 後に生成）。
- `audio_callback()` → `g_streaming_bpm->enqueueStereoInterleaved(data, sample_num)`。
- `a2dp_audio_state_callback()` → 停止時に `g_streaming_bpm->reset()`。
- `bpm::start_bpm_worker_task(g_streaming_bpm)`（ワーカーは **core 1** にピン留め）。
- `loop()` で `g_streaming_bpm->bpm()` を約 5 Hz で描画。

データフロー:

```
BT A2DP PCM（int16 ステレオ, 44.1 kHz）
  → enqueueStereoInterleaved  → 200 KiB PSRAM バイト FIFO
  → [ワーカータスク, core 1] service()
      → mono = 0.5*(L+R)/32768         （float, -1..1）
      → StftOdfPipeline.pushMonoFloat  （4096 モノラルリング, 2048/1024 Hann STFT）
          → complex_odf, rms_odf, melflux_odf   （1024 サンプルホップごとに 1 トリプレット）
      → 3× EssentiaStyleOdfBranch.pushStftHop(odf)
          → x2 線形アップサンプル → 適応しきい値 → Davies ACF テンポ
      → fuseMedianEwma()  → display_bpm_
  → loop() が display_bpm_ を描画
```

---

## 2. 参照実装: Going-Zero が実際にやっていること

Going-Zero の `BeatTracker.mm` は、macOS 上（デスクトップ CPU、`float`/`Real`）で動く
**Essentia のストリーミング `BeatTrackerMultiFeature`** アルゴリズムの薄いラッパである:

```objc
_intBeatTracker = factory.create("BeatTrackerMultiFeature");
_vecInput >> _intBeatTracker->input("signal");
_intBeatTracker->output("ticks") >> PC(_pool, "rhythm.ticks");
```

参照実装の主な挙動（`BeatTracker.mm` より）:

- **入力**: モノラル `(L+R)/2`、`float`、44.1 kHz。
- **時間方向のウィンドウ処理**（ファイル全体ではなくローリング解析）:
  - ウォームアップ: 最初のディスパッチは **3 秒**、次に **5 秒**。
  - 定常状態: バッファが **11 秒** に達したら処理
    （`_audioFragment.size() >= 11 * 44100`）。各回の **[5 秒, 10 秒]** スライスの tick のみ採用。
    その後 **5 秒分だけ左シフト**し、**6 秒のオーバーラップ**を保持
    （`shift_left(...,44100*5)` + `erase(begin+44100*6, end)`）、続いて新たにバッファした音声を追記。
    つまり **11 秒解析ウィンドウ・5 秒ホップ・6 秒オーバーラップ**。
  - `_audioFragment.reserve(44100*20)` → 最大 **20 秒** のモノラル float を予約。
- **出力**: **ビートの tick 位置**（`rhythm.ticks`、秒）。テンポのスカラ値ではない。
- **BPM の算出**（`-(void)update`）: `beatDuration = 直近 8 個の tick 間隔の平均`、
  そこから `BPM = 60 / beatDuration`。さらに off-beat / 位相の処理も行う
  （`pastBeatRelativeSec`, `estimatedNextBeatRelativeSec`, `flipOffBeat`）。

### `BeatTrackerMultiFeature` の内部（Essentia）

`BeatTrackerMultiFeature` は次の手順でビートを推定する:

1. **5 つのオンセット検出関数（ODF）** を計算し、それぞれから `TempoTapDegara` でビートを推定、
   最も一致度の高い推定を `TempoTapMaxAgreement` で選択する。5 つの ODF は:
   - **complex** スペクトル差分（`OnsetDetection` の `complex` 手法）
   - **energy flux**（`OnsetDetection` の `rms` / energy 手法）
   - **メルバンドのスペクトルフラックス**（`OnsetDetection` の `melflux` 手法）
   - **beat emphasis**（`OnsetDetectionGlobal` の `beat_emphasis` 手法）
   - **modified information gain**（`OnsetDetectionGlobal` の `infogain` 手法）
2. `TempoTapDegara` = **Degara et al. (2012)** の確率的ビートトラッキング。ODF をビートの *tick* に
   変換する。周期の顕著性を求める段（レゾネータ / コムフィルタ的）と、ビート周期 *と位相* を
   **隠れマルコフ / Viterbi** で復号する段からなる。入力 ODF レートは `44100/512 ≈ 86.13 Hz`。
3. `TempoTapMaxAgreement` は、ODF ごとの tick 系列のうち相互一致度が最大のものを選ぶ
   （特徴量間の頑健な投票）。

つまり参照実装は、単なるテンポ推定器ではなく **位相まで持つ本格的なマルチ特徴ビートトラッカ** である。

---

## 3. 本ブランチの移植内容 vs 省略内容（アルゴリズム別）

| 参照段（Essentia `BeatTrackerMultiFeature`） | 本ブランチ | 状態 |
|---|---|---|
| 入力: モノラル `(L+R)/2`, float, 44.1 kHz | `0.5*(L+R)/32768` float | **移植**（等価）。 |
| STFT フレーム/ホップ | 2048 / 1024, Hann（対称 `N-1`） | **移植**（complex/melflux パスのフレーム/ホップに一致）。 |
| ODF: **complex** スペクトル差分 | `onsetComplex()`（目標位相予測、`|pred-meas|`） | **移植**（正規化は簡略化）。 |
| ODF: **energy/rms flux** | `onsetRms()`（`sqrt(Σmag²)/N`、半波差分） | **移植**（スケール/正規化は Essentia と不一致）。 |
| ODF: **melflux**（40 バンド, 0–4000 Hz, dB） | `onsetMelFlux()` + `MelFilterbank`（HTK）、`amp2dbEssentia` | **移植**（メルワープ/境界はデフォルトに一致）。 |
| ODF: **beat_emphasis**（`OnsetDetectionGlobal`） | — | **省略**（グローバル/全信号処理のためストリーミング困難）。 |
| ODF: **infogain**（`OnsetDetectionGlobal`） | — | **省略**（グローバル/全信号処理）。 |
| ODF → テンポ段用に 2× アップサンプル（~86.13 Hz） | `pushStftHop()` 線形 x2（`0.5*(prev+cur)`, `cur`） | **移植**（レートは TempoTapDegara 入力に一致）。 |
| オンセット後処理（Essentia の正規化 / 移動平均） | `OdfAdaptiveThreshold`（17 タップ移動平均減算 + 半波整流） | **近似**（ウィンドウ/パラメータは未一致）。 |
| テンポ/ビート推定: **TempoTapDegara**（コム/レゾネータ + HMM/Viterbi、tick + 位相） | `DaviesStyleTempoEstimator`: **Rayleigh 重み付き素の自己相関**、lag の argmax → BPM | **大幅簡略化** — テンポスカラのみ、**コムフィルタバンクなし、HMM/Viterbi なし、ビート位相/tick なし**。 |
| マルチ特徴選択: **TempoTapMaxAgreement**（tick 系列の一致度） | `fuseMedianEwma()`: ≤3 個の瞬時 BPM のメディアン + EWMA（`0.88/0.12`） | **大幅簡略化** — スカラのメディアン、系列の一致度なし。 |
| BPM = `60 / mean(直近 8 tick 間隔)` | `60*sr_odf/best_lag`、40–208 にクランプ | **算出方法が異なる**（トラッキング済みビートではなく lag から算出）。 |
| ビート位相 / off-beat / 次ビート予測 | — | **省略**（テンポのみ、位相なし）。 |

### 縮退の要約（1 段落）

本移植は **フロントエンド** は Essentia に近い（モノラルミックス、2048/1024 Hann STFT、
5 つ中 3 つのオンセット検出関数: complex, rms, melflux）。しかし **ODF の後段はすべて、
はるかに単純なテンポ推定器に置き換えられている**。`TempoTapDegara`（レゾネータ/コム +
Viterbi による *tick* と *位相* を生成するビートトラッキング）の代わりに、短い ODF 履歴に対する
**Rayleigh 重み付き自己相関の argmax** を単発で用いる。`TempoTapMaxAgreement` の代わりに
3 つの ODF ごとの瞬時テンポの **メディアン + EWMA** を用いる。出力は **テンポスカラのみ** で、
ビートトラッキングも位相も off-beat もなく、**オクターブ/ハーモニクスの曖昧性解消もない**。
5 つ中 2 つの ODF（`beat_emphasis`, `infogain`）は省略。これが表示 BPM が Going-Zero と
乖離する最有力の理由である（§6 参照）。

---

## 4. 解析に使うバッファ: 型 / サイズ / 秒数、および Going-Zero との違い

レート: STFT ホップレート = `44100/1024 = 43.07 Hz`。ODF テンポ段レート（x2）=
`86.13 Hz`（Essentia `TempoTapDegara` の入力 `44100/512` に一致）。

### 4a. 本ブランチ

| バッファ | 場所 | 型 | 長さ | バイト数 | 時間長 | 備考 |
|---|---|---|---|---|---|---|
| PCM 転送 FIFO（`fifo_buf_`） | PSRAM | `uint8_t`（int16 ステレオ） | `200 KiB` = 51,200 フレーム | 204,800 | **~1.16 s** | BT→ワーカーの転送キューのみ。解析履歴では**ない**。溢れたら古い順に破棄。 |
| モノラル STFT リング（`ring_`） | PSRAM | `float` | `kMonoRingSize = 4096` | 16,384 | **~92.8 ms** | 2048 点 FFT を 1024 ホップでフレーム化するのに必要な最小限。 |
| Hann 窓（`hann_`） | PSRAM | `float` | 2048 | 8,192 | — | |
| FFT 実部/虚部（`fft_real_`, `fft_imag_`） | PSRAM | **`double`** | 各 2048 | 各 16,384 | — | **arduinoFFT は `double`**。Essentia は `float`。 |
| 振幅/位相（`mag_`,`phase_`,`phase_1_`,`phase_2_`,`spectrum_1_`） | PSRAM | `float` | 各 `kSpectrumBins = 1025` | 各 4,100 | — | complex ODF の位相履歴用。 |
| メルスクラッチ（`mel_linear_`,`prev_mel_db_`） | PSRAM | `float` | 各 `kMelBands = 40` | 各 160 | — | |
| ODF 履歴（ブランチごと `odf_ring_`） | アナライザヒープ（内部 DRAM） | `float` | `kBuffer = 512` | 2,048 | **~5.94 s** | **これが実際のテンポ解析ウィンドウ。** 推定は `ring_count_ >= 256`（**~3.0 s**）で開始。 |
| Rayleigh 重み（`rayleigh_weight_`） | アナライザヒープ | `float` | 256 | 1,024 | — | 120 BPM の lag 付近にピーク。 |
| 適応しきい値リング（`buf_`） | アナライザヒープ | `float` | `kWin = 17` | 68 | ~0.2 s | 移動平均ベースライン。 |

ODF ブランチは **3 つ**（complex, rms, melflux）あるため、テンポ段の配列は ×3。

**過去音声の実効解析メモリ ≈ 5.94 s**。ただし生音声ではなく **ダウンサンプルした ODF**
（86.13 Hz）として保持する。

### 4b. Going-Zero

| バッファ | 型 | 長さ | 時間長 | 備考 |
|---|---|---|---|---|
| `_audioFragment`（reserve） | `float`/`Real` モノラル | `44100*20` | 最大 **20 s** 予約 | 生音声、フル精度。 |
| 定常状態の解析ウィンドウ | `float` | `11 * 44100` | 1 回あたり **11 s** 処理 | 5 s ホップ、6 s オーバーラップ。 |
| 1 回で保持する tick スライス | 秒 | **[5 s, 10 s]** の tick | — | |
| 非同期処理中の溢れ | `_audioPool` | 無制限（`push_back`） | — | バックグラウンド実行中の入力を蓄積。 |

### 4c. 主なバッファの違い

- **履歴長**: 参照実装のテンポ/ビートトラッキングは **11 s の生音声**（20 s 予約）を見る。
  本ブランチは **~5.9 s の ODF**（最小 3 s）— おおよそ **時間的コンテキストが半分**、しかも
  ダウンサンプル済み。履歴が短い → 自己相関の lag 分解能が粗く、平均できるビート周期数も少ない。
- **lag 量子化**: 86.13 Hz では 120 BPM の lag ≈ 43 サンプル。lag が ±1 サンプルずれると
  **120 BPM 付近で ±約 3 BPM**、高テンポではさらに悪化（整数 lag の argmax で補間なし）。
- **保持する内容**: 参照実装は **生音声** を保持しパイプライン全体を再実行する。本ブランチは
  **整流済み ODF リング** のみを保持 — ODF で失われたスペクトル詳細は復元できない。
- **数値型**: ここでは FFT が **`double`**（arduinoFFT）、Essentia は **`float`** — 正しさの
  問題ではないが、メモリ/CPU コストの注記。
- **転送 FIFO**（200 KiB, ~1.16 s）は解析スパンとは無関係。ワーカーが滞留すると古い PCM を
  破棄する（解析にギャップが生じる）。一方 Going-Zero は破棄せず `_audioPool` に蓄積する。

### 4d. 設定定数（`src/bpm/bpm_config.hpp`）

```
kSampleRate   = 44100
kFftSize      = 2048     kHopSize   = 1024     kSpectrumBins = 1025
kMelBands     = 40       kMelLowHz  = 0        kMelHighHz    = 4000
kMinTempo     = 40       kMaxTempo  = 208
kMonoRingSize = 4096     kMonoRingMask = 4095
odfSampleRateBase = 44100/1024 = 43.07 Hz     odfSampleRateX2 = 86.13 Hz
```
テンポ推定器（`davies_tempo_estimator.hpp`）: `kBuffer = 512`、`rayleigh_weight_[256]`、
`OdfAdaptiveThreshold::kWin = 17`。融合（`streaming_bpm_analyzer.cpp`）: EWMA
`display_bpm_ = 0.88*display_bpm_ + 0.12*fused`。

---

## 5. スレッド / ランタイムの注記（バグ再現のコンテキスト）

- ワーカー `bpm_work` は **core 1** にピン留め（スタック 4096 ワード）。core 0 の BT
  （`BTC_TASK`）/ IDLE0 とは分離。これは、再生開始から約 1 分後にクラッシュしていた
  **Task WDT abort**（`IDLE0 (CPU 0)` がスタベーション）を止めるために必要だった。
- `service()` は 1 回あたり最大 `kServiceMonoSampleBudget = 2048` モノラルサンプルだけ処理し、
  `vTaskDelay(1 ms)` する。これにより CPU を独占しない。
- 大きな STFT バッファと FIFO はすべて **PSRAM 専用**（内部 DRAM フォールバックなし）。
  起動時の DRAM 枯渇による `esp_startup_start_app_common` アサートを避けるため。
- アナライザオブジェクトは静的グローバルではなく **`M5.begin()` 後に `new`** で生成。
  大きな確保の前に PSRAM が初期化されているようにするため。
- ボタンエフェクト（Freezer / delay）は現在 `main.cpp` の `kButtonEffectsEnabled = false`
  で **無効化**（BPM とは無関係だが、`g_ring` が確保されないことを意味する）。

---

## 6. BPM がずれる原因（デバッグ仮説、確度順）

ここから見ること。確度が高い順。

1. **素の自己相関によるオクターブ / ハーモニクス誤り（最有力）。**
   `estimateBpmFromBuffer()` は **Rayleigh 重み付き ACF の argmax** を取る。ACF は真の周期に加えて
   *その整数倍/整数分の一* にもピークを持つため、グローバル最大が **半分または倍** のテンポに
   容易に落ちる（例: 真値 150/75 に対して 75 や 150 を表示）。**オクターブの曖昧性解消がない。**
   Essentia の `TempoTapDegara` はレゾネータコム + HMM でこれを回避する。→ 表示値が真の BPM の
   約 ½ か約 2× になっていないか確認すること。

2. **コムフィルタバンクがない（Davies が半分しか実装されていない）。**
   本来の Davies (2007) のテンポ段は ACF に対する **シフト不変コムフィルタバンク** を使い、
   ある周期 *とそのハーモニクスをまとめて* 強調する。素の ACF はそれをしない。どの lag が
   勝つかが変わる。

3. **Rayleigh 重みが 120 BPM 中心のためバイアスがかかる。**
   `tau120 = 60*sr_odf/120`。重みは 120 BPM でピーク。中心から外れたテンポはペナルティを受け、
   推定が 120 に引っ張られ、仮説 (1) と悪く相互作用する。

4. **粗い lag 分解能 + 短いウィンドウ**（§4c 参照）。補間なしの整数 lag argmax は量子化された
   BPM になる（120 付近で ±約 3 BPM、高い側でさらに悪化）。~5.9 s の ODF ウィンドウ（vs 11 s）も、
   安定化に使えるビート周期数を減らす。

5. **5 つ中 3 つの ODF しかなく、融合が弱い。** `beat_emphasis`（まさにテンポ志向）と `infogain`
   が欠落。`fuseMedianEwma()` は *瞬時スカラテンポ* のメディアンで、tick 系列に対する
   `TempoTapMaxAgreement` よりはるかに弱い。1 つのブランチがハーモニクスに固着すると
   メディアンを引きずる。

6. **オンセット後処理の不一致。** `OdfAdaptiveThreshold`（17 タップ MA 減算 + 整流）は Essentia の
   オンセット正規化ではない。ODF の形が違うと ACF のピーク位置が変わる。また `onsetRms()` は
   エネルギーを `N`（1025）で割るため非常に小さな値になる — 融合前に他の ODF とのスケールを
   確認すること。

7. **Hann 窓の規約。** 対称 `0.5*(1-cos(2π n/(N-1)))` vs Essentia の正規化された周期 Hann —
   スペクトル漏れの差はわずかで、主要因の可能性は低いが、除外はしておくこと。

**推奨検証手順:** 同一音源を Going-Zero（`BeatTrackerMultiFeature`、モノラル 44.1 kHz）と本機の
両方に入力し、ブランチごとの `instantBpm()`（complex/rms/melflux）と融合値をログ出力する。
ブランチごとの値が参照値の ½/2× に集まっていれば、仮説 (1)–(3) が確定する。

---

## 7. 次に移植すべきもの（優先順位付きの提案）

ESP32 上での「労力/メモリあたりの精度向上の期待値」順。

1. **現行 ACF の上にオクターブ/ハーモニクス補正（安価・高インパクト）。**
   `best_lag` を見つけた後、候補テンポ `{T, 2T, T/2, 3T, T/3}` を評価し、**コム和**（lag の整数倍で
   ACF を合計）+ Rayleigh 事前分布で選ぶ。追加メモリはほぼゼロで、最有力バグ（§6.1）を直接叩く。

2. **シフト不変コムフィルタバンク（本来の Davies テンポ顕著性）。**
   素の ACF argmax を、候補周期に対するコムフィルタ顕著性に置き換える。メモリは小さい
   （~130 lag の顕著性カーブ）。これは Davies の前半分の「正しい」実装で、テンポ選択が大きく改善する。

3. **ODF 履歴を長く。** `kBuffer` を 512 → ~1024（**~12 s**、ブランチ ×3 で +2 KiB）にして
   Going-Zero の 11 s ウィンドウに合わせ、lag 分解能/安定性を改善。ACF/コムのピーク周辺に
   **放物線補間** を追加してサブ lag の BPM 精度を得る。

4. **より良いマルチ特徴融合（`TempoTapMaxAgreement` に近づける）。**
   スカラのメディアンの代わりに、オクターブ候補を含む全ブランチから **テンポヒストグラム** を作って
   投票し、一致度最大のビンを選ぶ。単一スカラの EWMA より頑健。

5. **`beat_emphasis` ODF を追加。** 省略した 2 つの ODF のうち最もテンポに関連する。
   ストリーミング近似でも効果があるはず。`infogain` は優先度低。

6. **Essentia に合わせたオンセット正規化** を complex/rms/melflux に対して行う（移動平均のウィンドウ
   サイズ、ODF ごとのスケール）。融合前にブランチ出力を比較可能にするため。

7. **（ビート *位相* が必要な場合のみ）** Going-Zero のような tick を生成する軽量なビート位相 /
   動的計画法トラッカを追加（`pastBeatRelativeSec`, `estimatedNextBeatRelativeSec`, off-beat）。
   BPM の数値だけが必要なら不要。

> 注: ESP32 上で `TempoTapDegara`（HMM/Viterbi ビートトラッキング）を完全再現するのは過剰と思われる。
> 項目 1–4 で、それなしでも **BPM スカラ** を Going-Zero に近づけられるはず。

---

## 8. 参照ポインタ

- Going-Zero のビートトラッカラッパ: `Going Zero/filter/BeatTracker.mm`
  （`BeatTrackerMultiFeature` を使用、11 s ローリングウィンドウ、BPM = 60 / 直近 8 tick 間隔の平均）。
  `BeatTrackerController.m` から供給される。
- Essentia: `BeatTrackerMultiFeature`, `OnsetDetection`（complex/rms/melflux）,
  `OnsetDetectionGlobal`（beat_emphasis/infogain）, `TempoTapDegara`,
  `TempoTapMaxAgreement`。MTG/essentia の `src/algorithms/rhythm/` 参照。
- Davies & Plumbley, "Context-Dependent Beat Tracking of Musical Audio" (2007) — ACF +
  コムフィルタバンク + Rayleigh 重み付け（我々が部分的に実装しているテンポの半分）。
- Degara et al., "Reliability-Informed Beat Tracking of Musical Signals" (2012) —
  `TempoTapDegara` の背景モデル。
