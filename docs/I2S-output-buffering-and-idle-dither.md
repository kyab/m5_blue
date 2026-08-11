# I2S 出力バッファと Idle Dither

A2DP 一時停止・曲間の完全無音で ES8388 が idle 相当になり、再開時にプチノイズが出る問題への対策メモ。

方針:

- `SUSPENDED` / `STOPPED` でも I2S を止めない（`BluetoothA2DPSinkKeepI2S`）
- I2S へは常に BT PCM または極小ディザを供給
- `i2s.write()` は **I2SWriter タスクだけ**が実行

---

## データ経路

```text
A2DP decode (ESP32-A2DP / BT task)
  -> audio_data_callback
       - digital volume: OFF（AVRCP→ES8388 DAC volume）
       - raw_stream_reader_writer = audio_callback（統計のみ）
       - write_audio → BluetoothA2DPOutputQueuedI2S::write()
  -> BT PCM ring（生PCM、エフェクト未適用）

I2SWriter task（Core1 / prio 2）※唯一の i2s.write()
  -> ring から 256 frame、または idle dither を生成
  -> （BT PCM のときのみ）apply_effects_before_i2s
  -> i2s.write()  ※blocking、DMA消費に同期
```

エフェクトは I2S 直前で適用する。

---

## Bluetooth 受け取り

| 箇所 | 役割 |
|------|------|
| `BluetoothA2DPOutputQueuedI2S::write()` | PCM を ring へ push。I2S 待ちで callback をブロックしない |
| `audio_callback()` | `sample_num` 統計のみ。PCM は触らない |
| `a2dp_audio_state_callback` | `STARTED` 以外で ring clear |
| `set_digital_volume_control(false)` | ソフトゲインなし。音量は `loop()` → `es8388.setDACVolume` |
| `g_force_silent_output` | 切断 / host volume=0 時。writer 側で PCM をゼロ化＋near-silence dither |

Ring 定数:

| 定数 | 値 | 意味 |
|------|-----|------|
| `kI2SWriterFrames` | 256 | ≈5.8 ms @44.1k。1 回の I2S 書き込み単位 |
| `kBtPcmRingBytes` | 32768 | ≈186 ms。ジッタ吸収 |
| `kBtPcmPrebufferBytes` | 24576 | ≈139 ms。drain 開始水位 |
| `kBtPcmPushWaitMs` | 20 | 満杯時の待ち。超えたら古い PCM を drop |

---

## I2SWriter の選択ロジック

```text
state == STARTED:
  未 drain かつ ring >= prebuffer → drain 開始
  drain 中かつ ring >= 1 block → BT PCM
  それ以外 → idle dither（drain 維持。短い underrun でフル prebuffer 待ちに戻らない）
else:
  drain 解除 → idle dither のみ
```

BT PCM ブロック間でサンプル跳躍が大きい場合（`|Δ| >= 28000`）、先頭 48 frame を線形クロスフェードで補修（seek 等のクリック緩和）。

---

## エフェクト（`apply_effects_before_i2s`）

**`from_bt_pcm == true` のときだけ**適用。idle dither ブロックにはかけない。

順序:

1. `g_force_silent_output` → ゼロ化 → near-silence dither → return
2. `g_ring->storeSamples` + Freezer（青ボタン / `g_effect_blue`）
3. DJ Filter（Joystick2 / Rotation。target は control 側、process はここ）
4. near-silence dither（ピーク条件）

Delay（赤）は `#if 0`（`g_ring` を Freezer と共有のため無効）。

制御タスク（エフェクト本体は動かさない）:

| タスク | Core | prio | 内容 |
|--------|------|------|------|
| `I2SWriter` | 1 | 2 | PCM/dither・エフェクト・`i2s.write` |
| `DualBtn`（任意） | 0 | 6 | Freezer ON/OFF → `g_effect_blue` |
| `ModRgbLed` | 0 | 3 | LED I2C（遅延） |
| Arduino `loop` | — | — | AVRCP→DAC volume、DJ target 更新、stats |

---

## ディザ

共通生成: TPDF（`esp_random` の 15bit×2 の差 × `kSilenceDitherLsbScale(=2)` >> 15）。聴感下の ±数 LSB。

### 1. Idle dither（writer が PCM を出さないとき）

- **条件**: `STARTED` 以外、または prebuffer 未達、または drain 中 underrun
- **処理**: `fill_dither_bytes` でブロック全体をディザで埋める（上書き）
- **目的**: pause/stop 中も I2S/DAC を完全ゼロ・停止相当にしない
- この経路ではエフェクト・near-silence 追加はスキップ（既に非ゼロ）

### 2. Near-silence dither（BT PCM 経路の末尾）

- **条件**: ブロック内 `|sample|` ピーク ≤ `kSilenceDitherPeakThreshold`（16）
- **処理**: 既存サンプルに TPDF を加算（clamp）
- **目的**: 曲間・ミュート等の「ほぼゼロ PCM」が続くのを防ぐ
- `g_force_silent_output` 時もゼロ化後に同じ処理を入れる

通常再生中の期待: `writer_dither=0`（idle 側は使われない）。near-silence は stats の `dither` には出ない（PCM ブロック扱い）。

---

## Pause / Resume

```text
再生:  callback → ring → Writer(PCM+FX) → I2S
一時停止: state≠STARTED → ring clear → Writer(idle dither) ※I2Sは継続
再開: ring 蓄積 → prebuffer 到達まで dither → 以降 PCM
```

---

## `audio_stats`（10 秒ごと）

```text
cb=.. samples=.. gap_us=.. ring_now=.. ring=.. drop=.. writer_pcm=.. dither=.. rebuf=.. write_us=.. state=..
```

| 項目 | 見方 |
|------|------|
| `drop` / `dither` / `rebuf` | 通常再生はすべて 0 が望ましい |
| `ring_now` | だいたい 20000–30000 |
| `write_us` avg | ≈5800 µs（256/44100）なら同期正常 |
| `state` | `2`=STARTED、`0`=停止/一時停止相当 |

Serial 負荷自体が稀な音飛び要因になり得る。診断後は頻度を下げること。
