# I2S Output Buffering and Idle Dither

このメモは、A2DP再生時のプチノイズ対策として現在採用している I2S 出力の扱いをまとめたものです。

## 背景

PC や iPhone などのA2DP再生側で一時停止、曲間、停止が起きると、PCMが完全なデジタル無音になったり、A2DPからPCMが供給されなくなったりする。

ES8388 / Module Audio のDAC・アナログ出力経路では、この状態でI2SやDAC内部がアイドル相当になり、再開時または停止後しばらくしてからプチノイズが出ることがあった。

そのため、現在の設計では以下を満たすようにしている。

- A2DPの `SUSPENDED` / `STOPPED` で I2S を停止しない。
- I2Sには常にBluetooth PCM、または極小ディザを供給する。
- 通常再生時はBluetooth PCMを優先し、ディザ混入やPCM dropを避ける。
- I2Sへの直接書き込み元を1つに集約し、複数タスクからの `i2s.write()` 競合を避ける。

## 全体構成

現在のI2S出力は、A2DP callback から直接 `i2s.write()` しない。

代わりに、Bluetooth PCMを一度リングバッファに入れ、専用の `I2SWriter` タスクだけが `i2s.write()` を実行する。

```text
A2DP decode callback
  -> audio_callback()
       - volume effect
       - delay effect
       - DJ filter
       - near-silence TPDF dither
  -> BluetoothA2DPOutputQueuedI2S::write()
  -> Bluetooth PCM ring buffer

I2SWriter task
  -> ring bufferにPCMが十分あればPCM blockを書く
  -> PCMが足りなければdither blockを書く
  -> i2s.write()
```

`i2s.write()` は AudioTools の `I2SStream::write()` 経由で、ESP32 legacy I2S driver の `i2s_write(..., portMAX_DELAY)` を呼ぶ。つまりブロッキングであり、I2S DMA側の消費速度に自然に同期する。

## A2DP Sink の扱い

標準の `BluetoothA2DPSink` は、A2DP audio state が `SUSPENDED` / `STOPPED` になると `set_i2s_active(false)` を呼び、下層で `i2s_stop` / `end()` が走る。

これは停止後のプチノイズ要因になるため、現在は `BluetoothA2DPSinkKeepI2S` を使って `set_i2s_active(false)` を無視している。

```text
STARTED:
  set_i2s_active(true) を通常通り通す

SUSPENDED / STOPPED:
  set_i2s_active(false) は無視する
  I2S writer task は動かし続ける
```

## Bluetooth PCM Ring Buffer

A2DP callbackはI2Sへ直接書かず、処理済みPCMをリングバッファへ入れる。

主なパラメータ:

```cpp
kI2SWriterFrames = 256;
kBtPcmRingBytes = 32768;
kBtPcmPrebufferBytes = 24576;
kBtPcmPushWaitMs = 20;
```

意味:

- `kI2SWriterFrames = 256`
  - 44.1kHzで約5.8ms分。
  - writer taskが1回にI2Sへ書く単位。
- `kBtPcmRingBytes = 32768`
  - stereo 16bitで約186ms分。
  - A2DP callbackのバーストやジッタを吸収するためのキュー。
- `kBtPcmPrebufferBytes = 24576`
  - stereo 16bitで約139ms分。
  - 再生開始・再開時にこの量までPCMが溜まってからBluetooth PCMのdrainを始める。
- `kBtPcmPushWaitMs = 20`
  - ring bufferが満杯に近い時、即dropせず最大20msだけ空きを待つ。
  - それでも空かなければ、遅延を無限に増やさないため古いPCMをdropする。

## I2SWriter Task

`I2SWriter` はI2S出力の唯一のproducerである。

処理の考え方:

```text
if A2DP state == STARTED:
    if PCM ring buffer >= prebuffer:
        PCM drain開始

    if drain中 かつ PCM ring buffer >= 1 writer block:
        PCM block を i2s.write()
    else:
        dither block を i2s.write()
else:
    dither block を i2s.write()
```

一度PCM drainに入った後は、短いunderrunで毎回フルプリバッファ待ちへ戻らない。PCMが1 writer block分戻ればすぐPCM出力に復帰する。

これにより、通常再生中にA2DP callback間隔が一瞬伸びた場合でも、大きな無音ギャップを作らないようにしている。

## Dither の役割

ディザは2箇所で使う。

### 1. callback内のnear-silence dither

A2DPからPCMが来ているが、block全体がほぼ無音の場合に、完全ゼロPCMを避けるため極小TPDFディザを足す。

目的:

- 曲間やアプリ側無音で完全ゼロPCMが続くことを避ける。
- DAC側のzero detect / idle相当の状態遷移を起こしにくくする。

### 2. I2SWriter側のidle dither

A2DPが停止中、またはPCM ring bufferが枯渇した時に、I2Sへ出すための代替信号として使う。

目的:

- A2DP callbackが呼ばれない `SUSPENDED` / `STOPPED` 中も、I2S/DAC経路を完全無音・停止相当にしない。
- I2S writer taskを止めず、DACに継続的に極小の非ゼロPCMを供給する。

## Pause / Resume 時の動作

### 再生中

```text
A2DP callback -> PCM ring buffer -> I2SWriter -> I2S
```

通常状態では、診断ログ上以下が期待値。

```text
drop=0/0B
writer_dither=0
rebuf=0
ring_now は概ね 20000 bytes 以上
```

### 一時停止時

```text
A2DP state -> SUSPENDED / STOPPED
PCM ring buffer clear
I2SWriter -> dither only
I2S itself is not stopped
```

PCM ring bufferをclearする理由は、再開時に古いPCMが遅れて出るのを避けるため。

### 再開時

```text
A2DP callback再開
PCM ring bufferへ蓄積
prebuffer到達までI2SWriterはdither継続
prebuffer到達後、PCM outputへ切り替え
```

現在のprebufferは約139msなので、再開直後にわずかな遅れは入るが、再開後のring buffer水位を高く保ち、数十ms級のA2DP callback gapを吸収しやすくしている。

## Diagnostic Log

1秒ごとに `audio_stats` ログを出す。

例:

```text
[audio_stats] cb=44 samples=1024..1024 gap_us=8200..56000 ring_now=26624 ring=22528..30720 drop=0/0B writer_pcm=174 dither=0 rebuf=0 write_us=10/5795/11620 state=2
```

主な見方:

- `cb`
  - 1秒間の `audio_callback()` 呼び出し回数。
- `samples`
  - callbackごとのframe数。現在は概ね `1024..1024` で一定。
- `gap_us`
  - callback間隔のmin/max。
  - maxが大きいほどA2DP供給がバースト的。
- `ring_now`
  - ログ出力時点のPCM ring buffer使用量。
- `ring`
  - 1秒間のring使用量min/max。
- `drop`
  - ring overflowで捨てたPCM数。
  - 通常再生中は `0/0B` が望ましい。
- `writer_pcm`
  - I2SWriterがPCM blockを書いた回数。
- `dither`
  - I2SWriterがdither blockを書いた回数。
  - 通常再生中は `0` が望ましい。
- `rebuf`
  - PCM drain中にPCMが不足した回数。
  - 通常再生中は `0` が望ましい。
- `write_us`
  - `i2s.write()` の min/avg/max。
  - avgが約5800us前後なら、256 frames / 44.1kHz の消費時間と一致しており正常。
- `state`
  - A2DP audio state。
  - ログ上は `state=2` が再生中、`state=0` が停止/一時停止相当として観測されている。

## 現在の安定状態

改善後の通常再生では、概ね以下の状態が観測されている。

```text
drop=0/0B
dither=0
rebuf=0
ring_now=20000..30000
write_us avg ~= 5795us
```

この状態では、I2S writerはI2Sクロックに同期してPCMを安定供給できていると見てよい。

## 注意点

`audio_stats` や `audio_callback()` のログ自体もSerial出力負荷になる。長時間運用でまだ稀な音飛びがある場合、診断が終わった段階でログ頻度を下げる、または不要なログを削除することを検討する。

