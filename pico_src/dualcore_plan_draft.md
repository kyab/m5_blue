# 質問

Core 0はBTStackの処理（SBCデコードも含めて）、およびLEDとして、Core1をjoystick2の処理およびDSPとその後段にするような構成は可能？

そのばあいBTStackの処理からDSPにはどう渡すべきか？

# 回答

可能です。ただし **SBC デコードは Core 0 に残し、I2S（DSP の後段）は Core 1 に移す**のが前提になります。いまのパイプラインをそのまま両コアから触る構成は危険です。

## いまの流れ（全部 Core 0）

```
A2DP受信 → sbc_frame_ring_buffer
                ↓  5ms BTstack timer
         playback_handler（SBCデコード＋リサンプル）
                ↓
         Pico Audio producer (512 × 3)
                ↓
         I2S consumer (256 × 2) → DMA IRQ → DAC
```

`playback_handler` のコメントどおり、デコードは **BTstack のメインスレッド前提**です。この関数を Core 1 から呼ぶのはやめた方がよいです。

## 推奨する分割

| Core 0 | Core 1 |
|---|---|
| BTstack / CYW43 / LED | Joystick2（I2C） |
| SBC デコード・リサンプル | DSP |
| AVRCP volume を atomic で渡す | I2S（`take` → DSP → `give`、DMA IRQ） |

Joystick は 100 ms で安いので、Core 1 の主役は **音声の空きバッファを埋めるループ**にして、Joystick はその合間に読む、が安全です。DSP を Joystick の 100 ms ループに入れない方がよいです。

## BTstack → DSP の渡し方

**渡すものは PCM ブロックだけ**です。BTstack の `sbc_frame_ring_buffer` / `decoded_audio_ring_buffer` は Core 0 専有のままにします。

いちばん自然な切り口は、今 `btstack_audio_pico_sink_fill_buffers()` が I2S 用バッファに書いている **512 stereo frames** を、I2S に `give` せず Core 1 向けキューに載せることです。

```
Core 0                              Core 1
playback_handler → PCM 512frm
       ↓
  slot に memcpy
       ↓
  queue_t で「準備できた」通知  →  pop
                                    DSP
                                    I2S producer take/give
                                    DMA IRQ が consumer を回収
```

具体的には Pico SDK の `queue_t` + **固定スロット数（3〜4）** が向いています。

- 実体は静的配列 `int16_t slots[N][512 * 2]`
- キューに載せるのは **スロット番号**（ポインタでも可）
- Core 0: 空きスロットに書いて push
- Core 1: pop → DSP → I2S に出す → スロットを返却キューへ

FIFO（`multicore_fifo_*`）は起動・lockout と衝突するので使わない方がよいです。

### なぜリングバッファ共有よりブロックキューか

- 512 フレームはまとめて一貫している（途中の `x` だけ新しい、が起きない）
- 44.1 kHz なら 1 ブロック ≈ 11.6 ms。3 段で数十 ms の余裕
- 書き手 1 / 読み手 1 なので所有権が明確
- アンダーラン時は Core 1 が無音を I2S に出せばよい（いまの DMA 側も無音フォールバックあり）

`std::atomic` の最新値上書き（latest-wins）は Joystick 向きで、音声には向きません。取りこぼすとプチノイズになります。

## Pico 2 で特に必要なこと

1. **I2S の DMA IRQ は有効化したコアで動く**  
   `audio_i2s_set_enabled(true)` は Core 1 で呼ぶ。セットアップも Core 1 所有がきれいです。
2. **Pico Audio の `take` / `give` は Core 1 + そのコアの DMA IRQ だけ**  
   Core 0 は触らない。pico-extras 側にも threading の TODO があり、コアをまたぐ利用は想定どおりではないです。
3. **volume** は Core 0 の AVRCP から `std::atomic<uint8_t>` で渡し、適用は Core 1（DSP 前でも後でも可）。
4. **start/stop** もメッセージで渡す（再生開始・停止・アンダーラン用の無音）。BTstack の `start_stream` / `stop_stream` から Core 1 の I2S を直接叩かない。

## やらない方がよい案

- Core 1 で `playback_handler` を呼ぶ（SBC 状態と BTstack リングを共有してしまう）
- DSP だけ Core 1、I2S は Core 0 のまま（PCM が往復して遅延とコピーが増える）
- サンプル単位の共有変数

---

この切り方で進めるなら、次の実装は「Core 0 の 5 ms fill をキュー push に変え、Core 1 で I2S を駆動」が最小差分です。ブロック長を 512 のままにするか、I2S consumer に合わせて 256 にするかは好みです。512 維持の方が今のデコード単位と一致します。

実装に進めてよければ、その方針で切ってよいです。



回答 by : Cursor Grok 4.6 High Fast.