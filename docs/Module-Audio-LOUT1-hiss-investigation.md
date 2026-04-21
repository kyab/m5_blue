# Module Audio (ES8388) 左チャンネルヒスノイズ調査結果

> This document is intentionally written in Japanese because the investigation
> was driven interactively in Japanese and the conclusions are specific to the
> particular Module Audio hardware in use.

## TL;DR

**左(LOUT1)チャンネルから再生時にだけヒスノイズが出る**。原因はソフトウェア(ES8388 レジスタ設定・I2S 送出)ではなく、**Module Audio 基板の LOUT1 出力経路側のハードウェア非対称性**である。DAC コア、I2S 経路、コーデックレジスタは L/R 完全対称に設定されているにもかかわらず、LOUT1 だけが駆動されるとヒスが出力される。

## 症状

- 再生環境: M5Stack Core2 + Module Audio (ES8388) + M5GO Bottom2 + TRRS 有線ヘッドホン
- **左耳からのみ「サー」という連続ヒスノイズが聞こえる**
- 右耳からはヒスは聞こえない
- PCM 入力が両チャンネル厳密に `0x0000` のときはヒスも完全無音
- PCM 入力がどこか(L/R いずれか)で 1 LSB でも非ゼロになると L ヒス復活
- USB ケーブル給電時 vs M5GO Bottom2 バッテリー駆動時ではヒス量がわずかに異なる(USB の方がやや大きい)が、根本差異ではない

## 調査手法

調査用ファームは `src/main_noise_test.cpp` (PlatformIO env `m5stack-core2-noise-test`) として実装。

### 測定条件

- BLE / A2DP コントローラと Wi-Fi は起動時に `esp_bt_controller_disable` / `esp_wifi_stop/deinit` で完全停止し、RF 由来のノイズ混入を排除
- M5Unified 内蔵スピーカ (`M5.Speaker`) も `end()` で切離し、I2S0 を Module Audio が単独占有
- ES8388 DAC ミュート(`DACCONTROL3` の `DACMute`)は **意図的に解除したまま**(= 出力段を生かしたまま観測)
- 44.1 kHz / 16 bit / stereo で AudioTools `I2SStream` 経由の送出のみ
- 1 サイクルあたり 7 区間を 2 秒ずつ繰り返し再生

### セグメント構成

| # | 名前 | L ch PCM | R ch PCM | DACPOWER | 目的 |
|---|---|---|---|---|---|
| 1 | `SINE_LR`   | 1 kHz サイン | 1 kHz サイン | `0x30` | 基準 |
| 2 | `ZERO`      | 0 | 0 | `0x30` | PCM 完全ゼロの無音基準 |
| 3 | `L_ONLY`    | 1 kHz サイン | 0 | `0x30` | L デジタル入力のみ駆動 |
| 4 | `R_ONLY`    | 0 | 1 kHz サイン | `0x30` | R デジタル入力のみ駆動 |
| 5 | `DITHER_LR` | ±1 LSB ディザ | ±1 LSB ディザ | `0x30` | 非ゼロ最小信号 |
| 6 | `PWR_0x20`  | 1 kHz サイン | 1 kHz サイン | `0x20` | codec で片側 HP 出力ピンを物理 OFF |
| 7 | `PWR_0x10`  | 1 kHz サイン | 1 kHz サイン | `0x10` | 逆側 HP 出力ピンを物理 OFF |

### レジスタダンプ

各区間の開始時(および `ZERO` の中間地点)に ES8388 の主要 26 レジスタを I2C で読み戻し、シリアルにビット単位で出力。対象:

- 電源系: `CHIPPOWER`, `ADCPOWER`, `DACPOWER`
- DAC 制御: `DACCONTROL3(mute/SoftRamp)`, `DACCONTROL4/5(L/R digital vol)`
- ミキサ: `DACCONTROL16/17(LD2LO)`, `DACCONTROL20(RD2RO)`
- ALC / Zero-cross 系: `DACCONTROL10`, `DACCONTROL18/19`
- 出力アンプ音量: `DACCONTROL24/25/26/27` (LOUT1/ROUT1/LOUT2/ROUT2)

## 観測結果

### 1) 基本 5 区間の聴感

| 区間 | 聴感 |
|---|---|
| `SINE_LR`   | 左: サイン波 + ヒス / 右: サイン波 |
| `ZERO`      | **完全無音** |
| `L_ONLY`    | 左: サイン波 + ヒス / 右: 無音 |
| `R_ONLY`    | 左: **無音のはずが、ヒス有り** / 右: サイン波 |
| `DITHER_LR` | 左: ヒス / 右: (ディザレベルなので実質無音) |

重要な事実:

- **`R_ONLY` では L ch PCM は厳密に 0 なのに、左からヒスが出ている** → ヒスは「駆動されている PCM チャンネル」を追わない
- **`ZERO` でのみヒスが消える** → 両 PCM チャンネルが厳密に `0x0000` のときだけ沈黙

### 2) レジスタダンプ(全区間で同一値)

5 区間 + `ZERO-mid` の計 6 ポイントで、対象 26 レジスタ全てが **完全に同じ値** を示した。抜粋:

| Reg | 値 | 意味 |
|---|---|---|
| `0x04 DACPOWER` | `0x30` | DACL/DACR 電源 ON、LOUT1/ROUT1 有効 |
| `0x19 DACCONTROL3` | `0xE0` | `DACMute=0`(ミュート解除)、Ramp 最遅 |
| `0x1A DACCONTROL4` (L digital vol) | `0x05` | 左右対称 |
| `0x1B DACCONTROL5` (R digital vol) | `0x05` | 左右対称 |
| `0x27 DACCONTROL17` | `0xD0` | `LD2LO=1` (L DAC → L ミキサ ON) |
| `0x2A DACCONTROL20` | `0xD0` | `RD2RO=1` (R DAC → R ミキサ ON) |
| `0x2E LOUT1 vol` | `0x1A` | **左右対称** (= ROUT1 vol) |
| `0x2F ROUT1 vol` | `0x1A` | **左右対称** (= LOUT1 vol) |

つまり runtime でコーデックは **一切自分の設定を書き換えていない** し、すべてのパラメータが **L/R 完全対称** である。

### 3) DACPOWER プローブ区間(決定打)

| 区間 | DACPOWER | 聴感 | 意味 |
|---|---|---|---|
| `PWR_0x20` | bit 5 のみ (`0b00100000`) | 左: サイン波 + ヒス / 右: 無音 | **bit 5 = LOUT1 = L 出力** |
| `PWR_0x10` | bit 4 のみ (`0b00010000`) | 右: サイン波 / 左: 無音 / **ヒス完全消失** | **bit 4 = ROUT1 = R 出力** |

- **LOUT1 を物理的に殺せばヒスも完全に消える** → ヒスは実際に LOUT1 ピン(およびその下流)から出力されている信号
- **ROUT1 単独再生ではヒスは皆無** → コーデック内の R 側 HP アンプ + ROUT1 ピン + 基板 R 配線 + HP ジャック R 端子の経路は本質的にクリーン

## 結論

### A. ノイズ源はソフトウェア起因ではない

以下の事実がソフトウェア起因を否定している:

1. **レジスタは L/R 完全対称**: `DACCONTROL4/5`, `DACCONTROL17/20`(LD2LO=RD2RO=1), `LOUT1 vol = ROUT1 vol` がすべて対称値。
2. **runtime でレジスタ書き換えが発生していない**: 同一値が全区間で維持される。
3. **I2S データはデジタル領域で L/R 対称**(コード上で検証済み): `R_ONLY` で L ch は厳密 `0x0000`。それでも L ヒスが出るので、デジタル送出側が L を「駆動」している訳ではない。
4. **`PWR_0x10` で ROUT1 単独駆動にするとヒスが消える**: 同一コード・同一クロック・同一振幅の sine を流しても R 側アナログ経路ではヒスが発生しない。

よって設定・ドライバ・AudioTools・A2DP 層・Wi-Fi/BLE のいずれもノイズ源ではない。

### B. ノイズ源は LOUT1 経路のハードウェア非対称性

確定している経路図:

```
PCM (どこかが非ゼロ)
   │
   ▼
ES8388 DAC Σ-Δ modulator (活動開始)
   │                               │
   ▼ L lane                        ▼ R lane
ES8388 HPA-L  ← ★ ヒス源            ES8388 HPA-R (クリーン)
   │                               │
   ▼ LOUT1 pin                     ▼ ROUT1 pin
Module Audio PCB 配線              Module Audio PCB 配線
   (AC 結合 C, ジャック L 端子)      (AC 結合 C, ジャック R 端子)
   │                               │
   ▼                               ▼
TRRS 左耳(ヒス + サイン)            TRRS 右耳(サインのみ)
```

ヒス源の候補(これ以上の絞り込みにはスコープ等のハード計測が必要):

1. **ES8388 ダイ個体の L ch HP アンプ部の特性ばらつき** (熱雑音 / バイアスオフセット / 高周波発振の素因)
2. **LOUT1 直近の AC 結合コンデンサ個体の ESR / 漏れ電流** が L 側のみ劣化 / 不良
3. **基板レイアウト上、LOUT1 配線近傍の結合源**(電源 / RGB LED / ADC などのスイッチングノイズ)
4. **HP ジャック L 端子のハンダ不良 / 接触抵抗**

### C. 「ZERO で無音になる」の解釈

これも間接的に結論を補強する。ES8388 DAC のΣΔ変調器は **両 PCM 入力が厳密に `0x0000` のとき DC 保持状態** になり、HPA を高周波駆動しないため、LOUT1 側のノイズ源も励起されず沈黙する。1 LSB でも非ゼロになると変調器が動き出し、LOUT1 HPA が通常動作となってヒスが露出する。この挙動はレジスタではなく DAC の本質的動作であり、ChatGPT との事前調査 (https://chatgpt.com/share/69e63dce-34dc-83ab-bdd7-f8aa55aae4d3) でも「データシートには明示されていないが排除もできない未記載挙動」として予想されていたものに合致する。

### D. 電源経路の寄与

USB vs バッテリー比較では**わずかに**ヒス量が変わる(USB で大きい)。これは LOUT1 経路のノイズ源に重畳している電源由来成分の寄与だが、主因ではない。**完全にバッテリーだけで駆動しても L ヒスは消えない**ので、電源系への対策は付随的効果しか得られない。

## 次の対策案

### 1) LOUT2 / ROUT2 を試す

ES8388 は LOUT1/ROUT1 とは独立した **LOUT2/ROUT2** 出力を持つ。Module Audio 基板上では LOUT2/ROUT2 のピンがスピーカ用途 or 外部アンプ接続用として引き出されている可能性がある(要 Module Audio 回路図確認)。

- `DACPOWER = 0x0C` (`DAC_OUTPUT_OUT2`) で OUT2 のみ有効化
- `DACPOWER = 0x3C` (`DAC_OUTPUT_ALL`) で OUT1/OUT2 同時
- `DACCONTROL26/27` の LOUT2/ROUT2 音量は現状 `0x00`(最小)なので、試すなら上げる

検証項目:

- LOUT2 駆動時にヒスが出るか
  - 出ないなら → LOUT1 アンプ or LOUT1 ピン直近の問題に限定される
  - 出るなら → ES8388 の L チャンネル共通段(ミキサ前 or DAC L コア)の問題
- LOUT2 経由で良好なら、**基板側で LOUT2 を拾う実装** がファームとして成立するか検討(要 Module Audio 基板改造 or 外部アンプ接続)

この切り分けは有力な次手だが、Module Audio 基板の物理的アクセス前提。

### 2) 個体ハードウェア対策による開発環境の改善

現状のままだと実機聴取時に常に L にサーノイズが乗り開発の集中を阻害する。当面の開発を快適にするためのハード対策案:

1. **別の Module Audio 個体と交換してみる**  
   同一症状なら Module Audio 設計上の弱点、変われば個体不良確定。最も安く確実な診断。
2. **LOUT1 近傍の AC 結合コンデンサを張り替え**  
   回路図で LOUT1 と HP ジャック L 端子の間にある DC カット用電解 or セラミックを同等品へ交換。ESR/漏れ電流起因なら改善する可能性がある。
3. **LOUT1 → HP ジャック L 間に直列 100 Ω 程度の抵抗 + GND にバイパス** (L-pad 風)  
   ChatGPT の一般論側の提案。アナログ減衰でヒスと信号を同時に落とし、下流側で信号だけ再増幅する発想。高感度 IEM 使用時に効く場合あり。
4. **HP ジャック L 端子のハンダやり直し**  
   接点抵抗の微小変動がノイズ源になる場合がある。テスターで L/R の導通抵抗差を確認。
5. **当面の運用として、R チャンネル単独のモノラル再生で開発を進める**  
   ファーム側で L DAC ミュート or L PCM 強制 0 + ミキサ設定で R に L+R 合成を回す運用。ヒス無しで開発継続可能。`main.cpp` の通常運用側に `NOISE_WORKAROUND_MONO_R` のようなフラグを設ける案。
6. **Module Audio ではなく別コーデックモジュール(別製品)で開発継続**  
   M5Stack には I2S スピーカ系(PoE、Speaker Hat 等)や I2S DAC 系の選択肢がある。深追いせず置き換える判断。

## 再現用ファーム

PR #7 の `cursor/audio-noise-i2s-test-d657` ブランチで再現可能:

```sh
pio run -e m5stack-core2-noise-test -t upload
pio device monitor -e m5stack-core2-noise-test
```

- 5+2 = 7 区間を 2 秒ずつ無限ループ
- LCD に現在の区間名 + USB / バッテリー状態を表示
- シリアルに各区間のレジスタダンプと電源状態を出力

この PR は調査の記録として **マージしてアーカイブ** する。以後の通常開発は `main` 側の `[env:m5stack-core2]` (A2DP + DJ Filter アプリ) で継続する。

## 参考

- 発端の議論: https://chatgpt.com/share/69e63dce-34dc-83ab-bdd7-f8aa55aae4d3
- テストファーム: `src/main_noise_test.cpp`
- PlatformIO env: `m5stack-core2-noise-test`
- ES8388 Datasheet: https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1141/ES8388.pdf
- 関連ドキュメント: `docs/ES8388_DAC_SoftRamp.md`
