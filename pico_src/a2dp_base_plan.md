A2DP Library選択

# 1. picoaudio
https://github.com/nicx17/picoaudio
- Dual Core活用
- Multipoint対応
- UI Sound Synthesizerによる起動音など音声案内あり

## Trial Build memo
以下でビルド・実行成功
```sh
$ # Needs pico_extra_repo
$ export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.0"
$ export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
$ # Edit CMakeLists.txt : Pico SDK version, tools version,  I2S PIN assignments...
$ cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH=/Users/koji/work/m5_blue/pico_src/pimoroni_example/pico-extras
$ cmake --build build
```

macbookからの再生で20-30秒程度ごとにブチブチいう。sync driftでのresampling対策が逆に良くないかも。UNDERRUNが発生している様子であった。Cursorと1時間ほど頑張ってもわからず。

UI Sound/Multi Pointは良い機能なので参考にしたい。

# 2. 公式pico_examplesのa2dp_sink_demo
https://github.com/raspberrypi/pico-examples 
bluetooth/btstack_examples/a2dp_sink_demo

Chat GPTおすすめその1
BTStackのa2dp_sink_exampleの特定HW実装として、pico-extrasのi2s(pico_i2s)を組み合わせたもの。
公式系で始める基本形。

## Trial Build memo
sdk2.3.1でビルド、実行成功。

```sh
$ export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
$ export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
$ cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH=/Users/koji/work/m5_blue/pico_src/pico-extras
$ cmake --build build --target a2dp_sink_demo
```

ノイズなし。
音量コントロールが効かない,デバッグメッセージがないといった不足があるが、A2DP/I2S出力自体は機能している。

# 3. PicoW_A2DP
https://github.com/joba-1/PicoW_A2DP?utm_source=chatgpt.com
Chat GPTおすすめその2
2. `公式pico_examplesのa2dp_sink_demo`をベースに機能発展させたもののようだ。

33 Github Stars.

## Trial Build memo
sdk2.3.1でビルド、実行成功。

```sh
$ export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
$ export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
$ cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH=/Users/koji/work/m5_blue/pico_src/pico-extras
$ cmake --build build --target a2dp_sink_demo
```
(playback_handlerでコンパイルエラーが出るがpico_examplesと同様にする引数追加でビルド可)

ノイズなし、音量コントールありで機能性は良いが、既存ペアリングを解除後、２度とmacから接続できなかった。
シリアルログにはBluetooth stack is up とでているがその後の処理が進んでいない可能性がある。
また、最終更新が2025/6なのでベースコードがpico examplesにもついていけてない様子か。

音量コントロールや付加機能の参考先、構造化の参考にはなる。


# 4. Note記事: Raspberry Pi Pico 2 W を Bluetooth オーディオレシーバーにしてみた の実装
https://note.com/huge_donkey2654/n/n12859c76e44e
I2Sについては自前のPIO実装を行っているようだ。

2 Stars.

# 結論
公式pico_examplesのa2dp_sink_demoをベースにする。

---


