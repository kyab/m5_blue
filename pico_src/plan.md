A2DP Library

# picoaudio
https://github.com/nicx17/picoaudio
- Dual Core 
- Multipoint
- UI Sound Synthesizer

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

macbookからの再生で20-30秒程度ごとにブチブチいう。sync driftでのresampling対策が逆に良くないかも。UNDERRUNが発生している様子Cursorと1時間ほど頑張ってもわからず。

UI Sound/Multi Pointは良い機能なのでそこは参考にしたい。

# pico_examples (a2dp_sink_demo)
https://github.com/raspberrypi/pico-examples 
bluetooth/btstack_examples/a2dp_sink_demo

Chat GPTおすすめその1
BTStackのa2dp sink exampleに pico-extrasのi2s(pico_i2s)を組み合わせたもの

## Trial Build memo
sdk2.3.1でビルド、実行成功。
ノイズなし。
音量コントロールが効かない,デバッグメッセージがない。がA2DP/I2S出力自体は機能しており、
標準APIベースのBasic実装として参考にしとくのが良さそう。

```sh
$ export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
$ export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
$ cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH=/Users/koji/work/m5_blue/pico_src/pico-extras
$ cmake --build build --target a2dp_sink_demo
```

# PicoW_A2DP
https://github.com/joba-1/PicoW_A2DP?utm_source=chatgpt.com
Chat GPTおすすめその2
pico_w/bt/a2dp_sink_demoをベースに機能発展させたもののようだ。
ハマらずに使えたらこちらを使う。

33 Stars.

## Trial Build memo
sdk2.3.1でビルド、実行成功。

```sh
$ export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.1"
$ export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
$ cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH=/Users/koji/work/m5_blue/pico_src/pico-extras
$ cmake --build build --target a2dp_sink_demo
```
(playback_handlerでコンパイルエラーが出るがpico_examplesと同様にする引数追加でビルド可)

ノイズなし、音量コントールありで機能性は良いが、他ファームウェアで残存していたペアリングを解除後２度とmacから接続できない。
シリアルログにはBluetooth stack is up とでている。

最終更新が2025/6なのでpico examplesにもついていけてない様子か。

音量コントロールや付加機能の参考先としては使えるのでは。


# Note記事: Raspberry Pi Pico 2 W を Bluetooth オーディオレシーバーにしてみた
https://note.com/huge_donkey2654/n/n12859c76e44e
I2Sについては自前のPIO実装を行っているようだ。

2 Stars.



# 結論
pico_exampleをベースにPicoW_A2DPからつまみ食い？
