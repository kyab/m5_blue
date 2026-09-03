# About PIMORONI Pico Audio Pack I2S Audio Module
https://shop.pimoroni.com/products/pico-audio-pack?variant=32369490853971

DataSheet, Schematics, C/C++ examples.

## Primary pins.
VSYS=39(VSYS)
I2S_DATA=12(GP9)
I2S_BCK=14(GP10)
I2S_LRCK=15(GP11)
MUTE=29(GP22)

# Build procedure of C/C++ Examples.
## PIMORONIのPico Audio Pack用C++サンプルのビルド方法

https://github.com/pimoroni/pimoroni-pico/tree/main/examples/pico_audio

pimoroni-picoレポジトリをクローン、サブモジュールもfetchする。
なお、さらにREADME.mdに記載の通りpico-extrasもRapi本家から別途クローン(https://github.com/raspberrypi/pico-extras)　してcmake時にパス指定必要。
```sh
$ git clone  https://github.com/pimoroni/pimoroni-pico.git
$ cd pimoroni-pico
$ git submodule update --init --recursive
```

以下VS Codeの拡張によりインストールされたSDK,ツールチェイン(~/.pico-sdk)を環境変数で設定しながらビルド。
ここではpico-extrasは/path/to/pico-extrasに存在すると仮定。
なおCMake -> Makeでビルドする方式

```sh
$ export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.0"
$ export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
$ mkdir build && cd build
$ cmake .. -DPICO_BOARD=pico2_w -DPICO_SDK_POST_LIST_DIRS=/path/to/pico-extras
$ make audio
```
build/examples/pico_audio以下にaudio.uf2が生成されるのでpico2wをStorageモードにしてファイルコピー。
自動で再起動後、イヤフォン・スピーカからシンセのメロディが聴こえればOK.

## Raspberry Pi C/C++ examples
https://github.com/raspberrypi/pico-playground
audio/sine_wave

```sh
$ git clone https://github.com/raspberrypi/pico-playground
$ cd pico-playground
```
なお、さらにpico-extraレポジトリ(https://github.com/raspberrypi/pico-extras)をしてcmake時にパス指定必要。

pico-playgroundのaudio/sine_wave/CMakeLists.txtでI2Sのピン定義を変更。

```cmake
    target_compile_definitions(sine_wave_i2s PRIVATE
            ...
            PICO_AUDIO_I2S_DATA_PIN=9
            PICO_AUDIO_I2S_CLOCK_PIN_BASE=10
            ...
    )
```

以下VS Codeの拡張によりインストールされたSDK,ツールチェイン(~/.pico-sdk)を環境変数で設定しながらビルド。
pico-extrasが/path/to/pico-extrasに存在すると仮定。
こちらはCMake->Ninjaでビルドする方式

```sh
$ export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.0"
$ export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
$ cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH=/path/to/pico-extras
$ cmake --build build --target pico_audio_i2s
```
build/audio/sine_wave/sine_wave.uf2が生成されるのでpico2wをStorageモードにしてファイルコピー。
自動で再起動後、イヤフォン・スピーカからサイン波の音が聞こえればOK.
app/usb_sound_cardも同様のI2Sピン設定でビルド、及び動作可能。Pico2 WがUSBサウンドカードして動作する。
