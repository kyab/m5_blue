A2DP Library

##picoaudio
https://github.com/nicx17/picoaudio
- Dual Core 
- Multipoint
- UI Sound Synthesizer

Trial Build memo
```sh
$ # Needs pico_extra_repo
$ export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.0"
$ export PATH="$HOME/.pico-sdk/cmake/v4.3.4/CMake.app/Contents/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH"
$ # Edit CMakeLists.txt : Pico SDK version, tools version,  I2S PIN assignments...
$ cmake -S . -B build -GNinja -DPICO_BOARD=pico2_w -DPICO_EXTRAS_PATH=/Users/koji/work/m5_blue/pico_src/pimoroni_example/pico-extras
$ cmake --build build
```
Device名:CAR


## Note: Raspberry Pi Pico 2 W を Bluetooth オーディオレシーバーにしてみた
https://note.com/huge_donkey2654/n/n12859c76e44e
I2Sについて自前のPIO処理を行っているようだ。
