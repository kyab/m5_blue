cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(USERHOME $ENV{HOME})
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
set(PICO_BOARD pico2_w CACHE STRING "Board type")
set(PICO_EXTRAS_PATH /Users/koji/work/m5_blue/pico_src/first/pico-extras)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)
project(tusb_probe C CXX ASM)
pico_sdk_init()

add_executable(tusb_probe
    src/main.c
    src/usb_descriptors.c
    src/quirk_os_guessing.c
    src/rp2040_hw_endpoint_fix.c
)
target_include_directories(tusb_probe PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${PICO_SDK_PATH}/lib/tinyusb/src/portable/raspberrypi/rp2040
)
target_compile_definitions(tusb_probe PRIVATE
    PICO_AUDIO_I2S_MONO_OUTPUT=0
    PICO_AUDIO_I2S_MONO_INPUT=0
    PICO_AUDIO_I2S_DATA_PIN=9
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=10
    # Custom descriptors in src/usb_descriptors.c; still want 1200-baud BOOTSEL.
    PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=1
    PICO_ENABLE_USB_RESET_VIA_VENDOR_INTERFACE=0
)
# TinyUSB 0.18 starts a new ISO transfer without aborting the previous buffer.
target_link_options(tusb_probe PRIVATE -Wl,--wrap=hw_endpoint_xfer_start)
target_link_libraries(tusb_probe pico_stdlib tinyusb_device pico_audio_i2s pico_unique_id)
pico_enable_stdio_uart(tusb_probe 1)
pico_enable_stdio_usb(tusb_probe 1)
pico_add_extra_outputs(tusb_probe)
