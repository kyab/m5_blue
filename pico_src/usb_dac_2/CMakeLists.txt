cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico2_w CACHE STRING "Board type")

set(PICO_TINYUSB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/tinyusb)

if (NOT PICO_EXTRAS_PATH AND NOT DEFINED ENV{PICO_EXTRAS_PATH})
    set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/../first/pico-extras)
endif()

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(usb_dac_2 C CXX ASM)
pico_sdk_init()

add_executable(usb_dac_2
    src/main.cpp
    src/usb_descriptors.c
    src/usb_audio.cpp
    src/usb_midi.cpp
    src/audio_engine.cpp
)

target_include_directories(usb_dac_2 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_definitions(usb_dac_2 PRIVATE
    PICO_AUDIO_I2S_MONO_OUTPUT=0
    PICO_AUDIO_I2S_MONO_INPUT=0
    PICO_AUDIO_I2S_DATA_PIN=9
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=10
    PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=1
    PICO_ENABLE_USB_RESET_VIA_VENDOR_INTERFACE=0
)

target_link_libraries(usb_dac_2
    pico_stdlib
    pico_multicore
    pico_unique_id
    tinyusb_device
    pico_audio_i2s
)

pico_enable_stdio_uart(usb_dac_2 0)
pico_enable_stdio_usb(usb_dac_2 1)
pico_add_extra_outputs(usb_dac_2)
