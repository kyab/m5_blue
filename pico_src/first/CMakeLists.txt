# Generated Cmake Pico project file
#
# Self-contained A2DP sink for Pico 2 W + Pimoroni Audio Pack.
# Based on raspberrypi/pico-examples bluetooth/btstack_examples/a2dp_sink_demo
# (logic unchanged; build glue + I2S pin defs for Pimoroni).

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Bundled pico-extras submodule (override with -DPICO_EXTRAS_PATH=... if needed)
if (NOT PICO_EXTRAS_PATH AND NOT DEFINED ENV{PICO_EXTRAS_PATH})
    set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-extras)
endif()

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(a2dp_sink_demo C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

if (NOT PICO_CYW43_SUPPORTED)
    message(FATAL_ERROR "PICO_CYW43_SUPPORTED is required (board must be pico_w / pico2_w)")
endif()

if (NOT TARGET pico_btstack_base)
    message(FATAL_ERROR "pico_btstack_base missing; initialize the btstack submodule in the Pico SDK")
endif()

if (NOT DEFINED PICO_BTSTACK_PATH)
    set(PICO_BTSTACK_PATH ${PICO_SDK_PATH}/lib/btstack)
endif()

# See https://bluekitchen-gmbh.com/btstack/#examples/examples/#sec:a2dpsinkdemoExample
# Source a2dp_sink_demo.c lives in pico-sdk/lib/btstack/example/
add_executable(a2dp_sink_demo
    main.cpp
    btstack_audio_pico.c
    ${PICO_BTSTACK_PATH}/example/a2dp_sink_demo.c
)

target_link_libraries(a2dp_sink_demo PRIVATE
    pico_stdlib
    pico_multicore
    pico_cyw43_arch_none
    pico_btstack_cyw43
    pico_btstack_ble
    pico_btstack_classic
    pico_btstack_sbc_decoder
    pico_audio_i2s
    hardware_i2c
)

target_include_directories(a2dp_sink_demo PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # for btstack_config.h
)

# Pimoroni Pico Audio Pack: DATA=GP9, BCK=GP10, LRCK=GP11 (CLOCK_PIN_BASE+1).
# MUTE=GP22 is unused by this demo / pico_audio_i2s path, so omit.
# USING_I2C is the upstream demo flag name that enables the I2S audio sink wiring.
# PICO_ENABLE_USB_RESET_VIA_BAUD_RATE: SDK 2.3.1 name (PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE is <=2.2.0 alias).
# Opening the USB CDC serial at PICO_USB_RESET_MAGIC_BAUD_RATE (1200) enters BOOTSEL without the button.
target_compile_definitions(a2dp_sink_demo PRIVATE
    USING_I2C=1
    PICO_AUDIO_I2S_DATA_PIN=9
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=10
    PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=1
)

# Effect switch: Default SYNTH_WITH_BUTTON. Use -DPARTY_PICO_MODE=SYNTH or DJ.
set(PARTY_PICO_MODE "SYNTH_WITH_BUTTON" CACHE STRING "PARTY_PICO_MODE: SYNTH_WITH_BUTTON (default), SYNTH, or DJ")
if (PARTY_PICO_MODE STREQUAL "DJ")
    target_compile_definitions(a2dp_sink_demo PRIVATE PARTY_PICO_MODE_DJ=1)
elseif (PARTY_PICO_MODE STREQUAL "SYNTH")
    target_compile_definitions(a2dp_sink_demo PRIVATE PARTY_PICO_MODE_SYNTH=1)
elseif (PARTY_PICO_MODE STREQUAL "SYNTH_WITH_BUTTON")
    target_compile_definitions(a2dp_sink_demo PRIVATE PARTY_PICO_MODE_SYNTH_WITH_BUTTON=1)
else()
    message(FATAL_ERROR "Unknown PARTY_PICO_MODE='${PARTY_PICO_MODE}' (expected SYNTH_WITH_BUTTON, SYNTH, or DJ)")
endif()

pico_add_extra_outputs(a2dp_sink_demo)

# Run the toolchain size tool after every link.
if (NOT CMAKE_SIZE)
    get_filename_component(_pico_objcopy_dir "${CMAKE_OBJCOPY}" DIRECTORY)
    find_program(CMAKE_SIZE NAMES arm-none-eabi-size llvm-size size HINTS "${_pico_objcopy_dir}" REQUIRED)
endif()
add_custom_command(TARGET a2dp_sink_demo POST_BUILD
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:a2dp_sink_demo>"
    VERBATIM
)

if (TARGET tinyusb_device)
    pico_enable_stdio_usb(a2dp_sink_demo 1)
    pico_enable_stdio_uart(a2dp_sink_demo 0)
elseif(PICO_ON_DEVICE)
    message("Skipping USB stdio because TinyUSB submodule is not initialized in the SDK")
endif()
