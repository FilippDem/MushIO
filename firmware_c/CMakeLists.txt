# CMakeLists.txt — MushIO V1.0 C Firmware
# Target board: Raspberry Pi Pico 2 W  (RP2350 + CYW43439)
#
# Prerequisites:
#   - PICO_SDK_PATH environment variable pointing to Pico SDK >= 2.0
#   - ARM GCC toolchain (arm-none-eabi-gcc)
#
# Build:
#   cd firmware_c
#   mkdir build && cd build
#   cmake .. -DPICO_BOARD=pico2_w
#   make -j4
#
# Output: build/mushio_c.uf2  (drag-and-drop to Pico in BOOTSEL mode)

cmake_minimum_required(VERSION 3.13)

# Board and platform must be set BEFORE pico_sdk_import.cmake
set(PICO_BOARD       pico2_w    CACHE STRING "Pico board type")
set(PICO_PLATFORM    rp2350     CACHE STRING "RP2350 platform")

# Pull in Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(mushio_c C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# Disable coprocessor disassembly post-build step.
# Without this, the SDK runs `picotool coprodis` on the ELF which on Windows
# can trigger an "Open With" dialog if the file association is not configured.
set(PICO_NO_COPRO_DIS 1 CACHE BOOL "Disable coprodis post-build step" FORCE)

pico_sdk_init()

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------

add_executable(mushio_c
    main.c
    core1.c
    crc16.c
    frame.c
    demo_adc.c
)

# Include our own headers (config.h, lwipopts.h, ring.h, ...)
target_include_directories(mushio_c PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------
# Build mode: synthetic demo data vs real ADS124S08 SPI hardware
# ---------------------------------------------------------------------------
# Pass -DMUSHIO_DEMO=0 to cmake to build for the real MushIO board.
# Default ON so any Pico 2 W can run demo without needing the shield.

option(MUSHIO_DEMO "Synthetic sine-wave ADC data (no real SPI hardware)" ON)
if(MUSHIO_DEMO)
    target_compile_definitions(mushio_c PRIVATE MUSHIO_DEMO=1)
    message(STATUS "MUSHIO_DEMO=1: synthetic sine-wave data")
else()
    message(STATUS "MUSHIO_DEMO=0: real ADS124S08 SPI data")
endif()

# ---------------------------------------------------------------------------
# Libraries
# ---------------------------------------------------------------------------

target_link_libraries(mushio_c
    pico_stdlib                               # stdio, sleep, time
    pico_multicore                            # multicore_launch_core1
    pico_float                               # sinf() via hardware FPU
    hardware_timer                           # get_absolute_time, busy_wait_us
    pico_cyw43_arch_lwip_threadsafe_background  # WiFi + lwIP (alarm-IRQ model)
)

# ---------------------------------------------------------------------------
# Serial output: USB only  (no UART so GP0/GP1 stay free for I2C)
# ---------------------------------------------------------------------------

pico_enable_stdio_usb(mushio_c  1)
pico_enable_stdio_uart(mushio_c 0)

# ---------------------------------------------------------------------------
# Build artefacts: .uf2, .bin, .hex, .map, .dis
# ---------------------------------------------------------------------------

pico_add_extra_outputs(mushio_c)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

target_compile_options(mushio_c PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter    # lwIP callbacks use void *arg frequently
    -O2                      # optimise for speed
)

# ---------------------------------------------------------------------------
# Print memory usage after linking
# ---------------------------------------------------------------------------

if(CMAKE_SIZE)
    add_custom_command(TARGET mushio_c POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:mushio_c>
        COMMENT "Firmware size"
    )
endif()
