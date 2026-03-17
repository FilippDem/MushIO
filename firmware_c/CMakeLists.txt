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

# GCC 15.2.1 on Windows crashes in the linker when the command line contains
# a bare -g flag (access violation 0xC0000005).  Scrub it from every cmake
# flag variable that gets forwarded to the link step.
foreach(_f CMAKE_C_FLAGS CMAKE_CXX_FLAGS CMAKE_EXE_LINKER_FLAGS
           CMAKE_C_FLAGS_RELEASE CMAKE_CXX_FLAGS_RELEASE
           CMAKE_C_FLAGS_DEBUG   CMAKE_CXX_FLAGS_DEBUG
           CMAKE_C_FLAGS_RELWITHDEBINFO CMAKE_CXX_FLAGS_RELWITHDEBINFO)
    string(REGEX REPLACE "(^| )-g( |$)" " " ${_f} "${${_f}}")
endforeach()

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------

add_executable(mushio_c
    main.c
    core1.c
    crc16.c
    frame.c
    demo_adc.c
    adc_manager.c
    ota_server.c
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

option(MUSHIO_DEMO "Synthetic ADC data (sine/triangle/chirp/sawtooth, no real SPI hardware)" ON)
if(MUSHIO_DEMO)
    target_compile_definitions(mushio_c PRIVATE MUSHIO_DEMO=1)
    message(STATUS "MUSHIO_DEMO=1: synthetic waveform data (sine/triangle/chirp/sawtooth)")
else()
    message(STATUS "MUSHIO_DEMO=0: real ADS124S08 SPI data")
endif()

# ---------------------------------------------------------------------------
# Libraries
# ---------------------------------------------------------------------------

target_link_libraries(mushio_c
    pico_stdlib                               # stdio, sleep, time
    pico_multicore                            # multicore_launch_core1, lockout
    pico_float                               # sinf() via hardware FPU
    hardware_timer                           # get_absolute_time, busy_wait_us
    hardware_watchdog                        # watchdog_enable, watchdog_update, scratch regs
    hardware_flash                           # flash_range_erase, flash_range_program (OTA)
    hardware_spi                             # SPI buses for ADS124S08 ADCs
    hardware_pio                             # PIO SPI for SPI1 bus (GP16 is SPI0, not SPI1)
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
# PIO programs  (SPI1 bus uses PIO because GP16 maps to HW SPI0, not SPI1)
# ---------------------------------------------------------------------------

pico_generate_pio_header(mushio_c ${CMAKE_CURRENT_SOURCE_DIR}/pio_spi.pio)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

target_compile_options(mushio_c PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter    # lwIP callbacks use void *arg frequently
    -O2                      # optimise for speed
    -g0                      # override SDK -g; no debug sections → avoids GCC 15.2.1 linker crash
)

# ARM GCC 15.2.1 linker crashes (0xC0000005) when debug info is present.
# -g0 above prevents debug sections in .o files; -g0 -s here keeps the
# linker command clean and strips any residual symbols from the ELF.
target_link_options(mushio_c PRIVATE -g0 -s)

# ---------------------------------------------------------------------------
# CYW43 WiFi throughput tuning
# ---------------------------------------------------------------------------

# CYW43 throughput: AMPDU tuning is patched directly in cyw43_ll.c.
# Credit-aware pacing is in do_udp_stream_work() (main.c).
# CYW43_USE_STATS and gSPI clock change disabled — caused boot failures.

# ---------------------------------------------------------------------------
# Print memory usage after linking
# ---------------------------------------------------------------------------

if(CMAKE_SIZE)
    add_custom_command(TARGET mushio_c POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:mushio_c>
        COMMENT "Firmware size"
    )
endif()
