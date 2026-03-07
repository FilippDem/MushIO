/*
 * config.h — MushIO V1.0 C Firmware Configuration
 *
 * Mirror of firmware/config.py for the native Pico SDK build.
 * Verify GPIO assignments against schematic before first power-on.
 */

#pragma once

#include <stdint.h>

/* =========================================================================
 * WiFi
 * ========================================================================= */

#define WIFI_SSID       "Occam's Router"
#define WIFI_PASSWORD   "purplelotus636"
#define WIFI_COUNTRY    "US"

/* =========================================================================
 * TCP Streaming  (data port)
 * ========================================================================= */

#define HOST_IP         "192.168.68.115"
#define HOST_PORT       9000

/* =========================================================================
 * CMD Server  (command/control port)
 * ========================================================================= */

#define CMD_PORT        9001

/* =========================================================================
 * Frame Format
 * ========================================================================= */

#define SYNC_WORD           0xAA55u

#define NUM_ADCS            6u
#define CHANNELS_PER_ADC    12u
#define TOTAL_CHANNELS      (NUM_ADCS * CHANNELS_PER_ADC)  /* 72 */

#define BYTES_PER_SAMPLE    3u
#define HEADER_SIZE         10u                                  /* sync(2)+ts(4)+seq(2)+nadc(1)+nch(1) */
#define DATA_SIZE           (TOTAL_CHANNELS * BYTES_PER_SAMPLE) /* 216 */
#define CRC_SIZE            2u
#define FRAME_SIZE          (HEADER_SIZE + DATA_SIZE + CRC_SIZE) /* 228 */

/* =========================================================================
 * Ring Buffer  (frames)
 * ring.h uses % (not bitwise-AND), so RING_SIZE does NOT need to be a
 * power of 2 — any integer value is correct.
 *
 * Memory budget (from linker map, old RING_SIZE=128 build):
 *   Non-ring overhead (CYW43 state, lwIP 32 KB static heap, BSS, data): 110 KB
 *   Main RAM total (RP2350):                                             512 KB
 *   Available for ring:                                                  402 KB
 *
 *   RING_SIZE=1536: 1536 × 228 B = 342 KB ring, 59 KB spare  →  3.07 s at 500 FPS
 *
 * The stacks (2 KB each per core) are in SCRATCH_X/Y and are not counted
 * in the main-RAM budget above.  There are no runtime malloc() calls in
 * the application; the spare 59 KB is genuinely unused.
 * ========================================================================= */

#define RING_SIZE           1536u

/* =========================================================================
 * Streaming Batch
 * 40 × 228 = 9,120 bytes per tcp_write() call
 * ========================================================================= */

#define STREAM_BATCH        40u
#define STREAM_TIMEOUT_MS   20u     /* flush partial batch after this many ms */

/* =========================================================================
 * Reconnect
 * ========================================================================= */

#define RECONNECT_INTERVAL_MS   1000u

/* =========================================================================
 * Demo ADC Timing
 * Default scan period used on boot.  Can be overridden at runtime via the
 * "set_fps <hz>" CMD command without requiring a rebuild.
 * 2000 µs → 500 FPS default.  Minimum practical: ~200 µs (5000 FPS).
 * ========================================================================= */

#define DEMO_SCAN_PERIOD_US     2000u

/* =========================================================================
 * SPI Bus Pin Assignments  (for real ADC firmware — not used in demo)
 * ========================================================================= */

#define SPI0_SCK    2
#define SPI0_MOSI   3
#define SPI0_MISO   4

#define SPI1_SCK    14
#define SPI1_MOSI   15
#define SPI1_MISO   12

/* Chip Selects: ADC0..ADC5 */
#define ADC_CS_PINS     {5, 7, 9, 11, 13, 17}
/* Data Ready:  ADC0..ADC5 */
#define ADC_DRDY_PINS   {6, 8, 10, 16, 18, 19}

#define ADC_START_PIN   21
#define ADC_RESET_N_PIN 20
#define ADC_CLK_EN_PIN  22

#define I2C0_SDA_PIN    0
#define I2C0_SCL_PIN    1
