/*
 * core1.c — Core 1 entry point: ADC scan loop
 *
 * Core 1 responsibility:
 *   1. Register as a multicore_lockout victim (so Core 0 can safely park
 *      Core 1 via multicore_lockout_start_blocking() before flash erase/
 *      program operations).
 *   2. Initialise the ADC subsystem (demo or real ADS124S08).
 *   3. Tight scan loop:
 *        a. Call adc scan — generates/reads 216-byte data.
 *        b. Pack the full 228-byte frame (header + data + CRC).
 *        c. Push frame into the lockless ring buffer.
 *        d. Sleep for scan period to pace frame rate.
 *   4. Never touch WiFi / lwIP — those are Core 0's domain.
 *
 * Flash safety (OTA)
 * ------------------
 * Before calling flash_range_erase() / flash_range_program(), Core 0 calls
 * multicore_lockout_start_blocking().  This sends an IPI to Core 1 via the
 * SIO FIFO.  Core 1's lockout handler (multicore_lockout_handler, placed in
 * SRAM by the SDK) fires, acknowledges the lockout, and spins with
 * interrupts disabled until Core 0 calls multicore_lockout_end_blocking()
 * (which it never does — it reboots via watchdog at OTA completion).
 *
 * Core 1 must call multicore_lockout_victim_init() before the scan loop so
 * that the SIO FIFO IRQ handler is registered and Core 0's lockout request
 * is handled promptly (< 1 ms).
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "config.h"
#include "ring.h"
#include "frame.h"

#ifdef MUSHIO_DEMO
#include "demo_adc.h"
#else
#include "adc_manager.h"
#endif

/* Shared ring buffer — defined in main.c, extern-declared here. */
extern ring_t g_ring;

/* Scan period — set at boot to DEMO_SCAN_PERIOD_US; updated by 'set_fps' CMD. */
extern volatile uint32_t g_scan_period_us;

/* Pause flag — set by Core 0 to temporarily stop SPI scanning.
 * Core 1 checks this each iteration and spins while paused.
 * Used by diagnostic commands (read_ch, set_pga, etc.) that
 * need exclusive SPI bus access from Core 0. */
extern volatile bool g_scan_paused;

/* =========================================================================
 * Core 1 entry
 * ========================================================================= */

void core1_main(void)
{
    printf("[C1] Core 1 started — ADC scan loop\n");

    /* Register Core 1 as a multicore_lockout victim.
     * This installs the SIO FIFO IRQ handler (multicore_lockout_handler,
     * SRAM-resident) so that Core 0 can safely park Core 1 before any flash
     * erase / program operation during OTA.  Must be called BEFORE the scan
     * loop to ensure the handler is ready if OTA starts immediately. */
    multicore_lockout_victim_init();

#ifdef MUSHIO_DEMO
    /* Initialise demo ADC (builds sine table using FPU sinf). */
    demo_adc_init();
    printf("[C1] Demo ADC ready.  Starting scan loop at ~%u FPS.\n",
           (unsigned)(1000000u / DEMO_SCAN_PERIOD_US));
#else
    /* Initialise real ADS124S08 ADCs over SPI. */
    if (!adc_manager_init()) {
        printf("[C1] WARNING: No ADCs responded — scan loop will output zeros.\n");
    }
    printf("[C1] Real ADC mode.  Starting scan loop at ~%u FPS.\n",
           (unsigned)(1000000u / DEMO_SCAN_PERIOD_US));
#endif

    uint8_t  adc_data[DATA_SIZE];
    uint8_t  frame[FRAME_SIZE];
    uint16_t seq        = 0;
    uint32_t scan_count = 0;

    absolute_time_t next_scan = get_absolute_time();

    while (true) {
        /* -------------------------------------------------------------------
         * 0. Honour pause request from Core 0 (diagnostic / config commands).
         *    Spin-wait with tight_loop_contents() to stay responsive to
         *    multicore_lockout (OTA) while paused.
         * ------------------------------------------------------------------- */
        while (g_scan_paused) {
            tight_loop_contents();
        }

        /* -------------------------------------------------------------------
         * 1. Capture timestamp first — used for both waveform synthesis and
         *    the frame header so phase is coherent with the packet timestamp.
         * ------------------------------------------------------------------- */
        uint32_t ts_us = (uint32_t)(to_us_since_boot(get_absolute_time()));

        /* -------------------------------------------------------------------
         * 2. Read ADC data (fills adc_data[DATA_SIZE]).
         * ------------------------------------------------------------------- */
#ifdef MUSHIO_DEMO
        demo_adc_scan(adc_data, ts_us);
#else
        adc_manager_scan(adc_data);
#endif

        /* -------------------------------------------------------------------
         * 3. Pack 228-byte frame (header + data + CRC-16).
         * ------------------------------------------------------------------- */
        frame_build(frame, ts_us, seq, adc_data);

        /* -------------------------------------------------------------------
         * 4. Push frame into ring buffer.
         *    ring_push() uses __dmb() before advancing head.
         * ------------------------------------------------------------------- */
        if (!ring_push(&g_ring, frame)) {
            /* Ring full — frame was silently dropped; ring.dropped counter
             * already incremented inside ring_push(). */
        }

        seq = (uint16_t)((seq + 1u) & 0xFFFFu);
        scan_count++;

        /* -------------------------------------------------------------------
         * 5. Pace to g_scan_period_us per frame using the hardware timer.
         *    busy_wait_until() avoids drift that sleep_us() accumulates.
         * ------------------------------------------------------------------- */
        next_scan = delayed_by_us(next_scan, g_scan_period_us);
        busy_wait_until(next_scan);
    }

    /* unreachable */
}
