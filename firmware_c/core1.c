/*
 * core1.c — Core 1 entry point: ADC scan loop
 *
 * Core 1 responsibility:
 *   1. Initialise the (demo) ADC subsystem.
 *   2. Tight scan loop at ~200 FPS:
 *        a. Call demo_adc_scan() — generates synthetic 216-byte data.
 *        b. Pack the full 228-byte frame (header + data + CRC).
 *        c. Push frame into the lockless ring buffer.
 *        d. Sleep for DEMO_SCAN_PERIOD_US to pace frame rate.
 *   3. Never touch WiFi / lwIP — those are Core 0's domain.
 *
 * The ring buffer's head pointer is the ONLY shared state Core 1 writes.
 * Core 0 reads head and writes tail.  __dmb() barriers in ring.h ensure
 * correct visibility without any spinlock.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "config.h"
#include "ring.h"
#include "frame.h"
#include "demo_adc.h"

/* Shared ring buffer — defined in main.c, extern-declared here. */
extern ring_t g_ring;

/* Scan period — set at boot to DEMO_SCAN_PERIOD_US; updated by 'set_fps' CMD. */
extern volatile uint32_t g_scan_period_us;

/* =========================================================================
 * Core 1 entry
 * ========================================================================= */

void core1_main(void)
{
    printf("[C1] Core 1 started — ADC scan loop\n");

    /* Register this core as a multicore_lockout victim so that Core 0 can
     * safely call multicore_lockout_start_blocking() before flash erase /
     * program operations.  Must be called before the scan loop begins. */
    multicore_lockout_victim_init();

    /* Initialise demo ADC (builds sine table using FPU sinf). */
    demo_adc_init();

    uint8_t  adc_data[DATA_SIZE];
    uint8_t  frame[FRAME_SIZE];
    uint16_t seq        = 0;
    uint32_t scan_count = 0;

    printf("[C1] Demo ADC ready.  Starting scan loop at ~%u FPS.\n",
           (unsigned)(1000000u / DEMO_SCAN_PERIOD_US));

    absolute_time_t next_scan = get_absolute_time();

    while (true) {
        /* -------------------------------------------------------------------
         * 1. Capture timestamp first — used for both waveform synthesis and
         *    the frame header so phase is coherent with the packet timestamp.
         * ------------------------------------------------------------------- */
        uint32_t ts_us = (uint32_t)(to_us_since_boot(get_absolute_time()));

        /* -------------------------------------------------------------------
         * 2. Generate synthetic ADC data (fills adc_data[DATA_SIZE]).
         *    Frequencies are fixed in Hz regardless of scan rate.
         * ------------------------------------------------------------------- */
        demo_adc_scan(adc_data, ts_us);

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
