/*
 * demo_adc.h — Synthetic ADC data generator  (no hardware required)
 *
 * Produces 72 channels of sine-wave data at slightly different frequencies
 * so that the GUI waveform display shows clearly distinct signals.
 *
 * Output format: DATA_SIZE (216) bytes, 72 × 3-byte big-endian signed 24-bit,
 * matching the real ADS124S08 SPI output format.
 */

#pragma once

#include <stdint.h>
#include "config.h"

/* Initialise the 256-entry sine lookup table.  Call once before first scan. */
void demo_adc_init(void);

/*
 * demo_adc_scan — fill out_data with one frame of synthetic samples.
 *
 * ts_us    : hardware timestamp in microseconds (from to_us_since_boot).
 *            Used to compute sine phase directly, so all channel frequencies
 *            are fixed in Hz regardless of scan rate.
 * out_data : must point to at least DATA_SIZE (216) bytes.
 */
void demo_adc_scan(uint8_t *out_data, uint32_t ts_us);
