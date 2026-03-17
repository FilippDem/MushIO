/*
 * adc_manager.h — ADS124S08 real ADC driver for MushIO V1.0
 *
 * Manages 6 × ADS124S08 ADCs over two SPI buses (SPI0 and SPI1),
 * each ADC reading 12 channels per scan frame.
 *
 * Output format: DATA_SIZE (216) bytes, 72 × 3-byte big-endian signed 24-bit,
 * matching the demo_adc output format so the host GUI is unchanged.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* Initialise SPI buses, reset ADCs, configure registers.
 * Returns true if at least one ADC responded with the correct device ID. */
bool adc_manager_init(void);

/*
 * adc_manager_scan — read one frame of 72 channels from the 6 ADCs.
 *
 * Reads 12 channels from each ADC sequentially (input mux scan).
 * out_data : must point to at least DATA_SIZE (216) bytes.
 *            Filled with 72 × 3-byte big-endian signed 24-bit samples.
 *
 * Returns the number of ADCs that responded successfully (0–6).
 */
int adc_manager_scan(uint8_t *out_data);

/* Read the device ID register from a specific ADC (0–5).
 * Returns the ID byte, or -1 on SPI failure. */
int adc_manager_read_id(int adc_index);

/* Check if a specific ADC's DRDY pin is asserted (data ready).
 * Returns true if DRDY is low (active). */
bool adc_manager_drdy(int adc_index);

/* ---- Low-level SPI helpers (exposed for diagnostic commands) ------------ */

/* ADS124S08 register addresses */
#define ADS_REG_INPMUX   0x02
#define ADS_REG_PGA      0x03
#define ADS_REG_DATARATE 0x04
#define ADS_REG_REF      0x05

/* ADS124S08 SPI command bytes */
#define ADS_CMD_START    0x08
#define ADS_CMD_STOP     0x0A
#define ADS_CMD_RDATA    0x12

/* Send a single command byte to an ADC */
void ads_cmd(int adc, uint8_t cmd);

/* Read n registers starting at addr. Returns bytes read, or 0 on error. */
int ads_rreg(int adc, uint8_t addr, uint8_t *buf, uint8_t n);

/* Write a single register */
void ads_wreg1(int adc, uint8_t addr, uint8_t val);

/* Read one conversion via RDATA. Returns 24-bit signed value. */
int32_t ads_rdata(int adc);

/* Wait for DRDY to go low, with timeout in µs. Returns true if asserted. */
bool ads_wait_drdy(int adc, uint32_t timeout_us);

/* Per-ADC channel scan masks (bitmask of AIN channels 0-11).
 * 0x0FFF = all channels.  Set via CMD "set_scan_masks" for checkerboard mode. */
extern volatile uint16_t g_scan_ch_mask[NUM_ADCS];

/* Scan timing diagnostic — measures per-channel cmd/drdy/read times.
 * Writes results into 'out' buffer (up to out_size chars). */
void adc_manager_scan_timing(char *out, int out_size);

/* Bit-bang SPI test on ADC3 — bypasses PIO to verify MISO is physically driven.
 * Writes diagnostic results into 'out' buffer (up to out_size chars).
 * Returns number of chars written. */
int bitbang_spi1_test(char *out, int out_size);
