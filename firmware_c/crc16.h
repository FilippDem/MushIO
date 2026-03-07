/*
 * crc16.h — CRC-16/CCITT-FALSE
 * Polynomial 0x1021, Init 0xFFFF, no reflection, no final XOR.
 * Table-driven: ~10x faster than the bit-loop used in the Python firmware.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

/* Initialise the 256-entry lookup table.  Call once at startup. */
void crc16_init_table(void);

/* Compute CRC over data[0..len-1]. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);
