/*
 * frame.h — MushIO 228-byte binary frame packer
 *
 * Frame layout (little-endian header):
 *   Offset  Size  Field
 *    0       2    Sync word: 0xAA55
 *    2       4    Timestamp µs (uint32, wraps ~71 min)
 *    6       2    Sequence number (uint16, wraps 65535)
 *    8       1    NUM_ADCS  (6)
 *    9       1    CHANNELS_PER_ADC  (12)
 *   10     216    ADC data: 72 ch × 3-byte big-endian signed 24-bit
 *  226       2    CRC-16/CCITT-FALSE (over bytes 0..225), little-endian
 *  Total: 228 bytes
 */

#pragma once

#include <stdint.h>
#include "config.h"

/*
 * frame_build — fill an already-allocated 228-byte buffer.
 *
 * Parameters:
 *   out          Pointer to FRAME_SIZE-byte output buffer.
 *   timestamp_us Microseconds since boot (to_us_since_boot).  Wraps ~71 min.
 *   seq_num      Sequence counter (caller manages wrap-around).
 *   adc_data     DATA_SIZE raw bytes (72 × 3 B big-endian 24-bit samples).
 */
void frame_build(uint8_t *out,
                 uint32_t timestamp_us,
                 uint16_t seq_num,
                 const uint8_t *adc_data);
