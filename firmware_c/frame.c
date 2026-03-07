/*
 * frame.c — MushIO 228-byte binary frame packer
 */

#include <string.h>
#include "frame.h"
#include "crc16.h"

void frame_build(uint8_t       *out,
                 uint32_t       timestamp_us,
                 uint16_t       seq_num,
                 const uint8_t *adc_data)
{
    /* Sync word — little-endian 0xAA55 → bytes [0x55, 0xAA] */
    out[0] = (uint8_t)(SYNC_WORD & 0xFFu);
    out[1] = (uint8_t)(SYNC_WORD >> 8);

    /* Timestamp (uint32 LE) */
    out[2] = (uint8_t)(timestamp_us        & 0xFFu);
    out[3] = (uint8_t)((timestamp_us >> 8) & 0xFFu);
    out[4] = (uint8_t)((timestamp_us >>16) & 0xFFu);
    out[5] = (uint8_t)((timestamp_us >>24) & 0xFFu);

    /* Sequence number (uint16 LE) */
    out[6] = (uint8_t)(seq_num & 0xFFu);
    out[7] = (uint8_t)(seq_num >> 8);

    /* ADC count / channels-per-ADC */
    out[8] = (uint8_t)NUM_ADCS;
    out[9] = (uint8_t)CHANNELS_PER_ADC;

    /* ADC data block */
    memcpy(out + HEADER_SIZE, adc_data, DATA_SIZE);

    /* CRC-16/CCITT-FALSE over header + data, appended LE */
    uint16_t crc = crc16_ccitt(out, HEADER_SIZE + DATA_SIZE);
    out[HEADER_SIZE + DATA_SIZE]     = (uint8_t)(crc & 0xFFu);
    out[HEADER_SIZE + DATA_SIZE + 1] = (uint8_t)(crc >> 8);
}
