/*
 * demo_adc.c — Synthetic sine-wave ADC data generator
 *
 * Each channel produces a sine wave at a FIXED frequency in Hz, computed
 * directly from the hardware timestamp.  Frequencies are FPS-independent:
 * they remain correct at any scan rate set via 'set_fps'.
 *
 * Frequency:    (ch % CHANNELS_PER_ADC) + 1  →  1 .. 12 Hz (integers) per ADC group
 * Phase offset: ch × 2π / TOTAL_CHANNELS     →  unique per channel (~5° steps)
 * Amplitude:    40 % of full scale
 * STIM chans:   20 % of full scale (halved) to visually distinguish them
 *
 * Output byte order: big-endian 24-bit (MSB first) per sample,
 * matching the real ADS124S08 SPI read format.
 */

#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "demo_adc.h"

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#define ADC_FULL_SCALE  8388607   /* 2^23 - 1 */
#define TWO_PI          (2.0f * (float)M_PI)

/* Precomputed per-channel constants (filled once by demo_adc_init). */
static float s_freq_hz[TOTAL_CHANNELS];   /* 1 .. 12 Hz */
static float s_phase_off[TOTAL_CHANNELS]; /* 0 .. 2π, unique per channel */

void demo_adc_init(void)
{
    for (int ch = 0; ch < (int)TOTAL_CHANNELS; ch++) {
        /* Integer Hz (1..12) so every frequency lands on an exact FFT bin
         * for any integer-second display window (df = 1/win_secs Hz):
         *   1 Hz → bin 1 × win_secs,  2 Hz → bin 2 × win_secs, etc.
         * This eliminates the leakage-driven "two-state" FFT oscillation
         * that occurred when frequencies (0.5, 1.5 … 6 Hz) straddled bins. */
        s_freq_hz[ch]   = (float)(ch % (int)CHANNELS_PER_ADC) + 1.0f;  /* 1..12 Hz */
        /* Spread initial phases evenly so identical-frequency channels on
         * different ADCs look distinct (≈5° steps for 72 channels). */
        s_phase_off[ch] = TWO_PI * (float)ch / (float)TOTAL_CHANNELS;
    }
}

void demo_adc_scan(uint8_t *out_data, uint32_t ts_us)
{
    /* Convert timestamp to seconds.  float32 loses ~128 µs precision at
     * ts_us ≈ 2^31 (≈35 min) which introduces <1° phase error at 12 Hz —
     * acceptable for a visual demo. */
    float t_s = (float)ts_us * 1e-6f;

    for (int ch = 0; ch < (int)TOTAL_CHANNELS; ch++) {
        float   angle = TWO_PI * s_freq_hz[ch] * t_s + s_phase_off[ch];
        int32_t val   = (int32_t)(sinf(angle) * (float)ADC_FULL_SCALE * 0.4f);

        /* STIM channels: halve amplitude to distinguish from recording chans */
        int  adc     = ch / (int)CHANNELS_PER_ADC;
        int  ain     = ch % (int)CHANNELS_PER_ADC;
        bool is_stim = ((adc == 2 || adc == 3) && ain >= 8);
        if (is_stim) val >>= 1;

        /* Pack as big-endian 24-bit signed (ADS124S08 SPI format) */
        out_data[ch * 3 + 0] = (uint8_t)((val >> 16) & 0xFF);
        out_data[ch * 3 + 1] = (uint8_t)((val >>  8) & 0xFF);
        out_data[ch * 3 + 2] = (uint8_t)( val        & 0xFF);
    }
}
