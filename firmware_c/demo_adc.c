/*
 * demo_adc.c — Synthetic ADC data generator with waveform variety
 *
 * Waveform type is assigned by ADC index so adjacent grid columns look
 * visually distinct:
 *
 *   ADC 0 (ch  0-11) : sine        1-12 Hz
 *   ADC 1 (ch 12-23) : triangle    1-12 Hz
 *   ADC 2 (ch 24-35) : chirp       2->20 Hz sweep, 4 s period
 *   ADC 3 (ch 36-47) : sawtooth    1-12 Hz
 *   ADC 4 (ch 48-59) : sine        1-12 Hz
 *   ADC 5 (ch 60-71) : triangle    1-12 Hz
 *
 * Each channel has a unique phase offset (~5 deg steps) so identical-type
 * channels on the same ADC still look distinct.
 *
 * All computations are driven by the hardware timestamp (us) so frequencies
 * are exact in Hz regardless of scan rate set via 'set_fps'.
 *
 * Amplitude: ~1 mV pk-pk input-referred (realistic biopotential level).
 *   amp_frac = 1 mV * AFE_GAIN / (2 * VREF) = 0.001 * 11 / 10.16 ≈ 0.0011
 * STIM channels (ADC 2 ain>=8, ADC 3 ain>=8): amplitude halved (~0.5 mV pk-pk).
 *
 * Output byte order: big-endian 24-bit signed (MSB first), matching the
 * real ADS124S08 SPI read format.
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

/* Chirp parameters (ADC 2) */
#define CHIRP_F_LO   2.0f   /* Hz — start frequency */
#define CHIRP_F_HI  20.0f   /* Hz — end frequency   */
#define CHIRP_T      4.0f   /* s  — period of one sweep cycle */

/* Precomputed per-channel constants (filled once by demo_adc_init). */
static float s_freq_hz[TOTAL_CHANNELS];    /* 1..12 Hz  */
static float s_phase_off[TOTAL_CHANNELS];  /* 0 .. 2π, unique per channel */

void demo_adc_init(void)
{
    for (int ch = 0; ch < (int)TOTAL_CHANNELS; ch++) {
        /* Integer Hz (1..12) so every frequency lands on an exact FFT bin
         * for any integer-second display window (df = 1/win_secs Hz). */
        s_freq_hz[ch]   = (float)(ch % (int)CHANNELS_PER_ADC) + 1.0f;
        /* Spread initial phases evenly so same-frequency channels on
         * different ADCs look distinct (~5 deg steps for 72 channels). */
        s_phase_off[ch] = TWO_PI * (float)ch / (float)TOTAL_CHANNELS;
    }
}

void demo_adc_scan(uint8_t *out_data, uint32_t ts_us)
{
    /* Convert timestamp to seconds.  float32 loses ~128 µs precision at
     * ts_us ~2^31 (~35 min) which introduces <1 deg phase error at 12 Hz —
     * acceptable for a visual demo. */
    float t_s = (float)ts_us * 1e-6f;

    for (int ch = 0; ch < (int)TOTAL_CHANNELS; ch++) {
        int   adc  = ch / (int)CHANNELS_PER_ADC;
        int   ain  = ch % (int)CHANNELS_PER_ADC;
        float freq = s_freq_hz[ch];           /* 1..12 Hz */
        float phi  = s_phase_off[ch];         /* 0..2π   */
        float val_f;

        switch (adc) {
        default:
        case 0:
        case 4: {
            /* ---- Sine ---- */
            val_f = sinf(TWO_PI * freq * t_s + phi);
            break;
        }

        case 1:
        case 5: {
            /* ---- Triangle ---- */
            /* phase in [0,1) using per-channel time offset derived from phi */
            float p = fmodf(freq * t_s + phi / TWO_PI, 1.0f);
            if (p < 0.0f) p += 1.0f;
            val_f = 1.0f - 4.0f * fabsf(p - 0.5f);   /* -1..1 */
            break;
        }

        case 2: {
            /* ---- Chirp (linear, 2->20 Hz over CHIRP_T seconds) ----
             * Each channel on this ADC starts its sweep at a different point
             * in the cycle so they don't all sweep in lockstep. */
            float t_off = (float)ain * CHIRP_T / (float)CHANNELS_PER_ADC;
            float t_mod = fmodf(t_s + t_off, CHIRP_T);
            if (t_mod < 0.0f) t_mod += CHIRP_T;
            /* Instantaneous phase = integral of linear frequency ramp */
            float angle = TWO_PI * (CHIRP_F_LO * t_mod
                          + (CHIRP_F_HI - CHIRP_F_LO) / (2.0f * CHIRP_T) * t_mod * t_mod)
                          + phi;
            val_f = sinf(angle);
            break;
        }

        case 3: {
            /* ---- Sawtooth ---- */
            float p = fmodf(freq * t_s + phi / TWO_PI, 1.0f);
            if (p < 0.0f) p += 1.0f;
            val_f = 2.0f * p - 1.0f;   /* -1..1 */
            break;
        }
        }

        /* ~1 mV pk-pk input-referred (was 40% FS = ~185 mV).
         * amp_frac = 1 mV * AFE_GAIN / (2 * VREF)
         *          = 0.001 * 11.0 / (2 * 5.08) = 0.001083                 */
        int32_t val = (int32_t)(val_f * (float)ADC_FULL_SCALE * 0.0011f);

        /* STIM channels: halve amplitude to visually distinguish them */
        bool is_stim = ((adc == 2 || adc == 3) && ain >= 8);
        if (is_stim) val >>= 1;

        /* Pack as big-endian 24-bit signed (ADS124S08 SPI format) */
        out_data[ch * 3 + 0] = (uint8_t)((val >> 16) & 0xFF);
        out_data[ch * 3 + 1] = (uint8_t)((val >>  8) & 0xFF);
        out_data[ch * 3 + 2] = (uint8_t)( val        & 0xFF);
    }
}
