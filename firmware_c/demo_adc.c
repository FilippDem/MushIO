/*
 * demo_adc.c — Synthetic ADC data generator with waveform variety
 *
 * Waveform type is assigned by PHYSICAL GRID ROW so each row in the 8x8
 * electrode grid displays one distinct waveform shape.  Frequency increases
 * with physical grid column (1, 2, 3, 4, 5, 6, 8, 12 Hz).
 *
 *   Row 0 : Sine wave               (basic rendering test)
 *   Row 1 : Sawtooth wave           (sharp ramp + reset)
 *   Row 2 : Square wave             (digital-style on/off)
 *   Row 3 : Slow DC drift           (random-walk baseline wander)
 *   Row 4 : Sine + noise            (noisy biopotential)
 *   Row 5 : Chirp (freq sweep)      (sweep from f to 3f)
 *   Row 6 : Damped sine burst       (evoked-potential-like)
 *   Row 7 : Action-potential spikes  (biological test signal)
 *
 *   Column frequencies (Hz): 1, 2, 3, 4, 5, 6, 8, 12
 *
 * STIM channels (col,row = -1,-1) get a slow sine at 0.5-1.2 Hz.
 *
 * Amplitude: ~1 mV pk-pk input-referred (realistic biopotential level).
 *   amp_frac = 1 mV * AFE_GAIN / (2 * VREF) = 0.001 * 11 / 10.16 ~ 0.0011
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

/* Chirp parameters (row 5) */
#define CHIRP_T      2.0f   /* s  -- period of one sweep cycle */

/* ---- Physical grid mapping: ch -> (col, row) ----
 * Derived from host/receiver.py CHANNEL_MAP.
 * STIM channels have col=row=-1.                    */
typedef struct { int8_t col; int8_t row; } grid_pos_t;

static const grid_pos_t s_grid[TOTAL_CHANNELS] = {
    /* ADC 0 (ch 0-11) */
    /* ch  0 = ELEC02 */ { 0, 2},
    /* ch  1 = ELEC23 */ { 2, 3},
    /* ch  2 = ELEC22 */ { 2, 2},
    /* ch  3 = ELEC12 */ { 1, 2},
    /* ch  4 = ELEC11 */ { 1, 1},
    /* ch  5 = ELEC21 */ { 2, 1},
    /* ch  6 = ELEC20 */ { 2, 0},
    /* ch  7 = ELEC01 */ { 0, 1},
    /* ch  8 = ELEC00 */ { 0, 0},
    /* ch  9 = ELEC10 */ { 1, 0},
    /* ch 10 = ELEC03 */ { 0, 3},
    /* ch 11 = ELEC13 */ { 1, 3},
    /* ADC 1 (ch 12-23) */
    /* ch 12 = ELEC26 */ { 2, 6},
    /* ch 13 = ELEC16 */ { 1, 6},
    /* ch 14 = ELEC25 */ { 2, 5},
    /* ch 15 = ELEC15 */ { 1, 5},
    /* ch 16 = ELEC24 */ { 2, 4},
    /* ch 17 = ELEC05 */ { 0, 5},
    /* ch 18 = ELEC04 */ { 0, 4},
    /* ch 19 = ELEC14 */ { 1, 4},
    /* ch 20 = ELEC17 */ { 1, 7},
    /* ch 21 = ELEC07 */ { 0, 7},
    /* ch 22 = ELEC06 */ { 0, 6},
    /* ch 23 = ELEC27 */ { 2, 7},
    /* ADC 2 (ch 24-35) */
    /* ch 24 = ELEC41 */ { 4, 1},
    /* ch 25 = ELEC31 */ { 3, 1},
    /* ch 26 = ELEC30 */ { 3, 0},
    /* ch 27 = ELEC40 */ { 4, 0},
    /* ch 28 = ELEC43 */ { 4, 3},
    /* ch 29 = ELEC33 */ { 3, 3},
    /* ch 30 = ELEC32 */ { 3, 2},
    /* ch 31 = ELEC42 */ { 4, 2},
    /* ch 32 = STIM_1 */ {-1,-1},
    /* ch 33 = STIM_0 */ {-1,-1},
    /* ch 34 = STIM_7 */ {-1,-1},
    /* ch 35 = STIM_6 */ {-1,-1},
    /* ADC 3 (ch 36-47) */
    /* ch 36 = ELEC45 */ { 4, 5},
    /* ch 37 = ELEC35 */ { 3, 5},
    /* ch 38 = ELEC34 */ { 3, 4},
    /* ch 39 = ELEC44 */ { 4, 4},
    /* ch 40 = ELEC47 */ { 4, 7},
    /* ch 41 = ELEC37 */ { 3, 7},
    /* ch 42 = ELEC36 */ { 3, 6},
    /* ch 43 = ELEC46 */ { 4, 6},
    /* ch 44 = STIM_3 */ {-1,-1},
    /* ch 45 = STIM_2 */ {-1,-1},
    /* ch 46 = STIM_5 */ {-1,-1},
    /* ch 47 = STIM_4 */ {-1,-1},
    /* ADC 4 (ch 48-59) */
    /* ch 48 = ELEC71 */ { 7, 1},
    /* ch 49 = ELEC50 */ { 5, 0},
    /* ch 50 = ELEC60 */ { 6, 0},
    /* ch 51 = ELEC70 */ { 7, 0},
    /* ch 52 = ELEC62 */ { 6, 2},
    /* ch 53 = ELEC52 */ { 5, 2},
    /* ch 54 = ELEC51 */ { 5, 1},
    /* ch 55 = ELEC61 */ { 6, 1},
    /* ch 56 = ELEC73 */ { 7, 3},
    /* ch 57 = ELEC63 */ { 6, 3},
    /* ch 58 = ELEC53 */ { 5, 3},
    /* ch 59 = ELEC72 */ { 7, 2},
    /* ADC 5 (ch 60-71) */
    /* ch 60 = ELEC75 */ { 7, 5},
    /* ch 61 = ELEC54 */ { 5, 4},
    /* ch 62 = ELEC64 */ { 6, 4},
    /* ch 63 = ELEC74 */ { 7, 4},
    /* ch 64 = ELEC66 */ { 6, 6},
    /* ch 65 = ELEC56 */ { 5, 6},
    /* ch 66 = ELEC55 */ { 5, 5},
    /* ch 67 = ELEC65 */ { 6, 5},
    /* ch 68 = ELEC67 */ { 6, 7},
    /* ch 69 = ELEC77 */ { 7, 7},
    /* ch 70 = ELEC76 */ { 7, 6},
    /* ch 71 = ELEC57 */ { 5, 7},
};

/* Column frequencies (Hz): 8 columns, increasing */
static const float s_col_freq[8] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f, 12.0f};

/* Precomputed per-channel phase offset (filled once by demo_adc_init). */
static float s_phase_off[TOTAL_CHANNELS];

/* Simple xorshift32 PRNG for noise / spike generation */
static uint32_t s_rng = 0x12345678u;
static float _randf(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return (float)(s_rng & 0x7FFFFF) / (float)0x7FFFFF;  /* 0..1 */
}
static float _gaussf(void) {
    /* Box-Muller (approximate, single-sided) */
    float u1 = _randf() * 0.9999f + 0.0001f;
    float u2 = _randf();
    return sqrtf(-2.0f * logf(u1)) * cosf(TWO_PI * u2);
}

/* Action-potential template (15 samples, normalised to -1..1) */
static const float s_ap[] = {
    0.00f, 0.35f, 0.75f, 1.00f, 0.70f, 0.20f,
   -0.45f,-0.60f,-0.45f,-0.25f,-0.10f,-0.03f, 0.00f, 0.00f, 0.00f
};
#define AP_LEN  (sizeof(s_ap) / sizeof(s_ap[0]))

/* Per-channel spike state (row 7 only, but sized for all for simplicity) */
static int16_t s_spike_cd[TOTAL_CHANNELS];  /* countdown: 0 = idle */

/* Per-channel random walk state (row 3: slow DC drift) */
static float s_rw[TOTAL_CHANNELS];  /* current random walk position, -1..1 */

void demo_adc_init(void)
{
    for (int ch = 0; ch < (int)TOTAL_CHANNELS; ch++) {
        /* Spread initial phases so same-row channels look distinct */
        s_phase_off[ch] = TWO_PI * (float)ch / (float)TOTAL_CHANNELS;
        s_spike_cd[ch]  = 0;
    }
}

void demo_adc_scan(uint8_t *out_data, uint32_t ts_us)
{
    float t_s = (float)ts_us * 1e-6f;

    for (int ch = 0; ch < (int)TOTAL_CHANNELS; ch++) {
        int   col = s_grid[ch].col;
        int   row = s_grid[ch].row;
        float phi = s_phase_off[ch];
        float val_f;

        /* ~1 mV pk-pk input-referred */
        float amp = 0.0011f;

        if (col < 0) {
            /* ---- STIM channel: slow sine ---- */
            float stim_f = 0.5f + (float)(ch % 8) * 0.1f;
            val_f = sinf(TWO_PI * stim_f * t_s + phi);
            amp *= 0.5f;  /* half amplitude for STIM */
        } else {
            float freq = s_col_freq[col];

            switch (row) {
            default:
            case 0: {
                /* ---- Sine ---- */
                val_f = sinf(TWO_PI * freq * t_s + phi);
                break;
            }
            case 1: {
                /* ---- Sawtooth ---- */
                float p = fmodf(freq * t_s + phi / TWO_PI, 1.0f);
                if (p < 0.0f) p += 1.0f;
                val_f = 2.0f * p - 1.0f;
                break;
            }
            case 2: {
                /* ---- Square ---- */
                float s = sinf(TWO_PI * freq * t_s + phi);
                val_f = (s >= 0.0f) ? 1.0f : -1.0f;
                break;
            }
            case 3: {
                /* ---- Slow DC drift (random-walk baseline wander) ---- */
                float drift_rate = 0.002f * freq;
                s_rw[ch] += _gaussf() * drift_rate;
                if (s_rw[ch] >  1.0f) s_rw[ch] =  1.0f;
                if (s_rw[ch] < -1.0f) s_rw[ch] = -1.0f;
                val_f = s_rw[ch];
                break;
            }
            case 4: {
                /* ---- Sine + noise (noisy biopotential) ---- */
                val_f = sinf(TWO_PI * freq * t_s + phi) + _gaussf() * 0.3f;
                if (val_f >  1.3f) val_f =  1.3f;
                if (val_f < -1.3f) val_f = -1.3f;
                break;
            }
            case 5: {
                /* ---- Chirp: freq sweeps from f to 3f over CHIRP_T s ---- */
                float t_off = (float)col * CHIRP_T / 8.0f;
                float t_mod = fmodf(t_s + t_off, CHIRP_T);
                if (t_mod < 0.0f) t_mod += CHIRP_T;
                float f_lo = freq;
                float f_hi = freq * 3.0f;
                float angle = TWO_PI * (f_lo * t_mod
                              + (f_hi - f_lo) / (2.0f * CHIRP_T) * t_mod * t_mod)
                              + phi;
                val_f = sinf(angle);
                break;
            }
            case 6: {
                /* ---- Damped sine burst (evoked-potential-like) ---- */
                float period = 1.0f / fmaxf(freq * 0.25f, 0.5f);
                float bt = fmodf(t_s, period);
                float decay = expf(-3.0f * freq * bt);
                val_f = decay * sinf(TWO_PI * freq * 3.0f * bt + phi);
                break;
            }
            case 7: {
                /* ---- Action-potential spike trains ---- */
                int16_t cd = s_spike_cd[ch];
                int refrac = 80;  /* samples (~160 ms at 500 Hz) */
                if (cd > 0) {
                    int ap_idx = refrac - cd;
                    if (ap_idx >= 0 && ap_idx < (int)AP_LEN) {
                        val_f = s_ap[ap_idx] * 0.15f;
                    } else {
                        val_f = _gaussf() * 0.05f;
                    }
                    s_spike_cd[ch] = cd - 1;
                } else {
                    /* Poisson spike rate ~3 Hz (vary by column) */
                    float rate = 1.0f + (float)col;
                    if (_randf() < rate / 500.0f) {
                        s_spike_cd[ch] = (int16_t)refrac;
                        val_f = s_ap[0] * 0.15f;
                    } else {
                        val_f = _gaussf() * 0.05f;
                    }
                }
                break;
            }
            }
        }

        int32_t val = (int32_t)(val_f * (float)ADC_FULL_SCALE * amp);

        /* Pack as big-endian 24-bit signed (ADS124S08 SPI format) */
        out_data[ch * 3 + 0] = (uint8_t)((val >> 16) & 0xFF);
        out_data[ch * 3 + 1] = (uint8_t)((val >>  8) & 0xFF);
        out_data[ch * 3 + 2] = (uint8_t)( val        & 0xFF);
    }
}
