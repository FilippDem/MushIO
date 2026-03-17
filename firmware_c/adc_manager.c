/*
 * adc_manager.c — ADS124S08 real ADC driver for MushIO V1.0
 *
 * Hardware:
 *   6 × ADS124S08 (24-bit delta-sigma ADCs) on two SPI buses
 *   ADC0..ADC2  →  SPI0 (GP2=SCK, GP3=MOSI, GP4=MISO)
 *   ADC3..ADC5  →  PIO SPI (GP14=SCK, GP15=MOSI, GP16=MISO)
 *                   GP16 maps to hardware SPI0 (not SPI1) on RP2350,
 *                   so we use a PIO state machine for the second bus.
 *   Each ADC has its own CS and DRDY pin (see config.h)
 *   Shared: ADC_RESET_N (GP20), ADC_START (GP21), STATUS_LED (GP22)
 *
 * Each ADC reads 12 channels per scan frame using input mux cycling.
 * Total: 6 × 12 = 72 channels × 3 bytes = 216 bytes per frame.
 *
 * ADS124S08 SPI protocol (Mode 0, MSB first):
 *   RREG:  TX [0x20|addr, n-1]  →  RX n bytes
 *   WREG:  TX [0x40|addr, n-1, data...]
 *   RDATA: TX [0x12, 0x00, 0x00, 0x00]  →  RX 3 data bytes
 *   START: TX [0x08]
 *   STOP:  TX [0x0A]
 *   RESET: TX [0x06]
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "config.h"
#include "adc_manager.h"
#include "pio_spi.h"

/* =========================================================================
 * ADS124S08 SPI Commands
 * ========================================================================= */

/* Commands already in adc_manager.h: START, STOP, RDATA */
#define ADS_CMD_NOP     0x00
#define ADS_CMD_WAKEUP  0x02
#define ADS_CMD_PWRDWN  0x04
#define ADS_CMD_RESET   0x06
#define ADS_CMD_RREG    0x20    /* | register address */
#define ADS_CMD_WREG    0x40    /* | register address */

/* =========================================================================
 * ADS124S08 Registers
 * ========================================================================= */

/* Registers already defined in adc_manager.h: INPMUX, PGA, DATARATE, REF */
#define ADS_REG_ID       0x00
#define ADS_REG_STATUS   0x01
#define ADS_REG_IDACMAG  0x06
#define ADS_REG_IDACMUX  0x07
#define ADS_REG_VBIAS    0x08
#define ADS_REG_SYS      0x09
#define ADS_REG_GPIODAT  0x10
#define ADS_REG_GPIOCON  0x11

/* Expected device ID for ADS124S08 (bits [2:0]) */
#define ADS124S08_DEV_ID  0x00

/* =========================================================================
 * Pin tables
 * ========================================================================= */

static const uint8_t adc_cs_pins[NUM_ADCS]   = ADC_CS_PINS;
static const uint8_t adc_drdy_pins[NUM_ADCS]  = ADC_DRDY_PINS;

/* ADC0..2 use hardware SPI0.  ADC3..5 use PIO SPI (GP16 ≠ HW SPI1). */
static spi_inst_t *adc_hw_spi = NULL;   /* hardware SPI0 for ADC0-2 */

/* PIO SPI state for ADC3-5 (non-static: accessed by bitbang_test in main.c) */
PIO   g_pio      = NULL;
uint  g_pio_sm   = 0;
uint  g_pio_off  = 0;

static inline bool adc_uses_pio(int adc) { return adc >= 3; }

/* SPI clock: 8 MHz for both buses — ADS124S08 supports up to 10 MHz at 3.3 V.
 * PIO timing margins: at 8 MHz SPI (125 ns/bit), 4 PIO cycles per bit gives
 * ~31 ns per phase — comfortably above ADS124S08's tDO ≤ 25 ns. */
#define ADC_SPI_BAUDRATE    8000000
#define PIO_SPI_BAUDRATE    8000000

/* =========================================================================
 * Low-level SPI helpers
 * ========================================================================= */

static inline void cs_select(int adc)
{
    gpio_put(adc_cs_pins[adc], 0);
    /* ADS124S08 requires ≥30 ns after CS goes low; a few NOPs suffice. */
    __asm volatile("nop \n nop \n nop \n nop");
}

static inline void cs_deselect(int adc)
{
    __asm volatile("nop \n nop \n nop \n nop");
    gpio_put(adc_cs_pins[adc], 1);
}

/* Send a single command byte */
void ads_cmd(int adc, uint8_t cmd)
{
    cs_select(adc);
    if (adc_uses_pio(adc))
        pio_spi_write(g_pio, g_pio_sm, &cmd, 1);
    else
        spi_write_blocking(adc_hw_spi, &cmd, 1);
    cs_deselect(adc);
}

/* Read n registers starting at addr.  Returns bytes read, or 0 on error.
 *
 * ADS124S08 RREG protocol:
 *   1. Send command byte (0x20 | addr)
 *   2. Send count byte (n - 1)
 *   3. Wait for ADC to decode command (td(SCCS) ~50 tCLK ≈ 12 µs at 4.096 MHz)
 *   4. Clock out n data bytes (send 0x00 NOP while reading)
 *
 * Previous implementation sent header + data in one burst; at 4 MHz SPI the
 * ADC had no time to decode the RREG command before data was clocked out,
 * resulting in all-0xFF or all-0x00 reads. */
int ads_rreg(int adc, uint8_t addr, uint8_t *buf, uint8_t n)
{
    if (n == 0 || n > 18) return 0;

    /* Send 2-byte RREG header */
    uint8_t hdr[2] = {
        (uint8_t)(ADS_CMD_RREG | (addr & 0x1F)),
        (uint8_t)(n - 1)
    };

    cs_select(adc);
    if (adc_uses_pio(adc)) {
        pio_spi_write(g_pio, g_pio_sm, hdr, 2);
        busy_wait_us(10);
        pio_spi_read(g_pio, g_pio_sm, 0x00, buf, n);
    } else {
        spi_write_blocking(adc_hw_spi, hdr, 2);
        busy_wait_us(10);
        spi_read_blocking(adc_hw_spi, 0x00, buf, n);
    }
    cs_deselect(adc);

    return n;
}

/* Write n registers starting at addr.
 * Uses a single SPI transaction to avoid timing gaps. */
static void ads_wreg(int adc, uint8_t addr, const uint8_t *data, uint8_t n)
{
    uint8_t tx[20] = {0};
    uint8_t total = (uint8_t)(2 + n);
    if (total > sizeof(tx)) total = sizeof(tx);
    tx[0] = (uint8_t)(ADS_CMD_WREG | (addr & 0x1F));
    tx[1] = (uint8_t)(n - 1);
    memcpy(&tx[2], data, n);
    cs_select(adc);
    if (adc_uses_pio(adc))
        pio_spi_write(g_pio, g_pio_sm, tx, total);
    else
        spi_write_blocking(adc_hw_spi, tx, total);
    cs_deselect(adc);
}

/* Write a single register */
void ads_wreg1(int adc, uint8_t addr, uint8_t val)
{
    ads_wreg(adc, addr, &val, 1);
}

/* Read one conversion via RDATA command.  Returns 24-bit signed value.
 * On failure returns 0. */
int32_t ads_rdata(int adc)
{
    uint8_t tx[4] = { ADS_CMD_RDATA, 0x00, 0x00, 0x00 };
    uint8_t rx[4] = { 0 };

    cs_select(adc);
    if (adc_uses_pio(adc))
        pio_spi_write_read(g_pio, g_pio_sm, tx, rx, 4);
    else
        spi_write_read_blocking(adc_hw_spi, tx, rx, 4);
    cs_deselect(adc);

    /* Data is in rx[1..3], big-endian 24-bit signed */
    int32_t val = ((int32_t)rx[1] << 16) | ((int32_t)rx[2] << 8) | rx[3];
    /* Sign-extend from 24-bit */
    if (val & 0x800000)
        val |= (int32_t)0xFF000000;
    return val;
}

/* Wait for DRDY to go low (data ready), with timeout.
 * Returns true if DRDY asserted within timeout_us. */
bool ads_wait_drdy(int adc, uint32_t timeout_us)
{
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (!time_reached(deadline)) {
        if (!gpio_get(adc_drdy_pins[adc]))   /* active low */
            return true;
    }
    return false;
}

/* =========================================================================
 * Initialisation
 * ========================================================================= */

bool adc_manager_init(void)
{
    printf("[ADC] Initialising ADS124S08 × %u\n", (unsigned)NUM_ADCS);

    /* ---- Hardware SPI0 for ADC0-2 ---- */
    adc_hw_spi = spi0;
    spi_init(spi0, ADC_SPI_BAUDRATE);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI0_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(SPI0_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_MISO, GPIO_FUNC_SPI);

    /* ---- PIO SPI for ADC3-5 ----
     * GP16 maps to hardware SPI0 on RP2350 (not SPI1), so we use PIO
     * to implement SPI Mode 1 on GP14(SCK)/GP15(MOSI)/GP16(MISO). */
    {
        /* Try PIO1 first (PIO0 is used by CYW43 WiFi, PIO2 on RP2350
         * may have GPIO input routing issues). Fall back to auto-alloc. */
        bool ok = false;
        int pio1_sm = pio_claim_unused_sm(pio1, false);
        if (pio1_sm >= 0) {
            if (pio_can_add_program(pio1, &pio_spi_cpha1_program)) {
                g_pio = pio1;
                g_pio_sm = (uint)pio1_sm;
                g_pio_off = pio_add_program(pio1, &pio_spi_cpha1_program);
                ok = true;
            } else {
                pio_sm_unclaim(pio1, (uint)pio1_sm);
            }
        }
        if (!ok) {
            /* Fallback: auto-allocate from any PIO */
            ok = pio_claim_free_sm_and_add_program_for_gpio_range(
                          &pio_spi_cpha1_program, &g_pio, &g_pio_sm, &g_pio_off,
                          SPI1_SCK,
                          SPI1_MISO - SPI1_SCK + 1,
                          true);
        }
        if (!ok) {
            printf("[ADC] FATAL: no free PIO for SPI1 bus\n");
        } else {
            /* PIO clock divider: SPI freq = PIO_clock / 4 instructions per bit.
             * For 4 MHz SPI: PIO_clock = 16 MHz → div = 150 / 16 = 9.375 */
            float clkdiv = (float)clock_get_hz(clk_sys) / (PIO_SPI_BAUDRATE * 4.0f);
            pio_spi_cpha1_program_init(g_pio, g_pio_sm, g_pio_off,
                                        SPI1_MOSI, SPI1_MISO, SPI1_SCK,
                                        clkdiv);
            printf("[ADC] PIO SPI on PIO%d SM%d  (GP%d=SCK GP%d=MOSI GP%d=MISO)  clkdiv=%.1f\n",
                   pio_get_index(g_pio), g_pio_sm,
                   SPI1_SCK, SPI1_MOSI, SPI1_MISO, (double)clkdiv);
        }
    }

    /* ---- Initialise CS pins (active low, start high) ---- */
    for (int i = 0; i < (int)NUM_ADCS; i++) {
        gpio_init(adc_cs_pins[i]);
        gpio_set_dir(adc_cs_pins[i], GPIO_OUT);
        gpio_put(adc_cs_pins[i], 1);
    }

    /* ---- Initialise DRDY pins (input, active low) ---- */
    for (int i = 0; i < (int)NUM_ADCS; i++) {
        gpio_init(adc_drdy_pins[i]);
        gpio_set_dir(adc_drdy_pins[i], GPIO_IN);
        gpio_pull_up(adc_drdy_pins[i]);  /* pull-up; DRDY drives low */
    }

    /* ---- Control pins ---- */
    /* ADC_RESET_N: active-low reset — pulse low to reset all ADCs */
    gpio_init(ADC_RESET_N_PIN);
    gpio_set_dir(ADC_RESET_N_PIN, GPIO_OUT);
    gpio_put(ADC_RESET_N_PIN, 0);  /* assert reset */
    sleep_ms(10);
    gpio_put(ADC_RESET_N_PIN, 1);  /* release reset */
    sleep_ms(50);                   /* wait for ADC power-on (>50 ms typ.) */

    /* ADC_START: global conversion start pin */
    gpio_init(ADC_START_PIN);
    gpio_set_dir(ADC_START_PIN, GPIO_OUT);
    gpio_put(ADC_START_PIN, 0);     /* low — we'll use SPI START command */

    /* ---- Software reset each ADC and verify ID ---- */
    int alive = 0;
    for (int i = 0; i < (int)NUM_ADCS; i++) {
        /* Software reset */
        ads_cmd(i, ADS_CMD_RESET);
        sleep_ms(5);   /* t_DRDY after reset ~4096 tCLK */

        /* Read device ID */
        uint8_t id = 0xFF;
        if (ads_rreg(i, ADS_REG_ID, &id, 1) > 0) {
            uint8_t dev = id & 0x07;
            printf("[ADC] ADC%d  ID=0x%02X  dev=%d  %s\n",
                   i, id, dev,
                   (dev == ADS124S08_DEV_ID) ? "ADS124S08 OK" : "UNKNOWN");
            if (dev == ADS124S08_DEV_ID)
                alive++;
        } else {
            printf("[ADC] ADC%d  SPI read failed\n", i);
        }

        /* ---- Configure ADC registers ----
         * PGA:      gain=1, PGA disabled (bypass)
         * DATARATE: 4000 SPS, continuous conversion mode, no filter
         * REF:      internal 2.5 V reference, always on
         * SYS:      no system monitor, SPI timeout disabled
         * INPMUX:   will be set per-channel during scan
         */
        ads_wreg1(i, ADS_REG_PGA,      0x00);  /* gain=1, PGA off */
        ads_wreg1(i, ADS_REG_DATARATE,  0x1D);  /* 4000 SPS, continuous, LL filter
                                                  * 0x1D = G_CHOP=0, CLK=int,
                                                  * MODE=0 (continuous),
                                                  * FILTER=1 (low-latency),
                                                  * DR=1101 (4000 SPS)
                                                  * In continuous mode, WREG INPMUX
                                                  * resets the digital filter.  The
                                                  * LL sinc1 settles in 1/4000 s =
                                                  * 250 µs, and the modulator stays
                                                  * warm (no startup delay). */
        ads_wreg1(i, ADS_REG_REF,       0x3A);  /* int ref on, REFP0/REFN0 */
        ads_wreg1(i, ADS_REG_SYS,       0x00);  /* no system monitor */
    }

    /* ---- Start continuous conversion on all ADCs ---- */
    for (int i = 0; i < (int)NUM_ADCS; i++)
        ads_cmd(i, ADS_CMD_START);
    sleep_ms(1);   /* allow first conversion to complete */

    printf("[ADC] %d of %u ADCs alive\n", alive, (unsigned)NUM_ADCS);
    return (alive > 0);
}

/* =========================================================================
 * Scan — read 12 channels from all 6 ADCs in parallel
 *
 * For each channel we:
 *   1. WREG  INPMUX on all 6 ADCs (3 bytes per ADC — resets digital filter)
 *   2. Poll  all 6 DRDY pins until every one goes LOW (or timeout)
 *   3. READ  3-byte data from all 6 ADCs (continuous-mode direct read)
 *
 * Continuous mode + WREG-only: the modulator stays warm (no startup delay).
 * WREG INPMUX resets the LL filter; sinc1 settles in 1/fDATA = 250 µs.
 * 8 MHz SPI on both buses.  Target: ~290 FPS (12 × ~286 µs/channel).
 * ========================================================================= */

/* Fast data read for continuous conversion mode.
 * In continuous mode, DRDY going low means data is ready — no RDATA command
 * needed.  Just clock out 3 bytes of NOP to read the 24-bit conversion.
 * Saves 1 byte (8 SPI clocks) per ADC per channel vs ads_rdata(). */
static inline int32_t ads_rdata_cont(int adc)
{
    uint8_t rx[3] = { 0 };

    cs_select(adc);
    if (adc_uses_pio(adc))
        pio_spi_read(g_pio, g_pio_sm, 0x00, rx, 3);
    else
        spi_read_blocking(adc_hw_spi, 0x00, rx, 3);
    cs_deselect(adc);

    /* Big-endian 24-bit signed */
    int32_t val = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | rx[2];
    if (val & 0x800000)
        val |= (int32_t)0xFF000000;
    return val;
}

/* Bitmask with all NUM_ADCS bits set */
#define ALL_ADCS_MASK  ((1u << NUM_ADCS) - 1u)   /* 0x3F for 6 ADCs */

/* Send WREG INPMUX only — no STOP, no START.
 * In continuous mode, writing INPMUX resets the digital filter (ADS124S08
 * datasheet: "Writing to any register resets the digital filter").
 * The LL filter then settles in 1 conversion cycle (sinc1 = 250 µs at 4000 SPS).
 * The modulator stays warm (already running from initial START in init),
 * so there is no modulator startup delay (~223 µs savings vs single-shot). */
static inline void ads_set_mux(int adc, uint8_t mux)
{
    uint8_t cmd[3] = {
        (uint8_t)(ADS_CMD_WREG | ADS_REG_INPMUX),  /* WREG at 0x02 */
        0x00,                                   /* write 1 register */
        mux                                     /* INPMUX value */
    };
    cs_select(adc);
    if (adc_uses_pio(adc))
        pio_spi_write(g_pio, g_pio_sm, cmd, 3);
    else
        spi_write_blocking(adc_hw_spi, cmd, 3);
    cs_deselect(adc);
}

int adc_manager_scan(uint8_t *out_data)
{
    for (int ch = 0; ch < (int)CHANNELS_PER_ADC; ch++) {
        uint8_t mux = (uint8_t)((ch << 4) | 0x0C);

        /* 1. Set INPMUX on all 6 ADCs (WREG resets filter, no STOP/START).
         *    3 bytes per ADC.  Modulator stays warm in continuous mode. */
        for (int a = 0; a < (int)NUM_ADCS; a++)
            ads_set_mux(a, mux);

        /* 2. Wait for ALL DRDY pins to assert LOW.
         *    Single-shot + LL filter at 4000 SPS: 250 µs per conversion.
         *    Use gpio_get_all() for a single register read per iteration. */
        uint8_t ready = 0;
        absolute_time_t deadline = make_timeout_time_us(2000);
        while (ready != ALL_ADCS_MASK && !time_reached(deadline)) {
            uint32_t pins = gpio_get_all();
            for (int a = 0; a < (int)NUM_ADCS; a++) {
                if (!(ready & (1u << a)) &&
                    !(pins & (1u << adc_drdy_pins[a])))
                    ready |= (uint8_t)(1u << a);
            }
        }

        /* 3. Read conversion data from all ADCs. */
        for (int a = 0; a < (int)NUM_ADCS; a++) {
            int idx = (a * (int)CHANNELS_PER_ADC + ch) * (int)BYTES_PER_SAMPLE;
            if (ready & (1u << a)) {
                int32_t val = ads_rdata_cont(a);
                out_data[idx]     = (uint8_t)((val >> 16) & 0xFF);
                out_data[idx + 1] = (uint8_t)((val >> 8)  & 0xFF);
                out_data[idx + 2] = (uint8_t)(val & 0xFF);
            } else {
                /* DRDY timeout — fill with zero */
                out_data[idx]     = 0;
                out_data[idx + 1] = 0;
                out_data[idx + 2] = 0;
            }
        }
    }

    return (int)NUM_ADCS;
}

/* =========================================================================
 * Scan Timing Diagnostic
 *
 * Measures per-channel breakdown: command send time, DRDY wait time,
 * data read time.  Called from CMD handler via "scan_timing" command.
 * ========================================================================= */

void adc_manager_scan_timing(char *out, int out_size)
{
    int pos = 0;
    #define TM_APPEND(...) do { \
        pos += snprintf(out + pos, (out_size - pos > 0) ? (size_t)(out_size - pos) : 0, __VA_ARGS__); \
    } while(0)

    uint8_t dummy_data[DATA_SIZE];
    uint64_t t_total_start = time_us_64();
    uint64_t cmd_total = 0, drdy_total = 0, read_total = 0;

    for (int ch = 0; ch < (int)CHANNELS_PER_ADC; ch++) {
        uint8_t mux = (uint8_t)((ch << 4) | 0x0C);

        uint64_t t0 = time_us_64();
        for (int a = 0; a < (int)NUM_ADCS; a++)
            ads_set_mux(a, mux);
        uint64_t t1 = time_us_64();

        uint8_t ready = 0;
        absolute_time_t deadline = make_timeout_time_us(2000);
        while (ready != ALL_ADCS_MASK && !time_reached(deadline)) {
            uint32_t pins = gpio_get_all();
            for (int a = 0; a < (int)NUM_ADCS; a++) {
                if (!(ready & (1u << a)) &&
                    !(pins & (1u << adc_drdy_pins[a])))
                    ready |= (uint8_t)(1u << a);
            }
        }
        uint64_t t2 = time_us_64();

        for (int a = 0; a < (int)NUM_ADCS; a++) {
            int idx = (a * (int)CHANNELS_PER_ADC + ch) * (int)BYTES_PER_SAMPLE;
            if (ready & (1u << a)) {
                int32_t val = ads_rdata_cont(a);
                dummy_data[idx]     = (uint8_t)((val >> 16) & 0xFF);
                dummy_data[idx + 1] = (uint8_t)((val >> 8)  & 0xFF);
                dummy_data[idx + 2] = (uint8_t)(val & 0xFF);
            } else {
                dummy_data[idx] = dummy_data[idx+1] = dummy_data[idx+2] = 0;
            }
        }
        uint64_t t3 = time_us_64();

        cmd_total  += (t1 - t0);
        drdy_total += (t2 - t1);
        read_total += (t3 - t2);

        if (ch < 3 || ch == 11) {
            TM_APPEND("[TIMING] ch%02d: cmd=%llu drdy=%llu read=%llu total=%llu us  ready=0x%02X\n",
                      ch, (unsigned long long)(t1-t0), (unsigned long long)(t2-t1),
                      (unsigned long long)(t3-t2), (unsigned long long)(t3-t0), ready);
        }
    }

    uint64_t t_total = time_us_64() - t_total_start;
    TM_APPEND("[TIMING] TOTALS: cmd=%llu drdy=%llu read=%llu frame=%llu us\n",
              (unsigned long long)cmd_total, (unsigned long long)drdy_total,
              (unsigned long long)read_total, (unsigned long long)t_total);
    TM_APPEND("[TIMING] Effective FPS: %.1f\n", 1000000.0 / (double)t_total);

    (void)dummy_data;
    #undef TM_APPEND
}

/* =========================================================================
 * Utility functions
 * ========================================================================= */

int adc_manager_read_id(int adc_index)
{
    if (adc_index < 0 || adc_index >= (int)NUM_ADCS)
        return -1;

    uint8_t id = 0xFF;
    if (ads_rreg(adc_index, ADS_REG_ID, &id, 1) > 0)
        return (int)id;
    return -1;
}

bool adc_manager_drdy(int adc_index)
{
    if (adc_index < 0 || adc_index >= (int)NUM_ADCS)
        return false;
    return !gpio_get(adc_drdy_pins[adc_index]);  /* active low */
}

/* =========================================================================
 * Bit-bang SPI diagnostic — bypasses PIO to test MISO hardware
 * ========================================================================= */

int bitbang_spi1_test(char *out, int out_size)
{
    int pos = 0;
    #define BB_APPEND(...) do { \
        pos += snprintf(out + pos, (out_size - pos > 0) ? (size_t)(out_size - pos) : 0, __VA_ARGS__); \
    } while(0)

    BB_APPEND("[BB] Bit-bang SPI test on ADC3 (GP14=SCK GP15=MOSI GP16=MISO CS=GP11)\n");

    /* 1. Read GP16 pad level right now (while PIO is active) */
    BB_APPEND("[BB] GP16 raw pad (PIO active): %d\n", gpio_get(SPI1_MISO));
    BB_APPEND("[BB] PIO instance: PIO%d SM%d offset=%d\n",
              pio_get_index(g_pio), g_pio_sm, g_pio_off);

    /* 2. Disable PIO state machine */
    pio_sm_set_enabled(g_pio, g_pio_sm, false);

    /* 3. Reconfigure GP14/15/16 as plain GPIO */
    gpio_init(SPI1_SCK);
    gpio_set_dir(SPI1_SCK, GPIO_OUT);
    gpio_put(SPI1_SCK, 0);   /* SCK idle LOW (CPOL=0) */

    gpio_init(SPI1_MOSI);
    gpio_set_dir(SPI1_MOSI, GPIO_OUT);
    gpio_put(SPI1_MOSI, 0);

    gpio_init(SPI1_MISO);
    gpio_set_dir(SPI1_MISO, GPIO_IN);
    gpio_pull_up(SPI1_MISO);

    sleep_us(10);

    /* Read GP16 with pull-up, no CS — should read 1 if floating/pulled up */
    BB_APPEND("[BB] GP16 no CS (pull-up): %d\n", gpio_get(SPI1_MISO));

    /* 4. Assert CS for ADC3 */
    gpio_put(adc_cs_pins[3], 0);
    sleep_us(10);
    BB_APPEND("[BB] GP16 after CS3 assert: %d\n", gpio_get(SPI1_MISO));

    /* 5. Bit-bang RREG ID: send 0x20 0x00 (read 1 reg at addr 0) */
    /* SPI Mode 1: data changes on rising edge, sampled on falling edge */
    uint8_t tx_bytes[2] = { 0x20, 0x00 };
    for (int b = 0; b < 2; b++) {
        uint8_t byte = tx_bytes[b];
        for (int bit = 7; bit >= 0; bit--) {
            gpio_put(SPI1_MOSI, (byte >> bit) & 1);
            sleep_us(2);
            gpio_put(SPI1_SCK, 1);   /* rising edge */
            sleep_us(2);
            gpio_put(SPI1_SCK, 0);   /* falling edge */
            sleep_us(2);
        }
    }

    /* 6. Wait for ADC to decode RREG */
    sleep_us(50);

    /* 7. Clock out 1 byte while reading MISO bit-by-bit */
    uint8_t rx_val = 0;
    char bits_str[9] = {0};
    gpio_put(SPI1_MOSI, 0);
    for (int bit = 7; bit >= 0; bit--) {
        sleep_us(2);
        gpio_put(SPI1_SCK, 1);   /* rising edge */
        sleep_us(2);
        gpio_put(SPI1_SCK, 0);   /* falling edge */
        sleep_us(1);
        int miso = gpio_get(SPI1_MISO);
        rx_val |= (uint8_t)(miso << bit);
        bits_str[7 - bit] = miso ? '1' : '0';
    }
    bits_str[8] = '\0';

    gpio_put(adc_cs_pins[3], 1);   /* deassert CS */

    BB_APPEND("[BB] ID readback: 0x%02X  bits=%s\n", rx_val, bits_str);

    /* 8. Test with pull-down to check if ADC drives MISO */
    gpio_pull_down(SPI1_MISO);
    sleep_us(10);
    BB_APPEND("[BB] GP16 pull-down (no CS): %d\n", gpio_get(SPI1_MISO));
    gpio_pull_up(SPI1_MISO);

    /* 9. Also try ADC4 and ADC5 ID reads */
    for (int a = 4; a <= 5; a++) {
        gpio_put(adc_cs_pins[a], 0);
        sleep_us(10);
        /* Send RREG ID */
        for (int b = 0; b < 2; b++) {
            uint8_t byte = tx_bytes[b];
            for (int bit = 7; bit >= 0; bit--) {
                gpio_put(SPI1_MOSI, (byte >> bit) & 1);
                sleep_us(2);
                gpio_put(SPI1_SCK, 1);
                sleep_us(2);
                gpio_put(SPI1_SCK, 0);
                sleep_us(2);
            }
        }
        sleep_us(50);
        uint8_t rx2 = 0;
        gpio_put(SPI1_MOSI, 0);
        for (int bit = 7; bit >= 0; bit--) {
            sleep_us(2);
            gpio_put(SPI1_SCK, 1);
            sleep_us(2);
            gpio_put(SPI1_SCK, 0);
            sleep_us(1);
            if (gpio_get(SPI1_MISO))
                rx2 |= (uint8_t)(1 << bit);
        }
        gpio_put(adc_cs_pins[a], 1);
        BB_APPEND("[BB] ADC%d ID: 0x%02X\n", a, rx2);
    }

    /* 10. Restore PIO SPI */
    float clkdiv = (float)clock_get_hz(clk_sys) / (PIO_SPI_BAUDRATE * 4.0f);
    pio_spi_cpha1_program_init(g_pio, g_pio_sm, g_pio_off,
                                SPI1_MOSI, SPI1_MISO, SPI1_SCK, clkdiv);
    BB_APPEND("[BB] PIO SPI restored\n");

    #undef BB_APPEND
    return pos;
}
