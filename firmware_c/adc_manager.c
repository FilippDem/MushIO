/*
 * adc_manager.c — ADS124S08 real ADC driver for MushIO V1.0
 *
 * Hardware:
 *   6 × ADS124S08 (24-bit delta-sigma ADCs) on two SPI buses
 *   ADC0..ADC2  →  SPI0 (GP2=SCK, GP3=MOSI, GP4=MISO)
 *   ADC3..ADC5  →  SPI1 (GP14=SCK, GP15=MOSI, GP12=MISO)
 *   Each ADC has its own CS and DRDY pin (see config.h)
 *   Shared: ADC_RESET_N (GP20), ADC_START (GP21), ADC_CLK_EN (GP22)
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

#include "config.h"
#include "adc_manager.h"

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

/* Which SPI instance each ADC uses: ADC0..2 → SPI0, ADC3..5 → SPI1 */
static spi_inst_t *adc_spi[NUM_ADCS];

/* SPI clock: 4 MHz — well within ADS124S08 max of ~8 MHz */
#define ADC_SPI_BAUDRATE    4000000

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
    spi_write_blocking(adc_spi[adc], &cmd, 1);
    cs_deselect(adc);
}

/* Read n registers starting at addr.  Returns bytes read, or 0 on error.
 * Uses a single spi_write_read_blocking transaction to avoid potential
 * timing gaps between separate write/read calls. */
int ads_rreg(int adc, uint8_t addr, uint8_t *buf, uint8_t n)
{
    /* Total transaction: 2 header bytes + n data bytes */
    uint8_t tx[20] = {0};   /* max 18 regs + 2 hdr = 20 */
    uint8_t rx[20] = {0};
    uint8_t total = (uint8_t)(2 + n);
    if (total > sizeof(tx)) total = sizeof(tx);

    tx[0] = (uint8_t)(ADS_CMD_RREG | (addr & 0x1F));
    tx[1] = (uint8_t)(n - 1);
    /* tx[2..] = 0x00 (NOP) to clock out register data */

    cs_select(adc);
    spi_write_read_blocking(adc_spi[adc], tx, rx, total);
    cs_deselect(adc);

    /* Register data starts at rx[2] (after the 2-byte header) */
    memcpy(buf, &rx[2], n);
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
    spi_write_blocking(adc_spi[adc], tx, total);
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
    spi_write_read_blocking(adc_spi[adc], tx, rx, 4);
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

    /* ---- Assign SPI instances ---- */
    for (int i = 0; i < (int)NUM_ADCS; i++)
        adc_spi[i] = (i < 3) ? spi0 : spi1;

    /* ---- Initialise SPI0 ---- */
    spi_init(spi0, ADC_SPI_BAUDRATE);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI0_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(SPI0_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_MISO, GPIO_FUNC_SPI);

    /* ---- Initialise SPI1 ---- */
    spi_init(spi1, ADC_SPI_BAUDRATE);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI1_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_MISO, GPIO_FUNC_SPI);

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
        ads_wreg1(i, ADS_REG_DATARATE,  0x0E);  /* 4000 SPS, sinc3 */
        ads_wreg1(i, ADS_REG_REF,       0x3A);  /* int ref on, REFP0/REFN0 */
        ads_wreg1(i, ADS_REG_SYS,       0x00);  /* no system monitor */
    }

    printf("[ADC] %d of %u ADCs alive\n", alive, (unsigned)NUM_ADCS);
    return (alive > 0);
}

/* =========================================================================
 * Scan — read 12 channels from each ADC
 * ========================================================================= */

int adc_manager_scan(uint8_t *out_data)
{
    int adc_ok = 0;

    for (int a = 0; a < (int)NUM_ADCS; a++) {
        for (int ch = 0; ch < (int)CHANNELS_PER_ADC; ch++) {
            /* Ensure ADC is stopped and DRDY is deasserted (HIGH) before
             * switching the MUX.  Without this, a lingering DRDY=LOW from
             * a previous conversion could cause ads_wait_drdy() to return
             * immediately with stale data. */
            ads_cmd(a, ADS_CMD_STOP);

            /* Set input mux: positive = AINch, negative = AINCOM (0x0C) */
            uint8_t mux = (uint8_t)((ch << 4) | 0x0C);
            ads_wreg1(a, ADS_REG_INPMUX, mux);

            /* Start conversion */
            ads_cmd(a, ADS_CMD_START);

            /* Wait for DRDY (conversion complete).
             * SINC3 at 4000 SPS: settling = 3/4000 = 750 µs.
             * Use 2 ms timeout for margin. */
            if (!ads_wait_drdy(a, 2000)) {
                /* Timeout — fill with zero */
                int idx = (a * (int)CHANNELS_PER_ADC + ch) * (int)BYTES_PER_SAMPLE;
                out_data[idx]     = 0;
                out_data[idx + 1] = 0;
                out_data[idx + 2] = 0;
                continue;
            }

            /* Read conversion result */
            int32_t val = ads_rdata(a);

            /* Pack as big-endian 24-bit into output buffer */
            int idx = (a * (int)CHANNELS_PER_ADC + ch) * (int)BYTES_PER_SAMPLE;
            out_data[idx]     = (uint8_t)((val >> 16) & 0xFF);
            out_data[idx + 1] = (uint8_t)((val >> 8)  & 0xFF);
            out_data[idx + 2] = (uint8_t)(val & 0xFF);
        }
        adc_ok++;
    }

    return adc_ok;
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
