/*
 * pio_spi.h — PIO-based SPI transfer helpers for MushIO V1.0
 *
 * Provides write/read/write_read functions that match the signature style
 * of the Pico SDK spi_write_blocking / spi_read_blocking / spi_write_read_blocking.
 *
 * Used for the SPI1 bus (ADC3-5) because GP16 maps to hardware SPI0,
 * not SPI1.  PIO can drive SPI on any GPIO.
 */

#pragma once

#include "hardware/pio.h"
#include "pio_spi.pio.h"   /* generated from pio_spi.pio */

/* Full-duplex: send tx[] while receiving into rx[].
 * Either tx or rx may be NULL (send zeros / discard received). */
static inline void pio_spi_write_read(PIO pio, uint sm,
                                       const uint8_t *tx, uint8_t *rx,
                                       size_t len)
{
    size_t tx_i = 0, rx_i = 0;
    while (rx_i < len) {
        /* Fill TX FIFO (byte goes to bits [31:24] for MSB-first shifting) */
        if (tx_i < len && !pio_sm_is_tx_fifo_full(pio, sm)) {
            pio_sm_put(pio, sm, (uint32_t)(tx ? tx[tx_i] : 0) << 24);
            tx_i++;
        }
        /* Drain RX FIFO (shift_left + autopush@8 → byte in bits [7:0]) */
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t v = pio_sm_get(pio, sm);
            if (rx) rx[rx_i] = (uint8_t)v;
            rx_i++;
        }
    }
}

/* Write-only: send tx[], discard received data. */
static inline void pio_spi_write(PIO pio, uint sm,
                                  const uint8_t *tx, size_t len)
{
    pio_spi_write_read(pio, sm, tx, NULL, len);
}

/* Read-only: send repeated_tx byte while reading into rx[]. */
static inline void pio_spi_read(PIO pio, uint sm,
                                 uint8_t repeated_tx, uint8_t *rx, size_t len)
{
    size_t tx_i = 0, rx_i = 0;
    while (rx_i < len) {
        if (tx_i < len && !pio_sm_is_tx_fifo_full(pio, sm)) {
            pio_sm_put(pio, sm, (uint32_t)repeated_tx << 24);
            tx_i++;
        }
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            uint32_t v = pio_sm_get(pio, sm);
            if (rx) rx[rx_i] = (uint8_t)v;
            rx_i++;
        }
    }
}
