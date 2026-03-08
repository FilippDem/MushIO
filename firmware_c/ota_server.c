/*
 * ota_server.c — True wireless OTA firmware update (Pico 2 W / RP2350)
 *
 * See ota_server.h for protocol and architecture description.
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"

#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"

#include "ota_server.h"

/* =========================================================================
 * UF2 block layout (512 bytes per block, 256 bytes payload)
 * ========================================================================= */

#define UF2_MAGIC1        0x0A324655u   /* "UF2\n"   */
#define UF2_MAGIC2        0x9E5D5157u
#define UF2_MAGIC_END     0x0AB16F30u
#define UF2_BLOCK_SIZE    512u
#define UF2_PAYLOAD       256u          /* must equal FLASH_PAGE_SIZE (256) */
#define UF2_FLAG_NOFLASH  0x00000001u   /* skip this block                 */

/* UF2 block wire format */
typedef struct __attribute__((packed)) {
    uint32_t magic1;
    uint32_t magic2;
    uint32_t flags;
    uint32_t target_addr;   /* XIP address to flash this block to */
    uint32_t payload_size;  /* always 256 for standard UF2        */
    uint32_t block_no;
    uint32_t num_blocks;
    uint32_t file_size;     /* total file size or family ID        */
    uint8_t  data[476];     /* 256 payload + 220 bytes padding     */
    uint32_t magic_end;
} uf2_block_t;

_Static_assert(sizeof(uf2_block_t) == 512, "uf2_block_t must be 512 bytes");
_Static_assert(UF2_PAYLOAD == FLASH_PAGE_SIZE,
               "UF2 payload must match flash page size");

/* =========================================================================
 * Receive ring buffer  (16 KB = TCP_WND from lwipopts.h)
 *
 * The lwIP recv callback pushes bytes here WITHOUT calling tcp_recved.
 * ota_server_poll() pops one UF2 block at a time, writes it to staging
 * flash, then calls tcp_recved(512) to advance the TCP receive window.
 * This creates natural backpressure: the sender can have at most TCP_WND
 * bytes in flight before stalling while we process.
 * ========================================================================= */

#define OTA_RXBUF_SIZE  (16u * 1024u)   /* power-of-2 for mask trick */
#define OTA_RXBUF_MASK  (OTA_RXBUF_SIZE - 1u)

static uint8_t  g_rx_buf[OTA_RXBUF_SIZE];
static uint32_t g_rx_head = 0;   /* cumulative write index */
static uint32_t g_rx_tail = 0;   /* cumulative read  index */

static inline uint32_t rx_count(void)                  { return g_rx_head - g_rx_tail; }
static inline uint32_t rx_space(void)                  { return OTA_RXBUF_SIZE - rx_count(); }

static void rx_push(const uint8_t *src, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
        g_rx_buf[g_rx_head++ & OTA_RXBUF_MASK] = src[i];
}

static void rx_read(uint8_t *dst, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
        dst[i] = g_rx_buf[g_rx_tail++ & OTA_RXBUF_MASK];
}

static void rx_drop(uint32_t n)   { g_rx_tail += n; }

static uint8_t rx_peek(uint32_t offset)
{
    return g_rx_buf[(g_rx_tail + offset) & OTA_RXBUF_MASK];
}

static void rx_reset(void) { g_rx_head = g_rx_tail = 0; }

/* =========================================================================
 * OTA state machine
 * ========================================================================= */

typedef enum {
    OTA_IDLE      = 0,
    OTA_HDR       = 1,   /* waiting for "OTAV1 {size}\r\n" header           */
    OTA_ERASE     = 2,   /* pre-erasing staging sectors (one per poll call) */
    OTA_DATA      = 3,   /* receiving UF2 data bytes                        */
    OTA_DONE      = 4,   /* all data received, metadata written             */
    OTA_ERROR     = 5,   /* protocol or flash error                         */
} ota_phase_t;

static struct tcp_pcb *g_ota_listen_pcb = NULL;
static struct tcp_pcb *g_ota_conn_pcb   = NULL;

static ota_phase_t g_ota_phase         = OTA_IDLE;
static uint32_t    g_ota_expect        = 0;   /* expected UF2 bytes          */
static uint32_t    g_ota_data_done     = 0;   /* UF2 bytes processed so far  */
static uint32_t    g_ota_flash_hi      = 0;   /* highest active-area byte written */

/* Pre-erase state: all staging sectors are erased BEFORE READY is sent,
 * one sector per ota_server_poll() call so WiFi is serviced between erases.
 * During OTA_DATA only fast flash_range_program() calls are made (~0.5 ms),
 * which never cause WiFi interrupts to be starved. */
static uint32_t g_ota_total_sectors   = 0;   /* sectors to pre-erase          */
static uint32_t g_ota_erase_idx       = 0;   /* next sector index to erase    */

/* Static metadata buffer used at OTA completion — avoids 4 KB on stack. */
static ota_meta_t g_ota_meta;

/* =========================================================================
 * ota_send — send a text line over the OTA TCP connection.
 * MUST be called with the lwIP lock held (cyw43_arch_lwip_begin).
 * ========================================================================= */

static void ota_send(const char *fmt, ...)
{
    if (!g_ota_conn_pcb) return;
    static char line[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line) - 2, fmt, ap);
    va_end(ap);
    if (n <= 0 || n >= (int)(sizeof(line) - 2)) return;
    line[n++] = '\r';
    line[n++] = '\n';
    tcp_write(g_ota_conn_pcb, line, (u16_t)n, TCP_WRITE_FLAG_COPY);
    tcp_output(g_ota_conn_pcb);
}

/* =========================================================================
 * process_uf2_block
 *
 * Reads one 512-byte UF2 block from the rx ring buffer and, if it targets
 * the active firmware region, writes the 256-byte payload to the staging
 * area in flash.
 *
 * Called from ota_server_poll() — main loop context, lwIP lock NOT held.
 * flash_range_erase / flash_range_program are wrapped in multicore lockout
 * to safely pause Core 1 (which executes from flash via XIP).
 *
 * Returns true on success, false on UF2 magic / format error.
 * ========================================================================= */

static bool process_uf2_block(void)
{
    uf2_block_t blk;
    rx_read((uint8_t *)&blk, UF2_BLOCK_SIZE);

    /* Validate magic words */
    if (blk.magic1 != UF2_MAGIC1 ||
        blk.magic2 != UF2_MAGIC2 ||
        blk.magic_end != UF2_MAGIC_END) {
        printf("[OTA] Block %lu: bad UF2 magic\n", (unsigned long)blk.block_no);
        return false;
    }

    /* Skip no-flash blocks (e.g. info blocks) */
    if (blk.flags & UF2_FLAG_NOFLASH) return true;

    /* Only handle standard 256-byte payloads */
    if (blk.payload_size != UF2_PAYLOAD) return true;

    /* Target must be within the XIP window */
    if (blk.target_addr < OTA_XIP_BASE) return true;

    uint32_t active_offset = blk.target_addr - OTA_XIP_BASE;

    /* Skip blocks that land in or beyond the metadata / staging regions */
    if (active_offset >= OTA_META_FLASH_OFFSET) return true;

    /* Staging flash offset for this 256-byte page */
    uint32_t staging_page = OTA_STAGE_FLASH_OFFSET + active_offset;

    /* Program 256-byte page into pre-erased staging area (~0.5 ms, WiFi safe) */
    multicore_lockout_start_blocking();
    flash_range_program(staging_page, blk.data, UF2_PAYLOAD);
    multicore_lockout_end_blocking();

    /* Track highest byte written (active-area coordinates) */
    uint32_t end = active_offset + UF2_PAYLOAD;
    if (end > g_ota_flash_hi) g_ota_flash_hi = end;

    return true;
}

/* =========================================================================
 * apply_ota_image
 *
 * Copies the staging area to the active firmware region, one 4 KB sector at
 * a time.  Must run from SRAM because it erases the sectors from which the
 * CPU is currently executing code.
 *
 * Called at startup BEFORE Core 1 is launched → no multicore lockout needed.
 * ========================================================================= */

static void __no_inline_not_in_flash_func(apply_ota_image)(uint32_t image_size)
{
    static uint8_t sector_buf[FLASH_SECTOR_SIZE];

    uint32_t sectors = (image_size + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE;

    printf("[OTA-APPLY] Copying %lu sectors (%lu bytes) from staging to active...\n",
           (unsigned long)sectors, (unsigned long)image_size);

    for (uint32_t s = 0; s < sectors; s++) {
        uint32_t flash_offs   = s * FLASH_SECTOR_SIZE;
        uint32_t staging_offs = OTA_STAGE_FLASH_OFFSET + flash_offs;

        /* Read staging sector into RAM via XIP (XIP is active here) */
        const uint8_t *src = (const uint8_t *)(OTA_XIP_BASE + staging_offs);
        memcpy(sector_buf, src, FLASH_SECTOR_SIZE);

        /* Erase and program active sector (flash briefly goes offline) */
        flash_range_erase(flash_offs, FLASH_SECTOR_SIZE);
        flash_range_program(flash_offs, sector_buf, FLASH_SECTOR_SIZE);

        if ((s % 16u) == 0u) {
            uint32_t pct = (s * 100u) / sectors;
            printf("[OTA-APPLY] %lu%%  (sector %lu/%lu)\n",
                   (unsigned long)pct, (unsigned long)s, (unsigned long)sectors);
        }
    }

    printf("[OTA-APPLY] 100%%  Done.\n");
}

/* =========================================================================
 * ota_check_and_apply — public, called from main() at startup
 * ========================================================================= */

void ota_check_and_apply(void)
{
    /* Read metadata via XIP — safe before any flash operations */
    const ota_meta_t *meta =
        (const ota_meta_t *)(OTA_XIP_BASE + OTA_META_FLASH_OFFSET);

    if (meta->magic != OTA_META_MAGIC) {
        return;   /* No pending OTA — normal boot */
    }

    uint32_t image_size = meta->image_size;

    if (image_size == 0u || image_size > OTA_META_FLASH_OFFSET) {
        printf("[OTA-APPLY] Metadata corrupt (image_size=%lu) — clearing.\n",
               (unsigned long)image_size);
        flash_range_erase(OTA_META_FLASH_OFFSET, FLASH_SECTOR_SIZE);
        return;
    }

    printf("\n");
    printf("============================================================\n");
    printf("  OTA: Applying firmware update (%lu bytes)\n",
           (unsigned long)image_size);
    printf("============================================================\n");

    /* Copy staging → active (runs from SRAM) */
    apply_ota_image(image_size);

    /* Clear metadata so we don't re-apply on the next boot */
    printf("[OTA-APPLY] Clearing metadata sector...\n");
    flash_range_erase(OTA_META_FLASH_OFFSET, FLASH_SECTOR_SIZE);

    printf("[OTA-APPLY] Rebooting into new firmware...\n");
    watchdog_enable(1u, false);   /* 1 ms timeout → immediate reset */
    while (true) { /* wait for watchdog reset */ }
}

/* =========================================================================
 * TCP callbacks
 * ========================================================================= */

static void ota_reset_state(void)
{
    g_ota_phase         = OTA_IDLE;
    g_ota_expect        = 0;
    g_ota_data_done     = 0;
    g_ota_flash_hi      = 0;
    g_ota_total_sectors = 0;
    g_ota_erase_idx     = 0;
    rx_reset();
}

static void ota_tcp_err_cb(void *arg, err_t err)
{
    (void)arg;
    printf("[OTA] TCP error %d — resetting\n", (int)err);
    g_ota_conn_pcb = NULL;
    ota_reset_state();
}

static err_t ota_tcp_recv_cb(void *arg, struct tcp_pcb *tpcb,
                              struct pbuf *p, err_t err)
{
    (void)arg; (void)tpcb; (void)err;

    if (p == NULL) {
        /* Remote host closed the connection */
        printf("[OTA] Client disconnected\n");
        if (g_ota_phase != OTA_DONE) ota_reset_state();
        g_ota_conn_pcb = NULL;
        tcp_close(tpcb);
        return ERR_OK;
    }

    /* Copy pbuf chain into ring buffer.
     * Intentionally do NOT call tcp_recved here.
     * ota_server_poll() calls tcp_recved after processing each block,
     * creating backpressure that prevents the rx ring from overflowing. */
    struct pbuf *q = p;
    while (q) {
        uint32_t space   = rx_space();
        uint32_t to_copy = (q->len <= space) ? q->len : space;
        if (to_copy > 0)
            rx_push((const uint8_t *)q->payload, to_copy);
        if (to_copy < q->len)
            printf("[OTA] WARN rx overflow: dropped %u bytes\n",
                   (unsigned)(q->len - to_copy));
        q = q->next;
    }

    pbuf_free(p);
    /* NO tcp_recved call here — flow control */
    return ERR_OK;
}

static err_t ota_tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg; (void)err;

    if (g_ota_conn_pcb != NULL) {
        /* Already serving an update — reject */
        tcp_close(newpcb);
        return ERR_OK;
    }

    printf("[OTA] Client connected from %s\n",
           ip4addr_ntoa(&newpcb->remote_ip));

    g_ota_conn_pcb = newpcb;
    ota_reset_state();
    g_ota_phase = OTA_HDR;   /* Start in header-reading state */

    tcp_err(g_ota_conn_pcb,  ota_tcp_err_cb);
    tcp_recv(g_ota_conn_pcb, ota_tcp_recv_cb);

    return ERR_OK;
}

/* =========================================================================
 * ota_server_init — public, call after WiFi is connected
 * ========================================================================= */

void ota_server_init(void)
{
    cyw43_arch_lwip_begin();

    g_ota_listen_pcb = tcp_new();
    if (!g_ota_listen_pcb) {
        cyw43_arch_lwip_end();
        printf("[OTA] tcp_new() failed\n");
        return;
    }

    tcp_bind(g_ota_listen_pcb, IP_ADDR_ANY, (u16_t)OTA_PORT);
    g_ota_listen_pcb = tcp_listen(g_ota_listen_pcb);
    tcp_accept(g_ota_listen_pcb, ota_tcp_accept_cb);

    cyw43_arch_lwip_end();
    printf("[OTA] Listening on port %u\n", (unsigned)OTA_PORT);
}

/* =========================================================================
 * ota_server_poll — public, call every main-loop iteration
 * ========================================================================= */

void ota_server_poll(void)
{
    if (!g_ota_conn_pcb) return;
    if (g_ota_phase == OTA_IDLE ||
        g_ota_phase == OTA_DONE ||
        g_ota_phase == OTA_ERROR) return;

    /* --------------------------------------------------------------------- */
    /* Phase HDR: parse "OTAV1 {size}\r\n"                                   */
    /* --------------------------------------------------------------------- */

    if (g_ota_phase == OTA_HDR) {
        /* Look for a newline within the first 128 bytes of the ring */
        uint32_t avail      = rx_count();
        uint32_t search_max = (avail < 128u) ? avail : 128u;
        uint32_t nl_pos     = UINT32_MAX;

        for (uint32_t i = 0; i < search_max; i++) {
            if (rx_peek(i) == '\n') { nl_pos = i; break; }
        }

        if (nl_pos == UINT32_MAX) return;  /* header not yet complete */

        /* Copy header bytes to a local buffer */
        uint32_t hdr_len = nl_pos + 1u;
        char hdr[65];
        uint32_t n = (hdr_len < 64u) ? hdr_len : 64u;
        for (uint32_t i = 0; i < n; i++) hdr[i] = (char)rx_peek(i);
        hdr[n] = '\0';
        /* Strip CR/LF */
        while (n > 0 && (hdr[n - 1] == '\r' || hdr[n - 1] == '\n'))
            hdr[--n] = '\0';

        printf("[OTA] Header: '%s'\n", hdr);

        /* Consume header bytes from ring */
        rx_drop(hdr_len);

        /* Parse "OTAV1 {decimal_size}" */
        uint32_t sz = 0;
        if (strncmp(hdr, "OTAV1 ", 6) == 0) {
            char *end = NULL;
            unsigned long v = strtoul(hdr + 6, &end, 10);
            if (end && end != hdr + 6) sz = (uint32_t)v;
        }

        /* Validate: must be a non-zero multiple of UF2_BLOCK_SIZE,
         * and must fit in the staging area (2 MB max) */
        bool valid = (sz > 0u &&
                      (sz % UF2_BLOCK_SIZE) == 0u &&
                      sz <= (OTA_STAGE_FLASH_OFFSET /* 2 MB */));

        /* Acknowledge header bytes; send READY or ERR */
        cyw43_arch_lwip_begin();
        if (g_ota_conn_pcb) tcp_recved(g_ota_conn_pcb, (u16_t)hdr_len);
        if (!valid) {
            printf("[OTA] Bad header / invalid size %lu\n", (unsigned long)sz);
            ota_send("ERR bad header");
            g_ota_phase = OTA_ERROR;
        }
        cyw43_arch_lwip_end();

        if (valid) {
            g_ota_expect       = sz;
            g_ota_data_done    = 0;
            /* Worst-case sector count: UF2 payload density = 256/512 bytes.
             * +1 sector margin for non-contiguous block layouts. */
            g_ota_total_sectors = ((sz / 2u) + FLASH_SECTOR_SIZE - 1u)
                                  / FLASH_SECTOR_SIZE + 1u;
            g_ota_erase_idx    = 0;
            printf("[OTA] Expecting %lu bytes (%lu blocks). "
                   "Pre-erasing %lu staging sectors before READY...\n",
                   (unsigned long)sz,
                   (unsigned long)(sz / UF2_BLOCK_SIZE),
                   (unsigned long)g_ota_total_sectors);
            g_ota_phase = OTA_ERASE;   /* erase before sending READY */
        }

        return;
    }

    /* --------------------------------------------------------------------- */
    /* Phase ERASE: pre-erase one staging sector per poll call               */
    /*                                                                       */
    /* flash_range_erase() disables interrupts for ~40 ms — long enough to  */
    /* starve the CYW43 WiFi SPI IRQ if called during data streaming.        */
    /* By pre-erasing here (before READY is sent) the PC is idle so brief   */
    /* WiFi outages are harmless.  OTA_DATA then uses only the fast          */
    /* flash_range_program() (~0.5 ms) which is WiFi-safe.                  */
    /* --------------------------------------------------------------------- */

    if (g_ota_phase == OTA_ERASE) {
        if (g_ota_erase_idx < g_ota_total_sectors) {
            uint32_t off = OTA_STAGE_FLASH_OFFSET +
                           g_ota_erase_idx * FLASH_SECTOR_SIZE;
            multicore_lockout_start_blocking();
            flash_range_erase(off, FLASH_SECTOR_SIZE);
            multicore_lockout_end_blocking();
            g_ota_erase_idx++;
            if ((g_ota_erase_idx % 8u) == 0u ||
                g_ota_erase_idx == g_ota_total_sectors) {
                printf("[OTA] Pre-erase %lu/%lu\n",
                       (unsigned long)g_ota_erase_idx,
                       (unsigned long)g_ota_total_sectors);
            }
        } else {
            /* All sectors erased — tell the client we are ready */
            printf("[OTA] Pre-erase complete. Sending READY.\n");
            cyw43_arch_lwip_begin();
            ota_send("READY");
            cyw43_arch_lwip_end();
            g_ota_phase = OTA_DATA;
        }
        return;
    }

    /* --------------------------------------------------------------------- */
    /* Phase DATA: process UF2 blocks one at a time                          */
    /* --------------------------------------------------------------------- */

    if (g_ota_phase == OTA_DATA) {
        if (rx_count() < UF2_BLOCK_SIZE) return;

        bool ok = process_uf2_block();   /* reads 512 bytes, writes to flash */

        if (!ok) {
            printf("[OTA] UF2 error — aborting\n");
            cyw43_arch_lwip_begin();
            ota_send("ERR uf2 parse error");
            g_ota_phase = OTA_ERROR;
            cyw43_arch_lwip_end();
            return;
        }

        g_ota_data_done += UF2_BLOCK_SIZE;

        /* Advance TCP receive window (allow sender to send more) */
        cyw43_arch_lwip_begin();
        if (g_ota_conn_pcb) {
            tcp_recved(g_ota_conn_pcb, (u16_t)UF2_BLOCK_SIZE);

            /* Progress: every 32 blocks (~16 KB) */
            uint32_t block_num = g_ota_data_done / UF2_BLOCK_SIZE;
            if ((block_num % 32u) == 0u) {
                uint32_t pct = (g_ota_data_done * 100u) / g_ota_expect;
                ota_send("PROG %lu%%", (unsigned long)pct);
            }
        }
        cyw43_arch_lwip_end();

        /* Check completion */
        if (g_ota_data_done < g_ota_expect) return;

        /* ----------------------------------------------------------------- */
        /* All UF2 data received — write metadata and reboot                 */
        /* ----------------------------------------------------------------- */

        /* Round image size up to the next sector boundary */
        uint32_t image_size =
            (g_ota_flash_hi + FLASH_SECTOR_SIZE - 1u) &
            ~((uint32_t)(FLASH_SECTOR_SIZE - 1u));

        printf("[OTA] All %lu blocks received. "
               "image_size=%lu bytes. Writing metadata...\n",
               (unsigned long)(g_ota_data_done / UF2_BLOCK_SIZE),
               (unsigned long)image_size);

        /* Prepare metadata in RAM first (avoids VLA on stack) */
        memset(&g_ota_meta, 0xFF, sizeof(g_ota_meta));
        g_ota_meta.magic      = OTA_META_MAGIC;
        g_ota_meta.image_size = image_size;

        /* Write metadata — needs multicore lockout (Core 1 is running) */
        multicore_lockout_start_blocking();
        flash_range_erase(OTA_META_FLASH_OFFSET, FLASH_SECTOR_SIZE);
        flash_range_program(OTA_META_FLASH_OFFSET,
                            (const uint8_t *)&g_ota_meta,
                            FLASH_SECTOR_SIZE);
        multicore_lockout_end_blocking();

        printf("[OTA] Metadata written. Sending DONE and rebooting in 2 s...\n");

        cyw43_arch_lwip_begin();
        ota_send("DONE");
        g_ota_phase = OTA_DONE;
        cyw43_arch_lwip_end();

        /* Give the TCP stack time to flush the DONE message */
        sleep_ms(2000);

        /* Reboot via watchdog — ota_check_and_apply() will apply on next boot */
        watchdog_enable(1u, false);
        while (true) { /* await watchdog reset */ }
    }
}
