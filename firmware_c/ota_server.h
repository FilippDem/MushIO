/*
 * ota_server.h — True wireless OTA firmware update (Pico 2 W / RP2350)
 *
 * Protocol (port 9002):
 *   Client → "OTAV1 {uf2_bytes}\r\n"
 *   Server → "READY\r\n"
 *   Client → raw UF2 bytes  ({uf2_bytes} total)
 *   Server → "PROG {pct}%\r\n"   (every 32 blocks ≈ 16 KB)
 *   Server → "DONE\r\n"           (metadata written, rebooting)
 *   Server → "ERR {reason}\r\n"   (on failure)
 *
 * Flash layout (Pico 2 W, 4 MB flash):
 *   0x000000 – 0x1FEFFF  Active firmware  (2 MB − 4 KB)
 *   0x1FF000 – 0x1FFFFF  OTA metadata     (4 KB sector)
 *   0x200000 – 0x3FFFFF  Staging area     (2 MB)
 *
 * Receive flow:
 *   The lwIP recv callback copies incoming bytes into a 16 KB ring buffer
 *   WITHOUT calling tcp_recved.  ota_server_poll() (called from the main loop)
 *   processes one 512-byte UF2 block at a time, writes to staging flash with
 *   multicore lockout, then calls tcp_recved to advance the TCP window.
 *   This gives natural flow control without stalling the lwIP IRQ.
 *
 * Apply flow (at next boot, before Core 1 launch):
 *   ota_check_and_apply() reads the metadata sector.  If valid, it copies
 *   the staging area to the active firmware region sector-by-sector, clears
 *   the metadata, and reboots via watchdog.  The copy function runs from RAM
 *   (__no_inline_not_in_flash_func) to survive its own sector erases.
 */

#pragma once

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Flash layout constants
 * These are raw flash offsets (not XIP addresses).
 * XIP address = OTA_XIP_BASE + flash_offset.
 * ------------------------------------------------------------------------- */

#define OTA_XIP_BASE            0x10000000u   /* RP2350 XIP window base   */
#define OTA_META_FLASH_OFFSET   0x001FF000u   /* metadata sector offset   */
#define OTA_STAGE_FLASH_OFFSET  0x00200000u   /* staging area start       */
#define OTA_META_MAGIC          0x4F544155u   /* 'OTAU' little-endian     */
#define OTA_PORT                9002u

/* -------------------------------------------------------------------------
 * OTA metadata record — written at OTA_META_FLASH_OFFSET when an update is
 * ready.  The struct is padded to exactly one 4 KB flash sector so a single
 * flash_range_program() call writes the whole thing.
 * ------------------------------------------------------------------------- */

typedef struct __attribute__((packed)) {
    uint32_t magic;         /* OTA_META_MAGIC when a valid OTA is pending    */
    uint32_t image_size;    /* bytes of firmware in staging (sector-aligned) */
    uint8_t  _pad[4088];    /* pad to 4096 bytes (one flash sector)          */
} ota_meta_t;

/* Compile-time size check — catches accidental padding changes. */
_Static_assert(sizeof(ota_meta_t) == 4096, "ota_meta_t must be exactly 4096 bytes");

/* -------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */

/*
 * Call once in main(), BEFORE ring_init() and multicore_launch_core1().
 * If a valid OTA is pending, applies it (sector-by-sector copy from staging
 * to active region) and reboots via watchdog.  Never returns if OTA found.
 */
void ota_check_and_apply(void);

/*
 * Call after WiFi is connected.
 * Opens a TCP listening socket on OTA_PORT.
 */
void ota_server_init(void);

/*
 * Call every main-loop iteration.
 * Drains the receive ring buffer, processes UF2 blocks, writes to staging
 * flash (with multicore lockout), and calls tcp_recved to advance the window.
 */
void ota_server_poll(void);
