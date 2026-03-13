/*
 * ota_server.h — Reliable dual-bank OTA with automatic rollback (Pico 2 W)
 *
 * Protocol (port 9002):
 *   Client → "OTAV2 {uf2_bytes}\r\n"     (or "OTAV1 ..." for legacy)
 *   Server → "READY\r\n"
 *   Client → raw UF2 bytes  ({uf2_bytes} total)
 *   Server → "PROG {pct}%\r\n"           (every 32 blocks ≈ 16 KB)
 *   Server → "CRC32 {hex}\r\n"           (V2 only: staged image CRC)
 *   Server → "DONE\r\n"                  (metadata written, rebooting)
 *   Server → "ERR {reason}\r\n"          (on failure)
 *
 * Flash layout (Pico 2 W, 4 MB flash):
 *   0x000000 – 0x0FEFFF  Bank A: Active firmware   (1 MB − 4 KB)
 *   0x0FF000 – 0x0FFFFF  OTA metadata              (4 KB sector)
 *   0x100000 – 0x1FFFFF  Bank B: Staging area       (1 MB)
 *   0x200000 – 0x2FFFFF  Bank C: Golden backup      (1 MB)
 *   0x300000 – 0x3FFFFF  Free                        (1 MB)
 *
 * Reliability features:
 *   1. CRC32 verification of staged image before applying
 *   2. Golden backup: Bank A copied to Bank C BEFORE overwriting
 *   3. Boot confirmation: new firmware must call ota_boot_confirmed()
 *      after WiFi + CMD + OTA + beacon are all operational
 *   4. Automatic rollback: if the watchdog fires 3 times without
 *      confirmation, Bank C (golden backup) is restored to Bank A
 *
 * Apply flow:
 *   ota_check_and_apply() runs at boot, BEFORE Core 1 launch.
 *   State machine in metadata drives the process:
 *     STAGED     → verify CRC → backup A→C → copy B→A → CONFIRMING → reboot
 *     CONFIRMING → boot_attempts < 3 → try to boot → ota_boot_confirmed()
 *     CONFIRMING → boot_attempts ≥ 3 → REVERTING → copy C→A → IDLE → reboot
 *     REVERTING  → copy C→A → IDLE → reboot  (crash recovery)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Flash layout constants
 * These are raw flash offsets (not XIP addresses).
 * XIP address = OTA_XIP_BASE + flash_offset.
 * ------------------------------------------------------------------------- */

#define OTA_XIP_BASE             0x10000000u   /* RP2350 XIP window base       */

/* Bank A: active firmware (boot location) */
#define OTA_BANK_A_OFFSET        0x00000000u
#define OTA_BANK_A_SIZE          0x000FF000u   /* 1 MB − 4 KB = 1,044,480      */

/* Metadata sector (between Bank A and Bank B) */
#define OTA_META_FLASH_OFFSET    0x000FF000u   /* 4 KB sector                   */

/* Bank B: staging area (receives new firmware via OTA) */
#define OTA_BANK_B_OFFSET        0x00100000u
#define OTA_BANK_B_SIZE          0x00100000u   /* 1 MB                          */

/* Bank C: golden backup (last confirmed working firmware) */
#define OTA_BANK_C_OFFSET        0x00200000u
#define OTA_BANK_C_SIZE          0x00100000u   /* 1 MB                          */

/* Magic values */
#define OTA_META_MAGIC           0x4F544155u   /* 'OTAU' — valid metadata       */
#define OTA_META_VERSION         2u            /* metadata format version        */

#define OTA_PORT                 9002u

/* -------------------------------------------------------------------------
 * OTA state machine
 * ------------------------------------------------------------------------- */

#define OTA_STATE_IDLE           0u   /* Normal operation, confirmed fw     */
#define OTA_STATE_STAGED         1u   /* New fw in Bank B, CRC verified     */
#define OTA_STATE_BACKUP         2u   /* Copying A → C (A still intact)     */
#define OTA_STATE_APPLYING       3u   /* Copying B → A (C has backup)       */
#define OTA_STATE_CONFIRMING     4u   /* New fw booted, awaiting confirm    */
#define OTA_STATE_REVERTING      5u   /* Restoring C → A (rolling back)     */

#define OTA_MAX_BOOT_ATTEMPTS    3u   /* Rollback after this many failures  */

/* -------------------------------------------------------------------------
 * OTA metadata record — written at OTA_META_FLASH_OFFSET.
 * Padded to exactly one 4 KB flash sector.
 * ------------------------------------------------------------------------- */

typedef struct __attribute__((packed)) {
    uint32_t magic;            /* OTA_META_MAGIC when valid                  */
    uint32_t version;          /* OTA_META_VERSION                           */
    uint32_t state;            /* OTA_STATE_*                                */
    uint32_t staged_size;      /* bytes of firmware in Bank B (sector-aligned)*/
    uint32_t staged_crc;       /* CRC32 of Bank B content [0..staged_size)   */
    uint32_t backup_size;      /* bytes of firmware in Bank C                */
    uint32_t backup_crc;       /* CRC32 of Bank C content [0..backup_size)   */
    uint32_t boot_attempts;    /* unconfirmed boot count (reset on confirm)  */
} ota_meta_t;

/* Compile-time size check — 32 bytes, fits in one flash page (256 bytes).
 * Written to the first page of the 4 KB metadata sector; rest stays 0xFF. */
_Static_assert(sizeof(ota_meta_t) == 32, "ota_meta_t must be exactly 32 bytes");

/* -------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */

/*
 * Call once in main(), BEFORE ring_init() and multicore_launch_core1().
 * Processes the OTA state machine.  May reboot (never returns) if:
 *   - STAGED: verifies CRC, backs up A→C, copies B→A, reboots
 *   - CONFIRMING + boot_attempts >= MAX: reverts C→A, reboots
 *   - REVERTING: continues/restarts C→A revert, reboots
 */
void ota_check_and_apply(void);

/*
 * Call after WiFi + CMD + OTA + beacon are all operational.
 * Marks the running firmware as confirmed (state → IDLE).
 * If state is not CONFIRMING, this is a no-op.
 */
void ota_boot_confirmed(void);

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

/*
 * Compute CRC32 over a region of flash (via XIP).
 * Used internally and exposed for diagnostics.
 */
uint32_t ota_crc32_flash(uint32_t flash_offset, uint32_t size);
