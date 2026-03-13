/*
 * ota_server.c — Reliable dual-bank OTA with automatic rollback (Pico 2 W)
 *
 * See ota_server.h for protocol, flash layout, and architecture description.
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"

#include "ota_server.h"

/* =========================================================================
 * CRC32 (standard polynomial 0xEDB88320, same as zlib/PNG)
 * Table stored in flash (.rodata) to save 1 KB of RAM.
 * ========================================================================= */

static const uint32_t g_crc32_table[256] = {
    0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,0xE963A535,0x9E6495A3,
    0x0EDB8832,0x79DCB8A4,0xE0D5E91E,0x97D2D988,0x09B64C2B,0x7EB17CBD,0xE7B82D07,0x90BF1D91,
    0x1DB71064,0x6AB020F2,0xF3B97148,0x84BE41DE,0x1ADAD47D,0x6DDDE4EB,0xF4D4B551,0x83D385C7,
    0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,0x14015C4F,0x63066CD9,0xFA0F3D63,0x8D080DF5,
    0x3B6E20C8,0x4C69105E,0xD56041E4,0xA2677172,0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,
    0x35B5A8FA,0x42B2986C,0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,
    0x26D930AC,0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F4B5,0x56B3C423,0xCFBA9599,0xB8BDA50F,
    0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,0x2F6F7C87,0x58684C11,0xC1611DAB,0xB6662D3D,
    0x76DC4190,0x01DB7106,0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,0x9FBFE4A5,0xE8B8D433,
    0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6A0DBB,0x086D3D2D,0x91646C97,0xE6635C01,
    0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,
    0x65B0D9C6,0x12B7E950,0x8BBEB8EA,0xFCB9887C,0x62DD1DDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,
    0x4DB26158,0x3AB551CE,0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,0xA4D1C46D,0xD3D6F4FB,
    0x4369E96A,0x346ED9FC,0xAD678846,0xDA60B8D0,0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,
    0x5005713C,0x270241AA,0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,0xCE61E49F,
    0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8B4,0x59B33D17,0x2EB40D81,0xB7BD5C3B,0xC0BA6CAD,
    0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,0xEAD54739,0x9DD277AF,0x04DB2615,0x73DC1683,
    0xE3630B12,0x94643B84,0x0D6D6A3E,0x7A6A5AA8,0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,
    0xF00F9344,0x8708A3D2,0x1E01F268,0x6906C2FE,0xF762575D,0x806567CB,0x196C3671,0x6E6B06E7,
    0xFED41B76,0x89D32BE0,0x10DA7A5A,0x67DD4ACC,0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,
    0xD6D6A3E8,0xA1D1937E,0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
    0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA867DF55,0x316E8EEF,0x4669BE79,
    0xCB61B38C,0xBC66831A,0x256FD2A0,0x5268E236,0xCC0C7795,0xBB0B4703,0x220216B9,0x5505262F,
    0xC5BA3BBE,0xB2BD0B28,0x2BB45A92,0x5CB36A04,0xC2D7FFA7,0xB5D0CF31,0x2CD99E8B,0x5BDEAE1D,
    0x9B64C2B0,0xEC63F226,0x756AA39C,0x026D930A,0x9C0906A9,0xEB0E363F,0x72076785,0x05005713,
    0x95BF4A82,0xE2B87A14,0x7BB12BAE,0x0CB61B38,0x92D28E9B,0xE5D5BE0D,0x7CDCEFB7,0x0BDBDF21,
    0x86D3D2D4,0xF1D4E242,0x68DDB3F8,0x1FDA836E,0x81BE16CD,0xF6B9265B,0x6FB077E1,0x18B74777,
    0x88085AE6,0xFF0F6A70,0x66063BCA,0x11010B5C,0x8F659EFF,0xF862AE69,0x616BFFD3,0x166CCF45,
    0xA00AE278,0xD70DD2EE,0x4E048354,0x3903B3C2,0xA7672661,0xD06016F7,0x4969474D,0x3E6E77DB,
    0xAED16A4A,0xD9D65ADC,0x40DF0B66,0x37D83BF0,0xA9BCAE53,0xDEBB9EC5,0x47B2CF7F,0x30B5FFE9,
    0xBDBDF21C,0xCABAC28A,0x53B39330,0x24B4A3A6,0xBAD03605,0xCDD70693,0x54DE5729,0x23D967BF,
    0xB3667A2E,0xC4614AB8,0x5D681B02,0x2A6F2B94,0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D,
};

static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, uint32_t len)
{
    crc = ~crc;
    for (uint32_t i = 0; i < len; i++)
        crc = g_crc32_table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

/* Compute CRC32 over a region of flash via XIP, 4 KB at a time */
uint32_t ota_crc32_flash(uint32_t flash_offset, uint32_t size)
{
    uint32_t crc = 0;
    const uint8_t *base = (const uint8_t *)(OTA_XIP_BASE + flash_offset);
    uint32_t done = 0;
    while (done < size) {
        uint32_t chunk = size - done;
        if (chunk > FLASH_SECTOR_SIZE) chunk = FLASH_SECTOR_SIZE;
        crc = crc32_update(crc, base + done, chunk);
        done += chunk;
    }
    return crc;
}

/* =========================================================================
 * UF2 block layout (512 bytes per block, 256 bytes payload)
 * ========================================================================= */

#define UF2_MAGIC1        0x0A324655u
#define UF2_MAGIC2        0x9E5D5157u
#define UF2_MAGIC_END     0x0AB16F30u
#define UF2_BLOCK_SIZE    512u
#define UF2_PAYLOAD       256u
#define UF2_FLAG_NOFLASH  0x00000001u

typedef struct __attribute__((packed)) {
    uint32_t magic1;
    uint32_t magic2;
    uint32_t flags;
    uint32_t target_addr;
    uint32_t payload_size;
    uint32_t block_no;
    uint32_t num_blocks;
    uint32_t file_size;
    uint8_t  data[476];
    uint32_t magic_end;
} uf2_block_t;

_Static_assert(sizeof(uf2_block_t) == 512, "uf2_block_t must be 512 bytes");
_Static_assert(UF2_PAYLOAD == FLASH_PAGE_SIZE, "UF2 payload must match flash page size");

/* =========================================================================
 * Receive ring buffer (16 KB = TCP_WND from lwipopts.h)
 * ========================================================================= */

#define OTA_RXBUF_SIZE  (16u * 1024u)
#define OTA_RXBUF_MASK  (OTA_RXBUF_SIZE - 1u)

static uint8_t  g_rx_buf[OTA_RXBUF_SIZE];
static uint32_t g_rx_head = 0;
static uint32_t g_rx_tail = 0;

static inline uint32_t rx_count(void)  { return g_rx_head - g_rx_tail; }
static inline uint32_t rx_space(void)  { return OTA_RXBUF_SIZE - rx_count(); }

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

static void rx_drop(uint32_t n) { g_rx_tail += n; }

static uint8_t rx_peek(uint32_t offset)
{
    return g_rx_buf[(g_rx_tail + offset) & OTA_RXBUF_MASK];
}

static void rx_reset(void) { g_rx_head = g_rx_tail = 0; }

/* =========================================================================
 * OTA receive state machine
 * ========================================================================= */

typedef enum {
    OTA_IDLE   = 0,
    OTA_HDR    = 1,
    OTA_ERASE  = 2,
    OTA_DATA   = 3,
    OTA_DONE   = 4,
    OTA_ERROR  = 5,
} ota_phase_t;

static struct tcp_pcb *g_ota_listen_pcb = NULL;
static struct tcp_pcb *g_ota_conn_pcb   = NULL;

static ota_phase_t g_ota_phase         = OTA_IDLE;
static uint32_t    g_ota_expect        = 0;
static uint32_t    g_ota_data_done     = 0;
static uint32_t    g_ota_flash_hi      = 0;
static bool        g_ota_v2            = false;  /* V2 protocol (CRC exchange) */

static uint32_t g_ota_total_sectors    = 0;
static uint32_t g_ota_erase_idx        = 0;

/* Running CRC32 computed during OTA_DATA phase */
static uint32_t g_ota_running_crc      = 0;

/* =========================================================================
 * Metadata helpers
 * ========================================================================= */

static void meta_read(ota_meta_t *out)
{
    const ota_meta_t *xip =
        (const ota_meta_t *)(OTA_XIP_BASE + OTA_META_FLASH_OFFSET);
    memcpy(out, xip, sizeof(ota_meta_t));
}

static void meta_write(const ota_meta_t *m)
{
    /* Metadata is 32 bytes.  We erase the 4 KB sector and program
     * only the first 256-byte page.  Rest stays 0xFF (erased). */
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, FLASH_PAGE_SIZE);
    memcpy(page, m, sizeof(ota_meta_t));

    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(OTA_META_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(OTA_META_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq);
}

static void meta_clear(void)
{
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(OTA_META_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(irq);
}

/* =========================================================================
 * ota_send — send a text line over the OTA TCP connection
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
 * Core 1 lockout for flash safety
 * ========================================================================= */

static bool g_core1_halted = false;

#define OTA_DIAG(val)  do { watchdog_hw->scratch[3] = (val); } while (0)

static void ota_halt_core1(void)
{
    if (g_core1_halted) return;
    printf("[OTA] Locking out Core 1 for flash operations...\n");

    watchdog_hw->scratch[0] = 0xDEADBEEFu;
    watchdog_hw->scratch[1] = 0x03u;
    watchdog_hw->scratch[2] = to_ms_since_boot(get_absolute_time());
    OTA_DIAG(0xAA000001u);

    watchdog_update();
    multicore_lockout_start_blocking();

    OTA_DIAG(0xAA000002u);
    g_core1_halted = true;
    printf("[OTA] Core 1 locked out — flash operations safe.\n");
}

/* =========================================================================
 * Flash copy helpers (sector-by-sector)
 * ========================================================================= */

/* Copy `size` bytes from one flash region to another (sector-aligned).
 * Runs entirely from SRAM so it survives erasing the active firmware region.
 *
 * CRITICAL: this function must NOT call memcpy, printf, or any other
 * function that lives in flash.  When dst is Bank A (the active firmware),
 * those code pages get erased mid-copy, causing a hard fault.
 * - memcpy  → replaced with inline byte loop (in SRAM)
 * - printf  → removed; callers print progress before/after
 * - flash_range_erase / flash_range_program are in bootrom (safe)
 */
static void __no_inline_not_in_flash_func(flash_copy)(
    uint32_t dst_offset, uint32_t src_offset, uint32_t size)
{
    static uint8_t page_buf[FLASH_PAGE_SIZE];       /* 256 bytes — saves 3840 B RAM */
    uint32_t sectors = (size + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE;
    uint32_t pages_per_sector = FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE;  /* 16 */

    for (uint32_t s = 0; s < sectors; s++) {
        uint32_t sec_off = s * FLASH_SECTOR_SIZE;

        /* Erase the destination sector */
        flash_range_erase(dst_offset + sec_off, FLASH_SECTOR_SIZE);

        /* Program page-by-page from XIP source */
        for (uint32_t p = 0; p < pages_per_sector; p++) {
            uint32_t page_off = sec_off + p * FLASH_PAGE_SIZE;
            const volatile uint8_t *src = (const volatile uint8_t *)(
                OTA_XIP_BASE + src_offset + page_off);

            /* Inline byte copy — MUST NOT call memcpy (it's in flash!) */
            for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++)
                page_buf[i] = src[i];

            flash_range_program(dst_offset + page_off, page_buf, FLASH_PAGE_SIZE);
        }
    }
}

/* =========================================================================
 * copy_and_reboot — SRAM trampoline for safe Bank A flash operations
 *
 * CRITICAL FIX: When flash_copy() overwrites Bank A (the active firmware),
 * the calling function's code changes.  If flash_copy() returns to its
 * caller (which lives in flash), the CPU resumes at an address that now
 * contains completely different code → hard fault.
 *
 * This trampoline runs entirely from SRAM:
 *   1. Calls flash_copy() (also SRAM)
 *   2. Triggers an immediate watchdog reset via direct register writes
 *   3. NEVER returns to flash code
 *
 * The watchdog reset reboots the Pico cleanly.  On the next boot,
 * ota_check_and_apply() re-reads the metadata and continues the
 * state machine from the appropriate state.
 * ========================================================================= */

static void __no_inline_not_in_flash_func(copy_and_reboot)(
    uint32_t dst_offset, uint32_t src_offset, uint32_t size)
{
    flash_copy(dst_offset, src_offset, size);

    /* Trigger immediate watchdog reset.
     * All register accesses are to hardware addresses — no flash dependency.
     * watchdog_hw is defined in hardware/structs/watchdog.h as a register pointer. */
    watchdog_hw->ctrl = 0;          /* disable watchdog first            */
    watchdog_hw->load = 100;        /* 100 µs timeout (minimum)          */
    watchdog_hw->ctrl = (1u << 30); /* set ENABLE bit → fires in 100 µs */
    while (true) { __wfi(); }       /* wait for reset (never returns)    */
}

/* =========================================================================
 * process_uf2_block — write one UF2 block to Bank B staging area
 * ========================================================================= */

static bool process_uf2_block(void)
{
    uf2_block_t blk;
    rx_read((uint8_t *)&blk, UF2_BLOCK_SIZE);

    if (blk.magic1 != UF2_MAGIC1 ||
        blk.magic2 != UF2_MAGIC2 ||
        blk.magic_end != UF2_MAGIC_END) {
        printf("[OTA] Block %lu: bad UF2 magic\n", (unsigned long)blk.block_no);
        return false;
    }

    if (blk.flags & UF2_FLAG_NOFLASH) return true;
    if (blk.payload_size != UF2_PAYLOAD) return true;
    if (blk.target_addr < OTA_XIP_BASE) return true;

    uint32_t active_offset = blk.target_addr - OTA_XIP_BASE;

    /* Skip blocks targeting beyond Bank A (metadata or other banks) */
    if (active_offset >= OTA_BANK_A_SIZE) return true;

    /* Write to Bank B at the corresponding offset */
    uint32_t staging_page = OTA_BANK_B_OFFSET + active_offset;

    uint32_t saved_irq = save_and_disable_interrupts();
    flash_range_program(staging_page, blk.data, UF2_PAYLOAD);
    restore_interrupts(saved_irq);

    uint32_t end = active_offset + UF2_PAYLOAD;
    if (end > g_ota_flash_hi) g_ota_flash_hi = end;

    return true;
}

/* =========================================================================
 * ota_check_and_apply — called at startup BEFORE Core 1 launch
 *
 * State machine:
 *   STAGED     → verify CRC → backup A→C → apply B→A → CONFIRMING → reboot
 *   CONFIRMING → attempts < MAX → let boot continue (fw calls ota_boot_confirmed)
 *   CONFIRMING → attempts ≥ MAX → REVERTING
 *   REVERTING  → copy C→A → IDLE → reboot
 * ========================================================================= */

void ota_check_and_apply(void)
{
    ota_meta_t meta;
    meta_read(&meta);

    /* No valid metadata — nothing to do */
    if (meta.magic != OTA_META_MAGIC) {
        if (meta.magic != 0xFFFFFFFFu) {
            printf("[OTA] Stale/corrupt metadata (magic=0x%08lX) — clearing.\n",
                   (unsigned long)meta.magic);
            meta_clear();
        }
        return;
    }

    /* Check metadata version */
    if (meta.version != OTA_META_VERSION) {
        printf("[OTA] Unknown metadata version %lu — clearing.\n",
               (unsigned long)meta.version);
        meta_clear();
        return;
    }

    printf("[OTA] Metadata: state=%lu staged=%lu backup=%lu attempts=%lu\n",
           (unsigned long)meta.state,
           (unsigned long)meta.staged_size,
           (unsigned long)meta.backup_size,
           (unsigned long)meta.boot_attempts);

    /* ── STAGED: new firmware ready to apply ────────────────────────────── */
    if (meta.state == OTA_STATE_STAGED) {
        /* Must be a watchdog reboot (OTA server triggers watchdog reset) */
        if (!watchdog_caused_reboot()) {
            printf("[OTA] STAGED but not watchdog reboot (POR) — clearing.\n");
            meta_clear();
            return;
        }

        /* Validate staged image size */
        if (meta.staged_size == 0 || meta.staged_size > OTA_BANK_A_SIZE) {
            printf("[OTA] Bad staged_size=%lu — clearing.\n",
                   (unsigned long)meta.staged_size);
            meta_clear();
            return;
        }

        /* Verify CRC32 of Bank B */
        printf("[OTA] Verifying Bank B CRC32 (%lu bytes)...\n",
               (unsigned long)meta.staged_size);
        uint32_t actual_crc = ota_crc32_flash(OTA_BANK_B_OFFSET, meta.staged_size);
        if (actual_crc != meta.staged_crc) {
            printf("[OTA] CRC MISMATCH! expected=0x%08lX actual=0x%08lX — aborting.\n",
                   (unsigned long)meta.staged_crc, (unsigned long)actual_crc);
            meta_clear();
            return;
        }
        printf("[OTA] CRC32 verified: 0x%08lX\n", (unsigned long)actual_crc);

        /* Phase 1: Backup A → C */
        printf("\n============================================================\n");
        printf("  OTA: Backing up current firmware (A → C)\n");
        printf("============================================================\n");
        meta.state = OTA_STATE_BACKUP;
        meta_write(&meta);

        /* Compute size of current Bank A firmware.
         * Scan backwards from end of Bank A to find last non-0xFF sector. */
        uint32_t backup_size = 0;
        {
            uint32_t sectors = OTA_BANK_A_SIZE / FLASH_SECTOR_SIZE;
            for (uint32_t s = sectors; s > 0; s--) {
                const uint8_t *p = (const uint8_t *)(
                    OTA_XIP_BASE + (s - 1) * FLASH_SECTOR_SIZE);
                bool blank = true;
                for (uint32_t i = 0; i < FLASH_SECTOR_SIZE; i++) {
                    if (p[i] != 0xFF) { blank = false; break; }
                }
                if (!blank) {
                    backup_size = s * FLASH_SECTOR_SIZE;
                    break;
                }
            }
        }
        if (backup_size == 0) backup_size = FLASH_SECTOR_SIZE; /* at least 1 sector */

        printf("[OTA] Backing up %lu bytes...\n", (unsigned long)backup_size);
        flash_copy(OTA_BANK_C_OFFSET, OTA_BANK_A_OFFSET, backup_size);

        uint32_t backup_crc = ota_crc32_flash(OTA_BANK_C_OFFSET, backup_size);
        printf("[OTA] Backup CRC32: 0x%08lX\n", (unsigned long)backup_crc);

        /* Phase 2: Apply B → A
         *
         * CRITICAL: flash_copy overwrites Bank A (the running firmware).
         * After copy, all flash code at old offsets is invalid.
         * We MUST use copy_and_reboot() which runs entirely from SRAM
         * and reboots via direct register writes — never returns to flash.
         *
         * Metadata is set to APPLYING before the copy.  On the next boot,
         * ota_check_and_apply() sees APPLYING and verifies the CRC to
         * determine if the copy completed successfully. */
        printf("\n============================================================\n");
        printf("  OTA: Applying new firmware (B → A)  (%lu bytes)\n",
               (unsigned long)meta.staged_size);
        printf("  Using SRAM trampoline — will reboot after copy.\n");
        printf("============================================================\n");
        meta.state       = OTA_STATE_APPLYING;
        meta.backup_size = backup_size;
        meta.backup_crc  = backup_crc;
        meta_write(&meta);

        /* NEVER RETURNS — reboots from SRAM after flash copy. */
        copy_and_reboot(OTA_BANK_A_OFFSET, OTA_BANK_B_OFFSET, meta.staged_size);
        /* unreachable */
    }

    /* ── CONFIRMING: new firmware is booting ─────────────────────────────── */
    if (meta.state == OTA_STATE_CONFIRMING) {
        meta.boot_attempts++;
        printf("[OTA] CONFIRMING: boot attempt %lu/%lu\n",
               (unsigned long)meta.boot_attempts,
               (unsigned long)OTA_MAX_BOOT_ATTEMPTS);

        if (meta.boot_attempts >= OTA_MAX_BOOT_ATTEMPTS) {
            printf("[OTA] Max boot attempts reached — REVERTING!\n");
            meta.state = OTA_STATE_REVERTING;
            meta_write(&meta);
            /* Fall through to REVERTING handler */
        } else {
            /* Update attempt count and let boot continue.
             * If the firmware is good, it will call ota_boot_confirmed(). */
            meta_write(&meta);

            /* ARM the watchdog with a generous 30 s timeout so the rollback
             * mechanism works even if the new firmware crashes before it gets
             * a chance to enable the normal 8 s watchdog.  The main loop's
             * watchdog_update() calls will keep feeding it.  If the firmware
             * never reaches main(), the 30 s watchdog fires → reboot →
             * boot_attempts increments → eventually REVERTING. */
            watchdog_enable(30000u, true);   /* 30 s, pause_on_debug=true */
            printf("[OTA] Continuing boot (watchdog armed 30 s) — "
                   "firmware must call ota_boot_confirmed().\n");
            return;
        }
    }

    /* ── REVERTING: restore Bank C → Bank A ──────────────────────────────
     *
     * Two sub-cases:
     *   (a) First entry: Bank A has new/corrupted firmware, need to copy C→A.
     *       Use copy_and_reboot (SRAM trampoline) since C→A overwrites Bank A.
     *       The reboot lands us back here for sub-case (b).
     *   (b) Second entry (after copy_and_reboot): Bank A now has the backup.
     *       Verify CRC, clear metadata, reboot normally.
     *
     * We distinguish the sub-cases by checking if Bank A already matches
     * the backup CRC.  If it does, the copy already succeeded (sub-case b).
     * ──────────────────────────────────────────────────────────────────── */
    if (meta.state == OTA_STATE_REVERTING) {
        if (meta.backup_size == 0 || meta.backup_size > OTA_BANK_A_SIZE) {
            printf("[OTA] REVERT: bad backup_size=%lu — clearing metadata.\n",
                   (unsigned long)meta.backup_size);
            meta_clear();
            return;
        }

        /* Verify backup in Bank C is intact */
        printf("[OTA] Verifying backup CRC32 (Bank C)...\n");
        uint32_t bcrc = ota_crc32_flash(OTA_BANK_C_OFFSET, meta.backup_size);
        if (bcrc != meta.backup_crc) {
            printf("[OTA] Backup CRC MISMATCH (0x%08lX vs 0x%08lX) — cannot revert!\n",
                   (unsigned long)bcrc, (unsigned long)meta.backup_crc);
            meta_clear();
            return;  /* boot whatever is in Bank A and hope for the best */
        }

        /* Check if Bank A already has the backup (sub-case b: copy already done) */
        uint32_t acrc = ota_crc32_flash(OTA_BANK_A_OFFSET, meta.backup_size);
        if (acrc == meta.backup_crc) {
            /* Bank A already matches backup — copy was completed on prev boot. */
            printf("[OTA] Bank A already matches backup CRC — restore verified.\n");
            meta_clear();
            printf("[OTA] Metadata cleared. Rebooting into restored firmware...\n");
            watchdog_enable(1u, false);
            while (true) {}
        }

        /* Sub-case (a): Bank A needs to be restored from Bank C.
         * Use SRAM trampoline — NEVER returns to flash. */
        printf("\n============================================================\n");
        printf("  OTA: REVERTING — Restoring backup (C → A)  (%lu bytes)\n",
               (unsigned long)meta.backup_size);
        printf("  Using SRAM trampoline — will reboot after copy.\n");
        printf("============================================================\n");

        /* NEVER RETURNS — reboots from SRAM after flash copy. */
        copy_and_reboot(OTA_BANK_A_OFFSET, OTA_BANK_C_OFFSET, meta.backup_size);
        /* unreachable */
    }

    /* ── BACKUP: interrupted mid-backup ──────────────────────────────────── */
    if (meta.state == OTA_STATE_BACKUP) {
        /* We were backing up A→C when interrupted. A is still intact. */
        printf("[OTA] Backup was interrupted — Bank A still intact. Clearing.\n");
        meta_clear();
        return;  /* Boot normally from intact Bank A */
    }

    /* ── APPLYING: copy_and_reboot() completed B→A, then rebooted ────────
     *
     * The copy_and_reboot trampoline always reboots after flash_copy,
     * so we arrive here on the NEXT boot.  We need to verify that the
     * copy was successful by checking the CRC of Bank A.
     *
     * Success → set CONFIRMING, reboot into new firmware
     * Failure → set REVERTING to restore the backup from Bank C
     * ──────────────────────────────────────────────────────────────────── */
    if (meta.state == OTA_STATE_APPLYING) {
        printf("[OTA] APPLYING: verifying Bank A CRC after copy...\n");

        if (meta.staged_size == 0 || meta.staged_size > OTA_BANK_A_SIZE) {
            printf("[OTA] Bad staged_size=%lu in APPLYING state.\n",
                   (unsigned long)meta.staged_size);
            /* Can't verify — try to revert if possible */
            if (meta.backup_size > 0 && meta.backup_size <= OTA_BANK_A_SIZE) {
                meta.state = OTA_STATE_REVERTING;
                meta_write(&meta);
                printf("[OTA] Set REVERTING. Rebooting...\n");
                watchdog_enable(1u, false);
                while (true) {}
            }
            meta_clear();
            return;
        }

        uint32_t applied_crc = ota_crc32_flash(OTA_BANK_A_OFFSET, meta.staged_size);
        if (applied_crc == meta.staged_crc) {
            /* Copy was successful! Transition to CONFIRMING. */
            printf("[OTA] Bank A CRC32 verified: 0x%08lX — apply succeeded!\n",
                   (unsigned long)applied_crc);
            meta.state         = OTA_STATE_CONFIRMING;
            meta.boot_attempts = 0;
            meta_write(&meta);
            printf("[OTA] Rebooting into new firmware (CONFIRMING)...\n");
            watchdog_enable(1u, false);
            while (true) {}
        } else {
            /* Copy failed or was interrupted — revert from backup. */
            printf("[OTA] CRC MISMATCH: expected=0x%08lX actual=0x%08lX\n",
                   (unsigned long)meta.staged_crc, (unsigned long)applied_crc);
            if (meta.backup_size > 0 && meta.backup_size <= OTA_BANK_A_SIZE) {
                meta.state = OTA_STATE_REVERTING;
                meta_write(&meta);
                printf("[OTA] Set REVERTING. Rebooting...\n");
                watchdog_enable(1u, false);
                while (true) {}
            }
            printf("[OTA] No valid backup — clearing. May need USB recovery.\n");
            meta_clear();
            return;
        }
    }

    /* ── IDLE or unknown: nothing to do ──────────────────────────────────── */
    if (meta.state != OTA_STATE_IDLE) {
        printf("[OTA] Unknown state %lu — clearing.\n",
               (unsigned long)meta.state);
        meta_clear();
    }
}

/* =========================================================================
 * ota_boot_confirmed — called from main() after WiFi is fully operational
 * ========================================================================= */

void ota_boot_confirmed(void)
{
    ota_meta_t meta;
    meta_read(&meta);

    if (meta.magic != OTA_META_MAGIC || meta.state != OTA_STATE_CONFIRMING) {
        return;  /* Not in CONFIRMING state — nothing to do */
    }

    printf("\n============================================================\n");
    printf("  OTA: Boot CONFIRMED — firmware is healthy!\n");
    printf("============================================================\n");
    printf("[OTA] Clearing metadata → IDLE.\n");

    /* Clear metadata to IDLE (blank = no OTA pending = IDLE).
     * CRITICAL: Core 1 is running from XIP flash at this point.
     * flash_range_erase() disables the XIP bus, which would hard-fault
     * Core 1 if it tries to fetch instructions during the erase.
     * We must park Core 1 for the duration of the flash operation. */
    multicore_lockout_start_blocking();
    meta_clear();
    multicore_lockout_end_blocking();
}

/* =========================================================================
 * TCP callbacks
 * ========================================================================= */

static void ota_reset_state(void)
{
    /* Release Core 1 from multicore lockout if it was halted for flash ops.
     * Without this, a client disconnect during ERASE/DATA phase would leave
     * Core 1 permanently parked — no more ADC frames produced. */
    if (g_core1_halted) {
        multicore_lockout_end_blocking();
        printf("[OTA] Core 1 released from lockout.\n");
    }

    g_ota_phase         = OTA_IDLE;
    g_ota_expect        = 0;
    g_ota_data_done     = 0;
    g_ota_flash_hi      = 0;
    g_ota_total_sectors  = 0;
    g_ota_erase_idx      = 0;
    g_ota_v2             = false;
    g_ota_running_crc    = 0;
    g_core1_halted       = false;
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
        printf("[OTA] Client disconnected\n");
        if (g_ota_phase != OTA_DONE) ota_reset_state();
        g_ota_conn_pcb = NULL;
        tcp_close(tpcb);
        return ERR_OK;
    }

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
    return ERR_OK;
}

static err_t ota_tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg; (void)err;

    if (g_ota_conn_pcb != NULL) {
        tcp_close(newpcb);
        return ERR_OK;
    }

    printf("[OTA] Client connected from %s\n",
           ip4addr_ntoa(&newpcb->remote_ip));

    g_ota_conn_pcb = newpcb;
    ota_reset_state();
    g_ota_phase = OTA_HDR;

    tcp_err(g_ota_conn_pcb,  ota_tcp_err_cb);
    tcp_recv(g_ota_conn_pcb, ota_tcp_recv_cb);

    return ERR_OK;
}

/* =========================================================================
 * ota_server_init
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
    printf("[OTA] Listening on port %u (reliable dual-bank, V2)\n",
           (unsigned)OTA_PORT);
}

/* =========================================================================
 * ota_server_poll
 * ========================================================================= */

void ota_server_poll(void)
{
    if (!g_ota_conn_pcb) return;
    if (g_ota_phase == OTA_IDLE ||
        g_ota_phase == OTA_DONE ||
        g_ota_phase == OTA_ERROR) return;

    /* ── Phase HDR: parse "OTAV1 {size}\r\n" or "OTAV2 {size}\r\n" ─────── */

    if (g_ota_phase == OTA_HDR) {
        uint32_t avail      = rx_count();
        uint32_t search_max = (avail < 128u) ? avail : 128u;
        uint32_t nl_pos     = UINT32_MAX;

        for (uint32_t i = 0; i < search_max; i++) {
            if (rx_peek(i) == '\n') { nl_pos = i; break; }
        }

        if (nl_pos == UINT32_MAX) return;

        uint32_t hdr_len = nl_pos + 1u;
        char hdr[65];
        uint32_t n = (hdr_len < 64u) ? hdr_len : 64u;
        for (uint32_t i = 0; i < n; i++) hdr[i] = (char)rx_peek(i);
        hdr[n] = '\0';
        while (n > 0 && (hdr[n - 1] == '\r' || hdr[n - 1] == '\n'))
            hdr[--n] = '\0';

        printf("[OTA] Header: '%s'\n", hdr);

        rx_drop(hdr_len);

        /* Parse "OTAV1 {size}" or "OTAV2 {size}" */
        uint32_t sz = 0;
        bool is_v2 = false;
        if (strncmp(hdr, "OTAV2 ", 6) == 0) {
            is_v2 = true;
            char *end = NULL;
            unsigned long v = strtoul(hdr + 6, &end, 10);
            if (end && end != hdr + 6) sz = (uint32_t)v;
        } else if (strncmp(hdr, "OTAV1 ", 6) == 0) {
            char *end = NULL;
            unsigned long v = strtoul(hdr + 6, &end, 10);
            if (end && end != hdr + 6) sz = (uint32_t)v;
        }

        bool valid = (sz > 0u &&
                      (sz % UF2_BLOCK_SIZE) == 0u &&
                      sz <= (OTA_BANK_B_SIZE * 2u));  /* UF2 is 2x payload */

        cyw43_arch_lwip_begin();
        if (g_ota_conn_pcb) tcp_recved(g_ota_conn_pcb, (u16_t)hdr_len);
        if (!valid) {
            printf("[OTA] Bad header / invalid size %lu\n", (unsigned long)sz);
            ota_send("ERR bad header");
            g_ota_phase = OTA_ERROR;
        }
        cyw43_arch_lwip_end();

        if (valid) {
            g_ota_expect        = sz;
            g_ota_data_done     = 0;
            g_ota_v2            = is_v2;
            g_ota_running_crc   = 0;
            g_ota_total_sectors = ((sz / 2u) + FLASH_SECTOR_SIZE - 1u)
                                   / FLASH_SECTOR_SIZE + 1u;
            /* Cap to Bank B size */
            uint32_t max_sectors = OTA_BANK_B_SIZE / FLASH_SECTOR_SIZE;
            if (g_ota_total_sectors > max_sectors)
                g_ota_total_sectors = max_sectors;
            g_ota_erase_idx     = 0;
            printf("[OTA] Expecting %lu bytes (%lu blocks). "
                   "Pre-erasing %lu staging sectors before READY...\n",
                   (unsigned long)sz,
                   (unsigned long)(sz / UF2_BLOCK_SIZE),
                   (unsigned long)g_ota_total_sectors);
            g_ota_phase = OTA_ERASE;
        }

        return;
    }

    /* ── Phase ERASE: pre-erase one staging sector per poll call ─────────── */

    if (g_ota_phase == OTA_ERASE) {
        if (g_ota_erase_idx == 0) {
            ota_halt_core1();
        }

        if (g_ota_erase_idx < g_ota_total_sectors) {
            uint32_t off = OTA_BANK_B_OFFSET +
                           g_ota_erase_idx * FLASH_SECTOR_SIZE;
            OTA_DIAG(0xAA010000u | g_ota_erase_idx);
            uint32_t saved_irq = save_and_disable_interrupts();
            flash_range_erase(off, FLASH_SECTOR_SIZE);
            restore_interrupts(saved_irq);
            OTA_DIAG(0xAA020000u | g_ota_erase_idx);
            g_ota_erase_idx++;
            if ((g_ota_erase_idx % 8u) == 0u ||
                g_ota_erase_idx == g_ota_total_sectors) {
                printf("[OTA] Pre-erase %lu/%lu\n",
                       (unsigned long)g_ota_erase_idx,
                       (unsigned long)g_ota_total_sectors);
            }
        } else {
            printf("[OTA] Pre-erase complete. Sending READY.\n");
            cyw43_arch_lwip_begin();
            ota_send("READY");
            cyw43_arch_lwip_end();
            g_ota_phase = OTA_DATA;
        }
        return;
    }

    /* ── Phase DATA: process UF2 blocks one at a time ────────────────────── */

    if (g_ota_phase == OTA_DATA) {
        if (rx_count() < UF2_BLOCK_SIZE) return;

        bool ok = process_uf2_block();

        if (!ok) {
            printf("[OTA] UF2 error — aborting\n");
            cyw43_arch_lwip_begin();
            ota_send("ERR uf2 parse error");
            g_ota_phase = OTA_ERROR;
            cyw43_arch_lwip_end();
            return;
        }

        g_ota_data_done += UF2_BLOCK_SIZE;

        cyw43_arch_lwip_begin();
        if (g_ota_conn_pcb) {
            tcp_recved(g_ota_conn_pcb, (u16_t)UF2_BLOCK_SIZE);

            uint32_t block_num = g_ota_data_done / UF2_BLOCK_SIZE;
            if ((block_num % 32u) == 0u) {
                uint32_t pct = (g_ota_data_done * 100u) / g_ota_expect;
                ota_send("PROG %lu%%", (unsigned long)pct);
            }
        }
        cyw43_arch_lwip_end();

        if (g_ota_data_done < g_ota_expect) return;

        /* ── All data received — compute CRC, write metadata, reboot ────── */

        /* Round image size up to sector boundary */
        uint32_t image_size =
            (g_ota_flash_hi + FLASH_SECTOR_SIZE - 1u) &
            ~((uint32_t)(FLASH_SECTOR_SIZE - 1u));

        printf("[OTA] All %lu blocks received. image_size=%lu bytes.\n",
               (unsigned long)(g_ota_data_done / UF2_BLOCK_SIZE),
               (unsigned long)image_size);

        /* Compute CRC32 of staged image in Bank B */
        printf("[OTA] Computing CRC32 of staged image...\n");
        uint32_t staged_crc = ota_crc32_flash(OTA_BANK_B_OFFSET, image_size);
        printf("[OTA] Staged CRC32: 0x%08lX\n", (unsigned long)staged_crc);

        /* Send CRC to client (V2) */
        cyw43_arch_lwip_begin();
        if (g_ota_v2) {
            ota_send("CRC32 %08lX", (unsigned long)staged_crc);
        }
        cyw43_arch_lwip_end();

        /* Write metadata: state=STAGED with CRC */
        printf("[OTA] Writing metadata (state=STAGED)...\n");
        {
            ota_meta_t staged_meta;
            staged_meta.magic         = OTA_META_MAGIC;
            staged_meta.version       = OTA_META_VERSION;
            staged_meta.state         = OTA_STATE_STAGED;
            staged_meta.staged_size   = image_size;
            staged_meta.staged_crc    = staged_crc;
            staged_meta.backup_size   = 0;
            staged_meta.backup_crc    = 0;
            staged_meta.boot_attempts = 0;
            meta_write(&staged_meta);
        }

        printf("[OTA] Metadata written. Sending DONE and rebooting in 2 s...\n");

        cyw43_arch_lwip_begin();
        ota_send("DONE");
        g_ota_phase = OTA_DONE;
        cyw43_arch_lwip_end();

        sleep_ms(2000);

        /* Reboot via watchdog — ota_check_and_apply() handles STAGED state */
        watchdog_enable(1u, false);
        while (true) {}
    }
}
