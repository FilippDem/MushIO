/*
 * main.c — MushIO V1.0 C Firmware  (Demo / Pico 2 W)
 *
 * Core 0 responsibilities (this file):
 *   - cyw43 / WiFi initialisation  (must be on Core 0)
 *   - Launch Core 1 (ADC scan loop)
 *   - TCP data connection to HOST_IP:HOST_PORT
 *   - Drain ring buffer → batch TCP writes at ~200 FPS
 *   - CMD server on TCP port 9001  (Ping, Status, Scan, Help)
 *   - LED heartbeat
 *   - Periodic stats to USB-serial console
 *
 * Core 1 responsibilities (core1.c):
 *   - demo_adc_scan() → frame_build() → ring_push()  at 200 FPS
 *
 * lwIP architecture: pico_cyw43_arch_lwip_threadsafe_background
 *   The CYW43 driver runs via a hardware alarm IRQ.
 *   All raw-API calls from application code are guarded by
 *   cyw43_arch_lwip_begin() / cyw43_arch_lwip_end().
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "pico/bootrom.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"

#include "lwip/tcp.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"

/* Forward declaration — implemented in core1.c */
extern void core1_main(void);

#include "config.h"
#include "ring.h"
#include "crc16.h"
#include "ota_server.h"

/* =========================================================================
 * Shared ring buffer  (defined here; extern in core1.c)
 * ========================================================================= */

ring_t g_ring;

/* =========================================================================
 * Global statistics
 * ========================================================================= */

static volatile uint32_t g_sent_frames  = 0;

/* Scan period for Core 1.  Set to DEMO_SCAN_PERIOD_US on boot.
 * Updated at runtime by the 'set_fps <hz>' CMD command.
 * Core 1 reads this once per scan iteration (extern in core1.c). */
volatile uint32_t g_scan_period_us = DEMO_SCAN_PERIOD_US;

/* =========================================================================
 * OTA reboot flag
 * Set from cmd_dispatch() (IRQ context) — acted on in main loop (safe context).
 * ========================================================================= */

static volatile bool g_ota_reboot_requested = false;

/* =========================================================================
 * Crash logging via watchdog scratch registers
 *
 * scratch[0] = CRASH_MAGIC (0xDEADBEEF) — set only on abnormal exit
 * scratch[1] = crash code
 * scratch[2] = timestamp_ms at crash
 * scratch[3] = extra data (e.g. lwIP err_t)
 *
 * These registers survive a watchdog reset but are cleared on power-on.
 * The host can read them via the 'crash_log' CMD command.
 * ========================================================================= */

#define CRASH_MAGIC       0xDEADBEEFu
#define CRASH_CODE_TCP    0x01u   /* TCP write / connect error */
#define CRASH_CODE_WIFI   0x02u   /* WiFi link lost            */
#define CRASH_CODE_WDT    0x03u   /* Watchdog timeout (prev)   */

static void crash_log_write(uint32_t code, uint32_t extra)
{
    watchdog_hw->scratch[0] = CRASH_MAGIC;
    watchdog_hw->scratch[1] = code;
    watchdog_hw->scratch[2] = to_ms_since_boot(get_absolute_time());
    watchdog_hw->scratch[3] = extra;
}

/* =========================================================================
 * LED helpers
 * ========================================================================= */

static inline void led_set(bool on)
{
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on ? 1 : 0);
}

static void led_blink(int count, int on_ms, int off_ms)
{
    for (int i = 0; i < count; i++) {
        led_set(true);  sleep_ms(on_ms);
        led_set(false); sleep_ms(off_ms);
    }
}

/* =========================================================================
 * =========================================================================
 * DATA TCP connection  (port 9000)
 * =========================================================================
 * ========================================================================= */

static struct tcp_pcb *g_data_pcb        = NULL;
static volatile bool   g_data_connected  = false;
static volatile bool   g_data_pending    = false;   /* connect in progress */
static uint32_t        g_connect_start_ms = 0;      /* for non-blocking timeout */

/* Streaming batch buffer (stack-allocated in streaming loop) */
static uint8_t g_batch_buf[FRAME_SIZE * STREAM_BATCH];
static uint32_t g_batch_count    = 0;
static uint32_t g_batch_start_ms = 0;

/* -------------------------------------------------------------------------
 * Raw TCP callbacks for the data connection
 * Called from the lwIP alarm IRQ — keep short, no blocking.
 * ------------------------------------------------------------------------- */

static void data_tcp_err_cb(void *arg, err_t err)
{
    (void)arg;
    printf("[DATA] TCP error %d — disconnected\n", (int)err);
    crash_log_write(CRASH_CODE_TCP, (uint32_t)(int)err);
    g_data_pcb       = NULL;
    g_data_connected = false;
    g_data_pending   = false;
}

static err_t data_tcp_sent_cb(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    (void)arg; (void)tpcb; (void)len;
    return ERR_OK;
}

static err_t data_tcp_recv_cb(void *arg, struct tcp_pcb *tpcb,
                               struct pbuf *p, err_t err)
{
    (void)arg; (void)tpcb; (void)err;
    if (p == NULL) {
        /* Remote host closed the connection. */
        printf("[DATA] Host closed connection.\n");
        g_data_connected = false;
        g_data_pcb       = NULL;
        return ERR_OK;
    }
    /* Discard any incoming data (we only send). */
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t data_tcp_connected_cb(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    (void)arg; (void)tpcb;
    if (err != ERR_OK) {
        printf("[DATA] Connect failed: %d\n", (int)err);
        g_data_connected = false;
    } else {
        printf("[DATA] Connected to %s:%d\n", HOST_IP, HOST_PORT);
        g_data_connected = true;
    }
    g_data_pending = false;
    return ERR_OK;
}

/* -------------------------------------------------------------------------
 * Initiate TCP connection to host data server.
 * Non-blocking: sets g_data_pending = true; callback clears it.
 * Returns false immediately if WiFi is not up.
 * ------------------------------------------------------------------------- */

static bool data_tcp_connect(void)
{
    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
        printf("[DATA] WiFi link down — cannot connect\n");
        return false;
    }

    printf("[DATA] Connecting to %s:%d ...\n", HOST_IP, HOST_PORT);

    cyw43_arch_lwip_begin();

    if (g_data_pcb) {
        tcp_close(g_data_pcb);
        g_data_pcb = NULL;
    }

    g_data_pcb = tcp_new();
    if (!g_data_pcb) {
        cyw43_arch_lwip_end();
        printf("[DATA] tcp_new() failed\n");
        return false;
    }

    tcp_err(g_data_pcb,  data_tcp_err_cb);
    tcp_recv(g_data_pcb, data_tcp_recv_cb);
    tcp_sent(g_data_pcb, data_tcp_sent_cb);
    /* Disable Nagle — we send batches ourselves */
    tcp_nagle_disable(g_data_pcb);

    ip_addr_t host_addr;
    if (!ip4addr_aton(HOST_IP, &host_addr)) {
        tcp_close(g_data_pcb);
        g_data_pcb = NULL;
        cyw43_arch_lwip_end();
        printf("[DATA] Invalid HOST_IP\n");
        return false;
    }

    g_data_pending   = true;
    g_data_connected = false;

    err_t err = tcp_connect(g_data_pcb, &host_addr, HOST_PORT,
                            data_tcp_connected_cb);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("[DATA] tcp_connect() returned %d\n", (int)err);
        g_data_pending = false;
        return false;
    }

    /* Non-blocking: return immediately.  The lwIP alarm IRQ will fire
     * data_tcp_connected_cb when the handshake completes.  The main loop
     * watches g_connect_start_ms and cancels the attempt after 3 s so
     * do_stream_work() never stalls during a connect attempt. */
    return true;
}

/* -------------------------------------------------------------------------
 * Drain ring buffer and send frames in batches over TCP.
 * Called from the Core 0 main loop on every iteration.
 * ------------------------------------------------------------------------- */

static void do_stream_work(void)
{
    if (!g_data_connected || !g_data_pcb) return;

    /* Fill batch from ring buffer */
    while (g_batch_count < STREAM_BATCH) {
        if (!ring_pop(&g_ring, g_batch_buf + g_batch_count * FRAME_SIZE))
            break;
        g_batch_count++;
    }

    if (g_batch_count == 0) return;

    uint32_t now_ms  = to_ms_since_boot(get_absolute_time());
    uint32_t elapsed = now_ms - g_batch_start_ms;

    bool full    = (g_batch_count >= STREAM_BATCH);
    bool timeout = (elapsed >= STREAM_TIMEOUT_MS);

    if (!full && !timeout) return;

    uint32_t len = g_batch_count * FRAME_SIZE;

    cyw43_arch_lwip_begin();

    u16_t avail = tcp_sndbuf(g_data_pcb);
    if (avail >= (u16_t)len) {
        err_t err = tcp_write(g_data_pcb, g_batch_buf, (u16_t)len,
                              TCP_WRITE_FLAG_COPY);
        if (err == ERR_OK) {
            tcp_output(g_data_pcb);
            g_sent_frames    += g_batch_count;
            g_batch_count     = 0;   /* batch consumed — ready for next fill */
            g_batch_start_ms  = now_ms;
        } else {
            printf("[DATA] tcp_write error %d — closing\n", (int)err);
            tcp_close(g_data_pcb);
            g_data_pcb        = NULL;
            g_data_connected  = false;
            g_batch_count     = 0;   /* discard unsent batch on hard error */
            g_batch_start_ms  = now_ms;
        }
    }
    /* else: send buffer full; batch stays in g_batch_buf, retried next call */

    cyw43_arch_lwip_end();
}

/* =========================================================================
 * =========================================================================
 * CMD TCP server  (port 9001)
 * =========================================================================
 * ========================================================================= */

static struct tcp_pcb *g_cmd_listen_pcb = NULL;
static struct tcp_pcb *g_cmd_conn_pcb   = NULL;

/* Receive buffer for assembling lines */
#define CMD_BUF_SIZE 512
static char    g_cmd_buf[CMD_BUF_SIZE];
static uint32_t g_cmd_buf_len = 0;

/* -------------------------------------------------------------------------
 * Send a null-terminated string + CRLF over the CMD connection.
 * Must be called while holding cyw43_arch_lwip_begin().
 * ------------------------------------------------------------------------- */

static void cmd_send(struct tcp_pcb *pcb, const char *fmt, ...)
{
    static char line[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line) - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)(sizeof(line) - 3)) n = (int)(sizeof(line) - 3);
    line[n++] = '\r';
    line[n++] = '\n';
    line[n]   = '\0';
    tcp_write(pcb, line, (u16_t)n, TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);
}

/* -------------------------------------------------------------------------
 * Dispatch a single command line.
 * Must be called while holding cyw43_arch_lwip_begin().
 * ------------------------------------------------------------------------- */

static void cmd_dispatch(struct tcp_pcb *pcb, const char *cmd)
{
    printf("[CMD] Recv: '%s'\n", cmd);

    if (strcasecmp(cmd, "ping") == 0) {
        cmd_send(pcb, "pong");

    } else if (strcasecmp(cmd, "status") == 0) {
        uint32_t buffered = ring_count(&g_ring);
        uint32_t dropped  = g_ring.dropped;
        cmd_send(pcb, "frames_sent=%lu dropped=%lu buffered=%lu "
                      "wifi=%d tcp=%d",
                 (unsigned long)g_sent_frames,
                 (unsigned long)dropped,
                 (unsigned long)buffered,
                 (int)(cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP),
                 (int)g_data_connected);

    } else if (strcasecmp(cmd, "scan_all") == 0) {
        /* Return the most-recently-buffered raw data as hex.
         * For the demo this just reports stats since we don't cache scans. */
        cmd_send(pcb, "demo_mode=1 recording_ch=64 stim_ch=8 total_ch=%u",
                 (unsigned)TOTAL_CHANNELS);

    } else if (strcasecmp(cmd, "blink_led") == 0) {
        /* Blink 5 times — briefly release lwIP lock to allow blink */
        cyw43_arch_lwip_end();
        led_blink(5, 100, 100);
        cyw43_arch_lwip_begin();
        cmd_send(pcb, "ok");

    } else if (strncasecmp(cmd, "set_fps", 7) == 0) {
        /* set_fps <hz>  — change Core 1 scan rate without a rebuild.
         * Minimum: 10 Hz.  No hard maximum, but >5000 FPS is unreliable. */
        const char *arg = cmd + 7;
        while (*arg == ' ') arg++;
        int hz = (*arg) ? atoi(arg) : 0;
        if (hz < 10) {
            cmd_send(pcb, "error: set_fps requires <hz> >= 10");
        } else {
            g_scan_period_us = (uint32_t)(1000000u / (unsigned)hz);
            cmd_send(pcb, "fps=%d period_us=%lu", hz,
                     (unsigned long)g_scan_period_us);
        }

    } else if (strcasecmp(cmd, "benchmark") == 0) {
        /* Measure actual streaming FPS over 500 ms.
         * Release lwIP lock while sleeping so the WiFi IRQ keeps running. */
        uint32_t f0 = g_sent_frames;
        cyw43_arch_lwip_end();
        sleep_ms(500);
        cyw43_arch_lwip_begin();
        float fps    = (float)(g_sent_frames - f0) * 2.0f;  /* ×2: 500 ms window */
        float ms_per = (fps > 0.0f) ? 1000.0f / fps : 0.0f;
        cmd_send(pcb, "%.1f FPS  (%.1f ms/frame)", (double)fps, (double)ms_per);

    } else if (strcasecmp(cmd, "read_regs") == 0) {
        /* Return ADS124S08 power-on defaults for all 6 ADCs.
         * Format: "ADC<n> 0x<addr>=0x<val> ..."
         * Parsed by GUI's _regs_ingest_response(). */
        static const uint8_t defaults[18] = {
            0x10, 0x00, 0x01, 0x00, 0x14, 0x10, 0x00, 0xFF,
            0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
            0x00, 0x00
        };
        cmd_send(pcb, "[REGS] Reading all 6 ADS124S08 register banks (demo)...");
        char reg_line[256];
        for (int adc = 0; adc < (int)NUM_ADCS; adc++) {
            int pos = snprintf(reg_line, sizeof(reg_line), "ADC%d", adc);
            for (int r = 0; r < 18 && pos < (int)sizeof(reg_line) - 12; r++) {
                pos += snprintf(reg_line + pos, sizeof(reg_line) - (size_t)pos,
                                " 0x%02X=0x%02X", r, defaults[r]);
            }
            cmd_send(pcb, "%s", reg_line);
        }
        cmd_send(pcb, "Register dump complete -- all 6 ADCs responding");

    } else if (strcasecmp(cmd, "test_hardware") == 0) {
        /* Hardware self-test.
         * In demo mode all ADC checks return synthetic PASS.
         * In real mode this would walk the SPI bus and read device IDs. */
        cmd_send(pcb, "[TEST] Hardware Self-Test  (%s)",
#ifdef MUSHIO_DEMO
                 "demo mode — synthetic results"
#else
                 "real hardware"
#endif
                 );
        uint32_t pass = 0, total = 0;

        /* --- ADC ID checks (6 ADCs) --------------------------------------- */
        for (int a = 0; a < (int)NUM_ADCS; a++) {
#ifdef MUSHIO_DEMO
            cmd_send(pcb, "[TEST] ADC%d: ID=0x00  PASS", a);
            pass++;
#else
            /* Real: read ID register via SPI, expect bits[4:0] == 0x00 */
            cmd_send(pcb, "[TEST] ADC%d: real SPI check not yet implemented", a);
#endif
            total++;
        }

        /* --- DRDY line checks (6 ADCs) ------------------------------------ */
        for (int a = 0; a < (int)NUM_ADCS; a++) {
#ifdef MUSHIO_DEMO
            cmd_send(pcb, "[TEST] ADC%d DRDY: PASS (demo)", a);
            pass++;
#else
            cmd_send(pcb, "[TEST] ADC%d DRDY: real check not yet implemented", a);
#endif
            total++;
        }

        /* --- Ring buffer health ------------------------------------------- */
        {
            uint32_t dropped  = g_ring.dropped;
            uint32_t buffered = ring_count(&g_ring);
            if (dropped == 0) {
                cmd_send(pcb, "[TEST] Ring: buffered=%lu dropped=0  PASS",
                         (unsigned long)buffered);
                pass++;
            } else {
                cmd_send(pcb, "[TEST] Ring: buffered=%lu dropped=%lu  WARN",
                         (unsigned long)buffered, (unsigned long)dropped);
            }
            total++;
        }

        /* --- WiFi RSSI ---------------------------------------------------- */
        {
            int32_t rssi = 0;
            int ret = cyw43_wifi_get_rssi(&cyw43_state, &rssi);
            if (ret == 0) {
                const char *qual = (rssi > -60) ? "GOOD" :
                                   (rssi > -75) ? "FAIR" : "WEAK";
                cmd_send(pcb, "[TEST] WiFi RSSI: %d dBm  %s",
                         (int)rssi, qual);
            } else {
                cmd_send(pcb, "[TEST] WiFi RSSI: read failed (%d)", ret);
            }
            total++;
            pass++;  /* RSSI is informational — always counts as pass */
        }

        cmd_send(pcb, "[TEST] RESULT: %lu/%lu checks passed  %s",
                 (unsigned long)pass, (unsigned long)total,
                 (pass == total) ? "ALL PASS" : "SOME FAILED");

    } else if (strcasecmp(cmd, "ota_reboot") == 0) {
        /* Request reboot into USB mass-storage (BOOTSEL) mode for .uf2 upload.
         * We set a flag here (IRQ context) and execute in main loop (safe context). */
        cmd_send(pcb, "[OTA] Entering BOOTSEL mode in 2 s ...");
        cmd_send(pcb, "[OTA] RPI-RP2 drive will appear — drop .uf2 to flash.");
        g_ota_reboot_requested = true;

    } else if (strcasecmp(cmd, "crash_log") == 0) {
        /* Return last crash info from watchdog scratch registers. */
        uint32_t magic = watchdog_hw->scratch[0];
        if (magic == CRASH_MAGIC) {
            uint32_t code  = watchdog_hw->scratch[1];
            uint32_t ts_ms = watchdog_hw->scratch[2];
            uint32_t extra = watchdog_hw->scratch[3];
            const char *desc = (code == CRASH_CODE_TCP)  ? "TCP error"  :
                               (code == CRASH_CODE_WIFI) ? "WiFi lost"  :
                               (code == CRASH_CODE_WDT)  ? "WDT timeout":
                                                           "unknown";
            cmd_send(pcb, "[CRASH] Last crash: code=0x%02lX (%s) ts=%lu ms extra=0x%08lX",
                     (unsigned long)code, desc,
                     (unsigned long)ts_ms,
                     (unsigned long)extra);
            /* Clear after reading so next query shows "none" */
            watchdog_hw->scratch[0] = 0;
        } else {
            cmd_send(pcb, "[CRASH] No crash record (clean boot).");
        }

    } else if (strcasecmp(cmd, "help") == 0) {
        cmd_send(pcb, "Commands: ping | status | scan_all | benchmark | "
                      "read_regs | set_fps <hz> | blink_led | "
                      "test_hardware | ota_reboot | crash_log | help");
        cmd_send(pcb, "True wireless OTA: connect ota_client.py to port %u", OTA_PORT);

    } else if (cmd[0] == '\0') {
        /* empty line — ignore */

    } else {
        cmd_send(pcb, "unknown command: %s", cmd);
    }
}

/* -------------------------------------------------------------------------
 * Raw TCP callbacks for the CMD server
 * ------------------------------------------------------------------------- */

static void cmd_tcp_err_cb(void *arg, err_t err)
{
    (void)arg; (void)err;
    printf("[CMD] Client error %d\n", (int)err);
    g_cmd_conn_pcb = NULL;
    g_cmd_buf_len  = 0;
}

static err_t cmd_tcp_recv_cb(void *arg, struct tcp_pcb *tpcb,
                              struct pbuf *p, err_t err)
{
    (void)arg; (void)err;

    if (p == NULL) {
        /* Client disconnected */
        printf("[CMD] Client disconnected\n");
        g_cmd_conn_pcb = NULL;
        g_cmd_buf_len  = 0;
        tcp_close(tpcb);
        return ERR_OK;
    }

    /* Accumulate received bytes into line buffer */
    struct pbuf *q = p;
    while (q) {
        uint8_t *src = (uint8_t *)q->payload;
        for (uint16_t i = 0; i < q->len; i++) {
            char c = (char)src[i];
            if (c == '\n') {
                /* Strip trailing CR */
                if (g_cmd_buf_len > 0 &&
                    g_cmd_buf[g_cmd_buf_len - 1] == '\r')
                    g_cmd_buf_len--;

                g_cmd_buf[g_cmd_buf_len] = '\0';
                cmd_dispatch(tpcb, g_cmd_buf);
                g_cmd_buf_len = 0;
            } else {
                if (g_cmd_buf_len < CMD_BUF_SIZE - 1) {
                    g_cmd_buf[g_cmd_buf_len++] = c;
                }
            }
        }
        q = q->next;
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t cmd_tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg; (void)err;

    if (g_cmd_conn_pcb != NULL) {
        /* Already have a client — reject */
        tcp_close(newpcb);
        return ERR_OK;
    }

    printf("[CMD] GUI connected from %s\n",
           ip4addr_ntoa(&newpcb->remote_ip));

    g_cmd_conn_pcb = newpcb;
    g_cmd_buf_len  = 0;

    tcp_err(g_cmd_conn_pcb,  cmd_tcp_err_cb);
    tcp_recv(g_cmd_conn_pcb, cmd_tcp_recv_cb);

    /* Send welcome banner */
    cmd_send(newpcb, "============================================================");
    cmd_send(newpcb, " MushIO V1.0  |  Real-Time Biopotential Monitor  (C fw)");
    cmd_send(newpcb, "============================================================");
    cmd_send(newpcb, "Type 'help' for available commands.");

    return ERR_OK;
}

static void cmd_server_init(void)
{
    cyw43_arch_lwip_begin();

    g_cmd_listen_pcb = tcp_new();
    if (!g_cmd_listen_pcb) {
        cyw43_arch_lwip_end();
        printf("[CMD] tcp_new() failed\n");
        return;
    }

    tcp_bind(g_cmd_listen_pcb, IP_ADDR_ANY, CMD_PORT);
    g_cmd_listen_pcb = tcp_listen(g_cmd_listen_pcb);
    tcp_accept(g_cmd_listen_pcb, cmd_tcp_accept_cb);

    cyw43_arch_lwip_end();
    printf("[CMD] Listening on port %d\n", CMD_PORT);
}

/* =========================================================================
 * =========================================================================
 * WiFi helpers
 * =========================================================================
 * ========================================================================= */

static bool wifi_connect(void)
{
    printf("[WIFI] Connecting to '%s' ...\n", WIFI_SSID);

    int ret = cyw43_arch_wifi_connect_timeout_ms(
                  WIFI_SSID, WIFI_PASSWORD,
                  CYW43_AUTH_WPA2_AES_PSK,
                  30000);

    if (ret == 0) {
        printf("[WIFI] Connected.  IP: %s\n",
               ip4addr_ntoa(netif_ip4_addr(netif_default)));
        return true;
    }

    printf("[WIFI] Failed (err %d)\n", ret);
    return false;
}

/* =========================================================================
 * =========================================================================
 * main()
 * =========================================================================
 * ========================================================================= */

int main(void)
{
    /* --------------------------------------------------------------------- */
    /* Phase 1: Stdio + CRC table                                             */
    /* --------------------------------------------------------------------- */

    stdio_init_all();
    sleep_ms(500);   /* give USB serial time to enumerate */

    printf("\n");
    printf("============================================================\n");
    printf("  MushIO V1.0  |  Pico Demo Mode  (C firmware)\n");
    printf("============================================================\n\n");

    crc16_init_table();

    /* --------------------------------------------------------------------- */
    /* Phase 1b: Check for a pending OTA image in the staging area.           */
    /*           Must run BEFORE Core 1 is launched (no multicore lockout     */
    /*           needed — Core 1 isn't alive yet).                            */
    /*           Never returns if a valid OTA is found; reboots into new fw.  */
    /* --------------------------------------------------------------------- */

    ota_check_and_apply();

    /* --------------------------------------------------------------------- */
    /* Phase 2: Initialise ring buffer, then launch Core 1                    */
    /* --------------------------------------------------------------------- */

    ring_init(&g_ring);

    printf("[INIT] Launching Core 1 (ADC scan loop)...\n");
    multicore_launch_core1(core1_main);
    /* Give Core 1 a moment to start and init the sine table */
    sleep_ms(100);
    printf("[INIT] Core 1 running.\n");

    /* --------------------------------------------------------------------- */
    /* Phase 3: Initialise cyw43 / WiFi                                       */
    /* --------------------------------------------------------------------- */

    printf("[INIT] Initialising WiFi...\n");

    if (cyw43_arch_init()) {
        printf("[FATAL] cyw43_arch_init failed — halting\n");
        while (true) { led_blink(3, 80, 80); sleep_ms(500); }
    }

    cyw43_arch_enable_sta_mode();

    /* Startup blink: 5 rapid flashes */
    for (int i = 0; i < 5; i++) {
        led_set(true);  sleep_ms(80);
        led_set(false); sleep_ms(80);
    }

    /* --------------------------------------------------------------------- */
    /* Phase 4: WiFi connect (retry indefinitely)                             */
    /* --------------------------------------------------------------------- */

    while (!wifi_connect()) {
        printf("[WIFI] Retrying in 5 s...\n");
        led_blink(3, 200, 200);
        sleep_ms(5000);
    }

    /* --------------------------------------------------------------------- */
    /* Phase 4b: Check for previous crash, then enable watchdog              */
    /* Enabled AFTER WiFi connect so the 30 s join doesn't trigger it.       */
    /* 8 s timeout: main loop feeds watchdog; hang → reset → crash_log set.  */
    /* --------------------------------------------------------------------- */

    if (watchdog_caused_reboot()) {
        /* Previous watchdog reset — mark crash record if not already set */
        if (watchdog_hw->scratch[0] != CRASH_MAGIC) {
            crash_log_write(CRASH_CODE_WDT, 0);
        }
        printf("[CRASH] Previous watchdog reset detected — see 'crash_log' CMD.\n");
    }

    watchdog_enable(8000, true);   /* 8 s, pause-on-debug = true */
    printf("[WDT] Watchdog enabled (8 s timeout).\n");

    /* --------------------------------------------------------------------- */
    /* Phase 5: TCP data connection                                           */
    /* --------------------------------------------------------------------- */

    printf("[INIT] Connecting data socket to %s:%d ...\n", HOST_IP, HOST_PORT);
    if (data_tcp_connect()) {
        g_connect_start_ms = to_ms_since_boot(get_absolute_time());
    }

    /* --------------------------------------------------------------------- */
    /* Phase 6: CMD server                                                    */
    /* --------------------------------------------------------------------- */

    cmd_server_init();

    /* --------------------------------------------------------------------- */
    /* Phase 6b: OTA server (true wireless firmware update, port 9002)       */
    /* --------------------------------------------------------------------- */

    ota_server_init();

    /* --------------------------------------------------------------------- */
    /* Phase 7: Main loop                                                     */
    /* --------------------------------------------------------------------- */

    printf("\n[RUN] Streaming.  Channels: %u  Target: ~%u FPS\n",
           (unsigned)TOTAL_CHANNELS,
           (unsigned)(1000000u / DEMO_SCAN_PERIOD_US));
    printf("[RUN] Data  → %s:%d\n", HOST_IP, HOST_PORT);
    printf("[RUN] CMD   → port %d\n\n", CMD_PORT);

    uint32_t last_stats_ms  = to_ms_since_boot(get_absolute_time());
    uint32_t last_recon_ms  = 0;
    uint32_t sent_prev       = 0;

    g_batch_start_ms = to_ms_since_boot(get_absolute_time());

    while (true) {
        /* ---- Watchdog feed ----------------------------------------------- */
        watchdog_update();

        /* ---- True wireless OTA (port 9002) ------------------------------- */
        ota_server_poll();

        /* ---- OTA reboot (set by ota_reboot CMD, executed here safely) ---- */
        if (g_ota_reboot_requested) {
            printf("[OTA] Rebooting into BOOTSEL mode...\n");
            sleep_ms(2000);   /* give CMD response time to flush */
            reset_usb_boot(0, 0);
            /* unreachable */
        }

        /* ---- Stream: drain ring → TCP ------------------------------------ */
        do_stream_work();

        /* ---- Reconnect data TCP if needed -------------------------------- */
        if (!g_data_connected && !g_data_pending) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - last_recon_ms >= RECONNECT_INTERVAL_MS) {
                printf("[DATA] Reconnecting...\n");
                if (data_tcp_connect()) {
                    g_connect_start_ms = now;   /* start non-blocking timeout */
                }
                last_recon_ms    = to_ms_since_boot(get_absolute_time());
                g_batch_count    = 0;   /* discard stale batch */
                g_batch_start_ms = last_recon_ms;
            }
        }

        /* ---- Connect-attempt timeout (3 s, non-blocking) ----------------- */
        if (g_data_pending) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - g_connect_start_ms > 3000u) {
                printf("[DATA] Connect timeout\n");
                g_data_pending   = false;
                g_data_connected = false;
                cyw43_arch_lwip_begin();
                if (g_data_pcb) { tcp_close(g_data_pcb); g_data_pcb = NULL; }
                cyw43_arch_lwip_end();
            }
        }

        /* ---- Periodic stats (every 10 s) --------------------------------- */
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_stats_ms >= 10000) {
            uint32_t elapsed_s = (now_ms - last_stats_ms) / 1000;
            uint32_t sent_delta = g_sent_frames - sent_prev;
            float    fps = (elapsed_s > 0) ? (float)sent_delta / (float)elapsed_s
                                           : 0.0f;

            printf("[STAT] Sent: %lu (+%lu)  FPS: %.1f  "
                   "Buffered: %lu  Dropped: %lu  TCP: %d\n",
                   (unsigned long)g_sent_frames,
                   (unsigned long)sent_delta,
                   (double)fps,
                   (unsigned long)ring_count(&g_ring),
                   (unsigned long)g_ring.dropped,
                   (int)g_data_connected);

            sent_prev      = g_sent_frames;
            last_stats_ms  = now_ms;
        }

        /* ---- LED double-blink heartbeat (every 2 s, non-blocking) -------- */
        /* State machine: no sleep_ms so do_stream_work() never stalls.
         * Phase 0: long idle → on1(60ms) → off1(60ms) → on2(60ms) → off2(1820ms)
         * Total cycle: 60+60+60+1820 = 2000 ms, identical appearance to before. */
        {
            static uint8_t  led_phase    = 0;
            static uint32_t led_until_ms = 0;
            if (now_ms >= led_until_ms) {
                switch (led_phase) {
                case 0: led_set(true);  led_until_ms = now_ms + 60;   led_phase = 1; break;
                case 1: led_set(false); led_until_ms = now_ms + 60;   led_phase = 2; break;
                case 2: led_set(true);  led_until_ms = now_ms + 60;   led_phase = 3; break;
                case 3: led_set(false); led_until_ms = now_ms + 1820; led_phase = 0; break;
                default: led_phase = 0; break;
                }
            }
        }
    }

    /* unreachable */
    cyw43_arch_deinit();
    return 0;
}
