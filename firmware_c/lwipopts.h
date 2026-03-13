/*
 * lwipopts.h — lwIP configuration for MushIO C firmware
 *
 * Used with pico_cyw43_arch_lwip_threadsafe_background.
 * The WiFi driver runs via a hardware alarm IRQ on Core 0.
 * All lwIP API calls from application code must be guarded with
 * cyw43_arch_lwip_begin() / cyw43_arch_lwip_end().
 *
 * Key sizing decisions:
 *   TCP_SND_BUF  32 KB  — holds ~142 frames (228 B each) before blocking
 *   TCP_WND      16 KB  — receive window (mostly unused; we only send)
 *   MEM_SIZE     40 KB  — lwIP heap for pbufs, PCBs, etc.
 *
 * Throughput tuning (2026-03-11):
 *   TCP_MAXRTX        4     — fast failure (~15 s) instead of 20+ min stalls
 *   TCP_OVERSIZE       MSS  — contiguous pbuf per segment, fewer allocs
 *   CHECKSUM_ON_COPY   1    — checksum computed during memcpy, saves a pass
 *   ETH_PAD_SIZE       0    — CYW43 driver doesn't support padding
 */

#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

/* --- OS / threading --------------------------------------------------------
 * threadsafe_background uses alarm IRQs, not a real OS.
 * Keep NO_SYS=1 (lwIP raw API only; no netconn/socket layers).
 * -------------------------------------------------------------------------- */
#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        1   /* critical-section protection via cyw43 spinlock */

/* --- Memory ---------------------------------------------------------------- */
#define MEM_ALIGNMENT               4
/* ETH_PAD_SIZE must be 0: the CYW43 driver (cyw43_lwip.c) does NOT strip
 * pad bytes on TX or add them on RX.  Non-zero values corrupt every
 * Ethernet frame, breaking DHCP, ARP, ping, and all IP traffic. */
#define ETH_PAD_SIZE                0
#define MEM_SIZE                    (40 * 1024)   /* 40 KB lwIP heap */
#define MEMP_NUM_TCP_SEG            180  /* must be >= TCP_SND_QUEUELEN (~175) */
#define MEMP_NUM_PBUF               64
#define PBUF_POOL_SIZE              32
/* Default MEMP_NUM_TCP_PCB is 5 — too few for data+cmd+ota+TIME_WAIT leftovers.
 * Bug fix: data_tcp_recv_cb now calls tcp_close() on EOF, but keep the pool
 * large enough to absorb a few TIME_WAIT connections after reconnects. */
#define MEMP_NUM_TCP_PCB            12
#define MEMP_NUM_TCP_PCB_LISTEN     4

/* --- TCP ------------------------------------------------------------------- */
#define LWIP_TCP                    1
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (32 * 1024)   /* 32 KB send buffer */
#define TCP_SND_QUEUELEN            (8 * (TCP_SND_BUF) / (TCP_MSS))
#define TCP_WND                     (16 * 1024)
#define TCP_OVERSIZE                TCP_MSS  /* pre-alloc contiguous pbuf per segment */
#define LWIP_CHECKSUM_ON_COPY       1        /* checksum computed during tcp_write memcpy */
#define TCP_MAXRTX                  4   /* fast failure: ~15 s vs 20+ min at default 12 */
#define TCP_SYNMAXRTX               4
#define LWIP_TCP_KEEPALIVE          1
#define TCP_KEEPIDLE_DEFAULT        10000u  /* 10 s */
#define TCP_KEEPINTVL_DEFAULT       5000u   /* 5 s */
#define TCP_KEEPCNT_DEFAULT         3u

/* --- ARP / IP -------------------------------------------------------------- */
#define LWIP_ARP                    1
#define ARP_TABLE_SIZE              4
#define ARP_QUEUEING                0
#define LWIP_DHCP                   1
#define LWIP_DNS                    0
#define LWIP_IGMP                   0
#define LWIP_ICMP                   1

/* --- UDP (needed by DHCP) -------------------------------------------------- */
#define LWIP_UDP                    1
#define MEMP_NUM_UDP_PCB            4

/* --- Socket / netconn (disabled — use raw API) ----------------------------- */
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1

/* --- Checksum -------------------------------------------------------------- */
#define LWIP_CHECKSUM_CTRL_PER_NETIF 0

/* --- Statistics / debug ---------------------------------------------------- */
#define LWIP_STATS                  0
#define LWIP_DEBUG                  0

/* --- Timers ---------------------------------------------------------------- */
#define LWIP_TIMERS                 1

/* --- Locking (provided by cyw43 arch) -------------------------------------- */
#ifndef LWIP_SINGLE_NETIF
#define LWIP_SINGLE_NETIF           1
#endif

#endif /* _LWIPOPTS_H */
