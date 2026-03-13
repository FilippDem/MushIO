/*
 * ring.h — Lockless SPSC Ring Buffer
 *
 * Single-Producer (Core 1) / Single-Consumer (Core 0).
 *
 * Rules:
 *   - Only Core 1 advances head.
 *   - Only Core 0 advances tail.
 *   - One slot is kept empty as the "buffer full" sentinel so that
 *     head == tail unambiguously means "empty".
 *   - __DMB() barriers ensure frame bytes are visible across cores
 *     before the head/tail pointer is updated.
 *
 * Capacity: RING_SIZE - 1 frames  (127 @ RING_SIZE=128).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/sync.h"   /* __dmb() */
#include "config.h"

typedef struct {
    uint8_t          buf[FRAME_SIZE * RING_SIZE];
    volatile uint32_t head;   /* written by producer (Core 1) only */
    volatile uint32_t tail;   /* written by consumer (Core 0) only */
    volatile uint32_t dropped; /* incremented by producer on overflow */
} ring_t;

/*
 * ring_push — called by Core 1 (producer).
 * Returns true if frame was enqueued, false if ring is full (frame dropped).
 */
static inline bool ring_push(ring_t *r, const uint8_t *frame)
{
    uint32_t head     = r->head;
    uint32_t next_head = (head + 1u) % RING_SIZE;

    if (next_head == r->tail) {
        /* Ring full — drop incoming frame.  Never touch tail (owned by Core 0). */
        r->dropped++;
        return false;
    }

    memcpy(r->buf + head * FRAME_SIZE, frame, FRAME_SIZE);

    /* DMB: ensure all frame bytes are visible before head is updated. */
    __dmb();
    r->head = next_head;
    return true;
}

/*
 * ring_pop — called by Core 0 (consumer).
 * Copies oldest frame into dest[].
 * Returns true if a frame was available, false if ring is empty.
 */
static inline bool ring_pop(ring_t *r, uint8_t *dest)
{
    uint32_t tail = r->tail;

    if (tail == r->head) {
        return false;   /* empty */
    }

    /* DMB: ensure head is read before we read the frame bytes. */
    __dmb();
    memcpy(dest, r->buf + tail * FRAME_SIZE, FRAME_SIZE);

    r->tail = (tail + 1u) % RING_SIZE;
    return true;
}

/*
 * ring_count — approximate number of frames buffered.
 * May be called from either core (no lock; result is approximate).
 */
static inline uint32_t ring_count(const ring_t *r)
{
    return (r->head - r->tail + RING_SIZE) % RING_SIZE;
}

/*
 * ring_peek — read a frame by offset from tail without consuming it.
 * offset=0 is the oldest frame (tail), offset=1 is next oldest, etc.
 * Returns true if the frame exists, false if offset >= count.
 * Called by Core 0 only (for retransmission requests).
 */
static inline bool ring_peek(const ring_t *r, uint32_t offset, uint8_t *dest)
{
    uint32_t count = (r->head - r->tail + RING_SIZE) % RING_SIZE;
    if (offset >= count) return false;

    uint32_t idx = (r->tail + offset) % RING_SIZE;
    __dmb();
    memcpy(dest, r->buf + idx * FRAME_SIZE, FRAME_SIZE);
    return true;
}

/*
 * ring_init — zero-initialise the ring buffer.
 * Call once before launching Core 1.
 */
static inline void ring_init(ring_t *r)
{
    r->head    = 0;
    r->tail    = 0;
    r->dropped = 0;
}
