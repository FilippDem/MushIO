"""
WiFi TCP Data Streamer for MushIO V1.0 — single-core edition

Architecture
------------
Core 0 runs everything in one event loop:
  1. scan_frame()        — generate ADC data (3 ms in demo mode)
  2. build_frame()       — pack + CRC16 (@micropython.native)
  3. push_frame()        — append to ring buffer
  4. _do_stream_work()   — batch-drain ring buffer over TCP
  5. cmd_server.poll_once() — non-blocking CMD check

No _thread is used.  This avoids the MicroPython GIL contention that
throttled the dual-core design to ~16 FPS.  The ring buffer is still
useful for absorbing occasional sock.write() stalls.
"""

import micropython
import time
import struct
import network
import socket
import config


@micropython.native
def crc16_ccitt(data):
    """
    CRC-16/CCITT-FALSE over a bytes-like object.
    Polynomial: 0x1021, Init: 0xFFFF
    @micropython.native gives ~5-10x speedup on the inner loop.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


class DataStreamer:
    """WiFi TCP client that streams binary ADC frames to a host server."""

    # Frame layout
    HEADER_SIZE = 10                             # sync(2)+ts(4)+seq(2)+nadc(1)+nch(1)
    DATA_SIZE   = config.TOTAL_CHANNELS * 3      # 72 ch × 3 bytes = 216
    CRC_SIZE    = 2
    FRAME_SIZE  = HEADER_SIZE + DATA_SIZE + CRC_SIZE   # 228 bytes

    # Core 1 stream parameters
    #
    # STREAM_BATCH: frames accumulated before one sock.write() call.
    #   40 × 228 = 9 120 bytes ≈ 7 TCP segments.  Even if each segment
    #   requires a WiFi radio wake, 7 wakes per write (not 7 per frame)
    #   yields ~40× improvement over sending one frame at a time.
    STREAM_BATCH      = 40

    # Flush a partial batch after this many ms so latency stays bounded
    # even at low frame rates or during startup.
    STREAM_TIMEOUT_MS = 20

    # Ring buffer capacity (frames).  128 × 228 B = 29 KB — fits easily in
    # the RP2350's 520 KB RAM and covers ~0.4 s of 333-FPS generation.
    RING_SIZE = config.STREAM_BUFFER_SIZE

    # =========================================================================
    # Construction
    # =========================================================================

    def __init__(self):
        self.wlan = None
        self.sock = None
        self._connected_wifi = False
        self._connected_tcp  = False
        self._dropped_frames = 0
        self._sent_frames    = 0

        # Ring buffer — single-core, no lock needed.
        # Head advances on push, tail advances on pop.
        # One slot kept empty as sentinel (full vs empty).
        self._ring_buf   = bytearray(self.FRAME_SIZE * self.RING_SIZE)
        self._ring_head  = 0
        self._ring_tail  = 0

        # Pre-allocated build buffer (Core 0 only — no lock needed)
        self._frame = bytearray(self.FRAME_SIZE)

        # Per-call stream state for _do_stream_work() batch sends
        self._stream_buf         = bytearray(self.FRAME_SIZE * self.STREAM_BATCH)
        self._stream_count       = 0
        self._stream_flush_t     = 0
        self._stream_reconnect_t = 0

    # =========================================================================
    # WiFi Connection  (called from Core 0 before stream thread starts)
    # =========================================================================

    def connect_wifi(self, max_retries=10):
        """Connect to WiFi AP with exponential backoff."""
        print(f"[WIFI] Connecting to '{config.WIFI_SSID}'...")

        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)

        # Disable WiFi power save BEFORE connecting.
        # The CYW43439 driver must receive this while the interface is up but
        # before 802.11 association so it never negotiates power-save with the AP.
        try:
            self.wlan.config(pm=0xa11140)   # 0xa11140 == network.WLAN.PM_NONE
            print("[WIFI] Power save disabled (PM_NONE).")
        except Exception:
            pass

        # Set country code if supported
        try:
            self.wlan.config(country=config.WIFI_COUNTRY)
        except Exception:
            pass

        if self.wlan.isconnected():
            self._connected_wifi = True
            ip = self.wlan.ifconfig()[0]
            print(f"[WIFI] Already connected. IP: {ip}")
            return True

        self.wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)

        delay = 1
        for attempt in range(max_retries):
            print(f"[WIFI] Attempt {attempt + 1}/{max_retries}...")
            t0 = time.ticks_ms()
            while not self.wlan.isconnected():
                if time.ticks_diff(time.ticks_ms(), t0) > delay * 1000:
                    break
                time.sleep_ms(100)

            if self.wlan.isconnected():
                self._connected_wifi = True
                ip = self.wlan.ifconfig()[0]
                print(f"[WIFI] Connected! IP: {ip}")
                return True

            delay = min(delay * 2, 30)

        print("[WIFI] Failed to connect after all retries.")
        self._connected_wifi = False
        return False

    def is_wifi_connected(self):
        if self.wlan is None:
            return False
        return self.wlan.isconnected()

    # =========================================================================
    # TCP Connection  (safe to call from Core 1)
    # =========================================================================

    def connect_server(self):
        """Open TCP connection to the host data server."""
        if not self.is_wifi_connected():
            print("[TCP] No WiFi — cannot connect.")
            return False

        print(f"[TCP] Connecting to {config.HOST_IP}:{config.HOST_PORT}...")
        try:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2)   # short: fewer SYN retransmits = less WiFi load on Core 0
            self.sock.connect((config.HOST_IP, config.HOST_PORT))
            self.sock.settimeout(5)   # longer timeout for actual data sends
            try:
                # Disable Nagle: send immediately without waiting for ACK
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except Exception:
                pass
            self._connected_tcp = True
            print("[TCP] Connected to host server.")
            return True
        except Exception as e:
            print(f"[TCP] Connection failed: {e}")
            self._connected_tcp = False
            return False

    def reconnect(self):
        """Close socket and re-establish TCP (and WiFi if needed)."""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self._connected_tcp = False

        if not self.is_wifi_connected():
            self._connected_wifi = False
            if not self.connect_wifi(max_retries=3):
                return False

        return self.connect_server()

    # =========================================================================
    # Frame Construction  (Core 0 only)
    # =========================================================================

    def build_frame(self, timestamp_ms, seq_num, adc_data):
        """Build a 228-byte binary frame and return it (same buffer each call)."""
        frame = self._frame
        struct.pack_into('<H', frame, 0, config.SYNC_WORD)
        struct.pack_into('<I', frame, 2, timestamp_ms & 0xFFFFFFFF)
        struct.pack_into('<H', frame, 6, seq_num & 0xFFFF)
        frame[8] = config.NUM_ADCS
        frame[9] = config.CHANNELS_PER_ADC
        frame[self.HEADER_SIZE:self.HEADER_SIZE + self.DATA_SIZE] = adc_data
        crc = crc16_ccitt(memoryview(frame)[0:self.HEADER_SIZE + self.DATA_SIZE])
        struct.pack_into('<H', frame, self.HEADER_SIZE + self.DATA_SIZE, crc)
        return frame

    # =========================================================================
    # Ring Buffer
    # =========================================================================

    def push_frame(self, frame_bytes):
        """
        Copy one frame into the ring buffer.  Non-blocking.
        Drops the incoming frame if the buffer is full.
        """
        head      = self._ring_head
        next_head = (head + 1) % self.RING_SIZE
        if next_head == self._ring_tail:
            # Buffer full — drop incoming frame; do NOT touch _ring_tail
            # (that pointer belongs to Core 1).
            self._dropped_frames += 1
            return
        off = head * self.FRAME_SIZE
        self._ring_buf[off:off + self.FRAME_SIZE] = frame_bytes
        self._ring_head = next_head

    def _ring_pop_into(self, dest, dest_off):
        """
        Copy the oldest frame from the ring directly into dest[dest_off:].
        Returns True if a frame was available, False if ring is empty.
        """
        tail = self._ring_tail
        if tail == self._ring_head:
            return False   # empty
        src_off = tail * self.FRAME_SIZE
        dest[dest_off:dest_off + self.FRAME_SIZE] = \
            self._ring_buf[src_off:src_off + self.FRAME_SIZE]
        self._ring_tail = (tail + 1) % self.RING_SIZE
        return True

    # =========================================================================
    # TCP Send
    # =========================================================================

    def _tcp_send(self, data):
        if not self._connected_tcp or self.sock is None:
            return False
        try:
            self.sock.write(data)
            return True
        except Exception:
            self._connected_tcp = False
            return False

    # =========================================================================
    # Stream Work  (called from main loop every iteration)
    # =========================================================================

    def _do_stream_work(self):
        """
        Drain the ring buffer and send over TCP.  Called from the main
        loop on every iteration.

        On each call:
          1. If TCP is down, attempt reconnect every RECONNECT_INTERVAL_S.
          2. Pull frames from ring into _stream_buf until batch is full.
          3. Flush when batch is full OR STREAM_TIMEOUT_MS has elapsed.
        """
        # --- Reconnect if TCP is down ---
        if not self._connected_tcp:
            now = time.ticks_ms()
            if time.ticks_diff(now, self._stream_reconnect_t) \
                    >= config.RECONNECT_INTERVAL_S * 1000:
                print("[STREAM] Attempting reconnect...")
                self.reconnect()
                self._stream_reconnect_t = time.ticks_ms()
            return

        # --- Fill batch from ring buffer ---
        while self._stream_count < self.STREAM_BATCH:
            if not self._ring_pop_into(self._stream_buf,
                                       self._stream_count * self.FRAME_SIZE):
                break
            self._stream_count += 1

        # --- Flush: full batch OR timeout ---
        now     = time.ticks_ms()
        elapsed = time.ticks_diff(now, self._stream_flush_t)
        if self._stream_count > 0 and (
                self._stream_count >= self.STREAM_BATCH or
                elapsed >= self.STREAM_TIMEOUT_MS):
            n = self._stream_count
            data = self._stream_buf if n == self.STREAM_BATCH \
                   else bytearray(self._stream_buf[:n * self.FRAME_SIZE])
            if self._tcp_send(data):
                self._sent_frames += n
            self._stream_count  = 0
            self._stream_flush_t = now

    # =========================================================================
    # Backward-compatible alias
    # =========================================================================

    def send_frame(self, frame_bytes):
        """Alias for push_frame() kept for compatibility."""
        self.push_frame(frame_bytes)

    # =========================================================================
    # Status
    # =========================================================================

    @property
    def stats(self):
        return {
            "sent":    self._sent_frames,
            "dropped": self._dropped_frames,
            "buffered": (self._ring_head - self._ring_tail) % self.RING_SIZE,
            "wifi":    self._connected_wifi,
            "tcp":     self._connected_tcp,
        }

    def print_stats(self):
        s = self.stats
        print(f"[STREAM] Sent: {s['sent']}  Dropped: {s['dropped']}  "
              f"Buffered: {s['buffered']}  WiFi: {s['wifi']}  TCP: {s['tcp']}")

    # =========================================================================
    # Cleanup
    # =========================================================================

    def deinit(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        if self.wlan:
            try:
                self.wlan.disconnect()
                self.wlan.active(False)
            except Exception:
                pass
        print("[STREAM] Deinitialized.")
