"""
MushIO V1.0 - Demo ADC Manager (Pico Demo mode — bare Pico 2 W)

Drop-in replacement for ADCManager that generates synthetic waveform data
without any SPI bus, ADS124S08 chips, external clock, or AFE hardware.

Exercises the complete firmware stack:
  - WiFi connection  (real)
  - TCP data streaming at ~200 FPS  (real)
  - 228-byte frame format + CRC-16  (real)
  - CMD server on port 9001 (most commands work; ADC-register ones skip gracefully)
  - Reconnection logic + ring buffer  (real)
  - Watchdog timer  (real)
  - Built-in LED blink via cmd_server.set_led()  (real)

What you will see in the GUI:
  - All 72 channels with distinct sine-wave frequencies
  - ~200 FPS counter
  - Ping / Status / Scan All / Blink LED commands responding
  - Waterfall, Overlay, Heatmap all populated with real streamed data

Usage:
  Copy these files to the Pico root:
      demo_adc_manager.py  (this file)
      main_test.py
      streamer.py
      cmd_server.py
      config.py  (fill in WIFI_SSID, WIFI_PASSWORD, HOST_IP)
  Then run:
      mpremote run main_test.py
  Or rename main_test.py -> main.py and copy for auto-start on boot.
"""

import math
import time
import config

# ---------------------------------------------------------------------------
# ADC timing constants  (derived from ADS124S08 datasheet)
# ---------------------------------------------------------------------------
# At ADC_DATARATE = 0x0D the chip runs at 4000 SPS → 250 µs per conversion.
# The real adc_manager.py scans all 6 ADCs on the same channel step, then
# moves to the next channel — 12 steps total — waiting for DRDY each time:
#
#   12 channels × (1 / 4000 SPS) = 12 × 250 µs = 3 000 µs = 3.0 ms / frame
#   Theoretical max frame rate ≈ 333 FPS (before SPI + Python overhead)
#
# Using time.sleep_us() instead of sleep_ms() for higher timing accuracy.
_ADC_SPS        = 4000                              # matches config.ADC_DATARATE 0x0D
_DRDY_PERIOD_US = 1_000_000 // _ADC_SPS            # 250 µs per conversion
_SCAN_SLEEP_US  = config.CHANNELS_PER_ADC * _DRDY_PERIOD_US  # 3 000 µs per frame

# ---------------------------------------------------------------------------
# Synthetic channel parameters
# Each of the 72 channels gets a unique frequency + amplitude combination
# so the GUI displays clearly distinct waveforms.
# ---------------------------------------------------------------------------

# 8 base frequencies (Hz) — repeat across channels
_FREQS_HZ = [0.3, 0.7, 1.2, 2.0, 3.5, 6.0, 10.0, 18.0]

# 8 base amplitudes (ADC counts, max = 2^23 - 1 = 8,388,607)
_AMPS = [180_000, 320_000, 500_000, 750_000,
         120_000, 260_000, 420_000, 600_000]

# Pre-compute 2*pi*f for each channel
_W = [2.0 * math.pi * _FREQS_HZ[ch % len(_FREQS_HZ)]
      for ch in range(config.TOTAL_CHANNELS)]
_A = [_AMPS[ch % len(_AMPS)]
      for ch in range(config.TOTAL_CHANNELS)]


class DemoADCManager:
    """
    Synthetic replacement for ADCManager (Pico Demo mode).
    Implements the same public interface; no GPIO/SPI pins touched.
    """

    def __init__(self):
        self._frame_buf = bytearray(config.TOTAL_CHANNELS * 3)
        self._t0        = time.ticks_ms()
        # Expose an empty adcs list so cmd_server commands that iterate
        # self._adc_mgr.adcs fail gracefully rather than AttributeError.
        self.adcs = []

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    def init(self):
        print("[DEMO] DemoADCManager initialised — no SPI hardware required.")
        print(f"[DEMO] Generating {config.TOTAL_CHANNELS}-channel synthetic data "
              f"at ~200 FPS.")
        print(f"[DEMO] Frequencies range: "
              f"{min(_FREQS_HZ):.1f} – {max(_FREQS_HZ):.1f} Hz across channels.")

    # ------------------------------------------------------------------
    # Frame generation  (called from the main acquisition loop)
    # ------------------------------------------------------------------

    def scan_frame(self):
        """
        Generate one 216-byte ADC data block (72 channels x 3 bytes, big-endian
        signed 24-bit), then sleep to target ~200 FPS.

        Channel i carries a sine wave:
            amplitude  = _A[i] counts
            frequency  = _FREQS_HZ[i % 8] Hz
        """
        t   = time.ticks_diff(time.ticks_ms(), self._t0) / 1000.0
        buf = self._frame_buf

        for ch in range(config.TOTAL_CHANNELS):
            val = int(math.sin(_W[ch] * t) * _A[ch])
            # Pack as signed 24-bit big-endian
            u = val & 0xFFFFFF
            buf[ch * 3]     = (u >> 16) & 0xFF
            buf[ch * 3 + 1] = (u >> 8)  & 0xFF
            buf[ch * 3 + 2] =  u        & 0xFF

        # Simulate DRDY-limited scan: 12 channels × 250 µs at 4000 SPS = 3 000 µs.
        # No artificial extra throttle — this matches the real hardware rate.
        time.sleep_us(_SCAN_SLEEP_US)
        return buf

    # ------------------------------------------------------------------
    # Single-channel read  (used by cmd_server scan_all)
    # ------------------------------------------------------------------

    def read_single_channel(self, adc_index, ain_channel):
        """Return the current synthetic value for one channel."""
        ch  = adc_index * config.CHANNELS_PER_ADC + ain_channel
        t   = time.ticks_diff(time.ticks_ms(), self._t0) / 1000.0
        return int(math.sin(_W[ch] * t) * _A[ch])

    # ------------------------------------------------------------------
    # No-op methods (no real hardware to talk to)
    # ------------------------------------------------------------------

    def read_temperatures(self):
        """No real temp sensor — return None for each ADC slot."""
        return [None] * config.NUM_ADCS

    def deinit(self):
        print("[DEMO] DemoADCManager: deinit (nothing to release).")


# Allow   from demo_adc_manager import ADCManager
ADCManager = DemoADCManager
