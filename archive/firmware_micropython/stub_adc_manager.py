"""
MushIO V1.0 - Stub ADC Manager (bare Pico 2 W hardware test)

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

What you will see in the GUI (live mode, not demo):
  - All 72 channels with distinct sine-wave frequencies
  - ~200 FPS counter
  - Ping / Status / Scan All / Blink LED commands responding
  - Waterfall, Overlay, Heatmap all populated with real streamed data

Usage:
  Copy these files to the Pico root:
      stub_adc_manager.py  (this file)
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


class StubADCManager:
    """
    Synthetic replacement for ADCManager.
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
        print("[STUB] StubADCManager initialised — no SPI hardware required.")
        print(f"[STUB] Generating {config.TOTAL_CHANNELS}-channel synthetic data "
              f"at ~200 FPS.")
        print("[STUB] Frequencies range: "
              f"{min(_FREQS_HZ):.1f} – {max(_FREQS_HZ):.1f} Hz across channels.")

    # ------------------------------------------------------------------
    # Frame generation  (called from the main acquisition loop)
    # ------------------------------------------------------------------

    def scan_frame(self):
        """
        Generate one 216-byte ADC data block (72 channels × 3 bytes, big-endian
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
            # Mask to 24 bits then split into three bytes
            u = val & 0xFFFFFF
            buf[ch * 3]     = (u >> 16) & 0xFF
            buf[ch * 3 + 1] = (u >> 8)  & 0xFF
            buf[ch * 3 + 2] =  u        & 0xFF

        # Throttle to ≈200 FPS  (5 ms/frame)
        time.sleep_ms(5)
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
    # Stubs for less-critical methods
    # ------------------------------------------------------------------

    def read_temperatures(self):
        """No real temp sensor — return None for each ADC slot."""
        return [None] * config.NUM_ADCS

    def deinit(self):
        print("[STUB] StubADCManager: deinit (nothing to release).")


# Allow   from stub_adc_manager import ADCManager
ADCManager = StubADCManager
