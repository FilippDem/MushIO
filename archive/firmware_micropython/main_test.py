"""
MushIO V1.0 - Pico Demo Mode Entry Point

Identical to main.py but uses DemoADCManager (synthetic sine-wave data)
instead of the real ADCManager.  No SPI bus, ADS124S08 chips, external
clock, or AFE hardware required.

What is exercised (real, not simulated):
  - WiFi connection + reconnection logic
  - TCP data streaming at ~200 FPS
  - 228-byte frame format + CRC-16
  - CMD server on port 9001 (Ping, Status, Scan All, Blink LED all work)
  - Ring buffer + dropped-frame counter
  - Watchdog timer (8-second timeout)
  - Built-in LED blink via 'blink_led' command

Setup:
  1. Edit config.py — fill in WIFI_SSID, WIFI_PASSWORD, HOST_IP.
  2. Copy to the Pico root:
       demo_adc_manager.py
       main_test.py          (this file)
       streamer.py
       cmd_server.py
       config.py
  3. On the host PC, start the GUI:
       python host/gui.py          (connects automatically)
  4. Run on the Pico:
       mpremote run main_test.py
     Or rename to main.py and copy for auto-start on boot.
"""

import time
import machine
from machine import Pin, WDT
from demo_adc_manager import ADCManager   # synthetic data, no hardware
from streamer import DataStreamer
import cmd_server
import config


# =============================================================================
# Status LED patterns
# =============================================================================

def led_solid(led, on=True):
    led.value(1 if on else 0)


def led_blink_fast(led, count=5, on_ms=100, off_ms=100):
    """Fast blink to indicate error."""
    for _ in range(count):
        led.value(1)
        time.sleep_ms(on_ms)
        led.value(0)
        time.sleep_ms(off_ms)


def led_blink_slow(led):
    """Single slow toggle for heartbeat."""
    led.value(not led.value())


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("  MushIO V1.0  |  Pico Demo Mode")
    print("=" * 60)
    print()

    # -------------------------------------------------------------------------
    # Phase 1: Hardware Init
    # -------------------------------------------------------------------------

    # Use the built-in LED on the Pico W (no ADC_CLK_EN pin needed)
    status_led = Pin("LED", Pin.OUT)
    status_led.value(1)
    print("[INIT] Built-in LED on (initializing)...")

    # Startup blink: 5 rapid flashes so you can see the LED works
    # immediately, before WiFi blocks anything.
    for _ in range(5):
        status_led.value(0); time.sleep_ms(80)
        status_led.value(1); time.sleep_ms(80)
    print("[INIT] LED OK (startup blink complete).")

    # Initialize demo ADC subsystem
    adc_mgr = ADCManager()
    try:
        adc_mgr.init()
    except RuntimeError as e:
        print(f"[FATAL] ADC init failed: {e}")
        while True:
            led_blink_fast(status_led, count=3)
            time.sleep(1)

    # Quick sanity read: read one channel from ADC0
    print("[INIT] Test read ADC0 channel 0...")
    test_val = adc_mgr.read_single_channel(0, 0)
    print(f"  ADC0 CH0 = {test_val} (synthetic 24-bit)")

    # -------------------------------------------------------------------------
    # Phase 2: WiFi + TCP Connection
    # -------------------------------------------------------------------------

    streamer = DataStreamer()

    # Only try 3 times during init so startup is fast.
    # The main loop retries every 5 s automatically.
    print("[INIT] Connecting to WiFi (3 attempts max)...")
    if not streamer.connect_wifi(max_retries=3):
        print("[WARN] WiFi not reached yet — will retry in main loop.")
    else:
        print("[INIT] Connecting to host server...")
        if not streamer.connect_server():
            print("[WARN] TCP connection failed. Will retry in main loop.")

    # -------------------------------------------------------------------------
    # Phase 3: Start Command Server on core 1 (port 9001)
    # -------------------------------------------------------------------------

    cmd_server.set_adc_manager(adc_mgr)
    cmd_server.set_led(status_led)
    srv = cmd_server.CmdServer()
    srv.init()   # bind + listen, no thread

    # -------------------------------------------------------------------------
    # Phase 4: Enable Watchdog
    # -------------------------------------------------------------------------

    # RP2350 MicroPython limits WDT to ~8388 ms.  Try long first so mpremote
    # re-deploy doesn't fire the watchdog; fall back to the hardware maximum.
    wdt = None
    for _wdt_ms in (30000, 8388, 8000):
        try:
            wdt = WDT(timeout=_wdt_ms)
            print(f"[INIT] Watchdog enabled ({_wdt_ms} ms timeout — Demo mode).")
            break
        except (ValueError, Exception):
            continue
    if wdt is None:
        print("[WARN] WDT not available — watchdog disabled for this session.")

    # -------------------------------------------------------------------------
    # Phase 5: Main Acquisition Loop
    # -------------------------------------------------------------------------

    print()
    print("[RUN] Starting Pico Demo Mode data acquisition...")
    print(f"  Channels: {config.TOTAL_CHANNELS} "
          f"({config.NUM_RECORDING_CHANNELS} recording, {config.NUM_STIM_CHANNELS} STIM)")
    print(f"  Mode: SYNTHETIC sine waves (no real ADC hardware)")
    print(f"  Expected frame rate: ~200 fps")
    print(f"  Streaming to: {config.HOST_IP}:{config.HOST_PORT}")
    print()

    seq             = 0
    frame_count     = 0
    last_stats_time = time.ticks_ms()
    last_fps_time   = time.ticks_ms()
    last_blink_time = time.ticks_ms()
    fps_count       = 0

    while True:
        try:
            # Feed watchdog
            if wdt:
                wdt.feed()

            # Poll CMD server for GUI connections/commands (~0 ms if idle)
            srv.poll_once()

            # Scan all 72 channels (synthetic)
            frame_data = adc_mgr.scan_frame()
            timestamp = time.ticks_ms()

            # Build frame, push to ring buffer, then drain over TCP.
            # All three calls run on Core 0 to avoid GIL contention with
            # Core 1's cmd_server.  _do_stream_work() batches ~40 frames
            # into one sock.write() so per-call overhead is small.
            packet = streamer.build_frame(timestamp, seq, frame_data)
            streamer.push_frame(packet)
            streamer._do_stream_work()

            seq = (seq + 1) & 0xFFFF
            frame_count += 1
            fps_count += 1

            # ---- Periodic Tasks (every ~10 seconds) ----
            now = time.ticks_ms()

            # Print stats and FPS
            if time.ticks_diff(now, last_stats_time) >= 10000:
                elapsed_s = time.ticks_diff(now, last_fps_time) / 1000.0
                fps = fps_count / elapsed_s if elapsed_s > 0 else 0
                stats = streamer.stats
                print(f"[RUN] Frame #{frame_count}  FPS: {fps:.1f}  ", end="")
                streamer.print_stats()

                # Push latest stats to command server for GUI queries
                cmd_server.update_status(
                    frame_count = frame_count,
                    fps         = fps,
                    crc_errors  = 0,
                    missed      = 0,
                    dropped     = stats['dropped'],
                    buffered    = stats['buffered'],
                    wifi        = streamer.is_wifi_connected(),
                    tcp         = streamer._connected_tcp,
                )

                last_stats_time = now
                last_fps_time = now
                fps_count = 0

                # Toggle LED as heartbeat
                led_blink_slow(status_led)

            # LED heartbeat: double-blink every 2 seconds (no FPS impact)
            if time.ticks_diff(now, last_blink_time) >= 2000:
                status_led.value(1); time.sleep_ms(60)
                status_led.value(0); time.sleep_ms(60)
                status_led.value(1); time.sleep_ms(60)
                status_led.value(0)
                last_blink_time = now

            # TCP reconnection is handled by _do_stream_work() above.

        except KeyboardInterrupt:
            print("\n[RUN] Stopped by user.")
            break

        except Exception as e:
            # Log error but keep running - watchdog will reset if we're stuck
            print(f"[ERR] Main loop exception: {e}")
            time.sleep_ms(100)

    # -------------------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------------------

    print("[SHUTDOWN] Cleaning up...")
    adc_mgr.deinit()
    streamer.deinit()
    led_solid(status_led, False)
    print("[SHUTDOWN] Done.")


# Auto-start on boot
if __name__ == "__main__":
    main()
