"""
MushIO V1.0 - Main Entry Point

72-channel electrophysiology recorder for mushroom mycelium studies.
Streams 24-bit ADC data over WiFi to a host TCP server.

Copy all firmware/*.py files to the Pico 2W filesystem.
Edit config.py with your WiFi credentials and host IP before running.
"""

import time
import machine
from machine import Pin, WDT
from adc_manager import ADCManager
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
    print("  MushIO V1.0  |  Real-Time Biopotential Monitor")
    print("=" * 60)
    print()

    # -------------------------------------------------------------------------
    # Phase 1: Hardware Init
    # -------------------------------------------------------------------------

    # Note: GP22 may be shared with ADC_CLK_EN on some board revisions.
    # If so, use a different GPIO for the status LED or skip LED control.
    # For now, we only toggle LED during init, then use clk_en_pin for clock.
    status_led = Pin(config.ADC_CLK_EN_PIN, Pin.OUT, value=1)
    print("[INIT] Status LED on (initializing)...")

    # Initialize ADC subsystem
    adc_mgr = ADCManager()
    try:
        adc_mgr.init()
    except RuntimeError as e:
        print(f"[FATAL] ADC init failed: {e}")
        # Blink fast to indicate error, then halt
        while True:
            led_blink_fast(status_led, count=3)
            time.sleep(1)

    # Quick sanity read: read one channel from ADC0
    print("[INIT] Test read ADC0 channel 0...")
    test_val = adc_mgr.read_single_channel(0, 0)
    print(f"  ADC0 CH0 = {test_val} (raw 24-bit)")

    # -------------------------------------------------------------------------
    # Phase 2: WiFi + TCP Connection
    # -------------------------------------------------------------------------

    streamer = DataStreamer()

    print("[INIT] Connecting to WiFi...")
    if not streamer.connect_wifi():
        print("[WARN] WiFi connection failed. Will retry in main loop.")
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
    srv.start()
    print(f"[INIT] Command server on port {cmd_server.CmdServer.PORT} "
          f"(connect GUI command channel here)")

    # -------------------------------------------------------------------------
    # Phase 4: Enable Watchdog
    # -------------------------------------------------------------------------

    # WDT with 8 second timeout - if main loop stalls, board resets
    wdt = WDT(timeout=8000)
    print("[INIT] Watchdog enabled (8s timeout).")

    # -------------------------------------------------------------------------
    # Phase 4: Main Acquisition Loop
    # -------------------------------------------------------------------------

    print()
    print("[RUN] Starting data acquisition...")
    print(f"  Channels: {config.TOTAL_CHANNELS} "
          f"({config.NUM_RECORDING_CHANNELS} recording, {config.NUM_STIM_CHANNELS} STIM)")
    print(f"  ADC data rate: {config.ADC_DATARATE} (register code)")
    print(f"  Expected frame rate: ~170-200 fps")
    print(f"  Streaming to: {config.HOST_IP}:{config.HOST_PORT}")
    print()

    seq                 = 0
    frame_count         = 0
    last_stats_time     = time.ticks_ms()
    last_reconnect_time = time.ticks_ms()
    last_fps_time       = time.ticks_ms()
    fps_count           = 0

    while True:
        try:
            # Feed watchdog
            wdt.feed()

            # Yield to command server if it requested a pause (< 5ms)
            cmd_server.check_pause()

            # Scan all 72 channels
            frame_data = adc_mgr.scan_frame()
            timestamp = time.ticks_ms()

            # Build and send frame
            packet = streamer.build_frame(timestamp, seq, frame_data)
            streamer.send_frame(packet)

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

            # Attempt reconnection if disconnected
            if not streamer.is_wifi_connected() or not streamer._connected_tcp:
                if time.ticks_diff(now, last_reconnect_time) >= config.RECONNECT_INTERVAL_S * 1000:
                    print("[RUN] Attempting reconnection...")
                    streamer.reconnect()
                    last_reconnect_time = now

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
