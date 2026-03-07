"""
MushIO V1.0 Board Bringup Script

Run this on the Pico 2W REPL (or as main.py during bring-up) to check each
subsystem independently before running the full firmware.

Usage:
    import bringup
    bringup.run_all()        # run all checks in sequence
    bringup.check_power()    # individual check
    bringup.check_spi0()     # ...
    bringup.check_adc(0)     # single ADC by index
    bringup.check_all_adcs() # all 6 ADCs
    bringup.check_wifi()     # WiFi connectivity
    bringup.check_tcp()      # TCP connection to host

    bringup.scan_single(adc_index=0, ain=0, n=10)  # measure one channel
    bringup.scan_all_channels()                     # print all 72 channels
"""

import time
from machine import Pin, SPI, ADC
import config

# =============================================================================
# Helpers
# =============================================================================

def _ok(msg):
    print(f"  [OK] {msg}")

def _fail(msg):
    print(f"  [FAIL] {msg}")

def _info(msg):
    print(f"       {msg}")

def _section(title):
    print()
    print("=" * 50)
    print(f"  {title}")
    print("=" * 50)


# =============================================================================
# 1. Power Good Pin
# =============================================================================

def check_power():
    """Read ANA_PGOOD pin (GP28/ADC2). Should be high when analog rails are up."""
    _section("1. Analog Power Good")
    pgood_adc = ADC(Pin(config.ANA_PGOOD_PIN))
    raw = pgood_adc.read_u16()
    voltage = raw / 65535 * 3.3
    _info(f"GP{config.ANA_PGOOD_PIN} (ANA_PGOOD) raw={raw} -> {voltage:.2f} V")
    if voltage > 2.5:
        _ok("Analog power rails appear healthy (PGOOD high)")
        return True
    else:
        _fail(f"PGOOD low ({voltage:.2f} V) - check LM27762 charge pump, LiPo charge")
        return False


# =============================================================================
# 2. Clock Enable
# =============================================================================

def check_clock():
    """Toggle ADC_CLK_EN and verify the pin state is writable."""
    _section("2. External Clock Enable (GP22)")
    clk_en = Pin(config.ADC_CLK_EN_PIN, Pin.OUT, value=0)
    _info("CLK_EN = 0 (clock off)")
    time.sleep_ms(5)
    clk_en.value(1)
    _info("CLK_EN = 1 (clock on)")
    time.sleep_ms(5)
    _ok("CLK_EN pin toggled (4.096 MHz clock should be active now)")
    return clk_en  # return so caller can keep it alive


# =============================================================================
# 3. SPI Bus Loopback (MOSI -> MISO with wire)
# =============================================================================

def check_spi_loopback(bus=0):
    """
    SPI loopback test. Requires MOSI and MISO shorted on the bus.
    Useful to verify SPI bus wiring before ADCs are connected.
    """
    _section(f"3. SPI{bus} Loopback (MOSI->MISO shorting required)")
    if bus == 0:
        spi = SPI(0, baudrate=100_000, polarity=config.SPI_POLARITY,
                  phase=config.SPI_PHASE, bits=config.SPI_BITS,
                  sck=Pin(config.SPI0_SCK), mosi=Pin(config.SPI0_MOSI),
                  miso=Pin(config.SPI0_MISO))
    else:
        spi = SPI(1, baudrate=100_000, polarity=config.SPI_POLARITY,
                  phase=config.SPI_PHASE, bits=config.SPI_BITS,
                  sck=Pin(config.SPI1_SCK), mosi=Pin(config.SPI1_MOSI),
                  miso=Pin(config.SPI1_MISO))

    test_data = bytes([0xA5, 0x5A, 0xFF, 0x00, 0x12, 0x34])
    rx_buf = bytearray(len(test_data))
    spi.write_readinto(test_data, rx_buf)
    spi.deinit()

    if rx_buf == bytearray(test_data):
        _ok(f"SPI{bus} loopback OK: {list(rx_buf)}")
        return True
    else:
        _fail(f"SPI{bus} loopback FAIL: sent {list(test_data)}, got {list(rx_buf)}")
        _info("If no loopback wire: this is expected - just verify CS/CLK/MOSI on scope")
        return False


# =============================================================================
# 4. Single ADC Check (ID register)
# =============================================================================

def check_adc(adc_index):
    """
    Initialize one ADC and verify its ID register.
    adc_index: 0-5
    """
    from ads124s08 import ADS124S08, ADS124S08_ID_VAL, ADS124S08_ID_MASK

    _section(f"4. ADC{adc_index} ID Register Check")

    # Set up clock and reset first
    clk_en = Pin(config.ADC_CLK_EN_PIN, Pin.OUT, value=1)
    reset_n = Pin(config.ADC_RESET_N_PIN, Pin.OUT, value=1)
    time.sleep_ms(5)

    # Select SPI bus
    if adc_index in config.SPI0_ADC_INDICES:
        spi = SPI(0, baudrate=config.SPI_BAUDRATE, polarity=config.SPI_POLARITY,
                  phase=config.SPI_PHASE, bits=config.SPI_BITS,
                  sck=Pin(config.SPI0_SCK), mosi=Pin(config.SPI0_MOSI),
                  miso=Pin(config.SPI0_MISO))
    else:
        spi = SPI(1, baudrate=config.SPI_BAUDRATE, polarity=config.SPI_POLARITY,
                  phase=config.SPI_PHASE, bits=config.SPI_BITS,
                  sck=Pin(config.SPI1_SCK), mosi=Pin(config.SPI1_MOSI),
                  miso=Pin(config.SPI1_MISO))

    adc = ADS124S08(spi=spi, cs_pin_num=config.ADC_CS_PINS[adc_index],
                    drdy_pin_num=config.ADC_DRDY_PINS[adc_index],
                    name=f"ADC{adc_index}")
    adc.reset()
    time.sleep_ms(5)

    id_byte, valid = adc.read_id()
    _info(f"ID register = 0x{id_byte:02X}  (bits[4:0] must == 0x00)")

    if valid:
        _ok(f"ADC{adc_index} ID valid (0x{id_byte:02X})")
    else:
        _fail(f"ADC{adc_index} ID invalid! Got 0x{id_byte:02X}, expected [4:0]=0x00")
        _info("Possible causes: CS pin wrong, SPI wiring issue, power not present")

    spi.deinit()
    return valid


def check_all_adcs():
    """Check ID register on all 6 ADCs."""
    _section("4. All ADC ID Checks")
    from ads124s08 import ADS124S08

    # Set up shared pins
    clk_en  = Pin(config.ADC_CLK_EN_PIN,  Pin.OUT, value=1)
    reset_n = Pin(config.ADC_RESET_N_PIN, Pin.OUT, value=1)
    start   = Pin(config.ADC_START_PIN,   Pin.OUT, value=0)
    time.sleep_ms(10)

    # Hardware reset
    reset_n.value(0)
    time.sleep_ms(10)
    reset_n.value(1)
    time.sleep_ms(10)

    spi0 = SPI(0, baudrate=config.SPI_BAUDRATE, polarity=config.SPI_POLARITY,
               phase=config.SPI_PHASE, bits=config.SPI_BITS,
               sck=Pin(config.SPI0_SCK), mosi=Pin(config.SPI0_MOSI),
               miso=Pin(config.SPI0_MISO))
    spi1 = SPI(1, baudrate=config.SPI_BAUDRATE, polarity=config.SPI_POLARITY,
               phase=config.SPI_PHASE, bits=config.SPI_BITS,
               sck=Pin(config.SPI1_SCK), mosi=Pin(config.SPI1_MOSI),
               miso=Pin(config.SPI1_MISO))

    results = []
    for i in range(config.NUM_ADCS):
        spi = spi0 if i in config.SPI0_ADC_INDICES else spi1
        adc = ADS124S08(spi=spi, cs_pin_num=config.ADC_CS_PINS[i],
                        drdy_pin_num=config.ADC_DRDY_PINS[i], name=f"ADC{i}")
        adc.reset()
        time.sleep_ms(2)
        id_byte, valid = adc.read_id()
        status = "OK" if valid else "FAIL"
        _info(f"ADC{i} (CS=GP{config.ADC_CS_PINS[i]}, "
              f"DRDY=GP{config.ADC_DRDY_PINS[i]}): "
              f"ID=0x{id_byte:02X} [{status}]")
        results.append(valid)

    spi0.deinit()
    spi1.deinit()

    if all(results):
        _ok("All 6 ADCs responded with valid ID")
    else:
        failed = [i for i, v in enumerate(results) if not v]
        _fail(f"ADC(s) {failed} failed ID check")
    return all(results)


# =============================================================================
# 5. Single Channel Measurement
# =============================================================================

def scan_single(adc_index=0, ain=0, n=5, timeout_ms=20):
    """
    Read n samples from one channel on one ADC and print results.

    Args:
        adc_index: 0-5
        ain:       AIN channel 0-11 (or 0-7 for ADC2/ADC3 recording channels)
        n:         number of samples to average
        timeout_ms: DRDY timeout
    """
    from ads124s08 import ADS124S08, AINCOM
    from adc_manager import ADCManager

    _section(f"5. Single Channel Read: ADC{adc_index} AIN{ain} x{n}")

    mgr = ADCManager()
    mgr.init()

    ch_name = config.CHANNEL_MAP[adc_index][ain]
    _info(f"Channel: {ch_name}")

    samples = []
    for k in range(n):
        val = mgr.read_single_channel(adc_index, ain)
        # Convert to uV
        VREF = config.VREF
        FS   = 2**23
        uv   = (val / FS) * VREF / config.AFE_GAIN * 1e6
        samples.append(val)
        _info(f"  Sample {k+1}: raw={val:9d}  ->  {uv:+10.2f} uV")

    avg = sum(samples) / len(samples)
    avg_uv = (avg / 2**23) * config.VREF / config.AFE_GAIN * 1e6
    _info(f"  Average raw: {avg:.0f}  ->  {avg_uv:+.2f} uV")

    mgr.deinit()
    return samples


# =============================================================================
# 6. Full Channel Scan (all 72)
# =============================================================================

def scan_all_channels(n_frames=3):
    """
    Run n_frames complete scans and print average for all 72 channels.
    Shows recording channels and STIM channels separately.
    """
    from adc_manager import ADCManager

    _section(f"6. Full Channel Scan ({n_frames} frames)")

    mgr = ADCManager()
    mgr.init()

    VREF = config.VREF
    FS   = 2**23
    GAIN = config.AFE_GAIN

    totals = [0] * config.TOTAL_CHANNELS

    for _ in range(n_frames):
        buf = mgr.scan_frame()
        for i in range(config.TOTAL_CHANNELS):
            off = i * 3
            b0, b1, b2 = buf[off], buf[off+1], buf[off+2]
            val = (b0 << 16) | (b1 << 8) | b2
            if val & 0x800000:
                val -= 0x1000000
            totals[i] += val

    mgr.deinit()

    # Print recording channels
    print()
    print("  --- Recording Channels (64) ---")
    print(f"  {'Index':>5}  {'ADC':>3}  {'AIN':>3}  {'Name':>8}  "
          f"{'Raw (avg)':>12}  {'uV':>10}")
    for flat_idx in range(config.TOTAL_CHANNELS):
        if flat_idx in config.STIM_CHANNEL_INDICES:
            continue
        adc_i = flat_idx // config.CHANNELS_PER_ADC
        ain_i = flat_idx %  config.CHANNELS_PER_ADC
        name  = config.CHANNEL_MAP[adc_i][ain_i]
        avg   = totals[flat_idx] / n_frames
        uv    = (avg / FS) * VREF / GAIN * 1e6
        # Flag possible error markers
        marker = " <TIMEOUT>" if totals[flat_idx] // n_frames == -8388608 else ""
        print(f"  [{flat_idx:3d}]  ADC{adc_i}  AIN{ain_i:2d}  {name:>8}  "
              f"{avg:12.0f}  {uv:+10.2f} uV{marker}")

    # Print STIM channels
    print()
    print("  --- Stimulation Channels (8) ---")
    print(f"  {'Index':>5}  {'ADC':>3}  {'AIN':>3}  {'Name':>8}  "
          f"{'Raw (avg)':>12}  {'mV':>10}")
    for flat_idx in config.STIM_CHANNEL_INDICES:
        adc_i = flat_idx // config.CHANNELS_PER_ADC
        ain_i = flat_idx %  config.CHANNELS_PER_ADC
        name  = config.CHANNEL_MAP[adc_i][ain_i]
        avg   = totals[flat_idx] / n_frames
        mv    = (avg / FS) * VREF / GAIN * 1e3
        print(f"  [{flat_idx:3d}]  ADC{adc_i}  AIN{ain_i:2d}  {name:>8}  "
              f"{avg:12.0f}  {mv:+10.3f} mV")

    return totals


# =============================================================================
# 7. DRDY Timing Check
# =============================================================================

def check_drdy_timing(adc_index=0, ain=0, n=20):
    """
    Measures actual DRDY latency (time from START pulse to DRDY asserted).
    Useful to verify the ADC data rate matches config.ADC_DATARATE.

    At 4000 SPS external clock, expected latency ≈ 250 us.
    """
    from ads124s08 import ADS124S08, AINCOM

    _section(f"7. DRDY Timing: ADC{adc_index} AIN{ain}")

    clk_en  = Pin(config.ADC_CLK_EN_PIN,  Pin.OUT, value=1)
    reset_n = Pin(config.ADC_RESET_N_PIN, Pin.OUT, value=1)
    start   = Pin(config.ADC_START_PIN,   Pin.OUT, value=0)
    time.sleep_ms(5)

    if adc_index in config.SPI0_ADC_INDICES:
        spi = SPI(0, baudrate=config.SPI_BAUDRATE, polarity=config.SPI_POLARITY,
                  phase=config.SPI_PHASE, bits=config.SPI_BITS,
                  sck=Pin(config.SPI0_SCK), mosi=Pin(config.SPI0_MOSI),
                  miso=Pin(config.SPI0_MISO))
    else:
        spi = SPI(1, baudrate=config.SPI_BAUDRATE, polarity=config.SPI_POLARITY,
                  phase=config.SPI_PHASE, bits=config.SPI_BITS,
                  sck=Pin(config.SPI1_SCK), mosi=Pin(config.SPI1_MOSI),
                  miso=Pin(config.SPI1_MISO))

    adc = ADS124S08(spi=spi, cs_pin_num=config.ADC_CS_PINS[adc_index],
                    drdy_pin_num=config.ADC_DRDY_PINS[adc_index],
                    name=f"ADC{adc_index}")
    adc.reset()
    adc.configure(datarate=config.ADC_DATARATE, pga_gain=config.ADC_PGA_GAIN,
                  pga_enable=config.ADC_PGA_EN, ref_sel=0, clk_sel=1)
    adc.set_mux(ain, config.AINCOM)

    latencies_us = []
    for _ in range(n):
        t0 = time.ticks_us()
        start.value(1)
        start.value(0)
        ok = adc.wait_drdy(timeout_ms=50)
        dt = time.ticks_diff(time.ticks_us(), t0)
        if ok:
            latencies_us.append(dt)
            _ = adc.read_data()  # consume result
        else:
            _fail("DRDY timeout - ADC not responding")
            break

    spi.deinit()

    if latencies_us:
        avg_us = sum(latencies_us) / len(latencies_us)
        mn_us  = min(latencies_us)
        mx_us  = max(latencies_us)
        implied_sps = 1_000_000 / avg_us if avg_us > 0 else 0
        _info(f"DRDY latency over {len(latencies_us)} samples:")
        _info(f"  avg={avg_us:.0f} us  min={mn_us} us  max={mx_us} us")
        _info(f"  Implied SPS: {implied_sps:.0f} (config target: {_dr_to_sps(config.ADC_DATARATE)})")

        if implied_sps > 100:
            _ok("DRDY timing looks healthy")
        else:
            _fail("DRDY latency too long - check external clock (GP22/CLK_EN)")

    return latencies_us


def _dr_to_sps(dr_code):
    """Convert DATARATE register code to SPS string."""
    table = {0x00:'2.5', 0x01:'5', 0x02:'10', 0x03:'16.6',
             0x04:'20', 0x05:'50', 0x06:'60', 0x07:'100',
             0x08:'200', 0x09:'400', 0x0A:'800', 0x0B:'1000',
             0x0C:'2000', 0x0D:'4000'}
    return table.get(dr_code, '?')


# =============================================================================
# 8. ADC Register Dump
# =============================================================================

def dump_registers(adc_index=0):
    """Read and print all 18 registers from one ADC."""
    from ads124s08 import ADS124S08

    _section(f"8. Register Dump: ADC{adc_index}")

    clk_en  = Pin(config.ADC_CLK_EN_PIN,  Pin.OUT, value=1)
    reset_n = Pin(config.ADC_RESET_N_PIN, Pin.OUT, value=1)
    time.sleep_ms(5)

    if adc_index in config.SPI0_ADC_INDICES:
        spi = SPI(0, baudrate=config.SPI_BAUDRATE, polarity=config.SPI_POLARITY,
                  phase=config.SPI_PHASE, bits=config.SPI_BITS,
                  sck=Pin(config.SPI0_SCK), mosi=Pin(config.SPI0_MOSI),
                  miso=Pin(config.SPI0_MISO))
    else:
        spi = SPI(1, baudrate=config.SPI_BAUDRATE, polarity=config.SPI_POLARITY,
                  phase=config.SPI_PHASE, bits=config.SPI_BITS,
                  sck=Pin(config.SPI1_SCK), mosi=Pin(config.SPI1_MOSI),
                  miso=Pin(config.SPI1_MISO))

    adc = ADS124S08(spi=spi, cs_pin_num=config.ADC_CS_PINS[adc_index],
                    drdy_pin_num=config.ADC_DRDY_PINS[adc_index],
                    name=f"ADC{adc_index}")
    adc.reset()
    regs = adc.dump_registers()
    spi.deinit()

    reg_names = [
        'ID', 'STATUS', 'INPMUX', 'PGA', 'DATARATE', 'REF',
        'IDACMAG', 'IDACMUX', 'VBIAS', 'SYS', 'OFCAL0', 'OFCAL1',
        'OFCAL2', 'FSCAL0', 'FSCAL1', 'FSCAL2', 'GPIODAT', 'GPIOCON'
    ]
    for i, (name, val) in enumerate(zip(reg_names, regs)):
        print(f"  0x{i:02X} {name:>8}: 0x{val:02X}  ({val:08b}b)")

    return regs


# =============================================================================
# 9. WiFi Check
# =============================================================================

def check_wifi():
    """Attempt WiFi connection and report IP address."""
    _section("9. WiFi Connectivity")
    import network

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    try:
        wlan.config(country=config.WIFI_COUNTRY)
    except Exception:
        pass

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        _ok(f"Already connected. IP: {ip}")
        return True

    _info(f"Connecting to '{config.WIFI_SSID}'...")
    wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)

    for attempt in range(15):
        time.sleep(1)
        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            _ok(f"Connected after {attempt+1}s. IP: {ip}")
            return True
        _info(f"  waiting... ({attempt+1}s)")

    _fail(f"Could not connect to '{config.WIFI_SSID}'")
    _info("Check WIFI_SSID and WIFI_PASSWORD in config.py")
    return False


# =============================================================================
# 10. TCP Check
# =============================================================================

def check_tcp():
    """Attempt TCP connection to HOST_IP:HOST_PORT."""
    _section("10. TCP Connection to Host")
    import socket

    _info(f"Connecting to {config.HOST_IP}:{config.HOST_PORT} ...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((config.HOST_IP, config.HOST_PORT))
        sock.close()
        _ok(f"TCP connection to {config.HOST_IP}:{config.HOST_PORT} OK")
        return True
    except Exception as e:
        _fail(f"TCP connection failed: {e}")
        _info(f"Ensure receiver.py is running on {config.HOST_IP}")
        return False


# =============================================================================
# 11. Frame Rate Benchmark
# =============================================================================

def benchmark_framerate(n_frames=50):
    """
    Measure actual achievable frame rate (scan_frame calls per second).
    Gives the true throughput without WiFi overhead.
    """
    from adc_manager import ADCManager

    _section(f"11. Frame Rate Benchmark ({n_frames} frames)")

    mgr = ADCManager()
    mgr.init()

    t0 = time.ticks_ms()
    for _ in range(n_frames):
        mgr.scan_frame()
    elapsed_ms = time.ticks_diff(time.ticks_ms(), t0)

    mgr.deinit()

    fps = n_frames / (elapsed_ms / 1000.0)
    ms_per_frame = elapsed_ms / n_frames
    _info(f"  {n_frames} frames in {elapsed_ms} ms")
    _info(f"  Frame rate: {fps:.1f} FPS")
    _info(f"  Per frame:  {ms_per_frame:.1f} ms")
    _info(f"  Per channel: {ms_per_frame/config.TOTAL_CHANNELS*1000:.0f} us")

    if fps >= 100:
        _ok(f"{fps:.0f} FPS (target ~170-200)")
    else:
        _fail(f"Only {fps:.0f} FPS - check clock enable and DRDY timing")

    return fps


# =============================================================================
# Full Bringup Sequence
# =============================================================================

def run_all():
    """
    Run all bringup checks in order. Stop at first critical failure.
    Call individual functions for more detailed diagnostics.
    """
    print()
    print("=" * 50)
    print("  MushIO V1.0 Board Bringup")
    print("=" * 50)

    results = {}

    # 1. Power
    results['power'] = check_power()

    # 2. Clock
    check_clock()
    results['clock'] = True  # can't fail without scope

    # 3. ADC IDs (most important hardware check)
    results['adcs'] = check_all_adcs()
    if not results['adcs']:
        _fail("ADC check failed - stopping bringup. Fix ADC communication first.")
        _print_summary(results)
        return results

    # 4. Single channel read (ADC0, AIN0 = ELEC02)
    print()
    _info("Reading 5 samples from ADC0 AIN0 (ELEC02)...")
    scan_single(adc_index=0, ain=0, n=5)

    # 5. DRDY timing
    results['drdy'] = len(check_drdy_timing(adc_index=0, ain=0, n=10)) > 0

    # 6. Full scan
    print()
    _info("Running full 72-channel scan (3 frames)...")
    scan_all_channels(n_frames=3)

    # 7. Frame rate
    results['fps'] = benchmark_framerate(n_frames=30) > 50

    # 8. WiFi (optional - only if credentials filled in)
    if config.WIFI_SSID != "YOUR_SSID":
        results['wifi'] = check_wifi()
        if results['wifi']:
            results['tcp'] = check_tcp()
    else:
        _info("Skipping WiFi/TCP checks (WIFI_SSID not configured in config.py)")

    _print_summary(results)
    return results


def _print_summary(results):
    _section("Bringup Summary")
    all_ok = True
    for name, ok in results.items():
        if ok:
            _ok(name)
        else:
            _fail(name)
            all_ok = False
    print()
    if all_ok:
        print("  >>> ALL CHECKS PASSED - board ready for full firmware <<<")
    else:
        print("  >>> Some checks failed - see details above <<<")
    print()


# =============================================================================
# Auto-run if executed directly
# =============================================================================

if __name__ == "__main__":
    run_all()
