"""
MushIO V1.0 - TCP Command Server (polled, single-core)

Non-blocking command server on port 9001.  The main loop calls
poll_once() every iteration (~3 ms) to accept GUI connections and
process commands.  No _thread — eliminates GIL contention that
throttled Core 0's scan+stream loop to ~16 FPS.

Protocol:
  Host sends: one-line command + newline
  Pico sends:  zero or more response lines, terminated by "END\\r\\n"

Commands:
  ping                       - connectivity check
  status                     - streaming statistics
  help                       - list all commands
  scan_single <adc> <ain> <n> - N samples from one channel
  check_adcs                 - verify all 6 ADC IDs
  scan_all                   - print all 72 channel values
  drdy <adc> <ain>           - measure DRDY timing (20 samples)
  dump_regs <adc>            - print all 18 ADS124S08 registers
  set_datarate <0x00-0x0D>   - change ADC data rate on all ADCs
  benchmark                  - measure maximum scan_frame() rate
  blink_led [count]          - blink status LED N times (proof of life)
"""

import socket
import select
import time
import config


# =============================================================================
# Shared state - written by main loop, read by cmd server
# =============================================================================

stream_status = {
    'frame_count': 0,
    'fps':         0.0,
    'crc_errors':  0,
    'missed':      0,
    'dropped':     0,
    'buffered':    0,
    'wifi':        False,
    'tcp':         False,
}

# ADC manager reference - set by main.py after init
_adc_mgr = None

# Status LED pin - set by main.py
_status_led = None


def set_led(pin):
    """Called by main.py to give the cmd server access to the status LED."""
    global _status_led
    _status_led = pin


def set_adc_manager(mgr):
    """Called by main.py after ADCManager.init() completes."""
    global _adc_mgr
    _adc_mgr = mgr


def update_status(frame_count, fps, crc_errors, missed, dropped, buffered,
                  wifi, tcp):
    """Called by main loop periodically to update shared stats."""
    stream_status.update({
        'frame_count': frame_count,
        'fps':         fps,
        'crc_errors':  crc_errors,
        'missed':      missed,
        'dropped':     dropped,
        'buffered':    buffered,
        'wifi':        wifi,
        'tcp':         tcp,
    })


# =============================================================================
# Command Server  (polled — no threading)
# =============================================================================

class CmdServer:
    """Non-blocking TCP command server.  Call init() once, then poll_once()
    from the main loop on every iteration."""

    PORT = 9001

    def __init__(self):
        self._srv    = None
        self._conn   = None
        self._buf    = b''
        self._poller = None

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def init(self):
        """Bind, listen, and register with poll.  Call once before main loop."""
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(('0.0.0.0', self.PORT))
        self._srv.listen(1)
        self._srv.setblocking(False)
        self._poller = select.poll()
        self._poller.register(self._srv, select.POLLIN)
        print(f"[CMD] Command server listening on port {self.PORT}")

    def poll_once(self):
        """Non-blocking check for new connections and incoming commands.
        Call from the main loop every iteration (~3 ms)."""
        for obj, ev in self._poller.poll(0):
            if obj == self._srv:
                self._try_accept()
            elif self._conn is not None and obj == self._conn:
                self._try_recv()

    # -------------------------------------------------------------------------
    # Connection management
    # -------------------------------------------------------------------------

    def _try_accept(self):
        try:
            conn, addr = self._srv.accept()
        except OSError:
            return
        # Replace any existing client
        if self._conn is not None:
            self._close_client()
        self._conn = conn
        self._buf = b''
        print(f"[CMD] GUI connected from {addr[0]}")
        try:
            self._conn.settimeout(5)
            self._tx(self._conn, "=" * 60)
            self._tx(self._conn, " MushIO V1.0  |  Real-Time Biopotential Monitor")
            self._tx(self._conn, "=" * 60)
            self._tx(self._conn, "Type 'help' for a list of commands.")
            self._conn.setblocking(False)
            self._poller.register(self._conn, select.POLLIN)
        except Exception as e:
            print(f"[CMD] Banner send failed: {e}")
            self._close_client()

    def _try_recv(self):
        try:
            chunk = self._conn.recv(256)
        except OSError:
            return   # would-block — no data yet
        if not chunk:
            self._close_client()
            return
        self._buf += chunk
        while b'\n' in self._buf:
            line, self._buf = self._buf.split(b'\n', 1)
            cmd = line.decode('utf-8', 'ignore').strip()
            if cmd:
                try:
                    self._conn.setblocking(True)
                    self._conn.settimeout(10)
                    self._dispatch(self._conn, cmd)
                    if self._conn:
                        self._conn.setblocking(False)
                except Exception as e:
                    print(f"[CMD] Dispatch error: {e}")
                    self._close_client()
                    return

    def _close_client(self):
        if self._conn is not None:
            try:
                self._poller.unregister(self._conn)
            except Exception:
                pass
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None
            self._buf = b''
            print("[CMD] GUI disconnected")

    # -------------------------------------------------------------------------
    # Transport helpers
    # -------------------------------------------------------------------------

    def _tx(self, conn, text):
        try:
            conn.sendall((text + '\r\n').encode())
        except Exception:
            pass

    def _end(self, conn):
        try:
            conn.sendall(b'END\r\n')
        except Exception:
            pass

    # -------------------------------------------------------------------------
    # Command dispatcher
    # -------------------------------------------------------------------------

    def _dispatch(self, conn, cmd_str):
        parts = cmd_str.split()
        op    = parts[0].lower() if parts else ''
        try:
            if   op == 'ping':          self._ping(conn)
            elif op == 'status':        self._status(conn)
            elif op == 'help':          self._help(conn)
            elif op == 'scan_single':   self._scan_single(conn, parts)
            elif op == 'check_adcs':    self._check_adcs(conn)
            elif op == 'scan_all':      self._scan_all(conn)
            elif op == 'drdy':          self._drdy(conn, parts)
            elif op == 'dump_regs':     self._dump_regs(conn, parts)
            elif op == 'set_datarate':  self._set_datarate(conn, parts)
            elif op == 'benchmark':     self._benchmark(conn)
            elif op == 'blink_led':     self._blink_led(conn, parts)
            else:
                self._tx(conn, f"ERROR: Unknown command '{op}'. Type 'help'.")
                self._end(conn)
        except Exception as e:
            self._tx(conn, f"ERROR: {e}")
            self._end(conn)

    # -------------------------------------------------------------------------
    # Commands
    # -------------------------------------------------------------------------

    def _ping(self, conn):
        self._tx(conn, "PONG")
        self._end(conn)

    def _status(self, conn):
        s = stream_status
        self._tx(conn, f"frames    = {s['frame_count']}")
        self._tx(conn, f"fps       = {s['fps']:.1f}")
        self._tx(conn, f"crc_err   = {s['crc_errors']}")
        self._tx(conn, f"missed    = {s['missed']}")
        self._tx(conn, f"dropped   = {s['dropped']}")
        self._tx(conn, f"buffered  = {s['buffered']}")
        self._tx(conn, f"wifi      = {'yes' if s['wifi'] else 'no'}")
        self._tx(conn, f"tcp       = {'yes' if s['tcp'] else 'no'}")
        self._end(conn)

    def _help(self, conn):
        lines = [
            "ping                         check connection",
            "status                       streaming statistics",
            "scan_single <adc> <ain> <n>  read N samples (default adc=0 ain=0 n=5)",
            "check_adcs                   verify all 6 ADC device IDs",
            "scan_all                     print all 72 channel values",
            "drdy <adc> <ain>             measure DRDY latency (20 samples)",
            "dump_regs <adc>              dump all 18 ADS124S08 registers",
            "set_datarate <hex>           change data rate (0x00-0x0D), e.g. 0x0D=4kSPS",
            "benchmark                    measure max scan_frame() rate",
            "blink_led [count]            blink status LED N times (default 5)",
            "help                         this message",
        ]
        for l in lines:
            self._tx(conn, l)
        self._end(conn)

    def _scan_single(self, conn, parts):
        if _adc_mgr is None:
            self._tx(conn, "ERROR: ADC not ready"); self._end(conn); return
        adc_i = int(parts[1]) if len(parts) > 1 else 0
        ain   = int(parts[2]) if len(parts) > 2 else 0
        n     = int(parts[3]) if len(parts) > 3 else 5
        name  = config.CHANNEL_MAP[adc_i][ain]
        self._tx(conn, f"ADC{adc_i} AIN{ain} ({name})  x{n}:")
        vals = []
        for _ in range(n):
            v = _adc_mgr.read_single_channel(adc_i, ain)
            vals.append(v)
        for k, raw in enumerate(vals):
            uv = (raw / 2**23) * config.VREF / config.AFE_GAIN * 1e6
            self._tx(conn, f"  [{k+1}] raw={raw:9d}  {uv:+10.2f} uV")
        avg_raw = sum(vals) / n
        avg_uv  = (avg_raw / 2**23) * config.VREF / config.AFE_GAIN * 1e6
        self._tx(conn, f"  avg  raw={avg_raw:9.0f}  {avg_uv:+10.2f} uV")
        self._end(conn)

    def _check_adcs(self, conn):
        if _adc_mgr is None:
            self._tx(conn, "ERROR: ADC not ready"); self._end(conn); return
        self._tx(conn, "Checking device IDs on all 6 ADCs...")
        results = []
        for i, adc in enumerate(_adc_mgr.adcs):
            id_byte, valid = adc.read_id()
            results.append((i, id_byte, valid))
        all_ok = True
        for i, id_byte, valid in results:
            st = "OK" if valid else "FAIL"
            self._tx(conn, f"  ADC{i}  CS=GP{config.ADC_CS_PINS[i]}"
                           f"  DRDY=GP{config.ADC_DRDY_PINS[i]}"
                           f"  ID=0x{id_byte:02X}  [{st}]")
            if not valid:
                all_ok = False
        self._tx(conn, "All ADCs OK" if all_ok else "FAILURES - check SPI wiring")
        self._end(conn)

    def _scan_all(self, conn):
        if _adc_mgr is None:
            self._tx(conn, "ERROR: ADC not ready"); self._end(conn); return
        self._tx(conn, "Scanning all 72 channels...")
        buf = _adc_mgr.scan_frame()
        FS   = 2**23
        VREF = config.VREF
        GAIN = config.AFE_GAIN
        for flat in range(config.TOTAL_CHANNELS):
            off = flat * 3
            raw = (buf[off] << 16) | (buf[off+1] << 8) | buf[off+2]
            if raw & 0x800000:
                raw -= 0x1000000
            adc_i = flat // config.CHANNELS_PER_ADC
            ain_i = flat %  config.CHANNELS_PER_ADC
            name  = config.CHANNEL_MAP[adc_i][ain_i]
            is_stim = flat in config.STIM_CHANNEL_INDICES
            uv = (raw / FS) * VREF / GAIN * 1e6
            if is_stim:
                self._tx(conn, f"  [{flat:2d}] ADC{adc_i} AIN{ain_i:2d}"
                               f"  {name:>8} [STIM]  raw={raw:9d}  {uv/1000:+8.3f} mV")
            else:
                self._tx(conn, f"  [{flat:2d}] ADC{adc_i} AIN{ain_i:2d}"
                               f"  {name:>8}         raw={raw:9d}  {uv:+10.2f} uV")
        self._end(conn)

    def _drdy(self, conn, parts):
        if _adc_mgr is None:
            self._tx(conn, "ERROR: ADC not ready"); self._end(conn); return
        adc_i = int(parts[1]) if len(parts) > 1 else 0
        ain   = int(parts[2]) if len(parts) > 2 else 0
        self._tx(conn, f"DRDY timing: ADC{adc_i} AIN{ain}  (20 pulses)...")
        adc  = _adc_mgr.adcs[adc_i]
        sp   = _adc_mgr.start_pin
        adc.set_mux(ain, config.AINCOM)
        lats = []
        for _ in range(20):
            t0 = time.ticks_us()
            sp.value(1); sp.value(0)
            ok = adc.wait_drdy(timeout_ms=50)
            dt = time.ticks_diff(time.ticks_us(), t0)
            if ok:
                lats.append(dt)
                adc.read_data()
            else:
                break
        if lats:
            avg = sum(lats) / len(lats)
            sps = 1_000_000 / avg if avg > 0 else 0
            self._tx(conn, f"  Samples: {len(lats)}")
            self._tx(conn, f"  avg={avg:.0f} us  min={min(lats)} us  max={max(lats)} us")
            self._tx(conn, f"  Implied: {sps:.0f} SPS")
        else:
            self._tx(conn, "  DRDY never asserted - check CLK_EN (GP22)")
        self._end(conn)

    def _dump_regs(self, conn, parts):
        if _adc_mgr is None:
            self._tx(conn, "ERROR: ADC not ready"); self._end(conn); return
        adc_i = int(parts[1]) if len(parts) > 1 else 0
        self._tx(conn, f"Register dump: ADC{adc_i}")
        regs = _adc_mgr.adcs[adc_i].dump_registers()
        names = ['ID','STATUS','INPMUX','PGA','DATARATE','REF',
                 'IDACMAG','IDACMUX','VBIAS','SYS','OFCAL0','OFCAL1',
                 'OFCAL2','FSCAL0','FSCAL1','FSCAL2','GPIODAT','GPIOCON']
        for i, (nm, v) in enumerate(zip(names, regs)):
            self._tx(conn, f"  0x{i:02X}  {nm:>8}  0x{v:02X}  {v:08b}b")
        self._end(conn)

    def _set_datarate(self, conn, parts):
        if _adc_mgr is None:
            self._tx(conn, "ERROR: ADC not ready"); self._end(conn); return
        if len(parts) < 2:
            self._tx(conn, "Usage: set_datarate <hex>  e.g. set_datarate 0x0D")
            self._end(conn); return
        dr = int(parts[1], 0)
        if not (0x00 <= dr <= 0x0D):
            self._tx(conn, "ERROR: code must be 0x00-0x0D"); self._end(conn); return
        sps_map = {0x00:'2.5',0x01:'5',0x02:'10',0x03:'16.6',0x04:'20',
                   0x05:'50',0x06:'60',0x07:'100',0x08:'200',0x09:'400',
                   0x0A:'800',0x0B:'1000',0x0C:'2000',0x0D:'4000'}
        sps = sps_map.get(dr, '?')
        self._tx(conn, f"Setting data rate to {sps} SPS on all ADCs...")
        for adc in _adc_mgr.adcs:
            adc.configure(datarate=dr, pga_gain=config.ADC_PGA_GAIN,
                          pga_enable=config.ADC_PGA_EN, ref_sel=0, clk_sel=1)
        self._tx(conn, f"Done. All 6 ADCs now at {sps} SPS.")
        self._end(conn)

    def _benchmark(self, conn):
        if _adc_mgr is None:
            self._tx(conn, "ERROR: ADC not ready"); self._end(conn); return
        n = 30
        self._tx(conn, f"Benchmarking scan_frame() x{n}...")
        t0 = time.ticks_ms()
        for _ in range(n):
            _adc_mgr.scan_frame()
        ms = time.ticks_diff(time.ticks_ms(), t0)
        fps    = n / (ms / 1000.0)
        ms_per = ms / n
        self._tx(conn, f"  {n} frames in {ms} ms")
        self._tx(conn, f"  {fps:.1f} FPS  ({ms_per:.1f} ms/frame)")
        self._end(conn)

    def _blink_led(self, conn, parts):
        """Blink the status LED N times so the user can confirm board identity."""
        n = 5
        try:
            if len(parts) > 1:
                n = max(1, min(20, int(parts[1])))
        except (ValueError, IndexError):
            pass
        if _status_led is None:
            self._tx(conn, "ERROR: LED pin not registered")
            self._end(conn)
            return
        self._tx(conn, f"Blinking LED {n}x...")
        for _ in range(n):
            _status_led.value(1)
            time.sleep_ms(120)
            _status_led.value(0)
            time.sleep_ms(120)
        _status_led.value(1)   # restore on
        self._tx(conn, "Done")
        self._end(conn)
