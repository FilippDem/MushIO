"""
MushIO V1.0 Real-Time Biopotential Monitor  (v4)

Views
-----
  Grid      : 10x10 physical layout (8x8 inner electrodes + outer STIM ring / AGND corners)
  Overlay   : 1/2/4 electrode panels + permanent STIM monitor panel (mV)
  Heatmap   : 10x10 spatial map (masked numpy array) + STIM bar chart
  STIM Ctrl : Per-channel waveform designer (IDAC current presets only)
  Settings  : Rolling binary data logging + webcam captures dir + JSON persistence

All views support Time / FFT toggle.
STIM channels display in mV (direct ADC voltage -- no sense resistor).
Data logging: rolling binary (.bin) files, configurable roll period.
Ctrl+D toggles demo mode.

Requirements
------------
    pip install matplotlib numpy
    pip install opencv-python   # optional -- webcam
    pip install scipy           # optional -- bandpass filter

Usage
-----
    python host/gui.py [--data-port 9000] [--cmd-host <ip>] [--cmd-port 9001] [--demo]
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
import threading
import socket
import queue
import time
import collections
import struct
import argparse
import sys
import os
import math
import colorsys
import random
import json
from datetime import datetime
from matplotlib.gridspec import GridSpec

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from PIL import Image as _PIL_probe; del _PIL_probe
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    from scipy.signal import butter, sosfilt, sosfiltfilt
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    import h5py
    HAS_H5PY = True
except ImportError:
    HAS_H5PY = False

sys.path.insert(0, os.path.dirname(__file__))
import receiver as rx


# =============================================================================
# Constants
# =============================================================================

BUFFER_SECS  = 20.0
DISPLAY_SECS = 5.0

# Datasheet-derived fallback FPS (used until real timestamps accumulate).
# ADS124S08 @ DR_4000 SPS, sinc3 filter, 6 ADCs on 1 MHz SPI, 12 ch/ADC:
#   Sinc3 settling per channel step  = 3 × (1/DR) = 3/4000 s = 750 µs
#   SPI overhead per step (6 ADCs)   = 6 × WREG(3B) + 6 × RDATA(4B)
#                                    = 42 B × 8 bits / 1 MHz SPI = 336 µs
#   Total per step ≈ 1086 µs  →  frame = 12 × 1086 µs ≈ 13 ms  →  ~77 FPS
_sinc3_us   = 3 * 1_000_000 // rx.ADC_DATA_RATE_SPS                      # 750 µs
_spi_us     = rx.NUM_ADCS * (3 + 4) * 8 * 1_000_000 // rx.SPI_CLOCK_HZ  # 336 µs
ASSUMED_FPS = 1_000_000 / ((_sinc3_us + _spi_us) * rx.CHANNELS_PER_ADC) # ~77 FPS

GRID_FPS     = 15
OVERLAY_FPS  = 25
HEATMAP_FPS  = 3

# ---- IDAC current presets (ADS124S08 datasheet, fixed steps only) ----------
IDAC_CURRENTS_UA = [10, 50, 100, 250, 500, 750, 1000, 1500, 2000]

# ---- physical electrode coordinate system -----------------------------------
# Inner sense electrodes:  col 0-7, row 0-7
# Outer ring:              col/row = -1 or 8
# STIM positions in outer ring (col, row)
STIM_PHYS = {
    'STIM_0': (2, -1),   # top side, clockwise start (left of pair)
    'STIM_1': (5, -1),   # top side (right of pair)
    'STIM_2': (8,  2),   # right side (top of pair)
    'STIM_3': (8,  5),   # right side (bottom of pair)
    'STIM_4': (5,  8),   # bottom side (right of pair, clockwise)
    'STIM_5': (2,  8),   # bottom side (left of pair)
    'STIM_6': (-1, 5),   # left side (bottom of pair, clockwise)
    'STIM_7': (-1, 2),   # left side (top of pair)
}
AGND_PHYS = {(-1, -1), (-1, 8), (8, -1), (8, 8)}   # permanent AGND corners

# Roll period options for data logging (label -> minutes)
LOG_ROLL_OPTIONS = {
    "15 min":  15,
    "30 min":  30,
    "1 hour":  60,
    "2 hours": 120,
    "4 hours": 240,
    "8 hours": 480,
}
LOG_ROLL_LABELS = list(LOG_ROLL_OPTIONS.keys())

SETTINGS_FILE = os.path.join(os.path.dirname(__file__), 'settings.json')

# ---- channel lists ----------------------------------------------------------
ELEC_NAMES = rx.ELEC_NAMES
STIM_NAMES = rx.STIM_NAMES
ELEC_IDX   = rx.RECORDING_CHANNEL_INDICES
STIM_IDX   = rx.STIM_CHANNEL_INDICES
ALL_NAMES  = rx.CHANNEL_NAMES

VREF = rx.VREF
GAIN = rx.AFE_GAIN
FS   = rx.ADC_FULL_SCALE

STIM_BY_NAME = sorted(zip(STIM_NAMES, STIM_IDX), key=lambda x: x[0])

# Reverse map: (col, row) -> (stim_name, flat_idx)
STIM_PHYS_TO_INFO = {}
for _sn, _si in STIM_BY_NAME:
    _pos = STIM_PHYS.get(_sn)
    if _pos is not None:
        STIM_PHYS_TO_INFO[_pos] = (_sn, _si)

# ---- spatial electrode grid -------------------------------------------------
ELEC_GRID = {}
GRID_FLAT = {}

for _flat in ELEC_IDX:
    _nm = ALL_NAMES[_flat]
    try:
        _c = int(_nm[4])
        _r = int(_nm[5])
        ELEC_GRID[(_c, _r)] = _flat
        GRID_FLAT[_flat]    = (_c, _r)
    except (IndexError, ValueError):
        pass

# ---- colour palette ---------------------------------------------------------
COLORS = [
    '#4e79a7', '#f28e2b', '#e15759', '#76b7b2', '#59a14f',
    '#edc948', '#b07aa1', '#ff9da7', '#9c755f', '#bab0ac',
    '#4dc36a', '#f1ce63', '#a0cbe8', '#ffbe7d', '#8cd17d',
    '#b6992d', '#499894', '#86bcb6', '#e49444', '#d37295',
]

BG_DARK   = '#1e1e2e'
BG_MID    = '#181825'
BG_LIGHT  = '#313244'
FG_MAIN   = '#cdd6f4'
FG_DIM    = '#a6adc8'
FG_DIMMER = '#6c7086'
ACCENT    = '#89b4fa'
GREEN     = '#a6e3a1'
RED       = '#f38ba8'
ORANGE    = '#fab387'
STIM_BG   = '#1a1020'   # darker purple tint for STIM panels

# ---- ADS124S08 register table -----------------------------------------------
# (addr, short_name, rw, description, power-on default, bit-field summary)
ADS124S08_REGS = [
    (0x00, 'ID',       'R',  'Device identification (read-only)',
     0x10, 'DEV_ID[4:0]'),
    (0x01, 'STATUS',   'R',  'Conversion status flags (read-only)',
     0x00, 'FL_POR | nRDY | FL_P_RAILP | FL_P_RAILN | FL_N_RAILP | FL_N_RAILN | FL_REF_L0 | FL_REF_L1'),
    (0x02, 'INPMUX',   'RW', 'Input multiplexer — MUXP selects positive input, MUXN negative',
     0x01, 'MUXP[7:4]  MUXN[3:0]  (0x0C = AINCOM)'),
    (0x03, 'PGA',      'RW', 'PGA settling delay, gain (1-32), PGA enable',
     0x00, 'DELAY[7:5]  GAIN[4:2]  PGA_EN[1]  (GAIN: 000=1 001=2 010=4 011=8 100=16 101=32)'),
    (0x04, 'DATARATE', 'RW', 'Global chop, clock source, conversion mode, digital filter, data rate',
     0x14, 'G_CHOP[7] CLK[6] MODE[5] FILTER[4:3] DR[3:0]  (DR 1111=4kSPS)'),
    (0x05, 'REF',      'RW', 'Reference inputs, buffering, source selection',
     0x10, 'FL_REF_EN[7:6] nREFP_BUF[5] nREFN_BUF[4] REFSEL[3:2] REFCON[1:0]'),
    (0x06, 'IDACMAG',  'RW', 'IDAC rail-detect flag enable, power switch, current magnitude',
     0x00, 'FL_RAIL_EN[7] PSW[6] IMAG[3:0]  (0=off 1=10 2=50 3=100 4=250 5=500 6=750 7=1000 8=1500 9=2000 uA)'),
    (0x07, 'IDACMUX',  'RW', 'IDAC1 and IDAC2 output pin routing (0xF = disabled)',
     0xFF, 'I1MUX[7:4]  I2MUX[3:0]  (0x0-0xB = AIN0-AIN11, 0xF = off)'),
    (0x08, 'VBIAS',    'RW', 'Mid-supply voltage bias enable per AIN pin',
     0x00, 'VB_LEVEL[7] VB_AINC[6] VB_AIN5[5] VB_AIN4[4] VB_AIN3[3] VB_AIN2[2] VB_AIN1[1] VB_AIN0[0]'),
    (0x09, 'SYS',      'RW', 'System monitor mux, calibration samples, timeout, CRC, SENDSTAT',
     0x10, 'SYS_MON[7:5] CAL_SAMP[4:3] TIMEOUT[2] CRC_EN[1] SENDSTAT[0]'),
    (0x0A, 'OFCAL0',   'RW', 'Offset calibration coefficient byte 0 (LSB)',
     0x00, 'OFC[7:0]'),
    (0x0B, 'OFCAL1',   'RW', 'Offset calibration coefficient byte 1',
     0x00, 'OFC[7:0]'),
    (0x0C, 'OFCAL2',   'RW', 'Offset calibration coefficient byte 2 (MSB)',
     0x00, 'OFC[7:0]'),
    (0x0D, 'FSCAL0',   'RW', 'Full-scale calibration coefficient byte 0 (LSB)',
     0x00, 'FSC[7:0]'),
    (0x0E, 'FSCAL1',   'RW', 'Full-scale calibration coefficient byte 1',
     0x00, 'FSC[7:0]'),
    (0x0F, 'FSCAL2',   'RW', 'Full-scale calibration coefficient byte 2 (MSB)',
     0x40, 'FSC[7:0]  (power-on = 0x400000 = unity gain)'),
    (0x10, 'GPIODAT',  'RW', 'GPIO pin direction (1=input) and data',
     0x00, 'DIR[7:4]  DAT[3:0]'),
    (0x11, 'GPIOCON',  'RW', 'GPIO pin function: 1=GPIO, 0=analog',
     0x00, 'CON[3:0]'),
]

NUM_ADCS = rx.NUM_ADCS   # 6

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'experiment_config.json')

# ---- Application icon (64x64 mushroom, PNG base64-encoded) ------------------
# Generated programmatically via Pillow — no external file needed.
_ICON_B64 = (
    "iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAACIUlEQVR4nO2Zr1LEMBDGNztR"
    "KASGE2gUAsVjFIsBgeIhKvoQKAQYLH0MFAKFRhwGgcIeE9GZo9Ok2WQ3TSf5zdzczaXd7Pfl"
    "3/YOoFKpVCqVSqmoJTr92b5uh8+Hm4sNlGDA4+35zrw33f3XuK1v746n7rl5eBPPT0kL3mdK"
    "/JwJ0oZoadFS8bnM0DkK358ptpkx9BlrhEot3LUHxCyRUCO05PqeStp85zPCoTlRjVChHXFu"
    "bnMxqLEoJiAIivdpl4CyNDEmqK+4nE1AzmAxzE3vkL3CJ2/kCJIzc/nrmJu5mToOOU4Jo8O2"
    "MWrIEK6j0QeMGX3fRFMKourBHDevlKgUJa4EofvEeC9Qa9v5uR+pEQoHQ0dhePGn5O43pp3F"
    "gGbUyRJlLidIudgmds0mIKwIiSMXoXCUrcF2FKY+7znrgKnnAWW72FULSPyklQKSAbkXRFRs"
    "T4MIhYOuxhR/TaXApQNjbl4Dc/kjR5Bc8ckbOYPlhG++KBF0acT+GDF8fv9CzlDzU5SLu+b0"
    "X11wcnQAuQpv+w/FakA3Ep+LEa4R9zFBcyaR0giupagoF7tmwRgJMyii2ZdAiAmxhoSOsq94"
    "Q9CxFmpCCijiDVHnek5GUIUPKM4kUhoSKniMAkHeX7pd//QcHae5voKzy1YkVw3CmORzBqFw"
    "EAoHoXAQCgehcBAKB6FwUDI4V/EiVQQlmQGxyUuKT7YEQkVIi69ABf4ADAYbM0WuEp4AAAAA"
    "SUVORK5CYII="
)

# =============================================================================
# Signal processing helpers
# =============================================================================

def bandpass_filter(data, low_hz, high_hz, fs, order=4):
    # sosfiltfilt requires padlen = 3 * (2 * n_sos) samples.
    # butter bandpass of order N → N SOS sections → padlen = 6*N.
    if not HAS_SCIPY or len(data) < max(order * 6 + 2, 30):
        return data
    nyq = fs / 2.0
    lo  = max(1e-3, low_hz)  / nyq
    hi  = min(0.95,  high_hz / nyq)   # cap at 0.95×Nyquist; 0.999 causes instability
    if lo >= hi or hi >= 1.0:
        return data
    try:
        sos = butter(order, [lo, hi], btype='band', output='sos')
        # sosfiltfilt: zero-phase (forward+backward) — no startup transient,
        # no phase distortion.  Replaces causal sosfilt which produced
        # visible ringing at the start of every display window.
        return sosfiltfilt(sos, data)
    except Exception:
        return data


def compute_fft(data, fs):
    n = len(data)
    if n < 8:
        return np.array([0.0, 0.5]), np.array([0.0, 0.0])
    win  = np.hanning(n)
    mag  = np.abs(np.fft.rfft((data - data.mean()) * win)) * 2.0 / win.sum()
    freq = np.fft.rfftfreq(n, 1.0 / fs)
    return freq, mag


def display_decimate(arr, max_pts=600):
    """Block-average arr to at most max_pts samples for visual display.

    Block averaging is a boxcar lowpass at ~FPS/(2*block) Hz followed by
    downsampling, providing anti-aliasing without distorting sub-Nyquist
    content.  Eliminates visual moiré/aliasing artefacts when sample
    density greatly exceeds screen pixel density (e.g. 2500 samples in a
    150-pixel-wide cell).  Returns the decimated array; caller should
    regenerate t_vec with np.linspace to match the new length.
    """
    n = len(arr)
    if n <= max_pts:
        return arr
    block = max(1, n // max_pts)
    n_out = n // block
    return arr[:n_out * block].reshape(n_out, block).mean(axis=1)


# =============================================================================
# Data Receiver Thread
# =============================================================================

class DataReceiver(threading.Thread):
    def __init__(self, port, buf_size):
        super().__init__(daemon=True)
        self.port      = port
        self.buf_size  = buf_size
        self.buffers   = [collections.deque(maxlen=buf_size)
                          for _ in range(rx.TOTAL_CHANNELS)]
        self.timestamps     = collections.deque(maxlen=buf_size)
        self._paused        = False   # True while SW demo injector is active
        self.frames_total   = 0
        self.frames_session = 0
        self.crc_errors     = 0
        self.missed_frames  = 0   # cumulative across all connections this session
        self.missed_session = 0   # per-connection (reset on each new TCP connect)
        self.fps            = 0.0
        self.connected      = False
        self.client_ip      = ""
        self.bytes_session  = 0   # raw bytes received in current TCP session
        self._last_seq      = None
        self._fps_count     = 0
        self._fps_time      = time.time()
        self._stop_event    = threading.Event()
        # --- binary rolling log ---
        self._log_lock      = threading.Lock()
        self._log_file      = None
        self.log_active     = False
        self.log_frames     = 0
        self._log_dir       = 'data'
        self._log_prefix    = 'mushio'
        self._log_roll_mins = 60
        self._log_next_roll = None
        self.log_curr_path  = ''
        # --- HDF5 log (optional; requires h5py) ---
        self._hdf5_file     = None
        self._hdf5_ds       = None   # dataset for raw int32 samples (N, 72)
        self._hdf5_ts_ds    = None   # dataset for timestamps_us (N,)
        self.hdf5_active    = False
        self.hdf5_frames    = 0

    def run(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.port))
        srv.listen(1)
        srv.settimeout(1.0)
        while not self._stop_event.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            self.connected      = True
            self.client_ip      = addr[0]
            self.frames_session = 0
            self.bytes_session  = 0
            self.missed_session = 0
            self._last_seq      = None
            self._process(conn)
            self.connected = False
        srv.close()

    def _process(self, conn):
        # TCP_NODELAY on the host side prompts the OS to ACK incoming data
        # quickly rather than batching ACKs (Windows delayed-ACK = 200 ms).
        # Together with TCP_NODELAY on the Pico this lifts FPS from ~10 to
        # the full ~100-200 FPS the hardware can sustain.
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        conn.settimeout(30.0)
        buf = bytearray()
        while not self._stop_event.is_set():
            try:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                self.bytes_session += len(chunk)
                buf.extend(chunk)
                while len(buf) >= rx.FRAME_SIZE:
                    pos = rx.MushIOReceiver._find_sync(buf)
                    if pos < 0:
                        buf = buf[-1:]; break
                    if pos > 0:
                        buf = buf[pos:]
                    if len(buf) < rx.FRAME_SIZE:
                        break
                    frame_bytes = bytes(buf[:rx.FRAME_SIZE])
                    buf = buf[rx.FRAME_SIZE:]
                    self._ingest(frame_bytes)
            except socket.timeout:
                break
            except Exception:
                break
        conn.close()

    def _ingest(self, frame_bytes):
        if self._paused:
            return
        parsed = rx.parse_frame(frame_bytes)
        if parsed is None:
            return
        self.frames_total   += 1
        self.frames_session += 1
        self._fps_count     += 1
        if not parsed['crc_valid']:
            self.crc_errors += 1
            return
        seq = parsed['seq']
        if self._last_seq is not None:
            expected = (self._last_seq + 1) & 0xFFFF
            if seq != expected:
                gap = (seq - expected) & 0xFFFF
                self.missed_frames += gap
                self.missed_session += gap
                # Only flush on large gaps (≥ 20 frames = 40 ms at 500 FPS).
                # At 12 Hz (highest demo frequency) a gap of 21 frames causes
                # a 180° phase shift — visible signal inversion.  Single-frame
                # WiFi drops cause ≤ 8.6° error at 12 Hz — completely invisible.
                # Flushing on every 1-frame drop keeps the buffer too short for
                # a useful FFT (→ broad triangular peaks, half-filled time view).
                if gap >= 20:
                    for buf in self.buffers:
                        buf.clear()
                    self.timestamps.clear()
        self._last_seq = seq
        for i, s in enumerate(parsed['samples']):
            self.buffers[i].append(s)
        self.timestamps.append(parsed['timestamp_us'])
        now = time.time()
        dt  = now - self._fps_time
        if dt >= 0.5:
            self.fps        = self._fps_count / dt
            self._fps_count = 0
            self._fps_time  = now
        # Binary log (valid frames only)
        if self.log_active:
            with self._log_lock:
                if self._log_next_roll and now >= self._log_next_roll:
                    self._open_log_file()
                if self._log_file:
                    self._log_file.write(frame_bytes)
                    self.log_frames += 1
        # HDF5 log (valid frames only)
        if self.hdf5_active and self._hdf5_file is not None:
            with self._log_lock:
                row = np.array(parsed['samples'], dtype=np.int32)
                ts  = parsed['timestamp_us']
                n   = self.hdf5_frames
                self._hdf5_ds.resize(n + 1, axis=0)
                self._hdf5_ts_ds.resize(n + 1, axis=0)
                self._hdf5_ds[n]    = row
                self._hdf5_ts_ds[n] = ts
                self.hdf5_frames += 1

    # --- binary rolling log API ---

    def start_logging(self, data_dir, prefix, roll_mins):
        with self._log_lock:
            self._log_dir       = data_dir
            self._log_prefix    = prefix
            self._log_roll_mins = max(1, roll_mins)
            self.log_frames     = 0
            self.log_active     = True
            self._open_log_file()

    def _open_log_file(self):
        """Open a new .bin file (must be called with _log_lock held)."""
        if self._log_file:
            try: self._log_file.close()
            except Exception: pass
        os.makedirs(self._log_dir, exist_ok=True)
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self._log_dir, f"{self._log_prefix}_{ts}.bin")
        self._log_file      = open(path, 'wb')
        self.log_curr_path  = path
        self._log_next_roll = time.time() + self._log_roll_mins * 60

    def stop_logging(self):
        with self._log_lock:
            self.log_active = False
            if self._log_file:
                try: self._log_file.close()
                except Exception: pass
                self._log_file = None

    # --- HDF5 log API -------------------------------------------------------

    def start_hdf5_logging(self, data_dir, prefix):
        """Open an HDF5 file and start appending frames.  Requires h5py."""
        import h5py  # guarded at call site by HAS_H5PY check
        with self._log_lock:
            os.makedirs(data_dir, exist_ok=True)
            ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(data_dir, f"{prefix}_{ts}.h5")
            f    = h5py.File(path, 'w')
            # Unlimited chunked dataset: rows = frames, cols = 72 channels
            self._hdf5_ds    = f.create_dataset(
                'samples',
                shape=(0, rx.TOTAL_CHANNELS),
                maxshape=(None, rx.TOTAL_CHANNELS),
                dtype=np.int32,
                chunks=(256, rx.TOTAL_CHANNELS),
                compression='gzip', compression_opts=1,
            )
            self._hdf5_ts_ds = f.create_dataset(
                'timestamp_us',
                shape=(0,), maxshape=(None,),
                dtype=np.uint32,
                chunks=(256,),
            )
            # Store channel names as attribute
            self._hdf5_ds.attrs['channel_names'] = rx.CHANNEL_NAMES
            self._hdf5_file  = f
            self.hdf5_active = True
            self.hdf5_frames = 0

    def stop_hdf5_logging(self):
        with self._log_lock:
            self.hdf5_active = False
            if self._hdf5_file is not None:
                try: self._hdf5_file.close()
                except Exception: pass
                self._hdf5_file  = None
                self._hdf5_ds    = None
                self._hdf5_ts_ds = None

    def stop(self):
        self._stop_event.set()

    def get_channel(self, flat_idx, n_samples):
        """Return up to n_samples most-recent values.

        Never zero-pads.  During the initial buffer fill-up (or after a
        gap-triggered flush) the returned list is shorter than n_samples;
        callers use the actual length to compute the correct time axis so the
        display grows in from the right with no false signal discontinuities.
        """
        data = list(self.buffers[flat_idx])
        return data[-n_samples:]


# =============================================================================
# Demo Data Injector  (column-based waveform types)
# =============================================================================

class DemoDataInjector(threading.Thread):
    """
    Injects synthetic biopotential data into DataReceiver buffers.
    Each electrode column (0-7) has a distinct signal type so all GUI
    features can be exercised without hardware.

    Column types
    ------------
    Col 0 : Gaussian white noise           (resting-state baseline)
    Col 1 : Triangle wave 1.5 Hz           (slow LFP — visually distinct from sine)
    Col 2 : Action-potential spike trains  (Poisson, ~5 Hz)
    Col 3 : Pure DC offsets per row        (demonstrates baseline/gain/saturation)
    Col 4 : Chirp 2→20 Hz over 4 s         (frequency sweep)
    Col 5 : 20 Hz burst firing             (periodic 0.5 s bursts / 2 s)
    Col 6 : Brownian random walk           (1/f-like noise)
    Col 7 : Sawtooth 6 Hz + noise          (asymmetric ramp)
    """

    FPS = 200.0

    # Action-potential template (15 samples @ 200 Hz = 75 ms)
    _AP = [0.00, 0.35, 0.75, 1.00, 0.70, 0.20,
           -0.45, -0.60, -0.45, -0.25, -0.10, -0.03, 0.00, 0.00, 0.00]

    def __init__(self, receiver):
        super().__init__(daemon=True)
        self._rx   = receiver
        self._stop = threading.Event()
        # Stateful waveform state per channel
        self._rw              = [0.0] * rx.TOTAL_CHANNELS  # random walk
        self._spike_countdown = [0] * rx.TOTAL_CHANNELS   # 0 = no active spike

    def run(self):
        frame_period = 1.0 / self.FPS
        frame_idx    = 0
        start        = time.time()

        self._rx.connected      = True
        self._rx.client_ip      = "demo"
        self._rx.frames_session = 0
        self._rx._last_seq      = None
        self._rx._fps_time      = time.time()
        self._rx._fps_count     = 0

        while not self._stop.is_set():
            target = start + frame_idx * frame_period
            now    = time.time()
            if target > now:
                time.sleep(target - now)

            t       = frame_idx / self.FPS
            samples = self._generate(t, frame_idx)
            for i, s in enumerate(samples):
                self._rx.buffers[i].append(s)
            self._rx.timestamps.append(int(time.monotonic() * 1_000_000))

            self._rx.frames_total   += 1
            self._rx.frames_session += 1
            self._rx._fps_count     += 1

            now2 = time.time()
            dt   = now2 - self._rx._fps_time
            if dt >= 0.5:
                self._rx.fps        = self._rx._fps_count / dt
                self._rx._fps_count = 0
                self._rx._fps_time  = now2

            frame_idx += 1

        self._rx.connected = False

    def _generate(self, t, frame_idx):
        samples = []
        for ch in range(rx.TOTAL_CHANNELS):
            if ch in GRID_FLAT:
                col, row = GRID_FLAT[ch]
                s = self._column_wave(col, row, ch, t, frame_idx)
            elif ch in rx.STIM_CHANNEL_INDICES:
                stim_i = rx.STIM_CHANNEL_INDICES.index(ch)
                s = int(800 * math.sin(2 * math.pi * (0.5 + stim_i * 0.15) * t
                                       + stim_i * 0.8))
            else:
                s = random.randint(-50, 50)
            samples.append(max(-8388607, min(8388607, int(s))))
        return samples

    def _column_wave(self, col, row, ch, t, frame_idx):
        amp   = 200 + row * 70          # row scales amplitude
        phase = row * math.pi / 4.0

        if col == 0:
            # Gaussian white noise
            return int(random.gauss(0, amp * 0.9))

        elif col == 1:
            # Triangle wave 1.5 Hz — slow LFP, visually distinct from sine
            # asin(sin(x))/pi*2 gives a triangle with range [-1, 1]
            tri = 2.0 / math.pi * math.asin(
                math.sin(2 * math.pi * 1.5 * t + phase))
            return int(amp * 3.5 * tri)

        elif col == 2:
            # Poisson action-potential spike train (~5 Hz)
            return self._spike(ch, amp)

        elif col == 3:
            # Pure DC offsets — each row holds a different constant level
            # (demonstrates baseline offsets, DC coupling, and gain saturation)
            dc_levels = [-8388600, -8200000, -3000000, -800, 800, 3000000, 8200000, 8388600]
            dc = dc_levels[row] if row < len(dc_levels) else 0
            return int(dc + random.gauss(0, 12))

        elif col == 4:
            # Chirp: frequency sweeps 2→20 Hz over a 4 s period + light noise
            chirp_period = 4.0
            f_lo, f_hi   = 2.0, 20.0
            frac         = math.fmod(t + row * chirp_period / 8.0, chirp_period) / chirp_period
            f_inst       = f_lo + (f_hi - f_lo) * frac
            chirp        = amp * 2.2 * math.sin(2 * math.pi * f_inst * t + phase)
            return int(chirp + random.gauss(0, amp * 0.2))

        elif col == 5:
            # Burst activity: 20 Hz oscillation in 0.5 s bursts every 2 s
            burst_period = 2.0
            burst_dur    = 0.5
            cycle_t      = math.fmod(t + row * burst_period / 8.0, burst_period)
            if cycle_t < burst_dur:
                osc = amp * 3.0 * math.sin(2 * math.pi * 20.0 * t + phase)
                return int(osc + random.gauss(0, amp * 0.2))
            return int(random.gauss(0, amp * 0.18))

        elif col == 6:
            # Brownian random walk (1/f noise)
            step         = random.gauss(0, 35)
            self._rw[ch] = max(-4500.0, min(4500.0, self._rw[ch] * 0.998 + step))
            return int(self._rw[ch])

        elif col == 7:
            # Sawtooth 6 Hz + light noise — asymmetric ramp, visually unlike sine
            # math.fmod gives [0,1) ramp; shift to [-1, 1)
            saw = 2.0 * math.fmod(6.0 * t + phase / (2 * math.pi), 1.0) - 1.0
            return int(amp * 2.2 * saw + random.gauss(0, amp * 0.25))

        return 0

    def _spike(self, ch, amp):
        """Return one sample of a Poisson spike train with realistic AP shape."""
        REFRAC = 40  # samples (200 ms)
        cd = self._spike_countdown[ch]
        if cd > 0:
            # Counting down through the AP waveform or refractory silence
            ap_idx = REFRAC - cd
            if 0 <= ap_idx < len(self._AP):          # guard negative indices too
                val = int(amp * 9 * self._AP[ap_idx])
            else:
                val = int(random.gauss(0, amp * 0.35))
            self._spike_countdown[ch] -= 1
            return val
        # Decide whether to fire (Poisson ~5 Hz at 200 FPS)
        if random.random() < 5.0 / self.FPS:
            self._spike_countdown[ch] = REFRAC
            return int(amp * 9 * self._AP[0])
        return int(random.gauss(0, amp * 0.35))

    def stop(self):
        self._stop.set()


# =============================================================================
# Command Client Thread
# =============================================================================

class CommandClient(threading.Thread):
    def __init__(self, host, port, rx_queue):
        super().__init__(daemon=True)
        self.host          = host
        self.port          = port
        self.rx_q          = rx_queue
        self.tx_q          = queue.Queue()
        self.connected     = False
        self._stop         = threading.Event()
        self._disconnect_ev = threading.Event()

    def send(self, cmd):
        self.tx_q.put(cmd.strip() + '\n')

    def run(self):
        while not self._stop.is_set():
            if not self.host:
                time.sleep(0.5); continue
            self._disconnect_ev.clear()
            try:
                self._connect_and_run()
            except Exception as e:
                self.rx_q.put(f"[CMD] Disconnected: {e}")
                self.connected = False
                if not self._disconnect_ev.is_set():
                    time.sleep(2)

    def _connect_and_run(self):
        self.rx_q.put(f"[CMD] Connecting to {self.host}:{self.port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((self.host, self.port))
        sock.settimeout(None)
        self.connected = True
        self.rx_q.put(f"[CMD] Connected to {self.host}:{self.port}")
        recv_buf = b''
        while not self._stop.is_set() and not self._disconnect_ev.is_set():
            while not self.tx_q.empty():
                sock.sendall(self.tx_q.get_nowait().encode())
            sock.settimeout(0.05)
            try:
                chunk = sock.recv(1024)
                if not chunk:
                    raise ConnectionResetError("closed")
                recv_buf += chunk
                while b'\n' in recv_buf:
                    line, recv_buf = recv_buf.split(b'\n', 1)
                    self.rx_q.put(line.decode('utf-8', 'ignore').rstrip('\r'))
            except socket.timeout:
                pass
        sock.close()
        self.connected = False
        if self._disconnect_ev.is_set():
            self.rx_q.put("[CMD] Disconnected by user.")

    def set_host(self, host, port):
        self.host = host
        self.port = port
        while not self.tx_q.empty():
            try: self.tx_q.get_nowait()
            except Exception: pass

    def disconnect(self):
        """Drop the active TCP connection without stopping the background thread."""
        self.host = ''
        self._disconnect_ev.set()

    def stop(self):
        self._stop.set()


# =============================================================================
# Demo Command Client
# =============================================================================

class DemoCmdClient:
    def __init__(self, rx_queue):
        self.rx_q      = rx_queue
        self.tx_q      = queue.Queue()
        self.connected = True
        self._stop     = threading.Event()
        self._thread   = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send(self, cmd):
        self.tx_q.put(cmd.strip())

    def set_host(self, host, port):
        pass

    _BANNER = (
        "=" * 60 + "\n"
        " MushIO V1.0  |  Real-Time Biopotential Monitor\n"
        "=" * 60
    )

    def _run(self):
        for line in self._BANNER.splitlines():
            self.rx_q.put(line)
        self.rx_q.put("[CMD] Demo mode — command responses are simulated")
        while not self._stop.is_set():
            try:
                cmd = self.tx_q.get(timeout=0.1)
                self._handle(cmd)
            except queue.Empty:
                pass

    def _handle(self, cmd):
        parts = cmd.strip().split()
        op    = parts[0].lower() if parts else ''

        if op == 'ping':
            self.rx_q.put("PONG")
        elif op == 'status':
            for line in ["frames=12345","fps=198.3","crc_err=0",
                         "missed=0","wifi=yes","tcp=yes"]:
                self.rx_q.put(line)
        elif op == 'read_regs':
            import random as _r
            self.rx_q.put("[REGS] Reading all 6 ADS124S08 register banks (simulated)...")
            # Simulated defaults with slight per-ADC variation
            _defaults = {0x00:0x10,0x01:0x00,0x02:0x01,0x03:0x00,
                         0x04:0x14,0x05:0x10,0x06:0x00,0x07:0xFF,
                         0x08:0x00,0x09:0x10,0x0A:0x00,0x0B:0x00,
                         0x0C:0x00,0x0D:0x00,0x0E:0x00,0x0F:0x40,
                         0x10:0x00,0x11:0x00}
            for i in range(6):
                pairs = ' '.join(f'0x{addr:02X}=0x{val:02X}'
                                 for addr, val in _defaults.items())
                self.rx_q.put(f"ADC{i} {pairs}")
            self.rx_q.put("Register dump complete -- all 6 ADCs responding")
            self.rx_q.put('END')
        elif op == 'scan_all':
            self.rx_q.put("Scanning all 72 channels...")
            for i in range(rx.TOTAL_CHANNELS):
                raw  = random.randint(-500, 500)
                uv   = raw / rx.ADC_FULL_SCALE * rx.VREF / rx.AFE_GAIN * 1e6
                name = rx.CHANNEL_NAMES[i]
                tag  = " [STIM]" if i in rx.STIM_CHANNEL_INDICES else ""
                self.rx_q.put(f"  [{i:2d}] {name:>8}{tag}  raw={raw:9d}  {uv:+10.2f} uV")
        elif op == 'benchmark':
            self.rx_q.put("196.1 FPS  (5.1 ms/frame)")
        elif op == 'blink_led':
            n = int(parts[1]) if len(parts) > 1 else 5
            self.rx_q.put(f"[LED] Blinking status LED {n}x  (simulated in demo)")
        elif op in ('stim_sine', 'stim_square', 'stim_triangle'):
            ch_i = int(parts[1]) if len(parts) > 1 else 0
            self.rx_q.put(f"[STIM] Loading {op[5:]} waveform on STIM_{ch_i}...")
            self.rx_q.put(f"[STIM] Waveform stored ({len(parts)} params)")
            self.rx_q.put("OK")
        elif op == 'stim_csv':
            ch_i = int(parts[1]) if len(parts) > 1 else 0
            n    = int(parts[3]) if len(parts) > 3 else 0
            self.rx_q.put(f"[STIM] Received {n}-sample CSV on STIM_{ch_i}")
            self.rx_q.put("OK")
        elif op == 'stim_start':
            ch_i = int(parts[1]) if len(parts) > 1 else 0
            self.rx_q.put(f"[STIM] Stimulation started on STIM_{ch_i}")
            self.rx_q.put("OK")
        elif op == 'stim_stop':
            ch_i = int(parts[1]) if len(parts) > 1 else 0
            self.rx_q.put(f"[STIM] Stimulation stopped on STIM_{ch_i}")
            self.rx_q.put("OK")
        elif op in ('stim_dc',):
            ch_i = int(parts[1]) if len(parts) > 1 else 0
            amp  = parts[2] if len(parts) > 2 else '?'
            self.rx_q.put(f"[STIM] DC {amp} uA on STIM_{ch_i}")
            self.rx_q.put("OK")
        elif op == 'stim_pulse':
            ch_i = int(parts[1]) if len(parts) > 1 else 0
            self.rx_q.put(f"[STIM] Pulse train on STIM_{ch_i} ({len(parts)} params)")
            self.rx_q.put("OK")
        elif op == 'selfocal':
            self.rx_q.put("[CAL] Offset calibration running on all 6 ADCs  (simulated)...")
            self.rx_q.put("[CAL] OFCAL0-2 registers updated — offset nulled to <1 LSB")
            self.rx_q.put("OK")
        elif op == 'selfgcal':
            self.rx_q.put("[CAL] Full-scale gain calibration running on all 6 ADCs  (simulated)...")
            self.rx_q.put("[CAL] FSCAL0-2 registers updated — gain error <±1 ppm")
            self.rx_q.put("OK")
        elif op == 'help':
            for line in ["ping","status","read_regs","scan_all","benchmark",
                         "blink_led [count]",
                         "stim_dc <ch> <uA>",
                         "stim_square <ch> <uA> <hz> <duty%> <dur>",
                         "stim_pulse <ch> <uA> <pw_ms> <ips_ms> <count>",
                         "stim_csv <ch> <n> <v0> <v1> ...",
                         "stim_start <ch>","stim_stop <ch>","help"]:
                self.rx_q.put(line)
        else:
            self.rx_q.put(f"[DEMO] '{op}' acknowledged")
        self.rx_q.put("END")

    def stop(self):
        self._stop.set()
        self.connected = False


# =============================================================================
# Webcam Capture
# =============================================================================

class WebcamCapture(threading.Thread):
    def __init__(self, output_dir="captures", interval_s=600, cam_index=0):
        super().__init__(daemon=True)
        self.output_dir        = output_dir
        self.interval_s        = interval_s
        self.cam_index         = cam_index
        self._stop             = threading.Event()
        self._enabled          = threading.Event()
        self._now_flag         = threading.Event()
        self.last_capture_time = None
        self.last_capture_path = None
        self.capture_count     = 0
        self.last_error        = None
        os.makedirs(output_dir, exist_ok=True)

    def enable(self):    self._enabled.set()
    def disable(self):   self._enabled.clear()
    def capture_now(self): self._now_flag.set()

    def run(self):
        if not HAS_CV2:
            return
        last_time = 0.0
        while not self._stop.wait(2.0):
            now   = time.time()
            force = self._now_flag.is_set()
            if self._enabled.is_set() and (force or now - last_time >= self.interval_s):
                self._now_flag.clear()
                self._do_capture()
                last_time = time.time()

    def _do_capture(self):
        try:
            cap = cv2.VideoCapture(self.cam_index)
            if not cap.isOpened():
                self.last_error = f"Cannot open camera {self.cam_index}"; return
            ret, frame = cap.read()
            cap.release()
            if ret:
                ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
                path = os.path.join(self.output_dir, f"mushio_capture_{ts}.jpg")
                cv2.imwrite(path, frame)
                self.last_capture_time = datetime.now()
                self.last_capture_path = path
                self.capture_count    += 1
                self.last_error        = None
            else:
                self.last_error = "Frame read failed"
        except Exception as e:
            self.last_error = str(e)

    def stop(self):
        self._stop.set()


# =============================================================================
# Main GUI
# =============================================================================

class MushIOGUI:

    TITLE = "MushIO V1.0  Real-Time Biopotential Monitor"

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(self, root, data_port, cmd_host, cmd_port):
        self.root      = root
        self.data_port = data_port
        self.cmd_host  = cmd_host
        self.cmd_port  = cmd_port

        root.title(self.TITLE)
        root.geometry("1440x900")
        root.configure(bg=BG_DARK)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.bind('<Control-d>', lambda e: self._toggle_demo())
        # Set window icon (mushroom PNG, embedded as base64 — no file needed)
        try:
            import base64 as _b64
            _icon_data = _b64.b64decode(_ICON_B64)
            if HAS_CV2:
                # PIL path: richer compatibility
                from PIL import Image as _PILImg, ImageTk as _PILTk
                import io as _io
                _pil_img = _PILImg.open(_io.BytesIO(_icon_data)).convert("RGBA")
                self._icon_photo = _PILTk.PhotoImage(_pil_img)
            else:
                # Tk 8.6+ native PNG support via data= parameter
                self._icon_photo = tk.PhotoImage(data=_ICON_B64)
            root.iconphoto(True, self._icon_photo)
        except Exception:
            pass   # icon is cosmetic; never crash over it

        # Fix Combobox/Listbox colours for all platforms
        root.option_add('*TCombobox*Listbox.foreground',       FG_MAIN)
        root.option_add('*TCombobox*Listbox.background',       BG_LIGHT)
        root.option_add('*TCombobox*Listbox.selectForeground', BG_DARK)
        root.option_add('*TCombobox*Listbox.selectBackground', ACCENT)

        self._cmd_rx_q = queue.Queue()

        # Demo / webcam state
        self._demo_mode     = False
        self._demo_injector = None
        self._webcam        = None

        # ---- display state -------------------------------------------------
        self._display_secs   = tk.DoubleVar(value=DISPLAY_SECS)
        self._uv_scale       = tk.DoubleVar(value=500.0)
        self._auto_scale     = tk.BooleanVar(value=True)
        self._fft_mode       = tk.BooleanVar(value=False)
        self._overlay_panels = tk.IntVar(value=1)
        self._waterfall_var  = tk.DoubleVar(value=500.0)  # µV stacking offset
        self._wf_panels      = tk.IntVar(value=1)
        # Waterfall figure/axes — populated in _build_waterfall_tab
        self._wf_fig         = None
        self._wf_canvas      = None
        self._wf_axes        = []
        self._wf_stim_ax     = None
        self._wf_fft_btn     = None
        # PGA gain (ADS124S08 PGA: 1,2,4,8,16,32).  Total gain = PGA x AFE_GAIN(11)
        self._pga_gain       = tk.IntVar(value=1)
        # ADS124S08 digital filter (DATARATE reg bits 4:2)
        self._adc_filter_var = tk.StringVar(value='SINC3')
        # ADS124S08 data rate (DATARATE reg bits 2:0)
        self._adc_dr_var     = tk.StringVar(value='4000')
        # ADS124S08 reference buffer (REF reg bits 5:4)
        self._adc_refbuf_var = tk.BooleanVar(value=False)
        # ADS124S08 system monitor (SYS reg bits 7:5)
        self._adc_sysmon_var = tk.StringVar(value='Off')

        # ---- bandpass state ------------------------------------------------
        self._bp_enabled   = tk.BooleanVar(value=False)
        self._bp_low       = tk.DoubleVar(value=1.0)
        self._bp_high      = tk.DoubleVar(value=300.0)
        self._bp_order_var = tk.IntVar(value=4)

        # ---- self-test / stream control ------------------------------------
        self._stream_paused_var = tk.BooleanVar(value=False)

        # ---- FFT zoom ------------------------------------------------------
        self._fft_max_hz   = tk.DoubleVar(value=0.0)  # 0 = auto (Nyquist)

        # ---- electrode / STIM selection ------------------------------------
        self._selected_elec  = set()
        self._stim_active    = set()

        # ---- zoom popup ----------------------------------------------------
        self._zoom_win       = None   # dict with win/fig/ax/line/canvas or None
        self._elec_btns      = {}
        self._stim_btns      = {}   # flat_idx -> Button (outer-ring in 10x10 grid)
        self._stim_phys_btns = {}   # same: populated in electrode grid section

        # ---- matplotlib state ----------------------------------------------
        self._grid_fig       = None
        self._grid_canvas    = None
        self._grid_lines     = {}
        self._grid_axes      = {}
        self._grid_texts     = {}
        self._stim_grid_lines = {}
        self._stim_grid_axes  = {}

        self._overlay_fig     = None
        self._overlay_canvas  = None
        self._overlay_axes    = []
        self._stim_overlay_ax = None

        self._heatmap_fig      = None
        self._heatmap_ax       = None
        self._heatmap_im       = None
        self._heatmap_canvas   = None
        self._heatmap_cbar     = None
        self._heatmap_stim_ax  = None
        self._heatmap_stim_bars = None
        self._heatmap_metric   = tk.StringVar(value="RMS")
        self._cmap_var         = tk.StringVar(value="plasma")
        self._regs_font_mode   = tk.StringVar(value='compact')  # 'compact' | 'large'

        # ---- STIM waveform designer (per-channel) --------------------------
        # Per-channel config dict, keyed by flat_idx
        self._stim_ch_configs = {
            fi: {'type': 'Off', 'amp': '100', 'freq': '2.0',
                 'duty': '50.0', 'duration': '1.0',
                 'pw_ms': '10.0', 'ips_ms': '90.0', 'count': '10',
                 'continuous': False,
                 'csv_data': None, 'csv_path': '(none)'}
            for _, fi in STIM_BY_NAME
        }
        # Editor vars (reflect the channel currently being edited)
        self._stim_edit_ch_var  = tk.StringVar(value=STIM_BY_NAME[0][0])
        self._stim_edit_type    = tk.StringVar(value='Off')
        self._stim_edit_amp     = tk.StringVar(value='100')
        self._stim_edit_freq    = tk.StringVar(value='2.0')
        self._stim_edit_duty    = tk.StringVar(value='50.0')
        self._stim_edit_dur     = tk.StringVar(value='1.0')
        self._stim_edit_pw          = tk.StringVar(value='10.0')
        self._stim_edit_ips         = tk.StringVar(value='90.0')
        self._stim_edit_count       = tk.StringVar(value='10')
        self._stim_edit_continuous  = tk.BooleanVar(value=False)
        self._stim_edit_csv_path = tk.StringVar(value='(none)')
        self._stim_edit_csv_data = None
        self._stim_summary_labels = {}   # kept for compat; now backed by _stim_ch_btns
        self._stim_ch_btns        = {}   # flat -> tk.Button (visual channel selector)
        self._stim_view_mode      = tk.StringVar(value='single')  # 'single' | 'all8'
        self._stim_upload_status  = tk.StringVar(value="Not uploaded")
        self._stim_fig    = None
        self._stim_ax     = None
        self._stim_canvas = None

        # ---- settings ------------------------------------------------------
        _s = self._load_settings()
        self._data_dir_var  = tk.StringVar(value=_s.get('data_dir',  'data'))
        self._log_prefix_var = tk.StringVar(value=_s.get('log_prefix', 'mushio'))
        self._log_roll_var  = tk.StringVar(value=_s.get('log_roll', '1 hour'))
        self._cam_dir_var   = tk.StringVar(value=_s.get('cam_dir',   'captures'))
        self._log_active        = False
        self._auto_log_started  = False   # True once auto-log fired for this connection
        self._cmd_auto_next_try = 0.0     # time.time() after which CMD auto-connect may fire
        self._log_status_var = tk.StringVar(value='Not recording')
        self._log_file_var   = tk.StringVar(value='')
        self._settings_status = tk.StringVar(value='')
        self._hdf5_enabled_var  = tk.BooleanVar(value=False)
        # Hardware test / OTA status
        self._hw_test_status_var = tk.StringVar(value='')
        self._ota_status_var     = tk.StringVar(value='')

        # ---- firmware / programming tab vars (init here so helpers always work) --
        self._fw_fw_dir_var   = tk.StringVar(value='firmware')
        self._fw_c_fw_dir_var = tk.StringVar(value='firmware_c')
        self._fw_flash_method = tk.StringVar(value='auto')   # 'auto' | 'usb' | 'ota'
        # 'c_demo' | 'c_real'
        self._fw_mode         = tk.StringVar(value='c_demo')
        self._fw_stub_mode    = self._fw_mode   # legacy alias — do not remove
        self._fw_pico_port    = tk.StringVar(value='')
        # Serial monitor
        self._serial_baud     = tk.StringVar(value='115200')
        self._serial_active   = False
        self._serial_thread   = None
        self._serial_text     = None   # ScrolledText widget, built in settings tab
        self._serial_status   = tk.StringVar(value='Disconnected')
        self._fw_log_text     = None   # Deploy log ScrolledText, built in programming tab
        self._fw_conn_deadline = 0.0   # time.time() deadline for _fw_test_connection retries
        self._fw_ssid_history = _s.get('ssid_history', [])   # list[str], most-recent first
        self._fw_ssid_cb      = None                          # Combobox widget, built lazily
        # Try to detect the SSID the PC is currently connected to
        _auto_ssid = ''
        try:
            import subprocess, platform
            if platform.system() == 'Windows':
                _raw = subprocess.check_output(
                    ['netsh', 'wlan', 'show', 'interfaces'],
                    encoding='utf-8', errors='ignore',
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                for _ln in _raw.splitlines():
                    _s2 = _ln.strip()
                    if _s2.startswith('SSID') and not _s2.startswith('BSSID'):
                        _auto_ssid = _s2.split(':', 1)[-1].strip()
                        break
            elif platform.system() == 'Darwin':
                _raw = subprocess.check_output(
                    ['/System/Library/PrivateFrameworks/Apple80211.framework'
                     '/Versions/Current/Resources/airport', '-I'],
                    encoding='utf-8', errors='ignore')
                for _ln in _raw.splitlines():
                    if ' SSID:' in _ln:
                        _auto_ssid = _ln.split(':', 1)[-1].strip()
                        break
            else:  # Linux
                _auto_ssid = subprocess.check_output(
                    ['iwgetid', '-r'], encoding='utf-8', errors='ignore').strip()
        except Exception:
            pass
        self._fw_wifi_ssid    = tk.StringVar(value=_auto_ssid or
                                             (self._fw_ssid_history[0]
                                              if self._fw_ssid_history else ''))
        # Pre-load password from firmware/config.py so it shows as **** without
        # the user ever having to type it.  Falls back to '' if not found.
        _fw_pass_init = ''
        try:
            _cfg_path = os.path.join(self._fw_fw_dir_var.get() or 'firmware',
                                     'config.py')
            with open(_cfg_path, 'r') as _f:
                for _ln in _f:
                    if _ln.strip().startswith('WIFI_PASSWORD'):
                        _fw_pass_init = (_ln.split('=', 1)[1]
                                         .split('#')[0].strip().strip("'\""))
                        break
        except Exception:
            pass
        self._fw_wifi_pass    = tk.StringVar(value=_fw_pass_init)
        # Auto-detect this machine's LAN IP so the user doesn't have to type it
        try:
            _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _sock.connect(('8.8.8.8', 80))   # doesn't actually send traffic
            _local_ip = _sock.getsockname()[0]
            _sock.close()
        except Exception:
            try:
                _local_ip = socket.gethostbyname(socket.gethostname())
            except Exception:
                _local_ip = ''
        self._fw_host_ip   = tk.StringVar(value=_local_ip)
        self._fw_data_port = tk.StringVar(value='9000')
        self._fw_cmd_port  = tk.StringVar(value='9001')
        self._fw_status    = tk.StringVar(value='Ready')

        # ---- experiment identity ------------------------------------------
        self._exp_name_var        = tk.StringVar(value=_s.get('exp_name', ''))
        self._board_id_var        = tk.StringVar(value=_s.get('board_id', ''))
        self._species_common_var  = tk.StringVar(value=_s.get('species_common', ''))
        self._species_sci_var     = tk.StringVar(value=_s.get('species_scientific', ''))
        # description is plain text; stored separately (loaded in _finalize_ui)
        self._exp_desc_loaded     = _s.get('experiment_desc', '')
        # Auto-update log prefix whenever experiment name changes
        def _on_exp_name_change(*_):
            n = self._exp_name_var.get().strip()
            if n:
                self._log_prefix_var.set(n)
            else:
                self._log_prefix_var.set('mushio')
        self._exp_name_var.trace_add('write', _on_exp_name_change)

        # ---- ADC register vars -------------------------------------------
        # _reg_vars[(adc_idx, addr)] -> tk.StringVar  (hex string e.g. "0x10")
        self._reg_vars = {}
        for adc_i in range(NUM_ADCS):
            for addr, name, rw, desc, default, bits in ADS124S08_REGS:
                self._reg_vars[(adc_i, addr)] = tk.StringVar(
                    value=f'0x{default:02X}')

        # ---- misc ----------------------------------------------------------
        self._active_tab     = 'grid'
        self._autoscale_ctr  = 0
        self._cmd_history    = []
        self._history_idx    = -1
        self._elec_count_var = tk.StringVar(value="0 of 64 selected")
        # Terminal widget — None until Self-Test tab is first visited (lazy build)
        self._term_out    = None
        self._term_buffer = []   # (text, tag) pairs buffered before terminal exists

        self._cam_enable_var   = tk.BooleanVar(value=False)
        self._cam_idx_var      = tk.IntVar(value=0)
        self._cam_interval_var = tk.IntVar(value=10)
        self._cam_status_var   = tk.StringVar(value="Disabled")
        self._cam_preview_label = None          # tk.Label widget, built in left panel
        self._cam_preview_photo = None          # keep PIL ref to avoid GC
        self._cam_preview_interval_var = tk.IntVar(value=10)   # auto-refresh minutes

        # Build UI
        self._build_ui()

        # Start background services
        self._start_receiver()
        self._start_cmd_client()

        self._webcam = WebcamCapture(
            output_dir=self._cam_dir_var.get(),
            interval_s=self._cam_interval_var.get() * 60,
            cam_index=self._cam_idx_var.get(),
        )
        self._webcam.start()

        # Start render loops
        self._start_grid_loop()

        # Polling
        self._poll_cmd_queue()
        self._poll_status()
        self._poll_webcam()
        self._poll_recording_status()

        # Defer initial canvas draw so the window appears instantly
        self.root.after(80, self._finalize_ui)

    def _finalize_ui(self):
        """Draw all matplotlib canvases after the window is rendered."""
        if self._grid_canvas:
            self._grid_canvas.draw_idle()
        if self._overlay_canvas:
            self._overlay_canvas.draw_idle()
        if self._heatmap_canvas:
            self._heatmap_canvas.draw_idle()
        if self._stim_canvas:
            self._stim_canvas.draw_idle()
        # Populate experiment description text widget (needs window to exist first)
        if self._exp_desc_loaded and hasattr(self, '_exp_desc_txt'):
            self._exp_desc_txt.delete('1.0', tk.END)
            self._exp_desc_txt.insert('1.0', self._exp_desc_loaded)

    # ------------------------------------------------------------------
    # UI Construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use('clam')

        # Base styles
        style.configure('.',               background=BG_DARK,  foreground=FG_MAIN)
        style.configure('TLabel',          background=BG_DARK,  foreground=FG_MAIN)
        style.configure('TFrame',          background=BG_DARK)
        style.configure('TButton',         background=BG_LIGHT, foreground=FG_MAIN, padding=4)
        style.map('TButton',               background=[('active', '#45475a')])

        style.configure('TLabelframe',     background=BG_DARK,  foreground=ACCENT)
        style.configure('TLabelframe.Label', background=BG_DARK, foreground=ACCENT,
                        font=('Helvetica', 9, 'bold'))

        style.configure('TEntry',          fieldbackground=BG_LIGHT, foreground=FG_MAIN,
                        insertcolor=FG_MAIN)

        style.configure('TCheckbutton',    background=BG_DARK,  foreground=FG_MAIN,
                        indicatorcolor=BG_LIGHT)
        style.map('TCheckbutton',
                  background=[('active', BG_DARK),  ('pressed', BG_DARK)],
                  foreground=[('active', FG_MAIN),   ('pressed', FG_MAIN)],
                  indicatorcolor=[('selected', ACCENT)])

        style.configure('TRadiobutton',    background=BG_DARK,  foreground=FG_MAIN,
                        indicatorcolor=BG_LIGHT)
        style.map('TRadiobutton',
                  background=[('active', BG_DARK)],
                  foreground=[('active', FG_MAIN)],
                  indicatorcolor=[('selected', ACCENT)])

        style.configure('TCombobox',       fieldbackground=BG_LIGHT, foreground=FG_MAIN,
                        background=BG_LIGHT, arrowcolor=FG_MAIN, insertcolor=FG_MAIN)
        style.map('TCombobox',
                  fieldbackground=[('readonly', BG_LIGHT)],
                  foreground=[('readonly', FG_MAIN)],
                  selectforeground=[('readonly', FG_MAIN)],
                  selectbackground=[('readonly', BG_LIGHT)])

        style.configure('TSpinbox',        fieldbackground=BG_LIGHT, foreground=FG_MAIN,
                        arrowcolor=FG_MAIN)
        style.map('TSpinbox',
                  fieldbackground=[('readonly', BG_LIGHT)],
                  foreground=[('readonly', FG_MAIN)])

        style.configure('TNotebook',       background=BG_MID)
        style.configure('TNotebook.Tab',   background=BG_LIGHT, foreground=FG_DIM,
                        padding=[10, 4])
        style.map('TNotebook.Tab',
                  background=[('selected', BG_DARK)],
                  foreground=[('selected', FG_MAIN)])

        self._build_top_bar()

        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                                   bg=BG_DARK, sashwidth=5, sashrelief=tk.FLAT)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        left = self._build_left_panel(main_pane)
        main_pane.add(left, minsize=240, width=248)

        nb_frame = self._build_notebook_area(main_pane)
        main_pane.add(nb_frame, minsize=700)

        self._build_status_bar()

    # ---- top bar ---------------------------------------------------------

    def _build_top_bar(self):
        bar = tk.Frame(self.root, bg=BG_MID, pady=6, padx=8)
        bar.pack(fill=tk.X)

        ds = tk.Frame(bar, bg=BG_MID)
        ds.pack(fill=tk.X, pady=1)
        # SW Demo mode — top-right corner (pack RIGHT before LEFT items)
        self._demo_btn = tk.Button(
            ds, text="SW Demo mode  [Ctrl+D]",
            bg=BG_LIGHT, fg=FG_MAIN,
            activebackground=GREEN, activeforeground=BG_DARK,
            relief=tk.FLAT, bd=0, font=('Consolas', 9, 'bold'), pady=3,
            command=self._toggle_demo)
        self._demo_btn.pack(side=tk.RIGHT, padx=8)
        tk.Label(ds, text="Data stream  port:", bg=BG_MID, fg=FG_DIM).pack(side=tk.LEFT)
        self._data_port_var = tk.StringVar(value=str(self.data_port))
        tk.Entry(ds, textvariable=self._data_port_var, width=6,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        self._data_status_dot = tk.Label(ds, text="●", fg=FG_DIMMER,
                                          bg=BG_MID, font=('Helvetica', 14))
        self._data_status_dot.pack(side=tk.LEFT, padx=4)
        self._data_status_lbl = tk.Label(ds, text="Waiting for Pico...",
                                          bg=BG_MID, fg=FG_DIM)
        self._data_status_lbl.pack(side=tk.LEFT)

        cc = tk.Frame(bar, bg=BG_MID)
        cc.pack(fill=tk.X, pady=1)
        tk.Label(cc, text="Command  Pico IP:", bg=BG_MID, fg=FG_DIM).pack(side=tk.LEFT)
        self._cmd_host_var = tk.StringVar(value=self.cmd_host or "")
        tk.Entry(cc, textvariable=self._cmd_host_var, width=15,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        tk.Label(cc, text="port:", bg=BG_MID, fg=FG_DIM).pack(side=tk.LEFT)
        self._cmd_port_var = tk.StringVar(value=str(self.cmd_port))
        tk.Entry(cc, textvariable=self._cmd_port_var, width=6,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        self._cmd_connect_btn = ttk.Button(cc, text="Connect", command=self._connect_cmd)
        self._cmd_connect_btn.pack(side=tk.LEFT, padx=4)
        self._cmd_status_dot = tk.Label(cc, text="●", fg=FG_DIMMER,
                                         bg=BG_MID, font=('Helvetica', 14))
        self._cmd_status_dot.pack(side=tk.LEFT, padx=4)
        self._cmd_status_lbl = tk.Label(cc, text="Not connected", bg=BG_MID, fg=FG_DIM)
        self._cmd_status_lbl.pack(side=tk.LEFT)

    # ---- left panel (no scroll -- fits in normal window height) ----------

    def _build_left_panel(self, parent):
        outer = tk.Frame(parent, bg=BG_DARK, width=256)
        outer.pack_propagate(False)

        self._build_display_section(outer)
        self._build_signal_chain_section(outer)
        self._build_electrode_grid_section(outer)
        self._build_cam_preview_left(outer)

        return outer

    def _build_cam_preview_left(self, parent):
        """Compact camera preview pane in the left sidebar."""
        frame = ttk.LabelFrame(parent, text="Camera Preview", padding=4)
        frame.pack(fill=tk.X, padx=4, pady=3)

        if not HAS_CV2 and not HAS_PIL:
            tk.Label(frame, text="pip install opencv-python  to enable",
                     bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 7),
                     wraplength=220).pack(anchor=tk.W)
            return

        # Preview image label
        self._cam_preview_label = tk.Label(
            frame, bg='#0d0d14',
            text="No preview", fg=FG_DIMMER,
            font=('Helvetica', 7), width=30, height=7)
        self._cam_preview_label.pack(fill=tk.X, pady=(0, 4))

        # Controls row
        ctrl = tk.Frame(frame, bg=BG_DARK)
        ctrl.pack(fill=tk.X)
        tk.Label(ctrl, text="Refresh (min):", bg=BG_DARK,
                 fg=FG_DIM, font=('Helvetica', 7)).pack(side=tk.LEFT)
        self._cam_preview_interval_var = tk.IntVar(value=10)
        ttk.Spinbox(ctrl, textvariable=self._cam_preview_interval_var,
                    from_=1, to=1440, width=5).pack(side=tk.LEFT, padx=2)
        ttk.Button(ctrl, text="Take Image",
                   command=self._cam_take_and_preview).pack(side=tk.LEFT, padx=4)

        # Start the auto-refresh loop
        self._schedule_cam_preview()

    # ---- left panel sections ---------------------------------------------

    def _build_display_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Display", padding=6)
        frame.pack(fill=tk.X, padx=4, pady=3)

        # ---- Time window -----------------------------------------------
        tk.Label(frame, text="Time window (s):", bg=BG_DARK,
                 fg=FG_DIM).grid(row=0, column=0, sticky=tk.W)
        _tw_values = [1, 2, 5, 10, 20, 'Custom']
        self._tw_combo = ttk.Combobox(frame, textvariable=self._display_secs,
                                       width=6, values=_tw_values, state='readonly')
        self._tw_combo.grid(row=0, column=1, sticky=tk.W, padx=4)

        # Custom time-window entry (hidden until 'Custom' is selected)
        self._tw_custom_frame = tk.Frame(frame, bg=BG_DARK)
        self._tw_custom_frame.grid(row=1, column=0, columnspan=2, sticky=tk.W,
                                    pady=(0, 2))
        self._tw_custom_frame.grid_remove()   # hidden by default
        tk.Label(self._tw_custom_frame, text="Seconds:", bg=BG_DARK,
                 fg=FG_DIM).pack(side=tk.LEFT)
        self._tw_custom_var = tk.StringVar(value='')
        _tw_entry = tk.Entry(self._tw_custom_frame, textvariable=self._tw_custom_var,
                              width=7, bg=BG_LIGHT, fg=FG_MAIN,
                              insertbackground=FG_MAIN, relief=tk.FLAT)
        _tw_entry.pack(side=tk.LEFT, padx=4)
        _tw_entry.bind('<Return>',      self._apply_custom_time_window)
        _tw_entry.bind('<FocusOut>',    self._apply_custom_time_window)
        self._tw_combo.bind('<<ComboboxSelected>>', self._on_tw_combo_select)

        # ---- Y-scale ---------------------------------------------------
        tk.Label(frame, text="Y-scale (uV):", bg=BG_DARK,
                 fg=FG_DIM).grid(row=2, column=0, sticky=tk.W)
        cb2 = ttk.Combobox(frame, textvariable=self._uv_scale, width=6,
                            values=[50, 100, 200, 500, 1000, 2000, 5000], state='readonly')
        cb2.grid(row=2, column=1, sticky=tk.W, padx=4)

        ttk.Checkbutton(frame, text="Auto-scale Y",
                         variable=self._auto_scale).grid(
                         row=3, column=0, columnspan=2, sticky=tk.W, pady=2)

        # ---- FFT max Hz ------------------------------------------------
        tk.Label(frame, text="FFT max (Hz):", bg=BG_DARK,
                 fg=FG_DIM).grid(row=4, column=0, sticky=tk.W, pady=(4, 0))
        tk.Entry(frame, textvariable=self._fft_max_hz, width=6,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 relief=tk.FLAT).grid(row=4, column=1, sticky=tk.W,
                                      padx=4, pady=(4, 0))
        tk.Label(frame, text="0 = auto (Nyquist)", bg=BG_DARK, fg=FG_DIMMER,
                 font=('Helvetica', 7)).grid(row=5, column=0, columnspan=2,
                                             sticky=tk.W)

    def _on_tw_combo_select(self, _event=None):
        val = self._tw_combo.get()
        if val == 'Custom':
            self._tw_custom_frame.grid()
            self._tw_custom_var.set('')
        else:
            self._tw_custom_frame.grid_remove()
            try:
                self._display_secs.set(float(val))
            except ValueError:
                pass

    def _apply_custom_time_window(self, _event=None):
        try:
            v = float(self._tw_custom_var.get())
            if v > 0:
                self._display_secs.set(v)
        except ValueError:
            pass

    def _update_gain_label(self, *_):
        total = self._pga_gain.get() * GAIN
        self._gain_info.config(text=f"Total gain: {self._pga_gain.get()} x {GAIN} (AFE) = {total}x")

    def _build_signal_chain_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Signal Chain", padding=6)
        frame.pack(fill=tk.X, padx=4, pady=3)

        # ---- PGA gain ---------------------------------------------------
        r0 = tk.Frame(frame, bg=BG_DARK)
        r0.pack(fill=tk.X, pady=(0, 2))
        tk.Label(r0, text="PGA gain:", bg=BG_DARK, fg=FG_DIM, width=9,
                 anchor=tk.W).pack(side=tk.LEFT)
        ttk.Combobox(r0, textvariable=self._pga_gain, width=5,
                     values=[1, 2, 4, 8, 16, 32],
                     state='readonly').pack(side=tk.LEFT, padx=2)
        self._gain_info = tk.Label(r0, bg=BG_DARK, fg=FG_DIMMER,
                                    font=('Helvetica', 7))
        self._gain_info.pack(side=tk.LEFT, padx=(4, 0))
        self._pga_gain.trace_add('write', self._update_gain_label)
        self._update_gain_label()

        # ---- ADC digital filter -----------------------------------------
        # ADS124S08 DATARATE register bits 4:2.  Lower = better noise, slower settling.
        r1 = tk.Frame(frame, bg=BG_DARK)
        r1.pack(fill=tk.X, pady=2)
        tk.Label(r1, text="ADC filter:", bg=BG_DARK, fg=FG_DIM, width=9,
                 anchor=tk.W).pack(side=tk.LEFT)
        _filt_opts = ['SINC1', 'SINC2', 'SINC3', 'SINC4', 'FIR']
        ttk.Combobox(r1, textvariable=self._adc_filter_var, width=7,
                     values=_filt_opts,
                     state='readonly').pack(side=tk.LEFT, padx=2)
        tk.Label(r1, text="(reg 0x04[4:2])", bg=BG_DARK, fg=FG_DIMMER,
                 font=('Helvetica', 7)).pack(side=tk.LEFT, padx=(4, 0))

        tk.Label(frame,
                 text="SINC1=fast/noisy  SINC3=default  FIR=50/60 Hz reject",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 4))

        # ---- Data rate --------------------------------------------------
        # ADS124S08 DATARATE register bits 2:0.  Lower SPS = better SNR.
        r2 = tk.Frame(frame, bg=BG_DARK)
        r2.pack(fill=tk.X, pady=2)
        tk.Label(r2, text="Data rate:", bg=BG_DARK, fg=FG_DIM, width=9,
                 anchor=tk.W).pack(side=tk.LEFT)
        _dr_opts = ['2.5', '5', '10', '16.6', '20', '50',
                    '100', '200', '400', '800', '1000', '2000', '4000']
        ttk.Combobox(r2, textvariable=self._adc_dr_var, width=7,
                     values=_dr_opts,
                     state='readonly').pack(side=tk.LEFT, padx=2)
        tk.Label(r2, text="SPS  (reg 0x04[2:0])", bg=BG_DARK, fg=FG_DIMMER,
                 font=('Helvetica', 7)).pack(side=tk.LEFT, padx=(4, 0))

        tk.Label(frame,
                 text="Lower SPS = lower noise floor  |  2.5 SPS ideal for slow LFP",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 4))

        # ---- Reference buffer -------------------------------------------
        # ADS124S08 REF register bits 5:4.  Buffers VREF to reduce noise on REFP/REFN.
        r3 = tk.Frame(frame, bg=BG_DARK)
        r3.pack(fill=tk.X, pady=2)
        ttk.Checkbutton(r3, text="Enable reference buffer",
                        variable=self._adc_refbuf_var).pack(side=tk.LEFT)
        tk.Label(r3, text="  (REF reg 0x05[5:4])", bg=BG_DARK, fg=FG_DIMMER,
                 font=('Helvetica', 7)).pack(side=tk.LEFT)

        tk.Label(frame,
                 text="Buffers REFP0/REFN0 to reduce VREF noise at cost of settling time",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 4))

        # ---- System monitor (internal temp / supply) --------------------
        # ADS124S08 SYS register bits 7:5.  Routes internal signals to ADC mux.
        r4 = tk.Frame(frame, bg=BG_DARK)
        r4.pack(fill=tk.X, pady=2)
        tk.Label(r4, text="Sys mon:", bg=BG_DARK, fg=FG_DIM, width=9,
                 anchor=tk.W).pack(side=tk.LEFT)
        _sm_opts = ['Off', 'Internal temp', 'AVDD/4', 'DVDD/4']
        ttk.Combobox(r4, textvariable=self._adc_sysmon_var, width=13,
                     values=_sm_opts,
                     state='readonly').pack(side=tk.LEFT, padx=2)
        tk.Label(r4, text="(SYS reg 0x09[7:5])", bg=BG_DARK, fg=FG_DIMMER,
                 font=('Helvetica', 7)).pack(side=tk.LEFT, padx=(4, 0))

        tk.Label(frame,
                 text="Routes ADC die temp or supply rails to ch 72 for board health mon.",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 4))

        # ---- Calibration ------------------------------------------------
        # SELFOCAL = offset calibration; SELFGCAL = full-scale gain calibration.
        # Applied per-ADC; results stored in OFCAL0-2 and FSCAL0-2 registers.
        r5 = tk.Frame(frame, bg=BG_DARK)
        r5.pack(fill=tk.X, pady=(2, 0))
        tk.Label(r5, text="Calibrate:", bg=BG_DARK, fg=FG_DIM, width=9,
                 anchor=tk.W).pack(side=tk.LEFT)
        tk.Button(r5, text="Offset (SELFOCAL)", bg=BG_LIGHT, fg=FG_MAIN,
                  relief=tk.FLAT, padx=6, pady=2,
                  command=lambda: self._do_adc_cal('selfocal')
                  ).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(r5, text="Gain (SELFGCAL)", bg=BG_LIGHT, fg=FG_MAIN,
                  relief=tk.FLAT, padx=6, pady=2,
                  command=lambda: self._do_adc_cal('selfgcal')
                  ).pack(side=tk.LEFT)

        tk.Label(frame,
                 text="Short AINP=AINN then run Offset cal.  Apply to live electrodes for Gain cal.",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 justify=tk.LEFT).pack(anchor=tk.W, pady=(2, 4))

        # ---- Separator --------------------------------------------------
        ttk.Separator(frame, orient='horizontal').pack(fill=tk.X, pady=4)

        # ---- Bandpass filter --------------------------------------------
        if not HAS_SCIPY:
            tk.Label(frame, text="Bandpass: install scipy to enable\n(pip install scipy)",
                     bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 8),
                     justify=tk.LEFT).pack(anchor=tk.W)
            return

        ttk.Checkbutton(frame, text="Enable bandpass",
                         variable=self._bp_enabled).pack(anchor=tk.W)

        for label, var, unit in [("Low  :", self._bp_low,  "Hz"),
                                  ("High :", self._bp_high, "Hz")]:
            r = tk.Frame(frame, bg=BG_DARK)
            r.pack(fill=tk.X, pady=1)
            tk.Label(r, text=label, bg=BG_DARK, fg=FG_DIM, width=5).pack(side=tk.LEFT)
            tk.Entry(r, textvariable=var, width=7,
                     bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                     relief=tk.FLAT).pack(side=tk.LEFT, padx=2)
            tk.Label(r, text=unit, bg=BG_DARK, fg=FG_DIM).pack(side=tk.LEFT)

        r3 = tk.Frame(frame, bg=BG_DARK)
        r3.pack(fill=tk.X, pady=1)
        tk.Label(r3, text="Order:", bg=BG_DARK, fg=FG_DIM, width=5).pack(side=tk.LEFT)
        ttk.Combobox(r3, textvariable=self._bp_order_var, width=4,
                     values=[2, 4, 6, 8], state='readonly').pack(side=tk.LEFT, padx=2)

        tk.Label(frame,
                 text="Tip: set High < Nyquist to limit display BW\n"
                      "e.g. Low=0.1 Hz  High=10 Hz  →  LFP view",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))

    def _build_stim_section(self, parent):
        frame = ttk.LabelFrame(parent, text="STIM Channels", padding=6)
        frame.pack(fill=tk.X, padx=4, pady=3)

        tk.Label(frame, text="ADS124S08 IDAC current presets (uA):",
                 bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 7)).pack(anchor=tk.W)
        tk.Label(frame,
                 text="10 · 50 · 100 · 250 · 500 · 750 · 1000 · 1500 · 2000",
                 bg=BG_DARK, fg=ACCENT, font=('Consolas', 7)).pack(anchor=tk.W, pady=(0, 4))
        tk.Label(frame,
                 text="Click outer-ring buttons in grid below to\ntoggle STIM channels in Overlay.",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 justify=tk.LEFT).pack(anchor=tk.W)
        tk.Label(frame, text="Configure waveforms in STIM Ctrl tab.",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7)).pack(anchor=tk.W)

    def _build_electrode_grid_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Physical Electrode Layout (10x10)", padding=4)
        frame.pack(fill=tk.X, padx=4, pady=3)

        tk.Label(frame,
                 text="Inner: click to highlight/Overlay.  Outer: STIM (click to toggle).",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 wraplength=230, justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 2))

        CW   = 2
        FONT = ('Consolas', 7)

        # Column header row: row-label spacer + outer-left + 0-7 + outer-right
        hdr = tk.Frame(frame, bg=BG_DARK)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="", bg=BG_DARK, font=FONT, width=2).pack(side=tk.LEFT, padx=1)
        tk.Label(hdr, text="·", bg=BG_DARK, fg='#252535', font=FONT, width=CW
                 ).pack(side=tk.LEFT, padx=1)
        for c in range(8):
            tk.Label(hdr, text=str(c), bg=BG_DARK, fg=FG_DIM,
                     font=FONT, width=CW, anchor=tk.CENTER).pack(side=tk.LEFT, padx=1)
        tk.Label(hdr, text="·", bg=BG_DARK, fg='#252535', font=FONT, width=CW
                 ).pack(side=tk.LEFT, padx=1)

        # 10 physical rows: pr = -1 .. 8
        for pr in range(-1, 9):
            rf = tk.Frame(frame, bg=BG_DARK)
            rf.pack(fill=tk.X)
            # Row label
            if 0 <= pr <= 7:
                tk.Label(rf, text=str(pr), bg=BG_DARK, fg=FG_DIM,
                         font=FONT, width=2).pack(side=tk.LEFT, padx=1)
            else:
                tk.Label(rf, text="·", bg=BG_DARK, fg='#252535',
                         font=FONT, width=2).pack(side=tk.LEFT, padx=1)
            # 10 physical columns: pc = -1 .. 8
            for pc in range(-1, 9):
                is_corner = (pc, pr) in AGND_PHYS
                is_stim   = (pc, pr) in STIM_PHYS_TO_INFO
                is_inner  = 0 <= pc <= 7 and 0 <= pr <= 7

                if is_corner:
                    # AGND corners are not shown — render as blank outer cell
                    tk.Label(rf, text="·", bg=BG_DARK, fg='#252535',
                             font=FONT, width=CW).pack(side=tk.LEFT, padx=1)
                elif is_stim:
                    sname, sfidx = STIM_PHYS_TO_INFO[(pc, pr)]
                    btn = tk.Button(
                        rf, text=sname[-1], width=CW,
                        bg='#251520', fg=ORANGE,
                        activebackground=ORANGE, activeforeground=BG_DARK,
                        relief=tk.FLAT, bd=0, font=FONT, pady=1,
                        command=lambda f=sfidx: self._toggle_stim(f))
                    btn.pack(side=tk.LEFT, padx=1, pady=1)
                    self._stim_btns[sfidx] = btn
                elif is_inner:
                    flat = ELEC_GRID.get((pc, pr))
                    if flat is not None:
                        btn = tk.Button(
                            rf, text=f"{pc}{pr}", width=CW,
                            bg=BG_LIGHT, fg=FG_DIM,
                            activebackground=ACCENT, activeforeground=BG_DARK,
                            relief=tk.FLAT, bd=0, font=FONT, pady=2,
                            command=lambda cc=pc, rr=pr: self._toggle_electrode(cc, rr))
                        btn.pack(side=tk.LEFT, padx=1, pady=1)
                        self._elec_btns[(pc, pr)] = btn
                    else:
                        tk.Label(rf, text="--", bg=BG_DARK, fg=BG_LIGHT,
                                 font=FONT, width=CW).pack(side=tk.LEFT, padx=1)
                else:
                    # Empty outer non-STIM non-AGND position
                    tk.Label(rf, text="·", bg=BG_DARK, fg='#252535',
                             font=FONT, width=CW).pack(side=tk.LEFT, padx=1)

        qs = tk.Frame(frame, bg=BG_DARK)
        qs.pack(fill=tk.X, pady=3)
        ttk.Button(qs, text="All",  command=self._select_all_elec).pack(side=tk.LEFT, padx=2)
        ttk.Button(qs, text="None", command=self._select_none_elec).pack(side=tk.LEFT, padx=2)
        tk.Label(qs, textvariable=self._elec_count_var,
                 bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 7)).pack(side=tk.LEFT, padx=4)

    def _build_selftest_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Self-Test", padding=6)
        frame.pack(fill=tk.X, padx=4, pady=3)

        # Pause / Resume streaming so self-test results aren't drowned out
        self._pause_btn = tk.Button(
            frame, text="  Pause Streaming  ",
            bg=BG_LIGHT, fg=FG_MAIN, activebackground=ORANGE,
            activeforeground=BG_DARK, relief=tk.FLAT, pady=4,
            font=('Consolas', 9, 'bold'),
            command=self._toggle_stream_pause)
        self._pause_btn.pack(fill=tk.X, pady=(0, 6))

        for label, cmd in [("Ping",                "ping"),
                            ("Status",             "status"),
                            ("Read ADC Registers", "read_regs"),
                            ("Scan All Channels",  "scan_all"),
                            ("Benchmark",          "benchmark")]:
            ttk.Button(frame, text=label,
                       command=lambda c=cmd: self._send_cmd(c)).pack(fill=tk.X, pady=1)
        # Proof-of-life button — blinks the board's status LED 5 times
        tk.Button(frame, text="  Blink LED  (proof of life)",
                  bg='#1a1a3a', fg=ORANGE, activebackground=ORANGE,
                  activeforeground=BG_DARK, relief=tk.FLAT, pady=4,
                  font=('Helvetica', 8, 'bold'),
                  command=lambda: self._send_cmd("blink_led 5")
                  ).pack(fill=tk.X, pady=(4, 1))

    def _toggle_stream_pause(self):
        """Pause or resume data ingestion for self-test operations."""
        paused = not self._stream_paused_var.get()
        self._stream_paused_var.set(paused)
        if self._receiver:
            self._receiver._paused = paused
        if paused:
            self._pause_btn.config(text="  Resume Streaming  ",
                                   bg=ORANGE, fg=BG_DARK)
        else:
            self._pause_btn.config(text="  Pause Streaming  ",
                                   bg=BG_LIGHT, fg=FG_MAIN)

    def _build_webcam_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Webcam Capture", padding=6)
        frame.pack(fill=tk.X, padx=4, pady=3)

        if not HAS_CV2:
            tk.Label(frame, text="pip install opencv-python to enable",
                     bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 8),
                     wraplength=200).pack(anchor=tk.W)
            return

        ttk.Checkbutton(frame, text="Enable periodic capture",
                         variable=self._cam_enable_var,
                         command=self._on_cam_enable).pack(anchor=tk.W)
        r1 = tk.Frame(frame, bg=BG_DARK)
        r1.pack(fill=tk.X, pady=1)
        tk.Label(r1, text="Camera idx:", bg=BG_DARK, fg=FG_DIM).pack(side=tk.LEFT)
        ttk.Spinbox(r1, textvariable=self._cam_idx_var, from_=0, to=9, width=3
                    ).pack(side=tk.LEFT, padx=4)
        r2 = tk.Frame(frame, bg=BG_DARK)
        r2.pack(fill=tk.X, pady=1)
        tk.Label(r2, text="Interval (min):", bg=BG_DARK, fg=FG_DIM).pack(side=tk.LEFT)
        ttk.Spinbox(r2, textvariable=self._cam_interval_var, from_=1, to=60, width=4
                    ).pack(side=tk.LEFT, padx=4)
        ttk.Button(frame, text="Capture Now",
                   command=self._cam_capture_now).pack(fill=tk.X, pady=2)
        tk.Label(frame, textvariable=self._cam_status_var,
                 bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 7),
                 wraplength=200).pack(anchor=tk.W)

    # ---- notebook (tabs built lazily on first visit) --------------------

    _TAB_DEFS = [
        # (key,          label,                    builder_attr)
        ('grid',         '  Grid Layout  ',        '_build_grid_tab'),
        ('overlay',      '  Overlay  ',            '_build_overlay_tab'),
        ('waterfall',    '  Waterfall  ',          '_build_waterfall_tab'),
        ('heatmap',      '  Heatmap  ',            '_build_heatmap_tab'),
        ('stim',         '  STIM Ctrl  ',          '_build_stim_waveform_tab'),
        ('settings',     '  Experiment Settings  ','_build_settings_tab'),
        ('regs',         '  ADC Regs  ',           '_build_regs_tab'),
        ('selftest',     '  Self-Test  ',          '_build_selftest_tab'),
        ('programming',  '  Programming  ',        '_build_programming_tab'),
    ]

    def _build_notebook_area(self, parent):
        frame = tk.Frame(parent, bg=BG_DARK)
        self._notebook = ttk.Notebook(frame)
        self._notebook.pack(fill=tk.BOTH, expand=True)

        self._tab_frames  = {}
        self._tab_built   = {}

        for key, label, _ in self._TAB_DEFS:
            tab = tk.Frame(self._notebook, bg=BG_DARK)
            self._notebook.add(tab, text=label)
            self._tab_frames[key] = tab
            self._tab_built[key]  = False

        # Build the startup tab (Grid) immediately
        self._build_grid_tab(self._tab_frames['grid'])
        self._tab_built['grid'] = True

        self._notebook.bind('<<NotebookTabChanged>>', self._on_tab_change)
        return frame

    # ---- Grid tab --------------------------------------------------------

    def _build_grid_tab(self, parent):
        tb = tk.Frame(parent, bg=BG_MID, pady=4)
        tb.pack(fill=tk.X, padx=4)

        self._grid_fft_btn = tk.Button(
            tb, text=" Time ", width=7, bg=BG_LIGHT, fg=FG_MAIN,
            relief=tk.FLAT, bd=0, font=('Consolas', 9, 'bold'), pady=3,
            command=self._toggle_fft)
        self._grid_fft_btn.pack(side=tk.LEFT, padx=6)
        ttk.Button(tb, text="Select All",  command=self._select_all_elec).pack(side=tk.LEFT, padx=2)
        ttk.Button(tb, text="Select None", command=self._select_none_elec).pack(side=tk.LEFT, padx=2)
        tk.Label(tb, text="  Selected = bright + added to Overlay",
                 bg=BG_MID, fg=FG_DIMMER, font=('Helvetica', 8)).pack(side=tk.LEFT, padx=8)
        self._grid_fft_note = tk.Label(tb, text="X=Hz  Y=Amplitude (uV / mV)",
                                        bg=BG_MID, fg=ACCENT, font=('Helvetica', 8))

        # 10x10 GridSpec: outer ring (row/col 0 and 9) = STIM/AGND, inner (1-8) = electrodes
        self._grid_fig = Figure(figsize=(11, 9.5), dpi=80, facecolor=BG_MID)
        gs = GridSpec(10, 10, figure=self._grid_fig,
                      height_ratios=[0.55] + [1]*8 + [0.55],
                      width_ratios=[0.55]  + [1]*8 + [0.55],
                      hspace=0.08, wspace=0.08,
                      left=0.01, right=0.99, top=0.99, bottom=0.01)

        self._grid_lines      = {}
        self._grid_axes       = {}
        self._grid_texts      = {}
        self._clip_texts      = {}   # flat -> Text artist, flashes CLIP! when raw signal saturates
        self._stim_grid_lines = {}
        self._stim_grid_axes  = {}
        # Reverse maps for click-to-toggle: axes object -> flat index
        self._grid_ax_to_elec = {}   # ax -> electrode flat index
        self._grid_ax_to_stim = {}   # ax -> stim flat index

        for pr in range(-1, 9):       # physical row  -1..8
            for pc in range(-1, 9):   # physical col  -1..8
                gs_row = pr + 1       # GridSpec index 0..9
                gs_col = pc + 1

                is_corner = (pc, pr) in AGND_PHYS
                is_stim   = (pc, pr) in STIM_PHYS_TO_INFO
                is_inner  = 0 <= pc <= 7 and 0 <= pr <= 7

                if not (is_stim or is_inner):
                    continue   # leave AGND corners and empty cells as figure background

                ax = self._grid_fig.add_subplot(gs[gs_row, gs_col])
                ax.tick_params(labelbottom=False, labelleft=False,
                               bottom=False, left=False)

                if is_stim:
                    sname, flat = STIM_PHYS_TO_INFO[(pc, pr)]
                    ax.set_facecolor(STIM_BG)
                    for sp in ax.spines.values():
                        sp.set_color(ORANGE); sp.set_linewidth(0.8)
                    line, = ax.plot([], [], color=ORANGE, linewidth=0.7, rasterized=True)
                    ax.text(0.04, 0.92, sname, transform=ax.transAxes,
                            fontsize=4, color=ORANGE, va='top', ha='left')
                    self._stim_grid_lines[flat] = line
                    self._stim_grid_axes[flat]  = ax
                    self._grid_ax_to_stim[ax]   = flat

                else:  # inner electrode
                    flat = ELEC_GRID.get((pc, pr))
                    # ---- Spatial colour-wheel background -------------------------
                    # Hue  = compass direction from array centre (0..7 × 0..7).
                    # Shade = distance from centre: outer cells are more saturated
                    #         and slightly brighter; centre cells are near-neutral.
                    _cx, _cy = 3.5, 3.5          # array centre in physical coords
                    _dx, _dy = pc - _cx, _cy - pr  # flip dy so N=top=warm colours
                    _max_d   = math.sqrt(_cx**2 + _cy**2)   # ≈ 4.95
                    _dist    = math.sqrt(_dx**2 + _dy**2) / _max_d  # 0..1
                    _hue     = (math.atan2(_dy, _dx) / (2 * math.pi)) % 1.0
                    _sat     = _dist * 0.55          # 0 at centre → 0.55 at corner
                    _val     = 0.11 + _dist * 0.06   # 0.11–0.17  (stays very dark)
                    _r, _g, _b = colorsys.hsv_to_rgb(_hue, _sat, _val)
                    _bg_cell = '#{:02x}{:02x}{:02x}'.format(
                        int(_r * 255), int(_g * 255), int(_b * 255))
                    ax.set_facecolor(_bg_cell)
                    for sp in ax.spines.values():
                        sp.set_color(BG_LIGHT); sp.set_linewidth(0.4)
                    if flat is not None:
                        nm = ALL_NAMES[flat]
                        line, = ax.plot([], [], color=FG_DIMMER,
                                        linewidth=0.5, rasterized=True)
                        self._grid_lines[flat] = line
                        self._grid_axes[flat]  = ax
                        self._grid_ax_to_elec[ax] = flat
                        self._grid_texts[flat] = ax.text(
                            0.04, 0.92, nm[4:], transform=ax.transAxes,
                            fontsize=4, color=FG_DIMMER, va='top', ha='left')
                        # Clipping warning — shown when bandpass on but raw clips
                        self._clip_texts[flat] = ax.text(
                            0.5, 0.5, 'CLIP!', transform=ax.transAxes,
                            ha='center', va='center', fontsize=5,
                            color=RED, fontweight='bold',
                            visible=False, zorder=10,
                            bbox=dict(boxstyle='round,pad=0.2',
                                      facecolor='#3a0000', edgecolor=RED,
                                      alpha=0.85))
                    else:
                        ax.set_facecolor('#0d0d14')

        self._grid_canvas = FigureCanvasTkAgg(self._grid_fig, master=parent)
        self._grid_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._grid_canvas.mpl_connect('button_press_event', self._on_grid_click)

    # ---- Overlay tab -----------------------------------------------------

    def _build_overlay_tab(self, parent):
        tb = tk.Frame(parent, bg=BG_MID, pady=4)
        tb.pack(fill=tk.X, padx=4)

        self._overlay_fft_btn = tk.Button(
            tb, text=" Time ", width=7, bg=BG_LIGHT, fg=FG_MAIN,
            relief=tk.FLAT, bd=0, font=('Consolas', 9, 'bold'), pady=3,
            command=self._toggle_fft)
        self._overlay_fft_btn.pack(side=tk.LEFT, padx=6)

        tk.Label(tb, text="Panels:", bg=BG_MID, fg=FG_MAIN).pack(side=tk.LEFT, padx=(8, 2))
        for n in (1, 2, 4):
            tk.Radiobutton(tb, text=str(n), variable=self._overlay_panels, value=n,
                           bg=BG_MID, fg=FG_MAIN, selectcolor=BG_LIGHT,
                           activebackground=BG_MID, activeforeground=ACCENT,
                           command=self._rebuild_overlay_axes).pack(side=tk.LEFT)

        # Y-axis scale controls (inline — no need to use the left panel)
        tk.Label(tb, text="  Y \u00b1", bg=BG_MID, fg=FG_DIM,
                 font=('Helvetica', 8)).pack(side=tk.LEFT, padx=(14, 2))
        ttk.Combobox(tb, textvariable=self._uv_scale, width=6,
                     values=[10, 50, 100, 200, 500, 1000, 2000, 5000],
                     state='readonly').pack(side=tk.LEFT)
        tk.Label(tb, text="uV", bg=BG_MID, fg=FG_DIMMER,
                 font=('Helvetica', 7)).pack(side=tk.LEFT, padx=(2, 0))
        ttk.Checkbutton(tb, text="Auto", variable=self._auto_scale
                        ).pack(side=tk.LEFT, padx=(8, 2))

        self._overlay_fig = Figure(figsize=(11, 6), dpi=96, facecolor=BG_MID)
        self._overlay_canvas = FigureCanvasTkAgg(self._overlay_fig, master=parent)
        self._overlay_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._overlay_axes    = []
        self._stim_overlay_ax = None
        self._rebuild_overlay_axes()
        self._start_overlay_loop()

    def _rebuild_overlay_axes(self):
        self._overlay_fig.clear()
        n = self._overlay_panels.get()
        # n electrode panels + 1 STIM panel
        gs = GridSpec(n + 1, 1, figure=self._overlay_fig,
                      height_ratios=[2]*n + [1],
                      hspace=0.30,
                      left=0.07, right=0.80, top=0.97, bottom=0.05)

        self._overlay_axes = []
        for i in range(n):
            ax = self._overlay_fig.add_subplot(gs[i, 0])
            ax.set_facecolor(BG_MID)
            ax.tick_params(colors=FG_DIMMER, labelsize=7)
            for sp in ax.spines.values():
                sp.set_color(BG_LIGHT); sp.set_linewidth(0.6)
            self._overlay_axes.append(ax)

        # STIM panel (always visible, orange border)
        self._stim_overlay_ax = self._overlay_fig.add_subplot(gs[n, 0])
        self._stim_overlay_ax.set_facecolor(STIM_BG)
        self._stim_overlay_ax.tick_params(colors=ORANGE, labelsize=7)
        for sp in self._stim_overlay_ax.spines.values():
            sp.set_color(ORANGE); sp.set_linewidth(0.9)
        self._stim_overlay_ax.set_title("STIM Programmed Output  (uA)",
                                         color=ORANGE, fontsize=7, pad=2)

    # ---- Waterfall tab ---------------------------------------------------

    def _build_waterfall_tab(self, parent):
        """Stacked-waterfall view: channels separated vertically by a fixed offset."""
        tb = tk.Frame(parent, bg=BG_MID, pady=4)
        tb.pack(fill=tk.X, padx=4)

        self._wf_fft_btn = tk.Button(
            tb, text=" Time ", width=7, bg=BG_LIGHT, fg=FG_MAIN,
            relief=tk.FLAT, bd=0, font=('Consolas', 9, 'bold'), pady=3,
            command=self._toggle_fft)
        self._wf_fft_btn.pack(side=tk.LEFT, padx=6)

        tk.Label(tb, text="Stack offset:", bg=BG_MID, fg=FG_DIM,
                 font=('Helvetica', 8)).pack(side=tk.LEFT, padx=(14, 2))
        tk.Entry(tb, textvariable=self._waterfall_var, width=7,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 relief=tk.FLAT).pack(side=tk.LEFT)
        tk.Label(tb, text="uV", bg=BG_MID, fg=FG_DIMMER,
                 font=('Helvetica', 7)).pack(side=tk.LEFT, padx=(2, 0))

        tk.Label(tb, text="  Y \u00b1", bg=BG_MID, fg=FG_DIM,
                 font=('Helvetica', 8)).pack(side=tk.LEFT, padx=(14, 2))
        ttk.Combobox(tb, textvariable=self._uv_scale, width=6,
                     values=[10, 50, 100, 200, 500, 1000, 2000, 5000],
                     state='readonly').pack(side=tk.LEFT)
        tk.Label(tb, text="uV", bg=BG_MID, fg=FG_DIMMER,
                 font=('Helvetica', 7)).pack(side=tk.LEFT, padx=(2, 0))
        ttk.Checkbutton(tb, text="Auto", variable=self._auto_scale
                        ).pack(side=tk.LEFT, padx=(8, 2))

        self._wf_fig    = Figure(figsize=(11, 6), dpi=96, facecolor=BG_MID)
        self._wf_canvas = FigureCanvasTkAgg(self._wf_fig, master=parent)
        self._wf_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._wf_axes    = []
        self._wf_stim_ax = None
        self._rebuild_waterfall_axes()
        self._start_waterfall_loop()

    def _rebuild_waterfall_axes(self):
        """Single full-screen axes — no panel split, no STIM sub-panel."""
        self._wf_fig.clear()
        # Use add_axes with manual margins so the channel labels on the left
        # have enough room and the plot fills the rest of the canvas.
        ax = self._wf_fig.add_axes([0.09, 0.05, 0.89, 0.93])
        ax.set_facecolor(BG_MID)
        ax.tick_params(colors=FG_DIMMER, labelsize=7)
        for sp in ax.spines.values():
            sp.set_color(BG_LIGHT); sp.set_linewidth(0.6)
        self._wf_axes    = [ax]
        self._wf_stim_ax = None   # no STIM sub-panel in waterfall

    # ---- Heatmap tab -----------------------------------------------------

    def _build_heatmap_tab(self, parent):
        tb = tk.Frame(parent, bg=BG_MID, pady=4)
        tb.pack(fill=tk.X, padx=4)

        tk.Label(tb, text="Metric:", bg=BG_MID, fg=FG_MAIN).pack(side=tk.LEFT, padx=6)
        ttk.Combobox(tb, textvariable=self._heatmap_metric, width=14,
                     values=["RMS","Bandpass-RMS","Peak","FFT-Peak-Freq"],
                     state='readonly').pack(side=tk.LEFT, padx=4)
        tk.Label(tb, text="  Colormap:", bg=BG_MID, fg=FG_MAIN).pack(side=tk.LEFT, padx=4)
        ttk.Combobox(tb, textvariable=self._cmap_var, width=10,
                     values=["plasma","viridis","inferno","magma","hot","RdBu_r"],
                     state='readonly').pack(side=tk.LEFT, padx=4)

        self._heatmap_fig = Figure(figsize=(7, 6.5), dpi=96, facecolor=BG_MID)
        gs = GridSpec(2, 1, figure=self._heatmap_fig,
                      height_ratios=[7, 2], hspace=0.55,
                      left=0.09, right=0.88, top=0.95, bottom=0.06)

        # -- Electrode heatmap --
        self._heatmap_ax = self._heatmap_fig.add_subplot(gs[0, 0])
        self._heatmap_ax.set_facecolor(BG_MID)
        self._heatmap_ax.tick_params(colors=FG_DIM, labelsize=8)
        for sp in self._heatmap_ax.spines.values():
            sp.set_color(BG_LIGHT)

        # 10x10 masked array: positions [pr+1, pc+1] for inner 8x8 electrodes
        _hm_data = np.zeros((10, 10))
        _hm_mask = np.ones((10, 10), dtype=bool)
        for _pr in range(8):
            for _pc in range(8):
                if ELEC_GRID.get((_pc, _pr)) is not None:
                    _hm_mask[_pr + 1, _pc + 1] = False
        self._heatmap_mask = _hm_mask
        try:
            _cmap_obj = matplotlib.colormaps[self._cmap_var.get()].copy()
        except (AttributeError, KeyError):
            _cmap_obj = matplotlib.cm.get_cmap(self._cmap_var.get()).copy()
        _cmap_obj.set_bad('#0d0d14')
        heat_masked = np.ma.array(_hm_data, mask=_hm_mask)
        self._heatmap_im = self._heatmap_ax.imshow(
            heat_masked, cmap=_cmap_obj,
            origin='upper', aspect='equal', vmin=0, vmax=1,
            interpolation='nearest')
        # Tick labels map array indices 1-8 to electrode cols/rows 0-7
        self._heatmap_ax.set_xticks([1, 2, 3, 4, 5, 6, 7, 8])
        self._heatmap_ax.set_yticks([1, 2, 3, 4, 5, 6, 7, 8])
        self._heatmap_ax.set_xticklabels([str(c) for c in range(8)],
                                          color=FG_DIM, fontsize=7)
        self._heatmap_ax.set_yticklabels([str(r) for r in range(8)],
                                          color=FG_DIM, fontsize=7)
        self._heatmap_ax.set_xlabel("Column", color=FG_DIM, fontsize=8)
        self._heatmap_ax.set_ylabel("Row",    color=FG_DIM, fontsize=8)
        self._heatmap_ax.set_title("Electrode Spatial Map  (outer ring = STIM)",
                                    color=FG_MAIN, fontsize=9)

        self._heatmap_cbar = self._heatmap_fig.colorbar(
            self._heatmap_im, ax=self._heatmap_ax)
        self._heatmap_cbar.ax.tick_params(labelcolor=FG_DIM, labelsize=7)
        self._heatmap_cbar.set_label("uV", color=FG_DIM, fontsize=8)

        # -- STIM bar chart --
        self._heatmap_stim_ax = self._heatmap_fig.add_subplot(gs[1, 0])
        self._heatmap_stim_ax.set_facecolor(STIM_BG)
        for sp in self._heatmap_stim_ax.spines.values():
            sp.set_color(ORANGE); sp.set_linewidth(0.7)
        self._heatmap_stim_bars = self._heatmap_stim_ax.bar(
            range(8), np.zeros(8), color=ORANGE, alpha=0.85, edgecolor='none')
        stim_labels = [name for name, _ in STIM_BY_NAME]
        self._heatmap_stim_ax.set_xticks(range(8))
        self._heatmap_stim_ax.set_xticklabels(
            stim_labels, fontsize=6, color=ORANGE, rotation=30, ha='right')
        self._heatmap_stim_ax.tick_params(colors=ORANGE, labelsize=6)
        self._heatmap_stim_ax.set_ylabel("uA", color=ORANGE, fontsize=7)
        self._heatmap_stim_ax.set_title("STIM Programmed Output (uA)",
                                         color=ORANGE, fontsize=8, pad=2)

        self._heatmap_canvas = FigureCanvasTkAgg(self._heatmap_fig, master=parent)
        self._heatmap_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._start_heatmap_loop()

    # ---- STIM Waveform Designer tab --------------------------------------

    def _build_stim_waveform_tab(self, parent):
        left  = tk.Frame(parent, bg=BG_DARK, width=310)
        left.pack_propagate(False)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=6, pady=6)

        right = tk.Frame(parent, bg=BG_DARK)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=6, padx=(0, 6))

        # ---- scrollable left panel ----
        lcanvas = tk.Canvas(left, bg=BG_DARK, highlightthickness=0)
        lsb     = ttk.Scrollbar(left, orient='vertical', command=lcanvas.yview)
        lcanvas.configure(yscrollcommand=lsb.set)
        lsb.pack(side=tk.RIGHT, fill=tk.Y)
        lcanvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        linner = tk.Frame(lcanvas, bg=BG_DARK)
        _lwin  = lcanvas.create_window((0, 0), window=linner, anchor='nw')
        linner.bind('<Configure>',
                    lambda e: lcanvas.configure(scrollregion=lcanvas.bbox('all')))
        lcanvas.bind('<Configure>',
                     lambda e: lcanvas.itemconfig(_lwin, width=e.width))

        # ---- Visual channel selector (replaces dropdown) ----
        sf = ttk.LabelFrame(linner, text="Channel Summary  —  click to select & edit", padding=6)
        sf.pack(fill=tk.X, padx=4, pady=4)
        for name, flat in STIM_BY_NAME:
            btn = tk.Button(
                sf, text=f"{name}  Off", font=('Consolas', 8),
                bg='#1a1a2a', fg=FG_DIM, activebackground=ORANGE,
                activeforeground=BG_DARK, relief=tk.FLAT, anchor=tk.W,
                padx=6, pady=3,
                command=lambda f=flat, n=name: self._stim_ch_select_by_flat(f, n))
            btn.pack(fill=tk.X, pady=1)
            self._stim_ch_btns[flat]        = btn
            self._stim_summary_labels[flat] = btn   # backward-compat alias

        # Highlight the initially-selected channel
        self._stim_update_ch_summary()

        # ---- Channel editor ----
        ef = ttk.LabelFrame(linner, text="Channel Editor", padding=6)
        ef.pack(fill=tk.X, padx=4, pady=4)

        # Small read-only label showing which channel is being edited
        self._stim_editing_lbl = tk.Label(ef,
            text=f"Editing: {self._stim_edit_ch_var.get()}",
            bg=BG_DARK, fg=ORANGE, font=('Consolas', 8, 'bold'))
        self._stim_editing_lbl.pack(anchor=tk.W, pady=(0, 4))

        def _erow(parent, label, var, widget_type='entry', values=None, width=10):
            r = tk.Frame(parent, bg=BG_DARK)
            r.pack(fill=tk.X, pady=2)
            tk.Label(r, text=label, bg=BG_DARK, fg=FG_DIM,
                     width=15, anchor=tk.W).pack(side=tk.LEFT)
            if widget_type == 'combo':
                w = ttk.Combobox(r, textvariable=var, values=values,
                                 width=width, state='readonly')
            else:
                w = tk.Entry(r, textvariable=var, width=width,
                             bg=BG_LIGHT, fg=FG_MAIN,
                             insertbackground=FG_MAIN, relief=tk.FLAT)
            w.pack(side=tk.LEFT, padx=2)
            return w

        self._stim_type_cb = _erow(ef, "Waveform type:", self._stim_edit_type, 'combo',
                                    ["Off", "DC", "Square Wave",
                                     "Pulse Train", "Custom CSV"], 12)
        self._stim_type_cb.bind('<<ComboboxSelected>>', self._stim_on_type_change)

        _amp_cb = _erow(ef, "Amplitude (uA):", self._stim_edit_amp, 'combo',
                        [str(v) for v in IDAC_CURRENTS_UA], 8)
        _amp_cb.bind('<<ComboboxSelected>>', lambda e: self._stim_update_preview())

        self._stim_freq_row = _erow(ef, "Frequency (Hz):", self._stim_edit_freq)
        self._stim_duty_row = _erow(ef, "Duty cycle (%):", self._stim_edit_duty)
        self._stim_dur_row  = _erow(ef, "Duration (s):",   self._stim_edit_dur)
        self._stim_pw_row   = _erow(ef, "Pulse width (ms):", self._stim_edit_pw)
        self._stim_ips_row  = _erow(ef, "Inter-pulse (ms):", self._stim_edit_ips)
        self._stim_cnt_row  = _erow(ef, "Pulse count:",      self._stim_edit_count)

        # "Continuous" checkbox — shown only for Pulse Train, hides count field
        _cont_f = tk.Frame(ef, bg=BG_DARK)
        _cont_f.pack(fill=tk.X, pady=2)
        tk.Checkbutton(
            _cont_f, text="Repeat continuously  (ignore count)",
            variable=self._stim_edit_continuous,
            bg=BG_DARK, fg=FG_DIM, selectcolor=BG_LIGHT,
            activebackground=BG_DARK, activeforeground=FG_MAIN,
            command=self._stim_on_continuous_toggle,
        ).pack(side=tk.LEFT, padx=2)
        self._stim_cont_row = _cont_f

        csv_r = tk.Frame(ef, bg=BG_DARK)
        csv_r.pack(fill=tk.X, pady=2)
        tk.Label(csv_r, text="CSV file:", bg=BG_DARK, fg=FG_DIM,
                 width=15, anchor=tk.W).pack(side=tk.LEFT)
        self._stim_csv_lbl = tk.Label(csv_r, textvariable=self._stim_edit_csv_path,
                                       bg=BG_DARK, fg=ACCENT, font=('Consolas', 7),
                                       wraplength=140, justify=tk.LEFT)
        self._stim_csv_lbl.pack(side=tk.LEFT, padx=2)

        # Auto-update preview whenever any entry var changes
        for _v in (self._stim_edit_freq, self._stim_edit_duty, self._stim_edit_dur,
                   self._stim_edit_pw, self._stim_edit_ips, self._stim_edit_count):
            _v.trace_add('write', lambda *_: self.root.after_idle(self._stim_update_preview))

        self._stim_on_type_change()

        ab = tk.Frame(linner, bg=BG_DARK)
        ab.pack(fill=tk.X, padx=4, pady=2)
        ttk.Button(ab, text="Apply to this channel",
                   command=self._stim_apply_to_ch).pack(fill=tk.X, pady=2)
        ttk.Button(ab, text="Copy config to all channels",
                   command=self._stim_copy_to_all).pack(fill=tk.X, pady=2)

        # ---- Action buttons ----
        bf = tk.Frame(linner, bg=BG_DARK)
        bf.pack(fill=tk.X, padx=4, pady=4)
        ttk.Button(bf, text="Import CSV for this channel",
                   command=self._stim_import_csv).pack(fill=tk.X, pady=2)
        tk.Button(bf, text="Upload All Channels to Pico",
                  bg=ACCENT, fg=BG_DARK, activebackground='#6ea5f0',
                  activeforeground=BG_DARK, font=('Helvetica', 9, 'bold'),
                  pady=6, relief=tk.FLAT,
                  command=self._stim_upload_all).pack(fill=tk.X, pady=(6, 2))
        tk.Label(bf, textvariable=self._stim_upload_status,
                 bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 8),
                 wraplength=260, justify=tk.LEFT).pack(anchor=tk.W)

        # ---- Right panel: view toggle + preview figure ----
        vtb = tk.Frame(right, bg=BG_MID, pady=3)
        vtb.pack(fill=tk.X)
        tk.Label(vtb, text="View:", bg=BG_MID, fg=FG_MAIN).pack(side=tk.LEFT, padx=6)
        for label, val in [("Single Channel", 'single'), ("All 8 Side-by-Side", 'all8')]:
            tk.Radiobutton(
                vtb, text=label, variable=self._stim_view_mode, value=val,
                bg=BG_MID, fg=FG_MAIN, selectcolor=BG_LIGHT,
                activebackground=BG_MID, activeforeground=ORANGE,
                command=self._stim_rebuild_figure
            ).pack(side=tk.LEFT, padx=4)

        self._stim_right = right
        self._stim_fig    = None
        self._stim_ax     = None
        self._stim_canvas = None
        self._stim_rebuild_figure()
        self.root.after(150, self._stim_update_preview)

    def _stim_on_type_change(self, event=None):
        wtype = self._stim_edit_type.get()
        # Show/hide rows based on waveform type
        for widget in (self._stim_freq_row, self._stim_duty_row, self._stim_dur_row,
                       self._stim_pw_row, self._stim_ips_row, self._stim_cnt_row,
                       self._stim_csv_lbl):
            widget.master.pack_forget()
        # Continuous checkbox only relevant for Pulse Train
        if hasattr(self, '_stim_cont_row'):
            self._stim_cont_row.pack_forget()

        if wtype in ('Square Wave',):
            self._stim_freq_row.master.pack(fill=tk.X, pady=2)
            self._stim_duty_row.master.pack(fill=tk.X, pady=2)
            self._stim_dur_row.master.pack(fill=tk.X, pady=2)
        elif wtype == 'Pulse Train':
            self._stim_pw_row.master.pack(fill=tk.X, pady=2)
            self._stim_ips_row.master.pack(fill=tk.X, pady=2)
            if hasattr(self, '_stim_cont_row'):
                self._stim_cont_row.pack(fill=tk.X, pady=2)
            # Only show count field when not in continuous mode
            if not self._stim_edit_continuous.get():
                self._stim_cnt_row.master.pack(fill=tk.X, pady=2)
        elif wtype == 'Custom CSV':
            self._stim_csv_lbl.master.pack(fill=tk.X, pady=2)
        # DC and Off: no extra params
        self._stim_update_preview()

    def _stim_on_continuous_toggle(self):
        """Show/hide the pulse count row based on the Continuous checkbox."""
        if self._stim_edit_continuous.get():
            self._stim_cnt_row.master.pack_forget()
        else:
            self._stim_cnt_row.master.pack(fill=tk.X, pady=2)
        self._stim_update_preview()

    def _stim_ch_select_by_flat(self, flat, name):
        """Called when user clicks a channel button in the visual selector."""
        self._stim_edit_ch_var.set(name)
        self._stim_ch_select()
        self._stim_update_ch_summary()   # refresh button highlights
        if hasattr(self, '_stim_editing_lbl') and self._stim_editing_lbl:
            self._stim_editing_lbl.config(text=f"Editing: {name}")

    def _stim_rebuild_figure(self):
        """Destroy and recreate the preview figure when view mode changes."""
        if self._stim_canvas:
            self._stim_canvas.get_tk_widget().destroy()
            self._stim_canvas = None
        self._stim_fig = None
        self._stim_ax  = None

        mode = self._stim_view_mode.get()
        if mode == 'single':
            self._stim_fig = Figure(figsize=(7, 4.5), dpi=96, facecolor=BG_MID)
            self._stim_ax  = self._stim_fig.add_subplot(1, 1, 1)
            self._stim_ax.set_facecolor(BG_MID)
            self._stim_ax.tick_params(colors=FG_DIM, labelsize=8)
            for sp in self._stim_ax.spines.values():
                sp.set_color(BG_LIGHT)
            self._stim_ax.set_xlabel("Time (s)", color=FG_DIM, fontsize=9)
            self._stim_ax.set_ylabel("Current (uA)", color=FG_DIM, fontsize=9)
            self._stim_fig.tight_layout(pad=1.8)
        else:
            # 2 rows x 4 cols for all 8 channels
            self._stim_fig = Figure(figsize=(9, 5), dpi=88, facecolor=BG_MID)
            self._stim_ax  = None   # individual axes stored in self._stim_all_axes

        self._stim_canvas = FigureCanvasTkAgg(self._stim_fig, master=self._stim_right)
        self._stim_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._stim_update_preview()

    def _stim_ch_select(self, event=None):
        """Load saved config for the selected channel into editor vars."""
        name = self._stim_edit_ch_var.get()
        flat = next((fi for n, fi in STIM_BY_NAME if n == name), None)
        if flat is None:
            return
        cfg = self._stim_ch_configs[flat]
        self._stim_edit_type.set(cfg['type'])
        self._stim_edit_amp.set(cfg['amp'])
        self._stim_edit_freq.set(cfg['freq'])
        self._stim_edit_duty.set(cfg['duty'])
        self._stim_edit_dur.set(cfg['duration'])
        self._stim_edit_pw.set(cfg['pw_ms'])
        self._stim_edit_ips.set(cfg['ips_ms'])
        self._stim_edit_count.set(cfg['count'])
        self._stim_edit_continuous.set(cfg.get('continuous', False))
        self._stim_edit_csv_path.set(cfg['csv_path'])
        self._stim_edit_csv_data = cfg['csv_data']
        self._stim_on_type_change()

    def _stim_apply_to_ch(self):
        """Save editor state into the selected channel's config."""
        name = self._stim_edit_ch_var.get()
        flat = next((fi for n, fi in STIM_BY_NAME if n == name), None)
        if flat is None:
            return
        self._stim_ch_configs[flat].update({
            'type':       self._stim_edit_type.get(),
            'amp':        self._stim_edit_amp.get(),
            'freq':       self._stim_edit_freq.get(),
            'duty':       self._stim_edit_duty.get(),
            'duration':   self._stim_edit_dur.get(),
            'pw_ms':      self._stim_edit_pw.get(),
            'ips_ms':     self._stim_edit_ips.get(),
            'count':      self._stim_edit_count.get(),
            'continuous': self._stim_edit_continuous.get(),
            'csv_path':   self._stim_edit_csv_path.get(),
            'csv_data':   self._stim_edit_csv_data,
        })
        self._stim_update_ch_summary()
        self._stim_upload_status.set(f"Config saved for {name}")

    def _stim_copy_to_all(self):
        """Copy editor state to every channel's config."""
        cfg = {
            'type':       self._stim_edit_type.get(),
            'amp':        self._stim_edit_amp.get(),
            'freq':       self._stim_edit_freq.get(),
            'duty':       self._stim_edit_duty.get(),
            'duration':   self._stim_edit_dur.get(),
            'pw_ms':      self._stim_edit_pw.get(),
            'ips_ms':     self._stim_edit_ips.get(),
            'count':      self._stim_edit_count.get(),
            'continuous': self._stim_edit_continuous.get(),
            'csv_path':   self._stim_edit_csv_path.get(),
            'csv_data':   self._stim_edit_csv_data,
        }
        for _, flat in STIM_BY_NAME:
            self._stim_ch_configs[flat].update(cfg)
        self._stim_update_ch_summary()
        self._stim_upload_status.set("Config copied to all 8 channels")

    def _stim_update_ch_summary(self):
        selected_name = self._stim_edit_ch_var.get()
        for name, flat in STIM_BY_NAME:
            cfg = self._stim_ch_configs[flat]
            wt  = cfg['type']
            amp = cfg['amp']
            if wt == 'Off':
                summary = f"{name}: Off"
            elif wt == 'DC':
                summary = f"{name}: DC  {amp} uA"
            elif wt == 'Square Wave':
                summary = f"{name}: Sq  {amp} uA  {cfg['freq']} Hz"
            elif wt == 'Pulse Train':
                cnt_str = "cont" if cfg.get('continuous') else f"{cfg['count']}x"
                summary = f"{name}: Pls {amp} uA  {cnt_str}"
            elif wt == 'Custom CSV':
                summary = f"{name}: CSV {os.path.basename(cfg['csv_path'])}"
            else:
                summary = f"{name}: {wt}"
            btn = self._stim_ch_btns.get(flat)
            if btn:
                is_sel = (name == selected_name)
                if is_sel:
                    bg_col = BG_LIGHT        # highlighted row
                    fg_col = FG_MAIN
                elif wt != 'Off':
                    bg_col = '#1e1a2e'       # slight warm tint for active
                    fg_col = ORANGE
                else:
                    bg_col = '#1a1a2a'
                    fg_col = FG_DIM
                btn.config(text=summary, fg=fg_col, bg=bg_col)

    def _stim_waveform_for_display(self, flat, n_samp, fps, win_secs):
        """Return (t, y) of the *programmed* STIM waveform for one channel.

        t runs from -win_secs to 0 (same x-axis as sense channels).
        y is in µA as commanded to the IDAC.  No ADC data is used.
        """
        cfg = self._stim_ch_configs.get(flat)
        t   = np.linspace(-win_secs, 0, n_samp)
        if cfg is None:
            return t, np.zeros(n_samp)

        wtype = cfg['type']
        try:    amp = float(cfg['amp'])
        except: amp = 0.0

        if wtype == 'Off':
            return t, np.zeros(n_samp)
        elif wtype == 'DC':
            return t, np.full(n_samp, amp)
        elif wtype == 'Square Wave':
            try:    freq = max(0.001, float(cfg['freq']))
            except: freq = 1.0
            try:    duty = max(1.0, min(99.0, float(cfg['duty']))) / 100.0
            except: duty = 0.5
            t_abs = np.linspace(0, win_secs, n_samp)
            y = np.where((t_abs * freq) % 1.0 < duty, amp, 0.0)
            return t, y
        elif wtype == 'Pulse Train':
            try:    pw  = float(cfg['pw_ms'])  / 1000.0
            except: pw  = 0.01
            try:    ips = float(cfg['ips_ms']) / 1000.0
            except: ips = 0.09
            period = max(pw + ips, 1e-6)
            t_abs  = np.linspace(0, win_secs, n_samp)
            y      = np.zeros(n_samp)
            if cfg.get('continuous'):
                # Tile the pulse indefinitely across the window
                phase = t_abs % period
                y[phase < pw] = amp
            else:
                try:    cnt = max(1, int(cfg['count']))
                except: cnt = 10
                for i in range(cnt):
                    t0 = i * period
                    mask = (t_abs >= t0) & (t_abs < t0 + pw)
                    y[mask] = amp
            return t, y
        elif wtype == 'Custom CSV':
            data = cfg.get('csv_data')
            if data is not None and len(data) >= 2:
                repeats = (n_samp // len(data)) + 2
                tiled   = np.tile(data, repeats)[:n_samp].astype(float)
                peak    = float(np.abs(data).max())
                if peak > 0:
                    tiled = tiled / peak * amp
                return t, tiled
        return t, np.zeros(n_samp)

    def _stim_generate_waveform(self):
        """Generate preview waveform from the current editor vars (not from a channel config)."""
        wtype = self._stim_edit_type.get()
        try:    amp = float(self._stim_edit_amp.get())
        except: amp = 0.0

        rate = 1000.0   # internal preview sample rate (Hz)

        if wtype == 'Off':
            t = np.linspace(0, 1.0, 200)
            return t, np.zeros(200)
        elif wtype == 'DC':
            t = np.linspace(0, 1.0, 200)
            return t, np.full(200, amp)
        elif wtype == 'Custom CSV':
            data = self._stim_edit_csv_data
            if data is None:
                return np.array([0.0, 1.0]), np.array([0.0, 0.0])
            t = np.arange(len(data)) / rate
            peak = float(np.abs(data).max())
            y    = data.copy().astype(float)
            if peak > 0: y = y / peak * amp
            return t, y

        try:    dur  = max(0.001, float(self._stim_edit_dur.get()))
        except: dur  = 1.0
        n = max(2, int(rate * dur))
        t = np.linspace(0, dur, n, endpoint=False)

        if wtype == 'Square Wave':
            try:    freq = max(0.001, float(self._stim_edit_freq.get()))
            except: freq = 1.0
            try:    duty = max(1.0, min(99.0, float(self._stim_edit_duty.get()))) / 100.0
            except: duty = 0.5
            y = np.where((t * freq) % 1.0 < duty, amp, 0.0).astype(float)
        elif wtype == 'Pulse Train':
            try:    pw  = float(self._stim_edit_pw.get())  / 1000.0
            except: pw  = 0.01
            try:    ips = float(self._stim_edit_ips.get()) / 1000.0
            except: ips = 0.09
            period = max(pw + ips, 1e-6)
            y      = np.zeros(n)
            if self._stim_edit_continuous.get():
                phase = t % period
                y[phase < pw] = amp
            else:
                try:    cnt = max(1, int(self._stim_edit_count.get()))
                except: cnt = 10
                for i in range(cnt):
                    t0   = i * period
                    mask = (t >= t0) & (t < t0 + pw)
                    y[mask] = amp
        else:
            y = np.zeros(n)
        return t, y

    def _stim_update_preview(self):
        if self._stim_fig is None:
            return
        if self._stim_view_mode.get() == 'all8':
            self._stim_update_preview_all8()
        else:
            self._stim_update_preview_single()

    def _stim_update_preview_single(self):
        """Draw the current editor waveform into the single-channel axes."""
        if self._stim_ax is None:
            return
        t, y = self._stim_generate_waveform()
        wtype = self._stim_edit_type.get()
        ch    = self._stim_edit_ch_var.get()
        self._stim_ax.cla()
        self._stim_ax.set_facecolor(BG_MID)
        self._stim_ax.tick_params(colors=FG_DIM, labelsize=8)
        for sp in self._stim_ax.spines.values():
            sp.set_color(BG_LIGHT)
        self._stim_ax.plot(t, y, color=ORANGE, linewidth=1.5)
        self._stim_ax.axhline(0, color=FG_DIMMER, linewidth=0.5, linestyle='--')
        self._stim_ax.set_xlabel("Time (s)",      color=FG_DIM, fontsize=9)
        self._stim_ax.set_ylabel("Current (uA)", color=FG_DIM, fontsize=9)
        self._stim_ax.set_title(f"{ch}  —  {wtype} Preview  ({len(y)} samples)",
                                 color=FG_MAIN, fontsize=9)
        self._stim_fig.tight_layout(pad=1.8)
        if self._stim_canvas:
            self._stim_canvas.draw_idle()

    def _stim_update_preview_all8(self):
        """Draw all 8 channels side-by-side in a 2x4 GridSpec."""
        if self._stim_fig is None or self._stim_canvas is None:
            return
        self._stim_fig.clear()
        gs = GridSpec(2, 4, figure=self._stim_fig,
                      hspace=0.55, wspace=0.40,
                      left=0.08, right=0.97, top=0.93, bottom=0.12)
        fps_est  = ASSUMED_FPS
        win_secs = 2.0
        n_samp   = max(2, int(fps_est * win_secs))
        for i, (name, flat) in enumerate(STIM_BY_NAME):
            ax = self._stim_fig.add_subplot(gs[i // 4, i % 4])
            t, y = self._stim_waveform_for_display(flat, n_samp, fps_est, win_secs)
            cfg    = self._stim_ch_configs.get(flat, {})
            active = cfg.get('type', 'Off') != 'Off'
            color  = ORANGE if active else FG_DIMMER
            ax.plot(t, y, color=color, linewidth=1.0)
            ax.set_facecolor(BG_MID)
            ax.set_title(name, color=FG_MAIN if active else FG_DIM,
                         fontsize=7, pad=2)
            ax.tick_params(colors=FG_DIMMER, labelsize=5)
            ax.set_xlabel("s",  color=FG_DIMMER, fontsize=6)
            ax.set_ylabel("uA", color=FG_DIMMER, fontsize=6)
            for sp in ax.spines.values():
                sp.set_color(BG_LIGHT)
            if not active:
                ax.set_ylim(-1, 1)
        self._stim_canvas.draw_idle()

    def _stim_import_csv(self):
        path = filedialog.askopenfilename(
            title="Import STIM waveform CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            data = []
            with open(path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    val = float(line.split(',')[0])
                    data.append(val)
            if len(data) < 2:
                self._stim_upload_status.set("Error: CSV must have >= 2 values")
                return
            self._stim_edit_csv_data = np.array(data, dtype=float)
            self._stim_edit_csv_path.set(os.path.basename(path))
            self._stim_edit_type.set("Custom CSV")
            self._stim_on_type_change()
            self._stim_update_preview()
            self._stim_upload_status.set(
                f"Loaded {len(data)} samples — click Apply to save to channel")
        except Exception as e:
            self._stim_upload_status.set(f"Error reading CSV: {e}")

    def _stim_upload_all(self):
        if not self._cmd_client or not self._cmd_client.connected:
            self._stim_upload_status.set("Error: not connected to Pico")
            return
        stim_idx_map = {flat: i for i, (_, flat) in enumerate(STIM_BY_NAME)}
        uploaded = []
        for name, flat in STIM_BY_NAME:
            cfg  = self._stim_ch_configs[flat]
            ch_i = stim_idx_map[flat]
            wtype = cfg['type']
            amp   = cfg['amp']
            if wtype == 'Off':
                self._send_cmd(f"stim_stop {ch_i}")
                continue
            elif wtype == 'DC':
                self._send_cmd(f"stim_dc {ch_i} {amp}")
            elif wtype == 'Square Wave':
                self._send_cmd(
                    f"stim_square {ch_i} {amp} {cfg['freq']} "
                    f"{cfg['duty']} {cfg['duration']}")
            elif wtype == 'Pulse Train':
                count_arg = 0 if cfg.get('continuous') else cfg['count']
                self._send_cmd(
                    f"stim_pulse {ch_i} {amp} {cfg['pw_ms']} "
                    f"{cfg['ips_ms']} {count_arg}")
            elif wtype == 'Custom CSV':
                data = cfg.get('csv_data')
                if data is not None:
                    n    = min(len(data), 8192)
                    vals = ' '.join(f"{v:.3f}" for v in data[:n])
                    self._send_cmd(f"stim_csv {ch_i} {n} {vals}")
                else:
                    continue
            self._send_cmd(f"stim_start {ch_i}")
            uploaded.append(name)
        if uploaded:
            self._stim_upload_status.set(f"Uploaded: {', '.join(uploaded)}")
        else:
            self._stim_upload_status.set("All channels are Off — nothing uploaded")

    # ---- Settings tab ----------------------------------------------------

    def _build_settings_tab(self, parent):
        # Outer scrollable canvas so the tab works at any window height
        outer_canvas = tk.Canvas(parent, bg=BG_DARK, highlightthickness=0)
        _osb = ttk.Scrollbar(parent, orient='vertical', command=outer_canvas.yview)
        outer_canvas.configure(yscrollcommand=_osb.set)
        _osb.pack(side=tk.RIGHT, fill=tk.Y)
        outer_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        pad = tk.Frame(outer_canvas, bg=BG_DARK)
        _owin = outer_canvas.create_window((0, 0), window=pad, anchor='nw')
        pad.bind('<Configure>', lambda e: outer_canvas.configure(
            scrollregion=outer_canvas.bbox('all')))
        outer_canvas.bind('<Configure>',
                          lambda e: outer_canvas.itemconfig(_owin, width=e.width))

        _LW = 26   # label column width (chars) — keeps all entries aligned

        def _field_row(parent_frame, label, var, width=32, label_fg=FG_DIM, label_font=None):
            f = tk.Frame(parent_frame, bg=BG_DARK)
            f.pack(fill=tk.X, pady=2)
            lkw = dict(bg=BG_DARK, fg=label_fg, width=_LW, anchor=tk.W)
            if label_font:
                lkw['font'] = label_font
            tk.Label(f, text=label, **lkw).pack(side=tk.LEFT)
            tk.Entry(f, textvariable=var, width=width,
                     bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                     relief=tk.FLAT).pack(side=tk.LEFT, padx=4)

        def _dir_row(parent_frame, label, var, browse_cmd, open_cmd):
            f = tk.Frame(parent_frame, bg=BG_DARK)
            f.pack(fill=tk.X, pady=2)
            tk.Label(f, text=label, bg=BG_DARK, fg=FG_DIM,
                     width=_LW, anchor=tk.W).pack(side=tk.LEFT)
            tk.Entry(f, textvariable=var, width=32,
                     bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                     relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
            ttk.Button(f, text="Browse", command=browse_cmd).pack(side=tk.LEFT, padx=2)
            ttk.Button(f, text="Open",   command=open_cmd  ).pack(side=tk.LEFT, padx=2)

        # =====================================================================
        # --- Experiment Identity (REQUIRED) ---
        # =====================================================================
        id_frame = ttk.LabelFrame(pad,
            text="Experiment Identity  \u26a0  Complete before recording",
            padding=10)
        id_frame.pack(fill=tk.X, padx=12, pady=(10, 4))

        # Required: experiment name (drives default file prefix)
        _field_row(id_frame, "Experiment name: *",
                   self._exp_name_var, width=32,
                   label_fg=ORANGE, label_font=('Helvetica', 9, 'bold'))
        tk.Label(id_frame,
                 text="  Default file prefix:  {name}_YYYYMMDD_HHMMSS",
                 bg=BG_DARK, fg=FG_DIMMER,
                 font=('Helvetica', 7)).pack(anchor=tk.W, pady=(0, 4))

        _field_row(id_frame, "MushIO Board ID:",        self._board_id_var)
        _field_row(id_frame, "Sample common name:",     self._species_common_var)
        _field_row(id_frame, "Sample scientific name:", self._species_sci_var)

        tk.Label(id_frame, text="Experiment description:",
                 bg=BG_DARK, fg=FG_DIM).pack(anchor=tk.W, pady=(6, 2))
        desc_frame = tk.Frame(id_frame, bg=BG_DARK)
        desc_frame.pack(fill=tk.X, pady=2)
        self._exp_desc_txt = tk.Text(
            desc_frame, height=5, wrap=tk.WORD,
            bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
            font=('Helvetica', 9), relief=tk.FLAT, padx=4, pady=4)
        self._exp_desc_txt.pack(side=tk.LEFT, fill=tk.X, expand=True)
        _desc_sb = ttk.Scrollbar(desc_frame, command=self._exp_desc_txt.yview)
        _desc_sb.pack(side=tk.LEFT, fill=tk.Y)
        self._exp_desc_txt.configure(yscrollcommand=_desc_sb.set)

        tk.Label(id_frame,
                 text="  e.g.  'Mapping mycelium network electrophysiology during light-dark cycle "
                      "transitions in Pleurotus ostreatus over 72 h. Primary goal: identify "
                      "propagating electrical pulses correlated with growth direction.'",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 wraplength=680, justify=tk.LEFT).pack(anchor=tk.W, pady=(2, 0))

        # =====================================================================
        # --- Configuration save / load ---
        # =====================================================================
        cfg_frame = ttk.LabelFrame(pad, text="Full Experiment Configuration  (includes STIM, gains, registers)",
                                    padding=8)
        cfg_frame.pack(fill=tk.X, padx=12, pady=4)

        cfg_btn_row = tk.Frame(cfg_frame, bg=BG_DARK)
        cfg_btn_row.pack(fill=tk.X, pady=4)
        tk.Button(cfg_btn_row, text="  \U0001f4be  Save Config As...",
                  bg='#1a2a3a', fg=ACCENT,
                  activebackground=ACCENT, activeforeground=BG_DARK,
                  font=('Consolas', 9, 'bold'), pady=5, relief=tk.FLAT,
                  command=self._save_config).pack(side=tk.LEFT, padx=2)
        tk.Button(cfg_btn_row, text="  \U0001f4c2  Load Config...",
                  bg='#1a2a3a', fg=ACCENT,
                  activebackground=ACCENT, activeforeground=BG_DARK,
                  font=('Consolas', 9, 'bold'), pady=5, relief=tk.FLAT,
                  command=self._load_config).pack(side=tk.LEFT, padx=8)

        self._cfg_status = tk.StringVar(value='')
        tk.Label(cfg_frame, textvariable=self._cfg_status,
                 bg=BG_DARK, fg=GREEN, font=('Helvetica', 8),
                 wraplength=600, justify=tk.LEFT).pack(anchor=tk.W, pady=2)

        tk.Label(cfg_frame,
                 text="Saves: experiment identity, display settings, bandpass, STIM channel configs, "
                      "all ADC register values, and logging paths.  Load to restore a previous setup instantly.",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7),
                 wraplength=680, justify=tk.LEFT).pack(anchor=tk.W)

        # =====================================================================
        # --- Data logging ---
        # =====================================================================
        log_frame = ttk.LabelFrame(pad, text="Data Logging  (rolling binary .bin files)",
                                    padding=8)
        log_frame.pack(fill=tk.X, padx=12, pady=4)

        _dir_row(log_frame, "Data directory:",
                 self._data_dir_var,
                 self._browse_data_dir,
                 lambda: self._open_dir(self._data_dir_var.get()))

        r2 = tk.Frame(log_frame, bg=BG_DARK)
        r2.pack(fill=tk.X, pady=2)
        tk.Label(r2, text="File prefix:", bg=BG_DARK, fg=FG_DIM,
                 width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        tk.Entry(r2, textvariable=self._log_prefix_var, width=16,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 relief=tk.FLAT).pack(side=tk.LEFT, padx=4)

        r3 = tk.Frame(log_frame, bg=BG_DARK)
        r3.pack(fill=tk.X, pady=2)
        tk.Label(r3, text="Roll period:", bg=BG_DARK, fg=FG_DIM,
                 width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        ttk.Combobox(r3, textvariable=self._log_roll_var,
                     values=LOG_ROLL_LABELS, width=10,
                     state='readonly').pack(side=tk.LEFT, padx=4)
        tk.Label(r3, text="(new file opened at each interval)",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7)).pack(side=tk.LEFT, padx=6)

        tk.Label(log_frame,
                 text="~45 KB/s at 200 FPS  \u00b7  164 MB/hr  \u00b7  3.9 GB/day  \u00b7  27 GB/week",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7)).pack(anchor=tk.W, pady=(2, 6))

        rh5 = tk.Frame(log_frame, bg=BG_DARK)
        rh5.pack(fill=tk.X, pady=2)
        _h5_state = tk.NORMAL if HAS_H5PY else tk.DISABLED
        _h5_txt   = "Also log to HDF5 (.h5) — chunked, gzip-compressed, channel-labelled" \
                    if HAS_H5PY else \
                    "HDF5 logging (pip install h5py to enable)"
        ttk.Checkbutton(rh5, text=_h5_txt,
                        variable=self._hdf5_enabled_var,
                        state=_h5_state).pack(side=tk.LEFT)

        bf = tk.Frame(log_frame, bg=BG_DARK)
        bf.pack(fill=tk.X, pady=4)
        self._rec_btn = tk.Button(
            bf, text="  \u25b6 Start Recording  ",
            bg='#1a3a1a', fg=GREEN,
            activebackground=GREEN, activeforeground=BG_DARK,
            font=('Consolas', 10, 'bold'), pady=6, relief=tk.FLAT,
            command=self._toggle_recording)
        self._rec_btn.pack(side=tk.LEFT, padx=2)

        sf = tk.Frame(log_frame, bg=BG_DARK)
        sf.pack(fill=tk.X, pady=2)
        tk.Label(sf, text="Status:", bg=BG_DARK, fg=FG_DIM,
                 width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        self._rec_status_lbl = tk.Label(sf, textvariable=self._log_status_var,
                                         bg=BG_DARK, fg=FG_DIM, font=('Consolas', 8),
                                         anchor=tk.W)
        self._rec_status_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)

        ff = tk.Frame(log_frame, bg=BG_DARK)
        ff.pack(fill=tk.X, pady=1)
        tk.Label(ff, text="Current file:", bg=BG_DARK, fg=FG_DIM,
                 width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        tk.Label(ff, textvariable=self._log_file_var, bg=BG_DARK, fg=ACCENT,
                 font=('Consolas', 8), anchor=tk.W).pack(side=tk.LEFT)

        # =====================================================================
        # --- Hardware self-test ---
        # =====================================================================
        hw_frame = ttk.LabelFrame(pad, text="Hardware Self-Test", padding=8)
        hw_frame.pack(fill=tk.X, padx=12, pady=4)

        hw_row = tk.Frame(hw_frame, bg=BG_DARK)
        hw_row.pack(fill=tk.X, pady=2)
        ttk.Button(hw_row, text="Run Hardware Test",
                   command=self._run_hardware_test).pack(side=tk.LEFT, padx=2)
        tk.Label(hw_row,
                 text="Sends 'test_hardware' CMD — checks ADC IDs, DRDY lines, ring buffer, WiFi RSSI",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7)).pack(side=tk.LEFT, padx=8)

        hw_st = tk.Frame(hw_frame, bg=BG_DARK)
        hw_st.pack(fill=tk.X, pady=1)
        tk.Label(hw_st, textvariable=self._hw_test_status_var,
                 bg=BG_DARK, fg=FG_DIM, font=('Consolas', 8), anchor=tk.W
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # =====================================================================
        # --- OTA firmware update ---
        # =====================================================================
        ota_frame = ttk.LabelFrame(pad, text="OTA Firmware Update", padding=8)
        ota_frame.pack(fill=tk.X, padx=12, pady=4)

        ota_row = tk.Frame(ota_frame, bg=BG_DARK)
        ota_row.pack(fill=tk.X, pady=2)
        ttk.Button(ota_row, text="OTA Flash\u2026",
                   command=self._ota_flash).pack(side=tk.LEFT, padx=2)
        tk.Label(ota_row,
                 text="Browse for .uf2, reboot Pico into BOOTSEL, copy file automatically",
                 bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7)).pack(side=tk.LEFT, padx=8)

        ota_st = tk.Frame(ota_frame, bg=BG_DARK)
        ota_st.pack(fill=tk.X, pady=1)
        tk.Label(ota_st, textvariable=self._ota_status_var,
                 bg=BG_DARK, fg=FG_DIM, font=('Consolas', 8), anchor=tk.W
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # =====================================================================
        # --- Webcam preview + periodic capture settings ---
        # =====================================================================
        cam_frame = ttk.LabelFrame(pad, text="Webcam  —  Live Preview & Periodic Capture",
                                    padding=8)
        cam_frame.pack(fill=tk.X, padx=12, pady=4)

        if not HAS_CV2 and not HAS_PIL:
            tk.Label(cam_frame,
                     text="pip install opencv-python  to enable webcam features",
                     bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 8)).pack(anchor=tk.W)
        else:
            # Settings row
            cfg_row = tk.Frame(cam_frame, bg=BG_DARK)
            cfg_row.pack(fill=tk.X, pady=2)
            tk.Label(cfg_row, text="Camera idx:", bg=BG_DARK, fg=FG_DIM).pack(side=tk.LEFT)
            ttk.Spinbox(cfg_row, textvariable=self._cam_idx_var,
                        from_=0, to=9, width=3).pack(side=tk.LEFT, padx=4)
            tk.Label(cfg_row, text="  Capture every (min):", bg=BG_DARK, fg=FG_DIM
                     ).pack(side=tk.LEFT, padx=(12, 0))
            ttk.Spinbox(cfg_row, textvariable=self._cam_interval_var,
                        from_=1, to=1440, width=5).pack(side=tk.LEFT, padx=4)

            en_row = tk.Frame(cam_frame, bg=BG_DARK)
            en_row.pack(fill=tk.X, pady=2)
            ttk.Checkbutton(en_row, text="Enable periodic file capture",
                             variable=self._cam_enable_var,
                             command=self._on_cam_enable).pack(side=tk.LEFT)
            ttk.Button(en_row, text="Capture Now",
                       command=self._cam_capture_now).pack(side=tk.LEFT, padx=12)
            tk.Label(en_row, textvariable=self._cam_status_var,
                     bg=BG_DARK, fg=FG_DIM, font=('Helvetica', 7)).pack(side=tk.LEFT)

            _dir_row(cam_frame, "Captures directory:",
                     self._cam_dir_var,
                     self._browse_cam_dir,
                     lambda: self._open_dir(self._cam_dir_var.get()))

            # Preview image
            prev_lbl_row = tk.Frame(cam_frame, bg=BG_DARK)
            prev_lbl_row.pack(fill=tk.X, pady=(6, 2))
            tk.Label(prev_lbl_row, text="Preview:", bg=BG_DARK, fg=FG_DIM).pack(side=tk.LEFT)
            tk.Label(prev_lbl_row, textvariable=self._cam_status_var,
                     bg=BG_DARK, fg=FG_DIMMER, font=('Helvetica', 7)).pack(side=tk.LEFT, padx=8)
            ttk.Button(prev_lbl_row, text="Refresh Preview",
                       command=self._cam_refresh_preview).pack(side=tk.LEFT, padx=8)

            self._cam_preview_label = tk.Label(cam_frame, bg='#0d0d14',
                                                text="No preview yet  —  click Refresh Preview",
                                                fg=FG_DIMMER, font=('Helvetica', 8),
                                                width=48, height=10)
            self._cam_preview_label.pack(anchor=tk.W, pady=4)

        # =====================================================================
        # --- Save defaults ---
        # =====================================================================
        sb = tk.Frame(pad, bg=BG_DARK)
        sb.pack(fill=tk.X, padx=12, pady=8)
        ttk.Button(sb, text="Save Paths as Defaults",
                   command=self._save_settings).pack(side=tk.LEFT, padx=4)
        tk.Label(sb, textvariable=self._settings_status,
                 bg=BG_DARK, fg=GREEN, font=('Helvetica', 8)).pack(side=tk.LEFT, padx=8)

    # ---- Settings helpers ------------------------------------------------

    # ---- Firmware flash helpers ------------------------------------------

    def _fw_browse_dir(self):
        path = filedialog.askdirectory(title="Select firmware folder")
        if path:
            self._fw_fw_dir_var.set(path)

    def _fw_record_ssid(self, ssid):
        """Add ssid to the top of the saved history (dedup, capped at 10).

        Updates the Combobox dropdown if it is already built and persists
        the new list immediately via a lightweight JSON merge-write.
        """
        if not ssid:
            return
        # Deduplicate — remove any older copy and put the new one at the front
        hist = [s for s in self._fw_ssid_history if s != ssid]
        hist.insert(0, ssid)
        self._fw_ssid_history = hist[:10]
        if self._fw_ssid_cb is not None:
            self._fw_ssid_cb['values'] = self._fw_ssid_history
        # Merge-write: read existing file → update ssid_history → write back
        try:
            try:
                with open(SETTINGS_FILE, 'r') as _f:
                    _s = json.load(_f)
            except Exception:
                _s = {}
            _s['ssid_history'] = self._fw_ssid_history
            with open(SETTINGS_FILE, 'w') as _f:
                json.dump(_s, _f, indent=2)
        except Exception:
            pass

    def _fw_deploy(self):
        """Deploy firmware to the Pico.
        Mode 'c_demo' → cmake build with MUSHIO_DEMO=1 (synthetic sine waves).
        Mode 'c_real' → cmake build with MUSHIO_DEMO=0 (real ADS124S08 SPI).
        """
        if self._fw_mode.get() in ('c_demo', 'c_real'):
            self._fw_deploy_c()
            return

        fw_dir = self._fw_fw_dir_var.get().strip()
        ssid   = self._fw_wifi_ssid.get().strip()
        pwd    = self._fw_wifi_pass.get()
        host   = self._fw_host_ip.get().strip()
        dport  = self._fw_data_port.get().strip()
        cport  = self._fw_cmd_port.get().strip()
        stub   = (self._fw_mode.get() == 'py_demo')
        port   = self._fw_pico_port.get().strip()

        errors = []
        if not fw_dir or not os.path.isdir(fw_dir):
            errors.append(f"Firmware folder not found: {fw_dir!r}")
        if not ssid:
            errors.append("WiFi SSID is required.")
        if not host:
            errors.append("Host PC IP is required.")
        if errors:
            self._fw_status.set("  \u26a0  " + "  |  ".join(errors))
            self._fw_status_lbl.config(fg=RED)
            return

        self._fw_status.set("Deploying\u2026  see terminal for progress.")
        self._fw_status_lbl.config(fg=ORANGE)
        self._fw_record_ssid(ssid)

        import threading as _thr
        t = _thr.Thread(target=self._fw_deploy_worker,
                        args=(fw_dir, ssid, pwd, host, dport, cport, stub, port),
                        daemon=True)
        t.start()

    def _fw_deploy_worker(self, fw_dir, ssid, pwd, host, dport, cport, stub, port):
        """Background thread: patch config + mpremote-copy all .py files + reboot."""
        import subprocess
        import shutil
        import tempfile

        def _log(msg):
            self._cmd_rx_q.put(f"[FW] {msg}")
            self.root.after(0, lambda m=msg: self._fw_log_append(f"[FW] {m}\n"))

        def _set_status(msg, colour):
            self.root.after(0, lambda m=msg, c=colour: (
                self._fw_status.set(m),
                self._fw_status_lbl.config(fg=c)))

        # 1. Locate mpremote
        mpremote = shutil.which('mpremote')
        if not mpremote:
            _log("ERROR: mpremote not found.")
            _log("       Install with:  pip install mpremote")
            _set_status("mpremote not found — run: pip install mpremote", RED)
            return

        # 2. Auto-detect Pico port if not set manually
        if not port:
            _log("Scanning USB for Pico 2 W  (VID 0x2e8a)...")
            try:
                r = subprocess.run([mpremote, 'connect', 'list'],
                                   capture_output=True, text=True, timeout=10)
                for ln in r.stdout.splitlines():
                    parts = ln.split()
                    if len(parts) >= 3 and '2e8a' in parts[2].lower():
                        port = parts[0]
                        break
            except Exception as e:
                _log(f"Port scan error: {e}")
            if not port:
                _log("ERROR: No Pico 2 W found on USB.")
                _log("       Check the USB cable and that MicroPython is installed.")
                _set_status("Pico not found — check USB cable.", RED)
                return
            _log(f"Found Pico at {port}")
            self.root.after(0, lambda p=port: self._fw_pico_port.set(p))
        else:
            _log(f"Using port: {port}")

        mode_label = "Pico Demo (synthetic data)" if stub else "Real (ADC hardware)"
        _log(f"Mode: {mode_label}")
        _log(f"Folder: {fw_dir}")
        _log(f"WiFi: {ssid}  Host: {host}:{dport}")
        _log("-" * 48)

        # Validate firmware folder — required files must be present
        _req_files = (['main_test.py', 'streamer.py', 'cmd_server.py',
                       'demo_adc_manager.py'] if stub else
                      ['main.py', 'streamer.py', 'cmd_server.py',
                       'ads124s08.py', 'adc_manager.py'])
        _missing = [f for f in _req_files
                    if not os.path.isfile(os.path.join(fw_dir, f))]
        if _missing:
            # Try fw_dir/firmware/ subfolder automatically
            _sub = os.path.join(fw_dir, 'firmware')
            _missing_sub = [f for f in _req_files
                            if not os.path.isfile(os.path.join(_sub, f))]
            if not _missing_sub:
                fw_dir = _sub
                _log(f"Auto-corrected folder to: {fw_dir}")
                self.root.after(0, lambda d=fw_dir: self._fw_folder_var.set(d))
            else:
                _log(f"ERROR: Wrong firmware folder.")
                _log(f"       Missing: {', '.join(_missing)}")
                _log(f"       Point 'Firmware folder' to the firmware/ subdirectory")
                _log(f"       (e.g. C:/Users/filip/OneDrive/Desktop/MushIV1p0/firmware)")
                _set_status("Wrong firmware folder — see Deploy Log for details.", RED)
                return

        _log("DO NOT unplug or touch the Pico during deploy.")
        _log("Estimated time: ~30-60 s (up to ~3 min if retries needed).")
        _log("-" * 48)

        # Stop serial monitor so it releases the COM port before mpremote uses it
        self.root.after(0, self._serial_stop)
        import time as _time

        def _countdown(label, seconds):
            """Sleep for `seconds`, logging a 1-second countdown tick each second."""
            for remaining in range(seconds, 0, -1):
                _log(f"{label}  [{remaining:2d} s remaining]")
                _time.sleep(1)

        _log("Releasing serial port...")
        _countdown("Waiting for serial port to close", 3)

        # ---- Serial escape: break crash loop before mpremote runs ----------
        # If the Pico is crash-looping (e.g. old firmware with bad WDT init),
        # mpremote can't get a clean REPL because USB drops on each reboot.
        # We directly open the serial port, hammer Ctrl+C until we catch the
        # REPL between reboots, then write a minimal sleep-only main.py.
        # Once that is running, mpremote can connect and copy the real files.

        def _serial_escape(escape_port, escape_timeout=30):
            """Open COM port directly, catch REPL, write crash-free main.py.
            Returns True on success, False if pyserial unavailable or timed out."""
            try:
                import serial as _ser_mod
            except ImportError:
                _log("  (pyserial not available — skipping crash-loop escape)")
                return False

            safe_src  = "import time\ntime.sleep(300)\n"
            # Command MicroPython will execute (text written to main.py on Pico)
            write_cmd = (
                f"open('main.py','w').write({safe_src!r})\r\n"
            ).encode()
            soft_reset = b'\x04'   # Ctrl+D

            _log(f"Serial escape: opening {escape_port} to break crash loop...")
            try:
                s = _ser_mod.Serial(escape_port, 115200,
                                    timeout=0.3, write_timeout=2,
                                    dsrdtr=False, rtscts=False)
            except Exception as e:
                _log(f"  Cannot open {escape_port}: {e}")
                return False

            got_repl = False
            try:
                deadline = _time.time() + escape_timeout
                while _time.time() < deadline:
                    s.reset_input_buffer()
                    s.write(b'\x03')        # Ctrl+C — interrupt running program
                    _time.sleep(0.3)
                    buf = s.read(512)
                    if b'>>>' in buf:
                        got_repl = True
                        break

                if not got_repl:
                    _log(f"  No REPL in {escape_timeout} s.")
                    _log("  ACTION: UNPLUG the Pico USB, wait 2 s, replug, then click Deploy again.")
                    _log("  (Clean power cycle lets mpremote catch the boot window reliably.)")
                    return False

                _log("  REPL caught — writing crash-free main.py...")
                s.reset_input_buffer()
                s.write(write_cmd)
                _time.sleep(0.8)
                _log("  Soft-resetting Pico into safe main.py...")
                s.write(soft_reset)
                _time.sleep(2.0)    # wait for Pico to boot into sleep-only main.py
                _log("  Crash loop broken.  mpremote will now connect cleanly.")
                return True
            finally:
                s.close()

        _serial_escape(port, escape_timeout=15)

        # 3. Build a patched config.py in a temp file
        cfg_content = (
            "# Auto-generated by MushIO GUI — do not edit by hand\n"
            f"WIFI_SSID     = {ssid!r}\n"
            f"WIFI_PASSWORD = {pwd!r}\n"
            f"HOST_IP       = {host!r}\n"
            f"HOST_PORT     = {int(dport)}\n"
        )
        # Append everything else from the original config.py (pin maps etc.)
        orig_cfg = os.path.join(fw_dir, 'config.py')
        if os.path.isfile(orig_cfg):
            skip_keys = {'WIFI_SSID', 'WIFI_PASSWORD', 'HOST_IP', 'HOST_PORT'}
            with open(orig_cfg, 'r', encoding='utf-8') as f:
                for ln in f:
                    key = ln.split('=')[0].strip()
                    if key not in skip_keys:
                        cfg_content += ln

        tmp = tempfile.NamedTemporaryFile(
            mode='w', suffix='.py', delete=False, encoding='utf-8')
        tmp.write(cfg_content)
        tmp.close()

        def _build_files():
            """Return list of (src_path, dst_name) for all files to deploy."""
            pairs = [
                (tmp.name,                                    'config.py'),
                (os.path.join(fw_dir, 'streamer.py'),         'streamer.py'),
                (os.path.join(fw_dir, 'cmd_server.py'),       'cmd_server.py'),
            ]
            if stub:
                pairs += [
                    (os.path.join(fw_dir, 'demo_adc_manager.py'), 'demo_adc_manager.py'),
                    (os.path.join(fw_dir, 'main_test.py'),         'main.py'),
                ]
            else:
                pairs += [
                    (os.path.join(fw_dir, 'ads124s08.py'),  'ads124s08.py'),
                    (os.path.join(fw_dir, 'adc_manager.py'),'adc_manager.py'),
                    (os.path.join(fw_dir, 'main.py'),       'main.py'),
                ]
            return pairs

        def _deploy_single_session(files, retries=2):
            """Copy all files in ONE mpremote session then reset.

            Using a single session means mpremote sends Ctrl+C once, gets a
            clean REPL, copies every file without reconnecting, then resets.
            This is much more reliable when the Pico is crash-looping on old
            firmware because there is only one chance to interrupt it.
            """
            # Build: mpremote connect PORT cp a :a + cp b :b + ... + reset
            cmd = [mpremote, 'connect', port]
            queued = []
            for src, dst in files:
                if os.path.isfile(src):
                    cmd += ['cp', src, f':{dst}', '+']
                    queued.append(dst)
                    _log(f"  Queued  {dst}")
                else:
                    _log(f"  SKIP    {dst}  (not found in {fw_dir})")
            cmd += ['reset']

            if not queued:
                _log("No files to copy — check firmware folder.")
                return False

            for attempt in range(1, retries + 1):
                _log(f"Copying {len(queued)} file(s) — single mpremote session "
                     f"(attempt {attempt}/{retries})...")
                try:
                    r = subprocess.run(cmd, capture_output=True, text=True,
                                       timeout=120)
                    if r.returncode == 0:
                        for dst in queued:
                            _log(f"  OK     :{dst}")
                        _log("Reboot command sent.")
                        return True
                    err = (r.stderr.strip() or r.stdout.strip())[:400]
                except subprocess.TimeoutExpired:
                    err = "timed out after 120 s"
                _log(f"  Attempt {attempt} failed: {err}")
                if attempt < retries:
                    _log("Waiting 5 s before retry (Pico may be rebooting)...")
                    _countdown("Retrying", 5)
            return False

        try:
            files = _build_files()
            ok = _deploy_single_session(files)

            _log("-" * 48)

            if not ok:
                _log("DEPLOY FAILED.  Try unplugging and replugging the Pico,")
                _log("then click Deploy again.")
                _set_status("Deploy failed — see Deploy Log.", RED)
                return

            _log("Deploy complete!  Watch for 5 rapid LED blinks on the board.")
            _log("Serial Monitor will connect automatically in ~10 s...")
            _log("TIP: If data shows as 0s — open Serial Monitor and type 'status'")
            _log("     in the CMD terminal to see what the Pico reports.")
            _set_status("Deploy complete — serial monitor connecting in 10 s.", GREEN)

            # Auto-test TCP connection after boot delay.
            # Allow 120 s for Pico to boot, connect WiFi, and reach the server.
            # Initial 12 s delay: Pico reboot (~2 s) + 5-blink (~1 s) +
            # WiFi connect up to 3 retries (~8 s) + margin.
            self._fw_conn_deadline = _time.time() + 120
            self.root.after(12000, self._fw_test_connection)

            # Auto-connect serial monitor after board boots
            self.root.after(10000, self._serial_start)

        except Exception as e:
            _log(f"Deploy error: {e}")
            _set_status(f"Deploy error: {e}", RED)
        finally:
            try:
                os.remove(tmp.name)
            except Exception:
                pass

    # ======================================================================= #
    # C firmware: cmake build + picotool flash                               #
    # ======================================================================= #

    def _fw_deploy_c(self):
        """Validate inputs and launch C firmware build+flash in background thread."""
        fw_dir = self._fw_c_fw_dir_var.get().strip()
        ssid   = self._fw_wifi_ssid.get().strip()
        pwd    = self._fw_wifi_pass.get()
        host   = self._fw_host_ip.get().strip()
        dport  = self._fw_data_port.get().strip()
        cport  = self._fw_cmd_port.get().strip()

        errors = []
        if not fw_dir or not os.path.isdir(fw_dir):
            errors.append(f"C firmware folder not found: {fw_dir!r}")
        if not ssid:
            errors.append("WiFi SSID is required.")
        if not host:
            errors.append("Host PC IP is required.")
        if errors:
            self._fw_status.set("  \u26a0  " + "  |  ".join(errors))
            self._fw_status_lbl.config(fg=RED)
            return

        self._fw_status.set("Building C firmware\u2026  see deploy log.")
        self._fw_status_lbl.config(fg=ORANGE)
        self._fw_record_ssid(ssid)

        import threading as _thr
        t = _thr.Thread(target=self._fw_deploy_c_worker,
                        args=(fw_dir, ssid, pwd, host, dport, cport),
                        daemon=True)
        t.start()

    def _fw_deploy_c_worker(self, fw_dir, ssid, pwd, host, dport, cport):
        """Background thread: patch config.h → cmake → ninja/make → picotool flash."""
        import subprocess
        import shutil
        import re as _re
        import time as _time

        def _log(msg):
            self._cmd_rx_q.put(f"[FW-C] {msg}")
            self.root.after(0, lambda m=msg: self._fw_log_append(f"[FW-C] {m}\n"))

        def _set_status(text, color=None):
            self.root.after(0, lambda t=text, c=color or FG_DIM: (
                self._fw_status.set(t),
                self._fw_status_lbl.config(fg=c)
            ))

        try:
            # ---------------------------------------------------------- #
            # 1. Locate build tools                                        #
            # ---------------------------------------------------------- #
            cmake    = shutil.which('cmake')
            ninja    = shutil.which('ninja')
            make_    = shutil.which('make')
            picotool = shutil.which('picotool')

            if not cmake:
                _log("ERROR: cmake not found.  Install CMake and add it to PATH.")
                _set_status("Build failed — cmake not found.", RED)
                return

            is_demo   = (self._fw_mode.get() == 'c_demo')
            demo_flag = '-DMUSHIO_DEMO=1' if is_demo else '-DMUSHIO_DEMO=0'
            mode_str  = 'demo (synthetic)' if is_demo else 'real (live ADCs)'

            if ninja:
                builder_cmd = [ninja, 'mushio_c']
                cmake_gen   = [cmake, '..', '-DPICO_BOARD=pico2_w', '-G', 'Ninja',
                               demo_flag]
                _log(f"Build system: Ninja  [{mode_str}]")
            elif make_:
                builder_cmd = [make_, '-j4', 'mushio_c']
                cmake_gen   = [cmake, '..', '-DPICO_BOARD=pico2_w', demo_flag]
                _log(f"Build system: make  [{mode_str}]")
            else:
                _log("ERROR: ninja or make not found.  Install Ninja (preferred) or GNU Make.")
                _set_status("Build failed — no build system found.", RED)
                return

            if not picotool:
                _log("WARNING: picotool not found — build will run but flash must be done manually.")

            # ---------------------------------------------------------- #
            # 2. Patch config.h with GUI network settings                 #
            # ---------------------------------------------------------- #
            config_h = os.path.join(fw_dir, 'config.h')
            if not os.path.isfile(config_h):
                _log(f"ERROR: config.h not found in {fw_dir!r}")
                _set_status("Build failed — config.h missing.", RED)
                return

            _log(f"Patching config.h  SSID={ssid!r}  HOST={host}:{dport}")
            with open(config_h, 'r', encoding='utf-8') as _f:
                cfg = _f.read()

            for pat, repl in [
                (r'#define\s+WIFI_SSID\s+.*',     f'#define WIFI_SSID       "{ssid}"'),
                (r'#define\s+WIFI_PASSWORD\s+.*', f'#define WIFI_PASSWORD   "{pwd}"'),
                (r'#define\s+HOST_IP\s+.*',        f'#define HOST_IP         "{host}"'),
                (r'#define\s+HOST_PORT\s+.*',      f'#define HOST_PORT       {int(dport)}'),
            ]:
                cfg = _re.sub(pat, repl, cfg)

            with open(config_h, 'w', encoding='utf-8') as _f:
                _f.write(cfg)

            # ---------------------------------------------------------- #
            # 3. cmake configure  (skipped if CMakeCache already present)  #
            # ---------------------------------------------------------- #
            build_dir = os.path.join(fw_dir, 'build')
            cache_txt = os.path.join(build_dir, 'CMakeCache.txt')

            # Always run cmake so the MUSHIO_DEMO flag is applied immediately
            # when switching between demo and real modes.  cmake is fast on
            # cached builds (< 1 s); only changed source files are recompiled.
            _log(f"cmake configure: {' '.join(cmake_gen)}")
            os.makedirs(build_dir, exist_ok=True)
            r = subprocess.run(cmake_gen, cwd=build_dir,
                               capture_output=True, text=True, timeout=180)
            for ln in (r.stdout + r.stderr).splitlines():
                _log(f"  {ln}")
            if r.returncode != 0:
                _log("cmake configure FAILED.")
                _set_status("Build failed — cmake error (see deploy log).", RED)
                return
            _log("cmake configure OK.")

            # ---------------------------------------------------------- #
            # 4. Build                                                     #
            # ---------------------------------------------------------- #
            _log(f"Building: {' '.join(builder_cmd)} ...")
            r = subprocess.run(builder_cmd, cwd=build_dir,
                               capture_output=True, text=True, timeout=300)
            for ln in (r.stdout + r.stderr).splitlines():
                if ln.strip():
                    _log(f"  {ln}")
            if r.returncode != 0:
                _log("Build FAILED — see compiler errors above.")
                _set_status("Build failed (see deploy log).", RED)
                return

            uf2 = os.path.join(build_dir, 'mushio_c.uf2')
            if not os.path.isfile(uf2):
                _log(f"ERROR: Expected UF2 not found: {uf2}")
                _set_status("Build failed — UF2 missing.", RED)
                return

            kb = os.path.getsize(uf2) // 1024
            _log(f"Build OK.  {kb} KB  →  {uf2}")

            # ---------------------------------------------------------- #
            # 5. Flash                                                     #
            #    Primary:  picotool load --force  (if in PATH)            #
            #    Fallback: detect BOOTSEL USB drive, copy UF2 to it       #
            # ---------------------------------------------------------- #
            import string as _str

            def _find_bootsel_drive():
                """Return drive letter of Pico in BOOTSEL mode, or None."""
                for letter in _str.ascii_uppercase:
                    info = os.path.join(f"{letter}:\\", "INFO_UF2.TXT")
                    try:
                        with open(info, 'r') as _f:
                            txt = _f.read()
                        if "RP2350" in txt or "RPI-RP2" in txt:
                            return f"{letter}:\\"
                    except OSError:
                        pass
                return None

            def _flash_via_copy():
                """Wait up to 30 s for BOOTSEL drive then copy UF2."""
                _log("Waiting for Pico BOOTSEL drive ...")
                _log("  1. Hold BOOTSEL button")
                _log("  2. Plug in USB cable")
                _log("  3. Release BOOTSEL  ← you can let go now")
                _set_status("Waiting for Pico BOOTSEL drive ...", ORANGE)
                deadline = _time.time() + 30
                drive = None
                while _time.time() < deadline:
                    drive = _find_bootsel_drive()
                    if drive:
                        break
                    _time.sleep(1)
                if not drive:
                    _log("ERROR: No Pico BOOTSEL drive found after 30 s.")
                    _log("  → Hold BOOTSEL, plug USB, release BOOTSEL — then retry.")
                    _set_status("Flash failed — BOOTSEL drive not found.", RED)
                    return False
                _log(f"Found BOOTSEL drive: {drive}")
                dest = os.path.join(drive, os.path.basename(uf2))
                try:
                    import shutil as _sh
                    _sh.copy2(uf2, dest)
                    _log(f"Copied UF2 to {dest}")
                    _log("Pico will reboot automatically when copy completes.")
                    return True
                except Exception as _ce:
                    _log(f"ERROR copying UF2: {_ce}")
                    _set_status("Flash failed — copy error.", RED)
                    return False

            flash_method = self._fw_flash_method.get()   # 'auto' | 'usb' | 'ota'

            def _flash_via_ota():
                """Trigger ota_reboot via CMD port then wait for BOOTSEL drive."""
                import sys as _sys
                script = os.path.join(os.path.dirname(__file__), 'ota_client.py')
                host = ''
                if self._cmd_client and self._cmd_client.connected:
                    host = self._cmd_client.host or ''
                if not host and self._receiver:
                    host = self._receiver.client_ip or ''
                if not host:
                    _log("OTA: no Pico IP known — connect CMD port or data stream first.")
                    _set_status("OTA flash failed — no Pico IP.", RED)
                    return False
                cmd = [_sys.executable, script, uf2]
                cmd += ['--host', host]
                _log(f"OTA: running ota_client.py --host {host} ...")
                _set_status(f"OTA flashing via {host} ...", ORANGE)
                try:
                    proc = subprocess.Popen(cmd,
                                            stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT,
                                            text=True, bufsize=1)
                    for line in proc.stdout:
                        ln = line.rstrip()
                        if ln:
                            _log(f"  {ln}")
                    proc.wait()
                    if proc.returncode == 0:
                        _log("OTA flash complete.")
                        return True
                    else:
                        _log(f"OTA flash exited with code {proc.returncode}.")
                        _set_status("OTA flash failed — see log.", RED)
                        return False
                except Exception as _oe:
                    _log(f"OTA error: {_oe}")
                    _set_status("OTA flash failed — see log.", RED)
                    return False

            flashed = False
            if flash_method == 'ota':
                _log("Flash method: OTA (WiFi)")
                flashed = _flash_via_ota()
            elif flash_method == 'usb':
                _log("Flash method: USB (BOOTSEL)")
                flashed = _flash_via_copy()
            else:
                # auto: picotool first, BOOTSEL fallback
                if picotool:
                    _log("Flash method: Auto — attempting picotool load --force ...")
                    r = subprocess.run([picotool, 'load', uf2, '--force'],
                                       capture_output=True, text=True, timeout=60)
                    for ln in (r.stdout + r.stderr).splitlines():
                        if ln.strip():
                            _log(f"  {ln}")
                    if r.returncode == 0:
                        _log("picotool flash OK.  Rebooting ...")
                        subprocess.run([picotool, 'reboot'],
                                       capture_output=True, text=True, timeout=10)
                        flashed = True
                    else:
                        _log("picotool failed — falling back to BOOTSEL drive copy ...")
                        flashed = _flash_via_copy()
                else:
                    _log("Flash method: Auto — picotool not in PATH, using BOOTSEL drive copy ...")
                    flashed = _flash_via_copy()

            if not flashed:
                return

            _log("Flash complete.  Waiting 15 s for WiFi + TCP startup ...")
            _set_status("Flashed OK — waiting for Pico to connect ...", ORANGE)
            self._fw_conn_deadline = _time.time() + 120
            self.root.after(15000, self._fw_test_connection)

        except subprocess.TimeoutExpired as e:
            _log(f"ERROR: Subprocess timeout — {e}")
            _set_status("Build/flash timed out.", RED)
        except Exception as e:
            _log(f"ERROR: {e}")
            _set_status(f"Deploy error: {e}", RED)

    def _fw_test_connection(self):
        """Check whether the Pico has connected to the data receiver.

        The Pico is the TCP CLIENT — it connects to this PC's port 9000.
        So we check self._receiver.connected rather than probing outward.
        Separately, try the Pico CMD port (9001) if a Pico IP is entered.
        """
        # Primary check: has a Pico connected to our data receiver?
        if self._receiver and self._receiver.connected:
            r = self._receiver
            if r.frames_session > 0:
                msg = (f"Pico streaming OK  —  {r.frames_session} frames "
                       f"@ {r.fps:.0f} FPS  CRC errors: {r.crc_errors}  "
                       f"Missed: {r.missed_session}  (from {r.client_ip})")
                self._fw_status.set(msg)
                self._fw_status_lbl.config(fg=GREEN)
                self._fw_log_append(f"[FW] {msg}\n")
                # Pico is live — automatically exit SW demo mode
                if self._demo_mode:
                    self._stop_demo()
                    self._fw_log_append("[FW] SW demo mode disabled — switching to live Pico data.\n")
            else:
                byte_info = (f"bytes rx: {r.bytes_session}  "
                             f"total frames ever: {r.frames_total}")
                if r.bytes_session == 0:
                    hint = ("  → Pico connected but sending no data.  "
                            "Connect Serial Monitor and type 'status' in CMD terminal.")
                else:
                    hint = (f"  → Bytes arriving but no valid frames.  "
                            f"CRC errors: {r.crc_errors}  (sync/format mismatch?)")
                msg = (f"Pico connected from {r.client_ip} — waiting for first frame...  "
                       f"({byte_info})")
                self._fw_status.set(msg)
                self._fw_status_lbl.config(fg=ORANGE)
                self._fw_log_append(f"[FW] {msg}\n")
                self._fw_log_append(f"[FW] {hint}\n")
                # No frames yet — re-check in 3 s (while within 120 s deadline)
                if time.time() < self._fw_conn_deadline:
                    self.root.after(3000, self._fw_test_connection)
            return

        # Pico not yet connected — try the CMD port if user entered Pico IP
        pico_ip = self._cmd_host_var.get().strip()
        if not pico_ip:
            if time.time() < self._fw_conn_deadline:
                msg = (f"Waiting for Pico data connection on port {self.data_port} ...")
                self._fw_status.set(msg)
                self._fw_status_lbl.config(fg=ORANGE)
                self._fw_log_append(f"[FW] {msg}\n")
                self.root.after(3000, self._fw_test_connection)
            else:
                msg = ("Pico did not connect within 120 s.  "
                       "Check: board powered, WiFi SSID/password correct, "
                       f"Host PC IP is {self._fw_host_var.get().strip()}, "
                       "Serial Monitor for error messages.")
                self._fw_status.set(msg)
                self._fw_status_lbl.config(fg=RED)
                self._fw_log_append(f"[FW] TIMEOUT: {msg}\n")
            return

        try:
            cmd_port = int(self._fw_cmd_port.get())
        except ValueError:
            cmd_port = 9001

        self._fw_status.set(f"Probing Pico CMD server {pico_ip}:{cmd_port} ...")
        self._fw_status_lbl.config(fg=FG_DIM)

        import threading as _thr
        def _probe():
            try:
                s = socket.create_connection((pico_ip, cmd_port), timeout=4)
                s.close()
                msg, fg = (f"Pico CMD server online at {pico_ip}:{cmd_port}  "
                           f"— click Connect in top bar to link CMD channel."), GREEN
            except Exception as e:
                msg, fg = (f"No CMD response from {pico_ip}:{cmd_port}: {e}  "
                           f"— Pico may still be booting, check Serial Monitor."), ORANGE
                # Schedule another check in 3 s while within deadline
                if time.time() < self._fw_conn_deadline:
                    self.root.after(3000, self._fw_test_connection)
            self.root.after(0, lambda: (
                self._fw_status.set(msg),
                self._fw_status_lbl.config(fg=fg),
                self._fw_log_append(f"[FW] {msg}\n")))

        _thr.Thread(target=_probe, daemon=True).start()

    def _fw_log_append(self, text):
        """Append a line to the Deploy Log panel in the Programming tab (main thread only)."""
        if self._fw_log_text is None:
            return
        ts = datetime.now().strftime("%H:%M:%S")
        # Prepend timestamp to every non-empty line in the text block
        stamped = "\n".join(
            f"[{ts}] {line}" if line else ""
            for line in text.rstrip("\n").split("\n")
        ) + "\n"
        self._fw_log_text.insert(tk.END, stamped)
        lines = int(self._fw_log_text.index(tk.END).split('.')[0])
        if lines > 2100:
            self._fw_log_text.delete('1.0', f'{lines - 2000}.0')
        self._fw_log_text.see(tk.END)

    # ---- Serial Monitor --------------------------------------------------

    def _serial_start(self):
        """Open the Pico's USB serial port and stream output to the monitor."""
        port = self._fw_pico_port.get().strip()
        if not port:
            self._serial_status.set("Set Pico USB port first (or run Deploy to auto-detect)")
            return
        if self._serial_active:
            return  # already running
        self._serial_active = True
        self._serial_status.set(f"Connecting to {port}...")
        if hasattr(self, '_serial_status_lbl'):
            self._serial_status_lbl.config(fg=ORANGE)
        import threading as _thr
        self._serial_thread = _thr.Thread(
            target=self._serial_worker, args=(port,), daemon=True)
        self._serial_thread.start()

    def _serial_stop(self):
        """Stop the serial monitor and release the COM port."""
        self._serial_active = False
        self._serial_status.set("Disconnected")
        if hasattr(self, '_serial_status_lbl'):
            self._serial_status_lbl.config(fg=FG_DIMMER)

    def _serial_clear(self):
        """Clear the serial monitor text area."""
        if self._serial_text:
            self._serial_text.delete('1.0', tk.END)

    def _serial_append(self, text):
        """Append text to the serial monitor widget (called on main thread)."""
        if self._serial_text is None:
            return
        self._serial_text.insert(tk.END, text)
        # Keep buffer bounded: trim to last 4000 lines
        lines = int(self._serial_text.index(tk.END).split('.')[0])
        if lines > 4100:
            self._serial_text.delete('1.0', f'{lines - 4000}.0')
        self._serial_text.see(tk.END)

    def _serial_worker(self, port):
        """Background thread: reads bytes from the Pico's USB serial port."""
        try:
            import serial
        except ImportError:
            self.root.after(0, lambda: self._serial_append(
                "[SERIAL] pyserial not found. Run: pip install pyserial\n"))
            self._serial_active = False
            return

        baud = int(self._serial_baud.get())
        self.root.after(0, lambda: self._serial_append(
            f"--- Serial monitor connected: {port} @ {baud} baud ---\n"))
        self.root.after(0, lambda: (
            self._serial_status.set(f"{port} @ {baud}"),
            self._serial_status_lbl.config(fg=GREEN)
            if hasattr(self, '_serial_status_lbl') else None))
        try:
            with serial.Serial(port, baud, timeout=0.1,
                               dtr=False, rts=False) as ser:
                while self._serial_active:
                    try:
                        data = ser.read(256)
                    except serial.SerialException:
                        break
                    if data:
                        text = data.decode('utf-8', errors='replace')
                        self.root.after(0, lambda t=text: self._serial_append(t))
        except Exception as e:
            self.root.after(0, lambda: self._serial_append(
                f"\n--- Serial error: {e} ---\n"))
        finally:
            self._serial_active = False
            self.root.after(0, lambda: (
                self._serial_status.set("Disconnected"),
                self._serial_status_lbl.config(fg=FG_DIMMER)
                if hasattr(self, '_serial_status_lbl') else None))
            self.root.after(0, lambda: self._serial_append(
                "--- Serial monitor disconnected ---\n"))

    def _run_hardware_test(self):
        """Send 'test_hardware' CMD and display results in terminal + status label."""
        self._hw_test_status_var.set("Running test...")
        if self._cmd_client and self._cmd_client.connected:
            self._send_cmd("test_hardware")
            self._term_append("> test_hardware\n", 'cmd')
            # Also check crash log while we're at it
            self._send_cmd("crash_log")
            self._term_append("> crash_log\n", 'cmd')
        else:
            self._hw_test_status_var.set("Not connected — connect CMD port first")
            self._term_append("[TEST] Not connected.\n", 'err')

    def _ota_flash(self):
        """Browse for a .uf2 file, then run ota_client.py in a background thread."""
        uf2_path = filedialog.askopenfilename(
            title="Select firmware .uf2 file",
            filetypes=[("UF2 firmware", "*.uf2"), ("All files", "*.*")],
        )
        if not uf2_path:
            return

        self._ota_status_var.set(f"Flashing: {os.path.basename(uf2_path)}")
        self._term_append(f"[OTA] Starting OTA flash: {uf2_path}\n", 'info')

        def _run():
            import subprocess, sys as _sys
            script = os.path.join(os.path.dirname(__file__), 'ota_client.py')
            # Determine host IP from CMD client or receiver
            host = ''
            if hasattr(self, '_cmd_client') and self._cmd_client:
                host = self._cmd_client.host or ''
            if not host and self._receiver:
                host = self._receiver.client_ip or ''
            cmd = [_sys.executable, script, uf2_path]
            if host:
                cmd += ['--host', host]
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1,
                )
                for line in proc.stdout:
                    line = line.rstrip()
                    self.root.after(0, lambda l=line:
                        (self._term_append(l + '\n', 'info'),
                         self._ota_status_var.set(l)))
                proc.wait()
                final = "[OTA] Done." if proc.returncode == 0 else f"[OTA] Exited with code {proc.returncode}"
                self.root.after(0, lambda: self._ota_status_var.set(final))
            except Exception as e:
                self.root.after(0, lambda: self._ota_status_var.set(f"[OTA] Error: {e}"))

        import threading as _thr
        _thr.Thread(target=_run, daemon=True).start()

    def _toggle_recording(self):
        if self._log_active:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self):
        data_dir = self._data_dir_var.get().strip() or 'data'
        prefix   = self._log_prefix_var.get().strip() or 'mushio'
        roll_key = self._log_roll_var.get()
        roll_min = LOG_ROLL_OPTIONS.get(roll_key, 60)
        self._receiver.start_logging(data_dir, prefix, roll_min)
        if HAS_H5PY and self._hdf5_enabled_var.get():
            try:
                self._receiver.start_hdf5_logging(data_dir, prefix)
            except Exception as e:
                self._log_status_var.set(f"HDF5 open failed: {e}")
        self._log_active = True
        self._rec_btn.config(text="  \u25a0 Stop Recording  ",
                              bg='#3a1a1a', fg=RED,
                              activebackground=RED)
        self._log_status_var.set("Starting...")

    def _stop_recording(self):
        self._receiver.stop_logging()
        if self._receiver.hdf5_active:
            self._receiver.stop_hdf5_logging()
        self._log_active = False
        self._rec_btn.config(text="  \u25b6 Start Recording  ",
                              bg='#1a3a1a', fg=GREEN,
                              activebackground=GREEN)
        self._log_status_var.set("Not recording")
        self._log_file_var.set('')

    def _poll_recording_status(self):
        if self._log_active and self._receiver:
            r = self._receiver
            if r.log_active:
                frames  = r.log_frames
                path    = r.log_curr_path
                fname   = os.path.basename(path) if path else '?'
                size_mb = 0.0
                if path and os.path.exists(path):
                    size_mb = os.path.getsize(path) / (1024 * 1024)
                if r._log_next_roll:
                    secs_left = max(0, r._log_next_roll - time.time())
                    m, s = divmod(int(secs_left), 60)
                    h, m = divmod(m, 60)
                    roll_str = (f"{h}h{m:02d}m" if h else f"{m}m{s:02d}s")
                else:
                    roll_str = ''
                self._log_status_var.set(
                    f"● REC  {frames:,} frames  {size_mb:.1f} MB"
                    + (f"  next roll in {roll_str}" if roll_str else ""))
                self._log_file_var.set(fname)
                self._rec_status_lbl.config(fg=GREEN)
        self.root.after(2000, self._poll_recording_status)

    def _browse_data_dir(self):
        d = filedialog.askdirectory(title="Select data directory",
                                     initialdir=self._data_dir_var.get())
        if d:
            self._data_dir_var.set(d)

    def _browse_cam_dir(self):
        d = filedialog.askdirectory(title="Select captures directory",
                                     initialdir=self._cam_dir_var.get())
        if d:
            self._cam_dir_var.set(d)
            if self._webcam:
                self._webcam.output_dir = d
                os.makedirs(d, exist_ok=True)

    def _open_dir(self, path):
        path = os.path.abspath(path)
        os.makedirs(path, exist_ok=True)
        try:
            if sys.platform == 'win32':
                os.startfile(path)
            elif sys.platform == 'darwin':
                import subprocess; subprocess.Popen(['open', path])
            else:
                import subprocess; subprocess.Popen(['xdg-open', path])
        except Exception:
            pass

    def _load_settings(self):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_settings(self):
        s = {
            'data_dir':            self._data_dir_var.get(),
            'log_prefix':          self._log_prefix_var.get(),
            'log_roll':            self._log_roll_var.get(),
            'cam_dir':             self._cam_dir_var.get(),
            'board_id':            self._board_id_var.get(),
            'exp_name':            self._exp_name_var.get(),
            'species_common':      self._species_common_var.get(),
            'species_scientific':  self._species_sci_var.get(),
            'experiment_desc':     self._exp_desc_txt.get('1.0', tk.END).strip()
                                   if hasattr(self, '_exp_desc_txt') else '',
            'ssid_history':        self._fw_ssid_history,
        }
        try:
            with open(SETTINGS_FILE, 'w') as f:
                json.dump(s, f, indent=2)
            self._settings_status.set("Paths saved.")
            self.root.after(3000, lambda: self._settings_status.set(''))
        except Exception as e:
            self._settings_status.set(f"Error: {e}")

    # ---- Full experiment config save / load --------------------------------

    def _config_snapshot(self):
        """Return a dict representing the full current experiment configuration."""
        desc = self._exp_desc_txt.get('1.0', tk.END).strip() \
               if hasattr(self, '_exp_desc_txt') else ''
        regs = {}
        for (adc_i, addr), var in self._reg_vars.items():
            regs[f'{adc_i}:{addr:#04x}'] = var.get()
        stim = {}
        for flat, cfg in self._stim_ch_configs.items():
            c = dict(cfg)
            c['csv_data'] = None   # numpy array — not JSON serialisable
            stim[str(flat)] = c
        return {
            'version': '1.0',
            'saved_at': datetime.now().isoformat(timespec='seconds'),
            'experiment': {
                'board_id':            self._board_id_var.get(),
                'species_common':      self._species_common_var.get(),
                'species_scientific':  self._species_sci_var.get(),
                'description':         desc,
            },
            'display': {
                'pga_gain':     self._pga_gain.get(),
                'bp_enabled':   self._bp_enabled.get(),
                'bp_low':       self._bp_low.get(),
                'bp_high':      self._bp_high.get(),
                'bp_order':     self._bp_order_var.get(),
                'uv_scale':     self._uv_scale.get(),
                'display_secs': self._display_secs.get(),
            },
            'stim_configs': stim,
            'registers': regs,
            'logging': {
                'data_dir':   self._data_dir_var.get(),
                'log_prefix': self._log_prefix_var.get(),
                'log_roll':   self._log_roll_var.get(),
                'cam_dir':    self._cam_dir_var.get(),
            },
        }

    def _apply_config(self, cfg):
        """Apply a loaded config dict to all GUI state."""
        exp  = cfg.get('experiment', {})
        disp = cfg.get('display',    {})
        log  = cfg.get('logging',    {})

        self._board_id_var.set(exp.get('board_id', ''))
        self._species_common_var.set(exp.get('species_common', ''))
        self._species_sci_var.set(exp.get('species_scientific', ''))
        if hasattr(self, '_exp_desc_txt'):
            self._exp_desc_txt.delete('1.0', tk.END)
            self._exp_desc_txt.insert('1.0', exp.get('description', ''))

        if 'pga_gain' in disp:     self._pga_gain.set(disp['pga_gain'])
        if 'bp_enabled' in disp:   self._bp_enabled.set(disp['bp_enabled'])
        if 'bp_low' in disp:       self._bp_low.set(disp['bp_low'])
        if 'bp_high' in disp:      self._bp_high.set(disp['bp_high'])
        if 'bp_order' in disp:     self._bp_order_var.set(disp['bp_order'])
        if 'uv_scale' in disp:     self._uv_scale.set(disp['uv_scale'])
        if 'display_secs' in disp: self._display_secs.set(disp['display_secs'])

        for key, val_str in cfg.get('registers', {}).items():
            try:
                adc_i, addr_s = key.split(':')
                k = (int(adc_i), int(addr_s, 16))
                if k in self._reg_vars:
                    self._reg_vars[k].set(val_str)
            except Exception:
                pass

        for flat_s, c in cfg.get('stim_configs', {}).items():
            try:
                flat = int(flat_s)
                if flat in self._stim_ch_configs:
                    self._stim_ch_configs[flat].update({
                        k: v for k, v in c.items() if k != 'csv_data'
                    })
            except Exception:
                pass
        self._stim_update_ch_summary()

        if 'data_dir'   in log: self._data_dir_var.set(log['data_dir'])
        if 'log_prefix' in log: self._log_prefix_var.set(log['log_prefix'])
        if 'log_roll'   in log: self._log_roll_var.set(log['log_roll'])
        if 'cam_dir'    in log: self._cam_dir_var.set(log['cam_dir'])

    def _save_config(self):
        path = filedialog.asksaveasfilename(
            title="Save Experiment Config",
            defaultextension=".json",
            filetypes=[("JSON config", "*.json"), ("All files", "*.*")],
            initialfile=f"mushio_config_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
        if not path:
            return
        try:
            snap = self._config_snapshot()
            with open(path, 'w') as f:
                json.dump(snap, f, indent=2)
            self._cfg_status.set(f"Saved: {os.path.basename(path)}")
            self.root.after(5000, lambda: self._cfg_status.set(''))
        except Exception as e:
            self._cfg_status.set(f"Save error: {e}")

    def _load_config(self):
        path = filedialog.askopenfilename(
            title="Load Experiment Config",
            filetypes=[("JSON config", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path, 'r') as f:
                cfg = json.load(f)
            ver = cfg.get('version', '?')
            if ver not in ('1.0',):
                self._cfg_status.set(f"Warning: unknown config version {ver}, attempting load...")
            self._apply_config(cfg)
            saved_at = cfg.get('saved_at', 'unknown')
            self._cfg_status.set(
                f"Loaded: {os.path.basename(path)}  (saved {saved_at})")
            self.root.after(8000, lambda: self._cfg_status.set(''))
        except Exception as e:
            self._cfg_status.set(f"Load error: {e}")

    # ---- ADC Registers tab -----------------------------------------------

    def _build_regs_tab(self, parent):
        # Top toolbar
        tb = tk.Frame(parent, bg=BG_MID, pady=5, padx=8)
        tb.pack(fill=tk.X)

        tk.Button(tb, text="  Read All from Hardware  ",
                  bg='#1a2a1a', fg=GREEN,
                  activebackground=GREEN, activeforeground=BG_DARK,
                  font=('Consolas', 9, 'bold'), pady=4, relief=tk.FLAT,
                  command=self._regs_read_all).pack(side=tk.LEFT, padx=2)

        tk.Button(tb, text="  Write All to Hardware  ",
                  bg='#2a1a1a', fg=ORANGE,
                  activebackground=ORANGE, activeforeground=BG_DARK,
                  font=('Consolas', 9, 'bold'), pady=4, relief=tk.FLAT,
                  command=self._regs_write_all).pack(side=tk.LEFT, padx=6)

        tk.Button(tb, text="Reset to Power-On Defaults",
                  bg=BG_LIGHT, fg=FG_DIM,
                  activebackground=FG_DIMMER, activeforeground=BG_DARK,
                  font=('Consolas', 9), pady=4, relief=tk.FLAT,
                  command=self._regs_reset_defaults).pack(side=tk.LEFT, padx=2)

        # Font / layout mode toggle
        tk.Label(tb, text="  |  Text size:",
                 bg=BG_MID, fg=FG_DIMMER).pack(side=tk.LEFT, padx=(8, 2))
        for label, val in [("Compact  (fit)", 'compact'), ("Large  (scroll)", 'large')]:
            tk.Radiobutton(tb, text=label, variable=self._regs_font_mode, value=val,
                           bg=BG_MID, fg=FG_MAIN, selectcolor=BG_LIGHT,
                           activebackground=BG_MID, activeforeground=ACCENT,
                           command=self._regs_rebuild_table
                           ).pack(side=tk.LEFT, padx=3)

        self._regs_status = tk.StringVar(value='')
        tk.Label(tb, textvariable=self._regs_status,
                 bg=BG_MID, fg=FG_DIM, font=('Consolas', 8)).pack(side=tk.LEFT, padx=12)

        # ---- Scrollable table canvas (shared; inner content rebuilt on mode change) ----
        tbl_frame = tk.Frame(parent, bg=BG_DARK)
        tbl_frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        h_sb = ttk.Scrollbar(tbl_frame, orient='horizontal')
        v_sb = ttk.Scrollbar(tbl_frame, orient='vertical')
        self._regs_tbl_canvas = tk.Canvas(tbl_frame, bg=BG_DARK, highlightthickness=0,
                                          xscrollcommand=h_sb.set,
                                          yscrollcommand=v_sb.set)
        h_sb.configure(command=self._regs_tbl_canvas.xview)
        v_sb.configure(command=self._regs_tbl_canvas.yview)
        h_sb.pack(side=tk.BOTTOM, fill=tk.X)
        v_sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._regs_tbl_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._regs_inner = None
        self._reg_write_btns = {}
        self._regs_rebuild_table()

    def _regs_rebuild_table(self):
        """Destroy and recreate the register table grid for the current font mode."""
        if self._regs_inner is not None:
            self._regs_inner.destroy()
            self._regs_inner = None

        large = (self._regs_font_mode.get() == 'large')
        fs    = 10 if large else 8          # body font size
        fs_sm = 8  if large else 7          # bit-field font (slightly smaller)
        VAL_W = 9  if large else 7          # ADC entry column width

        HDR_FONT   = ('Consolas', fs, 'bold')
        CELL_FONT  = ('Consolas', fs)
        LABEL_FONT = ('Consolas', fs)

        # Column widths scale with font
        scale = fs / 8.0
        col_headers = ['Addr', 'Name', 'R/W', 'Description', 'Bit fields'] + \
                      [f'ADC{i}' for i in range(NUM_ADCS)] + ['']
        col_widths  = [int(5*scale), int(9*scale), int(4*scale),
                       int(42*scale), int(32*scale)] + \
                      [VAL_W] * NUM_ADCS + [int(10*scale)]

        inner = tk.Frame(self._regs_tbl_canvas, bg=BG_DARK)
        _twin = self._regs_tbl_canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: self._regs_tbl_canvas.configure(
            scrollregion=self._regs_tbl_canvas.bbox('all')))
        self._regs_inner = inner

        hdr_bg = BG_MID
        for ci, (ch, cw) in enumerate(zip(col_headers, col_widths)):
            tk.Label(inner, text=ch, bg=hdr_bg, fg=ACCENT,
                     font=HDR_FONT, width=cw, anchor=tk.W,
                     padx=4, pady=4, relief=tk.FLAT
                     ).grid(row=0, column=ci, sticky='ew', padx=1, pady=(0, 2))

        tk.Frame(inner, bg=BG_LIGHT, height=1).grid(
            row=1, column=0, columnspan=len(col_headers), sticky='ew', pady=1)

        self._reg_write_btns = {}
        for ri, (addr, name, rw, desc, default, bits) in enumerate(ADS124S08_REGS):
            row_i  = ri + 2
            row_bg = BG_DARK if ri % 2 == 0 else BG_MID
            fg_label = FG_DIM if rw == 'R' else FG_MAIN
            rw_col   = FG_DIMMER if rw == 'R' else GREEN

            tk.Label(inner, text=f'0x{addr:02X}', bg=row_bg, fg=FG_DIMMER,
                     font=LABEL_FONT, width=col_widths[0], anchor=tk.W, padx=4
                     ).grid(row=row_i, column=0, sticky='ew', padx=1, pady=2)

            tk.Label(inner, text=name, bg=row_bg, fg=fg_label,
                     font=(LABEL_FONT[0], fs, 'bold'),
                     width=col_widths[1], anchor=tk.W, padx=4
                     ).grid(row=row_i, column=1, sticky='ew', padx=1, pady=2)

            tk.Label(inner, text=rw, bg=row_bg, fg=rw_col,
                     font=LABEL_FONT, width=col_widths[2], anchor=tk.CENTER
                     ).grid(row=row_i, column=2, sticky='ew', padx=1, pady=2)

            tk.Label(inner, text=desc, bg=row_bg, fg=fg_label,
                     font=LABEL_FONT, width=col_widths[3], anchor=tk.W, padx=4
                     ).grid(row=row_i, column=3, sticky='ew', padx=1, pady=2)

            tk.Label(inner, text=bits, bg=row_bg, fg=FG_DIMMER,
                     font=('Consolas', fs_sm), width=col_widths[4], anchor=tk.W, padx=4,
                     wraplength=int(240 * scale), justify=tk.LEFT
                     ).grid(row=row_i, column=4, sticky='ew', padx=1, pady=2)

            entry_bg = BG_LIGHT if rw == 'RW' else row_bg
            entry_fg = FG_MAIN  if rw == 'RW' else FG_DIMMER
            for adc_i in range(NUM_ADCS):
                var = self._reg_vars[(adc_i, addr)]
                e = tk.Entry(inner, textvariable=var, width=VAL_W,
                             bg=entry_bg, fg=entry_fg,
                             disabledbackground=row_bg, disabledforeground=FG_DIMMER,
                             insertbackground=FG_MAIN, relief=tk.FLAT,
                             font=CELL_FONT,
                             state=tk.NORMAL if rw == 'RW' else tk.DISABLED)
                e.grid(row=row_i, column=5 + adc_i, sticky='ew', padx=2, pady=2)

            btn = ttk.Button(inner, text="Write row",
                             command=lambda a=addr, n=name: self._regs_write_row(a, n))
            btn.grid(row=row_i, column=5 + NUM_ADCS, padx=4, pady=2)
            if rw == 'R':
                btn.state(['disabled'])
            self._reg_write_btns[addr] = btn

        self._regs_tbl_canvas.update_idletasks()
        self._regs_tbl_canvas.configure(
            scrollregion=self._regs_tbl_canvas.bbox('all'))

    # ---- Register helpers ------------------------------------------------

    def _regs_read_all(self):
        self._regs_status.set("Sending read_regs ...")
        self._send_cmd('read_regs')

    def _regs_write_all(self):
        """Write every RW register to all ADCs via the command channel."""
        cmds = []
        for addr, name, rw, desc, default, bits in ADS124S08_REGS:
            if rw != 'RW':
                continue
            for adc_i in range(NUM_ADCS):
                val_str = self._reg_vars[(adc_i, addr)].get().strip()
                try:
                    val = int(val_str, 16) if val_str.startswith('0x') else int(val_str, 0)
                    val = max(0, min(255, val))
                except ValueError:
                    continue
                cmds.append(f'wreg {adc_i} 0x{addr:02X} 0x{val:02X}')
        for c in cmds:
            self._send_cmd(c)
        self._regs_status.set(f"Queued {len(cmds)} write commands.")

    def _regs_write_row(self, addr, name):
        """Write one register row to all ADCs."""
        count = 0
        for adc_i in range(NUM_ADCS):
            val_str = self._reg_vars[(adc_i, addr)].get().strip()
            try:
                val = int(val_str, 16) if val_str.startswith('0x') else int(val_str, 0)
                val = max(0, min(255, val))
            except ValueError:
                continue
            self._send_cmd(f'wreg {adc_i} 0x{addr:02X} 0x{val:02X}')
            count += 1
        self._regs_status.set(f"Wrote {name} (0x{addr:02X}) to {count} ADCs.")

    def _regs_reset_defaults(self):
        """Reload all register vars to their power-on defaults."""
        for addr, name, rw, desc, default, bits in ADS124S08_REGS:
            for adc_i in range(NUM_ADCS):
                self._reg_vars[(adc_i, addr)].set(f'0x{default:02X}')
        self._regs_status.set("Reset to power-on defaults (not written to hardware).")

    def _regs_ingest_response(self, text):
        """Parse a read_regs response line and update _reg_vars.

        Expected line format (from firmware or DemoCmdClient):
          ADC<n> 0x<addr>=0x<val> [0x<addr>=0x<val> ...]
        or:
          ADC<n>: <name>(0x<addr>)=0x<val> ...
        """
        import re
        adc_m = re.match(r'ADC(\d)', text)
        if not adc_m:
            return
        adc_i = int(adc_m.group(1))
        if adc_i >= NUM_ADCS:
            return
        # Match addr=val pairs
        for m in re.finditer(r'0x([0-9A-Fa-f]{2})\s*=\s*0x([0-9A-Fa-f]{2})', text):
            addr = int(m.group(1), 16)
            val  = int(m.group(2), 16)
            key  = (adc_i, addr)
            if key in self._reg_vars:
                self._reg_vars[key].set(f'0x{val:02X}')

    # ---- Self-Test tab ---------------------------------------------------

    def _build_selftest_tab(self, parent):
        """Self-Test tab: diagnostic buttons + command terminal.
        Connection settings (IP/port) remain in the top bar."""
        # Self-test buttons
        self._build_selftest_section(parent)

        # Terminal fills remaining space
        term = self._build_terminal(parent)
        term.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

    # ---- Programming tab -------------------------------------------------

    def _build_programming_tab(self, parent):
        """Board programming tab: flash firmware, WiFi config, connection test."""
        # Deploy log — packed first with side=BOTTOM so the scrollable canvas gets
        # all remaining space above it.  This shows [FW] deploy messages in-tab.
        _fw_log_frame = ttk.LabelFrame(parent, text="Deploy Log", padding=4)
        _fw_log_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(0, 8))
        self._fw_log_text = scrolledtext.ScrolledText(
            _fw_log_frame, height=10, wrap=tk.WORD,
            bg=BG_MID, fg=GREEN, insertbackground=FG_MAIN,
            font=('Consolas', 9), state=tk.NORMAL)
        self._fw_log_text.pack(fill=tk.BOTH, expand=True)

        outer_canvas = tk.Canvas(parent, bg=BG_DARK, highlightthickness=0)
        _osb = ttk.Scrollbar(parent, orient='vertical', command=outer_canvas.yview)
        outer_canvas.configure(yscrollcommand=_osb.set)
        _osb.pack(side=tk.RIGHT, fill=tk.Y)
        outer_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        pad = tk.Frame(outer_canvas, bg=BG_DARK)
        _owin = outer_canvas.create_window((0, 0), window=pad, anchor='nw')
        pad.bind('<Configure>', lambda e: outer_canvas.configure(
            scrollregion=outer_canvas.bbox('all')))
        outer_canvas.bind('<Configure>',
                          lambda e: outer_canvas.itemconfig(_owin, width=e.width))

        fw_frame = ttk.LabelFrame(pad,
            text="Board Programming  (flash firmware to Raspberry Pi Pico 2 W)",
            padding=10)
        fw_frame.pack(fill=tk.X, padx=12, pady=(10, 4))

        # Base fonts for this tab — deliberately larger than the rest of the UI
        _FL  = ('Helvetica', 10)          # label / note
        _FC  = ('Consolas',  10)          # monospace entry / status
        _FLd = ('Helvetica',  9)          # dim sub-note
        _LW  = 24                         # label column width (chars)

        fw_note = ("Fill in WiFi credentials and this PC's IP address, then click "
                   "Deploy.  The GUI patches config.h, runs cmake + ninja, and "
                   "copies the UF2 to the Pico in BOOTSEL mode automatically.  "
                   "CMake, Ninja, and arm-none-eabi-gcc must be in PATH "
                   "(installed via Scoop).  Delete the build/ subfolder to force "
                   "a full reconfigure.")
        tk.Label(fw_frame, text=fw_note,
                 bg=BG_DARK, fg=FG_DIMMER, font=_FLd,
                 wraplength=700, justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 8))

        def _fw_field(parent_frame, label, var, width=22):
            f = tk.Frame(parent_frame, bg=BG_DARK)
            f.pack(fill=tk.X, pady=3)
            tk.Label(f, text=label, bg=BG_DARK, fg=FG_DIM,
                     font=_FL, width=_LW, anchor=tk.W).pack(side=tk.LEFT)
            tk.Entry(f, textvariable=var, width=width,
                     bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                     font=_FC, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)

        # Mode row: C Demo / C Real
        fw_rm = tk.Frame(fw_frame, bg=BG_DARK)
        fw_rm.pack(fill=tk.X, pady=3)
        tk.Label(fw_rm, text="Firmware mode:", bg=BG_DARK, fg=FG_DIM,
                 font=_FL, width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        tk.Radiobutton(fw_rm,
                       text="C Demo  (Raspberry Pi Pico 2 W — synthetic sine waves)",
                       variable=self._fw_mode, value='c_demo',
                       bg=BG_DARK, fg=ORANGE, selectcolor=BG_LIGHT,
                       activebackground=BG_DARK, activeforeground=ORANGE,
                       font=_FL).pack(side=tk.LEFT, padx=4)
        tk.Radiobutton(fw_rm,
                       text="C Real  (MushIO board — live ADS124S08 ADCs)",
                       variable=self._fw_mode, value='c_real',
                       bg=BG_DARK, fg=GREEN, selectcolor=BG_LIGHT,
                       activebackground=BG_DARK, activeforeground=GREEN,
                       font=_FL).pack(side=tk.LEFT, padx=8)

        # C firmware folder row (only relevant in C Native mode)
        fw_rc = tk.Frame(fw_frame, bg=BG_DARK)
        fw_rc.pack(fill=tk.X, pady=2)
        tk.Label(fw_rc, text="C fw folder:", bg=BG_DARK, fg=FG_DIM,
                 font=_FL, width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        tk.Entry(fw_rc, textvariable=self._fw_c_fw_dir_var, width=36,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 font=_FC, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        ttk.Button(fw_rc, text="Browse",
                   command=lambda: (lambda p=filedialog.askdirectory(
                       title="Select C firmware folder"):
                       self._fw_c_fw_dir_var.set(p) if p else None)()
                   ).pack(side=tk.LEFT, padx=2)
        tk.Label(fw_rc, text="(contains config.h + CMakeLists.txt)",
                 bg=BG_DARK, fg=FG_DIMMER, font=_FLd).pack(side=tk.LEFT, padx=6)

        # Pico USB port row (auto-detect or manual override)
        fw_rport = tk.Frame(fw_frame, bg=BG_DARK)
        fw_rport.pack(fill=tk.X, pady=3)
        tk.Label(fw_rport, text="Pico USB port:", bg=BG_DARK, fg=FG_DIM,
                 font=_FL, width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        tk.Entry(fw_rport, textvariable=self._fw_pico_port, width=10,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 font=_FC, relief=tk.FLAT).pack(side=tk.LEFT, padx=4)
        tk.Label(fw_rport, text="(leave blank to auto-detect by USB ID)",
                 bg=BG_DARK, fg=FG_DIMMER, font=_FLd).pack(side=tk.LEFT, padx=6)

        # SSID — Combobox with saved history
        _ssid_row = tk.Frame(fw_frame, bg=BG_DARK)
        _ssid_row.pack(fill=tk.X, pady=3)
        tk.Label(_ssid_row, text="WiFi SSID:", bg=BG_DARK, fg=FG_DIM,
                 font=_FL, width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        self._fw_ssid_cb = ttk.Combobox(
            _ssid_row, textvariable=self._fw_wifi_ssid,
            values=self._fw_ssid_history, width=24, font=_FC)
        self._fw_ssid_cb.pack(side=tk.LEFT, padx=4)
        tk.Label(_ssid_row, text="(past SSIDs auto-saved)",
                 bg=BG_DARK, fg=FG_DIMMER, font=_FLd).pack(side=tk.LEFT, padx=6)

        # Password row — always masked, no reveal button (pre-loaded from config)
        fw_rp = tk.Frame(fw_frame, bg=BG_DARK)
        fw_rp.pack(fill=tk.X, pady=3)
        tk.Label(fw_rp, text="WiFi Password:", bg=BG_DARK, fg=FG_DIM,
                 font=_FL, width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        tk.Entry(fw_rp, textvariable=self._fw_wifi_pass, width=24,
                 bg=BG_LIGHT, fg=FG_MAIN, insertbackground=FG_MAIN,
                 font=_FC, relief=tk.FLAT, show='*').pack(side=tk.LEFT, padx=4)
        tk.Label(fw_rp, text="(pre-loaded)", bg=BG_DARK, fg=FG_DIMMER,
                 font=_FLd).pack(side=tk.LEFT, padx=6)

        # Host IP
        _fw_field(fw_frame, "Host PC IP (receiver):", self._fw_host_ip, 18)
        tk.Label(fw_frame,
                 text="  ^ This PC's LAN IP — auto-detected.  "
                      "The Pico connects here on boot.  "
                      "Ensure both devices are on the same WiFi network.",
                 bg=BG_DARK, fg=FG_DIMMER, font=_FLd,
                 wraplength=680, justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 6))

        _fw_field(fw_frame, "Data port:",    self._fw_data_port,  8)
        _fw_field(fw_frame, "Command port:", self._fw_cmd_port,   8)

        # ---- Flash method --------------------------------------------------
        fw_fm = tk.Frame(fw_frame, bg=BG_DARK)
        fw_fm.pack(fill=tk.X, pady=(6, 2))
        tk.Label(fw_fm, text="Flash method:", bg=BG_DARK, fg=FG_DIM,
                 font=_FL, width=_LW, anchor=tk.W).pack(side=tk.LEFT)
        for _val, _lbl, _tip in [
            ('auto', 'Auto',        'picotool if available, else BOOTSEL drive copy'),
            ('usb',  'USB (BOOTSEL)', 'hold BOOTSEL + plug USB → drive copy'),
            ('ota',  'OTA (WiFi)',  'send ota_reboot via CMD port → network flash'),
        ]:
            tk.Radiobutton(fw_fm, text=_lbl, value=_val,
                           variable=self._fw_flash_method,
                           bg=BG_DARK, fg=FG_MAIN, selectcolor=BG_LIGHT,
                           activebackground=BG_DARK, activeforeground=ACCENT,
                           font=_FL).pack(side=tk.LEFT, padx=6)
        tk.Label(fw_fm, textvariable=tk.StringVar(), bg=BG_DARK, fg=FG_DIMMER,
                 font=_FLd).pack(side=tk.LEFT)

        # Flash-method tip (updates on selection change)
        _fm_tip_texts = {
            'auto': 'picotool if available, else BOOTSEL drive copy',
            'usb':  'Hold BOOTSEL + plug USB cable → drive appears → UF2 copied',
            'ota':  'Pico must be running firmware with CMD port — reboots wirelessly',
        }
        _fm_tip_var = tk.StringVar(value=_fm_tip_texts['auto'])
        def _on_fm_change(*_):
            _fm_tip_var.set(_fm_tip_texts.get(self._fw_flash_method.get(), ''))
        self._fw_flash_method.trace_add('write', _on_fm_change)
        tk.Label(fw_frame, textvariable=_fm_tip_var, bg=BG_DARK, fg=FG_DIMMER,
                 font=_FLd, justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 4))

        fw_btn_row = tk.Frame(fw_frame, bg=BG_DARK)
        fw_btn_row.pack(fill=tk.X, pady=8)
        tk.Button(fw_btn_row, text="  \u26a1  Deploy to Pico  ",
                  bg='#2a1a0a', fg=ORANGE,
                  activebackground=ORANGE, activeforeground=BG_DARK,
                  font=('Consolas', 11, 'bold'), pady=8, relief=tk.FLAT,
                  command=self._fw_deploy).pack(side=tk.LEFT, padx=2)
        tk.Button(fw_btn_row, text="  Test Connection  ",
                  bg=BG_LIGHT, fg=FG_DIM,
                  activebackground=ACCENT, activeforeground=BG_DARK,
                  font=('Consolas', 10), pady=7, relief=tk.FLAT,
                  command=self._fw_test_connection).pack(side=tk.LEFT, padx=10)

        self._fw_status_lbl = tk.Label(fw_frame, textvariable=self._fw_status,
                                        bg=BG_DARK, fg=FG_DIM, font=_FC,
                                        wraplength=700, justify=tk.LEFT)
        self._fw_status_lbl.pack(anchor=tk.W, pady=4)

        # ---- Serial Monitor ----------------------------------------------
        sm_frame = ttk.LabelFrame(pad,
            text="Pico Serial Monitor  (USB)",
            padding=8)
        sm_frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=(4, 10))

        sm_tb = tk.Frame(sm_frame, bg=BG_DARK)
        sm_tb.pack(fill=tk.X, pady=(0, 4))

        tk.Label(sm_tb, text="Baud:", bg=BG_DARK, fg=FG_DIM,
                 font=_FL).pack(side=tk.LEFT)
        ttk.Combobox(sm_tb, textvariable=self._serial_baud,
                     values=['115200', '9600', '57600', '230400'],
                     width=8, state='readonly').pack(side=tk.LEFT, padx=(2, 10))

        tk.Button(sm_tb, text="  Connect  ",
                  bg='#0a2a0a', fg=GREEN,
                  activebackground=GREEN, activeforeground=BG_DARK,
                  font=('Consolas', 9, 'bold'), pady=3, relief=tk.FLAT,
                  command=self._serial_start).pack(side=tk.LEFT, padx=2)
        tk.Button(sm_tb, text="  Disconnect  ",
                  bg=BG_LIGHT, fg=FG_DIM,
                  activebackground=RED, activeforeground=BG_DARK,
                  font=('Consolas', 9), pady=3, relief=tk.FLAT,
                  command=self._serial_stop).pack(side=tk.LEFT, padx=2)
        tk.Button(sm_tb, text="  Clear  ",
                  bg=BG_LIGHT, fg=FG_DIM,
                  font=('Consolas', 9), pady=3, relief=tk.FLAT,
                  command=self._serial_clear).pack(side=tk.LEFT, padx=8)

        self._serial_status_lbl = tk.Label(sm_tb, textvariable=self._serial_status,
                                            bg=BG_DARK, fg=FG_DIMMER,
                                            font=('Consolas', 9))
        self._serial_status_lbl.pack(side=tk.LEFT, padx=12)

        self._serial_text = scrolledtext.ScrolledText(
            sm_frame, height=14, wrap=tk.WORD,
            bg='#0a0a14', fg='#b0ffb0', insertbackground=FG_MAIN,
            font=('Consolas', 9), state=tk.NORMAL)
        self._serial_text.pack(fill=tk.BOTH, expand=True)

    # ---- terminal --------------------------------------------------------

    def _build_terminal(self, parent):
        frame = ttk.LabelFrame(parent, text="Command Terminal", padding=4)
        self._term_out = scrolledtext.ScrolledText(
            frame, height=7, wrap=tk.WORD,
            bg=BG_MID, fg=GREEN, insertbackground=FG_MAIN,
            font=('Consolas', 9), state=tk.DISABLED)
        self._term_out.pack(fill=tk.BOTH, expand=True)
        self._term_out.tag_configure('cmd',  foreground=ACCENT)
        self._term_out.tag_configure('end',  foreground=FG_DIMMER)
        self._term_out.tag_configure('err',  foreground=RED)
        self._term_out.tag_configure('info', foreground=ORANGE)
        # Flush any messages that arrived before the terminal was built
        for _txt, _tag in self._term_buffer:
            self._term_append(_txt, _tag)
        self._term_buffer.clear()

        inp = tk.Frame(frame, bg=BG_DARK)
        inp.pack(fill=tk.X, pady=2)
        tk.Label(inp, text=">", fg=ACCENT, bg=BG_DARK,
                 font=('Consolas', 11, 'bold')).pack(side=tk.LEFT, padx=2)
        self._cmd_entry = tk.Entry(inp, bg=BG_LIGHT, fg=FG_MAIN,
                                    insertbackground=FG_MAIN,
                                    font=('Consolas', 10), relief=tk.FLAT)
        self._cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self._cmd_entry.bind('<Return>', lambda e: self._on_send())
        self._cmd_entry.bind('<Up>',     self._history_up)
        self._cmd_entry.bind('<Down>',   self._history_down)
        ttk.Button(inp, text="Send",  command=self._on_send).pack(side=tk.LEFT)
        ttk.Button(inp, text="Clear", command=self._clear_terminal).pack(side=tk.LEFT, padx=4)
        return frame

    # ---- status bar ------------------------------------------------------

    def _build_status_bar(self):
        bar = tk.Frame(self.root, bg=BG_MID, pady=2, padx=8)
        bar.pack(fill=tk.X, side=tk.BOTTOM)
        self._stat_vars = {}
        for key, default in [('fps','FPS: --'),('frames','Frames: 0'),
                              ('crc','CRC err: 0'),('missed','Missed: 0'),
                              ('session','Session: 0')]:
            v = tk.StringVar(value=default)
            self._stat_vars[key] = v
            tk.Label(bar, textvariable=v, bg=BG_MID, fg=FG_DIM,
                     font=('Helvetica', 9)).pack(side=tk.LEFT, padx=12)
        flags = []
        flags.append("scipy:YES" if HAS_SCIPY else "scipy:NO")
        flags.append("webcam:YES" if HAS_CV2 else "webcam:NO")
        tk.Label(bar, text="  |  " + "  |  ".join(flags),
                 bg=BG_MID, fg=FG_DIMMER, font=('Helvetica', 8)).pack(side=tk.RIGHT)

    # ==========================================================================
    # Electrode / STIM selection
    # ==========================================================================

    def _on_grid_click(self, event):
        """Matplotlib button_press_event: click a cell to open a zoom popup."""
        ax = event.inaxes
        if ax is None:
            return
        if ax in self._grid_ax_to_elec:
            self._open_zoom_window(self._grid_ax_to_elec[ax], is_stim=False)
        elif ax in self._grid_ax_to_stim:
            self._open_zoom_window(self._grid_ax_to_stim[ax], is_stim=True)

    def _toggle_electrode(self, col, row):
        flat = ELEC_GRID.get((col, row))
        if flat is None: return
        if flat in self._selected_elec:
            self._selected_elec.discard(flat)
        else:
            self._selected_elec.add(flat)
        self._update_electrode_btn_colors()
        self._update_elec_count()

    def _select_all_elec(self):
        self._selected_elec = set(ELEC_IDX)
        self._update_electrode_btn_colors()
        self._update_elec_count()

    def _select_none_elec(self):
        self._selected_elec = set()
        self._update_electrode_btn_colors()
        self._update_elec_count()

    def _update_electrode_btn_colors(self):
        needs_draw = False
        for (c, r), btn in self._elec_btns.items():
            flat = ELEC_GRID.get((c, r))
            if flat in self._selected_elec:
                col = COLORS[(c * 8 + r) % len(COLORS)]
                btn.config(bg=col, fg=BG_DARK, font=('Consolas', 7, 'bold'))
                # Highlight the matching grid axes
                ax = self._grid_axes.get(flat)
                if ax is not None:
                    for sp in ax.spines.values():
                        sp.set_color(col); sp.set_linewidth(1.0)
                    needs_draw = True
            else:
                btn.config(bg=BG_LIGHT, fg=FG_DIM, font=('Consolas', 7))
                ax = self._grid_axes.get(flat)
                if ax is not None:
                    for sp in ax.spines.values():
                        sp.set_color(BG_LIGHT); sp.set_linewidth(0.4)
                    needs_draw = True
        if needs_draw and hasattr(self, '_grid_canvas') and self._grid_canvas:
            self._grid_canvas.draw_idle()

    def _update_elec_count(self):
        self._elec_count_var.set(f"{len(self._selected_elec)} of 64 selected")

    def _toggle_stim(self, flat):
        if flat in self._stim_active:
            self._stim_active.discard(flat)
        else:
            self._stim_active.add(flat)
        self._update_stim_btn_colors()

    def _update_stim_btn_colors(self):
        for flat, btn in self._stim_btns.items():
            btn.config(bg=ORANGE if flat in self._stim_active else BG_LIGHT,
                       fg=BG_DARK if flat in self._stim_active else FG_MAIN)

    # ==========================================================================
    # Zoom popup
    # ==========================================================================

    def _open_zoom_window(self, flat, is_stim=False):
        """Open (or bring to front) a zoom popup for the given channel flat index."""
        z = self._zoom_win
        if z is not None:
            try:
                if z['flat'] == flat and z['win'].winfo_exists():
                    z['win'].lift()
                    return
                z['win'].destroy()
            except tk.TclError:
                pass
            self._zoom_win = None

        # Resolve channel name and line colour
        if is_stim:
            ch_name = next((n for n, f in STIM_BY_NAME if f == flat), f'STIM_{flat}')
            color   = ORANGE
        else:
            ch_name = ALL_NAMES[flat]
            col, row = next(((c, r) for (c, r), f in ELEC_GRID.items() if f == flat), (0, 0))
            color   = COLORS[(col * 8 + row) % len(COLORS)]

        win = tk.Toplevel(self.root)
        win.title(f'Zoom  {ch_name}')
        win.configure(bg=BG_DARK)
        win.geometry('720x360')
        win.resizable(True, True)

        fig = Figure(figsize=(7.2, 3.6), dpi=100, facecolor=BG_DARK)
        fig.subplots_adjust(left=0.10, right=0.97, top=0.90, bottom=0.15)
        zoom_ax = fig.add_subplot(111, facecolor=BG_MID)
        zoom_ax.tick_params(colors=FG_DIM, labelsize=9)
        zoom_ax.title.set_text(ch_name)
        zoom_ax.title.set_color(color)
        zoom_ax.title.set_fontsize(11)
        for sp in zoom_ax.spines.values():
            sp.set_color(BG_LIGHT)
        zoom_line, = zoom_ax.plot([], [], color=color, lw=1.2)

        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._zoom_win = {
            'flat':    flat,
            'win':     win,
            'ax':      zoom_ax,
            'line':    zoom_line,
            'canvas':  canvas,
            'is_stim': is_stim,
        }

        win.bind('<Escape>', lambda e: self._close_zoom())
        win.protocol('WM_DELETE_WINDOW', self._close_zoom)

        self._update_zoom()

    def _close_zoom(self):
        z = self._zoom_win
        if z is not None:
            try:
                z['win'].destroy()
            except tk.TclError:
                pass
            self._zoom_win = None

    def _update_zoom(self):
        """Refresh the zoom popup at ~10 FPS, honouring all display settings."""
        z = self._zoom_win
        if z is None:
            return
        try:
            if not z['win'].winfo_exists():
                self._zoom_win = None
                return
        except tk.TclError:
            self._zoom_win = None
            return

        win_secs   = self._display_secs.get()
        fft_mode   = self._fft_mode.get()
        auto_sc    = self._auto_scale.get()
        uv_half    = self._uv_scale.get()
        fps_est, fps_stable, n_samp = self._get_fps_nsamp(win_secs)
        total_gain = self._pga_gain.get() * GAIN
        bp_on      = self._bp_enabled.get()
        ax         = z['ax']
        line       = z['line']
        flat       = z['flat']

        if z['is_stim']:
            t_prog, arr = self._stim_waveform_for_display(flat, n_samp, fps_est, win_secs)
            if fft_mode:
                freq, mag = compute_fft(arr, fps_stable)
                line.set_xdata(freq); line.set_ydata(mag)
                ax.set_xlim(0, self._fft_xlim(fps_stable))
                ax.set_ylim(0, max(float(mag.max()) * 1.2, 0.1))
                ax.set_xlabel('Hz', color=FG_DIM, fontsize=9)
                ax.set_ylabel('µA', color=FG_DIM, fontsize=9)
            else:
                line.set_xdata(t_prog); line.set_ydata(arr)
                ax.set_xlim(-win_secs, 0)
                mn, mx = float(arr.min()), float(arr.max())
                pad = max((mx - mn) * 0.15, 0.1)
                ax.set_ylim(mn - pad, mx + pad)
                ax.set_xlabel('s', color=FG_DIM, fontsize=9)
                ax.set_ylabel('µA', color=FG_DIM, fontsize=9)
        else:
            raw = self._receiver.get_channel(flat, n_samp) if self._receiver else []
            if fft_mode:
                raw_fft = self._receiver.get_channel(flat, self._get_n_fft(fps_stable)) \
                          if self._receiver else []
                arr = np.array(raw_fft, dtype=float) / FS * VREF / total_gain * 1e6
            else:
                arr = np.array(raw, dtype=float) / FS * VREF / total_gain * 1e6

            if bp_on and len(arr) > 10:
                arr = bandpass_filter(arr, self._bp_low.get(), self._bp_high.get(),
                                      fps_stable, self._bp_order_var.get())

            if fft_mode:
                freq, mag = compute_fft(arr, fps_stable)
                line.set_xdata(freq); line.set_ydata(mag)
                ax.set_xlim(0, self._fft_xlim(fps_stable))
                ax.set_ylim(0, max(float(mag.max()) * 1.2, 1.0))
                ax.set_xlabel('Hz', color=FG_DIM, fontsize=9)
                ax.set_ylabel(self._smart_uv_label(ax.get_ylim()),
                              color=FG_DIM, fontsize=9)
            else:
                n_raw  = len(raw)
                arr    = display_decimate(arr)
                win_actual = n_raw / fps_stable if fps_stable > 0 else win_secs
                t = np.linspace(-win_actual, 0, len(arr))
                line.set_xdata(t); line.set_ydata(arr)
                ax.set_xlim(-win_secs, 0)
                if auto_sc and len(arr) > 0:
                    mn, mx = float(arr.min()), float(arr.max())
                    pad = max((mx - mn) * 0.15, 1.0)
                    ax.set_ylim(mn - pad, mx + pad)
                else:
                    ax.set_ylim(-uv_half, uv_half)
                ax.set_xlabel('s', color=FG_DIM, fontsize=9)
                ax.set_ylabel(self._smart_uv_label(ax.get_ylim()),
                              color=FG_DIM, fontsize=9)

        z['canvas'].draw_idle()
        self.root.after(100, self._update_zoom)   # 10 FPS

    # ==========================================================================
    # FFT / tab switching
    # ==========================================================================

    def _toggle_fft(self):
        new_state = not self._fft_mode.get()
        self._fft_mode.set(new_state)
        label = " FFT  " if new_state else " Time "
        bg    = ACCENT  if new_state else BG_LIGHT
        fg    = BG_DARK if new_state else FG_MAIN
        self._grid_fft_btn.config(text=label, bg=bg, fg=fg)
        self._overlay_fft_btn.config(text=label, bg=bg, fg=fg)
        if self._wf_fft_btn:
            self._wf_fft_btn.config(text=label, bg=bg, fg=fg)
        # Show/hide FFT axis note in grid toolbar
        if new_state:
            self._grid_fft_note.pack(side=tk.LEFT, padx=8)
        else:
            self._grid_fft_note.pack_forget()

    def _on_tab_change(self, event=None):
        idx  = self._notebook.index(self._notebook.select())
        keys = [k for k, _, _ in self._TAB_DEFS]
        key  = keys[idx] if idx < len(keys) else 'grid'
        self._active_tab = key

        # Lazy build: construct tab content on first visit
        if not self._tab_built.get(key, True):
            self._tab_built[key] = True
            builder_attr = {k: b for k, _, b in self._TAB_DEFS}.get(key)
            if builder_attr:
                getattr(self, builder_attr)(self._tab_frames[key])

    # ==========================================================================
    # Render loops
    # ==========================================================================

    # ---- helpers ---------------------------------------------------------

    def _get_fps_nsamp(self, win_secs):
        # Derive FPS from embedded hardware timestamps (µs, uint32).
        # Use up to the last 500 timestamps for a stable rolling estimate.
        # Falls back to ASSUMED_FPS until enough frames have arrived.
        ts  = self._receiver.timestamps
        fps = ASSUMED_FPS
        if len(ts) >= 20:
            ts_sub = list(ts)[-500:]
            dt_us  = ts_sub[-1] - ts_sub[0]
            if dt_us < 0:           # uint32 wraparound at ~71 min
                dt_us += 2**32
            if dt_us > 0:
                fps = max(1.0, (len(ts_sub) - 1) / (dt_us * 1e-6))
        # Snap fps to nearest 5 Hz so n_samp is stable across minor timing jitter.
        # Prevents FFT window from changing by 1-10 samples each render frame,
        # which would otherwise cause the leakage pattern to oscillate.
        fps_stable = max(5.0, round(fps / 5) * 5)
        n_samp = max(50, min(int(fps_stable * win_secs),
                             self._receiver.buffers[0].maxlen))
        # Return both raw fps (for non-FFT uses) and fps_stable (for FFT math).
        # Using fps_stable in _get_n_fft / compute_fft / _fft_xlim ensures
        # n_fft and the frequency axis are completely fixed between render
        # frames, eliminating the "two-state" FFT oscillation caused by minor
        # timing jitter changing n_fft by 10–100 samples each frame.
        return fps, fps_stable, n_samp

    def _get_n_fft(self, fps_est):
        """Number of samples for FFT — always larger than the display window.

        Uses 10 s of buffer data regardless of the display time-window setting.
        At 500 FPS this gives 5 000 samples and df = 0.1 Hz, which places
        every 0.5 Hz multiple (demo channels: 0.5, 1.0, … 6.0 Hz) on an exact
        FFT bin → zero spectral leakage → completely stable FFT magnitude.

        With integer-Hz demo frequencies (1–12 Hz after firmware update) it is
        also stable for any display window size.
        """
        return min(self._receiver.buffers[0].maxlen,
                   max(256, int(fps_est * 10.0)))

    def _fft_xlim(self, fps_est):
        """Stable FFT x-axis upper limit.
        If the user has set a non-zero FFT max Hz, use that.
        Otherwise quantize Nyquist to the nearest 25 Hz so minor
        fps_est fluctuations don't shift the axis every frame."""
        user = self._fft_max_hz.get()
        if user > 0:
            return float(user)
        nyq = fps_est / 2.0
        return max(25.0, round(nyq / 25) * 25)

    @staticmethod
    def _smart_uv_label(ylim):
        """Return 'µV', 'mV', or 'V' depending on the y-axis range (in µV).
        Only changes the label text — data values are never rescaled."""
        abs_max = max(abs(ylim[0]), abs(ylim[1]))
        if abs_max >= 1_000_000:
            return 'V'
        if abs_max >= 1_000:
            return 'mV'
        return 'µV'

    # ---- grid ------------------------------------------------------------

    def _start_grid_loop(self):
        self._update_grid()

    def _update_grid(self):
        interval = 1000 // GRID_FPS
        if self._active_tab != 'grid' or not (self._receiver and self._receiver.connected):
            self.root.after(interval, self._update_grid)
            return

        win_secs   = self._display_secs.get()
        fft_mode   = self._fft_mode.get()
        auto_sc    = self._auto_scale.get()
        uv_half    = self._uv_scale.get()
        fps_est, fps_stable, n_samp = self._get_fps_nsamp(win_secs)
        total_gain = self._pga_gain.get() * GAIN

        self._autoscale_ctr = (self._autoscale_ctr + 1) % 5
        do_auto = auto_sc and (self._autoscale_ctr == 0)

        # -- Electrode subplots --
        for row in range(8):
            for col in range(8):
                flat = ELEC_GRID.get((col, row))
                if flat is None: continue
                line = self._grid_lines.get(flat)
                ax   = self._grid_axes.get(flat)
                txt  = self._grid_texts.get(flat)
                if line is None: continue

                # Time-domain display uses the screen window (n_samp).
                # FFT uses a larger buffer window (_get_n_fft) so df is small
                # enough to place every signal frequency on an exact FFT bin,
                # eliminating the "two-state" spectral-leakage oscillation.
                raw = self._receiver.get_channel(flat, n_samp)

                # Clipping detection: if raw signal saturates ADC AND bandpass is on,
                # the filtered trace looks clean but DC clipping is hidden — warn user.
                bp_on      = self._bp_enabled.get()
                CLIP_THR   = int(FS * 0.97)   # 97 % of full scale = saturation
                is_clipping = bool(raw) and max(abs(v) for v in raw) >= CLIP_THR
                clip_txt    = self._clip_texts.get(flat)
                if clip_txt is not None:
                    if is_clipping and bp_on:
                        clip_txt.set_visible(not clip_txt.get_visible())  # flash
                    else:
                        clip_txt.set_visible(False)

                if fft_mode:
                    raw_fft = self._receiver.get_channel(flat,
                                                         self._get_n_fft(fps_stable))
                    arr = np.array(raw_fft, dtype=float) / FS * VREF / total_gain * 1e6
                else:
                    arr = np.array(raw, dtype=float) / FS * VREF / total_gain * 1e6

                if bp_on:
                    arr = bandpass_filter(arr, self._bp_low.get(),
                                         self._bp_high.get(), fps_stable,
                                         self._bp_order_var.get())

                is_sel = flat in self._selected_elec
                color  = COLORS[(col*8+row) % len(COLORS)] if is_sel else FG_DIMMER
                lw     = 0.9 if is_sel else 0.4

                if fft_mode:
                    freq, mag = compute_fft(arr, fps_stable)
                    line.set_xdata(freq); line.set_ydata(mag)
                    ax.set_xlim(0, self._fft_xlim(fps_stable))
                    if do_auto: ax.set_ylim(0, max(float(mag.max())*1.2, 1.0))
                    elif not auto_sc: ax.set_ylim(0, uv_half)
                else:
                    n_raw = len(raw)   # save pre-decimate count for correct time span
                    arr = display_decimate(arr)
                    # win_actual may be < win_secs when buffer is filling or
                    # was just flushed after a frame gap.  Anchor the waveform
                    # to t=0 so it grows in from the right with no false zeros.
                    # Use n_raw (not len(arr)) because display_decimate shrinks
                    # the array; dividing the decimated length by the original
                    # fps would under-estimate the window and compress the trace
                    # into the right portion of the cell.
                    win_actual = n_raw / fps_stable if fps_stable > 0 else win_secs
                    t = np.linspace(-win_actual, 0, len(arr))
                    line.set_xdata(t); line.set_ydata(arr)
                    ax.set_xlim(-win_secs, 0)
                    if do_auto:
                        mn, mx = float(arr.min()), float(arr.max())
                        pad = max((mx-mn)*0.15, 1.0)
                        ax.set_ylim(mn-pad, mx+pad)
                    elif not auto_sc:
                        ax.set_ylim(-uv_half, uv_half)

                line.set_color(color); line.set_linewidth(lw)
                if txt: txt.set_color(color)

        # -- STIM outer ring -- show programmed output only when channel is active --
        for name, flat in STIM_BY_NAME:
            line = self._stim_grid_lines.get(flat)
            ax   = self._stim_grid_axes.get(flat)
            if line is None: continue

            cfg = self._stim_ch_configs.get(flat, {})
            if cfg.get('type', 'Off') == 'Off':
                line.set_xdata([]); line.set_ydata([])
                continue

            t_prog, arr = self._stim_waveform_for_display(flat, n_samp, fps_est, win_secs)
            if fft_mode:
                freq, mag = compute_fft(arr, fps_stable)
                line.set_xdata(freq); line.set_ydata(mag)
                ax.set_xlim(0, self._fft_xlim(fps_stable))
                if do_auto: ax.set_ylim(0, max(float(mag.max())*1.2, 0.1))
            else:
                line.set_xdata(t_prog); line.set_ydata(arr)
                ax.set_xlim(-win_secs, 0)
                if do_auto:
                    mn, mx = float(arr.min()), float(arr.max())
                    pad = max((mx - mn) * 0.15, 0.1)
                    ax.set_ylim(mn - pad, mx + pad)

        self._grid_canvas.draw_idle()
        self.root.after(interval, self._update_grid)

    # ---- overlay ---------------------------------------------------------

    def _start_overlay_loop(self):
        self._update_overlay()

    def _update_overlay(self):
        interval  = 1000 // OVERLAY_FPS
        if self._active_tab != 'overlay':
            self.root.after(interval, self._update_overlay)
            return

        elec_ch    = sorted(self._selected_elec)
        n_panels   = self._overlay_panels.get()
        fft_mode   = self._fft_mode.get()
        auto_sc    = self._auto_scale.get()
        uv_half    = self._uv_scale.get()
        offset     = 0          # Overlay: channels always overlap (offset=0)
        win_secs   = self._display_secs.get()
        connected  = self._receiver and self._receiver.connected
        total_gain = self._pga_gain.get() * GAIN
        fps_est, fps_stable, n_samp = self._get_fps_nsamp(win_secs)

        # Distribute electrode channels across panels
        n_ch = len(elec_ch)
        if n_ch > 0:
            per = max(1, (n_ch + n_panels - 1) // n_panels)
            panel_channels = [elec_ch[i*per:(i+1)*per] for i in range(n_panels)]
        else:
            panel_channels = [[] for _ in range(n_panels)]

        # -- Electrode panels --
        for p_idx, (ax, ch_list) in enumerate(zip(self._overlay_axes, panel_channels)):
            ax.cla()
            ax.set_facecolor(BG_MID)
            ax.tick_params(colors=FG_DIMMER, labelsize=7)
            for sp in ax.spines.values():
                sp.set_color(BG_LIGHT); sp.set_linewidth(0.6)

            if not connected:
                ax.text(0.5, 0.5, 'Waiting for Pico connection',
                        transform=ax.transAxes, ha='center', va='center',
                        color=FG_DIM, fontsize=9)
                continue
            if not ch_list:
                ax.text(0.5, 0.5, 'No channels selected\n(use electrode grid)',
                        transform=ax.transAxes, ha='center', va='center',
                        color=FG_DIM, fontsize=9, multialignment='center')
                continue

            ch_medians = []   # per-channel median (uV), for robust center
            ch_mads    = []   # per-channel MAD (uV), for robust range
            _n_fft = self._get_n_fft(fps_stable)
            for ch_i, flat in enumerate(ch_list):
                # FFT: use larger buffer (10 s) → df = 0.1 Hz → exact bins.
                # Time: use display window only.
                raw = self._receiver.get_channel(
                    flat, _n_fft if fft_mode else n_samp)
                arr = np.array(raw, dtype=float) / FS * VREF / total_gain * 1e6  # uV
                if self._bp_enabled.get():
                    arr = bandpass_filter(arr, self._bp_low.get(),
                                         self._bp_high.get(), fps_stable,
                                         self._bp_order_var.get())
                color = COLORS[ch_i % len(COLORS)]
                label = ALL_NAMES[flat]
                if fft_mode:
                    freq, mag = compute_fft(arr, fps_stable)
                    shifted = mag + ch_i * abs(offset)
                    ax.plot(freq, shifted, color=color, linewidth=0.8, label=label)
                    ax.set_xlabel('Frequency (Hz)', fontsize=7, color=FG_DIM)
                    ax.set_ylabel(f'Amplitude ({self._smart_uv_label(ax.get_ylim())})',
                                  fontsize=7, color=FG_DIM)
                    ax.set_xlim(0, self._fft_xlim(fps_stable))
                else:
                    med = float(np.median(arr))
                    # When auto-scaling, mean-centre each trace so large DC
                    # offsets (clipped channels) don't collapse the Y-axis span.
                    # Flat / saturated channels have MAD≈0 and would otherwise
                    # drive the auto-scale to its 10 µV floor.
                    arr_disp = display_decimate((arr - med) if auto_sc else arr)
                    win_actual = len(arr) / fps_stable if fps_stable > 0 else win_secs
                    t_disp   = np.linspace(-win_actual, 0, len(arr_disp))
                    shifted = arr_disp + ch_i * offset
                    ax.plot(t_disp, shifted, color=color, linewidth=0.8, label=label)
                    ax.set_xlabel('Time (s)', fontsize=7, color=FG_DIM)
                    _lbl = self._smart_uv_label(ax.get_ylim())
                    ax.set_ylabel(f'{_lbl} (AC)' if auto_sc else _lbl,
                                  fontsize=7, color=FG_DIM)
                    ax.set_xlim(-win_secs, 0)
                    arr_ac = arr - med   # AC component for scale regardless of mode
                    mad = float(np.median(np.abs(arr_ac))) * 1.4826
                    ch_medians.append(0.0 if auto_sc else med)
                    ch_mads.append(mad)
                ax.tick_params(colors=FG_DIMMER, labelsize=7)

            # Y-axis scaling (offset=0 in overlay, so no stacking contribution)
            if not auto_sc:
                ax.set_ylim(0 if fft_mode else -uv_half, uv_half)
            elif ch_medians and not fft_mode:
                # MAD auto-scale: channels are mean-centred, so center≈0.
                # median(ch_mads)*4 gives ~4σ headroom; floor at 10 µV.
                span = max(float(np.median(ch_mads)) * 4.0, 10.0)
                ax.set_ylim(-span, span)

            if ch_list:
                ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0),
                          borderaxespad=0, fontsize=6, ncol=1,
                          facecolor=BG_LIGHT, edgecolor=BG_LIGHT,
                          labelcolor=FG_MAIN, framealpha=0.85, handlelength=1.2)

        # -- STIM panel (only visible when at least one STIM channel is active) --
        any_stim_active = any(
            self._stim_ch_configs.get(f, {}).get('type', 'Off') != 'Off'
            for _, f in STIM_BY_NAME)
        stim_ax = self._stim_overlay_ax
        stim_ax.cla()
        if any_stim_active:
            stim_ax.set_facecolor(STIM_BG)
            stim_ax.tick_params(colors=ORANGE, labelsize=7)
            for sp in stim_ax.spines.values():
                sp.set_color(ORANGE); sp.set_linewidth(0.9)
            stim_ax.set_title("STIM Monitor  (uA)", color=ORANGE, fontsize=7, pad=2)

            for i, (name, flat) in enumerate(STIM_BY_NAME):
                cfg = self._stim_ch_configs.get(flat, {})
                if cfg.get('type', 'Off') == 'Off':
                    continue
                t_prog, arr = self._stim_waveform_for_display(flat, n_samp, fps_est, win_secs)
                color = COLORS[i % len(COLORS)]
                if fft_mode:
                    freq, mag = compute_fft(arr, fps_stable)
                    stim_ax.plot(freq, mag, color=color, linewidth=0.8, label=name)
                    stim_ax.set_xlim(0, self._fft_xlim(fps_stable))
                    stim_ax.set_xlabel('Frequency (Hz)', fontsize=7, color=ORANGE)
                else:
                    stim_ax.plot(t_prog, arr, color=color, linewidth=0.8, label=name)
                    stim_ax.set_xlim(-win_secs, 0)
                    stim_ax.set_xlabel('Time (s)', fontsize=7, color=ORANGE)
            stim_ax.set_ylabel('uA', fontsize=9, color=ORANGE)
            stim_ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0),
                           borderaxespad=0, fontsize=6, ncol=1,
                           facecolor=STIM_BG, edgecolor=ORANGE,
                           labelcolor=FG_MAIN, framealpha=0.85, handlelength=1.2)
        else:
            stim_ax.set_facecolor(BG_MID)
            stim_ax.tick_params(left=False, bottom=False,
                                labelleft=False, labelbottom=False)
            for sp in stim_ax.spines.values():
                sp.set_visible(False)
            stim_ax.text(0.5, 0.5, 'No STIM channels active',
                         transform=stim_ax.transAxes,
                         ha='center', va='center', fontsize=8, color=FG_DIMMER)

        self._overlay_canvas.draw_idle()
        self.root.after(interval, self._update_overlay)

    # ---- waterfall -------------------------------------------------------

    def _start_waterfall_loop(self):
        self._update_waterfall()

    def _update_waterfall(self):
        interval   = 1000 // OVERLAY_FPS
        if self._active_tab != 'waterfall':
            self.root.after(interval, self._update_waterfall)
            return
        if self._wf_fig is None or not self._wf_axes:
            self.root.after(interval, self._update_waterfall)
            return

        ax         = self._wf_axes[0]
        elec_ch    = sorted(self._selected_elec)
        fft_mode   = self._fft_mode.get()
        auto_sc    = self._auto_scale.get()
        uv_half    = self._uv_scale.get()
        offset     = self._waterfall_var.get()
        win_secs   = self._display_secs.get()
        connected  = self._receiver and self._receiver.connected
        total_gain = self._pga_gain.get() * GAIN
        fps_est, fps_stable, n_samp = self._get_fps_nsamp(win_secs)

        ax.cla()
        ax.set_facecolor(BG_MID)
        ax.tick_params(colors=FG_DIMMER, labelsize=7)
        for sp in ax.spines.values():
            sp.set_color(BG_LIGHT); sp.set_linewidth(0.6)

        if not connected:
            ax.text(0.5, 0.5, 'Waiting for Pico connection',
                    transform=ax.transAxes, ha='center', va='center',
                    color=FG_DIM, fontsize=11)
            self._wf_canvas.draw_idle()
            self.root.after(interval, self._update_waterfall)
            return
        if not elec_ch:
            ax.text(0.5, 0.5, 'No channels selected\n(click electrodes in Grid Layout)',
                    transform=ax.transAxes, ha='center', va='center',
                    color=FG_DIM, fontsize=11, multialignment='center')
            self._wf_canvas.draw_idle()
            self.root.after(interval, self._update_waterfall)
            return

        ch_mads = []

        _n_fft = self._get_n_fft(fps_stable)
        for ch_i, flat in enumerate(elec_ch):
            raw = self._receiver.get_channel(
                flat, _n_fft if fft_mode else n_samp)
            arr = np.array(raw, dtype=float) / FS * VREF / total_gain * 1e6
            if self._bp_enabled.get():
                arr = bandpass_filter(arr, self._bp_low.get(),
                                      self._bp_high.get(), fps_stable,
                                      self._bp_order_var.get())
            color = COLORS[ch_i % len(COLORS)]
            med   = float(np.median(arr))
            if fft_mode:
                freq, mag = compute_fft(arr, fps_stable)
                shifted = mag + ch_i * abs(offset)
                ax.plot(freq, shifted, color=color, linewidth=0.8,
                        label=ALL_NAMES[flat])
            else:
                # Always mean-centre so offset controls visual separation only
                arr_dec  = display_decimate(arr - med)
                win_actual = len(arr) / fps_stable if fps_stable > 0 else win_secs
                t_dec    = np.linspace(-win_actual, 0, len(arr_dec))
                shifted  = arr_dec + ch_i * offset
                ax.plot(t_dec, shifted, color=color, linewidth=0.8)
                arr_ac = arr - med
                ch_mads.append(float(np.median(np.abs(arr_ac))) * 1.4826)

        if fft_mode:
            ax.set_xlabel('Frequency (Hz)', fontsize=8, color=FG_DIM)
            ax.set_ylabel(f'Amplitude ({self._smart_uv_label(ax.get_ylim())})',
                          fontsize=8, color=FG_DIM)
            ax.set_xlim(0, self._fft_xlim(fps_stable))
            ax.legend(loc='upper right', fontsize=6, ncol=2,
                      facecolor=BG_LIGHT, edgecolor=BG_LIGHT,
                      labelcolor=FG_MAIN, framealpha=0.85, handlelength=1.2)
        else:
            ax.set_xlabel('Time (s)', fontsize=8, color=FG_DIM)
            ax.set_ylabel(f'{self._smart_uv_label(ax.get_ylim())} (AC)',
                          fontsize=8, color=FG_DIM)
            ax.set_xlim(-win_secs, 0)

            # Y-ticks: one per channel, labelled with channel name
            if offset != 0 and elec_ch:
                ticks = [i * offset for i in range(len(elec_ch))]
                ax.set_yticks(ticks)
                ax.set_yticklabels([ALL_NAMES[f] for f in elec_ch],
                                   fontsize=6, color=FG_DIM)
            else:
                ax.tick_params(axis='y', labelsize=7)

            # Y limits
            if not auto_sc:
                half = uv_half
            else:
                ac_spread = max(float(np.median(ch_mads)) * 4.0, 10.0) if ch_mads else uv_half
                half = max(ac_spread, abs(offset) * 0.5)
            total_stack = abs(offset) * max(len(elec_ch) - 1, 0)
            ax.set_ylim(-half, total_stack + half)

            # Per-channel amplitude scale bar — vertical line ± half µV
            # anchored at channel-0 baseline (y=0), left edge of the plot.
            _sb_x = -win_secs + win_secs * 0.015
            ax.plot([_sb_x, _sb_x], [-half, half],
                    color=FG_DIM, lw=2, solid_capstyle='butt', clip_on=True)
            _lbl = f'±{half:.0f} µV' if half < 1000 else f'±{half/1000:.1f} mV'
            ax.text(_sb_x + win_secs * 0.015, 0,
                    _lbl, color=FG_DIM, fontsize=7, va='center', ha='left')

        self._wf_canvas.draw_idle()
        self.root.after(interval, self._update_waterfall)

    # ---- heatmap ---------------------------------------------------------

    def _start_heatmap_loop(self):
        self._update_heatmap()

    def _update_heatmap(self):
        interval = 1000 // HEATMAP_FPS
        if self._active_tab != 'heatmap' or not (self._receiver and self._receiver.connected):
            self.root.after(interval, self._update_heatmap)
            return

        metric     = self._heatmap_metric.get()
        cmap       = self._cmap_var.get()
        win_secs   = min(self._display_secs.get(), 3.0)
        fps_est, fps_stable, n_samp = self._get_fps_nsamp(win_secs)
        total_gain = self._pga_gain.get() * GAIN
        heat       = np.zeros((10, 10))

        for row in range(8):
            for col in range(8):
                flat = ELEC_GRID.get((col, row))
                if flat is None: continue
                raw = self._receiver.get_channel(flat, n_samp)
                arr = np.array(raw, dtype=float) / FS * VREF / total_gain * 1e6

                if metric == "RMS":
                    heat[row+1, col+1] = float(np.sqrt(np.mean(arr**2)))
                elif metric == "Bandpass-RMS":
                    filt = bandpass_filter(arr, self._bp_low.get(),
                                          self._bp_high.get(), fps_stable,
                                          self._bp_order_var.get())
                    heat[row+1, col+1] = float(np.sqrt(np.mean(filt**2)))
                elif metric == "Peak":
                    heat[row+1, col+1] = float(np.max(np.abs(arr)))
                elif metric == "FFT-Peak-Freq":
                    freq, mag = compute_fft(arr, fps_stable)
                    if len(freq) > 1 and mag.sum() > 0:
                        heat[row+1, col+1] = float(freq[int(np.argmax(mag))])

        heat_masked = np.ma.array(heat, mask=self._heatmap_mask)
        try:
            _cmap_obj = matplotlib.colormaps[cmap].copy()
        except (AttributeError, KeyError):
            _cmap_obj = matplotlib.cm.get_cmap(cmap).copy()
        _cmap_obj.set_bad('#0d0d14')
        self._heatmap_im.set_data(heat_masked)
        self._heatmap_im.set_cmap(_cmap_obj)
        vmin, vmax = float(heat.min()), float(heat.max())
        self._heatmap_im.set_clim(vmin, max(vmax, vmin + 1.0))
        units = "uV" if metric in ("RMS","Bandpass-RMS","Peak") else "Hz"
        self._heatmap_cbar.set_label(f"{metric} ({units})", color=FG_DIM, fontsize=8)

        # Update STIM bar chart -- show programmed amplitude (uA), not ADC data
        stim_vals = []
        for name, flat in STIM_BY_NAME:
            cfg = self._stim_ch_configs.get(flat, {})
            wtype = cfg.get('type', 'Off')
            if wtype == 'Off':
                stim_vals.append(0.0)
            else:
                try:    stim_vals.append(float(cfg.get('amp', 0)))
                except: stim_vals.append(0.0)

        if self._heatmap_stim_bars and stim_vals:
            for bar, h in zip(self._heatmap_stim_bars, stim_vals):
                bar.set_height(h)
            self._heatmap_stim_ax.set_ylim(0, max(max(stim_vals)*1.2, 1.0))

        self._heatmap_canvas.draw_idle()
        self.root.after(interval, self._update_heatmap)

    # ==========================================================================
    # Connection management
    # ==========================================================================

    def _start_receiver(self):
        # 1000 samples/sec headroom — handles anything up to ~1000 FPS.
        # ASSUMED_FPS is no longer used here; buf_size is rate-agnostic.
        buf_size = int(BUFFER_SECS * 1000)
        self._receiver = DataReceiver(self.data_port, buf_size)
        self._receiver.start()

    def _start_cmd_client(self):
        self._cmd_client = CommandClient(
            self.cmd_host or '', self.cmd_port, self._cmd_rx_q)
        self._cmd_client.start()

    def _connect_cmd(self):
        if self._demo_mode:
            self._term_append("[Demo] Cannot connect in demo mode\n", 'err'); return
        # Toggle: disconnect if currently connected
        if self._cmd_client.connected:
            self._cmd_client.disconnect()
            self._cmd_connect_btn.config(text="Connect")
            self._cmd_status_dot.config(fg=FG_DIMMER)
            self._cmd_status_lbl.config(text="Not connected")
            return
        host = self._cmd_host_var.get().strip()
        try:
            port = int(self._cmd_port_var.get())
        except ValueError:
            self._term_append("[Error] Invalid port\n", 'err'); return
        if not host:
            self._term_append("[Error] Enter Pico IP first\n", 'err'); return
        self._cmd_client.set_host(host, port)
        self._term_append(f"[CMD] Connecting to {host}:{port}...\n", 'info')

    # ==========================================================================
    # Demo mode
    # ==========================================================================

    def _toggle_demo(self):
        if self._demo_mode: self._stop_demo()
        else:               self._start_demo()

    def _start_demo(self):
        self._demo_mode = True
        self._demo_btn.config(text="SW Demo mode  ON", bg=GREEN, fg=BG_DARK)
        # Pause live TCP ingestion so demo and real data don't interleave
        self._receiver._paused = True
        for _buf in self._receiver.buffers:
            _buf.clear()
        self._receiver.timestamps.clear()
        self._demo_injector = DemoDataInjector(self._receiver)
        self._demo_injector.start()
        self._cmd_client.stop()
        self._cmd_client = DemoCmdClient(self._cmd_rx_q)
        # Show command channel as "connected" so Self-Test tab is fully usable
        self._cmd_status_dot.config(fg=GREEN)
        self._cmd_status_lbl.config(text="Demo — Pico simulated")
        self._term_append("[DEMO] Started -- simulating 8 column waveform types\n", 'info')

    def _stop_demo(self):
        self._demo_mode = False
        self._demo_btn.config(text="SW Demo mode  [Ctrl+D]", bg=BG_LIGHT, fg=FG_MAIN)
        if self._demo_injector:
            self._demo_injector.stop(); self._demo_injector = None
        # Resume live TCP ingestion
        self._receiver._paused = False
        self._cmd_client.stop()
        self._start_cmd_client()
        self._cmd_status_dot.config(fg=FG_DIMMER)
        self._cmd_status_lbl.config(text="Not connected")
        self._term_append("[DEMO] Stopped\n", 'info')

    # ==========================================================================
    # Webcam
    # ==========================================================================

    def _on_cam_enable(self):
        if not self._webcam: return
        if self._cam_enable_var.get():
            self._webcam.interval_s = max(1, self._cam_interval_var.get()) * 60
            self._webcam.cam_index  = self._cam_idx_var.get()
            self._webcam.enable()
            self._cam_status_var.set(f"Capturing every {self._cam_interval_var.get()} min")
        else:
            self._webcam.disable()
            self._cam_status_var.set("Disabled")

    def _cam_capture_now(self):
        if self._webcam:
            self._webcam.cam_index  = self._cam_idx_var.get()
            self._webcam.interval_s = max(1, self._cam_interval_var.get()) * 60
            self._webcam.capture_now()
            self._cam_status_var.set("Capturing...")

    def _poll_webcam(self):
        if self._webcam:
            if self._webcam.last_error:
                self._cam_status_var.set(f"Error: {self._webcam.last_error}")
            elif self._webcam.last_capture_path:
                self._cam_status_var.set(
                    f"Last: {os.path.basename(self._webcam.last_capture_path)}"
                    f" ({self._webcam.capture_count} total)")
        self.root.after(4000, self._poll_webcam)

    def _cam_grab_frame(self):
        """Return a PIL Image from the camera, or None on failure.

        In demo mode a synthetic image is generated with PIL so no real
        camera or opencv install is required.
        """
        if self._demo_mode:
            try:
                from PIL import Image, ImageDraw
                W, H = 640, 480
                img  = Image.new('RGB', (W, H), color=(13, 13, 20))
                draw = ImageDraw.Draw(img)
                # Subtle grid (microscope aesthetic)
                for x in range(0, W, 64):
                    draw.line([(x, 0), (x, H)], fill=(25, 25, 42), width=1)
                for y in range(0, H, 48):
                    draw.line([(0, y), (W, y)], fill=(25, 25, 42), width=1)
                # Cross-hair + reticle circle
                cx, cy = W // 2, H // 2
                draw.line([(cx - 30, cy), (cx + 30, cy)], fill=(255, 140, 0), width=2)
                draw.line([(cx, cy - 30), (cx, cy + 30)], fill=(255, 140, 0), width=2)
                draw.ellipse([(cx - 50, cy - 50), (cx + 50, cy + 50)],
                             outline=(255, 140, 0), width=1)
                # Labels
                ts = datetime.now().strftime('%Y-%m-%d  %H:%M:%S')
                draw.text((10, 10),       "DEMO  |  CAMERA PREVIEW", fill=(255, 140, 0))
                draw.text((10, 28),       ts,                         fill=(160, 160, 200))
                draw.text((10, H - 22),
                          f"MushIO V1.0  |  cam idx {self._cam_idx_var.get()}",
                          fill=(70, 70, 100))
                return img
            except ImportError:
                return None

        if not HAS_CV2:
            return None
        try:
            import cv2
            cap = cv2.VideoCapture(self._cam_idx_var.get())
            ok, frame = cap.read()
            cap.release()
            if not ok:
                return None
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            from PIL import Image
            return Image.fromarray(frame)
        except Exception:
            return None

    def _cam_take_and_preview(self):
        """Grab a frame, show it in the left-panel preview, and optionally save it."""
        img = self._cam_grab_frame()
        if img is None:
            self._cam_status_var.set("Camera open failed")
            return
        self._cam_update_preview_label(img)
        # Also save to disk if periodic capture is enabled
        if self._cam_enable_var.get() and self._webcam:
            self._webcam.capture_now()
        self._cam_status_var.set("Image captured")

    def _cam_update_preview_label(self, img):
        """Resize PIL image to fit the sidebar label and display it."""
        if self._cam_preview_label is None:
            return
        try:
            from PIL import ImageTk
            target_w = 230
            w, h = img.size
            img = img.resize((target_w, int(h * target_w / w)), Image.LANCZOS
                              if hasattr(Image, 'LANCZOS') else Image.ANTIALIAS)
            self._cam_preview_photo = ImageTk.PhotoImage(img)
            self._cam_preview_label.config(
                image=self._cam_preview_photo, text='',
                width=target_w, height=img.height)
        except ImportError:
            self._cam_status_var.set("pip install Pillow for preview")
        except Exception as exc:
            self._cam_status_var.set(f"Preview error: {exc}")

    def _schedule_cam_preview(self):
        """Auto-refresh the left-panel preview at the user-specified interval."""
        if (not HAS_CV2 and not HAS_PIL) or self._cam_preview_label is None:
            return
        img = self._cam_grab_frame()
        if img is not None:
            self._cam_update_preview_label(img)
        # Reschedule (interval in minutes, minimum 1)
        try:
            mins = max(1, self._cam_preview_interval_var.get())
        except Exception:
            mins = 10
        self.root.after(mins * 60 * 1000, self._schedule_cam_preview)

    def _cam_refresh_preview(self):
        """Alias used by Settings tab Refresh Preview button."""
        self._cam_take_and_preview()

    # ==========================================================================
    # Terminal
    # ==========================================================================

    def _on_send(self):
        cmd = self._cmd_entry.get().strip()
        if not cmd: return
        self._cmd_entry.delete(0, tk.END)
        self._cmd_history.append(cmd)
        self._history_idx = len(self._cmd_history)
        self._term_append(f"> {cmd}\n", 'cmd')
        self._send_cmd(cmd)

    def _send_cmd(self, cmd):
        if self._cmd_client and self._cmd_client.connected:
            self._cmd_client.send(cmd)
        else:
            self._term_append("[Not connected]\n", 'err')

    def _do_adc_cal(self, cal_type):
        """Send a self-calibration command to all 6 ADCs.

        cal_type: 'selfocal' (offset) or 'selfgcal' (full-scale gain).
        Results are stored by the firmware in OFCAL0-2 / FSCAL0-2 registers.
        """
        self._send_cmd(cal_type)

    def _term_append(self, text, tag=None):
        if self._term_out is None:
            self._term_buffer.append((text, tag))
            return
        self._term_out.configure(state=tk.NORMAL)
        if tag: self._term_out.insert(tk.END, text, tag)
        else:   self._term_out.insert(tk.END, text)
        self._term_out.see(tk.END)
        self._term_out.configure(state=tk.DISABLED)

    def _clear_terminal(self):
        self._term_out.configure(state=tk.NORMAL)
        self._term_out.delete('1.0', tk.END)
        self._term_out.configure(state=tk.DISABLED)

    def _history_up(self, event):
        if self._cmd_history and self._history_idx > 0:
            self._history_idx -= 1
            self._cmd_entry.delete(0, tk.END)
            self._cmd_entry.insert(0, self._cmd_history[self._history_idx])

    def _history_down(self, event):
        if self._history_idx < len(self._cmd_history) - 1:
            self._history_idx += 1
            self._cmd_entry.delete(0, tk.END)
            self._cmd_entry.insert(0, self._cmd_history[self._history_idx])
        else:
            self._history_idx = len(self._cmd_history)
            self._cmd_entry.delete(0, tk.END)

    def _poll_cmd_queue(self):
        while not self._cmd_rx_q.empty():
            line = self._cmd_rx_q.get_nowait()
            # Feed register lines to register tab parser (silent, no terminal echo needed)
            if line.startswith('ADC') and '0x' in line and '=' in line:
                self._regs_ingest_response(line)
                self._regs_status.set(f"Registers updated from hardware.")
                self._term_append(line + '\n', 'info')
                continue
            if line == 'END':
                self._term_append("--- end ---\n", 'end')
            elif line.startswith('ERROR'):
                self._term_append(line + '\n', 'err')
            elif line.startswith('[TEST]'):
                self._term_append(line + '\n', 'info')
                # Mirror last result line to the Settings tab status label
                self._hw_test_status_var.set(line)
            elif line.startswith('[OTA]'):
                self._term_append(line + '\n', 'info')
                self._ota_status_var.set(line)
            elif line.startswith('[CRASH]'):
                self._term_append(line + '\n', 'err')
            elif line.startswith('[CMD]') or line.startswith('[REGS]'):
                self._term_append(line + '\n', 'info')
                if '[CMD]' in line:
                    if 'Connected to' in line:
                        self._cmd_status_dot.config(fg=GREEN)
                        self._cmd_status_lbl.config(text=line)
                        self._cmd_connect_btn.config(text="Disconnect")
                    elif 'Disconnected' in line or 'Connecting' in line:
                        self._cmd_status_dot.config(fg=RED if 'Disconnected' in line else FG_DIMMER)
                        self._cmd_status_lbl.config(text=line)
                        self._cmd_connect_btn.config(text="Connect")
            else:
                self._term_append(line + '\n')
        self.root.after(50, self._poll_cmd_queue)

    # ==========================================================================
    # Status bar
    # ==========================================================================

    def _poll_status(self):
        if self._receiver:
            r = self._receiver
            dot_color = GREEN if r.connected else FG_DIMMER
            self._data_status_dot.config(fg=dot_color)
            self._data_status_lbl.config(
                text=f"Streaming from {r.client_ip}" if r.connected else "Waiting for Pico...")
            fps_str = f"{r.fps:.1f}" if r.fps > 0 else "--"
            self._stat_vars['fps'    ].set(f"FPS: {fps_str}")
            self._stat_vars['frames' ].set(f"Frames: {r.frames_total}")
            self._stat_vars['crc'    ].set(f"CRC err: {r.crc_errors}")
            self._stat_vars['missed' ].set(f"Missed: {r.missed_session}")
            self._stat_vars['session'].set(f"Session: {r.frames_session}")

            # Auto-start binary logging when a live Pico connects.
            # This ensures every received frame is written to disk immediately,
            # protecting data even if the GUI is closed unexpectedly.
            # Only triggers once per connection event; demo mode is excluded.
            if r.connected and not self._demo_mode:
                if not self._auto_log_started:
                    self._auto_log_started = True
                    if not self._log_active:
                        self._start_recording()
            elif not r.connected:
                self._auto_log_started  = False  # reset so next connect auto-starts
                self._cmd_auto_next_try = 0.0    # allow immediate auto-connect on next data link
                # Clear any streaming pause so next connection starts unpaused
                if self._stream_paused_var.get():
                    self._stream_paused_var.set(False)
                    r._paused = False
                    if hasattr(self, '_pause_btn'):
                        self._pause_btn.config(text="  Pause Streaming  ",
                                               bg=BG_LIGHT, fg=FG_MAIN)

            # Auto-detect Pico IP from the data connection and wire up CMD channel.
            # When the Pico connects on port 9000 its source IP is stored in
            # r.client_ip.  We use that to:
            #   1. Pre-fill the "Command Pico IP" entry (if the user left it blank)
            #   2. Auto-connect the CMD client so ping/status/blink-LED just work
            pico_ip = r.client_ip
            if (r.connected and pico_ip and pico_ip != "demo"
                    and not self._demo_mode
                    and hasattr(self, '_cmd_client') and self._cmd_client
                    and not self._cmd_client.connected
                    and time.time() >= self._cmd_auto_next_try):
                # Fill field only if the user hasn't typed something already
                if not self._cmd_host_var.get().strip():
                    self._cmd_host_var.set(pico_ip)
                try:
                    cmd_port = int(self._fw_cmd_port.get())
                except (ValueError, AttributeError):
                    cmd_port = 9001
                self._cmd_client.set_host(pico_ip, cmd_port)
                self._cmd_auto_next_try = time.time() + 10.0  # back-off: retry at most every 10 s
                self._term_append(
                    f"[CMD] Auto-detected Pico at {pico_ip} — connecting CMD channel...\n",
                    'info')

        self.root.after(500, self._poll_status)

    # ==========================================================================
    # Close
    # ==========================================================================

    def _on_close(self):
        if self._log_active:
            self._stop_recording()
        if self._demo_injector: self._demo_injector.stop()
        if self._webcam:        self._webcam.stop()
        if hasattr(self, '_cmd_client') and self._cmd_client:
            self._cmd_client.stop()
        if self._receiver: self._receiver.stop()
        self.root.destroy()


# =============================================================================
# Entry point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="MushIO V1.0 Real-Time Biopotential Monitor")
    parser.add_argument('--data-port', type=int, default=9000)
    parser.add_argument('--cmd-host',  type=str, default='')
    parser.add_argument('--cmd-port',  type=int, default=9001)
    parser.add_argument('--demo',      action='store_true',
                        help='Start in demo mode')
    args = parser.parse_args()

    root = tk.Tk()
    root.withdraw()   # hide while building -- prevents white-box flash

    app = MushIOGUI(root,
                    data_port=args.data_port,
                    cmd_host=args.cmd_host,
                    cmd_port=args.cmd_port)

    root.deiconify()  # show complete window all at once

    if args.demo:
        root.after(200, app._start_demo)

    root.mainloop()


if __name__ == '__main__':
    main()
