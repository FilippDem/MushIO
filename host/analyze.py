"""
MushIO V1.0  —  Post-Processing Analysis Tool

Reads one or more .bin raw-frame files recorded by receiver.py (--raw flag),
parses every 228-byte frame, and produces per-channel statistics and optional
plots / CSV export.

Usage
-----
    python analyze.py recording.bin [recording2.bin ...]
    python analyze.py recording.bin --csv summary.csv
    python analyze.py recording.bin --plot
    python analyze.py recording.bin --channels 0,1,2,32 --plot
    python analyze.py recording.bin --secs 10            # analyse first 10 s

Requirements
------------
    Python 3.8+   (no external dependencies for stats)
    matplotlib    (optional, only needed for --plot)
    numpy         (optional, faster path used if present)
"""

import argparse
import math
import os
import struct
import sys
from datetime import timedelta

# ---------------------------------------------------------------------------
# Constants — must match config.h / receiver.py
# ---------------------------------------------------------------------------

SYNC_WORD        = 0xAA55
NUM_ADCS         = 6
CHANNELS_PER_ADC = 12
TOTAL_CHANNELS   = NUM_ADCS * CHANNELS_PER_ADC   # 72

HEADER_SIZE      = 10
DATA_SIZE        = TOTAL_CHANNELS * 3             # 216
CRC_SIZE         = 2
FRAME_SIZE       = HEADER_SIZE + DATA_SIZE + CRC_SIZE  # 228

VREF             = 5.08    # V  (AVDD_2V5 − AVSS_N2V5)
AFE_GAIN         = 11.0
ADC_FULL_SCALE   = 2 ** 23  # 24-bit signed positive full scale

CHANNEL_MAP = [
    ["ELEC02", "ELEC23", "ELEC22", "ELEC12",
     "ELEC11", "ELEC21", "ELEC20", "ELEC01",
     "ELEC00", "ELEC10", "ELEC03", "ELEC13"],
    ["ELEC26", "ELEC16", "ELEC25", "ELEC15",
     "ELEC24", "ELEC05", "ELEC04", "ELEC14",
     "ELEC17", "ELEC07", "ELEC06", "ELEC27"],
    ["ELEC41", "ELEC31", "ELEC30", "ELEC40",
     "ELEC43", "ELEC33", "ELEC32", "ELEC42",
     "STIM_1", "STIM_0", "STIM_7", "STIM_6"],
    ["ELEC45", "ELEC35", "ELEC34", "ELEC44",
     "ELEC47", "ELEC37", "ELEC36", "ELEC46",
     "STIM_3", "STIM_2", "STIM_5", "STIM_4"],
    ["ELEC71", "ELEC50", "ELEC60", "ELEC70",
     "ELEC62", "ELEC52", "ELEC51", "ELEC61",
     "ELEC73", "ELEC63", "ELEC53", "ELEC72"],
    ["ELEC75", "ELEC54", "ELEC64", "ELEC74",
     "ELEC66", "ELEC56", "ELEC55", "ELEC65",
     "ELEC67", "ELEC77", "ELEC76", "ELEC57"],
]

CHANNEL_NAMES = []
for _adc_map in CHANNEL_MAP:
    CHANNEL_NAMES.extend(_adc_map)

STIM_CHANNEL_INDICES = [
    2 * CHANNELS_PER_ADC + 8,   # 32: STIM_1
    2 * CHANNELS_PER_ADC + 9,   # 33: STIM_0
    2 * CHANNELS_PER_ADC + 10,  # 34: STIM_7
    2 * CHANNELS_PER_ADC + 11,  # 35: STIM_6
    3 * CHANNELS_PER_ADC + 8,   # 44: STIM_3
    3 * CHANNELS_PER_ADC + 9,   # 45: STIM_2
    3 * CHANNELS_PER_ADC + 10,  # 46: STIM_5
    3 * CHANNELS_PER_ADC + 11,  # 47: STIM_4
]

RECORDING_CHANNEL_INDICES = [
    i for i in range(TOTAL_CHANNELS) if i not in STIM_CHANNEL_INDICES
]


# ---------------------------------------------------------------------------
# CRC-16 CCITT
# ---------------------------------------------------------------------------

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Frame parsing
# ---------------------------------------------------------------------------

def parse_frame(raw: bytes):
    """
    Parse one 228-byte frame.  Returns dict or None on bad sync / wrong length.
    """
    if len(raw) != FRAME_SIZE:
        return None
    sync = struct.unpack_from('<H', raw, 0)[0]
    if sync != SYNC_WORD:
        return None

    timestamp_us = struct.unpack_from('<I', raw, 2)[0]
    seq          = struct.unpack_from('<H', raw, 6)[0]

    samples = []
    for i in range(TOTAL_CHANNELS):
        off   = HEADER_SIZE + i * 3
        value = (raw[off] << 16) | (raw[off + 1] << 8) | raw[off + 2]
        if value & 0x800000:
            value -= 0x1000000
        samples.append(value)

    crc_rx   = struct.unpack_from('<H', raw, HEADER_SIZE + DATA_SIZE)[0]
    crc_ok   = (crc_rx == crc16_ccitt(raw[:HEADER_SIZE + DATA_SIZE]))

    return {
        'timestamp_us': timestamp_us,
        'seq':          seq,
        'samples':      samples,
        'crc_ok':       crc_ok,
    }


def counts_to_uv(raw_count: int) -> float:
    """Convert raw 24-bit signed ADC count to input voltage in µV."""
    if raw_count == -8388608:   # 0x800000 error marker
        return float('nan')
    adc_v = (raw_count / ADC_FULL_SCALE) * VREF
    return (adc_v / AFE_GAIN) * 1e6   # µV


# ---------------------------------------------------------------------------
# File reading
# ---------------------------------------------------------------------------

def read_frames(path: str, max_secs: float = None):
    """
    Generator: yield parsed frame dicts from a .bin file.
    Skips frames with bad sync or CRC errors.
    Stops early if max_secs is set (based on firmware timestamp).
    """
    file_size = os.path.getsize(path)
    n_frames  = file_size // FRAME_SIZE
    print(f"  {os.path.basename(path)}: {file_size} bytes → "
          f"{n_frames} potential frames ({file_size % FRAME_SIZE} trailing bytes)")

    first_ts = None
    with open(path, 'rb') as f:
        while True:
            raw = f.read(FRAME_SIZE)
            if len(raw) < FRAME_SIZE:
                break
            frame = parse_frame(raw)
            if frame is None or not frame['crc_ok']:
                continue
            if first_ts is None:
                first_ts = frame['timestamp_us']
            if max_secs is not None:
                elapsed_s = (frame['timestamp_us'] - first_ts) / 1e6
                if elapsed_s > max_secs:
                    break
            yield frame


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

class ChannelStats:
    """Accumulates per-channel statistics without storing all samples."""

    def __init__(self, n_channels: int):
        self.n = n_channels
        self.count   = [0]   * n_channels
        self.sum_    = [0.0] * n_channels
        self.sum2    = [0.0] * n_channels
        self.min_    = [float('inf')]  * n_channels
        self.max_    = [float('-inf')] * n_channels
        self.nan_ct  = [0]   * n_channels

    def update(self, samples):
        for i, raw in enumerate(samples):
            v = counts_to_uv(raw)
            if math.isnan(v):
                self.nan_ct[i] += 1
                continue
            self.count[i]  += 1
            self.sum_[i]   += v
            self.sum2[i]   += v * v
            if v < self.min_[i]: self.min_[i] = v
            if v > self.max_[i]: self.max_[i] = v

    def mean(self, i):
        c = self.count[i]
        return self.sum_[i] / c if c else float('nan')

    def std(self, i):
        c = self.count[i]
        if c < 2:
            return float('nan')
        var = self.sum2[i] / c - (self.sum_[i] / c) ** 2
        return math.sqrt(max(var, 0.0))

    def rms(self, i):
        c = self.count[i]
        return math.sqrt(self.sum2[i] / c) if c else float('nan')


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def analyse_files(paths, channels=None, max_secs=None, csv_out=None, do_plot=False):
    """
    Analyse one or more .bin files.  channels = list of int indices (None = all).
    """
    ch_indices = channels if channels is not None else list(range(TOTAL_CHANNELS))

    stats      = ChannelStats(TOTAL_CHANNELS)
    total_frames = 0
    seq_prev   = None
    missed     = 0
    timestamps = []   # ms, for FPS estimation

    print("\nReading frames...")
    for path in paths:
        for frame in read_frames(path, max_secs=max_secs):
            stats.update(frame['samples'])
            total_frames += 1
            timestamps.append(frame['timestamp_us'] / 1000.0)

            if seq_prev is not None:
                gap = (frame['seq'] - seq_prev - 1) & 0xFFFF
                missed += gap
            seq_prev = frame['seq']

    if total_frames == 0:
        print("No valid frames found.")
        return

    # Estimate FPS from firmware timestamps
    if len(timestamps) >= 2:
        duration_s = (timestamps[-1] - timestamps[0]) / 1000.0
        actual_fps = (total_frames - 1) / duration_s if duration_s > 0 else 0.0
    else:
        duration_s = 0.0
        actual_fps = 0.0

    print(f"\n{'='*60}")
    print(f"  MushIO Analysis Summary")
    print(f"{'='*60}")
    print(f"  Files         : {', '.join(os.path.basename(p) for p in paths)}")
    print(f"  Valid frames  : {total_frames}")
    print(f"  Missed frames : {missed}")
    print(f"  Duration      : {timedelta(seconds=int(duration_s))}"
          f"  ({duration_s:.1f} s)")
    print(f"  Effective FPS : {actual_fps:.1f}")
    print(f"{'='*60}")
    print(f"\n{'Ch':>4}  {'Name':>8}  {'Mean µV':>10}  {'Std µV':>10}  "
          f"{'RMS µV':>10}  {'Min µV':>10}  {'Max µV':>10}  {'Samples':>8}")
    print("-" * 80)

    rows = []
    for i in ch_indices:
        if i >= TOTAL_CHANNELS:
            continue
        name  = CHANNEL_NAMES[i]
        mean  = stats.mean(i)
        std_  = stats.std(i)
        rms_  = stats.rms(i)
        mn    = stats.min_[i]
        mx    = stats.max_[i]
        cnt   = stats.count[i]
        print(f"  {i:>3}  {name:>8}  {mean:>10.2f}  {std_:>10.2f}  "
              f"{rms_:>10.2f}  {mn:>10.2f}  {mx:>10.2f}  {cnt:>8}")
        rows.append({
            'index': i, 'name': name,
            'mean_uv': mean, 'std_uv': std_, 'rms_uv': rms_,
            'min_uv': mn, 'max_uv': mx, 'samples': cnt,
        })

    # --- CSV export ---------------------------------------------------------
    if csv_out:
        import csv
        with open(csv_out, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nCSV written: {csv_out}")

    # --- Plot ---------------------------------------------------------------
    if do_plot:
        _plot(rows, ch_indices, paths)


def _plot(rows, ch_indices, paths):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not installed — skipping plots.")
        return

    # Build arrays for selected channels
    names    = [r['name']    for r in rows]
    std_vals = [r['std_uv']  for r in rows]
    rms_vals = [r['rms_uv']  for r in rows]
    mean_vals= [r['mean_uv'] for r in rows]
    x        = list(range(len(rows)))

    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle(f"MushIO Analysis — {', '.join(os.path.basename(p) for p in paths)}",
                 fontsize=11)

    # Top: RMS noise per channel
    ax = axes[0]
    ax.bar(x, rms_vals, color='steelblue', alpha=0.8, label='RMS µV')
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=90, fontsize=7)
    ax.set_ylabel("RMS (µV)")
    ax.set_title("Per-Channel RMS Signal Level")
    ax.legend()
    ax.grid(axis='y', alpha=0.3)

    # Bottom: Mean ± Std
    ax = axes[1]
    ax.errorbar(x, mean_vals, yerr=std_vals, fmt='o', markersize=3,
                color='darkorange', ecolor='gray', elinewidth=0.8, capsize=2)
    ax.axhline(0, color='black', linewidth=0.5, linestyle='--')
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=90, fontsize=7)
    ax.set_ylabel("Mean ± Std (µV)")
    ax.set_title("Per-Channel Mean and Standard Deviation")
    ax.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="MushIO V1.0 Post-Processing Analysis Tool"
    )
    parser.add_argument(
        'files', nargs='+',
        help='.bin raw frame file(s) from receiver.py --raw'
    )
    parser.add_argument(
        '--channels', '-c', type=str, default=None,
        help='Comma-separated channel indices to analyse (default: all 72)'
    )
    parser.add_argument(
        '--secs', '-s', type=float, default=None,
        help='Analyse only the first N seconds of each file'
    )
    parser.add_argument(
        '--csv', type=str, default=None,
        help='Export per-channel statistics to this CSV file'
    )
    parser.add_argument(
        '--plot', '-p', action='store_true',
        help='Show matplotlib summary plots'
    )
    args = parser.parse_args()

    # Validate files
    for path in args.files:
        if not os.path.isfile(path):
            print(f"Error: file not found: {path}", file=sys.stderr)
            sys.exit(1)

    # Parse channel list
    channels = None
    if args.channels:
        try:
            channels = [int(c.strip()) for c in args.channels.split(',')]
        except ValueError:
            print("Error: --channels must be comma-separated integers", file=sys.stderr)
            sys.exit(1)

    analyse_files(
        args.files,
        channels=channels,
        max_secs=args.secs,
        csv_out=args.csv,
        do_plot=args.plot,
    )


if __name__ == '__main__':
    main()
