"""
MushIO V1.0 Host Receiver

TCP server that receives binary ADC frames from the Pico 2W,
validates CRC-16, and logs to timestamped CSV files.

Usage:
    python receiver.py [--port 9000] [--raw]

Requirements:
    Python 3.8+ (no external dependencies)
"""

import socket
import struct
import time
import csv
import argparse
import os
from datetime import datetime


# =============================================================================
# Constants (must match firmware/config.py)
# =============================================================================

SYNC_WORD = 0xAA55
NUM_ADCS = 6
CHANNELS_PER_ADC = 12
TOTAL_CHANNELS = NUM_ADCS * CHANNELS_PER_ADC  # 72

HEADER_SIZE = 10   # sync(2) + timestamp(4) + seq(2) + adc_count(1) + ch_per_adc(1)
DATA_SIZE = TOTAL_CHANNELS * 3  # 216 bytes
CRC_SIZE = 2
FRAME_SIZE = HEADER_SIZE + DATA_SIZE + CRC_SIZE  # 228 bytes

# Reference voltage for ADC count -> voltage conversion
# ADS124S08 internal reference = 2.5 V.  Full-scale input = ±VREF/PGA.
# AFE_GAIN is the analog front-end gain before the ADC.  The GUI divides
# by total_gain (AFE_GAIN × PGA) to display input-referred voltage.
VREF = 2.5
AFE_GAIN = 11.0
ADC_FULL_SCALE = 2**23  # 24-bit signed, positive full scale

# ADS124S08 operating configuration (must match firmware ADC register setup)
ADC_DATA_RATE_SPS = 4000      # DATARATE register: DR_4000 → 4000 output samples/sec
SPI_CLOCK_HZ      = 1_000_000 # Conservative 1 MHz SPI; datasheet max is 16 MHz

# Electrode channel map — empirically verified via per-electrode stimulation.
# Each sub-list is one ADC (0–5), 12 entries for AIN0–AIN11.
# Flat index = adc_index * 12 + ain_channel.
#
# Channels requiring user verification (assumptions made from test data):
#   - ELEC70: assumed grid pos "50" (digit transposition from "05" in table)
#   - ELEC27: assumed ADC1 AIN7 → grid "14" (was listed as "25", conflicted)
#   - ELEC67: assumed ADC5 AIN8 → grid "67" (was listed as "76", conflicted)
#   - ADC2 AIN8-11: ELEC42/32/33/43 order is pattern-based (no test data)
#   - ADC3 AIN8-11: ELEC37/36/35/34 order is pattern-based (no test data)
CHANNEL_MAP = [
    ["ELEC02", "ELEC23", "ELEC12", "ELEC22",   # ADC0: AIN0-3
     "ELEC21", "ELEC11", "ELEC20", "ELEC01",   #       AIN4-7
     "ELEC00", "ELEC10", "ELEC03", "ELEC13"],  #       AIN8-11
    ["ELEC16", "ELEC24", "ELEC17", "ELEC04",   # ADC1: AIN0-3
     "ELEC05", "ELEC06", "ELEC07", "ELEC27",   #       AIN4-7
     "ELEC15", "ELEC26", "ELEC14", "ELEC25"],  #       AIN8-11
    ["ELEC40", "ELEC30", "ELEC31", "ELEC41",   # ADC2: AIN0-3
     "STIM_1", "STIM_0", "STIM_7", "STIM_6",  #       AIN4-7 (STIM)
     "ELEC42", "ELEC32", "ELEC33", "ELEC43"],  #       AIN8-11 (assumed)
    ["ELEC47", "ELEC46", "ELEC45", "ELEC44",   # ADC3: AIN0-3
     "STIM_5", "STIM_4", "STIM_3", "STIM_2",  #       AIN4-7 (STIM)
     "ELEC37", "ELEC36", "ELEC35", "ELEC34"],  #       AIN8-11 (assumed)
    ["ELEC71", "ELEC70", "ELEC61", "ELEC62",   # ADC4: AIN0-3
     "ELEC72", "ELEC53", "ELEC63", "ELEC73",   #       AIN4-7
     "ELEC50", "ELEC60", "ELEC52", "ELEC51"],  #       AIN8-11
    ["ELEC75", "ELEC54", "ELEC74", "ELEC64",   # ADC5: AIN0-3
     "ELEC66", "ELEC55", "ELEC56", "ELEC65",   #       AIN4-7
     "ELEC67", "ELEC77", "ELEC76", "ELEC57"],  #       AIN8-11
]

# Flat list of all 72 channel names in frame order
CHANNEL_NAMES = []
for adc_map in CHANNEL_MAP:
    CHANNEL_NAMES.extend(adc_map)

# =============================================================================
# Channel Classification: 64 recording electrodes + 8 stimulation channels
# =============================================================================
# ADC2 and ADC3 each have AIN8-11 wired to stimulation drive circuitry.
# Flat index = adc_index * CHANNELS_PER_ADC + ain_channel

STIM_CHANNEL_INDICES = [
    2 * CHANNELS_PER_ADC + 4,   # 28: STIM_1
    2 * CHANNELS_PER_ADC + 5,   # 29: STIM_0
    2 * CHANNELS_PER_ADC + 6,   # 30: STIM_7
    2 * CHANNELS_PER_ADC + 7,   # 31: STIM_6
    3 * CHANNELS_PER_ADC + 4,   # 40: STIM_5
    3 * CHANNELS_PER_ADC + 5,   # 41: STIM_4
    3 * CHANNELS_PER_ADC + 6,   # 42: STIM_3
    3 * CHANNELS_PER_ADC + 7,   # 43: STIM_2
]  # 8 STIM channels

RECORDING_CHANNEL_INDICES = [
    i for i in range(TOTAL_CHANNELS) if i not in STIM_CHANNEL_INDICES
]  # 64 recording channels

NUM_RECORDING_CHANNELS = 64
NUM_STIM_CHANNELS      = 8

# Ordered name lists for each CSV
ELEC_NAMES = [CHANNEL_NAMES[i] for i in RECORDING_CHANNEL_INDICES]
STIM_NAMES = [CHANNEL_NAMES[i] for i in STIM_CHANNEL_INDICES]


# =============================================================================
# CRC-16 CCITT (must match firmware implementation)
# =============================================================================

def crc16_ccitt(data: bytes) -> int:
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


# =============================================================================
# Frame Parsing
# =============================================================================

def parse_frame(frame_bytes: bytes):
    """
    Parse a binary frame into its components.

    Returns:
        dict with keys: sync, timestamp_ms, seq, adc_count, ch_per_adc,
                        samples (list of 72 signed ints), crc, crc_valid
        or None if sync word doesn't match
    """
    if len(frame_bytes) != FRAME_SIZE:
        return None

    sync = struct.unpack_from('<H', frame_bytes, 0)[0]
    if sync != SYNC_WORD:
        return None

    timestamp_us = struct.unpack_from('<I', frame_bytes, 2)[0]
    seq = struct.unpack_from('<H', frame_bytes, 6)[0]
    adc_count = frame_bytes[8]
    ch_per_adc = frame_bytes[9]

    # Parse 24-bit samples (big-endian, signed)
    samples = []
    for i in range(TOTAL_CHANNELS):
        offset = HEADER_SIZE + i * 3
        b0 = frame_bytes[offset]
        b1 = frame_bytes[offset + 1]
        b2 = frame_bytes[offset + 2]
        value = (b0 << 16) | (b1 << 8) | b2
        if value & 0x800000:
            value -= 0x1000000
        samples.append(value)

    # Validate CRC
    crc_received = struct.unpack_from('<H', frame_bytes, HEADER_SIZE + DATA_SIZE)[0]
    crc_computed = crc16_ccitt(frame_bytes[:HEADER_SIZE + DATA_SIZE])
    crc_valid = (crc_received == crc_computed)

    return {
        'sync': sync,
        'timestamp_us': timestamp_us,
        'seq': seq,
        'adc_count': adc_count,
        'ch_per_adc': ch_per_adc,
        'samples': samples,
        'crc': crc_received,
        'crc_valid': crc_valid,
    }


def adc_to_voltage(raw_count: int) -> float:
    """Convert raw 24-bit ADC count to input voltage (before AFE gain)."""
    if raw_count == -8388608:  # 0x800000 = error marker
        return float('nan')
    # ADC voltage = (count / 2^23) * VREF (bipolar)
    adc_voltage = (raw_count / ADC_FULL_SCALE) * VREF
    # Input voltage = ADC voltage / AFE gain
    return adc_voltage / AFE_GAIN


# =============================================================================
# TCP Receiver
# =============================================================================

class MushIOReceiver:
    """TCP server that receives and logs MushIO data frames."""

    def __init__(self, port=9000, output_dir="data", log_raw=False):
        self.port = port
        self.output_dir = output_dir
        self.log_raw = log_raw

        self.total_frames = 0
        self.bad_crc_frames = 0
        self.last_seq = None
        self.missed_frames = 0

        os.makedirs(output_dir, exist_ok=True)

    def run(self):
        """Start the TCP server and process connections."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.port))
        server.listen(1)

        print(f"[SERVER] Listening on port {self.port}...")
        print(f"[SERVER] Waiting for Pico connection...")
        print(f"[SERVER] Press Ctrl+C to stop.")
        print()

        while True:
            try:
                conn, addr = server.accept()
                print(f"[SERVER] Connection from {addr[0]}:{addr[1]}")
                self._handle_connection(conn, addr)
            except KeyboardInterrupt:
                print("\n[SERVER] Shutting down.")
                break
            except Exception as e:
                print(f"[SERVER] Error: {e}")
                time.sleep(1)

        server.close()

    def _handle_connection(self, conn, addr):
        """Process a single Pico connection until it disconnects."""
        # Create session files
        session_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        elec_path = os.path.join(self.output_dir, f"mushio_elec_{session_time}.csv")
        stim_path = os.path.join(self.output_dir, f"mushio_stim_{session_time}.csv")
        raw_path  = os.path.join(self.output_dir, f"mushio_{session_time}.bin")

        print(f"[SESSION] Electrode CSV  ({NUM_RECORDING_CHANNELS} ch): {elec_path}")
        print(f"[SESSION] Stimulation CSV ({NUM_STIM_CHANNELS} ch):       {stim_path}")

        # Electrode CSV: timestamp_ms, seq, ELEC* x64
        elec_file   = open(elec_path, 'w', newline='')
        elec_writer = csv.writer(elec_file)
        elec_writer.writerow(['timestamp_ms', 'seq'] + ELEC_NAMES)

        # Stimulation CSV: timestamp_ms, seq, STIM_* x8
        stim_file   = open(stim_path, 'w', newline='')
        stim_writer = csv.writer(stim_file)
        stim_writer.writerow(['timestamp_ms', 'seq'] + STIM_NAMES)

        raw_file = None
        if self.log_raw:
            raw_file = open(raw_path, 'wb')
            print(f"[SESSION] Raw binary log: {raw_path}")

        # Reset session stats
        session_frames = 0
        session_bad_crc = 0
        session_start = time.time()
        last_print = time.time()
        fps_count = 0

        # Receive buffer - accumulate bytes until we have complete frames
        recv_buf = bytearray()

        try:
            conn.settimeout(30)  # 30s timeout for keepalive

            while True:
                try:
                    chunk = conn.recv(4096)
                    if not chunk:
                        print("[SESSION] Connection closed by Pico.")
                        break

                    recv_buf.extend(chunk)

                    # Process all complete frames in buffer
                    while len(recv_buf) >= FRAME_SIZE:
                        # Search for sync word
                        sync_pos = self._find_sync(recv_buf)

                        if sync_pos < 0:
                            # No sync found - discard all but last byte
                            recv_buf = recv_buf[-1:]
                            break

                        if sync_pos > 0:
                            # Discard bytes before sync
                            print(f"[WARN] Discarded {sync_pos} bytes before sync")
                            recv_buf = recv_buf[sync_pos:]

                        if len(recv_buf) < FRAME_SIZE:
                            break

                        # Extract one frame
                        frame_bytes = bytes(recv_buf[:FRAME_SIZE])
                        recv_buf = recv_buf[FRAME_SIZE:]

                        # Log raw
                        if raw_file:
                            raw_file.write(frame_bytes)

                        # Parse
                        parsed = parse_frame(frame_bytes)
                        if parsed is None:
                            continue

                        session_frames += 1
                        self.total_frames += 1
                        fps_count += 1

                        if not parsed['crc_valid']:
                            session_bad_crc += 1
                            self.bad_crc_frames += 1
                            continue

                        # Check for missed frames
                        if self.last_seq is not None:
                            expected = (self.last_seq + 1) & 0xFFFF
                            if parsed['seq'] != expected:
                                gap = (parsed['seq'] - expected) & 0xFFFF
                                self.missed_frames += gap
                        self.last_seq = parsed['seq']

                        samples = parsed['samples']
                        ts      = parsed['timestamp_ms']
                        seq     = parsed['seq']

                        # Write electrode CSV (64 recording channels)
                        elec_row = [ts, seq] + [samples[i] for i in RECORDING_CHANNEL_INDICES]
                        elec_writer.writerow(elec_row)

                        # Write stimulation CSV (8 STIM channels)
                        stim_row = [ts, seq] + [samples[i] for i in STIM_CHANNEL_INDICES]
                        stim_writer.writerow(stim_row)

                        # Periodic flush and status print
                        now = time.time()
                        if now - last_print >= 5.0:
                            elapsed = now - last_print
                            fps = fps_count / elapsed if elapsed > 0 else 0
                            elapsed_total = now - session_start

                            print(
                                f"[DATA] Frames: {session_frames}  "
                                f"FPS: {fps:.1f}  "
                                f"CRC errors: {session_bad_crc}  "
                                f"Missed: {self.missed_frames}  "
                                f"Time: {elapsed_total:.0f}s"
                            )

                            # Show one electrode and one STIM value for sanity
                            elec0_uv = adc_to_voltage(samples[RECORDING_CHANNEL_INDICES[0]]) * 1e6
                            stim0_mv = adc_to_voltage(samples[STIM_CHANNEL_INDICES[0]]) * 1e3
                            print(
                                f"       {ELEC_NAMES[0]}: {elec0_uv:.1f} uV  "
                                f"{STIM_NAMES[0]}: {stim0_mv:.2f} mV"
                            )

                            elec_file.flush()
                            stim_file.flush()
                            if raw_file:
                                raw_file.flush()

                            last_print = now
                            fps_count = 0

                except socket.timeout:
                    print("[SESSION] Socket timeout - Pico may have disconnected.")
                    break

        except KeyboardInterrupt:
            print("\n[SESSION] Interrupted by user.")

        except Exception as e:
            print(f"[SESSION] Error: {e}")

        finally:
            conn.close()
            elec_file.close()
            stim_file.close()
            if raw_file:
                raw_file.close()

            elapsed_total = time.time() - session_start
            print(f"\n[SESSION] Session ended.")
            print(f"  Duration: {elapsed_total:.1f}s")
            print(f"  Total frames: {session_frames}")
            print(f"  CRC errors: {session_bad_crc}")
            print(f"  Missed frames: {self.missed_frames}")
            print(f"  Electrode CSV: {elec_path}")
            print(f"  Stim CSV:      {stim_path}")
            print()

    @staticmethod
    def _find_sync(buf):
        """Find the position of the sync word (0xAA55 little-endian) in buffer."""
        for i in range(len(buf) - 1):
            if buf[i] == 0x55 and buf[i + 1] == 0xAA:
                return i
        return -1


# =============================================================================
# Entry Point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="MushIO V1.0 Host Receiver - TCP server for ADC data logging"
    )
    parser.add_argument(
        '--port', type=int, default=9000,
        help='TCP port to listen on (default: 9000)'
    )
    parser.add_argument(
        '--output', type=str, default='data',
        help='Output directory for CSV/binary files (default: data/)'
    )
    parser.add_argument(
        '--raw', action='store_true',
        help='Also log raw binary frames to .bin file'
    )

    args = parser.parse_args()

    receiver = MushIOReceiver(
        port=args.port,
        output_dir=args.output,
        log_raw=args.raw,
    )
    receiver.run()


if __name__ == '__main__':
    main()
