"""
MushIO V1.0 Pico Simulator

Simulates the Raspberry Pi Pico 2W firmware over TCP.
Use this to test the host receiver without real hardware.

Usage:
    # Terminal 1 - start receiver:
    python host/receiver.py --port 9000

    # Terminal 2 - run simulator:
    python test/simulator.py [--host 127.0.0.1] [--port 9000] [--frames 500] [--fps 200]

Options:
    --host   Host IP (default: 127.0.0.1)
    --port   TCP port (default: 9000)
    --frames Number of frames to send (default: 500, 0 = infinite)
    --fps    Target frame rate (default: 200)
    --noise  ADC noise amplitude in raw counts (default: 100)
    --bad    Fraction of frames with corrupted CRC (default: 0.0)
    --drop   Fraction of frames to drop (simulates seq gaps) (default: 0.0)
"""

import socket
import struct
import time
import argparse
import math
import random


# =============================================================================
# Protocol constants (must match firmware/config.py and host/receiver.py)
# =============================================================================

SYNC_WORD        = 0xAA55
NUM_ADCS         = 6
CHANNELS_PER_ADC = 12
TOTAL_CHANNELS   = NUM_ADCS * CHANNELS_PER_ADC   # 72

HEADER_SIZE = 10
DATA_SIZE   = TOTAL_CHANNELS * 3   # 216
CRC_SIZE    = 2
FRAME_SIZE  = HEADER_SIZE + DATA_SIZE + CRC_SIZE  # 228


# =============================================================================
# CRC-16 CCITT (identical to firmware and receiver)
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
# Frame builder (mirrors firmware streamer.py build_frame)
# =============================================================================

def build_frame(timestamp_ms: int, seq: int, samples: list) -> bytes:
    """
    Build a 228-byte frame.
    samples: list of 72 signed 24-bit integers.
    """
    frame = bytearray(FRAME_SIZE)

    struct.pack_into('<H', frame, 0, SYNC_WORD)
    struct.pack_into('<I', frame, 2, timestamp_ms & 0xFFFFFFFF)
    struct.pack_into('<H', frame, 6, seq & 0xFFFF)
    frame[8] = NUM_ADCS
    frame[9] = CHANNELS_PER_ADC

    for i, s in enumerate(samples):
        raw = s & 0xFFFFFF
        offset = HEADER_SIZE + i * 3
        frame[offset]     = (raw >> 16) & 0xFF
        frame[offset + 1] = (raw >> 8)  & 0xFF
        frame[offset + 2] = raw & 0xFF

    crc = crc16_ccitt(bytes(frame[:HEADER_SIZE + DATA_SIZE]))
    struct.pack_into('<H', frame, HEADER_SIZE + DATA_SIZE, crc)
    return bytes(frame)


# =============================================================================
# Signal generator (realistic electrophysiology noise)
# =============================================================================

def generate_samples(t: float, noise_amp: int) -> list:
    """
    Generate 72 realistic-ish ADC samples.

    Models:
    - A slow drift sinusoid (1/30 Hz, ~50 uV amplitude, mimics spontaneous activity)
    - Per-channel white noise
    - Occasional spike-like transient on a few channels

    At G=11, VREF=5.08V:
      1 uV input  -> raw ≈ 1.82 counts
      10 uV input -> raw ≈ 18.2 counts
      1 mV input  -> raw ≈ 1820 counts
    """
    samples = []
    for ch in range(TOTAL_CHANNELS):
        # Slow drift: different phase per channel
        phase = (ch / TOTAL_CHANNELS) * 2 * math.pi
        drift = int(noise_amp * 0.5 * math.sin(2 * math.pi * t / 30.0 + phase))

        # White noise
        noise = random.randint(-noise_amp, noise_amp)

        # Occasional spike on channel 0 (simulates a real signal)
        spike = 0
        if ch == 0 and random.random() < 0.005:  # ~1 spike/sec at 200fps
            spike = random.choice([-1, 1]) * noise_amp * 5

        val = drift + noise + spike
        # Clamp to 24-bit signed range
        val = max(-8388607, min(8388607, val))
        samples.append(val)

    return samples


# =============================================================================
# Simulator
# =============================================================================

def run(host: str, port: int, n_frames: int, fps: float,
        noise_amp: int, bad_frac: float, drop_frac: float):

    frame_period = 1.0 / fps
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    print(f"[SIM] Connecting to {host}:{port}...")
    try:
        sock.connect((host, port))
    except ConnectionRefusedError:
        print("[SIM] ERROR: Connection refused. Is receiver.py running?")
        return
    print(f"[SIM] Connected. Sending {n_frames if n_frames else '∞'} frames "
          f"@ {fps} fps, noise_amp={noise_amp} counts")
    if bad_frac > 0:
        print(f"[SIM] {bad_frac*100:.1f}% of frames will have bad CRC")
    if drop_frac > 0:
        print(f"[SIM] {drop_frac*100:.1f}% of frames will be dropped (seq gaps)")

    seq         = 0
    sent        = 0
    bad_sent    = 0
    dropped_seq = 0
    start_real  = time.time()
    last_print  = start_real

    frame_idx = 0
    while n_frames == 0 or frame_idx < n_frames:
        # Real-time pacing
        target_time = start_real + frame_idx * frame_period
        now = time.time()
        if target_time > now:
            time.sleep(target_time - now)

        timestamp_ms = int((time.time() - start_real) * 1000)

        # Simulate dropped frame (seq gap)
        if drop_frac > 0 and random.random() < drop_frac:
            seq = (seq + 1) & 0xFFFF  # advance seq without sending
            dropped_seq += 1
            frame_idx += 1
            continue

        samples = generate_samples(timestamp_ms / 1000.0, noise_amp)
        frame = build_frame(timestamp_ms, seq, samples)

        # Optionally corrupt the CRC
        if bad_frac > 0 and random.random() < bad_frac:
            frame = bytearray(frame)
            frame[-1] ^= 0xFF  # flip all bits in last CRC byte
            frame = bytes(frame)
            bad_sent += 1

        try:
            sock.sendall(frame)
        except BrokenPipeError:
            print("\n[SIM] Receiver disconnected.")
            break

        seq = (seq + 1) & 0xFFFF
        sent += 1
        frame_idx += 1

        # Print stats every 5 seconds
        now = time.time()
        if now - last_print >= 5.0:
            elapsed = now - start_real
            actual_fps = sent / elapsed if elapsed > 0 else 0
            print(f"[SIM] Frames sent: {sent}  "
                  f"Actual FPS: {actual_fps:.1f}  "
                  f"Bad CRC: {bad_sent}  "
                  f"Seq gaps: {dropped_seq}  "
                  f"Elapsed: {elapsed:.0f}s")
            last_print = now

    sock.close()
    elapsed = time.time() - start_real
    actual_fps = sent / elapsed if elapsed > 0 else 0
    print(f"\n[SIM] Done. Sent {sent} frames in {elapsed:.1f}s "
          f"({actual_fps:.1f} fps)")
    if bad_sent:
        print(f"[SIM] Bad CRC frames: {bad_sent}")
    if dropped_seq:
        print(f"[SIM] Seq gaps introduced: {dropped_seq}")


# =============================================================================
# Entry point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="MushIO V1.0 Pico Simulator - sends frames to host/receiver.py"
    )
    parser.add_argument('--host',   default='127.0.0.1', help='Receiver host IP')
    parser.add_argument('--port',   type=int, default=9000, help='Receiver TCP port')
    parser.add_argument('--frames', type=int, default=500,
                        help='Number of frames to send (0=infinite)')
    parser.add_argument('--fps',    type=float, default=200.0,
                        help='Target frame rate (Hz)')
    parser.add_argument('--noise',  type=int, default=100,
                        help='ADC noise amplitude (raw counts)')
    parser.add_argument('--bad',    type=float, default=0.0,
                        help='Fraction of frames with bad CRC (0.0-1.0)')
    parser.add_argument('--drop',   type=float, default=0.0,
                        help='Fraction of frames to drop as seq gaps (0.0-1.0)')
    args = parser.parse_args()

    run(
        host=args.host,
        port=args.port,
        n_frames=args.frames,
        fps=args.fps,
        noise_amp=args.noise,
        bad_frac=args.bad,
        drop_frac=args.drop,
    )


if __name__ == '__main__':
    main()
