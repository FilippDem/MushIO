"""
MushIO V1.0 Protocol Unit Tests

Tests that can run on any Python 3.8+ machine (no hardware needed):
  - CRC-16 CCITT correctness (firmware vs receiver implementations)
  - Frame build / parse round-trip
  - Sync word byte order
  - 24-bit sign extension
  - Missed-frame detection
  - Bad-CRC rejection
"""

import struct
import sys
import os

# ---------------------------------------------------------------------------
# Pull in receiver code directly (it's pure Python)
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'host'))
import receiver as rx

# ---------------------------------------------------------------------------
# Re-implement firmware CRC exactly as streamer.py does (pure Python)
# ---------------------------------------------------------------------------
def fw_crc16_ccitt(data: bytes) -> int:
    """Identical to streamer.py crc16_ccitt()."""
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


# ---------------------------------------------------------------------------
# Re-implement firmware frame builder (mirrors streamer.py build_frame)
# ---------------------------------------------------------------------------
SYNC_WORD        = 0xAA55
NUM_ADCS         = 6
CHANNELS_PER_ADC = 12
TOTAL_CHANNELS   = NUM_ADCS * CHANNELS_PER_ADC   # 72
HEADER_SIZE      = 10
DATA_SIZE        = TOTAL_CHANNELS * 3             # 216
CRC_SIZE         = 2
FRAME_SIZE       = HEADER_SIZE + DATA_SIZE + CRC_SIZE  # 228

def build_frame(timestamp_ms: int, seq: int, samples: list) -> bytes:
    """
    Build a 228-byte frame the same way the firmware does.
    samples: list of 72 signed 24-bit integers.
    """
    frame = bytearray(FRAME_SIZE)

    # Header (little-endian)
    struct.pack_into('<H', frame, 0, SYNC_WORD)
    struct.pack_into('<I', frame, 2, timestamp_ms & 0xFFFFFFFF)
    struct.pack_into('<H', frame, 6, seq & 0xFFFF)
    frame[8] = NUM_ADCS
    frame[9] = CHANNELS_PER_ADC

    # Pack 24-bit samples big-endian (matches firmware's raw ADC byte order)
    for i, s in enumerate(samples):
        raw = s & 0xFFFFFF  # two's complement truncation to 24 bits
        offset = HEADER_SIZE + i * 3
        frame[offset]     = (raw >> 16) & 0xFF
        frame[offset + 1] = (raw >> 8)  & 0xFF
        frame[offset + 2] = raw & 0xFF

    # CRC over header + data
    crc = fw_crc16_ccitt(bytes(frame[:HEADER_SIZE + DATA_SIZE]))
    struct.pack_into('<H', frame, HEADER_SIZE + DATA_SIZE, crc)

    return bytes(frame)


# ===========================================================================
# Tests
# ===========================================================================

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
_failures = []

def check(name: str, condition: bool, detail: str = ""):
    if condition:
        print(f"  [{PASS}] {name}")
    else:
        print(f"  [{FAIL}] {name}" + (f": {detail}" if detail else ""))
        _failures.append(name)


# ---------------------------------------------------------------------------
# 1. CRC-16 agreement
# ---------------------------------------------------------------------------
def test_crc_known_vectors():
    """CRC-16 CCITT-FALSE: well-known test vectors."""
    print("\n[1] CRC-16 known vectors")

    # "123456789" -> 0x29B1
    data = b"123456789"
    got = fw_crc16_ccitt(data)
    check("fw CRC('123456789') == 0x29B1", got == 0x29B1, f"got 0x{got:04X}")

    got_rx = rx.crc16_ccitt(data)
    check("rx CRC('123456789') == 0x29B1", got_rx == 0x29B1, f"got 0x{got_rx:04X}")

    # Empty string -> 0xFFFF (init value, nothing XORed)
    got_empty = fw_crc16_ccitt(b"")
    check("fw CRC(empty) == 0xFFFF", got_empty == 0xFFFF, f"got 0x{got_empty:04X}")

    # All zeros
    got_zeros = fw_crc16_ccitt(bytes(10))
    got_zeros_rx = rx.crc16_ccitt(bytes(10))
    check("fw and rx CRC(zeros) agree", got_zeros == got_zeros_rx,
          f"fw=0x{got_zeros:04X} rx=0x{got_zeros_rx:04X}")


def test_crc_firmware_receiver_agree():
    """CRC produced by firmware frame builder == CRC validated by receiver."""
    print("\n[2] Firmware / receiver CRC agreement")

    for seq in [0, 1, 255, 256, 65535]:
        samples = [seq * (i + 1) % 8388607 for i in range(72)]
        frame = build_frame(123456, seq, samples)

        fw_crc = fw_crc16_ccitt(frame[:HEADER_SIZE + DATA_SIZE])
        rx_crc = rx.crc16_ccitt(frame[:HEADER_SIZE + DATA_SIZE])
        in_frame = struct.unpack_from('<H', frame, HEADER_SIZE + DATA_SIZE)[0]

        check(f"seq={seq}: fw CRC == rx CRC == embedded CRC",
              fw_crc == rx_crc == in_frame,
              f"fw=0x{fw_crc:04X} rx=0x{rx_crc:04X} embed=0x{in_frame:04X}")


# ---------------------------------------------------------------------------
# 2. Frame size
# ---------------------------------------------------------------------------
def test_frame_size():
    print("\n[3] Frame size")
    frame = build_frame(0, 0, [0] * 72)
    check(f"frame is {FRAME_SIZE} bytes", len(frame) == FRAME_SIZE,
          f"got {len(frame)}")
    check("receiver FRAME_SIZE constant matches", rx.FRAME_SIZE == FRAME_SIZE,
          f"rx.FRAME_SIZE={rx.FRAME_SIZE}")


# ---------------------------------------------------------------------------
# 3. Sync word byte order
# ---------------------------------------------------------------------------
def test_sync_word():
    print("\n[4] Sync word byte order")
    frame = build_frame(0, 0, [0] * 72)
    # 0xAA55 little-endian -> bytes [0x55, 0xAA]
    check("byte[0] == 0x55", frame[0] == 0x55, f"got 0x{frame[0]:02X}")
    check("byte[1] == 0xAA", frame[1] == 0xAA, f"got 0x{frame[1]:02X}")

    # Receiver _find_sync scans for [0x55, 0xAA]
    pos = rx.MushIOReceiver._find_sync(bytearray(frame))
    check("_find_sync finds sync at position 0", pos == 0, f"got {pos}")

    # Sync in the middle of garbage
    garbage = bytes(20) + frame + bytes(10)
    pos2 = rx.MushIOReceiver._find_sync(bytearray(garbage))
    check("_find_sync finds sync after 20 garbage bytes", pos2 == 20, f"got {pos2}")

    # No sync
    pos3 = rx.MushIOReceiver._find_sync(bytearray(b'\x00' * 50))
    check("_find_sync returns -1 when no sync", pos3 == -1, f"got {pos3}")


# ---------------------------------------------------------------------------
# 4. Round-trip: build frame -> parse_frame -> compare
# ---------------------------------------------------------------------------
def test_round_trip():
    print("\n[5] Build / parse round-trip")

    # Positive values
    samples_pos = list(range(72))
    frame = build_frame(9999, 42, samples_pos)
    parsed = rx.parse_frame(frame)

    check("parse_frame returns non-None", parsed is not None)
    check("timestamp_ms", parsed['timestamp_ms'] == 9999)
    check("seq", parsed['seq'] == 42)
    check("adc_count", parsed['adc_count'] == 6)
    check("ch_per_adc", parsed['ch_per_adc'] == 12)
    check("crc_valid", parsed['crc_valid'])
    check("samples length", len(parsed['samples']) == 72)
    check("samples match (positive)", parsed['samples'] == samples_pos,
          f"first mismatch: {parsed['samples'][:5]} vs {samples_pos[:5]}")

    # Negative values (24-bit two's complement)
    samples_neg = [-1, -8388608, -1000, 0, 8388607, -500] * 12
    frame2 = build_frame(0, 0, samples_neg)
    parsed2 = rx.parse_frame(frame2)
    check("crc_valid (negative samples)", parsed2['crc_valid'])
    check("samples match (negative)", parsed2['samples'] == samples_neg,
          f"first mismatch: {parsed2['samples'][:4]} vs {samples_neg[:4]}")

    # Mixed realistic values (±100 uV at G=11, VREF=5.08 -> raw ≈ ±2396)
    import random
    random.seed(0)
    samples_real = [random.randint(-2500, 2500) for _ in range(72)]
    frame3 = build_frame(1_234_567, 1000, samples_real)
    parsed3 = rx.parse_frame(frame3)
    check("samples match (realistic)", parsed3['samples'] == samples_real)


# ---------------------------------------------------------------------------
# 5. 24-bit sign extension
# ---------------------------------------------------------------------------
def test_sign_extension():
    print("\n[6] 24-bit sign extension")

    cases = [
        (0,        0),
        (1,        1),
        (8388607,  8388607),   # +FS = 0x7FFFFF
        (8388608,  -8388608),  # -FS = 0x800000
        (16777215, -1),        # 0xFFFFFF
        (8388609,  -8388607),  # 0x800001
    ]

    for raw_unsigned, expected_signed in cases:
        # Build frame with raw bytes
        frame = bytearray(FRAME_SIZE)
        struct.pack_into('<H', frame, 0, SYNC_WORD)
        frame[8] = NUM_ADCS
        frame[9] = CHANNELS_PER_ADC
        frame[HEADER_SIZE]     = (raw_unsigned >> 16) & 0xFF
        frame[HEADER_SIZE + 1] = (raw_unsigned >> 8)  & 0xFF
        frame[HEADER_SIZE + 2] = raw_unsigned & 0xFF
        crc = rx.crc16_ccitt(bytes(frame[:HEADER_SIZE + DATA_SIZE]))
        struct.pack_into('<H', frame, HEADER_SIZE + DATA_SIZE, crc)

        parsed = rx.parse_frame(bytes(frame))
        got = parsed['samples'][0]
        check(f"raw 0x{raw_unsigned:06X} -> {expected_signed}",
              got == expected_signed, f"got {got}")


# ---------------------------------------------------------------------------
# 6. Bad CRC detection
# ---------------------------------------------------------------------------
def test_bad_crc():
    print("\n[7] Bad CRC detection")

    frame = bytearray(build_frame(0, 0, [42] * 72))

    # Flip one bit in the CRC
    frame[-1] ^= 0x01
    parsed = rx.parse_frame(bytes(frame))
    check("flipped CRC byte -> crc_valid=False", not parsed['crc_valid'])

    # Flip one bit in the data
    frame = bytearray(build_frame(0, 0, [42] * 72))
    frame[HEADER_SIZE] ^= 0x01
    parsed2 = rx.parse_frame(bytes(frame))
    check("flipped data byte -> crc_valid=False", not parsed2['crc_valid'])


# ---------------------------------------------------------------------------
# 7. Wrong-size frame rejected
# ---------------------------------------------------------------------------
def test_size_rejection():
    print("\n[8] Wrong-size frame rejection")

    frame = build_frame(0, 0, [0] * 72)
    check("correct size accepted", rx.parse_frame(frame) is not None)
    check("1 byte short rejected",  rx.parse_frame(frame[:-1]) is None)
    check("1 byte extra rejected",  rx.parse_frame(frame + b'\x00') is None)
    check("empty rejected",         rx.parse_frame(b'') is None)


# ---------------------------------------------------------------------------
# 8. adc_to_voltage sanity
# ---------------------------------------------------------------------------
def test_voltage_conversion():
    print("\n[9] adc_to_voltage sanity")

    VREF    = 5.08
    GAIN    = 11.0
    FS      = 2**23

    # Zero -> 0V
    v = rx.adc_to_voltage(0)
    check("zero count -> 0.0 V", v == 0.0, f"got {v}")

    # Positive full scale
    v_pos = rx.adc_to_voltage(FS - 1)
    expected = ((FS - 1) / FS) * VREF / GAIN
    check(f"+FS -> ~{expected*1000:.3f} mV",
          abs(v_pos - expected) < 1e-9, f"got {v_pos*1000:.6f} mV")

    # Negative full scale (error marker)
    v_err = rx.adc_to_voltage(-FS)
    import math
    check("error marker (-8388608) -> NaN", math.isnan(v_err))

    # Sign: negative count -> negative voltage
    v_neg = rx.adc_to_voltage(-1000)
    check("negative count -> negative voltage", v_neg < 0, f"got {v_neg}")


# ---------------------------------------------------------------------------
# 9. Receiver constant consistency
# ---------------------------------------------------------------------------
def test_constants():
    print("\n[10] Constant consistency (receiver vs firmware)")
    check("SYNC_WORD",        rx.SYNC_WORD        == 0xAA55)
    check("NUM_ADCS",         rx.NUM_ADCS         == 6)
    check("CHANNELS_PER_ADC", rx.CHANNELS_PER_ADC == 12)
    check("TOTAL_CHANNELS",   rx.TOTAL_CHANNELS   == 72)
    check("HEADER_SIZE",      rx.HEADER_SIZE      == 10)
    check("DATA_SIZE",        rx.DATA_SIZE        == 216)
    check("CRC_SIZE",         rx.CRC_SIZE         == 2)
    check("FRAME_SIZE",       rx.FRAME_SIZE       == 228)
    check("VREF",             abs(rx.VREF - 5.08) < 1e-9)
    check("AFE_GAIN",         rx.AFE_GAIN         == 11.0)

    # CHANNEL_NAMES must have exactly 72 entries
    check("CHANNEL_NAMES length == 72", len(rx.CHANNEL_NAMES) == 72,
          f"got {len(rx.CHANNEL_NAMES)}")

    # All channel names must be unique
    unique = len(set(rx.CHANNEL_NAMES)) == 72
    if not unique:
        dupes = [n for n in rx.CHANNEL_NAMES if rx.CHANNEL_NAMES.count(n) > 1]
        check("all channel names unique", False, f"duplicates: {set(dupes)}")
    else:
        check("all channel names unique", True)


# ===========================================================================
# Runner
# ===========================================================================

if __name__ == "__main__":
    print("=" * 55)
    print("  MushIO V1.0 Protocol Unit Tests")
    print("=" * 55)

    test_crc_known_vectors()
    test_crc_firmware_receiver_agree()
    test_frame_size()
    test_sync_word()
    test_round_trip()
    test_sign_extension()
    test_bad_crc()
    test_size_rejection()
    test_voltage_conversion()
    test_constants()

    print()
    print("=" * 55)
    if _failures:
        print(f"  RESULT: {len(_failures)} FAILED")
        for f in _failures:
            print(f"    - {f}")
        sys.exit(1)
    else:
        print(f"  RESULT: ALL PASSED")
    print("=" * 55)
