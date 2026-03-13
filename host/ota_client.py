"""
MushIO V1.0 -- Reliable OTA Firmware Client (V2 protocol)

Connects to the Pico's OTA server on port 9002 and streams the UF2 firmware
image over WiFi.  No USB cable or BOOTSEL mode needed after the first flash.

Reliability features (V2 protocol):
  - CRC32 verification: client and Pico both compute CRC32 of the staged
    image and compare.  If mismatch, the update is aborted.
  - Dual-bank backup: Pico backs up current firmware before applying.
  - Automatic rollback: if new firmware fails to boot 3 times, the Pico
    automatically reverts to the previous working firmware.

Protocol (V2)
-------------
    Client -> "OTAV2 {uf2_size}\\r\\n"
    Server -> "READY\\r\\n"
    Client -> {uf2_size bytes of raw UF2 data}
    Server -> "PROG {pct}%\\r\\n"    (progress, every ~16 KB)
    Server -> "CRC32 {hex}\\r\\n"    (staged image CRC for verification)
    Server -> "DONE\\r\\n"           (metadata written, rebooting)
    Server -> "ERR {reason}\\r\\n"   (on failure)

Usage
-----
    python ota_client.py firmware.uf2
    python ota_client.py firmware.uf2 --host 192.168.68.115
"""

import argparse
import os
import socket
import struct
import sys
import time
import zlib


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

DEFAULT_HOST     = "192.168.68.115"
DEFAULT_OTA_PORT = 9002
DEFAULT_CMD_PORT = 9001

SEND_CHUNK    = 4096
PROGRESS_COLS = 40

REBOOT_WAIT_S   = 20      # allow extra time for backup + apply + reboot
PROBE_TIMEOUT_S = 90      # longer timeout for dual-bank OTA
PROBE_INTERVAL  = 2.0


# ---------------------------------------------------------------------------
# UF2 constants
# ---------------------------------------------------------------------------

UF2_MAGIC1     = 0x0A324655
UF2_MAGIC2     = 0x9E5D5157
UF2_MAGIC_END  = 0x0AB16F30
UF2_BLOCK_SIZE = 512
UF2_PAYLOAD    = 256
UF2_FLAG_NOFLASH = 0x00000001
XIP_BASE       = 0x10000000
BANK_A_SIZE    = 0x0FF000      # 1 MB - 4 KB


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def progress_bar(done: int, total: int, width: int = PROGRESS_COLS) -> str:
    frac  = done / total if total else 0.0
    filled = int(frac * width)
    bar   = "#" * filled + "-" * (width - filled)
    pct   = frac * 100.0
    mb    = done / 1_048_576
    return f"[{bar}] {pct:5.1f}%  ({mb:.2f} MB)"


def recv_line(sock: socket.socket, timeout: float = 30.0) -> str:
    sock.settimeout(timeout)
    buf = b""
    deadline = time.time() + timeout
    while True:
        try:
            chunk = sock.recv(256)
        except socket.timeout:
            chunk = b""
        if not chunk:
            if time.time() >= deadline:
                raise TimeoutError("[OTA] Timed out waiting for server response")
            continue
        buf += chunk
        if b"\n" in buf:
            line, _, _ = buf.partition(b"\n")
            return line.decode(errors="replace").strip()


def compute_uf2_crc32(uf2_path: str) -> tuple[int, int]:
    """
    Parse the UF2 file and compute CRC32 of the binary image that will
    be written to Bank B.  Returns (image_size, crc32).

    This mirrors what the Pico computes: CRC32 over the flash region from
    offset 0 to the highest written byte (sector-aligned).
    """
    SECTOR_SIZE = 4096

    # First pass: determine image extent and collect payload pages
    pages: dict[int, bytes] = {}
    flash_hi = 0

    with open(uf2_path, "rb") as f:
        while True:
            block = f.read(UF2_BLOCK_SIZE)
            if len(block) < UF2_BLOCK_SIZE:
                break

            magic1, magic2, flags, target_addr, payload_size = struct.unpack_from("<5I", block, 0)
            magic_end = struct.unpack_from("<I", block, 508)[0]

            if magic1 != UF2_MAGIC1 or magic2 != UF2_MAGIC2 or magic_end != UF2_MAGIC_END:
                continue
            if flags & UF2_FLAG_NOFLASH:
                continue
            if payload_size != UF2_PAYLOAD:
                continue
            if target_addr < XIP_BASE:
                continue

            active_offset = target_addr - XIP_BASE
            if active_offset >= BANK_A_SIZE:
                continue

            pages[active_offset] = block[32:32 + UF2_PAYLOAD]
            end = active_offset + UF2_PAYLOAD
            if end > flash_hi:
                flash_hi = end

    if flash_hi == 0:
        return (0, 0)

    # Sector-align
    image_size = ((flash_hi + SECTOR_SIZE - 1) // SECTOR_SIZE) * SECTOR_SIZE

    # Build the flash image (0xFF for unwritten regions)
    image = bytearray(b'\xFF' * image_size)
    for offset, data in pages.items():
        image[offset:offset + len(data)] = data

    crc = zlib.crc32(bytes(image)) & 0xFFFFFFFF
    return (image_size, crc)


# ---------------------------------------------------------------------------
# Main OTA flow
# ---------------------------------------------------------------------------

def ota_flash(uf2_path: str, host: str, ota_port: int, cmd_port: int) -> None:

    # -- Validate file -------------------------------------------------------

    if not os.path.isfile(uf2_path):
        print(f"[OTA] Error: file not found: {uf2_path}", file=sys.stderr)
        sys.exit(1)

    uf2_size = os.path.getsize(uf2_path)
    if uf2_size == 0 or uf2_size % 512 != 0:
        print(f"[OTA] Error: {uf2_path} is not a valid UF2 file "
              f"(size={uf2_size} not a multiple of 512)", file=sys.stderr)
        sys.exit(1)

    # Pre-compute CRC32 of the UF2 payload image
    print(f"[OTA] Computing CRC32 of firmware image...")
    image_size, client_crc = compute_uf2_crc32(uf2_path)
    if image_size == 0:
        print("[OTA] Error: no valid UF2 payload blocks found", file=sys.stderr)
        sys.exit(1)
    print(f"[OTA] Image: {image_size:,} bytes, CRC32: 0x{client_crc:08X}")

    print(f"[OTA] Firmware : {uf2_path}  ({uf2_size:,} bytes, "
          f"{uf2_size // 512} blocks)")
    print(f"[OTA] Target   : {host}:{ota_port}")

    # -- Connect to OTA server -----------------------------------------------

    print(f"[OTA] Connecting...")
    try:
        sock = socket.create_connection((host, ota_port), timeout=10.0)
    except (ConnectionRefusedError, OSError) as e:
        print(f"[OTA] Cannot connect to {host}:{ota_port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        # -- Send V2 handshake header ----------------------------------------

        header = f"OTAV2 {uf2_size}\r\n".encode()
        sock.sendall(header)
        print(f"[OTA] Sent: OTAV2 {uf2_size}")

        # -- Wait for READY --------------------------------------------------

        print("[OTA] Waiting for READY (Pico pre-erasing staging flash)...")
        resp = recv_line(sock, timeout=120.0)
        print(f"[OTA] Server: {resp}")
        if resp.upper() != "READY":
            print(f"[OTA] Expected READY, got: {resp!r}", file=sys.stderr)
            sys.exit(1)

        # -- Stream UF2 data -------------------------------------------------

        print(f"[OTA] Streaming UF2...")
        sent  = 0
        t0    = time.time()

        with open(uf2_path, "rb") as f:
            sock.settimeout(30.0)

            while sent < uf2_size:
                chunk = f.read(SEND_CHUNK)
                if not chunk:
                    break

                # Drain progress lines
                sock.setblocking(False)
                try:
                    incoming = sock.recv(4096)
                    if incoming:
                        for line in incoming.decode(errors="replace").splitlines():
                            line = line.strip()
                            if line:
                                print(f"\r[OTA] Server: {line:<60}")
                except BlockingIOError:
                    pass
                sock.setblocking(True)
                sock.settimeout(30.0)

                sock.sendall(chunk)
                sent += len(chunk)

                bar = progress_bar(sent, uf2_size)
                print(f"\r{bar}", end="", flush=True)

        elapsed = time.time() - t0
        speed   = sent / elapsed / 1024.0 if elapsed > 0 else 0.0
        print(f"\n[OTA] Sent {sent:,} bytes in {elapsed:.1f} s  ({speed:.0f} KB/s)")

        # -- Wait for CRC32 + DONE ------------------------------------------

        print("[OTA] Waiting for server CRC32 verification and confirmation...")
        server_crc = None

        while True:
            resp = recv_line(sock, timeout=60.0)
            print(f"[OTA] Server: {resp}")

            if resp.upper().startswith("CRC32 "):
                try:
                    server_crc = int(resp.split()[1], 16)
                except (IndexError, ValueError):
                    print(f"[OTA] Warning: could not parse CRC32 from: {resp}")

            elif resp.upper() == "DONE":
                break

            elif resp.upper().startswith("ERR"):
                print(f"[OTA] Server reported error: {resp}", file=sys.stderr)
                sys.exit(1)

        # -- Verify CRC match ------------------------------------------------

        if server_crc is not None:
            if server_crc == client_crc:
                print(f"[OTA] OK CRC32 MATCH: client=0x{client_crc:08X} "
                      f"server=0x{server_crc:08X}")
            else:
                print(f"[OTA] FAIL CRC32 MISMATCH! client=0x{client_crc:08X} "
                      f"server=0x{server_crc:08X}", file=sys.stderr)
                print("[OTA] The firmware image may be corrupt. "
                      "The Pico will verify CRC before applying.", file=sys.stderr)
        else:
            print("[OTA] Note: server did not send CRC32 (V1 compatibility mode)")

    finally:
        try:
            sock.close()
        except Exception:
            pass

    # -- Wait for Pico to backup, apply, and reboot ---------------------------

    print(f"\n[OTA] Pico is backing up current firmware, applying update, "
          f"and rebooting...")
    print(f"[OTA] This takes longer than before (backup + apply + verify)...")
    print(f"[OTA] Waiting {REBOOT_WAIT_S} s...")
    time.sleep(REBOOT_WAIT_S)

    # -- Probe CMD port to confirm new firmware is running ---------------------

    print(f"[OTA] Probing CMD port {host}:{cmd_port} ...")
    deadline = time.time() + PROBE_TIMEOUT_S
    attempt  = 0

    while time.time() < deadline:
        attempt += 1
        try:
            s2 = socket.create_connection((host, cmd_port), timeout=2.0)
            s2.close()
            elapsed = REBOOT_WAIT_S + attempt * PROBE_INTERVAL
            print(f"\n[OTA] Pico CMD port online (~{elapsed:.0f} s after DONE).")
            print("[OTA] OK OTA complete -- new firmware is running!")
            print("[OTA] Note: firmware must fully boot (WiFi + servers) to be")
            print("[OTA]       marked as confirmed. If it fails 3 times, the")
            print("[OTA]       Pico will automatically revert to the backup.")
            return
        except OSError:
            remaining = int(deadline - time.time())
            print(f"  [{attempt}] not yet... ({remaining} s remaining)", end="\r")
            time.sleep(PROBE_INTERVAL)

    print(f"\n[OTA] CMD port not reachable at {host}:{cmd_port} after "
          f"{PROBE_TIMEOUT_S} s.")
    print("[OTA] The Pico may have received a new DHCP address, or is still")
    print("[OTA] applying the firmware (backup + apply can take ~30 s).")
    print("[OTA] If the new firmware is bad, the Pico will automatically")
    print("[OTA] revert to the backup after 3 failed boot attempts.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="MushIO V1.0 -- Reliable OTA Firmware Client (V2 protocol)",
        epilog=("Connects to the Pico OTA server on port 9002 and streams the\n"
                "UF2 image over WiFi.  Includes CRC32 verification and the Pico\n"
                "automatically backs up current firmware before applying.\n"
                "If the new firmware fails to boot 3 times, the Pico reverts."),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "uf2",
        metavar="firmware.uf2",
        help="Path to the .uf2 firmware file",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help=f"Pico IP address (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_OTA_PORT,
        help=f"OTA TCP port (default: {DEFAULT_OTA_PORT})",
    )
    parser.add_argument(
        "--cmd-port",
        type=int,
        default=DEFAULT_CMD_PORT,
        help=f"CMD port for post-reboot probe (default: {DEFAULT_CMD_PORT})",
    )

    args = parser.parse_args()
    ota_flash(args.uf2, args.host, args.port, args.cmd_port)


if __name__ == "__main__":
    main()
