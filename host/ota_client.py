"""
MushIO V1.0  —  OTA Firmware Flash Client

Sends the 'ota_reboot' command to the Pico's CMD port (9001),
waits for the RPI-RP2 USB mass-storage drive to appear,
then copies the given .uf2 file onto it.

Usage
-----
    python ota_client.py firmware.uf2
    python ota_client.py firmware.uf2 --host 192.168.68.115 --port 9001
    python ota_client.py firmware.uf2 --host 192.168.68.115 --no-reboot
        (--no-reboot: skip sending ota_reboot — Pico is already in BOOTSEL)

Requirements
------------
    Python 3.8+   (no external dependencies)

Platform notes
--------------
    Windows : scans drive letters D..Z for a volume named RPI-RP2.
    macOS   : checks /Volumes/RPI-RP2
    Linux   : checks /media/*/RPI-RP2  and  /run/media/*/RPI-RP2
"""

import argparse
import os
import platform
import shutil
import socket
import sys
import time


# ---------------------------------------------------------------------------
# Default network settings (match config.h)
# ---------------------------------------------------------------------------

DEFAULT_HOST = "192.168.68.115"
DEFAULT_PORT = 9001

BOOTSEL_TIMEOUT_S = 60    # seconds to wait for RPI-RP2 drive
POLL_INTERVAL_S   = 0.5   # polling interval while waiting


# ---------------------------------------------------------------------------
# Find the RPI-RP2 USB mass-storage drive
# ---------------------------------------------------------------------------

def find_rpi_drive() -> str | None:
    """
    Return the mount-point / drive-root of the RPI-RP2 volume, or None.
    """
    system = platform.system()

    if system == "Windows":
        import string
        import ctypes
        DRIVE_REMOVABLE = 2
        for letter in string.ascii_uppercase[3:]:  # D..Z
            drive = f"{letter}:\\"
            try:
                if ctypes.windll.kernel32.GetDriveTypeW(drive) == DRIVE_REMOVABLE:
                    vol_buf  = ctypes.create_unicode_buffer(1024)
                    fs_buf   = ctypes.create_unicode_buffer(1024)
                    ret = ctypes.windll.kernel32.GetVolumeInformationW(
                        drive, vol_buf, len(vol_buf),
                        None, None, None,
                        fs_buf, len(fs_buf)
                    )
                    if ret and vol_buf.value.upper() in ("RPI-RP2", "RP2350"):
                        return drive
            except Exception:
                pass

    elif system == "Darwin":
        path = "/Volumes/RPI-RP2"
        if os.path.isdir(path):
            return path
        path = "/Volumes/RP2350"
        if os.path.isdir(path):
            return path

    else:  # Linux
        for base in ("/media", "/run/media"):
            if not os.path.isdir(base):
                continue
            for user in os.listdir(base):
                for name in ("RPI-RP2", "RP2350"):
                    candidate = os.path.join(base, user, name)
                    if os.path.isdir(candidate):
                        return candidate
                # Also check without user sub-dir
                for name in ("RPI-RP2", "RP2350"):
                    candidate = os.path.join(base, name)
                    if os.path.isdir(candidate):
                        return candidate

    return None


# ---------------------------------------------------------------------------
# Send ota_reboot over CMD TCP
# ---------------------------------------------------------------------------

def send_ota_reboot(host: str, port: int, timeout: float = 5.0) -> bool:
    """
    Connect to CMD server and send 'ota_reboot'.
    Returns True on success, False on failure.
    """
    print(f"[OTA] Connecting to {host}:{port} ...")
    try:
        sock = socket.create_connection((host, port), timeout=timeout)
    except (ConnectionRefusedError, OSError) as e:
        print(f"[OTA] Cannot connect: {e}")
        return False

    try:
        sock.settimeout(timeout)
        # Drain welcome banner (up to 2 KB)
        try:
            banner = sock.recv(2048)
            print("[OTA] Banner:")
            for line in banner.decode(errors='replace').splitlines():
                print(f"  {line}")
        except socket.timeout:
            pass

        # Send command
        sock.sendall(b"ota_reboot\r\n")
        print("[OTA] Sent: ota_reboot")

        # Read response lines for up to 3 s
        time.sleep(0.1)
        sock.settimeout(3.0)
        try:
            resp = sock.recv(2048)
            for line in resp.decode(errors='replace').splitlines():
                print(f"  {line}")
        except socket.timeout:
            pass

    finally:
        try:
            sock.close()
        except Exception:
            pass

    return True


# ---------------------------------------------------------------------------
# Main OTA flow
# ---------------------------------------------------------------------------

def ota_flash(uf2_path: str, host: str, port: int, no_reboot: bool):
    # Validate UF2 file
    if not os.path.isfile(uf2_path):
        print(f"[OTA] Error: file not found: {uf2_path}", file=sys.stderr)
        sys.exit(1)

    uf2_size = os.path.getsize(uf2_path)
    print(f"[OTA] UF2 file : {uf2_path}  ({uf2_size:,} bytes)")

    # Step 1: Trigger reboot via CMD port
    if not no_reboot:
        ok = send_ota_reboot(host, port)
        if not ok:
            print("[OTA] Could not reach CMD port - put Pico in BOOTSEL mode now.")

    # Step 2: Wait for RPI-RP2 drive
    print(f"\n[OTA] Waiting for RPI-RP2 drive (up to {BOOTSEL_TIMEOUT_S} s)...")
    deadline = time.time() + BOOTSEL_TIMEOUT_S
    drive    = None
    dots     = 0

    while time.time() < deadline:
        drive = find_rpi_drive()
        if drive:
            break
        print(".", end="", flush=True)
        dots += 1
        if dots % 40 == 0:
            print()
        time.sleep(POLL_INTERVAL_S)

    print()   # newline after dots

    if drive is None:
        print("[OTA] Timed out waiting for RPI-RP2 drive.")
        print("[OTA] Hold BOOTSEL on the Pico and plug USB; "
              "then re-run with --no-reboot.")
        sys.exit(1)

    print(f"[OTA] Found drive: {drive}")

    # Step 3: Copy UF2
    dest = os.path.join(drive, os.path.basename(uf2_path))
    print(f"[OTA] Copying {uf2_path} → {dest} ...")

    try:
        shutil.copy2(uf2_path, dest)
    except Exception as e:
        print(f"[OTA] Copy failed: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"[OTA] Copy complete ({uf2_size:,} bytes written).")
    print("[OTA] Pico will reboot automatically after flashing.")
    print("[OTA] Waiting 10 s for Pico WiFi to come back online...")
    time.sleep(10)

    # Step 4: Quick connectivity check (best-effort — flash already succeeded)
    print(f"[OTA] Probing CMD port {host}:{port} ...")
    for attempt in range(10):
        try:
            sock = socket.create_connection((host, port), timeout=2.0)
            sock.close()
            print(f"[OTA] Pico CMD port online after ~{10 + (attempt + 1) * 2} s.")
            return
        except OSError:
            time.sleep(2)
            print(f"  [{attempt + 1}/10] not yet...")

    # Probe failed — but the UF2 copy succeeded so exit 0.
    # The Pico may have a new DHCP IP; the GUI will find it via the data stream.
    print("[OTA] CMD port not reachable at this IP (Pico may have a new DHCP address).")
    print("[OTA] Flash complete — GUI will connect once the Pico rejoins the network.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="MushIO V1.0 OTA Firmware Flash Client"
    )
    parser.add_argument(
        'uf2', metavar='firmware.uf2',
        help='Path to the .uf2 firmware file'
    )
    parser.add_argument(
        '--host', default=DEFAULT_HOST,
        help=f'Pico IP address (default: {DEFAULT_HOST})'
    )
    parser.add_argument(
        '--port', type=int, default=DEFAULT_PORT,
        help=f'CMD TCP port (default: {DEFAULT_PORT})'
    )
    parser.add_argument(
        '--no-reboot', action='store_true',
        help='Skip sending ota_reboot; assume Pico is already in BOOTSEL mode'
    )
    args = parser.parse_args()

    ota_flash(args.uf2, args.host, args.port, args.no_reboot)


if __name__ == '__main__':
    main()
