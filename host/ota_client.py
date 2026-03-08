"""
MushIO V1.0  —  True Wireless OTA Firmware Client

Connects directly to the Pico's OTA server on port 9002 and streams the
UF2 firmware image over WiFi.  No USB cable or BOOTSEL mode needed after
the first USB flash.

Protocol
--------
    Client → "OTAV1 {uf2_size}\\r\\n"
    Server → "READY\\r\\n"
    Client → {uf2_size bytes of raw UF2 data}
    Server → "PROG {pct}%\\r\\n"    (progress, every ~16 KB)
    Server → "DONE\\r\\n"            (written to staging, rebooting)
    Server → "ERR {reason}\\r\\n"    (on failure)

After DONE, the Pico reboots, applies the new firmware from its staging
area, and comes back online.  This client waits and confirms reconnect.

Usage
-----
    python ota_client.py firmware.uf2
    python ota_client.py firmware.uf2 --host 192.168.68.115
    python ota_client.py firmware.uf2 --host 192.168.68.115 --port 9002

Requirements
------------
    Python 3.8+   (no external dependencies)
"""

import argparse
import os
import socket
import sys
import time


# ---------------------------------------------------------------------------
# Defaults (must match config.h / ota_server.h)
# ---------------------------------------------------------------------------

DEFAULT_HOST     = "192.168.68.115"
DEFAULT_OTA_PORT = 9002
DEFAULT_CMD_PORT = 9001   # used only for post-reboot probe

SEND_CHUNK    = 4096      # bytes per socket.send() call
PROGRESS_COLS = 40        # width of the progress bar

REBOOT_WAIT_S   = 15      # seconds to wait for Pico to reboot and apply OTA
PROBE_TIMEOUT_S = 60      # seconds to wait for CMD port to come back
PROBE_INTERVAL  = 2.0     # seconds between CMD port probes


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
    """
    Read bytes from sock until '\\n' is found or timeout expires.
    Returns the decoded line (stripped of CR/LF).
    """
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
        # -- Send handshake header -------------------------------------------

        header = f"OTAV1 {uf2_size}\r\n".encode()
        sock.sendall(header)
        print(f"[OTA] Sent: OTAV1 {uf2_size}")

        # -- Wait for READY --------------------------------------------------

        resp = recv_line(sock, timeout=10.0)
        print(f"[OTA] Server: {resp}")
        if resp.upper() != "READY":
            print(f"[OTA] Expected READY, got: {resp!r}", file=sys.stderr)
            sys.exit(1)

        # -- Stream UF2 data -------------------------------------------------

        print(f"[OTA] Streaming UF2...")
        sent  = 0
        t0    = time.time()

        with open(uf2_path, "rb") as f:
            sock.settimeout(30.0)   # generous per-chunk timeout

            while sent < uf2_size:
                chunk = f.read(SEND_CHUNK)
                if not chunk:
                    break

                # Drain any progress lines that arrived while we were sending
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

                # Overwrite the current line with a progress bar
                bar = progress_bar(sent, uf2_size)
                print(f"\r{bar}", end="", flush=True)

        elapsed = time.time() - t0
        speed   = sent / elapsed / 1024.0 if elapsed > 0 else 0.0
        print(f"\n[OTA] Sent {sent:,} bytes in {elapsed:.1f} s  ({speed:.0f} KB/s)")

        # -- Wait for DONE (or ERR) -----------------------------------------

        print("[OTA] Waiting for server confirmation...")
        while True:
            resp = recv_line(sock, timeout=60.0)
            print(f"[OTA] Server: {resp}")
            if resp.upper() == "DONE":
                break
            if resp.upper().startswith("ERR"):
                print(f"[OTA] Server reported error: {resp}", file=sys.stderr)
                sys.exit(1)
            # PROG messages are informational — keep waiting

    finally:
        try:
            sock.close()
        except Exception:
            pass

    # -- Wait for Pico to apply OTA and reboot --------------------------------

    print(f"\n[OTA] Pico is applying firmware and rebooting...")
    print(f"[OTA] Waiting {REBOOT_WAIT_S} s for apply + reboot...")
    time.sleep(REBOOT_WAIT_S)

    # -- Probe CMD port to confirm the new firmware is running ----------------

    print(f"[OTA] Probing CMD port {host}:{cmd_port} ...")
    deadline = time.time() + PROBE_TIMEOUT_S
    attempt  = 0

    while time.time() < deadline:
        attempt += 1
        try:
            s2 = socket.create_connection((host, cmd_port), timeout=2.0)
            s2.close()
            elapsed = REBOOT_WAIT_S + attempt * PROBE_INTERVAL
            print(f"[OTA] Pico CMD port online (~{elapsed:.0f} s after DONE).")
            print("[OTA] OTA complete — new firmware is running.")
            return
        except OSError:
            remaining = int(deadline - time.time())
            print(f"  [{attempt}] not yet... ({remaining} s remaining)", end="\r")
            time.sleep(PROBE_INTERVAL)

    print(f"\n[OTA] CMD port not reachable at {host}:{cmd_port} after "
          f"{PROBE_TIMEOUT_S} s.")
    print("[OTA] The Pico may have received a new DHCP address.")
    print("[OTA] Flash write succeeded — GUI will reconnect once Pico is online.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="MushIO V1.0 — True Wireless OTA Firmware Client",
        epilog=("Connects to the Pico OTA server on port 9002 and streams the\n"
                "UF2 image over WiFi.  No USB cable needed."),
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
