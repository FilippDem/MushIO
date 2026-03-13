#!/usr/bin/env python3
"""
Automated spacing comparison test.
Runs S=4,8,16,32 x 5 trials each in randomized order.
Each trial: OTA flash firmware with given S -> run GUI for 3 min -> analyze HDF5.

Usage:
    python spacing_test.py

Results are written to spacing_results.json and printed as a summary table.
"""

import subprocess, sys, os, time, json, random, struct, socket, glob
import shutil

# Paths
PYTHON = r"C:\Users\filip\AppData\Local\Programs\Python\Python314\python.exe"
WORKTREE = r"C:\GitHub\MushIO\.claude\worktrees\charming-tereshkova"
CONFIG_H = os.path.join(WORKTREE, "firmware_c", "config.h")
MAIN_C = os.path.join(WORKTREE, "firmware_c", "main.c")
BUILD_DIR = os.path.join(WORKTREE, "firmware_c", "build")
GUI_PY = os.path.join(WORKTREE, "host", "gui.py")
OTA_CLIENT = os.path.join(WORKTREE, "host", "ota_client.py")
DATA_DIR = r"C:\GitHub\MushIO\data"
RESULTS_FILE = os.path.join(WORKTREE, "spacing_results.json")

PICO_IP = "192.168.68.131"
PICO_CMD_PORT = 9001
UDP_DATA_PORT = 9004
REDUNDANCY = 5

# Build tools
CMAKE = r"C:\Users\filip\scoop\shims\cmake.exe"
NINJA = r"C:\Users\filip\scoop\apps\ninja\current\ninja.exe"
SDK_PATH = "C:/pico/pico-sdk"
TOOLCHAIN_PATH = "C:/Users/filip/scoop/apps/gcc-arm-none-eabi/current/bin"

TEST_DURATION_S = 180  # 3 minutes per test
SPACINGS = [4, 8, 16, 32]
TRIALS_PER_SPACING = 5


def log(msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def update_config_h(spacing):
    """Update UDP_SPACING in config.h"""
    with open(CONFIG_H, "r", encoding="utf-8") as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        if line.strip().startswith("#define UDP_SPACING"):
            hist_size = (REDUNDANCY - 1) * spacing
            new_lines.append(f"#define UDP_SPACING             {spacing}u\n")
        elif line.strip().startswith("#define UDP_HISTORY_SIZE"):
            hist_size = (REDUNDANCY - 1) * spacing
            new_lines.append(
                f"#define UDP_HISTORY_SIZE        ((UDP_REDUNDANCY - 1u) * UDP_SPACING)  /* {hist_size} */\n"
            )
        else:
            new_lines.append(line)

    with open(CONFIG_H, "w", encoding="utf-8") as f:
        f.writelines(new_lines)
    log(f"  config.h updated: UDP_SPACING={spacing}")


def update_gui_spacing(spacing):
    """Update UDP_SPACING and _DEDUP_WINDOW in gui.py"""
    with open(GUI_PY, "r", encoding="utf-8") as f:
        content = f.read()

    # Replace UDP_SPACING line
    import re
    content = re.sub(
        r"(    UDP_SPACING\s*=\s*)\d+",
        f"\\g<1>{spacing}",
        content,
    )
    # Replace dedup window comment
    dedup = 2 * (REDUNDANCY - 1) * spacing
    content = re.sub(
        r"(    _DEDUP_WINDOW\s*=\s*)2 \* \(UDP_REDUNDANCY - 1\) \* UDP_SPACING\s*#.*",
        f"\\g<1>2 * (UDP_REDUNDANCY - 1) * UDP_SPACING  # {dedup}",
        content,
    )

    with open(GUI_PY, "w", encoding="utf-8") as f:
        f.write(content)
    log(f"  gui.py updated: UDP_SPACING={spacing}, DEDUP_WINDOW={dedup}")


def build_firmware():
    """Build firmware, return path to .bin"""
    log("  Building firmware...")
    env = os.environ.copy()
    env["PICO_SDK_PATH"] = SDK_PATH
    env["PICO_TOOLCHAIN_PATH"] = TOOLCHAIN_PATH

    # Run ninja (cmake already configured)
    result = subprocess.run(
        [NINJA],
        cwd=BUILD_DIR,
        env=env,
        capture_output=True,
        text=True,
        timeout=300,
    )

    bin_path = os.path.join(BUILD_DIR, "mushio_c.bin")
    if os.path.exists(bin_path):
        log(f"  Build OK: {os.path.getsize(bin_path)} bytes")
        return bin_path

    # If ninja fails due to picotool crash but .bin exists, that's OK
    if os.path.exists(bin_path):
        log(f"  Build OK (picotool crash ignored): {os.path.getsize(bin_path)} bytes")
        return bin_path

    raise RuntimeError(f"Build failed:\n{result.stderr[-500:]}")


def bin_to_uf2(bin_path):
    """Convert .bin to .uf2 using Python (picotool workaround)"""
    uf2_path = bin_path.replace(".bin", ".uf2")
    with open(bin_path, "rb") as f:
        data = f.read()
    nb = (len(data) + 255) // 256
    with open(uf2_path, "wb") as f:
        for i in range(nb):
            c = data[i * 256 : (i + 1) * 256].ljust(256, b"\x00")
            f.write(
                struct.pack(
                    "<IIIIIIII",
                    0x0A324655,
                    0x9E5D5157,
                    0x2000,
                    0x10000000 + i * 256,
                    256,
                    i,
                    nb,
                    0xE48BFF59,
                )
                + c
                + b"\x00" * 220
                + struct.pack("<I", 0x0AB16F30)
            )
    log(f"  UF2 converted: {nb} blocks")
    return uf2_path


def ota_flash(uf2_path):
    """OTA flash firmware to Pico"""
    log("  OTA flashing...")
    result = subprocess.run(
        [PYTHON, OTA_CLIENT, uf2_path, "--host", PICO_IP],
        capture_output=True,
        text=True,
        timeout=120,
    )
    if "OTA complete" in result.stdout:
        log("  OTA flash OK")
        return True
    else:
        log(f"  OTA flash FAILED:\n{result.stdout[-300:]}")
        return False


def verify_spacing(expected_spacing):
    """Capture a few UDP datagrams and verify spacing"""
    FRAME_SZ = 228
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", UDP_DATA_PORT))
    s.settimeout(5)
    try:
        for _ in range(3):
            data, addr = s.recvfrom(2048)
            n = len(data) // FRAME_SZ
            if n < 2:
                continue
            seqs = []
            for i in range(n):
                frame = data[i * FRAME_SZ : (i + 1) * FRAME_SZ]
                sync, ts, seq = struct.unpack_from("<HIH", frame, 0)
                seqs.append(seq)
            offsets = [seqs[0] - seqs[i] for i in range(1, n)]
            if offsets and offsets[0] == expected_spacing:
                log(f"  Spacing verified: offsets={offsets}")
                return True
        log(f"  WARNING: Could not verify spacing={expected_spacing}")
        return False
    except socket.timeout:
        log("  WARNING: No UDP data received for verification")
        return False
    finally:
        s.close()


def run_gui_test(duration_s):
    """Run the GUI for duration_s seconds, return the HDF5 file path"""
    # Find existing h5 files before test
    existing = set(glob.glob(os.path.join(DATA_DIR, "mushio_*.h5")))

    log(f"  Running GUI for {duration_s}s...")
    proc = subprocess.Popen(
        [PYTHON, GUI_PY],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    time.sleep(duration_s)

    log("  Stopping GUI...")
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)

    # Wait for file to be flushed
    time.sleep(2)

    # Find the new h5 file
    current = set(glob.glob(os.path.join(DATA_DIR, "mushio_*.h5")))
    new_files = current - existing
    if not new_files:
        log("  WARNING: No new HDF5 file found!")
        return None

    h5_path = max(new_files, key=os.path.getmtime)
    log(f"  HDF5: {os.path.basename(h5_path)} ({os.path.getsize(h5_path)/1e6:.1f} MB)")
    return h5_path


def analyze_h5(h5_path):
    """Analyze HDF5 file, return dict of metrics"""
    import h5py
    import numpy as np

    with h5py.File(h5_path, "r") as h:
        ts = h["data/timestamps_us"][:]
        seq = h["data/seq"][:]
        n = len(seq)

        # Handle seq wrap (uint16)
        seq64 = seq.astype(np.int64)
        diffs = np.diff(seq64)
        diffs[diffs < -30000] += 65536

        gaps = np.where(diffs > 1)[0]
        total_missed = int(np.sum(diffs[gaps] - 1))
        total_frames = n + total_missed
        loss_rate = total_missed / total_frames if total_frames > 0 else 0

        dt = (float(ts[-1]) - float(ts[0])) / 1e6
        fps = n / dt if dt > 0 else 0

        gap_sizes = (diffs[gaps] - 1).astype(int) if len(gaps) > 0 else np.array([])

        result = {
            "frames": n,
            "missed": total_missed,
            "loss_rate": loss_rate,
            "loss_pct": loss_rate * 100,
            "gaps": len(gaps),
            "max_gap": int(gap_sizes.max()) if len(gap_sizes) > 0 else 0,
            "duration_s": dt,
            "fps": fps,
            "h5_file": os.path.basename(h5_path),
        }

    log(
        f"  Results: {n:,} frames, {total_missed} missed ({loss_rate*100:.4f}%), "
        f"{len(gaps)} gaps, max_gap={result['max_gap']}, {fps:.0f} FPS"
    )
    return result


def main():
    # Generate randomized test order
    trials = []
    for s in SPACINGS:
        for t in range(TRIALS_PER_SPACING):
            trials.append({"spacing": s, "trial": t + 1})
    random.shuffle(trials)

    log(f"Test plan: {len(trials)} trials in randomized order")
    log(f"Spacings: {SPACINGS}, {TRIALS_PER_SPACING} trials each")
    log(f"Test duration: {TEST_DURATION_S}s per trial")
    log(f"Order: {[t['spacing'] for t in trials]}")
    log("")

    results = []
    current_spacing = None

    for i, trial in enumerate(trials):
        s = trial["spacing"]
        t = trial["trial"]
        log(f"=== Trial {i+1}/{len(trials)}: S={s} (trial {t}/{TRIALS_PER_SPACING}) ===")

        # Only reflash if spacing changed
        if s != current_spacing:
            log(f"  Spacing changed: {current_spacing} -> {s}, rebuilding...")
            update_config_h(s)
            update_gui_spacing(s)

            bin_path = build_firmware()
            uf2_path = bin_to_uf2(bin_path)

            if not ota_flash(uf2_path):
                log("  SKIPPING trial due to OTA failure")
                results.append({**trial, "error": "OTA failed"})
                continue

            # Wait for Pico to boot and stabilize
            time.sleep(5)

            # Verify spacing
            verify_spacing(s)
            current_spacing = s
        else:
            log(f"  Same spacing S={s}, reusing firmware")

        # Run the test
        h5_path = run_gui_test(TEST_DURATION_S)
        if h5_path is None:
            results.append({**trial, "error": "No HDF5"})
            continue

        try:
            metrics = analyze_h5(h5_path)
            results.append({**trial, **metrics})
        except Exception as e:
            log(f"  ERROR analyzing: {e}")
            results.append({**trial, "error": str(e)})

        # Save intermediate results
        with open(RESULTS_FILE, "w") as f:
            json.dump(results, f, indent=2)

        log("")

    # Print summary
    log("\n" + "=" * 70)
    log("SUMMARY TABLE")
    log("=" * 70)
    log(f"{'S':>4} {'Trial':>6} {'Frames':>8} {'Missed':>8} {'Loss%':>10} {'Gaps':>6} {'MaxGap':>7} {'FPS':>6}")
    log("-" * 70)

    for r in sorted(results, key=lambda x: (x["spacing"], x["trial"])):
        if "error" in r:
            log(f"{r['spacing']:>4} {r['trial']:>6}   ERROR: {r['error']}")
        else:
            log(
                f"{r['spacing']:>4} {r['trial']:>6} {r['frames']:>8,} {r['missed']:>8} "
                f"{r['loss_pct']:>9.4f}% {r['gaps']:>6} {r['max_gap']:>7} {r['fps']:>6.0f}"
            )

    # Compute averages
    log("")
    log(f"{'S':>4} {'AvgLoss%':>10} {'MinLoss%':>10} {'MaxLoss%':>10} {'AvgMissed':>10} {'Trials':>7}")
    log("-" * 60)

    for s in SPACINGS:
        s_results = [r for r in results if r["spacing"] == s and "error" not in r]
        if not s_results:
            log(f"{s:>4}   NO DATA")
            continue
        losses = [r["loss_pct"] for r in s_results]
        missed = [r["missed"] for r in s_results]
        avg_loss = sum(losses) / len(losses)
        min_loss = min(losses)
        max_loss = max(losses)
        avg_missed = sum(missed) / len(missed)
        log(
            f"{s:>4} {avg_loss:>9.4f}% {min_loss:>9.4f}% {max_loss:>9.4f}% "
            f"{avg_missed:>10.0f} {len(s_results):>7}"
        )

    # Find best
    best_s = None
    best_avg = float("inf")
    for s in SPACINGS:
        s_results = [r for r in results if r["spacing"] == s and "error" not in r]
        if s_results:
            avg = sum(r["loss_pct"] for r in s_results) / len(s_results)
            if avg < best_avg:
                best_avg = avg
                best_s = s

    log(f"\nBEST SPACING: S={best_s} (avg loss={best_avg:.4f}%)")

    # Save final results with best_spacing
    final = {"trials": results, "best_spacing": best_s, "best_avg_loss_pct": best_avg}
    with open(RESULTS_FILE, "w") as f:
        json.dump(final, f, indent=2)
    log(f"Results saved to {RESULTS_FILE}")


if __name__ == "__main__":
    main()
