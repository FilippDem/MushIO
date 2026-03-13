# MushIO V1.0 — Getting Started

72-channel electrophysiology recorder for mushroom mycelium studies.
Pico 2W (C firmware) streams 500 FPS over WiFi to a Python GUI.

---

## 1. Prerequisites

### Host PC
- Python 3.10 or later
- Run `install.bat` (Windows) or `install.sh` (macOS/Linux) to install dependencies
- Or manually: `pip install -r host/requirements.txt`

### Hardware
- MushIO V1.0 board (Raspberry Pi Pico 2W + ADS124S08 ×6)
- USB cable (for first flash)
- Same WiFi network as the host PC

### Firmware build tools (only needed to recompile firmware)
- [CMake](https://cmake.org/) 3.13+
- [Ninja](https://ninja-build.org/)
- [ARM GCC toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (arm-none-eabi-gcc)
- [Pico SDK](https://github.com/raspberrypi/pico-sdk) — clone and set `PICO_SDK_PATH`

---

## 2. Try the GUI in Demo Mode (no hardware needed)

Double-click **`launch_gui_demo.bat`** — or from a terminal:

```bash
python host/gui.py --demo
```

This streams 500 FPS of simulated electrophysiology data so you can
explore all UI controls before hardware arrives.

---

## 3. Configure and Build Firmware

### 3a. Edit WiFi credentials

Edit `firmware_c/config.h`:

```c
#define WIFI_SSID       "YourNetworkName"
#define WIFI_PASSWORD   "YourPassword"
#define HOST_IP         "192.168.1.X"   // your PC's IP — run ipconfig to find it
```

> **Tip:** On Windows, open a terminal and run `ipconfig`. Look for the IPv4 address
> on your WiFi adapter (e.g. `192.168.1.42`).

### 3b. Build the firmware

Set up your environment and run the build script:

```bat
set PICO_SDK_PATH=C:\path\to\pico-sdk
build_and_launch.bat
```

Or build manually:

```bash
cd firmware_c
mkdir build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DMUSHIO_DEMO=1
ninja -j4
```

Set `-DMUSHIO_DEMO=0` for real ADC hardware (production mode).

The build produces `firmware_c/build/mushio_c.uf2`.

---

## 4. Flash Firmware to the Pico

### Option A: USB BOOTSEL (first time)

1. Hold the **BOOTSEL** button while plugging in USB.
2. A drive called `RPI-RP2` appears — copy `mushio_c.uf2` onto it.

### Option B: OTA (over WiFi, for updates)

If the Pico is already running MushIO firmware:

```bash
python host/ota_client.py firmware_c/build/mushio_c.uf2 --host <PICO_IP>
```

The `build_and_launch.bat` script handles both methods automatically.

---

## 5. Start Recording

### 5a. Launch the GUI

Double-click **`launch_gui.bat`**, or:

```bash
python host/gui.py
```

### 5b. The Pico connects automatically

After flashing, the Pico will:
1. Connect to your WiFi network
2. Broadcast UDP beacon on port 9003 every 3 seconds
3. Start streaming data via UDP to your PC on port 9004
4. Listen for commands on TCP port 9001

The GUI's **Data stream** indicator turns green when data arrives.
Recording starts automatically when the Pico connects.

### 5c. Connect the command channel (optional)

Enter the Pico's IP in the **Command → Pico IP** field and click **Connect**.
This enables diagnostic commands and register configuration.

---

## 6. GUI Quick Reference

### Electrode Grid (8×8)

The main view shows all 64 electrode channels as an 8×8 grid with
real-time waveforms. Each cell is color-coded by signal amplitude.

### Waveform panel

| Control | Function |
|---------|----------|
| Time window | Scroll history: 1–20 seconds visible |
| Y-scale | Manual voltage range (disabled when Auto-scale is on) |
| Auto-scale Y | Fit waveform to window automatically |
| Channel listbox | Select up to 8 channels to display simultaneously |
| Bandpass filter | Configurable low/high cutoff for electrode channels |

### Command terminal

Type commands and press **Enter** or click **Send**:

```
ping              — check Pico is alive
status            — streaming stats (fps, dropped frames, WiFi state)
set_fps <hz>      — change scan rate (default: 500 FPS)
check_adcs        — verify all 6 ADC device IDs
scan_all          — print current value of all 72 channels
benchmark         — measure maximum scan_frame() rate
set_datarate 0x0D — set 4000 SPS on all ADCs
dump_regs 0       — print all registers on ADC0
```

### Recording

| Feature | Details |
|---------|---------|
| Format | HDF5 (.h5) with gzip compression |
| Auto-record | Starts when Pico connects, survives brief disconnects (60s grace) |
| Data dir | Configurable in Settings tab (default: `data/`) |
| Rolling | New file every hour (configurable) |
| Gap markers | Sequence gaps recorded in `data/gap_events` dataset |
| Recovery | Always-on ring buffer saves last ~20 min to disk |

### Webcam capture

1. Tick **Enable** in the webcam panel
2. Select camera index (0 = default webcam)
3. Choose capture interval (1–60 minutes)
4. Images saved alongside data files

---

## 7. Data Output

Recordings are saved in HDF5 format (`.h5`) to the configured data directory:

```
data/
  mushio_20260313_143000.h5
  mushio_20260313_153000.h5
```

Each file contains:

| Dataset | Shape | Description |
|---------|-------|-------------|
| `data/samples` | (N, 72) | Raw ADC samples, int32 |
| `data/timestamps_us` | (N,) | Pico microsecond timestamps |
| `data/seq` | (N,) | Sequence numbers (gap detection) |
| `data/gap_events` | (M, 5) | Detected gaps: frame_idx, expected_seq, actual_seq, gap_size, wall_clock |
| `channels` | (72,) | Channel names (ELEC00–ELEC63, STIM_0–STIM_7) |
| `channel_types` | (72,) | "recording" or "stim" |
| `metadata/*` | attrs | Hardware config (Vref, gain, ADC settings) |
| `experiment/*` | attrs | Start time, board ID, species info |

### Reading data in Python

```python
import h5py
import numpy as np

with h5py.File('data/mushio_20260313_143000.h5', 'r') as f:
    samples = f['data/samples'][:]      # (N, 72) array
    timestamps = f['data/timestamps_us'][:]
    seq = f['data/seq'][:]
    gaps = f['data/gap_events'][:]      # (M, 5) — empty if no gaps
    channels = [c.decode() for c in f['channels'][:]]
    print(f"Frames: {len(seq)}, Duration: {(timestamps[-1]-timestamps[0])/1e6:.1f}s")
```

---

## 8. Network Architecture

```
Pico 2W (firmware_c)              Host PC (host/gui.py)
─────────────────────             ─────────────────────
UDP beacon :9003  ───────────────▶  Discovery listener
UDP data   :9004  ───────────────▶  DataReceiver (500 FPS)
TCP cmd    :9001  ◀──────────────▶  Command channel
OTA server :9002  ◀──────────────   OTA firmware updater
```

- **Data**: UDP with 5× staggered redundancy (S=16 spacing). Each frame
  is sent 5 times across different packets. Typical loss: <0.01%.
- **Commands**: TCP for reliable bidirectional control.
- **OTA**: Firmware updates over WiFi without USB.

---

## 9. Long-Run Experiments

The system is designed for multi-day unattended operation:

- **Watchdog timer** (8s): board resets automatically if firmware hangs
- **WiFi reconnection**: automatic, with UDP beacon for discovery
- **Ring buffer**: 1536 frames (3.1s at 500 FPS) buffered on Pico
- **UDP redundancy**: 5× staggered copies eliminate most packet loss
- **Auto-recording**: starts on connect, 60s grace period on disconnect
- **HDF5 flush**: every 100 frames (~200ms) to minimize crash data loss
- **Recovery files**: 20 min rolling buffer always active on host

Recommended setup:
- Flash firmware via OTA or USB BOOTSEL
- Run `launch_gui.bat` on the host PC
- Enable webcam capture to photograph the preparation periodically
- Data auto-records to the configured data directory

---

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Data dot stays grey | Pico not connected or wrong HOST_IP | Check `config.h`, verify WiFi |
| FPS shown as 0 | No frames received | Check firewall — allow Python on UDP port 9004 |
| CRC errors > 0 | WiFi interference | Move AP closer, reduce channel congestion |
| Missed frames > 0 | Brief WiFi bursts | Normal at <0.01%; check WiFi signal if higher |
| All channels read 0x800000 | ADC DRDY timeout | Check CLK_EN (GP22) and SPI wiring |
| GUI won't start | Missing packages | Run `install.bat` or `pip install -r host/requirements.txt` |
| Build fails | Missing toolchain | Install CMake, Ninja, ARM GCC; set PICO_SDK_PATH |
| OTA fails | Pico not reachable | Verify Pico IP, ensure UDP port 9003 beacon visible |

---

## 11. Project Structure

```
MushIO/
├── firmware_c/           C firmware for Pico 2W (Pico SDK)
│   ├── config.h          WiFi credentials, network ports, ADC pins
│   ├── main.c            Main firmware (WiFi, UDP streaming, CMD server)
│   ├── CMakeLists.txt    Build configuration
│   └── BUILD.md          Firmware build instructions
├── host/                 Python GUI and utilities
│   ├── gui.py            Main GUI application (tkinter + matplotlib)
│   ├── receiver.py       Frame parser and protocol constants
│   ├── ota_client.py     Over-the-air firmware updater
│   ├── analyze.py        Post-hoc data analysis tools
│   └── requirements.txt  Python dependencies
├── HW/                   Hardware design files (schematics, PCB)
├── archive/              Legacy MicroPython firmware (historical)
├── install.bat / .sh     Dependency installer
├── launch_gui.bat        GUI launcher with dependency check
├── build_and_launch.bat  Full build → flash → launch pipeline
├── GETTING_STARTED.md    This file
└── SPEC.md               Hardware specification
```
