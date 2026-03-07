# MushIO V1.0 — Getting Started

72-channel electrophysiology recorder for mushroom mycelium studies.

---

## 1. Prerequisites

### Host PC
- Python 3.8 or later
- Packages: `pip install matplotlib numpy opencv-python`
- Windows 10/11 (batch launchers), macOS/Linux (run `python host/gui.py` directly)

### Hardware
- MushIO V1.0 board (Raspberry Pi Pico 2W + ADS124S08 x6)
- USB cable (for first flash)
- Same WiFi network as the host PC

---

## 2. Try the GUI in Demo Mode (no hardware needed)

Double-click **`launch_gui_demo.bat`** — or from a terminal:

```bat
python host\gui.py --demo
```

This streams 200 fps of simulated electrophysiology data so you can explore all UI controls before hardware arrives.

To exit demo mode, click the green **DEMO MODE ON** button.

---

## 3. Flash Firmware to the Pico

### 3a. Install MicroPython on the Pico 2W

1. Hold the **BOOTSEL** button while plugging in USB.
2. A drive called `RPI-RP2` appears — drag the MicroPython `.uf2` file onto it.
   - Download: https://micropython.org/download/rp2350/

### 3b. Copy firmware files

Copy every file from `firmware/` to the Pico's root filesystem:

```
ads124s08.py
adc_manager.py
streamer.py
cmd_server.py
bringup.py
config.py
main.py
```

Tools: [Thonny IDE](https://thonny.org) (beginner-friendly) or `mpremote`:
```bash
pip install mpremote
mpremote connect auto cp firmware/*.py :
```

---

## 4. Configure Credentials

Edit `firmware/config.py` on the Pico:

```python
WIFI_SSID     = "YourNetworkName"
WIFI_PASSWORD = "YourPassword"
HOST_IP       = "192.168.1.X"   # your PC's IP — run `ipconfig` to find it
```

> **Tip:** On Windows, open a terminal and run `ipconfig`. Look for the IPv4 address
> on your WiFi adapter (e.g. `192.168.1.42`).

---

## 5. Run Board Bringup

With the Pico connected via USB, open a REPL (Thonny or `mpremote repl`) and run:

```python
import bringup
bringup.run_all()
```

This checks each subsystem in order:

| Check | What it verifies |
|-------|-----------------|
| Analog power good | Charge pump rails healthy |
| Clock enable | 4.096 MHz external clock toggling |
| ADC IDs | All 6 ADS124S08 respond on SPI |
| Single channel read | One electrode reads a plausible value |
| DRDY timing | Conversion latency matches configured data rate |
| Full 72-ch scan | All channels return data (no timeouts) |
| Frame rate | scan_frame() achieves ≥ 100 FPS |
| WiFi | Pico connects to your AP |
| TCP | Pico can reach HOST_IP:9000 |

Fix any failures before proceeding. The most common issues:

- **ADC ID FAIL** → check SPI wiring and CS pin assignments in `config.py`
- **DRDY never asserts** → check CLK_EN (GP22) and external 4.096 MHz oscillator
- **WiFi won't connect** → verify SSID/password; Pico 2W supports 2.4 GHz only

---

## 6. Start Recording

### 6a. On the Host PC — start the GUI

Double-click **`launch_gui.bat`**, or:

```bash
python host\gui.py
```

Optional arguments:
```
--data-port 9000      TCP port for Pico data stream (default 9000)
--cmd-host  <IP>      Pico's IP for command channel
--cmd-port  9001      Pico command server port (default 9001)
--demo                Start in demo mode (no hardware)
```

### 6b. On the Pico — run the firmware

In Thonny, press **Run** (`main.py`), or reset the Pico (it auto-runs `main.py` on boot).

The Pico will:
1. Initialize all 6 ADCs
2. Connect to WiFi
3. Connect to the host GUI on port 9000
4. Start streaming ~200 frames/sec

The GUI's **Data stream** dot turns green when the Pico connects.

### 6c. Connect the command channel (optional)

Enter the Pico's IP in the **Command → Pico IP** field and click **Connect**.
The command dot turns green. You can now send diagnostic commands from the terminal.

---

## 7. GUI Quick Reference

### Waveform panel

| Control | Function |
|---------|----------|
| Time window | Scroll history: 1–20 seconds visible |
| Y-scale | Manual voltage range (disabled when Auto-scale is on) |
| Auto-scale Y | Fit waveform to window automatically |
| Channel listbox | Select up to 8 channels to display simultaneously |
| 8 Elec / All STIM / Clear | Quick-select presets |
| STIM_0…STIM_7 buttons | Toggle individual STIM channels in/out of view |

### Command terminal

Type commands and press **Enter** or click **Send**. Up/Down arrows scroll history.

Common commands:
```
ping          — check Pico is alive
status        — streaming stats (fps, dropped frames, WiFi/TCP state)
check_adcs    — verify all 6 ADC device IDs
scan_all      — print current value of all 72 channels
scan_single 0 0 10   — read ADC0 AIN0, 10 samples
benchmark     — measure maximum scan_frame() rate
set_datarate 0x0D    — set 4000 SPS on all ADCs
dump_regs 0   — print all registers on ADC0
```

### Webcam capture (left panel, bottom)

1. Tick **Enable**
2. Select camera index (0 = default webcam)
3. Choose capture interval (1–60 minutes)
4. Images saved to `captures/mushio_capture_YYYYMMDD_HHMMSS.jpg`
5. Click **Capture Now** for an immediate shot

---

## 8. Data Output

CSV files are written to `data/` each session:

| File | Contents |
|------|----------|
| `mushio_elec_YYYYMMDD_HHMMSS.csv` | 64 recording electrode channels (raw ADC counts) |
| `mushio_stim_YYYYMMDD_HHMMSS.csv` | 8 stimulation monitor channels |

Columns: `timestamp_ms`, `seq`, then one column per channel named by electrode (e.g. `ELEC02`, `STIM_0`).

To also log raw binary frames (for replay/offline analysis):
```bash
python host\receiver.py --raw
```

---

## 9. Long-Run Experiments

The firmware is designed for multi-day unattended operation:
- **Watchdog timer** (8 s): board resets automatically if firmware hangs
- **WiFi reconnection**: retries every 5 s if connection drops
- **Ring buffer**: 64 frames buffered during brief WiFi outages
- **CSV auto-flush**: every 5 s to minimize data loss on crash

Recommended setup:
- Set the Pico to auto-run `main.py` on boot (default behaviour)
- Run `launch_gui.bat` on the host PC at the start of each session
- Enable webcam capture to photograph the preparation periodically

---

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Data dot stays grey | Pico not connected or wrong HOST_IP | Check `config.py`, rerun firmware |
| FPS shown as 0 | No frames received | Check firewall — allow Python on port 9000 |
| CRC errors > 0 | WiFi interference | Move AP closer, reduce channel congestion |
| Missed frames > 0 | Host CPU overloaded | Close other apps; lower display FPS in `gui.py` |
| All channels read 0x800000 | ADC DRDY timeout | Check CLK_EN (GP22) and SPI wiring |
| GUI won't start | Missing packages | Run `pip install matplotlib numpy opencv-python` |
| Webcam error | Wrong camera index | Try Camera 1 or 2 in the Webcam section |
