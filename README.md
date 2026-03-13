# MushIO V1.0

72-channel electrophysiology recorder for mushroom mycelium studies.

**Pico 2W** streams 500 FPS over WiFi → **Python GUI** displays, records, and analyzes data in real time.

## Quick Start

```bash
# 1. Install Python dependencies
pip install -r host/requirements.txt

# 2. Try the GUI in demo mode (no hardware needed)
python host/gui.py --demo

# 3. Or on Windows, double-click:
launch_gui_demo.bat
```

## With Hardware

1. Edit `firmware_c/config.h` with your WiFi credentials and host PC IP
2. Build and flash firmware (see [GETTING_STARTED.md](GETTING_STARTED.md))
3. Run `python host/gui.py` — data streams automatically when the Pico boots

## Documentation

- **[GETTING_STARTED.md](GETTING_STARTED.md)** — Full setup guide, GUI reference, data format
- **[SPEC.md](SPEC.md)** — Hardware specification
- **[firmware_c/BUILD.md](firmware_c/BUILD.md)** — Firmware build instructions

## Architecture

| Component | Technology | Purpose |
|-----------|-----------|---------|
| Firmware | C (Pico SDK) | ADC sampling, WiFi streaming, OTA updates |
| Data transport | UDP + 5× redundancy | <0.01% frame loss over WiFi |
| GUI | Python (tkinter + matplotlib) | Real-time 8×8 grid, waveforms, recording |
| Data format | HDF5 | Compressed, self-describing, with gap markers |
