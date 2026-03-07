# MushIO V1.0 — Technical Specification

---

## 1. System Overview

MushIO V1.0 is a wireless 72-channel electrophysiology recording system for long-term
measurement of electrical potentials in mushroom mycelium networks.

**Key parameters:**

| Parameter | Value |
|-----------|-------|
| Recording channels | 64 (ELEC00–ELEC77, 8×8 grid) |
| Stimulation monitor channels | 8 (STIM_0–STIM_7) |
| ADC resolution | 24-bit signed |
| ADC data rate | 4000 SPS per channel (configurable) |
| Effective frame rate | ~170–200 frames/sec (all 72 ch) |
| Input voltage range | ±231 mV at AFE input (after ÷11 gain) |
| Input noise floor | ~1 µV RMS (limited by AFE) |
| Reference voltage | 5.08 V (AVDD\_2V5 − AVSS\_N2V5) |
| AFE gain | 11× (G = 1 + 5.1 kΩ / 510 Ω) |
| Communication | WiFi 802.11n 2.4 GHz → TCP/IP |
| Power | LiPo battery + LM27762 charge pump |

---

## 2. Hardware

### 2.1 Microcontroller

**Raspberry Pi Pico 2W** (RP2350, dual Cortex-M33 @ 150 MHz, 520 KB SRAM)

- Core 0: main acquisition loop (ADC scan + TCP stream)
- Core 1: TCP command server (diagnostic commands from GUI)
- Watchdog timer: 8 s timeout, resets on hang

### 2.2 ADCs

**Texas Instruments ADS124S08** × 6

| Property | Value |
|----------|-------|
| Resolution | 24-bit |
| Max data rate | 4000 SPS |
| Input channels | 12 per chip (AIN0–AIN11) |
| Interface | SPI Mode 1 (CPOL=0, CPHA=1) |
| Reference | External: REFP0/REFN0 = AVDD\_2V5/AVSS\_N2V5 |
| Clock | External 4.096 MHz oscillator (GP22/CLK\_EN) |
| Filter | sinc3 (continuous conversion mode) |

ADC configuration (set at startup):

| Register | Setting |
|----------|---------|
| DATARATE [3:0] | 0x0D = 4000 SPS |
| DATARATE [6] CLK | 1 = external clock |
| PGA GAIN | 0 = ×1 (bypass; AFE provides gain) |
| PGA\_EN | 0 = PGA bypassed |
| REFSEL | 0 = REFP0/REFN0 |
| SYS TIMEOUT | 1 = SPI timeout enabled |

### 2.3 Analog Front End

**Texas Instruments TLV4387** instrumentation amplifier (×6 AFE blocks)

- Gain: G = 1 + R\_f / R\_g = 1 + 5.1 kΩ / 510 Ω = **11**
- Each AFE block drives one ADC chip
- 64 recording inputs + 8 stimulation monitor inputs

### 2.4 Power Supply

- **LM27762**: dual-output charge pump
  - AVDD\_2V5 = +2.52 V
  - AVSS\_N2V5 = −2.56 V
  - Reference span: VREF = 2.52 − (−2.56) = **5.08 V**
- Power-good signal on GP28 (ADC2 analog input)

### 2.5 Ancillary Sensors (I2C0, GP0/GP1)

| Device | Address | Function |
|--------|---------|----------|
| P3T1755DPZ | 0x48 | Temperature sensor |
| LIS2DH12TR | 0x18 | 3-axis accelerometer / vibration |

---

## 3. Pin Assignments

### SPI Buses

| Signal | GPIO | Connected to |
|--------|------|-------------|
| SPI0\_SCK | GP2 | ADC0, ADC1, ADC2 (via 22 Ω) |
| SPI0\_MOSI | GP3 | ADC0, ADC1, ADC2 DIN |
| SPI0\_MISO | GP4 | ADC0, ADC1, ADC2 DOUT |
| SPI1\_SCK | GP14 | ADC3, ADC4, ADC5 (via 22 Ω) |
| SPI1\_MOSI | GP15 | ADC3, ADC4, ADC5 DIN |
| SPI1\_MISO | GP12 | ADC3, ADC4, ADC5 DOUT |

### Chip Select (active low, 22 Ω series)

| ADC | CS GPIO |
|-----|---------|
| ADC0 | GP5 |
| ADC1 | GP7 |
| ADC2 | GP9 |
| ADC3 | GP11 |
| ADC4 | GP13 |
| ADC5 | GP17 |

### Data Ready (active low, direct)

| ADC | DRDY GPIO |
|-----|----------|
| ADC0 | GP6 |
| ADC1 | GP8 |
| ADC2 | GP10 |
| ADC3 | GP16 |
| ADC4 | GP18 |
| ADC5 | GP19 |

### Shared Control

| Signal | GPIO | Function |
|--------|------|----------|
| ADC\_START | GP21 | Shared START/SYNC pulse (all ADCs) |
| ADC\_RESET\_N | GP20 | Active-low hardware reset (all ADCs) |
| ADC\_CLK\_EN | GP22 | External clock output enable |
| ANA\_PGOOD | GP28 | Analog power-good sense (ADC input) |

---

## 4. Channel Map

72 total channels across 6 ADCs × 12 AIN inputs each.

### ADC0 (SPI0) — 12 recording channels

| AIN | Electrode |
|-----|----------|
| 0 | ELEC02 |
| 1 | ELEC23 |
| 2 | ELEC22 |
| 3 | ELEC12 |
| 4 | ELEC11 |
| 5 | ELEC21 |
| 6 | ELEC20 |
| 7 | ELEC01 |
| 8 | ELEC00 |
| 9 | ELEC10 |
| 10 | ELEC03 |
| 11 | ELEC13 |

### ADC1 (SPI0) — 12 recording channels

| AIN | Electrode |
|-----|----------|
| 0 | ELEC26 |
| 1 | ELEC16 |
| 2 | ELEC25 |
| 3 | ELEC15 |
| 4 | ELEC24 |
| 5 | ELEC05 |
| 6 | ELEC04 |
| 7 | ELEC14 |
| 8 | ELEC17 |
| 9 | ELEC07 |
| 10 | ELEC06 |
| 11 | ELEC27 |

### ADC2 (SPI0) — 8 recording + 4 STIM monitor

| AIN | Channel |
|-----|---------|
| 0 | ELEC41 |
| 1 | ELEC31 |
| 2 | ELEC30 |
| 3 | ELEC40 |
| 4 | ELEC43 |
| 5 | ELEC33 |
| 6 | ELEC32 |
| 7 | ELEC42 |
| 8 | **STIM\_1** |
| 9 | **STIM\_0** |
| 10 | **STIM\_7** |
| 11 | **STIM\_6** |

### ADC3 (SPI1) — 8 recording + 4 STIM monitor

| AIN | Channel |
|-----|---------|
| 0 | ELEC45 |
| 1 | ELEC35 |
| 2 | ELEC34 |
| 3 | ELEC44 |
| 4 | ELEC47 |
| 5 | ELEC37 |
| 6 | ELEC36 |
| 7 | ELEC46 |
| 8 | **STIM\_3** |
| 9 | **STIM\_2** |
| 10 | **STIM\_5** |
| 11 | **STIM\_4** |

### ADC4 (SPI1) — 12 recording channels

| AIN | Electrode |
|-----|----------|
| 0 | ELEC71 |
| 1 | ELEC50 |
| 2 | ELEC60 |
| 3 | ELEC70 |
| 4 | ELEC62 |
| 5 | ELEC52 |
| 6 | ELEC51 |
| 7 | ELEC61 |
| 8 | ELEC73 |
| 9 | ELEC63 |
| 10 | ELEC53 |
| 11 | ELEC72 |

### ADC5 (SPI1) — 12 recording channels

| AIN | Electrode |
|-----|----------|
| 0 | ELEC75 |
| 1 | ELEC54 |
| 2 | ELEC64 |
| 3 | ELEC74 |
| 4 | ELEC66 |
| 5 | ELEC56 |
| 6 | ELEC55 |
| 7 | ELEC65 |
| 8 | ELEC67 |
| 9 | ELEC77 |
| 10 | ELEC76 |
| 11 | ELEC57 |

> **Note:** The channel map above is a best-effort extraction from the schematic PDF.
> Verify AIN-to-electrode routing against the physical schematic before interpreting data.

---

## 5. Acquisition Timing

### Scan sequence (one frame = 12 mux steps × 6 ADCs)

```
For each AIN channel 0..11:
  1. Write INPMUX register on all 6 ADCs  (~18 µs × 6 = 108 µs total)
  2. Pulse START pin high → low            (~2 µs)
  3. Wait for DRDY low on all 6 ADCs      (~250 µs at 4000 SPS)
  4. Send RDATA + read 3 bytes per ADC    (~32 µs × 6 = 192 µs)

Total per channel step: ~450–500 µs
Total per frame (72 ch): ~5.4–6.0 ms  →  167–185 FPS
```

At 4000 SPS with sinc3 filter, each ADC conversion takes exactly 250 µs.
All 6 ADCs start simultaneously via the shared START pin and complete together.

### MUX settling

The sinc3 filter requires 3 conversion periods to settle after a MUX change.
`MUX_SETTLE_CONVERSIONS = 1` trades a small accuracy reduction for maximum speed.
Set to 3 in `config.py` for highest accuracy at lower throughput.

---

## 6. Wire Protocol

### TCP connections

| Port | Direction | Purpose |
|------|-----------|---------|
| 9000 | Host listens, Pico connects | Binary ADC data stream |
| 9001 | Pico listens, Host connects | Text command/response |

### 6.1 Data Frame Format (port 9000)

Binary, 228 bytes per frame:

```
Offset  Size  Type        Field
──────────────────────────────────────────────────
  0      2    uint16 LE   Sync word: 0xAA55
  2      4    uint32 LE   Timestamp (ms since boot)
  6      2    uint16 LE   Sequence number (wraps at 65535)
  8      1    uint8       ADC count = 6
  9      1    uint8       Channels per ADC = 12
 10    216    bytes       ADC data: 72 × 3 bytes (see below)
226      2    uint16 LE   CRC-16/CCITT-FALSE over bytes 0–225
──────────────────────────────────────────────────
Total: 228 bytes
```

**ADC data layout (bytes 10–225):**
- 72 samples in frame order: ADC0\_CH0, ADC0\_CH1, …, ADC0\_CH11, ADC1\_CH0, …, ADC5\_CH11
- Each sample: 3 bytes, big-endian, signed 24-bit two's complement
- Error marker: 0x800000 (= −8388608) indicates DRDY timeout on that channel

**Voltage conversion:**
```
V_adc   = (raw_count / 2^23) × VREF        where VREF = 5.08 V
V_input = V_adc / AFE_GAIN                  where AFE_GAIN = 11
```

For recording channels (µV): `V_input × 1e6`
For STIM channels (mV): `V_input × 1e3`

**CRC-16/CCITT-FALSE:**
- Polynomial: 0x1021
- Initial value: 0xFFFF
- No input/output reflection
- Known vector: CRC("123456789") = 0x29B1

### 6.2 Command Protocol (port 9001)

Text-based, newline-delimited:

- Host sends: `<command>\n`
- Pico responds: zero or more lines, terminated by `END\r\n`

Available commands:

| Command | Arguments | Description |
|---------|-----------|-------------|
| `ping` | — | Returns `PONG` |
| `status` | — | Streaming statistics |
| `help` | — | List all commands |
| `scan_single` | `<adc> <ain> <n>` | Read N samples from one channel |
| `check_adcs` | — | Verify all 6 ADC device IDs |
| `scan_all` | — | Print all 72 channel values |
| `drdy` | `<adc> <ain>` | Measure DRDY timing (20 samples) |
| `dump_regs` | `<adc>` | Dump all 18 ADS124S08 registers |
| `set_datarate` | `<0x00–0x0D>` | Change ADC data rate on all ADCs |
| `benchmark` | — | Measure maximum scan_frame() rate |

---

## 7. Software Architecture

### Firmware (`firmware/`)

```
main.py          Entry point. Init sequence, WDT, main acquisition loop.
config.py        All constants: pin map, ADC settings, WiFi, channel map.
ads124s08.py     SPI driver for one ADS124S08 chip.
adc_manager.py   Manages 6 ADCs. Provides scan_frame() → bytearray(216).
streamer.py      WiFi TCP client. Frame builder, ring buffer, reconnection.
cmd_server.py    TCP command server on port 9001. Runs on core 1.
bringup.py       Interactive diagnostic tool (REPL use during hardware bring-up).
```

**Main loop (core 0):**
```
while True:
    wdt.feed()
    cmd_server.check_pause()          # yield to core 1 if diagnostic command active
    frame = adc_mgr.scan_frame()      # ~5 ms: scan all 72 channels
    packet = streamer.build_frame(ts, seq, frame)
    streamer.send_frame(packet)       # TCP send or ring-buffer if disconnected
    # every 10s: print stats, attempt reconnect if needed
```

**Resilience:**
- Watchdog timer (8 s) resets board if main loop stalls
- TCP reconnect attempt every 5 s when disconnected
- 64-frame ring buffer absorbs brief WiFi outages
- Per-channel DRDY timeout fills error marker (0x800000) and continues

### Host Software (`host/`)

```
receiver.py    TCP server, frame parser, CRC validator, dual CSV logger.
gui.py         Real-time GUI: waveform display, command terminal, webcam capture.
```

**GUI threads:**
| Thread | Purpose |
|--------|---------|
| Main (Tk) | UI rendering, 25 Hz waveform refresh |
| DataReceiver | TCP server, frame ingestion into ring buffers |
| CommandClient | Persistent TCP connection to Pico port 9001 |
| DemoDataInjector | (demo mode) Injects simulated data at 200 fps |
| DemoCmdClient | (demo mode) Responds to commands with fake data |
| WebcamCapture | Periodic JPEG capture from local camera |

### Test Utilities (`test/`)

```
simulator.py       Simulates Pico data stream over TCP (no hardware needed).
test_protocol.py   42 unit tests: CRC, frame format, sign extension, voltage conversion.
```

Run tests:
```bash
python test/test_protocol.py
```

---

## 8. File Outputs

| Path | Created by | Contents |
|------|-----------|----------|
| `data/mushio_elec_*.csv` | receiver.py / gui.py | 64 recording channels, raw ADC counts |
| `data/mushio_stim_*.csv` | receiver.py / gui.py | 8 STIM monitor channels, raw ADC counts |
| `data/mushio_*.bin` | receiver.py `--raw` | Raw binary frames (228 bytes each) |
| `captures/mushio_capture_*.jpg` | gui.py webcam | Timestamped webcam photos |

CSV column order:
```
timestamp_ms, seq, <channel_name_1>, <channel_name_2>, ...
```

---

## 9. Configuration Reference (`firmware/config.py`)

| Constant | Default | Description |
|----------|---------|-------------|
| `WIFI_SSID` | `"YOUR_SSID"` | WiFi network name |
| `WIFI_PASSWORD` | `"YOUR_PASSWORD"` | WiFi password |
| `WIFI_COUNTRY` | `"US"` | Regulatory domain |
| `HOST_IP` | `"192.168.1.100"` | Host PC IP address |
| `HOST_PORT` | `9000` | Data stream TCP port |
| `ADC_DATARATE` | `0x0D` | 4000 SPS |
| `ADC_PGA_GAIN` | `0` | ×1 (PGA bypassed) |
| `ADC_PGA_EN` | `0` | PGA disabled |
| `VREF` | `5.08` | Reference voltage (V) |
| `AFE_GAIN` | `11.0` | Analog front-end gain |
| `AINCOM` | `0x0C` | Single-ended negative input |
| `DRDY_TIMEOUT_MS` | `10` | Per-channel conversion timeout |
| `MUX_SETTLE_CONVERSIONS` | `1` | Conversions to discard after MUX change |
| `STREAM_BUFFER_SIZE` | `64` | Ring buffer depth (frames) |
| `RECONNECT_INTERVAL_S` | `5` | WiFi/TCP reconnect interval |
| `SPI_BAUDRATE` | `1_000_000` | SPI clock speed (1 MHz) |

---

## 10. Known Limitations / Future Work

- **Channel map unverified**: ELEC/STIM assignments are extracted from schematic PDF and must be confirmed against physical board routing before interpreting spatial data.
- **Single-ended only**: All inputs measured vs. AINCOM (AGND). Differential recording not implemented.
- **No hardware timestamping**: Timestamp is from Pico's `time.ticks_ms()`, subject to ±1 ms jitter and potential drift over multi-day runs. For precise timing, correlate with an external reference.
- **WiFi 2.4 GHz only**: Pico 2W does not support 5 GHz.
- **SPI speed**: Currently 1 MHz (conservative). The ADS124S08 supports up to 4 MHz — increasing `SPI_BAUDRATE` to `4_000_000` may improve throughput if wiring is short.
- **Temperature / IMU not used in firmware**: Sensors are populated on board but not read in `main.py`. `bringup.py` provides a hook for manual reads.
