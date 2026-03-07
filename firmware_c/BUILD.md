# Building the MushIO C Firmware

## Requirements

| Tool | Version |
|------|---------|
| Pico SDK | >= 2.0 (`pico2_w` board support) |
| arm-none-eabi-gcc | >= 12 |
| CMake | >= 3.13 |
| Ninja or Make | any recent |

## Environment

Set `PICO_SDK_PATH` to your SDK clone:

```cmd
set PICO_SDK_PATH=C:\path\to\pico-sdk
```

## Build

```cmd
cd MushIV1p0\firmware_c
mkdir build && cd build
cmake .. -G "Ninja" -DPICO_BOARD=pico2_w
ninja
```

or with Make:

```cmd
cmake .. -DPICO_BOARD=pico2_w
make -j4
```

Output: `build/mushio_c.uf2`

## Flash

1. Hold BOOTSEL on the Pico 2 W, plug USB — it mounts as a drive.
2. Copy the UF2:
   ```cmd
   copy build\mushio_c.uf2 D:\
   ```
3. The Pico reboots automatically.

## Monitor (USB serial)

```cmd
python -m serial.tools.miniterm - 115200
```
or use the Pico SDK's `picotool`:
```cmd
picotool console
```

## Architecture

```
Core 0 (main.c)                         Core 1 (core1.c)
────────────────────────────────────    ──────────────────────────────
cyw43_arch_init()                       demo_adc_init()  (sine table)
WiFi connect                            ┌─────────────────────────────
TCP connect → HOST_IP:9000              │ demo_adc_scan()   →  216 B
CMD server listen → port 9001           │ frame_build()     →  228 B
                                        │ ring_push(&g_ring, frame)
Main loop:                              │ busy_wait_until(next_scan)
  do_stream_work()                      └─────────────────────────────
    ring_pop → batch_buf
    tcp_write(batch) every 40 frames
  reconnect if TCP drops
  stats every 10 s
  LED heartbeat every 2 s
```

Ring buffer: lockless SPSC, 128 slots × 228 B = 29 KB shared SRAM.
Core 1 owns `head`; Core 0 owns `tail`; `__dmb()` barriers ensure
visibility across cores without any spinlock.

## Expected output

```
============================================================
  MushIO V1.0  |  Pico Demo Mode  (C firmware)
============================================================

[INIT] Launching Core 1 (ADC scan loop)...
[C1] Core 1 started — ADC scan loop
[C1] Demo ADC ready.  Starting scan loop at ~200 FPS.
[INIT] Core 1 running.
[INIT] Initialising WiFi...
[WIFI] Connecting to 'Occam's Router' ...
[WIFI] Connected.  IP: 192.168.68.xxx
[INIT] Connecting data socket to 192.168.68.115:9000 ...
[DATA] Connected to 192.168.68.115:9000
[CMD] Listening on port 9001

[RUN] Streaming.  Channels: 72  Target: ~200 FPS
[RUN] Data  → 192.168.68.115:9000
[RUN] CMD   → port 9001

[STAT] Sent: 2000 (+2000)  FPS: 200.0  Buffered: 0  Dropped: 0  TCP: 1
```

## Porting to real ADC hardware

Replace `demo_adc.c` / `demo_adc.h` with a real `adc_manager.c` that:
- Initialises SPI0 and SPI1 buses
- Resets all 6 ADS124S08 chips via ADC_RESET_N_PIN
- Enables external clock via ADC_CLK_EN_PIN
- Runs `scan_frame()` which reads 12 channels per ADC sequentially
  and fills a DATA_SIZE byte buffer in big-endian 24-bit format

`core1.c` calls `demo_adc_init()` and `demo_adc_scan()` — rename those
to `adc_manager_init()` and `adc_manager_scan()` and update the includes.
