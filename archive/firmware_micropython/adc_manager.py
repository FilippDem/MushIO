"""
Multi-ADC Manager for MushIO V1.0

Orchestrates 6 ADS124S08 ADCs across 2 SPI buses.
Handles initialization, synchronized scanning, and frame assembly.
"""

import time
from machine import Pin, SPI
from ads124s08 import ADS124S08, AINCOM, ERROR_MARKER, DR_4000
import config


class ADCManager:
    """Manages 6 ADS124S08 ADCs for synchronized multi-channel scanning."""

    def __init__(self):
        self.adcs = []
        self.spi0 = None
        self.spi1 = None
        self.start_pin = None
        self.reset_pin = None
        self.clk_en_pin = None
        self._frame_buf = bytearray(config.TOTAL_CHANNELS * 3)  # 72 * 3 = 216 bytes

    def init(self):
        """
        Full hardware initialization sequence.
        Call once at startup. Raises RuntimeError if any ADC fails ID check.
        """
        print("[ADC] Initializing hardware...")

        # Set up shared control pins
        self.start_pin = Pin(config.ADC_START_PIN, Pin.OUT, value=0)
        self.reset_pin = Pin(config.ADC_RESET_N_PIN, Pin.OUT, value=1)
        self.clk_en_pin = Pin(config.ADC_CLK_EN_PIN, Pin.OUT, value=0)

        # Hardware reset all ADCs (active low)
        print("[ADC] Hardware reset...")
        self.reset_pin.value(0)
        time.sleep_ms(10)
        self.reset_pin.value(1)
        time.sleep_ms(10)

        # Enable external clock
        print("[ADC] Enabling external 4.096 MHz clock...")
        self.clk_en_pin.value(1)
        time.sleep_ms(5)

        # Initialize SPI buses
        self.spi0 = SPI(
            0,
            baudrate=config.SPI_BAUDRATE,
            polarity=config.SPI_POLARITY,
            phase=config.SPI_PHASE,
            bits=config.SPI_BITS,
            sck=Pin(config.SPI0_SCK),
            mosi=Pin(config.SPI0_MOSI),
            miso=Pin(config.SPI0_MISO),
        )

        self.spi1 = SPI(
            1,
            baudrate=config.SPI_BAUDRATE,
            polarity=config.SPI_POLARITY,
            phase=config.SPI_PHASE,
            bits=config.SPI_BITS,
            sck=Pin(config.SPI1_SCK),
            mosi=Pin(config.SPI1_MOSI),
            miso=Pin(config.SPI1_MISO),
        )

        # Create ADC instances
        self.adcs = []
        for i in range(config.NUM_ADCS):
            if i in config.SPI0_ADC_INDICES:
                spi = self.spi0
            else:
                spi = self.spi1

            adc = ADS124S08(
                spi=spi,
                cs_pin_num=config.ADC_CS_PINS[i],
                drdy_pin_num=config.ADC_DRDY_PINS[i],
                name=f"ADC{i}",
            )
            self.adcs.append(adc)

        # Software reset each ADC
        print("[ADC] Software reset all ADCs...")
        for adc in self.adcs:
            adc.reset()

        # Verify device IDs
        print("[ADC] Verifying device IDs...")
        all_ok = True
        for i, adc in enumerate(self.adcs):
            id_byte, valid = adc.read_id()
            status = "OK" if valid else "FAIL"
            print(f"  ADC{i}: ID=0x{id_byte:02X} [{status}]")
            if not valid:
                all_ok = False

        if not all_ok:
            raise RuntimeError("One or more ADCs failed ID verification!")

        # Configure all ADCs
        print("[ADC] Configuring ADCs...")
        for adc in self.adcs:
            adc.configure(
                datarate=config.ADC_DATARATE,
                pga_gain=config.ADC_PGA_GAIN,
                pga_enable=config.ADC_PGA_EN,
                ref_sel=0,    # REFP0/REFN0
                clk_sel=1,    # External clock
            )

        # Self-offset calibration: shorts ADC inputs internally and measures
        # the offset, storing it in OFCAL registers to correct DC error.
        # Must run after configure() with the gain settings used for recording.
        print("[ADC] Running self-offset calibration (SFOCAL)...")
        all_cal_ok = True
        for i, adc in enumerate(self.adcs):
            ok = adc.self_offset_calibrate(timeout_ms=200)
            status = "OK" if ok else "TIMEOUT"
            print(f"  ADC{i}: SFOCAL {status}")
            if not ok:
                all_cal_ok = False
        if not all_cal_ok:
            print("[ADC] WARNING: SFOCAL timed out on one or more ADCs. "
                  "DC offset may be uncorrected.")

        print(f"[ADC] Initialization complete. {config.NUM_ADCS} ADCs ready.")

    def _pulse_start(self):
        """Pulse the shared START pin to trigger conversion on all ADCs."""
        self.start_pin.value(1)
        # ADS124S08 needs START high for at least 2 tCLK = ~0.5us at 4.096MHz
        # MicroPython GPIO toggle is ~1-5us, so no explicit delay needed
        self.start_pin.value(0)

    def scan_frame(self):
        """
        Perform a complete scan of all channels on all ADCs.

        Scans through 12 AIN channels sequentially. For each channel:
        1. Set INPMUX on all 6 ADCs
        2. Trigger simultaneous conversion via shared START pin
        3. Wait for all DRDYs
        4. Read data from all 6 ADCs

        Returns:
            bytearray of 216 bytes (72 channels * 3 bytes each, big-endian 24-bit)
            Channel order: ADC0_CH0, ADC0_CH1, ..., ADC0_CH11,
                          ADC1_CH0, ..., ADC5_CH11
        """
        buf = self._frame_buf

        for ch in range(config.CHANNELS_PER_ADC):
            # Step 1: Set MUX on all ADCs to this channel
            ain = config.AIN_CHANNELS[ch]
            for adc in self.adcs:
                adc.set_mux(ain, AINCOM)

            # Step 2: Pulse START to begin conversion on all ADCs simultaneously
            self._pulse_start()

            # Step 3: Wait for all ADCs to complete
            # Poll DRDY pins with overall timeout
            deadline = time.ticks_add(time.ticks_ms(), config.DRDY_TIMEOUT_MS)
            ready = [False] * config.NUM_ADCS

            while not all(ready):
                for i in range(config.NUM_ADCS):
                    if not ready[i] and self.adcs[i].is_drdy():
                        ready[i] = True
                if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                    break

            # Step 4: Read data from all ADCs
            for i in range(config.NUM_ADCS):
                # Calculate buffer offset: adc_index * 12 channels * 3 bytes + ch * 3
                offset = (i * config.CHANNELS_PER_ADC + ch) * 3

                if ready[i]:
                    self.adcs[i].read_data_into(buf, offset)
                else:
                    # Timeout - fill with error marker (0x80, 0x00, 0x00)
                    buf[offset] = 0x80
                    buf[offset + 1] = 0x00
                    buf[offset + 2] = 0x00

        return buf

    def scan_frame_fast(self):
        """
        Faster scan that interleaves SPI0 and SPI1 bus reads.

        Same result as scan_frame() but reads from SPI0 ADCs first,
        then SPI1 ADCs, to avoid bus contention.
        """
        buf = self._frame_buf
        spi0_adcs = config.SPI0_ADC_INDICES
        spi1_adcs = config.SPI1_ADC_INDICES

        for ch in range(config.CHANNELS_PER_ADC):
            ain = config.AIN_CHANNELS[ch]

            # Set MUX on all ADCs
            for adc in self.adcs:
                adc.set_mux(ain, AINCOM)

            # Start all conversions
            self._pulse_start()

            # Wait for DRDY with timeout
            deadline = time.ticks_add(time.ticks_ms(), config.DRDY_TIMEOUT_MS)
            ready = [False] * config.NUM_ADCS

            while not all(ready):
                for i in range(config.NUM_ADCS):
                    if not ready[i] and self.adcs[i].is_drdy():
                        ready[i] = True
                if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                    break

            # Read SPI0 bus ADCs
            for i in spi0_adcs:
                offset = (i * config.CHANNELS_PER_ADC + ch) * 3
                if ready[i]:
                    self.adcs[i].read_data_into(buf, offset)
                else:
                    buf[offset] = 0x80
                    buf[offset + 1] = 0x00
                    buf[offset + 2] = 0x00

            # Read SPI1 bus ADCs
            for i in spi1_adcs:
                offset = (i * config.CHANNELS_PER_ADC + ch) * 3
                if ready[i]:
                    self.adcs[i].read_data_into(buf, offset)
                else:
                    buf[offset] = 0x80
                    buf[offset + 1] = 0x00
                    buf[offset + 2] = 0x00

        return buf

    def read_single_channel(self, adc_index, ain_channel):
        """
        Read a single channel from a single ADC. Useful for testing.

        Returns:
            Signed 24-bit integer
        """
        return self.adcs[adc_index].read_channel(ain_channel)

    def read_temperatures(self):
        """
        Read the internal temperature sensor on each ADC.
        Uses the ADS124S08 built-in temp sensor (SYS_MON mode).

        Returns:
            List of temperature readings (raw codes) from each ADC
        """
        temps = []
        for adc in self.adcs:
            # Set SYS register to enable system monitor (temp sensor)
            sys_reg = adc.read_reg(0x09, 1)[0]
            adc.write_reg(0x09, sys_reg | 0x80)  # SYS_MON = 1

            # Read one conversion
            adc.start()
            if adc.wait_drdy(50):
                temps.append(adc.read_data())
            else:
                temps.append(None)

            # Restore SYS register
            adc.write_reg(0x09, sys_reg)

        return temps

    def deinit(self):
        """Shut down ADCs and release resources."""
        # Stop all conversions
        for adc in self.adcs:
            try:
                adc.stop()
            except Exception:
                pass

        # Disable clock
        if self.clk_en_pin:
            self.clk_en_pin.value(0)

        # Deinit SPI buses
        if self.spi0:
            self.spi0.deinit()
        if self.spi1:
            self.spi1.deinit()

        print("[ADC] Deinitialized.")
