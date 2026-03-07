"""
ADS124S08 24-bit ADC SPI Driver for MicroPython on RP2350.

Datasheet: Texas Instruments ADS124S08
SPI Mode 1 (CPOL=0, CPHA=1)
24-bit conversion result, MSB first.

Each instance represents one physical ADS124S08 chip with its own
CS and DRDY pins, sharing an SPI bus with other ADCs.
"""

import time
from machine import Pin


# =============================================================================
# Command Opcodes
# =============================================================================
CMD_NOP       = 0x00
CMD_WAKEUP    = 0x02
CMD_POWERDOWN = 0x04
CMD_RESET     = 0x06
CMD_START     = 0x08
CMD_STOP      = 0x0A
CMD_RDATA     = 0x12
CMD_RREG      = 0x20  # OR with register address
CMD_WREG      = 0x40  # OR with register address
CMD_SFOCAL    = 0x19  # Self offset calibration
CMD_SYGCAL    = 0x13  # System gain calibration
CMD_SYOCAL    = 0x16  # System offset calibration
CMD_LOCK      = 0x16
CMD_UNLOCK    = 0x16

# =============================================================================
# Register Addresses
# =============================================================================
REG_ID       = 0x00
REG_STATUS   = 0x01
REG_INPMUX   = 0x02
REG_PGA      = 0x03
REG_DATARATE = 0x04
REG_REF      = 0x05
REG_IDACMAG  = 0x06
REG_IDACMUX  = 0x07
REG_VBIAS    = 0x08
REG_SYS      = 0x09
REG_OFCAL0   = 0x0A
REG_OFCAL1   = 0x0B
REG_OFCAL2   = 0x0C
REG_FSCAL0   = 0x0D
REG_FSCAL1   = 0x0E
REG_FSCAL2   = 0x0F
REG_GPIODAT  = 0x10
REG_GPIOCON  = 0x11

# =============================================================================
# Data Rate Settings (DATARATE register bits [3:0])
# =============================================================================
DR_2P5   = 0x00
DR_5     = 0x01
DR_10    = 0x02
DR_16P6  = 0x03
DR_20    = 0x04
DR_50    = 0x05
DR_60    = 0x06
DR_100   = 0x07
DR_200   = 0x08
DR_400   = 0x09
DR_800   = 0x0A
DR_1000  = 0x0B
DR_2000  = 0x0C
DR_4000  = 0x0D

# ADS124S08 device ID: bits [4:0] should read 0x00
ADS124S08_ID_MASK = 0x1F
ADS124S08_ID_VAL  = 0x00

# AINCOM channel for single-ended measurements
AINCOM = 0x0C

# Error marker for failed reads
ERROR_MARKER = 0x800000


class ADS124S08:
    """Driver for a single ADS124S08 ADC on a shared SPI bus."""

    def __init__(self, spi, cs_pin_num, drdy_pin_num, name="ADC"):
        """
        Args:
            spi: machine.SPI instance (already configured for mode 1)
            cs_pin_num: GPIO number for chip select (active low)
            drdy_pin_num: GPIO number for data ready (active low)
            name: label for debug prints
        """
        self.spi = spi
        self.cs = Pin(cs_pin_num, Pin.OUT, value=1)   # CS high = deselected
        self.drdy = Pin(drdy_pin_num, Pin.IN, Pin.PULL_UP)
        self.name = name

        # Pre-allocate buffers to reduce GC pressure in hot loop
        self._cmd1 = bytearray(1)
        self._cmd2 = bytearray(2)
        self._cmd3 = bytearray(3)
        self._rxbuf3 = bytearray(3)
        self._txnop3 = bytearray(3)  # All zeros = NOP bytes for reading

    def _cs_low(self):
        self.cs.value(0)

    def _cs_high(self):
        self.cs.value(1)

    def _send_command(self, cmd):
        """Send a single-byte command."""
        self._cmd1[0] = cmd
        self._cs_low()
        self.spi.write(self._cmd1)
        self._cs_high()

    # =========================================================================
    # Register Access
    # =========================================================================

    def read_reg(self, addr, count=1):
        """
        Read one or more consecutive registers.

        Args:
            addr: Starting register address (0x00-0x11)
            count: Number of registers to read (1-18)

        Returns:
            bytearray of register values
        """
        buf = bytearray(count)
        self._cmd2[0] = CMD_RREG | (addr & 0x1F)
        self._cmd2[1] = (count - 1) & 0xFF

        self._cs_low()
        self.spi.write(self._cmd2)
        # Need a small delay (td(SCCS)) before reading back
        self.spi.readinto(buf)
        self._cs_high()
        return buf

    def write_reg(self, addr, data):
        """
        Write one or more consecutive registers.

        Args:
            addr: Starting register address (0x00-0x11)
            data: bytes or bytearray of values to write
        """
        if isinstance(data, int):
            data = bytes([data])
        n = len(data)
        self._cmd2[0] = CMD_WREG | (addr & 0x1F)
        self._cmd2[1] = (n - 1) & 0xFF

        self._cs_low()
        self.spi.write(self._cmd2)
        self.spi.write(data)
        self._cs_high()

    # =========================================================================
    # Device Control
    # =========================================================================

    def reset(self):
        """Software reset. Wait for device to come out of reset."""
        self._send_command(CMD_RESET)
        # After reset, wait at least 4096 * tCLK = 4096/4.096MHz = 1ms
        time.sleep_ms(5)

    def start(self):
        """Send START command to begin conversion."""
        self._send_command(CMD_START)

    def stop(self):
        """Send STOP command to halt conversions."""
        self._send_command(CMD_STOP)

    def wakeup(self):
        """Wake from power-down mode."""
        self._send_command(CMD_WAKEUP)

    def powerdown(self):
        """Enter power-down mode."""
        self._send_command(CMD_POWERDOWN)

    # =========================================================================
    # Data Acquisition
    # =========================================================================

    def wait_drdy(self, timeout_ms=10):
        """
        Poll DRDY pin until it goes low (data ready) or timeout.

        Returns:
            True if data is ready, False if timed out
        """
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        while self.drdy.value() != 0:
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return False
        return True

    def is_drdy(self):
        """Check if DRDY is asserted (low) right now."""
        return self.drdy.value() == 0

    def read_data_raw(self):
        """
        Read 24-bit conversion result via RDATA command.

        Returns:
            bytearray(3) with raw MSB-first data, or None on error.
        """
        self._cmd1[0] = CMD_RDATA
        self._cs_low()
        self.spi.write(self._cmd1)
        self.spi.readinto(self._rxbuf3)
        self._cs_high()
        return self._rxbuf3

    def read_data(self):
        """
        Read 24-bit conversion result as a signed integer.

        Returns:
            Signed integer (-8388608 to +8388607) or ERROR_MARKER on failure.
        """
        raw = self.read_data_raw()
        if raw is None:
            return ERROR_MARKER
        value = (raw[0] << 16) | (raw[1] << 8) | raw[2]
        # Two's complement sign extension
        if value & 0x800000:
            value -= 0x1000000
        return value

    def read_data_into(self, buf, offset):
        """
        Read 24-bit conversion result directly into a buffer.

        Args:
            buf: bytearray target
            offset: byte offset to write 3 bytes into buf
        """
        self._cmd1[0] = CMD_RDATA
        self._cs_low()
        self.spi.write(self._cmd1)
        # Read 3 bytes directly into the target buffer at offset
        mv = memoryview(buf)[offset:offset+3]
        self.spi.readinto(mv)
        self._cs_high()

    # =========================================================================
    # Input Multiplexer
    # =========================================================================

    def set_mux(self, pos_ain, neg_ain=AINCOM):
        """
        Set the input multiplexer.

        Args:
            pos_ain: Positive input channel (0-11, or 0x0C for AINCOM)
            neg_ain: Negative input channel (default AINCOM = 0x0C)
        """
        mux_val = ((pos_ain & 0x0F) << 4) | (neg_ain & 0x0F)
        self.write_reg(REG_INPMUX, mux_val)

    # =========================================================================
    # Configuration
    # =========================================================================

    def read_id(self):
        """
        Read the device ID register.

        Returns:
            (id_byte, is_valid) - raw ID byte and whether it matches ADS124S08
        """
        data = self.read_reg(REG_ID, 1)
        id_byte = data[0]
        is_valid = (id_byte & ADS124S08_ID_MASK) == ADS124S08_ID_VAL
        return id_byte, is_valid

    def configure(self, datarate=DR_4000, pga_gain=0, pga_enable=0,
                  ref_sel=0, clk_sel=1):
        """
        Configure ADC registers for measurement.

        Args:
            datarate: Data rate code (DR_2P5 through DR_4000)
            pga_gain: PGA gain code (0=1x, 1=2x, 2=4x, ... 7=128x)
            pga_enable: 0=PGA bypass, 1=PGA enabled
            ref_sel: Reference selection (0=REFP0/REFN0, 1=REFP1/REFN1, 2=internal)
            clk_sel: Clock source (0=internal, 1=external)
        """
        # PGA register (0x03):
        # [7] DELAY[2] = 0
        # [6:5] DELAY[1:0] = 0b00 (no extra delay)
        # [4:2] GAIN[2:0] = pga_gain  (bits 4:2, not 2:0)
        # [0] PGA_EN = pga_enable      (bit 0, not bit 3)
        pga_reg = ((pga_gain & 0x07) << 2) | (pga_enable & 0x01)
        self.write_reg(REG_PGA, pga_reg)

        # DATARATE register (0x04):
        # [7] G_CHOP = 0 (global chop disabled for speed)
        # [6] CLK = clk_sel (0=internal, 1=external)
        # [5] MODE = 0 (continuous conversion mode)
        # [4] FILTER = 0 (sinc3 filter)
        # [3:0] DR = datarate
        dr_reg = (clk_sel << 6) | (datarate & 0x0F)
        self.write_reg(REG_DATARATE, dr_reg)

        # REF register (0x05):
        # [3:2] REFSEL = ref_sel  (bits 3:2, not 5:4)
        # [1] REFN_BUF = 0
        # [0] REFP_BUF = 0
        ref_reg = (ref_sel & 0x03) << 2
        self.write_reg(REG_REF, ref_reg)

        # SYS register (0x09):
        # [7] SYS_MON = 0
        # [6] CAL_SAMP = 0
        # [5] TIMEOUT = 1 (enable SPI timeout for robustness)
        # [4] CRC = 0 (no CRC on SPI for speed)
        # [3] SENDSTAT = 0 (don't prepend status byte to data)
        # [2:0] reserved
        sys_reg = 0x20  # Just timeout enabled
        self.write_reg(REG_SYS, sys_reg)

        # Configure GPIO pins as analog inputs (for AIN8-11)
        # GPIOCON register (0x11): set bits [3:0] = 0 for analog input mode
        self.write_reg(REG_GPIOCON, 0x00)

        # Set initial MUX to AIN0 vs AINCOM
        self.set_mux(0, AINCOM)

    def read_channel(self, ain, timeout_ms=10):
        """
        Set MUX to a channel, start conversion, wait for result, and read.

        Args:
            ain: AIN channel number (0-11)
            timeout_ms: DRDY timeout

        Returns:
            Signed 24-bit integer, or ERROR_MARKER on timeout
        """
        self.set_mux(ain, AINCOM)
        self.start()
        if not self.wait_drdy(timeout_ms):
            return ERROR_MARKER
        return self.read_data()

    def self_offset_calibrate(self, timeout_ms=200):
        """
        Run self offset calibration (SFOCAL command).

        Shorts the ADC inputs internally and measures the offset, storing the
        result in the OFCAL registers. Must be called after configure() with
        the ADC in the same gain/mux setting used for measurement.

        Returns:
            True if calibration completed, False if timed out.
        """
        self._send_command(CMD_SFOCAL)
        # SFOCAL takes up to 100ms at low data rates; DRDY pulses when done
        return self.wait_drdy(timeout_ms)

    def dump_registers(self):
        """Read and return all 18 registers for debugging."""
        return self.read_reg(REG_ID, 18)
