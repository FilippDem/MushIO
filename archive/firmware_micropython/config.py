"""
MushIO V1.0 Configuration
Pin assignments, ADC parameters, WiFi credentials, and channel mapping.

IMPORTANT: GPIO pin assignments are best-guess from schematic extraction.
Verify against your physical schematic before first power-on.
"""

# =============================================================================
# SPI Bus Pin Assignments
# =============================================================================

# SPI0 Bus -> ADC0, ADC1, ADC2
SPI0_SCK  = 2   # GP2  -> R505 (22R) -> SPI0_SCLK
SPI0_MOSI = 3   # GP3  -> R506 (22R) -> SPI0_SDI (Pico TX -> ADC DIN)
SPI0_MISO = 4   # GP4  <- SPI0_SDO (ADC DOUT -> Pico RX)

# SPI1 Bus -> ADC3, ADC4, ADC5
SPI1_SCK  = 14  # GP14 -> R517 (22R) -> SPI1_SCLK
SPI1_MOSI = 15  # GP15 -> R518 (22R) -> SPI1_SDI
SPI1_MISO = 12  # GP12 <- SPI1_SDO

# =============================================================================
# ADC Chip Select and Data Ready Pins
# =============================================================================

# Chip selects (active low outputs, accent 22R series resistors on board)
# Index: [ADC0, ADC1, ADC2, ADC3, ADC4, ADC5]
ADC_CS_PINS = [5, 7, 9, 11, 13, 17]

# Data ready (active low inputs, accent directly connected)
# Index: [ADC0, ADC1, ADC2, ADC3, ADC4, ADC5]
ADC_DRDY_PINS = [6, 8, 10, 16, 18, 19]

# Which ADCs are on which SPI bus (index into ADC_CS_PINS / ADC_DRDY_PINS)
SPI0_ADC_INDICES = [0, 1, 2]  # ADC0, ADC1, ADC2 on SPI0
SPI1_ADC_INDICES = [3, 4, 5]  # ADC3, ADC4, ADC5 on SPI1

# =============================================================================
# Shared ADC Control Pins
# =============================================================================

ADC_START_PIN   = 21  # GP21 -> R511/R512 (22R) -> shared ADC START/SYNC
ADC_RESET_N_PIN = 20  # GP20 -> shared ADC RESET# (active low)
ADC_CLK_EN_PIN  = 22  # GP22 -> clock output enable (high = ext clock on)

# =============================================================================
# I2C Pins
# =============================================================================

I2C0_SDA_PIN = 0   # GP0 -> R503 (22R) -> I2C0_SDA (4.7k pull-up on board)
I2C0_SCL_PIN = 1   # GP1 -> R504 (22R) -> I2C0_SCL (4.7k pull-up on board)

# I2C device addresses
TEMP_SENSOR_ADDR = 0x48  # P3T1755DPZ (schematic shows 0x48, verify A0-A2)
IMU_ADDR         = 0x18  # LIS2DH12TR

# =============================================================================
# Analog / Status Pins
# =============================================================================

ANA_PGOOD_PIN = 28  # GP28 (ADC2) - analog power good sense

# =============================================================================
# SPI Configuration
# =============================================================================

SPI_BAUDRATE = 1_000_000  # 1 MHz (conservative, can go up to 4 MHz)
SPI_POLARITY = 0          # CPOL = 0
SPI_PHASE    = 1          # CPHA = 1 (SPI Mode 1 per ADS124S08 datasheet)
SPI_BITS     = 8

# =============================================================================
# ADS124S08 ADC Configuration
# =============================================================================

NUM_ADCS            = 6
CHANNELS_PER_ADC    = 12
TOTAL_CHANNELS      = NUM_ADCS * CHANNELS_PER_ADC  # 72

# Data rate setting for DATARATE register bits [3:0]
# 0x00=2.5, 0x01=5, 0x02=10, 0x03=16.6, 0x04=20, 0x05=50,
# 0x06=60, 0x07=100, 0x08=200, 0x09=400, 0x0A=800, 0x0B=1000,
# 0x0C=2000, 0x0D=4000
ADC_DATARATE = 0x0D  # 4000 SPS -> ~205 SPS/ch effective with 12-ch mux

# PGA register configuration
ADC_PGA_GAIN = 0  # PGA gain = 1 (bits [2:0]: 0=1, 1=2, 2=4, ... 7=128)
ADC_PGA_EN   = 0  # 0 = PGA disabled (bypass), 1 = PGA enabled

# Reference configuration (REF register)
# Using REFP0/REFN0 which are connected to AVDD_2V5/AVSS_N2V5
# REFSEL[1:0] = 0b00 -> REFP0/REFN0
# REFCON[1:0] = 0b00 -> internal ref off
# REFN_BUF = 0, REFP_BUF = 0 -> ref buffers disabled
ADC_REF_CONFIG = 0x00

# Input MUX: single-ended vs AINCOM
# AINCOM = 0x0C (pin 1, connected to AGND via 0R)
AINCOM = 0x0C

# Channel list: AIN0 through AIN11 (positive input, negative = AINCOM)
AIN_CHANNELS = list(range(12))  # [0, 1, 2, ..., 11]

# DRDY timeout in milliseconds
DRDY_TIMEOUT_MS = 10

# Conversion settling: wait this many conversions after MUX change
# sinc3 filter needs 3 conversion periods to settle
MUX_SETTLE_CONVERSIONS = 1  # Use 1 for speed, 3 for accuracy

# =============================================================================
# Reference Voltage (for converting raw counts to voltage)
# =============================================================================

# REFP0 = AVDD_2V5 = +2.52V, REFN0 = AVSS_N2V5 = -2.56V (from charge pump)
VREF_POS = 2.52   # Volts
VREF_NEG = -2.56  # Volts
VREF = VREF_POS - VREF_NEG  # 5.08V total reference span

# AFE gain (TLV4387, G = 1 + Rf/Rg = 1 + 5.1k/510 = 11)
AFE_GAIN = 11.0

# =============================================================================
# WiFi Configuration
# =============================================================================

WIFI_SSID     = "Occam's Router"
WIFI_PASSWORD  = "purplelotus636"  # TODO: remove before sharing
WIFI_COUNTRY   = "US"

# =============================================================================
# Streaming Configuration
# =============================================================================

HOST_IP   = "192.168.68.115"  # Host PC LAN IP on Occam's Router
HOST_PORT = 9000

# Frame sync word
SYNC_WORD = 0xAA55

# Ring buffer size (frames).  128 × 228 B = 29 KB.
# Sized to absorb ~0.4 s of 333-FPS generation if Core 1 TCP stalls briefly.
STREAM_BUFFER_SIZE = 128

# Reconnect interval in seconds
RECONNECT_INTERVAL_S = 5

# =============================================================================
# Electrode Channel Map
# =============================================================================
# Maps (adc_index, ain_channel) -> electrode name string
# Derived from schematic AFE->ADC routing on pages 4, 13, 20
# Format: CHANNEL_MAP[adc_index][ain_channel] = "ELECrc" (row, col)
#
# TODO: Verify this mapping against your schematic net connections.
# The GO (Gain Output) signals route through the AFE blocks to ADC inputs.
# This is a best-effort extraction from the schematic PDF.

CHANNEL_MAP = [
    # ADC0 (SPI0): 12 recording channels
    ["ELEC02", "ELEC23", "ELEC22", "ELEC12",
     "ELEC11", "ELEC21", "ELEC20", "ELEC01",
     "ELEC00", "ELEC10", "ELEC03", "ELEC13"],

    # ADC1 (SPI0): 12 recording channels
    ["ELEC26", "ELEC16", "ELEC25", "ELEC15",
     "ELEC24", "ELEC05", "ELEC04", "ELEC14",
     "ELEC17", "ELEC07", "ELEC06", "ELEC27"],

    # ADC2 (SPI0): AIN0-7 recording (8 ch), AIN8-11 stimulation (4 ch)
    ["ELEC41", "ELEC31", "ELEC30", "ELEC40",
     "ELEC43", "ELEC33", "ELEC32", "ELEC42",
     "STIM_1", "STIM_0", "STIM_7", "STIM_6"],

    # ADC3 (SPI1): AIN0-7 recording (8 ch), AIN8-11 stimulation (4 ch)
    ["ELEC45", "ELEC35", "ELEC34", "ELEC44",
     "ELEC47", "ELEC37", "ELEC36", "ELEC46",
     "STIM_3", "STIM_2", "STIM_5", "STIM_4"],

    # ADC4 (SPI1): 12 recording channels
    ["ELEC71", "ELEC50", "ELEC60", "ELEC70",
     "ELEC62", "ELEC52", "ELEC51", "ELEC61",
     "ELEC73", "ELEC63", "ELEC53", "ELEC72"],

    # ADC5 (SPI1): 12 recording channels
    ["ELEC75", "ELEC54", "ELEC64", "ELEC74",
     "ELEC66", "ELEC56", "ELEC55", "ELEC65",
     "ELEC67", "ELEC77", "ELEC76", "ELEC57"],
]

# =============================================================================
# Channel Classification: Recording vs Stimulation
# =============================================================================
# ADC2 and ADC3 are mixed: AIN0-7 are recording inputs connected to AFE,
# AIN8-11 are stimulation monitor inputs connected to STIM drive circuitry.
# ADC0, ADC1, ADC4, ADC5 are fully recording (all 12 AIN channels).
#
# Physical layout:
#   64 recording electrodes (8x8 grid: ELEC_rc where r=row 0-7, c=col 0-7)
#    8 stimulation channels  (STIM_0 through STIM_7)

# AIN channels used for recording on each ADC (0-indexed within that ADC)
RECORDING_AINCH_PER_ADC = [
    list(range(12)),  # ADC0: AIN0-11 all recording
    list(range(12)),  # ADC1: AIN0-11 all recording
    list(range(8)),   # ADC2: AIN0-7 recording, AIN8-11 STIM
    list(range(8)),   # ADC3: AIN0-7 recording, AIN8-11 STIM
    list(range(12)),  # ADC4: AIN0-11 all recording
    list(range(12)),  # ADC5: AIN0-11 all recording
]

# AIN channels used for stimulation on each ADC
STIM_AINCH_PER_ADC = [
    [],             # ADC0: no STIM channels
    [],             # ADC1: no STIM channels
    [8, 9, 10, 11], # ADC2: AIN8-11 are STIM_1, STIM_0, STIM_7, STIM_6
    [8, 9, 10, 11], # ADC3: AIN8-11 are STIM_3, STIM_2, STIM_5, STIM_4
    [],             # ADC4: no STIM channels
    [],             # ADC5: no STIM channels
]

# Flat indices into the 72-channel stream for STIM channels
# Flat index = adc_index * CHANNELS_PER_ADC + ain_channel
STIM_CHANNEL_INDICES = [
    2 * CHANNELS_PER_ADC + 8,   # 32: STIM_1
    2 * CHANNELS_PER_ADC + 9,   # 33: STIM_0
    2 * CHANNELS_PER_ADC + 10,  # 34: STIM_7
    2 * CHANNELS_PER_ADC + 11,  # 35: STIM_6
    3 * CHANNELS_PER_ADC + 8,   # 44: STIM_3
    3 * CHANNELS_PER_ADC + 9,   # 45: STIM_2
    3 * CHANNELS_PER_ADC + 10,  # 46: STIM_5
    3 * CHANNELS_PER_ADC + 11,  # 47: STIM_4
]  # 8 STIM channels

# Flat indices for recording channels (complement of STIM)
RECORDING_CHANNEL_INDICES = [
    i for i in range(TOTAL_CHANNELS) if i not in STIM_CHANNEL_INDICES
]  # 64 recording channels

NUM_RECORDING_CHANNELS = 64
NUM_STIM_CHANNELS      = 8
