"""BATTERY VOLTAGE SENSING."""

# =========================================================
# battery.py
#
# NiMH Battery Voltage Measurement
#
# MCP3425
#   I2C(1)
#   GP10 = SDA
#   GP11 = SCL
#   Address = 0x68
#
# Voltage divider
#   200kΩ / 22kΩ
# =========================================================

import uasyncio as asyncio
import utime
from machine import I2C, Pin

# =========================================================
# MCP3425 SETTINGS
# =========================================================

MCP_I2C_ID = 1
MCP_SDA_PIN = 10
MCP_SCL_PIN = 11
MCP_ADDR = 0x68
MCP_I2C_FREQ = 100000

# =========================================================
# VOLTAGE DIVIDER
# =========================================================

R_HIGH = 200000.0
R_LOW = 22000.0

VOLTAGE_MULTIPLIER = (
    (R_HIGH + R_LOW) / R_LOW
)

# =========================================================
# CALIBRATION
#
# Based on actual measurements
#
# 7V  -> 6.884V
# 8V  -> 7.867V
# 10V -> 9.834V
# 12V -> 11.801V
# 14V -> 13.769V
#
# Calibration factor
# =========================================================

CALIBRATION_FACTOR = 1.0167

# =========================================================
# MCP3425 CONFIGURATION
#
# 16-bit
# Continuous conversion
# PGA x1
#
# 0x98
# =========================================================

MCP_CONFIG = 0x98
ADC_LSB_V = 2.048 / 32768.0

# =========================================================
# MEASUREMENT SETTINGS
# =========================================================

MEASURE_INTERVAL_MS = 100
AVERAGE_COUNT = 8

# =========================================================
# VOLTAGE STATUS
# =========================================================

LOW_VOLTAGE = 12.0
VERY_LOW_VOLTAGE = 11.0
HIGH_VOLTAGE = 18.5

# =========================================================
# BatteryMonitor
# =========================================================

class BatteryMonitor:

    def __init__(self):

        self.i2c = I2C(MCP_I2C_ID,sda=Pin(MCP_SDA_PIN,Pin.PULL_UP),scl=Pin(MCP_SCL_PIN,Pin.PULL_UP),freq=MCP_I2C_FREQ)

        self.voltage_buffer = []
        self.raw = 0
        self.adc_voltage = 0.0
        self.battery_voltage = 0.0
        self.average_voltage = 0.0
        self.config = 0
        self.status = "INIT"
        self.connected = False

    # =====================================================
    # INITIALIZE
    # =====================================================

    def init(self):

        devices = self.i2c.scan()

        print("MCP I2C devices:",
            [
                "0x{:02X}".format(x)
                for x in devices
            ]
        )

        if MCP_ADDR not in devices:

            print("MCP3425 NOT FOUND")

            self.connected = False

            return False

        print("MCP3425 FOUND")

        # -------------------------------------------------
        # CONFIG
        # -------------------------------------------------

        self.i2c.writeto(
            MCP_ADDR,
            bytes([MCP_CONFIG])
        )

        print(
            "MCP3425 CONFIG = 0x{:02X}".format(
                MCP_CONFIG
            )
        )

        # 16-bit conversion
        utime.sleep_ms(100)

        self.connected = True

        return True

    # =====================================================
    # READ
    # =====================================================

    def read(self):

        if not self.connected:

            return False

        data = self.i2c.readfrom(MCP_ADDR,3)

        # -------------------------------------------------
        # RAW
        # -------------------------------------------------

        raw = (
            (data[0] << 8)
            | data[1]
        )

        # signed 16-bit

        if raw & 0x8000:

            raw -= 65536

        self.raw = raw

        # -------------------------------------------------
        # ADC voltage
        # -------------------------------------------------

        self.adc_voltage = (raw * ADC_LSB_V)

        # -------------------------------------------------
        # Battery voltage
        # -------------------------------------------------

        self.battery_voltage = (self.adc_voltage * VOLTAGE_MULTIPLIER * CALIBRATION_FACTOR)

        # -------------------------------------------------
        # CONFIG
        # -------------------------------------------------

        self.config = data[2]

        # -------------------------------------------------
        # Moving average
        # -------------------------------------------------

        self.voltage_buffer.append(
            self.battery_voltage
        )

        if len(self.voltage_buffer) > AVERAGE_COUNT:

            self.voltage_buffer.pop(0)

        self.average_voltage = (
            sum(self.voltage_buffer)
            / len(self.voltage_buffer)
        )

        # -------------------------------------------------
        # Status
        # -------------------------------------------------

        if self.average_voltage <= VERY_LOW_VOLTAGE:

            self.status = "VERY LOW"

        elif self.average_voltage <= LOW_VOLTAGE:

            self.status = "LOW"

        elif self.average_voltage >= HIGH_VOLTAGE:

            self.status = "HIGH"

        else:

            self.status = "OK"

        return True

    # =====================================================
    # GETTERS
    # =====================================================

    def get_voltage(self):

        return self.average_voltage


    def get_raw_voltage(self):

        return self.battery_voltage


    def get_adc_voltage(self):

        return self.adc_voltage


    def get_raw(self):

        return self.raw


    def get_status(self):

        return self.status


    def get_config(self):

        return self.config