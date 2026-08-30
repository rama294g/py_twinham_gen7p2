"""Interrupt-driven BNO055 UART protocol for Raspberry Pi Pico 2 W."""
import struct
import utime
from machine import Pin, UART, disable_irq, enable_irq

from app_state import Config

UART_ID = 1
UART_BAUDRATE = 115200
UART_TX_PIN = 4
UART_RX_PIN = 5

REG_CHIP_ID = 0x00
REG_SENSOR_DATA = 0x08
REG_OPR_MODE = 0x3D

SENSOR_DATA_LENGTH = 18
BNO_CHIP_ID = 0xA0
MODE_NDOF = 0x0C

ACC_SCALE = 100.0
GYRO_SCALE = 16.0

# The old mbed driver reserved 200 bytes.  Power-of-two rings make full/empty
# handling cheap in an interrupt while retaining ample room for BNO055 packets.
UART_RX_BUFFER_SIZE = 256
UART_TX_BUFFER_SIZE = 256
UART_BUFFER_MASK = UART_RX_BUFFER_SIZE - 1

BNO_READ_PENDING = 0
BNO_READ_COMPLETE = 1
BNO_READ_TIMEOUT = 2
BNO_READ_DATALEN_UNMATCH = 3


class InterruptUART:
    """Fixed-buffer UART transport driven by RX and TX interrupt callbacks."""

    def __init__(self):
        self.uart = UART(
            UART_ID,
            baudrate=UART_BAUDRATE,
            bits=8,
            parity=None,
            stop=1,
            tx=Pin(UART_TX_PIN),
            rx=Pin(UART_RX_PIN),
            rxbuf=UART_RX_BUFFER_SIZE,
            txbuf=UART_TX_BUFFER_SIZE
        )
        self.rx_buffer = bytearray(UART_RX_BUFFER_SIZE)
        self.tx_buffer = bytearray(UART_TX_BUFFER_SIZE)
        self.rx_head = 0
        self.rx_tail = 0
        self.tx_head = 0
        self.tx_tail = 0
        self.rx_overflow = False
        self.tx_overflow = False
        self._rx_byte = bytearray(1)
        self._tx_byte = bytearray(1)

        # Pico/RP2350 supports RX-idle and TX-idle UART events.  The callback is
        # a soft IRQ so the MicroPython UART methods are safe to call from it.
        trigger = UART.IRQ_RXIDLE | UART.IRQ_TXIDLE
        self.uart.irq(self._uart_irq, trigger=trigger, hard=False)

    def _uart_irq(self, uart):
        self._receive_irq(uart)
        self._transmit_irq(uart)

    def _receive_irq(self, uart):
        while uart.any():
            if uart.readinto(self._rx_byte, 1) != 1:
                break
            next_head = (self.rx_head + 1) & UART_BUFFER_MASK
            if next_head == self.rx_tail:
                self.rx_overflow = True
                break
            self.rx_buffer[self.rx_head] = self._rx_byte[0]
            self.rx_head = next_head

    def _transmit_irq(self, uart):
        # Send one byte per TX event, as in ref/cpp_sample/BNO055.h.  Calling
        # this once when queueing data starts transmission immediately.
        if self.tx_tail == self.tx_head:
            return
        self._tx_byte[0] = self.tx_buffer[self.tx_tail]
        if uart.write(self._tx_byte) == 1:
            self.tx_tail = (self.tx_tail + 1) & UART_BUFFER_MASK

    def write(self, data):
        irq_state = disable_irq()
        was_empty = self.tx_head == self.tx_tail
        used = (self.tx_head - self.tx_tail) & UART_BUFFER_MASK
        free = UART_TX_BUFFER_SIZE - 1 - used
        if len(data) > free:
            self.tx_overflow = True
            enable_irq(irq_state)
            return False
        for value in data:
            next_head = (self.tx_head + 1) & UART_BUFFER_MASK
            self.tx_buffer[self.tx_head] = value
            self.tx_head = next_head
        enable_irq(irq_state)

        if was_empty:
            self._transmit_irq(self.uart)
        return True

    def read_byte(self):
        irq_state = disable_irq()
        if self.rx_tail == self.rx_head:
            enable_irq(irq_state)
            return None
        value = self.rx_buffer[self.rx_tail]
        self.rx_tail = (self.rx_tail + 1) & UART_BUFFER_MASK
        enable_irq(irq_state)
        return value

    def clear(self):
        irq_state = disable_irq()
        self.rx_tail = self.rx_head
        self.rx_overflow = False
        enable_irq(irq_state)
        while self.uart.any():
            self.uart.readinto(self._rx_byte, 1)


print("UART INIT")
transport = InterruptUART()
print("UART INIT OK")

# Packet assembly happens outside IRQ context.  It is also fixed-size so a
# disconnected or noisy sensor cannot consume the Pico's heap indefinitely.
packet_buffer = bytearray(UART_RX_BUFFER_SIZE)
packet_length = 0
bno_read_pending = False
bno_read_reg = None
bno_read_length = 0
bno_read_started_ms = 0


def _receive_packets():
    global packet_length
    while True:
        value = transport.read_byte()
        if value is None:
            return
        if packet_length < UART_RX_BUFFER_SIZE:
            packet_buffer[packet_length] = value
            packet_length += 1
        else:
            # Retain the newest possible header and report a recoverable fault.
            packet_length = 1 if value == 0xBB else 0
            if packet_length:
                packet_buffer[0] = value
            transport.rx_overflow = True


def _drop_packet_prefix(count):
    global packet_length
    remaining = packet_length - count
    if remaining > 0:
        packet_buffer[:remaining] = packet_buffer[count:packet_length]
    packet_length = max(0, remaining)


def _take_response(expected_length):
    """Return ``(header, body)`` for one complete BNO055 response."""
    while packet_length:
        header = packet_buffer[0]
        if header == 0xEE:
            if packet_length < 2:
                return None
            response = bytes(packet_buffer[:2])
            _drop_packet_prefix(2)
            return 0xEE, response
        if header != 0xBB:
            _drop_packet_prefix(1)
            continue
        if packet_length < 2:
            return None
        response_length = packet_buffer[1]
        total_length = response_length + 2
        if packet_length < total_length:
            return None
        payload = bytes(packet_buffer[2:total_length])
        _drop_packet_prefix(total_length)
        return 0xBB, payload
    return None


def uart_clear(resync=False):
    global packet_length
    if resync:
        transport.clear()
        packet_length = 0


def bno_read(reg, length):
    global bno_read_pending, bno_read_reg, bno_read_length, bno_read_started_ms
    if not bno_read_pending:
        if not transport.write(bytes((0xAA, 0x01, reg, length))):
            return BNO_READ_TIMEOUT, None
        bno_read_pending = True
        bno_read_reg = reg
        bno_read_length = length
        bno_read_started_ms = utime.ticks_ms()
    elif reg != bno_read_reg or length != bno_read_length:
        return BNO_READ_PENDING, None

    _receive_packets()
    response = _take_response(length)
    if response is not None and response[0] == 0xBB:
        bno_read_pending = False
        if len(response[1]) != length:
            return BNO_READ_DATALEN_UNMATCH, None
        return BNO_READ_COMPLETE, response[1]
    if utime.ticks_diff(utime.ticks_ms(), bno_read_started_ms) > Config.UART_TIMEOUT_MS:
        bno_read_pending = False
        uart_clear(resync=True)
        return BNO_READ_TIMEOUT, None
    return BNO_READ_PENDING, None


def bno_read_blocking(reg, length):
    while True:
        result, data = bno_read(reg, length)
        if result == BNO_READ_COMPLETE:
            return data
        if result == BNO_READ_TIMEOUT:
            return None
        utime.sleep_ms(1)


def bno_write(reg, value):
    if not transport.write(bytes((0xAA, 0x00, reg, 0x01, value))):
        return False
    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) <= Config.UART_TIMEOUT_MS:
        _receive_packets()
        response = _take_response(0)
        if response is not None:
            return (
                response[0] == 0xEE and
                len(response[1]) == 2 and
                response[1][1] == 0x01
            )
        utime.sleep_ms(1)
    uart_clear(resync=True)
    return False


def read_acc_gyro():
    result, data = bno_read(REG_SENSOR_DATA, SENSOR_DATA_LENGTH)
    if result != BNO_READ_COMPLETE:
        return result, None
    if len(data) != SENSOR_DATA_LENGTH:
        return BNO_READ_DATALEN_UNMATCH, None
    values = struct.unpack("<9h", data)
    return BNO_READ_COMPLETE, (
        values[0] / ACC_SCALE,
        values[1] / ACC_SCALE,
        values[2] / ACC_SCALE,
        values[6] / GYRO_SCALE,
        values[7] / GYRO_SCALE,
        values[8] / GYRO_SCALE
    )


def bno_init():
    print("\nCHECK CHIP ID")
    uart_clear(resync=True)
    chip_id = bno_read_blocking(REG_CHIP_ID, 1)
    if chip_id is None:
        print("CHIP ID READ ERROR")
        return False
    print("CHIP ID = 0x{:02X}".format(chip_id[0]))
    if chip_id[0] != BNO_CHIP_ID:
        print("INVALID CHIP ID")
        return False

    print("SET NDOF")
    if not bno_write(REG_OPR_MODE, MODE_NDOF):
        print("NDOF WRITE ERROR")
        return False
    utime.sleep_ms(100)
    mode = bno_read_blocking(REG_OPR_MODE, 1)
    if mode is None:
        print("OPR_MODE READ ERROR")
        return False
    print("OPR_MODE = 0x{:02X}".format(mode[0]))
    if mode[0] != MODE_NDOF:
        print("NDOF MODE ERROR")
        return False
    print("NDOF MODE OK")
    return True
