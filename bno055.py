
import struct
from machine import Pin, UART


class BNO055:

    _BUFFER_SIZE = 200

    def __init__(self, tx, rx, uart_id=1):
        self.i = 0
        self.mbno055_uart = UART(
            uart_id,
            baudrate=115200,
            bits=8,
            parity=None,
            stop=1,
            tx=tx if isinstance(tx, Pin) else Pin(tx),
            rx=rx if isinstance(rx, Pin) else Pin(rx),
            rxbuf=self._BUFFER_SIZE,
        )
        self.m_uart_data_len = 0
        self.m_uart_data = bytearray(self._BUFFER_SIZE)
        self.m_bno055_rx_gyro = None
        self.m_bno055_rx_chip_id = None
        self.rx_data = bytearray(self._BUFFER_SIZE)
        self.rx_data_len = 0

        # RP2350's UART RX-idle interrupt is the MicroPython equivalent of
        # mbed's SerialBase::RxIrq.  A soft IRQ permits normal UART methods.
        self.mbno055_uart.irq(
            self.bno055_rx, trigger=UART.IRQ_RXIDLE, hard=False
        )

    def attach_rx_callback(self, bno055_rx_gyro, bno055_rx_chip_id):
        self.m_bno055_rx_gyro = bno055_rx_gyro
        self.m_bno055_rx_chip_id = bno055_rx_chip_id

    def set_baudrate(self, baudrate):
        self.mbno055_uart.init(
            baudrate=baudrate, bits=8, parity=None, stop=1
        )

    def tx_CR(self):
        self.add_uart_data(bytes((0x0D,)))

    def tx_CHIP_ID(self):
        self.add_uart_data(bytes((0xAA, 0x01, 0x00, 0x01)))

    def tx_OPR_MODE(self):
        # Change to ACCGYRO mode.
        self.add_uart_data(bytes((0xAA, 0x00, 0x3D, 0x01, 0x05)))

    def tx_OPR_MODE2(self):
        self.mbno055_uart.write(bytes((3,)))

    def tx_GET_ACCGYRO(self):
        self.add_uart_data(bytes((0xAA, 0x01, 0x08, 0x12)))

    @staticmethod
    def memsft(data, data_len, shift_to_left):
        """Shift the active portion of ``data`` as the C++ ``memsft`` did."""
        if shift_to_left > 0:
            remaining = data_len - shift_to_left
            if remaining > 0:
                data[:remaining] = data[shift_to_left:data_len]
            data[max(0, remaining):data_len] = bytes(min(shift_to_left, data_len))
        elif shift_to_left < 0:
            shift_to_right = -shift_to_left
            remaining = data_len - shift_to_right
            if remaining > 0:
                data[shift_to_right:data_len] = data[:remaining]
            data[:min(shift_to_right, data_len)] = bytes(
                min(shift_to_right, data_len)
            )

    def cue_uartdata(self, data, data_len):
        for index in range(data_len):
            if data[index] == 0xBB:
                self.memsft(data, data_len, index)
                return True, data_len - index
        return False, 0

    def extract_packet(self, data, data_len):
        found, data_len = self.cue_uartdata(data, data_len)
        if not found or data_len < 3:
            return False, data_len

        packet_len = data[1] + 2
        if data_len < packet_len:
            return False, data_len

        self.parse_packet(data, packet_len)
        self.memsft(data, data_len, packet_len)
        return True, data_len - packet_len

    def parse_packet(self, packet, packet_len):
        if packet_len == 20: # and packet[8:14] == b"\x00" * 6:
            values = struct.unpack_from("<9h", packet, 2)
            accx = values[0] * 0.001 * 9.801
            accy = values[1] * 0.001 * 9.801
            accz = values[2] * 0.001 * 9.801
            gyrox = values[6] / 16.0 / 180.0 * 3.14156
            gyroy = values[7] / 16.0 / 180.0 * 3.14156
            gyroz = values[8] / 16.0 / 180.0 * 3.14156
            if self.m_bno055_rx_gyro is not None:
                self.m_bno055_rx_gyro(
                    accx, accy, accz, gyrox, gyroy, gyroz
                )
        elif packet_len == 3 and packet[1] == 1:
            if self.m_bno055_rx_chip_id is not None:
                self.m_bno055_rx_chip_id(packet[2])

    def bno055_rx(self, uart=None):
        # MicroPython passes the UART object to IRQ handlers.
        uart = self.mbno055_uart if uart is None else uart
        while uart.any():
            room = self._BUFFER_SIZE - self.rx_data_len
            if room <= 0:
                # The C++ sample used a fixed 200-byte receive buffer.  On
                # overflow, discard it so the next response can resynchronise.
                self.rx_data_len = 0
                room = self._BUFFER_SIZE
            chunk = uart.read(min(uart.any(), room))
            if not chunk:
                break
            end = self.rx_data_len + len(chunk)
            self.rx_data[self.rx_data_len:end] = chunk
            self.rx_data_len = end

        extracted = True
        while extracted:
            extracted, self.rx_data_len = self.extract_packet(
                self.rx_data, self.rx_data_len
            )

    def pc_tx(self):
        """Transmit the queued bytes (UART.write is buffered on MicroPython)."""
        if self.m_uart_data_len:
            self.mbno055_uart.write(
                memoryview(self.m_uart_data)[:self.m_uart_data_len]
            )
        self.m_uart_data_len = 0
        self.i = 0

    def add_uart_data(self, uart_data, uart_data_len=None):
        if uart_data_len is None:
            uart_data_len = len(uart_data)
        if self.m_uart_data_len + uart_data_len > self._BUFFER_SIZE:
            raise ValueError("BNO055 UART transmit buffer overflow")
        start = self.m_uart_data_len
        end = start + uart_data_len
        self.m_uart_data[start:end] = uart_data[:uart_data_len]
        self.m_uart_data_len = end
        self.pc_tx()
