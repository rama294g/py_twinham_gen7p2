"""BNO055 UART protocol and device initialization."""
import struct
import utime
from machine import Pin, UART

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

print("UART INIT")

uart = UART(
    UART_ID,
    baudrate=UART_BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(UART_TX_PIN),
    rx=Pin(UART_RX_PIN)
)

print("UART INIT OK")


# Bytes and transaction state live across sensor-task iterations.  UART replies
# commonly arrive in more than one scheduler step, so a step must never own a
# temporary receive buffer.
bno_rx_queue = bytearray()
bno_read_pending = False
bno_read_reg = None
bno_read_length = 0
bno_read_started_ms = 0

BNO_READ_PENDING = 0
BNO_READ_COMPLETE = 1
BNO_READ_TIMEOUT = 2


def uart_clear(resync=False):

    # Clearing is deliberately restricted to startup or an explicit recovery.
    # Normal reads preserve both partial packets and following packets.
    if not resync:
        return

    bno_rx_queue[:] = b""

    while uart.any():
        uart.read()


def _uart_receive_step():

    # Check any() exactly once per polling step and never wait for input.
    available = uart.any()

    if available == 0:
        return

    chunk = uart.read(available)

    if chunk:
        bno_rx_queue.extend(chunk)


def _take_bno_packet(expected_length):

    while True:

        # Discard only bytes which cannot start a read response.
        try:
            start_index = bno_rx_queue.index(0xBB)
        except ValueError:
            bno_rx_queue[:] = b""
            return None

        if start_index:
            del bno_rx_queue[:start_index]

        if len(bno_rx_queue) < 2:
            return None

        payload_length = bno_rx_queue[1]
        packet_length = 2 + payload_length

        if len(bno_rx_queue) < packet_length:
            return None

        payload = bytes(bno_rx_queue[2:packet_length])
        del bno_rx_queue[:packet_length]

        if payload_length == expected_length:
            return payload


def _discard_incomplete_response():

    # At timeout the bytes belonging to this unfinished response are no longer
    # useful.  Do not flush hardware UART data or complete trailing packets.
    try:
        start_index = bno_rx_queue.index(0xBB)
    except ValueError:
        bno_rx_queue[:] = b""
        return

    if len(bno_rx_queue) < start_index + 2:
        del bno_rx_queue[:]
        return

    packet_length = 2 + bno_rx_queue[start_index + 1]

    if len(bno_rx_queue) < start_index + packet_length:
        # A later header may already begin a separate complete response.  Drop
        # only through the bytes of the timed-out fragment in that case.
        try:
            next_start = bno_rx_queue.index(0xBB, start_index + 2)
        except ValueError:
            del bno_rx_queue[:]
        else:
            del bno_rx_queue[:next_start]


def bno_read(reg, length):

    global bno_read_pending
    global bno_read_reg
    global bno_read_length
    global bno_read_started_ms

    if not bno_read_pending:

        uart.write(bytes([
            0xAA,
            0x01,
            reg,
            length
        ]))

        bno_read_pending = True
        bno_read_reg = reg
        bno_read_length = length
        bno_read_started_ms = utime.ticks_ms()

    elif reg != bno_read_reg or length != bno_read_length:
        # A caller cannot replace an in-flight transaction.
        return BNO_READ_PENDING, None

    _uart_receive_step()

    payload = _take_bno_packet(length)

    if payload is not None:
        bno_read_pending = False
        return BNO_READ_COMPLETE, payload

    if utime.ticks_diff(
        utime.ticks_ms(),
        bno_read_started_ms
    ) > Config.UART_TIMEOUT_MS:
        _discard_incomplete_response()
        bno_read_pending = False
        return BNO_READ_TIMEOUT, None

    return BNO_READ_PENDING, None


def bno_read_blocking(reg, length):

    # Initialization runs before asyncio tasks start, so it may poll the same
    # state machine while yielding briefly to the UART.
    while True:

        result, data = bno_read(reg, length)

        if result == BNO_READ_COMPLETE:
            return data

        if result == BNO_READ_TIMEOUT:
            return None

        utime.sleep_ms(1)


def bno_write(reg, value):

    uart.write(bytes([
        0xAA,
        0x00,
        reg,
        0x01,
        value
    ]))

    start = utime.ticks_ms()

    while uart.any() == 0:

        if utime.ticks_diff(
            utime.ticks_ms(),
            start
        ) > Config.UART_TIMEOUT_MS:
            return False

        utime.sleep_ms(1)

    utime.sleep_ms(2)

    data = uart.read()

    if data is None:
        return False

    return (
        len(data) >= 2 and
        data[0] == 0xEE and
        data[1] == 0x01
    )


def read_acc_gyro():

    result, data = bno_read(
        REG_SENSOR_DATA,
        SENSOR_DATA_LENGTH
    )

    if result != BNO_READ_COMPLETE:
        return result, None

    if len(data) != SENSOR_DATA_LENGTH:
        return BNO_READ_TIMEOUT, None

    try:

        acc_x_raw = struct.unpack(
            "<h", data[0:2]
        )[0]

        acc_y_raw = struct.unpack(
            "<h", data[2:4]
        )[0]

        acc_z_raw = struct.unpack(
            "<h", data[4:6]
        )[0]

        gyro_x_raw = struct.unpack(
            "<h", data[12:14]
        )[0]

        gyro_y_raw = struct.unpack(
            "<h", data[14:16]
        )[0]

        gyro_z_raw = struct.unpack(
            "<h", data[16:18]
        )[0]

    except Exception:
        return BNO_READ_TIMEOUT, None

    return BNO_READ_COMPLETE, (
        acc_x_raw / ACC_SCALE,
        acc_y_raw / ACC_SCALE,
        acc_z_raw / ACC_SCALE,
        gyro_x_raw / GYRO_SCALE,
        gyro_y_raw / GYRO_SCALE,
        gyro_z_raw / GYRO_SCALE
    )



def bno_init():

    print()
    print("CHECK CHIP ID")

    uart_clear(resync=True)

    chip_id = bno_read_blocking(
        REG_CHIP_ID,
        1
    )

    if chip_id is None:

        print("CHIP ID READ ERROR")
        return False

    print(
        "CHIP ID = 0x{:02X}".format(
            chip_id[0]
        )
    )

    if chip_id[0] != BNO_CHIP_ID:

        print("INVALID CHIP ID")
        return False

    print("SET NDOF")

    write_ok = bno_write(
        REG_OPR_MODE,
        MODE_NDOF
    )

    if not write_ok:
        print("NDOF WRITE ERROR")

    utime.sleep_ms(100)

    mode = bno_read_blocking(
        REG_OPR_MODE,
        1
    )

    if mode is None:

        print("OPR_MODE READ ERROR")
        return False

    print(
        "OPR_MODE = 0x{:02X}".format(
            mode[0]
        )
    )

    if mode[0] != MODE_NDOF:

        print("NDOF MODE ERROR")
        return False

    print("NDOF MODE OK")
    return True


