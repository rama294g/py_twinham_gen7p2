"""TwinHAM composition root.

Hardware drivers and application tasks live in feature-specific modules; this
module only wires them together and owns application startup/shutdown.
"""

import math
import gc
import uasyncio as asyncio
import utime
from machine import Pin, PWM

from app_state import Config, MENU_MAIN, clamp, state
from bno055 import BNO055
from config_store import load_config
from lcd_menu import display_task, joystick_task, lcd_print
from sensing import update_angle
from blue_commu import BLECommunication


# =========================================================
# Raspberry Pi Pico2W Pin Assignment
UART1_TX_PIN = 4
UART1_RX_PIN = 5
ZERO_SWITCH_PIN = 6
MOTOR_CW_PIN = 16
MOTOR_CCW_PIN = 17
MOTOR_SLEEP_PIN = 18


# =========================================================
# Create Pin
uart1_tx_pin = Pin(UART1_TX_PIN)
uart1_rx_pin = Pin(UART1_RX_PIN)
zero_switch_pin = Pin(ZERO_SWITCH_PIN, Pin.IN, Pin.PULL_UP)
cw_pin = Pin(MOTOR_CW_PIN, Pin.OUT)
ccw_pin = Pin(MOTOR_CCW_PIN, Pin.OUT)
sleep_pin = Pin(MOTOR_SLEEP_PIN, Pin.OUT)


m_gyro2 = BNO055(uart1_tx_pin, uart1_rx_pin)
m_chip_id = None
m_accx = 0.0
m_accy = 0.0
m_accz = 0.0
m_gyrox = 0.0
m_gyroy = 0.0
m_gyroz = 0.0
last_sensor_us = None
sensor_dt_samples_ms = []


# =========================================================
# BLE
ble_comm = BLECommunication()
ble_comm.advertise()

gc.collect()
led = Pin("LED", Pin.OUT)
led.off()

print()
print("================================")
print("PICO 2W INTEGRATED START")
print("================================")


ZERO_DEBOUNCE_MS = 300
last_switch = zero_switch_pin.value()
last_zero_time = utime.ticks_ms()


# =========================================================
# MOTOR
# =========================================================
print("MOTOR INIT")
try:
    cw_pin.drive(3)
    ccw_pin.drive(3)
    sleep_pin.drive(3)
    print("GPIO DRIVE = 12mA")
except Exception as e:
    print("GPIO DRIVE SET NOT AVAILABLE")
    print("Drive setting skipped:", e)


cw_pwm = PWM(cw_pin)
ccw_pwm = PWM(ccw_pin)

cw_pwm.freq(Config.MOTOR_PWM_FREQ)
ccw_pwm.freq(Config.MOTOR_PWM_FREQ)


def send_config():
    ble_comm.send_text(
        (
            "CONFIG,"
            "PWM_MAX,{:.1f},"
            "PWM_MIN,{:.1f},"
            "ANG_MAX,{:.1f},"
            "ANG_MIN,{:.1f},"
            "NEUTRAL,{:.1f},"
            "GAIN,{:.1f}"
        ).format(
            Config.PWM_MAX,
            Config.PWM_MIN,
            Config.ANGLE_MAX,
            Config.ANGLE_MIN,
            Config.NEUTRAL_ANG,
            Config.GAIN,
        )
    )


def send_status():
    motor = "ENABLE" if state.motor_enabled else "STOP"
    sensor = "OK" if state.sensor_ok else "NG"
    ble_comm.send_text(
        ("STATUS," "{:.1f}," "{}," "{}," "{:.1f}," "{}," "{}").format(
            state.angle, state.joystick, state.switch_pressed, 0.0, motor, sensor
        )
    )


def process_command(command):
    command = command.strip()
    if not command:
        return
    print("BLE CMD:", command)

    parts = command.split(",")
    key = parts[0].strip().upper()
    if key == "SET" and len(parts) >= 3:
        name = parts[1].strip().upper()

        try:
            value = float(parts[2].strip())
        except Exception:
            ble_comm.send_text("ERROR,INVALID_VALUE")
            return

        if name == "PWM_MAX":
            Config.PWM_MAX = clamp(value, 0, 100)
        elif name == "PWM_MIN":
            Config.PWM_MIN = clamp(value, -100, 0)
        elif name == "ANG_MAX":
            Config.ANGLE_MAX = clamp(value, 0.1, 180)
        elif name == "ANG_MIN":
            Config.ANGLE_MIN = clamp(value, -180, -0.1)
        elif name == "NEUTRAL":
            Config.NEUTRAL_ANG = clamp(value, -180, 180)
        else:
            ble_comm.send_text("ERROR,UNKNOWN_SETTING,{}".format(name))
            return

        ble_comm.send_text("OK,SET,{},{}".format(name, value))
        return

    if key == "GET" and len(parts) >= 2 and parts[1].strip().upper() == "CONFIG":
        send_config()
        return

    if key == "GET" and len(parts) >= 2 and parts[1].strip().upper() == "STATUS":
        send_status()
        return

    if key == "MOTOR" and len(parts) >= 2:
        motor_command = parts[1].strip().upper()
        if motor_command == "STOP":
            state.motor_enabled = False
            ble_comm.send_text("OK,MOTOR,STOP")
            return
        if motor_command == "ENABLE":
            state.motor_enabled = True
            ble_comm.send_text("OK,MOTOR,ENABLE")
            return

    if key == "SAVE":
        ble_comm.send_text("OK,SAVE")
        return

    if key == "PING":
        ble_comm.send_text("PONG")
        return

    ble_comm.send_text("ERROR,UNKNOWN_COMMAND")


async def command_task():
    while True:
        command = ble_comm.get_command()
        if command is not None:
            try:
                process_command(command)
            except Exception as e:
                print("command error:", e)
                ble_comm.send_text("ERROR,COMMAND")

        await asyncio.sleep_ms(1000)


async def ble_send_task():
    while True:
        motor = "ENABLE" if state.motor_enabled else "STOP"
        sensor = "OK" if state.sensor_ok else "NG"
        msg = ("DATA," "{:.1f}," "{}," "{}," "{:.1f}," "{}," "{}").format(
            state.angle, state.joystick, state.switch_pressed, 0.0, motor, sensor
        )
        if ble_comm.is_connected():
            ble_comm.send_text(msg)
        await asyncio.sleep_ms(Config.BLE_INTERVAL_MS)


def pwm_duty_percent(percent):
    percent = clamp(percent, 0.0, 100.0)
    return int(percent * 65535.0 / 100.0)


def motor_stop():
    cw_pwm.duty_u16(0)
    ccw_pwm.duty_u16(0)
    state.motor_state = "STOP"


def motor_sleep():
    motor_stop()
    sleep_pin.value(0)
    state.motor_enabled = False
    state.motor_state = "SLEEP"


def motor_enable():
    sleep_pin.value(1)
    state.motor_enabled = True


def motor_cw(percent):
    percent = abs(clamp(percent, 0.0, 100.0))
    ccw_pwm.duty_u16(0)
    cw_pwm.duty_u16(pwm_duty_percent(percent))
    state.motor_state = "CW"


def motor_ccw(percent):
    percent = abs(clamp(percent, 0.0, 100.0))
    cw_pwm.duty_u16(0)
    ccw_pwm.duty_u16(pwm_duty_percent(percent))
    state.motor_state = "CCW"


# Safe initial condition
motor_sleep()


async def switch_task():

    global last_switch
    global last_zero_time

    while True:
        switch_now = zero_switch_pin.value()
        state.switch_pressed = 1 if switch_now == 0 else 0

        # Falling edge = zero set.
        if last_switch == 1 and switch_now == 0:
            now = utime.ticks_ms()
            if utime.ticks_diff(now, last_zero_time) >= ZERO_DEBOUNCE_MS:
                state.neutral = -state.lpf2_angle
                state.zeroed = True
                print()
                print("==========================")
                print("ZERO SET")
                print("NEUTRAL = {:+.3f}".format(state.neutral))
                print("ANGLE = 0.000")
                print("==========================")
                last_zero_time = now

        last_switch = switch_now
        await asyncio.sleep_ms(10)


async def control_task():
    while True:
        try:
            m_gyro2.tx_GET_ACCGYRO()
            if state.menu_mode:
                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_stop()
                motor_sleep()
                await asyncio.sleep_ms(Config.CONTROL_INTERVAL_MS)
                continue

            if not state.sensor_ok:
                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_sleep()
                await asyncio.sleep_ms(Config.CONTROL_INTERVAL_MS)
                continue

            if Config.MOTOR_REQUIRE_ZERO and not state.zeroed:
                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_sleep()
                await asyncio.sleep_ms(Config.CONTROL_INTERVAL_MS)
                continue

            if not state.switch_pressed:
                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_sleep()
                await asyncio.sleep_ms(Config.CONTROL_INTERVAL_MS)
                continue

            if state.switch_pressed:
                ble_comm.send_text("PONG")

            com = ble_comm.get_command()
            print(com)

            motor_enable()

            # -------------------------------------------------
            # Angle -> PWM
            # -------------------------------------------------
            angle_clamped = clamp(state.angle, Config.ANGLE_MIN, Config.ANGLE_MAX)

            if angle_clamped >= 0:
                if Config.ANGLE_MAX > 0:
                    normalized = angle_clamped / float(Config.ANGLE_MAX)
                else:
                    normalized = 0.0
                state.target_pwm_command = normalized * Config.PWM_MAX
            else:
                if Config.ANGLE_MIN < 0:
                    normalized = angle_clamped / abs(float(Config.ANGLE_MIN))
                else:
                    normalized = 0.0
                state.target_pwm_command = normalized * abs(Config.PWM_MIN)

            # -------------------------------------------------
            # PWM lag
            # -------------------------------------------------

            state.current_pwm_command += (
                state.target_pwm_command - state.current_pwm_command
            ) * 1.0

            duty = state.current_pwm_command

            # -------------------------------------------------
            # Deadband
            # -------------------------------------------------

            if abs(duty) < Config.PWM_DEADBAND:
                motor_stop()
            elif duty > 0:
                motor_ccw(abs(duty))
            else:
                motor_cw(abs(duty))

        except Exception as e:

            print("CONTROL ERROR:", e)

            state.target_pwm_command = 0.0
            state.current_pwm_command = 0.0
            motor_sleep()

        await asyncio.sleep_ms(Config.CONTROL_INTERVAL_MS)


async def monitor_task():

    while True:

        if state.sensor_ok:

            age = utime.ticks_diff(utime.ticks_ms(), state.last_sensor_time)

            if age > Config.SENSOR_HOLD_TIMEOUT_MS:

                print("SENSOR TIMEOUT - MOTOR STOP")
                state.sensor_ok = False
                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_sleep()

        gc.collect()

        await asyncio.sleep_ms(1000)


def bno055_rx_chip_id(_chip_id):
    global m_chip_id
    m_chip_id = _chip_id
    print("BNO055 chip id = 0x{:02X}".format(m_chip_id))


def bno055_rx_gyro(_accx, _accy, _accz, _gyrox, _gyroy, _gyroz):
    global last_sensor_us
    global m_accx, m_accy, m_accz
    global m_gyrox, m_gyroy, m_gyroz
    m_accx = _accx
    m_accy = _accy
    m_accz = _accz
    m_gyrox = _gyrox * (180.0 / math.pi)
    m_gyroy = _gyroy * (180.0 / math.pi)
    m_gyroz = _gyroz * (180.0 / math.pi)

    now_us = utime.ticks_us()
    if last_sensor_us is None:
        dt = Config.SENSOR_INTERVAL_MS / 1000.0
    else:
        measured_dt_us = utime.ticks_diff(now_us, last_sensor_us)
        measured_dt_ms = measured_dt_us / 1000.0
        sensor_dt_samples_ms.append(measured_dt_ms)
        if len(sensor_dt_samples_ms) > 10:
            sensor_dt_samples_ms.pop(0)
        average_dt_ms = sum(sensor_dt_samples_ms) / len(sensor_dt_samples_ms)
        dt = measured_dt_us / 1000000.0
        dt = clamp(dt, 0.001, 0.1)
    last_sensor_us = now_us

    sensor = (m_accx, m_accy, m_accz, m_gyrox, m_gyroy, m_gyroz)
    # try:
    update_angle(sensor, dt)
    print(
        "dt = {:.1f}, dt_ave = {:.1f} ms, ang = {:.1f} deg".format(
            measured_dt_ms,
            average_dt_ms,
            state.angle,
        )
    )
    state.sensor_error_count = 0
    state.sensor_ok = True
    state.last_sensor_time = utime.ticks_ms()
    # except Exception as e:
    #    state.sensor_error_count += 1
    #    state.total_sensor_errors += 1
    #    print("BNO055 UPDATE ERROR:", e)


async def check_bno055_chip_id(timeout_ms=500):
    """Request and validate the BNO055 chip ID with a bounded wait."""
    global m_chip_id
    m_chip_id = None
    m_gyro2.tx_CHIP_ID()
    started = utime.ticks_ms()
    while m_chip_id is None:
        if utime.ticks_diff(utime.ticks_ms(), started) > timeout_ms:
            return False
        await asyncio.sleep_ms(10)

    return m_chip_id == 0xA0


async def main():

    print("MAIN START")

    motor_sleep()
    load_config()

    # -----------------------------------------------------
    # MENU SAFE START
    # -----------------------------------------------------
    state.menu_mode = False
    state.menu_level = MENU_MAIN
    state.menu_index = 0
    state.last_joy_state = "NONE"

    lcd_print("INIT", 0)
    lcd_print("BNO055", 1)

    # -----------------------------------------------------
    # BNO055
    # -----------------------------------------------------
    delay = 10
    m_gyro2.attach_rx_callback(bno055_rx_gyro, bno055_rx_chip_id)
    if not await check_bno055_chip_id():
        lcd_print("BNO ERR", 0)
        lcd_print("CHECK", 1)
        while True:
            motor_sleep()
            await asyncio.sleep(1)

    m_gyro2.tx_OPR_MODE()
    await asyncio.sleep_ms(delay)
    await asyncio.sleep_ms(1500)
    if not await check_bno055_chip_id():

        lcd_print("BNO ERR", 0)
        lcd_print("CHECK", 1)

        while True:
            motor_sleep()
            await asyncio.sleep(1)

    # -----------------------------------------------------
    # Tasks
    # -----------------------------------------------------

    asyncio.create_task(switch_task())
    asyncio.create_task(joystick_task())
    asyncio.create_task(display_task())
    asyncio.create_task(control_task())
    asyncio.create_task(monitor_task())
    asyncio.create_task(command_task())
    asyncio.create_task(ble_send_task())

    print()
    print("================================")
    print("SYSTEM READY")
    print("================================")
    print("BNO055 : WIRED UART")
    print("LCD    : GP0/GP1")
    print("SW     : GP6")
    print("MOTOR  : GP16/GP17/GP18")
    print("JOY    : GP26")
    print("PWM    : 5kHz")
    print("================================")

    if Config.MOTOR_REQUIRE_ZERO:
        print("MOTOR WAITING FOR ZERO SWITCH")

    print("MOTOR RUN ONLY WHILE GP6 BUTTON IS PRESSED")
    print("SENSOR READ ERROR -> KEEP LAST VALID ANGLE")

    while True:
        await asyncio.sleep(1)


try:
    asyncio.run(main())

except KeyboardInterrupt:

    print("STOP")
    motor_sleep()

except Exception as e:

    print("FATAL ERROR:", e)
    motor_sleep()
