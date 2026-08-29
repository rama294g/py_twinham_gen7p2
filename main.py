# =========================================================
# RASPBERRY PI PICO 2W
#
# Integrated Control Program
#
# LCD + JOYSTICK + ZERO SWITCH + BNO055 UART + MOTOR
#
# GPIO
#   GP0  : LCD SDA
#   GP1  : LCD SCL
#   GP4  : BNO055 UART TX
#   GP5  : BNO055 UART RX
#   GP6  : ZERO SWITCH
#   GP16 : MOTOR CW PWM
#   GP17 : MOTOR CCW PWM
#   GP18 : MOTOR nSleep
#   GP26 : JOYSTICK
#
# BNO055
#   UART 115200bps
#   NDOF mode
#   ACC + GYRO state estimation
#
# Motor
#   PWM = 5kHz
#   GPIO drive = 12mA (when supported)
#
# NOTE
#   BLE is intentionally removed.
#   Angle data now comes directly from the wired BNO055.
# =========================================================

import utime
import gc
import ujson
import uasyncio as asyncio
import struct
import math
from machine import I2C, Pin, PWM, ADC, UART


gc.collect()

print()
print("================================")
print("PICO 2W INTEGRATED START")
print("================================")


# =========================================================
# CONFIG
# =========================================================

class Config:

    CONFIG_FILE = "config.json"

    # Angle range used for motor mapping
    ANGLE_MIN = -90
    ANGLE_MAX = 90

    # Motor PWM command range
    PWM_MIN = -30
    PWM_MAX = 30
    PWM_DEADBAND = 3

    # Motor PWM
    MOTOR_PWM_FREQ = 5000

    # Control cycle
    CONTROL_INTERVAL_MS = 20

    # Sensor cycle
    # センサー更新周期：20ms = 50Hz
    SENSOR_INTERVAL_MS = 20

    # UART
    UART_TIMEOUT_MS = 50

    # Sensor communication timeout before forced motor stop
    SENSOR_HOLD_TIMEOUT_MS = 500

    # Angle estimation
    Q1_DEG = 0.0
    SW_IS_RIGHT = False
    KALMAN_GAIN = 0.01
    LPF_TAU = 0.01

    # Motor safety
    MOTOR_REQUIRE_ZERO = True


# =========================================================
# STATE
# =========================================================

class SystemState:

    angle = 0.0
    observed_angle = 0.0
    kalman_angle = 0.0
    lpf_angle = 0.0
    gyro_angle_rate = 0.0
    gyro_z = 0.0
    dt = 0.0

    neutral = 0.0
    zeroed = False

    sensor_ok = False
    sensor_error_count = 0
    total_sensor_errors = 0
    last_sensor_time = 0

    joystick = "NONE"
    joystick_value = 0

    switch_pressed = 0

    target_pwm_command = 0.0
    current_pwm_command = 0.0

    motor_enabled = False
    motor_state = "STOP"

    # Menu
    menu_mode = False
    menu_level = 0
    menu_index = 0
    last_joy_state = "NONE"

    line1_setting = 0
    line2_setting = 1
    last_line1 = ""
    last_line2 = ""


state = SystemState()


# =========================================================
# MENU CONSTANTS
# =========================================================

MENU_MAIN = 0
MENU_DISPLAY = 1
MENU_SETTING = 2
MENU_EDIT_LINE1 = 3
MENU_EDIT_LINE2 = 4
MENU_EDIT_PWM_MAX = 10
MENU_EDIT_PWM_MIN = 11
MENU_EDIT_ANG_MAX = 12
MENU_EDIT_ANG_MIN = 13

main_menu_items = [
    "SETTING",
    "DISPLAY",
    "RETURN"
]

display_menu_items = [
    "1stLine",
    "2ndLine",
    "RETURN"
]

display_value_items = [
    "ANGLE",
    "PWM",
    "SENSOR",
    "SWITCH",
    "ZERO",
    "JOY",
    "OBS",
    "KAL",
    "LPF",
    "GYRO",
    "ERROR"
]


# =========================================================
# UTIL
# =========================================================

def clamp(value, minimum, maximum):

    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def fitin360(angle):

    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


# =========================================================
# CONFIG SAVE / LOAD
# =========================================================

def save_config():

    try:

        cfg = {
            "PWM_MAX": Config.PWM_MAX,
            "PWM_MIN": Config.PWM_MIN,
            "ANGLE_MAX": Config.ANGLE_MAX,
            "ANGLE_MIN": Config.ANGLE_MIN,
            "line1_setting": state.line1_setting,
            "line2_setting": state.line2_setting,
        }

        with open(Config.CONFIG_FILE, "w") as f:
            ujson.dump(cfg, f)

        print("Config saved")

    except Exception as e:
        print("save_config error:", e)


def load_config():

    # ---------------------------------------------------------
    # 設定ファイルは壊れていても起動できるようにします。
    # 特に1stLine/2ndLineの番号は必ず範囲内へ補正します。
    # ---------------------------------------------------------
    try:
        with open(Config.CONFIG_FILE, "r") as f:
            cfg = ujson.load(f)

        Config.PWM_MAX = clamp(
            int(cfg.get("PWM_MAX", 30)), 1, 100
        )
        Config.PWM_MIN = clamp(
            int(cfg.get("PWM_MIN", -30)), -100, -1
        )
        Config.ANGLE_MAX = clamp(
            int(cfg.get("ANGLE_MAX", 90)), 1, 180
        )
        Config.ANGLE_MIN = clamp(
            int(cfg.get("ANGLE_MIN", -90)), -180, -1
        )

        # -----------------------------------------------------
        # DISPLAY設定を厳密に検証
        # -----------------------------------------------------
        line1 = int(cfg.get("line1_setting", 0))
        line2 = int(cfg.get("line2_setting", 1))

        state.line1_setting = clamp(
            line1, 0, len(display_value_items) - 1
        )
        state.line2_setting = clamp(
            line2, 0, len(display_value_items) - 1
        )

        print("Config loaded")
        print("DISPLAY 1stLine =", state.line1_setting)
        print("DISPLAY 2ndLine =", state.line2_setting)

        # -----------------------------------------------------
        # 不正値があった場合は修正値を保存
        # -----------------------------------------------------
        if (
            line1 != state.line1_setting or
            line2 != state.line2_setting
        ):
            print("DISPLAY CONFIG CORRECTED")
            save_config()

    except Exception as e:
        print("Config load error:", e)
        print("Default config")

        Config.PWM_MAX = 30
        Config.PWM_MIN = -30
        Config.ANGLE_MAX = 90
        Config.ANGLE_MIN = -90
        state.line1_setting = 0
        state.line2_setting = 1


# =========================================================
# LED
# =========================================================

led = Pin("LED", Pin.OUT)
led.off()


# =========================================================
# LCD
# =========================================================

LCD_I2C_ID = 0
LCD_SDA_PIN = 0
LCD_SCL_PIN = 1
LCD_I2C_FREQ = 100000
LCD_ADDR = 0x3E

print("LCD INIT")

lcd_i2c = I2C(
    LCD_I2C_ID,
    sda=Pin(LCD_SDA_PIN),
    scl=Pin(LCD_SCL_PIN),
    freq=LCD_I2C_FREQ
)

print("I2C devices:", lcd_i2c.scan())


def lcd_cmd(cmd):

    try:

        lcd_i2c.writeto(
            LCD_ADDR,
            bytearray([0x00, cmd])
        )

        utime.sleep_ms(1)

    except Exception as e:
        print("LCD CMD error:", e)


def lcd_data(value):

    try:

        lcd_i2c.writeto(
            LCD_ADDR,
            bytearray([0x40, value])
        )

    except Exception as e:
        print("LCD DATA error:", e)


def lcd_init():

    utime.sleep_ms(100)

    for cmd in [
        0x38,
        0x39,
        0x14,
        0x70,
        0x56,
        0x6C,
    ]:
        lcd_cmd(cmd)

    utime.sleep_ms(200)

    for cmd in [
        0x38,
        0x0C,
        0x01,
        0x06,
    ]:
        lcd_cmd(cmd)

    utime.sleep_ms(2)


def lcd_print(text, line=0):

    text = str(text)
    text = (text + "        ")[:8]

    lcd_cmd(
        0x80 if line == 0 else 0xC0
    )

    for ch in text:
        lcd_data(ord(ch))


lcd_init()
lcd_print("PICO 2W", 0)
lcd_print("START", 1)


# =========================================================
# JOYSTICK
# =========================================================

JOYSTICK_PIN = 26
joy_adc = ADC(JOYSTICK_PIN)

# Actual measured values:
# NONE/CENTER ~ 50
# UP          ~ 12000
# RIGHT       ~ 26000
# DOWN        ~ 42000
# LEFT        ~ 52000

JOY_CENTER_MAX = 5000
JOY_UP_MIN = 5001
JOY_UP_MAX = 19000
JOY_RIGHT_MIN = 19001
JOY_RIGHT_MAX = 34000
JOY_DOWN_MIN = 34001
JOY_DOWN_MAX = 47000
JOY_LEFT_MIN = 47001
JOY_LEFT_MAX = 60000


def read_joystick():

    value = joy_adc.read_u16()

    if value <= JOY_CENTER_MAX:
        return "CENTER", value

    if JOY_UP_MIN <= value <= JOY_UP_MAX:
        return "UP", value

    if JOY_RIGHT_MIN <= value <= JOY_RIGHT_MAX:
        return "RIGHT", value

    if JOY_DOWN_MIN <= value <= JOY_DOWN_MAX:
        return "DOWN", value

    if JOY_LEFT_MIN <= value <= JOY_LEFT_MAX:
        return "LEFT", value

    return "NONE", value


# =========================================================
# ZERO SWITCH
# =========================================================

ZERO_SWITCH_PIN = 6
zero_switch = Pin(
    ZERO_SWITCH_PIN,
    Pin.IN,
    Pin.PULL_UP
)

ZERO_DEBOUNCE_MS = 300
last_switch = zero_switch.value()
last_zero_time = utime.ticks_ms()


# =========================================================
# MOTOR
# =========================================================

MOTOR_CW_PIN = 16
MOTOR_CCW_PIN = 17
MOTOR_SLEEP_PIN = 18

print("MOTOR INIT")

cw_pin = Pin(MOTOR_CW_PIN, Pin.OUT)
ccw_pin = Pin(MOTOR_CCW_PIN, Pin.OUT)
sleep_pin = Pin(MOTOR_SLEEP_PIN, Pin.OUT)

# RP2350 MicroPython versions may expose drive() differently.
# drive(3) corresponds to the 12mA setting used in the verified
# motor test program.
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


def pwm_duty_percent(percent):

    percent = clamp(
        percent,
        0.0,
        100.0
    )

    return int(
        percent * 65535.0 / 100.0
    )


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

    percent = abs(
        clamp(percent, 0.0, 100.0)
    )

    ccw_pwm.duty_u16(0)
    cw_pwm.duty_u16(
        pwm_duty_percent(percent)
    )

    state.motor_state = "CW"


def motor_ccw(percent):

    percent = abs(
        clamp(percent, 0.0, 100.0)
    )

    cw_pwm.duty_u16(0)
    ccw_pwm.duty_u16(
        pwm_duty_percent(percent)
    )

    state.motor_state = "CCW"


# Safe initial condition
motor_sleep()


# =========================================================
# BNO055 UART
# =========================================================

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


def uart_clear():

    while uart.any():
        uart.read()


def bno_read(reg, length):

    uart_clear()

    uart.write(bytes([
        0xAA,
        0x01,
        reg,
        length
    ]))

    expected = 2 + length
    data = bytearray()
    start = utime.ticks_ms()

    while len(data) < expected:

        if uart.any():

            chunk = uart.read()

            if chunk:
                data.extend(chunk)

        else:

            if utime.ticks_diff(
                utime.ticks_ms(),
                start
            ) > Config.UART_TIMEOUT_MS:
                return None

            utime.sleep_ms(1)

    # Search for BB to re-synchronize if extra bytes arrived.
    start_index = -1

    for i in range(len(data)):

        if data[i] == 0xBB:
            start_index = i
            break

    if start_index < 0:
        return None

    if len(data) - start_index < 2:
        return None

    rx_length = data[start_index + 1]

    if rx_length != length:
        return None

    end_index = start_index + 2 + length

    if len(data) < end_index:
        return None

    return bytes(
        data[start_index + 2:end_index]
    )


def bno_write(reg, value):

    uart_clear()

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

    data = bno_read(
        REG_SENSOR_DATA,
        SENSOR_DATA_LENGTH
    )

    if data is None or len(data) != 18:
        return None

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
        return None

    return (
        acc_x_raw / ACC_SCALE,
        acc_y_raw / ACC_SCALE,
        acc_z_raw / ACC_SCALE,
        gyro_x_raw / GYRO_SCALE,
        gyro_y_raw / GYRO_SCALE,
        gyro_z_raw / GYRO_SCALE
    )


# =========================================================
# ANGLE ESTIMATION
# =========================================================

Q1_RAD = math.radians(Config.Q1_DEG)
COS_Q1 = math.cos(Q1_RAD)
SIN_Q1 = math.sin(Q1_RAD)

if Config.SW_IS_RIGHT:
    SIGN_LR = -1.0
else:
    SIGN_LR = 1.0


def initialize_angle(sensor):

    m_accx = sensor[0]
    m_accy = sensor[1]
    m_accz = sensor[2]

    new_accx = m_accx

    new_accy = SIGN_LR * (
        COS_Q1 * m_accy +
        SIN_Q1 * m_accz
    )

    new_accz = SIGN_LR * (
        COS_Q1 * m_accz -
        SIN_Q1 * m_accy
    )

    angle = math.degrees(
        math.atan2(
            -new_accy,
            -new_accx
        )
    )

    if Config.SW_IS_RIGHT:
        angle -= 90.0
    else:
        angle += 90.0

    return fitin360(angle)


def update_angle(sensor, dt):

    m_accx = sensor[0]
    m_accy = sensor[1]
    m_accz = sensor[2]

    m_gyrox = sensor[3]
    m_gyroy = sensor[4]
    m_gyroz = sensor[5]

    # -----------------------------------------------------
    # Coordinate transform
    # -----------------------------------------------------

    new_accx = m_accx

    new_accy = SIGN_LR * (
        COS_Q1 * m_accy +
        SIN_Q1 * m_accz
    )

    new_accz = SIGN_LR * (
        COS_Q1 * m_accz -
        SIN_Q1 * m_accy
    )

    new_gyrox = m_gyrox

    new_gyroy = SIGN_LR * (
        COS_Q1 * m_gyroy +
        SIN_Q1 * m_gyroz
    )

    new_gyroz = SIGN_LR * (
        COS_Q1 * m_gyroz -
        SIN_Q1 * m_gyroy
    )

    # -----------------------------------------------------
    # Acceleration observation
    # -----------------------------------------------------

    tilt_observe = math.degrees(
        math.atan2(
            -new_accy,
            -new_accx
        )
    )

    if Config.SW_IS_RIGHT:
        tilt_observe -= 90.0
    else:
        tilt_observe += 90.0

    tilt_observe = fitin360(
        tilt_observe
    )

    # -----------------------------------------------------
    # Gyro prediction
    # -----------------------------------------------------

    tilt_priest = (
        state.kalman_angle -
        new_gyroz * dt
    )

    tilt_delta = (
        tilt_observe -
        tilt_priest
    )

    tilt_delta = fitin360(
        tilt_delta
    )

    state.kalman_angle = (
        Config.KALMAN_GAIN * tilt_delta +
        tilt_priest
    )

    state.kalman_angle = fitin360(
        state.kalman_angle
    )

    state.kalman_angle = clamp(
        state.kalman_angle,
        -90.0,
        90.0
    )

    # -----------------------------------------------------
    # LPF 1
    # -----------------------------------------------------

    state.lpf_angle = (
        dt * state.kalman_angle +
        Config.LPF_TAU * state.lpf_angle
    ) / (
        Config.LPF_TAU + dt
    )

    # -----------------------------------------------------
    # LPF 2
    # -----------------------------------------------------

    state.lpf_angle = (
        dt * state.lpf_angle +
        Config.LPF_TAU * state.lpf_angle
    ) / (
        Config.LPF_TAU + dt
    )

    # The two-stage LPF above needs separate states. Keep a
    # second state in the function object for MicroPython simplicity.
    # This block is replaced below by the explicit state variable.

    state.observed_angle = tilt_observe

    state.angle = (
        -state.lpf2_angle -
        state.neutral
    ) if hasattr(state, "lpf2_angle") else (
        -state.lpf_angle -
        state.neutral
    )

    state.angle = clamp(
        state.angle,
        -90.0,
        90.0
    )


# Explicit second LPF state.
state.lpf2_angle = 0.0


def update_angle(sensor, dt):

    m_accx = sensor[0]
    m_accy = sensor[1]
    m_accz = sensor[2]

    m_gyroy = sensor[4]
    m_gyroz = sensor[5]

    state.gyro_z = m_gyroz
    state.dt = dt

    new_accx = m_accx

    new_accy = SIGN_LR * (
        COS_Q1 * m_accy +
        SIN_Q1 * m_accz
    )

    new_accz = SIGN_LR * (
        COS_Q1 * m_accz -
        SIN_Q1 * m_accy
    )

    new_gyroy = SIGN_LR * (
        COS_Q1 * m_gyroy +
        SIN_Q1 * m_gyroz
    )

    new_gyroz = SIGN_LR * (
        COS_Q1 * m_gyroz -
        SIN_Q1 * m_gyroy
    )

    state.gyro_angle_rate = -new_gyroz

    tilt_observe = math.degrees(
        math.atan2(
            -new_accy,
            -new_accx
        )
    )

    if Config.SW_IS_RIGHT:
        tilt_observe -= 90.0
    else:
        tilt_observe += 90.0

    tilt_observe = fitin360(tilt_observe)

    tilt_priest = (
        state.kalman_angle -
        new_gyroz * dt
    )

    tilt_delta = fitin360(
        tilt_observe - tilt_priest
    )

    state.kalman_angle = (
        Config.KALMAN_GAIN * tilt_delta +
        tilt_priest
    )

    state.kalman_angle = fitin360(
        state.kalman_angle
    )

    state.kalman_angle = clamp(
        state.kalman_angle,
        -90.0,
        90.0
    )

    state.lpf1_angle = (
        dt * state.kalman_angle +
        Config.LPF_TAU * state.lpf1_angle
    ) / (
        Config.LPF_TAU + dt
    )

    state.lpf2_angle = (
        dt * state.lpf1_angle +
        Config.LPF_TAU * state.lpf2_angle
    ) / (
        Config.LPF_TAU + dt
    )

    state.observed_angle = tilt_observe

    state.angle = (
        -state.lpf2_angle -
        state.neutral
    )

    state.angle = clamp(
        state.angle,
        -90.0,
        90.0
    )


# Initialize filter state attributes.
state.lpf1_angle = 0.0
state.lpf2_angle = 0.0


# =========================================================
# BNO INITIALIZATION
# =========================================================

def bno_init():

    print()
    print("CHECK CHIP ID")

    chip_id = bno_read(
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

    mode = bno_read(
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


# =========================================================
# DISPLAY DATA
# =========================================================

def get_line(setting):

    # 設定ファイルが壊れていても配列外アクセスを起こさない。
    try:
        setting = int(setting)
    except Exception:
        setting = 0

    setting = clamp(
        setting,
        0,
        len(display_value_items) - 1
    )

    if setting == 0:
        return "ANG:{:+.1f}".format(state.angle)

    if setting == 1:
        return "PWM:{:+.0f}".format(
            state.current_pwm_command
        )

    if setting == 2:
        return (
            "SNS:OK" if state.sensor_ok
            else "SNS:NG"
        )

    if setting == 3:
        return (
            "SW:ON" if state.switch_pressed
            else "SW:OFF"
        )

    if setting == 4:
        return "Z:{:+.1f}".format(
            state.neutral
        )

    if setting == 5:
        return "JOY:" + state.joystick

    if setting == 6:
        return "OBS:{:+.1f}".format(
            state.observed_angle
        )

    if setting == 7:
        return "KAL:{:+.1f}".format(
            state.kalman_angle
        )

    if setting == 8:
        return "LPF:{:+.1f}".format(
            state.lpf2_angle
        )

    if setting == 9:
        return "GYR:{:+.1f}".format(
            state.gyro_z
        )

    if setting == 10:
        return "ERR:" + str(
            state.total_sensor_errors
        )

    return "--------"


# =========================================================
# MENU MANAGER
# =========================================================

class MenuManager:

    def __init__(self):

        self.last_input_time = 0
        self.debounce_ms = 150

    def get_input(self):

        joy = state.joystick
        now = utime.ticks_ms()

        if utime.ticks_diff(
            now,
            self.last_input_time
        ) < self.debounce_ms:
            return "NONE"

        if joy == state.last_joy_state:
            return "NONE"

        state.last_joy_state = joy

        if joy != "NONE":
            self.last_input_time = now

        return joy

    def update(self):

        joy = self.get_input()

        if joy == "NONE":
            return

        # Center enters menu.
        if not state.menu_mode:

            if joy == "CENTER":

                state.menu_mode = True
                state.menu_level = MENU_MAIN
                state.menu_index = 0

            return

        # -------------------------------------------------
        # Edit values
        # -------------------------------------------------

        if state.menu_level == MENU_EDIT_PWM_MAX:

            if joy == "UP":
                Config.PWM_MAX += 1
            elif joy == "DOWN":
                Config.PWM_MAX -= 1
            elif joy == "RIGHT":
                Config.PWM_MAX += 5
            elif joy == "LEFT":
                Config.PWM_MAX -= 5
            elif joy == "CENTER":
                Config.PWM_MAX = clamp(
                    Config.PWM_MAX, 1, 100
                )
                save_config()
                state.menu_level = MENU_SETTING

            Config.PWM_MAX = clamp(
                Config.PWM_MAX, 1, 100
            )
            return

        if state.menu_level == MENU_EDIT_PWM_MIN:

            if joy == "UP":
                Config.PWM_MIN += 1
            elif joy == "DOWN":
                Config.PWM_MIN -= 1
            elif joy == "RIGHT":
                Config.PWM_MIN += 5
            elif joy == "LEFT":
                Config.PWM_MIN -= 5
            elif joy == "CENTER":
                Config.PWM_MIN = clamp(
                    Config.PWM_MIN, -100, -1
                )
                save_config()
                state.menu_level = MENU_SETTING

            Config.PWM_MIN = clamp(
                Config.PWM_MIN, -100, -1
            )
            return

        if state.menu_level == MENU_EDIT_ANG_MAX:

            if joy == "UP":
                Config.ANGLE_MAX += 1
            elif joy == "DOWN":
                Config.ANGLE_MAX -= 1
            elif joy == "RIGHT":
                Config.ANGLE_MAX += 5
            elif joy == "LEFT":
                Config.ANGLE_MAX -= 5
            elif joy == "CENTER":
                Config.ANGLE_MAX = clamp(
                    Config.ANGLE_MAX, 1, 180
                )
                save_config()
                state.menu_level = MENU_SETTING

            Config.ANGLE_MAX = clamp(
                Config.ANGLE_MAX, 1, 180
            )
            return

        if state.menu_level == MENU_EDIT_ANG_MIN:

            if joy == "UP":
                Config.ANGLE_MIN += 1
            elif joy == "DOWN":
                Config.ANGLE_MIN -= 1
            elif joy == "RIGHT":
                Config.ANGLE_MIN += 5
            elif joy == "LEFT":
                Config.ANGLE_MIN -= 5
            elif joy == "CENTER":
                Config.ANGLE_MIN = clamp(
                    Config.ANGLE_MIN, -180, -1
                )
                save_config()
                state.menu_level = MENU_SETTING

            Config.ANGLE_MIN = clamp(
                Config.ANGLE_MIN, -180, -1
            )
            return

        # -------------------------------------------------
        # Current menu
        # -------------------------------------------------

        if state.menu_level == MENU_MAIN:
            current_menu = main_menu_items

        elif state.menu_level == MENU_DISPLAY:
            current_menu = display_menu_items

        elif state.menu_level == MENU_SETTING:
            current_menu = [
                "PWM_MAX",
                "PWM_MIN",
                "ANG_MAX",
                "ANG_MIN",
                "RESET",
                "RETURN"
            ]

        elif state.menu_level in (
            MENU_EDIT_LINE1,
            MENU_EDIT_LINE2
        ):
            current_menu = display_value_items

        else:
            current_menu = ["RETURN"]

        # -------------------------------------------------
        # Navigation
        # -------------------------------------------------
        # 設定値や再起動後の状態が不正でも、必ず範囲内に戻します。
        # -------------------------------------------------
        if not current_menu:
            state.menu_level = MENU_MAIN
            state.menu_index = 0
            return

        if (
            state.menu_index < 0 or
            state.menu_index >= len(current_menu)
        ):
            state.menu_index = 0

        if joy == "UP":
            state.menu_index -= 1

        elif joy == "DOWN":
            state.menu_index += 1

        if state.menu_index < 0:
            state.menu_index = len(current_menu) - 1

        if state.menu_index >= len(current_menu):
            state.menu_index = 0

        # -------------------------------------------------
        # LEFT = BACK / CANCEL
        # -------------------------------------------------
        # CENTERの押下/離し状態に依存せず、
        # 左操作でも一つ上のメニューへ戻れるようにする。
        if joy == "LEFT":

            if state.menu_level in (
                MENU_EDIT_LINE1,
                MENU_EDIT_LINE2,
                MENU_EDIT_PWM_MAX,
                MENU_EDIT_PWM_MIN,
                MENU_EDIT_ANG_MAX,
                MENU_EDIT_ANG_MIN
            ):
                state.menu_level = MENU_DISPLAY if state.menu_level in (MENU_EDIT_LINE1, MENU_EDIT_LINE2) else MENU_SETTING
                state.menu_index = 0
                return

            if state.menu_level == MENU_DISPLAY:
                state.menu_level = MENU_MAIN
                state.menu_index = 0
                return

            if state.menu_level == MENU_SETTING:
                state.menu_level = MENU_MAIN
                state.menu_index = 0
                return

            if state.menu_level == MENU_MAIN:
                state.menu_mode = False
                state.menu_level = MENU_MAIN
                state.menu_index = 0
                return

        # -------------------------------------------------
        # Select
        # -------------------------------------------------

        if joy != "CENTER":
            return

        state.menu_index = clamp(
            int(state.menu_index),
            0,
            len(current_menu) - 1
        )

        selected = current_menu[
            state.menu_index
        ]

        if state.menu_level == MENU_MAIN:

            if selected == "SETTING":
                state.menu_level = MENU_SETTING
                state.menu_index = 0

            elif selected == "DISPLAY":
                state.menu_level = MENU_DISPLAY
                state.menu_index = 0

            elif selected == "RETURN":
                state.menu_mode = False

        elif state.menu_level == MENU_DISPLAY:

            if selected == "1stLine":
                state.menu_level = MENU_EDIT_LINE1
                state.menu_index = clamp(
                    int(state.line1_setting),
                    0,
                    len(display_value_items) - 1
                )

            elif selected == "2ndLine":
                state.menu_level = MENU_EDIT_LINE2
                state.menu_index = clamp(
                    int(state.line2_setting),
                    0,
                    len(display_value_items) - 1
                )

            elif selected == "RETURN":
                state.menu_level = MENU_MAIN
                state.menu_index = 0

        elif state.menu_level == MENU_SETTING:

            if selected == "PWM_MAX":
                state.menu_level = MENU_EDIT_PWM_MAX

            elif selected == "PWM_MIN":
                state.menu_level = MENU_EDIT_PWM_MIN

            elif selected == "ANG_MAX":
                state.menu_level = MENU_EDIT_ANG_MAX

            elif selected == "ANG_MIN":
                state.menu_level = MENU_EDIT_ANG_MIN

            elif selected == "RESET":

                Config.PWM_MAX = 30
                Config.PWM_MIN = -30
                Config.ANGLE_MAX = 90
                Config.ANGLE_MIN = -90

                state.line1_setting = 0
                state.line2_setting = 1

                save_config()

            elif selected == "RETURN":
                state.menu_level = MENU_MAIN
                state.menu_index = 0

        elif state.menu_level == MENU_EDIT_LINE1:

            state.line1_setting = clamp(
                state.menu_index,
                0,
                len(display_value_items) - 1
            )

            save_config()
            state.menu_level = MENU_DISPLAY

        elif state.menu_level == MENU_EDIT_LINE2:

            state.line2_setting = clamp(
                state.menu_index,
                0,
                len(display_value_items) - 1
            )

            save_config()
            state.menu_level = MENU_DISPLAY


menu_manager = MenuManager()


# =========================================================
# SENSOR TASK
# =========================================================

async def sensor_task():

    # -----------------------------------------------------
    # Initial read
    # -----------------------------------------------------

    sensor = read_acc_gyro()

    if sensor is None:

        state.sensor_ok = False
        state.sensor_error_count = 1
        state.total_sensor_errors += 1

        print("INITIAL SENSOR READ ERROR")

    else:

        initial_angle = initialize_angle(sensor)

        state.observed_angle = initial_angle
        state.kalman_angle = initial_angle
        state.lpf1_angle = initial_angle
        state.lpf2_angle = initial_angle
        state.angle = -initial_angle
        state.sensor_ok = True
        state.last_sensor_time = utime.ticks_ms()

        print(
            "INITIAL ANGLE = {:+.3f}".format(
                state.angle
            )
        )

    # -----------------------------------------------------
    # 50Hz fixed-rate scheduler
    #
    # 20msごとを「次回実行時刻」で管理します。
    # 固定20ms sleepではなく、センサー読み出し・計算時間を
    # 20msから差し引くことで、可能な範囲で50Hzに近づけます。
    # -----------------------------------------------------
    SENSOR_PERIOD_US = Config.SENSOR_INTERVAL_MS * 1000
    next_sample_us = utime.ticks_add(
        utime.ticks_us(),
        SENSOR_PERIOD_US
    )

    last_us = utime.ticks_us()
    last_diag_ms = utime.ticks_ms()

    while True:

        # -------------------------------------------------
        # 次回サンプル時刻まで待つ
        # -------------------------------------------------
        now_us = utime.ticks_us()
        wait_us = utime.ticks_diff(
            next_sample_us,
            now_us
        )

        if wait_us > 0:
            # asyncioに制御を返しながら待つ
            await asyncio.sleep_ms(
                max(1, wait_us // 1000)
            )

        now_us = utime.ticks_us()

        dt = (
            utime.ticks_diff(
                now_us,
                last_us
            ) / 1000000.0
        )

        last_us = now_us

        # 次回時刻を20ms進める
        next_sample_us = utime.ticks_add(
            next_sample_us,
            SENSOR_PERIOD_US
        )

        # -------------------------------------------------
        # 長時間処理でスケジュールが遅れた場合
        # 一気に過去の予定を追いかけず、現在から20ms後を
        # 次回時刻にして周期を再同期します。
        # -------------------------------------------------
        if utime.ticks_diff(
            next_sample_us,
            now_us
        ) < -SENSOR_PERIOD_US:
            next_sample_us = utime.ticks_add(
                now_us,
                SENSOR_PERIOD_US
            )

        if dt <= 0.0:
            dt = 0.001

        if dt > 0.1:
            dt = 0.1

        sensor = read_acc_gyro()

        if sensor is None:

            # -------------------------------------------------
            # Keep the last valid sensor/angle value.
            # A transient UART read error must NOT overwrite
            # the current angle with an invalid value.
            # -------------------------------------------------
            state.sensor_error_count += 1
            state.total_sensor_errors += 1

            print(
                "SENSOR READ ERROR count={} total={} "
                "ANGLE HOLD={:+.1f}".format(
                    state.sensor_error_count,
                    state.total_sensor_errors,
                    state.angle
                )
            )

            # Keep sensor_ok=True while the last valid sample is
            # still within the hold timeout. This allows a short
            # communication glitch without changing the angle.
            age = utime.ticks_diff(
                utime.ticks_ms(),
                state.last_sensor_time
            )

            if (
                state.last_sensor_time == 0 or
                age > Config.SENSOR_HOLD_TIMEOUT_MS
            ):
                state.sensor_ok = False

            # Filter states and state.angle are intentionally
            # unchanged here.
            # エラー時も次回の20msスケジュールへ進む
            continue

        state.sensor_error_count = 0
        state.sensor_ok = True
        state.last_sensor_time = utime.ticks_ms()

        update_angle(
            sensor,
            dt
        )

        # -------------------------------------------------
        # ANGLE DIAGNOSTIC OUTPUT
        # 100 msごとに現在の各段階を表示
        # OBS  : 加速度から求めた観測角
        # GYR  : ジャイロ角速度
        # KAL  : ジャイロ予測＋観測補正後
        # LPF  : 2段LPF後
        # ANG  : 最終角度
        # -------------------------------------------------
        now_diag_ms = utime.ticks_ms()
        if utime.ticks_diff(now_diag_ms, last_diag_ms) >= 100:
            print(
                "DIAG "
                "OBS:{:+7.2f} "
                "GYR:{:+7.2f} "
                "KAL:{:+7.2f} "
                "LPF:{:+7.2f} "
                "ANG:{:+7.2f} "
                "dt:{:.4f} Hz:{:5.1f}".format(
                    state.observed_angle,
                    state.gyro_z,
                    state.kalman_angle,
                    state.lpf2_angle,
                    state.angle,
                    state.dt,
                    (1.0 / state.dt) if state.dt > 0.0 else 0.0
                )
            )
            last_diag_ms = now_diag_ms

        # -------------------------------------------------
        # ここではsleepせず、次回予定時刻との差分で
        # 次ループの待ち時間を決めます。
        # -------------------------------------------------
        # （次ループ先頭で待機）


# =========================================================
# ZERO SWITCH TASK
# =========================================================

async def switch_task():

    global last_switch
    global last_zero_time

    while True:

        switch_now = zero_switch.value()

        state.switch_pressed = (
            1 if switch_now == 0 else 0
        )

        # Falling edge = zero set.
        if (
            last_switch == 1 and
            switch_now == 0
        ):

            now = utime.ticks_ms()

            if utime.ticks_diff(
                now,
                last_zero_time
            ) >= ZERO_DEBOUNCE_MS:

                state.neutral = (
                    -state.lpf2_angle
                )

                state.zeroed = True

                print()
                print("==========================")
                print("ZERO SET")
                print(
                    "NEUTRAL = {:+.3f}".format(
                        state.neutral
                    )
                )
                print("ANGLE = 0.000")
                print("==========================")

                last_zero_time = now

        last_switch = switch_now

        await asyncio.sleep_ms(10)


# =========================================================
# JOYSTICK TASK
# =========================================================

async def joystick_task():

    while True:

        name, value = read_joystick()

        state.joystick = name
        state.joystick_value = value

        await asyncio.sleep_ms(20)


# =========================================================
# CONTROL TASK
# =========================================================

async def control_task():

    while True:

        try:

            # -------------------------------------------------
            # Menu mode: motor must stop.
            # -------------------------------------------------

            if state.menu_mode:

                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_stop()
                motor_sleep()

                await asyncio.sleep_ms(
                    Config.CONTROL_INTERVAL_MS
                )
                continue

            # -------------------------------------------------
            # Sensor invalid -> safe stop.
            # -------------------------------------------------

            if not state.sensor_ok:

                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_sleep()

                await asyncio.sleep_ms(
                    Config.CONTROL_INTERVAL_MS
                )
                continue

            # -------------------------------------------------
            # Require ZERO switch once after power-up.
            # -------------------------------------------------

            if (
                Config.MOTOR_REQUIRE_ZERO and
                not state.zeroed
            ):

                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_sleep()

                await asyncio.sleep_ms(
                    Config.CONTROL_INTERVAL_MS
                )
                continue

            # -------------------------------------------------
            # Motor enable: ONLY while GP6 button is pressed.
            # GP6 is active-low (PULL_UP).
            # Releasing the button immediately stops the motor.
            # -------------------------------------------------

            if not state.switch_pressed:

                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_sleep()

                await asyncio.sleep_ms(
                    Config.CONTROL_INTERVAL_MS
                )
                continue

            motor_enable()

            # -------------------------------------------------
            # Angle -> PWM
            # -------------------------------------------------

            diff = clamp(
                state.angle,
                Config.ANGLE_MIN,
                Config.ANGLE_MAX
            )

            if diff >= 0:

                if Config.ANGLE_MAX > 0:
                    normalized = (
                        diff /
                        float(Config.ANGLE_MAX)
                    )
                else:
                    normalized = 0.0

                state.target_pwm_command = (
                    normalized *
                    Config.PWM_MAX
                )

            else:

                if Config.ANGLE_MIN < 0:
                    normalized = (
                        diff /
                        abs(float(Config.ANGLE_MIN))
                    )
                else:
                    normalized = 0.0

                state.target_pwm_command = (
                    normalized *
                    abs(Config.PWM_MIN)
                )

            # -------------------------------------------------
            # PWM lag
            # -------------------------------------------------

            state.current_pwm_command += (
                state.target_pwm_command -
                state.current_pwm_command
            ) * 0.2

            duty = state.current_pwm_command

            # -------------------------------------------------
            # Deadband
            # -------------------------------------------------

            if abs(duty) < Config.PWM_DEADBAND:

                motor_stop()

            elif duty > 0:

                motor_cw(
                    abs(duty)
                )

            else:

                motor_ccw(
                    abs(duty)
                )

        except Exception as e:

            print(
                "CONTROL ERROR:",
                e
            )

            state.target_pwm_command = 0.0
            state.current_pwm_command = 0.0
            motor_sleep()

        await asyncio.sleep_ms(
            Config.CONTROL_INTERVAL_MS
        )


# =========================================================
# DISPLAY TASK
# =========================================================

async def display_task():

    while True:

        try:
            menu_manager.update()
        except Exception as e:
            # MENU処理で例外が発生しても、表示タスク自体を停止させない。
            print("MENU ERROR:", e)
            # MENU処理の例外で通常表示へ戻す。
            # 例外発生時にMENUを強制的に開き直さない。
            state.menu_level = MENU_MAIN
            state.menu_index = 0
            state.menu_mode = False
            state.last_joy_state = "NONE"

        # 表示前にも必ずmenu_indexを検証します。
        if state.menu_mode:
            if state.menu_level == MENU_MAIN:
                _menu_len = len(main_menu_items)
            elif state.menu_level == MENU_DISPLAY:
                _menu_len = len(display_menu_items)
            elif state.menu_level == MENU_SETTING:
                _menu_len = 6
            elif state.menu_level in (MENU_EDIT_LINE1, MENU_EDIT_LINE2):
                _menu_len = len(display_value_items)
            else:
                _menu_len = 1

            if _menu_len <= 0:
                state.menu_level = MENU_MAIN
                state.menu_index = 0
            else:
                state.menu_index = clamp(
                    int(state.menu_index),
                    0,
                    _menu_len - 1
                )

        if state.menu_mode:

            if state.menu_level == MENU_MAIN:

                line1 = "SELECT"
                line2 = ">" + main_menu_items[
                    state.menu_index
                ]

            elif state.menu_level == MENU_DISPLAY:

                line1 = "DISPLAY"
                line2 = ">" + display_menu_items[
                    state.menu_index
                ]

            elif state.menu_level == MENU_SETTING:

                items = [
                    "PWM_MAX",
                    "PWM_MIN",
                    "ANG_MAX",
                    "ANG_MIN",
                    "RESET",
                    "RETURN"
                ]

                line1 = "SETTING"
                line2 = ">" + items[
                    state.menu_index
                ]

            elif state.menu_level == MENU_EDIT_LINE1:

                line1 = "1stLine"
                line2 = ">" + display_value_items[
                    state.menu_index
                ]

            elif state.menu_level == MENU_EDIT_LINE2:

                line1 = "2ndLine"
                line2 = ">" + display_value_items[
                    state.menu_index
                ]

            elif state.menu_level == MENU_EDIT_PWM_MAX:

                line1 = "PWM_MAX"
                line2 = ">" + str(
                    Config.PWM_MAX
                )

            elif state.menu_level == MENU_EDIT_PWM_MIN:

                line1 = "PWM_MIN"
                line2 = ">" + str(
                    Config.PWM_MIN
                )

            elif state.menu_level == MENU_EDIT_ANG_MAX:

                line1 = "ANG_MAX"
                line2 = ">" + str(
                    Config.ANGLE_MAX
                )

            elif state.menu_level == MENU_EDIT_ANG_MIN:

                line1 = "ANG_MIN"
                line2 = ">" + str(
                    Config.ANGLE_MIN
                )

            else:

                line1 = "MENU"
                line2 = "RETURN"

        else:

            line1 = get_line(
                state.line1_setting
            )

            line2 = get_line(
                state.line2_setting
            )

        line1 = str(line1)[:8]
        line2 = str(line2)[:8]

        if line1 != state.last_line1:

            lcd_print(
                line1,
                0
            )

            state.last_line1 = line1

        if line2 != state.last_line2:

            lcd_print(
                line2,
                1
            )

            state.last_line2 = line2

        await asyncio.sleep_ms(200)


# =========================================================
# MONITOR TASK
# =========================================================

async def monitor_task():

    while True:

        if state.sensor_ok:

            age = utime.ticks_diff(
                utime.ticks_ms(),
                state.last_sensor_time
            )

            if age > Config.SENSOR_HOLD_TIMEOUT_MS:

                print("SENSOR TIMEOUT - MOTOR STOP")
                state.sensor_ok = False
                state.target_pwm_command = 0.0
                state.current_pwm_command = 0.0
                motor_sleep()

        gc.collect()

        await asyncio.sleep_ms(1000)


# =========================================================
# MAIN
# =========================================================

async def main():

    print("MAIN START")

    # Safe motor state before anything else.
    motor_sleep()

    load_config()

    # -----------------------------------------------------
    # MENU SAFE START
    # -----------------------------------------------------
    # 設定ファイルにはMENU状態を保存しない。
    # 電源投入/再起動時は必ず通常表示から開始する。
    state.menu_mode = False
    state.menu_level = MENU_MAIN
    state.menu_index = 0
    state.last_joy_state = "NONE"

    lcd_print("INIT", 0)
    lcd_print("BNO055", 1)

    # -----------------------------------------------------
    # BNO055
    # -----------------------------------------------------

    if not bno_init():

        lcd_print("BNO ERR", 0)
        lcd_print("CHECK", 1)

        while True:
            motor_sleep()
            await asyncio.sleep(1)

    # -----------------------------------------------------
    # Tasks
    # -----------------------------------------------------

    asyncio.create_task(
        sensor_task()
    )

    asyncio.create_task(
        switch_task()
    )

    asyncio.create_task(
        joystick_task()
    )

    asyncio.create_task(
        display_task()
    )

    asyncio.create_task(
        control_task()
    )

    asyncio.create_task(
        monitor_task()
    )

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


# =========================================================
# RUN
# =========================================================

try:
    asyncio.run(main())

except KeyboardInterrupt:

    print("STOP")
    motor_sleep()

except Exception as e:

    print("FATAL ERROR:", e)
    motor_sleep()
