"""LCD output and joystick-driven configuration user interface."""
import uasyncio as asyncio
import utime
from machine import ADC, I2C, Pin

from app_state import (
    Config, MENU_DISPLAY, MENU_EDIT_ANG_MAX, MENU_EDIT_ANG_MIN,
    MENU_EDIT_LINE1, MENU_EDIT_LINE2,
    MENU_EDIT_GAIN, MENU_EDIT_NEUTRAL, MENU_EDIT_PWM_MAX, MENU_EDIT_PWM_MIN,
    MENU_EDIT_SW_SIDE,
    MENU_MAIN, MENU_SETTING, clamp, display_menu_items, display_value_items,
    main_menu_items, setting_menu_items, state,
)
from config_store import save_config

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

        if state.menu_level == MENU_EDIT_GAIN:
            if joy == "UP":
                Config.GAIN += 1
            elif joy == "DOWN":
                Config.GAIN -= 1
            elif joy == "RIGHT":
                Config.GAIN += 5
            elif joy == "LEFT":
                Config.GAIN -= 5
            elif joy == "CENTER":
                save_config()
                state.menu_level = MENU_SETTING
            Config.GAIN = clamp(Config.GAIN, 0, 100)
            return

        if state.menu_level == MENU_EDIT_NEUTRAL:
            if joy == "UP":
                Config.NEUTRAL_ANG += 1
            elif joy == "DOWN":
                Config.NEUTRAL_ANG -= 1
            elif joy == "RIGHT":
                Config.NEUTRAL_ANG += 5
            elif joy == "LEFT":
                Config.NEUTRAL_ANG -= 5
            elif joy == "CENTER":
                save_config()
                state.menu_level = MENU_SETTING
            Config.NEUTRAL_ANG = clamp(Config.NEUTRAL_ANG, 10, 50)
            return

        if state.menu_level == MENU_EDIT_PWM_MAX:
            if joy == "UP":         Config.PWM_MAX += 1
            elif joy == "DOWN":     Config.PWM_MAX -= 1
            elif joy == "RIGHT":    Config.PWM_MAX += 5
            elif joy == "LEFT":     Config.PWM_MAX -= 5
            elif joy == "CENTER":
                save_config()
                state.menu_level = MENU_SETTING
            Config.PWM_MAX = clamp(                Config.PWM_MAX, 1, 100            )
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

        if state.menu_level == MENU_EDIT_SW_SIDE:
            if joy in ("UP", "RIGHT"):
                Config.SW_IS_RIGHT = True
            elif joy in ("DOWN", "LEFT"):
                Config.SW_IS_RIGHT = False
            elif joy == "CENTER":
                save_config()
                state.menu_level = MENU_SETTING
            return

        # -------------------------------------------------
        # Current menu
        # -------------------------------------------------

        if state.menu_level == MENU_MAIN:
            current_menu = main_menu_items

        elif state.menu_level == MENU_DISPLAY:
            current_menu = display_menu_items

        elif state.menu_level == MENU_SETTING:
            current_menu = setting_menu_items

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
                MENU_EDIT_GAIN,
                MENU_EDIT_NEUTRAL,
                MENU_EDIT_PWM_MAX,
                MENU_EDIT_PWM_MIN,
                MENU_EDIT_ANG_MAX,
                MENU_EDIT_ANG_MIN,
                MENU_EDIT_SW_SIDE
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

            if selected == "GAIN":
                state.menu_level = MENU_EDIT_GAIN
            elif selected == "NEUTRAL":
                state.menu_level = MENU_EDIT_NEUTRAL
            elif selected == "PWM_MAX":
                state.menu_level = MENU_EDIT_PWM_MAX

            elif selected == "PWM_MIN":
                state.menu_level = MENU_EDIT_PWM_MIN

            elif selected == "ANG_MAX":
                state.menu_level = MENU_EDIT_ANG_MAX

            elif selected == "ANG_MIN":
                state.menu_level = MENU_EDIT_ANG_MIN

            elif selected == "SW_SIDE":
                state.menu_level = MENU_EDIT_SW_SIDE

            elif selected == "RESET":

                Config.PWM_MAX = 30
                Config.PWM_MIN = -30
                Config.ANGLE_MAX = 90
                Config.ANGLE_MIN = -90
                Config.GAIN = 50
                Config.NEUTRAL_ANG = 30
                Config.SW_IS_RIGHT = False

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



async def joystick_task():

    while True:

        name, value = read_joystick()

        state.joystick = name
        state.joystick_value = value

        await asyncio.sleep_ms(20)



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
                _menu_len = len(setting_menu_items)
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
                line1 = "SETTING"
                line2 = ">" + setting_menu_items[
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

            elif state.menu_level == MENU_EDIT_GAIN:

                line1 = "GAIN"
                line2 = ">" + str(Config.GAIN)

            elif state.menu_level == MENU_EDIT_NEUTRAL:

                line1 = "NEUTRAL"
                line2 = ">" + str(Config.NEUTRAL_ANG)

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

            elif state.menu_level == MENU_EDIT_SW_SIDE:

                line1 = "SW_SIDE"
                line2 = ">RIGHT" if Config.SW_IS_RIGHT else ">LEFT"

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
