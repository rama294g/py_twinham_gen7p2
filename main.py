"""TwinHAM composition root.

Hardware drivers and application tasks live in feature-specific modules; this
module only wires them together and owns application startup/shutdown.
"""
import gc
import uasyncio as asyncio
import utime
from machine import Pin, PWM

from app_state import Config, MENU_MAIN, clamp, state
from bno055 import bno_init
from config_store import load_config
from lcd_menu import display_task, joystick_task, lcd_print
from sensing import sensor_task


gc.collect()
led = Pin("LED", Pin.OUT)
led.off()

print()
print("================================")
print("PICO 2W INTEGRATED START")
print("================================")



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



try:
    asyncio.run(main())

except KeyboardInterrupt:

    print("STOP")
    motor_sleep()

except Exception as e:

    print("FATAL ERROR:", e)
    motor_sleep()
