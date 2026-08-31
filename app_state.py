"""Shared configuration values, runtime state, and small pure helpers."""


class Config:
    CONFIG_FILE = "config.json"
    GAIN = 50
    NEUTRAL_ANG = 30
    ANGLE_MIN = -90
    ANGLE_MAX = 90
    PWM_MIN = -100
    PWM_MAX = 100
    PWM_DEADBAND = 0
    MOTOR_PWM_FREQ = 5000
    CONTROL_INTERVAL_MS = 20
    SENSOR_INTERVAL_MS = 20
    BLE_INTERVAL_MS = 200
    UART_TIMEOUT_MS = 50
    SENSOR_HOLD_TIMEOUT_MS = 500
    Q1_DEG = 0.0
    SW_IS_RIGHT = False
    KALMAN_GAIN = 0.01
    LPF_TAU = 0.01
    MOTOR_REQUIRE_ZERO = True


class SystemState:
    angle = 0.0
    observed_angle = 0.0
    kalman_angle = 0.0
    lpf_angle = 0.0
    lpf1_angle = 0.0
    lpf2_angle = 0.0
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
    motor_remote_enabled = True
    motor_state = "STOP"
    menu_mode = False
    menu_level = 0
    menu_index = 0
    last_joy_state = "NONE"
    line1_setting = 0
    line2_setting = 1
    last_line1 = ""
    last_line2 = ""


state = SystemState()

MENU_MAIN = 0
MENU_DISPLAY = 1
MENU_SETTING = 2
MENU_EDIT_LINE1 = 3
MENU_EDIT_LINE2 = 4
MENU_EDIT_GAIN = 10
MENU_EDIT_NEUTRAL = 11
MENU_EDIT_PWM_MAX = 12
MENU_EDIT_PWM_MIN = 13
MENU_EDIT_ANG_MAX = 14
MENU_EDIT_ANG_MIN = 15
MENU_EDIT_SW_SIDE = 16
main_menu_items = ["SETTING", "DISPLAY", "RETURN"]
display_menu_items = ["1stLine", "2ndLine", "RETURN"]
setting_menu_items = [
    "GAIN",
    "NEUTRAL",
    "PWM_MAX",
    "PWM_MIN",
    "ANG_MAX",
    "ANG_MIN",
    "SW_SIDE",
    "RESET",
    "RETURN",
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
    "ERROR",
]


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
