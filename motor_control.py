"""Motor output state transitions, kept independent from MicroPython hardware."""

from app_state import clamp


def force_motor_sleep(state, motor_sleep):
    """Clear every ramp/output command and put the driver to sleep immediately."""
    state.switch_gain = 0.0
    state.target_pwm_command = 0.0
    state.current_pwm_command = 0.0
    motor_sleep()


def apply_motor_output(
    state,
    switch_pressed,
    permitted,
    motor_enable,
    motor_sleep,
    motor_cw,
    motor_ccw,
    ramp_step=0.04,
):
    """Advance one GP6 ramp step and apply PWM/nSLEEP atomically.

    Safety interlocks bypass the ramp.  A normal GP6 release keeps nSLEEP high
    while duty ramps down, then sleeps the driver on the same step on which
    both the gain and effective duty reach zero.
    """
    state.switch_pressed = 1 if switch_pressed else 0
    if not permitted:
        force_motor_sleep(state, motor_sleep)
        return 0.0

    if state.switch_pressed > state.switch_gain:
        state.switch_gain = clamp(state.switch_gain + ramp_step, 0.0, 1.0)
    elif state.switch_pressed < state.switch_gain:
        state.switch_gain = clamp(state.switch_gain - ramp_step, 0.0, 1.0)

    duty = state.current_pwm_command * state.switch_gain
    if state.switch_gain == 0.0 and duty == 0.0:
        motor_sleep()
    else:
        # nSLEEP is asserted only after all immediate-stop conditions have
        # passed, and before either PWM output is applied.
        motor_enable()
        if duty > 0.0:
            motor_ccw(abs(duty))
        elif duty < 0.0:
            motor_cw(abs(duty))
        else:
            # Keep nSLEEP high while the GP6 ramp is active, even at neutral.
            motor_cw(0.0)

    return duty
