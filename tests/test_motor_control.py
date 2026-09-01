from types import SimpleNamespace

import pytest

from motor_control import apply_motor_output


class FakeMotor:
    def __init__(self):
        self.nsleep = 0
        self.cw = 0.0
        self.ccw = 0.0

    def enable(self):
        self.nsleep = 1

    def sleep(self):
        self.cw = self.ccw = 0.0
        self.nsleep = 0

    def drive_cw(self, duty):
        self.ccw = 0.0
        self.cw = duty

    def drive_ccw(self, duty):
        self.cw = 0.0
        self.ccw = duty


def make_state(command=50.0):
    return SimpleNamespace(
        switch_pressed=0,
        switch_gain=0.0,
        target_pwm_command=command,
        current_pwm_command=command,
    )


def step(state, motor, pressed, permitted=True, ramp_step=0.5):
    return apply_motor_output(
        state, pressed, permitted, motor.enable, motor.sleep,
        motor.drive_cw, motor.drive_ccw, ramp_step
    )


def test_press_release_and_end_of_ramp_sleep_transition():
    state, motor = make_state(), FakeMotor()
    step(state, motor, True)
    assert (state.switch_gain, motor.ccw, motor.nsleep) == (0.5, 25.0, 1)
    step(state, motor, True)
    assert (state.switch_gain, motor.ccw, motor.nsleep) == (1.0, 50.0, 1)

    step(state, motor, False)
    assert (state.switch_gain, motor.ccw, motor.nsleep) == (0.5, 25.0, 1)
    step(state, motor, False)
    assert (state.switch_gain, motor.cw, motor.ccw, motor.nsleep) == (0.0, 0.0, 0.0, 0)


@pytest.mark.parametrize("stop_condition", ["menu", "ble_stop", "sensor_error"])
def test_every_immediate_stop_condition_clears_pwm_and_nsleep(stop_condition):
    state, motor = make_state(-40.0), FakeMotor()
    step(state, motor, True)
    assert motor.cw == 20.0 and motor.nsleep == 1

    # Each named interlock is represented by permitted=False at the hardware
    # boundary; all must bypass an in-progress ramp in exactly the same way.
    step(state, motor, True, permitted=False)
    assert (state.switch_gain, state.current_pwm_command) == (0.0, 0.0)
    assert (motor.cw, motor.ccw, motor.nsleep) == (0.0, 0.0, 0)


def test_zero_angle_keeps_nsleep_high_only_until_release_ramp_finishes():
    state, motor = make_state(0.0), FakeMotor()
    step(state, motor, True)
    assert (state.switch_gain, motor.cw, motor.ccw, motor.nsleep) == (0.5, 0.0, 0.0, 1)
    step(state, motor, False)
    assert (state.switch_gain, motor.cw, motor.ccw, motor.nsleep) == (0.0, 0.0, 0.0, 0)
