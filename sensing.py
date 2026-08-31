"""Angle estimation from a parsed BNO055 sample."""

import math

from app_state import Config, clamp, fitin360, state

Q1_RAD = math.radians(Config.Q1_DEG)
COS_Q1 = math.cos(Q1_RAD)
SIN_Q1 = math.sin(Q1_RAD)

def update_angle(sensor, dt):

    m_accx = sensor[0]
    m_accy = sensor[1]
    m_accz = sensor[2]

    m_gyroy = sensor[4]
    m_gyroz = sensor[5]

    state.gyro_z = m_gyroz
    state.dt = dt

    # The mounting side is user-editable at runtime, so do not cache its sign
    # when this module is imported (configuration is loaded later at startup).
    sign_lr = -1.0 if Config.SW_IS_RIGHT else 1.0

    new_accx = m_accx

    new_accy = sign_lr * (COS_Q1 * m_accy + SIN_Q1 * m_accz)

    new_gyroz = sign_lr * (COS_Q1 * m_gyroz - SIN_Q1 * m_gyroy)

    state.gyro_angle_rate = -new_gyroz

    tilt_observe = math.degrees(math.atan2(new_accy, new_accx))

    if Config.SW_IS_RIGHT:
        tilt_observe -= 90.0
    else:
        tilt_observe += 90.0

    tilt_observe = fitin360(tilt_observe)

    tilt_priest = state.kalman_angle - new_gyroz * dt

    tilt_delta = fitin360(tilt_observe - tilt_priest)

    state.kalman_angle = Config.KALMAN_GAIN * tilt_delta + tilt_priest

    state.kalman_angle = fitin360(state.kalman_angle)

    state.kalman_angle = clamp(state.kalman_angle, -90.0, 90.0)

    state.lpf1_angle = (dt * state.kalman_angle + Config.LPF_TAU * state.lpf1_angle) / (
        Config.LPF_TAU + dt
    )

    state.lpf2_angle = (dt * state.lpf1_angle + Config.LPF_TAU * state.lpf2_angle) / (
        Config.LPF_TAU + dt
    )

    state.observed_angle = tilt_observe

    state.angle = -state.lpf2_angle - state.neutral * 0 - Config.NEUTRAL_ANG

    state.angle = clamp(state.angle, -90.0, 90.0)
