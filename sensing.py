"""Angle estimation and the periodic BNO055 sensing task."""
import math
import uasyncio as asyncio
import utime

from app_state import Config, clamp, fitin360, state
from bno055 import BNO_READ_PENDING, BNO_READ_TIMEOUT, read_acc_gyro

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

    m_gyroy = sensor[4]
    m_gyroz = sensor[5]

    state.gyro_z = m_gyroz
    state.dt = dt

    new_accx = m_accx

    new_accy = SIGN_LR * (
        COS_Q1 * m_accy +
        SIN_Q1 * m_accz
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



async def sensor_task():

    initialized = False

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

    last_sample_us = utime.ticks_us()
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

        read_result, sensor = read_acc_gyro()

        if read_result == BNO_READ_PENDING:

            # A normal scheduler step with no bytes (or only part of a reply)
            # is not a sensor error.  The persistent queue is polled next step.
            continue

        if read_result == BNO_READ_TIMEOUT:

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

        sample_us = utime.ticks_us()
        dt = (
            utime.ticks_diff(
                sample_us,
                last_sample_us
            ) / 1000000.0
        )
        last_sample_us = sample_us

        if dt <= 0.0:
            dt = 0.001

        if dt > 0.1:
            dt = 0.1

        if not initialized:

            initial_angle = initialize_angle(sensor)
            state.observed_angle = initial_angle
            state.kalman_angle = initial_angle
            state.lpf1_angle = initial_angle
            state.lpf2_angle = initial_angle
            state.angle = -initial_angle
            initialized = True

            print(
                "INITIAL ANGLE = {:+.3f}".format(
                    state.angle
                )
            )

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


