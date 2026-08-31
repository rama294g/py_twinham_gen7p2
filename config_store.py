"""Persistence for user-editable settings."""

import ujson

from app_state import Config, clamp, display_value_items, state


def save_config():

    try:

        cfg = {
            "GAIN": Config.GAIN,
            "NEUTRAL_ANG": Config.NEUTRAL_ANG,
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

        Config.GAIN = clamp(int(cfg.get("GAIN", Config.GAIN)), 0, 100)
        Config.NEUTRAL_ANG = clamp(
            int(cfg.get("NEUTRAL_ANG", Config.NEUTRAL_ANG)), 10, 50
        )
        Config.PWM_MAX = clamp(int(cfg.get("PWM_MAX", 30)), 1, 100)
        Config.PWM_MIN = clamp(int(cfg.get("PWM_MIN", -30)), -100, -1)
        Config.ANGLE_MAX = clamp(int(cfg.get("ANGLE_MAX", 90)), 1, 180)
        Config.ANGLE_MIN = clamp(int(cfg.get("ANGLE_MIN", -90)), -180, -1)

        # -----------------------------------------------------
        # DISPLAY設定を厳密に検証
        # -----------------------------------------------------
        line1 = int(cfg.get("line1_setting", 0))
        line2 = int(cfg.get("line2_setting", 1))

        state.line1_setting = clamp(line1, 0, len(display_value_items) - 1)
        state.line2_setting = clamp(line2, 0, len(display_value_items) - 1)

        print("Config loaded")
        print("DISPLAY 1stLine =", state.line1_setting)
        print("DISPLAY 2ndLine =", state.line2_setting)

        # -----------------------------------------------------
        # 不正値があった場合は修正値を保存
        # -----------------------------------------------------
        if line1 != state.line1_setting or line2 != state.line2_setting:
            print("DISPLAY CONFIG CORRECTED")
            save_config()

    except Exception as e:
        print("Config load error:", e)
        print("Default config")

        Config.PWM_MAX = 30
        Config.PWM_MIN = -30
        Config.ANGLE_MAX = 90
        Config.ANGLE_MIN = -90
        Config.GAIN = 50
        Config.NEUTRAL_ANG = 30
        state.line1_setting = 0
        state.line2_setting = 1
