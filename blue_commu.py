"""Bluetooth communication"""
# =========================================================
# BLE通信だけを担当するファイル
#
# このファイルでは、
#
# ① BLEを開始する
# ② 「TwinHAM_LH」としてAdvertisingする
# ③ PCが接続したことを検出する
# ④ PCが切断したことを検出する
# ⑤ PCから文字列を受信する
# ⑥ PicoからPCへNotifyする
#
# を行います。
# =========================================================


# =========================================================
# 1. BLEライブラリを読み込む
# =========================================================

import bluetooth

from micropython import const


# =========================================================
# 2. BLEデバイス名
# =========================================================
#
# PCのBLEスキャン画面に
#
#     TwinHAM_LH
#
# と表示されます。
# =========================================================

DEVICE_NAME = b"TwinHAM_LH"


# =========================================================
# 3. Nordic UART ServiceのUUID
# =========================================================
#
# PCとPicoの間で、
# UARTのように文字列を送受信するために使用します。
#
# Service:
# 6e400001...
#
# TX:
# 6e400003...
# Pico → PC
#
# RX:
# 6e400002...
# PC → Pico
# =========================================================

UART_UUID = bluetooth.UUID(
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
)


# ---------------------------------------------------------
# TX
# ---------------------------------------------------------
#
# PicoからPCへデータを送るためのCharacteristic。
#
# NOTIFYを使用します。
# =========================================================

UART_TX = (
    bluetooth.UUID(
        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
    ),
    bluetooth.FLAG_NOTIFY
)


# ---------------------------------------------------------
# RX
# ---------------------------------------------------------
#
# PCからPicoへコマンドを送るためのCharacteristic。
#
# WRITEを使用します。
# =========================================================

UART_RX = (
    bluetooth.UUID(
        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
    ),
    bluetooth.FLAG_WRITE |
    bluetooth.FLAG_WRITE_NO_RESPONSE
)


# =========================================================
# 4. BLEサービスをまとめる
# =========================================================

UART_SERVICE = (
    UART_UUID,
    (
        UART_TX,
        UART_RX,
    )
)


# =========================================================
# 5. BLEイベント番号
# =========================================================
#
# BLEで何か起こるとIRQが呼ばれます。
#
# CONNECT
#   PCが接続
#
# DISCONNECT
#   PCが切断
#
# WRITE
#   PCからデータ受信
# =========================================================

_IRQ_CENTRAL_CONNECT = const(1)

_IRQ_CENTRAL_DISCONNECT = const(2)

_IRQ_GATTS_WRITE = const(3)


# =========================================================
# 6. BLE通信クラス
# =========================================================
#
# main.pyでは、
#
# ble_comm = BLECommunication()
#
# とするだけでBLEを使えます。
# =========================================================

class BLECommunication:


    # =====================================================
    # 6-1. 初期化
    # =====================================================

    def __init__(self):

        print(
            "BLE module init"
        )


        # PicoのBLE機能を作る
        self.ble = bluetooth.BLE()


        # BLEをONにする
        self.ble.active(True)


        # 現在接続しているPCの一覧
        self.connections = set()


        # PCから受信したコマンドを一時保存
        #
        # 例：
        # ["PING", "GET,STATUS"]
        #
        self.rx_commands = []


        # BLEサービスを登録
        #
        # tx_handle
        #   Pico → PC
        #
        # rx_handle
        #   PC → Pico
        (
            (self.tx_handle, self.rx_handle),
        ) = self.ble.gatts_register_services(
            (UART_SERVICE,)
        )


        # BLEイベントを受け取る関数を登録
        self.ble.irq(
            self._irq
        )


        print(
            "BLE module ready"
        )


    # =====================================================
    # 6-2. BLEイベント処理
    # =====================================================
    #
    # BLEで接続・切断・受信が起きると、
    # この関数が自動的に呼ばれます。
    # =====================================================

    def _irq(
        self,
        event,
        data
    ):


        # -------------------------------------------------
        # PCが接続した
        # -------------------------------------------------

        if event == _IRQ_CENTRAL_CONNECT:

            conn_handle, _, _ = data

            self.connections.add(
                conn_handle
            )

            print(
                "Central connected"
            )


        # -------------------------------------------------
        # PCが切断した
        # -------------------------------------------------

        elif event == _IRQ_CENTRAL_DISCONNECT:

            conn_handle, _, _ = data


            if conn_handle in self.connections:

                self.connections.remove(
                    conn_handle
                )


            print(
                "Central disconnected"
            )


            # 切断後、もう一度Advertisingする
            self.advertise()


        # -------------------------------------------------
        # PCからデータを受信した
        # -------------------------------------------------

        elif event == _IRQ_GATTS_WRITE:

            conn_handle, attr_handle = data


            # RX Characteristic以外なら無視
            if attr_handle != self.rx_handle:

                return


            try:

                # RXからデータを読む
                raw = self.ble.gatts_read(
                    self.rx_handle
                )


                if not raw:

                    return


                # bytes → 文字列
                text = bytes(
                    raw
                ).decode(
                    "utf-8",
                    "ignore"
                )


                # 改行があれば複数コマンドとして分ける
                for line in text.split(
                    "\n"
                ):

                    line = line.strip()


                    if line:

                        # main.pyが後で処理するため、
                        # 一旦キューに保存する
                        self.rx_commands.append(
                            line
                        )


            except Exception as e:

                print(
                    "BLE RX error:",
                    e
                )


    # =====================================================
    # 6-3. Advertising開始
    # =====================================================
    #
    # PCからTwinHAM_LHを検索できるようにします。
    # =====================================================

    def advertise(self):

        # BLE Advertisingの基本情報
        adv_data = bytearray(
            b"\x02\x01\x06"
        )


        # デバイス名「TwinHAM_LH」を追加
        adv_data += bytes(
            (
                len(DEVICE_NAME) + 1,
                0x09
            )
        ) + DEVICE_NAME


        # 100ms間隔でAdvertising
        self.ble.gap_advertise(
            100000,
            adv_data
        )


        print(
            "BLE advertising"
        )


    # =====================================================
    # 6-4. BLE接続確認
    # =====================================================

    def is_connected(self):

        if len(
            self.connections
        ) > 0:

            return True

        return False


    # =====================================================
    # 6-5. PCから受信したコマンドを1件取得
    # =====================================================
    #
    # main.pyのcommand_task()が使用します。
    # =====================================================

    def get_command(self):

        if not self.rx_commands:

            return None


        # 一番古いコマンドを取り出す
        return self.rx_commands.pop(
            0
        )


    # =====================================================
    # 6-6. PicoからPCへ文字列を送信
    # =====================================================
    #
    # Notifyを使ってPCへ送ります。
    #
    # 例えば、
    #
    # ble_comm.send_text("PONG")
    #
    # とするとPCへ
    #
    # PONG
    #
    # が届きます。
    # =====================================================

    def send_text(
        self,
        text
    ):

        # 接続中のPCすべてへ送信
        for conn in list(
            self.connections
        ):

            try:

                self.ble.gatts_notify(
                    conn,
                    self.tx_handle,
                    text
                )


            except Exception as e:

                print(
                    "BLE notify error:",
                    e
                )
