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


import bluetooth
from micropython import const


DEVICE_NAME = b"TwinHAM_LH"
RX_BUFFER_SIZE = 128
MAX_QUEUED_COMMANDS = 16

UART_UUID_SVCS = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
UART_UUID_TX = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
UART_UUID_RX = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

UART_TX = (UART_UUID_TX, bluetooth.FLAG_NOTIFY)
UART_RX = (UART_UUID_RX, bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE)
UART_SERVICE = (UART_UUID_SVCS, (UART_TX, UART_RX))

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)


class BLECommunication:

    def __init__(self):
        print("BLE module init")
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.connections = set()
        self.rx_commands = []
        ((self.tx_handle, self.rx_handle),) = self.ble.gatts_register_services(
            (UART_SERVICE,)
        )
        # The default characteristic buffer is only 20 bytes.  Use append mode
        # so writes made before the IRQ is serviced are not silently replaced.
        self.ble.gatts_set_buffer(self.rx_handle, RX_BUFFER_SIZE, True)
        self.ble.irq(self._irq)
        print("BLE module ready")

    def _irq(self, event, data):

        # -------------------------------------------------
        # PCが接続した
        # -------------------------------------------------
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self.connections.add(conn_handle)
            print("Central connected")

        # -------------------------------------------------
        # PCが切断した
        # -------------------------------------------------
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self.connections:
                self.connections.remove(conn_handle)
            print("Central disconnected")
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
                raw = self.ble.gatts_read(self.rx_handle)

                if not raw:
                    return

                # bytes → 文字列
                text = bytes(raw).decode("utf-8", "ignore")

                # 改行があれば複数コマンドとして分ける
                for line in text.split("\n"):
                    line = line.strip(" \t\r")
                    if line:
                        # main.pyが後で処理するため、
                        # 一旦キューに保存する
                        if len(self.rx_commands) < MAX_QUEUED_COMMANDS:
                            self.rx_commands.append(line)
                        else:
                            print("BLE RX queue full")

            except Exception as e:
                print("BLE RX error:", e)

    # =====================================================
    # 6-3. Advertising開始
    # =====================================================
    #
    # PCからTwinHAM_LHを検索できるようにします。
    # =====================================================

    def advertise(self):
        adv_data = bytearray(b"\x02\x01\x06")
        adv_data += bytes((len(DEVICE_NAME) + 1, 0x09)) + DEVICE_NAME
        self.ble.gap_advertise(100000, adv_data)
        print("BLE advertising")

    # =====================================================
    # 6-4. BLE接続確認
    # =====================================================

    def is_connected(self):
        return bool(self.connections)

    # =====================================================
    # 6-5. PCから受信したコマンドを1件取得
    # =====================================================
    # main.pyのcommand_task()が使用します。
    # =====================================================

    def get_command(self):
        if not self.rx_commands:
            return None
        return self.rx_commands.pop(0)

    # =====================================================
    # 6-6. PicoからPCへ文字列を送信
    # =====================================================
    # Notifyを使ってPCへ送ります。
    # 例えば、
    # ble_comm.send_text("PONG")
    # とするとPCへ
    # PONG
    # が届きます。
    # =====================================================

    def send_text(self, text):
        if not isinstance(text, bytes):
            text = str(text).encode("utf-8")
        if not text.endswith(b"\n"):
            text += b"\n"

        # 接続中のPCすべてへ送信
        for conn in list(self.connections):
            try:
                self.ble.gatts_notify(conn, self.tx_handle, text)
            except Exception as e:
                print("BLE notify error:", e)
