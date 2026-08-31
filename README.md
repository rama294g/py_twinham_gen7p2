# TwinHAM v7.2 マイコンソフトウェア

TwinHAM は、手動車椅子を電動化するためのユニットです。本リポジトリには、Raspberry Pi Pico 2 W 上の MicroPython で動作する TwinHAM v7.2 の制御ソフトウェアを収録しています。

> [!WARNING]
> 本ソフトウェアはモーターを駆動します。現状の実装では GP6 を離した後も出力が約 0.5 秒かけて減衰し、モータードライバーの `nSLEEP` はセンサー異常、メニュー表示、BLE 停止指令などの条件がない限り High のままです。また、起動後のゼロ点設定処理は実装されていません。車椅子へ搭載する前に車輪を浮かせ、低出力で配線、回転方向、姿勢角、PWM 範囲およびスイッチ動作を確認してください。独立した非常停止手段を必ず用意してください。

## 現在の主な機能

- BNO055 の加速度・ジャイロ値を組み合わせた車体角度の推定
- 推定角度を正負それぞれの設定範囲で線形変換する CW / CCW PWM 制御
- GP6 スイッチ入力によるモーター出力のランプアップ／ランプダウン
- 8 文字 × 2 行 I2C LCD への状態表示
- 5 方向ジョイスティックによる設定・表示項目の変更
- 設定の `config.json` への保存と起動時の読み込み
- Nordic UART Service（NUS）互換の Bluetooth LE コマンド／状態通知
- センサータイムアウト、メニュー表示、BLE 停止指令、制御例外時のモーター停止

## 対象環境

| 項目 | 内容 |
| --- | --- |
| マイコン | Raspberry Pi Pico 2 W（RP2350） |
| 実行環境 | MicroPython（Bluetooth、UART RX-idle IRQ 対応ビルド） |
| エントリーポイント | `main.py` |
| 姿勢センサー | BNO055（UART1、115200 bps、ACCGYRO モード） |
| LCD | I2C0、アドレス `0x3E`、8 文字 × 2 行 |
| モーター PWM | 5 kHz |
| センサー要求／制御周期 | 20 ms（50 Hz） |
| LCD 更新／BLE 定期通知 | 200 ms |

外部 Python パッケージは使用せず、`machine`、`bluetooth`、`micropython`、`uasyncio`、`ujson` など MicroPython のモジュールを使用します。ハードウェア API をモジュールの import 時から使用するため、通常の CPython ではアプリケーションを実行できません。

## ピンアサイン

| GPIO | 用途 | 備考 |
| --- | --- | --- |
| GP0 | LCD SDA | I2C0、100 kHz |
| GP1 | LCD SCL | I2C0、100 kHz |
| GP4 | BNO055 UART TX | UART1、Pico から BNO055 へ接続 |
| GP5 | BNO055 UART RX | UART1、BNO055 から Pico へ接続 |
| GP6 | 出力操作スイッチ | Active Low、内部プルアップ |
| GP16 | モータードライバー CW PWM | 5 kHz |
| GP17 | モータードライバー CCW PWM | 5 kHz |
| GP18 | モータードライバー `nSLEEP` | Low でスリープ |
| GP26 | 5 方向ジョイスティック | ADC 入力、抵抗分圧式 |

各機器の GND は共通にしてください。BNO055 側の TX は Pico の GP5（RX）へ、BNO055 側の RX は Pico の GP4（TX）へ交差接続します。モータードライバーや周辺機器の電圧・電流仕様は、それぞれのデータシートに従ってください。

## セットアップ

### 1. MicroPython の準備

Pico 2 W に対応し、上記の必要モジュールと UART RX-idle IRQ を利用できる MicroPython ファームウェアを書き込みます。

### 2. Python ファイルの転送

Pico を USB で PC に接続し、リポジトリ直下の全 `.py` ファイルをデバイスのルートへコピーします。`mpremote` を使う例は次のとおりです。

```bash
for file in *.py; do mpremote connect auto fs cp "$file" ":$file"; done
```

リポジトリにある設定値から始める場合は、`config.json` もコピーします。コピーしない場合は組み込みの初期値で起動し、LCD メニューで設定を確定するか BLE の `SAVE` を実行した時点でファイルが作成されます。

```bash
mpremote connect auto fs cp config.json :config.json
```

> [!NOTE]
> BLE 実装が読み込む任意設定ファイル名は `ble_settings.json` です。リポジトリ内の `ble_settings_LH.json` と `ble_settings_RH.json` は現在の起動手順では自動的に読み込まれません。`ble_settings.json` がない場合は、コード内の `TwinHAM_LH` と標準 NUS UUID が使われます。

転送後に Pico を再起動し、必要に応じて REPL でログを確認します。

```bash
mpremote connect auto reset
mpremote connect auto repl
```

## 起動とモーター制御

1. import 時に GPIO、PWM、LCD、BNO055、BLE を初期化し、BLE Advertising を開始します。モータードライバーは初期状態でスリープします。
2. `config.json` を読み込み、BNO055 の Chip ID `0xA0` を確認します。
3. BNO055 を ACCGYRO モードへ切り替え、再度 Chip ID を確認します。各確認の待ち時間は最大 500 ms です。
4. 認識できない場合は LCD に `BNO ERR` / `CHECK` と表示し、モーターをスリープさせたまま待機します。
5. 正常起動後は 20 ms ごとにセンサー値を要求し、角度と PWM 指令を更新します。

算出角度 `angle` は、加速度から求めた観測角とジャイロを Kalman 相当の補正および 2 段 LPF に通し、`NEUTRAL` を差し引いた値です。最終角度は `-90` ～ `90` 度に制限されます。正の角度は `0` ～ `ANG_MAX` を `0` ～ `PWM_MAX` に、負の角度は `ANG_MIN` ～ `0` を `PWM_MIN` ～ `0` に線形変換します。正の指令で CCW、負の指令で CW を駆動します。

GP6 を押すと `switch_gain` が制御周期ごとに `0.04` ずつ増え、約 0.5 秒で指令値の 100% に達します。離した場合も同じ割合で 0 まで減少します。GP6 の押下は姿勢のゼロ点を設定するものではありません。基準姿勢は `NEUTRAL` で調整してください。

## ジョイスティックメニュー

通常画面でジョイスティックの中央（ADC 値がおおむね 0）を入力するとメニューを開きます。

- **UP / DOWN**: 項目移動、編集値を 1 ずつ変更
- **RIGHT / LEFT**: 編集中の値を 5 ずつ変更（`SW_SIDE` では RIGHT／UP が右、LEFT／DOWN が左）
- **CENTER**: 選択、編集値の保存
- **LEFT**: 項目選択画面では一つ上へ戻る／メインメニューを閉じる

メニュー表示中はモーター出力を 0 にして `nSLEEP` を Low にします。編集画面で LEFT を入力した場合は値を減らしており、キャンセル操作にはなりません。CENTER で確定すると現在の全設定を保存します。

### 設定メニュー

- `GAIN`: 保存可能な互換用パラメーター。**現状の角度推定および PWM 演算では参照されません**
- `NEUTRAL`: 推定角度から差し引く基準角度
- `PWM_MAX` / `PWM_MIN`: 正／負方向の最大 PWM デューティー（%）
- `ANG_MAX` / `ANG_MIN`: 最大 PWM を割り当てる正／負の角度
- `SW_SIDE`: センサー取付側（`LEFT` / `RIGHT`）。角度計算の符号とオフセットに反映
- `RESET`: 全設定と LCD 表示項目を組み込み初期値へ戻して保存
- `RETURN`: 上のメニューへ戻る

### LCD 表示項目

`DISPLAY` では 1 行目と 2 行目について、`ANGLE`、`PWM`、`SENSOR`、`SWITCH`、`ZERO`（実際には `NEUTRAL` 値）、`JOY`、`OBS`、`KAL`、`LPF`、`GYRO`、`ERROR` から選択できます。LCD の幅に合わせて各行は先頭 8 文字に切り詰められます。

## 設定値と `config.json`

`config.json` の読み込みに成功した場合、数値は次の範囲へ補正されます。ファイルが存在しない、JSON が壊れている、または変換できない値を含む場合は、一式を組み込み初期値へ戻して処理を継続します。このフォールバックだけではファイルを作成しません。

| JSON キー | 組み込み初期値 | 設定可能範囲／内容 |
| --- | ---: | --- |
| `GAIN` | `50` | `0` ～ `100`（現状は制御に未使用） |
| `NEUTRAL_ANG` | `30` | `10` ～ `50` 度 |
| `PWM_MAX` | `30` | `1` ～ `100` % |
| `PWM_MIN` | `-30` | `-100` ～ `-1` % |
| `ANGLE_MAX` | `90` | `1` ～ `180` 度 |
| `ANGLE_MIN` | `-90` | `-180` ～ `-1` 度 |
| `SW_IS_RIGHT` | `false` | `false` = LEFT、`true` = RIGHT |
| `line1_setting` | `0` | LCD 1 行目の項目番号（`0` ～ `10`） |
| `line2_setting` | `1` | LCD 2 行目の項目番号（`0` ～ `10`） |

なお、リポジトリ同梱の `config.json` は組み込み初期値とは異なり、`PWM_MAX=50`、`PWM_MIN=-50`、`ANGLE_MAX=30`、`ANGLE_MIN=-30`、`SW_IS_RIGHT=true` です。

## Bluetooth LE 通信

デフォルトでは `TwinHAM_LH` として 100 ms 間隔で Advertising します。サービスと Characteristic は Nordic UART Service 互換です。

| 種別 | UUID / 動作 |
| --- | --- |
| Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX（Central → Pico） | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`、Write / Write Without Response |
| TX（Pico → Central） | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`、Notify |

RX の受信バッファは 128 byte、コマンドキューは最大 16 件です。UTF-8 テキストを受信し、1 回の Write 内にある改行で複数コマンドへ分割します。応答と通知は改行で終端されます。接続中は `DATA` を 200 ms ごとに送信します。

### コマンド

| コマンド | 動作 |
| --- | --- |
| `SET,PWM_MAX,<値>` | `0` ～ `100` に補正して RAM 上の値を変更 |
| `SET,PWM_MIN,<値>` | `-100` ～ `0` に補正して RAM 上の値を変更 |
| `SET,ANG_MAX,<値>` | `0.1` ～ `180` に補正して RAM 上の値を変更 |
| `SET,ANG_MIN,<値>` | `-180` ～ `-0.1` に補正して RAM 上の値を変更 |
| `SET,NEUTRAL,<値>` | `10` ～ `50` に補正して RAM 上の値を変更 |
| `SET,GAIN,<値>` | `0` ～ `100` に補正して RAM 上の値を変更（現状は制御に未使用） |
| `GET,CONFIG` | `CONFIG,PWM_MAX,...,GAIN,...` 形式で現在値を通知 |
| `GET,STATUS` | `STATUS,<角度>,<JOY>,<SW>,<PWM>,<MOTOR>,<SENSOR>` を通知 |
| `SAVE` | 現在値を `config.json` に保存 |
| `MOTOR,STOP` | 遠隔許可を解除し、直ちに PWM を 0、`nSLEEP` を Low にする |
| `MOTOR,ENABLE` | 遠隔許可を再設定する |
| `PING` | `PONG` を通知 |

`SET` は自動保存されません。再起動後も保持するには `SAVE` を送信してください。BLE で許可しても、センサー正常かつメニュー外でなければモーターは駆動しません。一方、現状はゼロ点設定を駆動条件にしていません。

定期通知は次の形式です。

```text
DATA,<angle>,<joystick>,<switch_pressed>,<current_pwm_command>,<ENABLE|STOP>,<OK|NG>
```

`current_pwm_command` は角度から算出したランプ適用前の値です。実際のデューティーは、この値に `switch_gain` を掛けた値になります。

## 停止条件と実装上の注意

- 起動直後と BNO055 初期化失敗時は `nSLEEP` を Low にします。
- 有効なセンサーデータを一度も受信していない間は駆動しません。
- 最後の有効データから 500 ms を超えるとセンサー異常として PWM を 0 にし、`nSLEEP` を Low にします。監視タスクは 1 秒周期のため、実際の検出には追加の遅延があり得ます。
- メニュー表示中、`MOTOR,STOP` 後、制御処理の例外時は PWM を 0 にし、`nSLEEP` を Low にします。
- 一時的にセンサーデータが来ない間は最後に算出した角度を保持します。
- BNO055 の受信は UART RX-idle soft IRQ で処理し、ドライバー内部の受信・送信バッファは各 200 byte です。
- `PWM_DEADBAND` は現在 `0` のため、ソフトウェア上の非ゼロ・デッドバンドはありません。
- `MOTOR_REQUIRE_ZERO`、ゼロスイッチ用の定数、および `GAIN` は定義されていますが、現状の制御フローでは使用されていません。

これらは補助的なソフトウェア機能であり、独立した非常停止回路、過電流保護、機械的な制動装置などのハードウェア安全対策を代替しません。実機では関係法令・規格を確認し、十分なリスク評価と検証を行ってください。

## リポジトリ構成

```text
.
├── main.py                 # 初期化、BLE コマンド、モーター制御、タスク構成
├── app_state.py            # 設定、実行時状態、共通関数
├── bno055.py               # BNO055 UART 通信とパケット解析
├── sensing.py              # 加速度・ジャイロからの角度推定
├── lcd_menu.py             # LCD、ADC ジョイスティック、設定メニュー
├── blue_commu.py           # BLE Advertising、NUS 通信、受信キュー
├── config_store.py         # config.json の読み書きと値の補正
├── config.json             # 同梱の実機設定例
├── ble_settings_LH.json    # 左側用 BLE 設定テンプレート
├── ble_settings_RH.json    # 右側用 BLE 設定テンプレート
└── README.md               # 本ドキュメント
```

## 開発時の確認

通常の CPython ではハードウェア依存モジュールを import できませんが、構文だけなら次のコマンドで確認できます。

```bash
python -m py_compile *.py
```

実機では最低限、次を確認してください。

1. 起動直後、BNO055 異常時、メニュー表示中、BLE 停止指令後に PWM が 0 かつ `nSLEEP` が Low になること
2. GP6 の押下／解放時に約 0.5 秒の出力ランプが意図どおり動作すること
3. `NEUTRAL` と実際の基準姿勢が一致すること
4. 車体の傾きと CW / CCW の回転方向が一致すること
5. 正負それぞれの角度範囲と PWM 上限が意図どおりであること
6. LCD または BLE で保存した設定が再起動後も保持されること
7. BNO055 の切断後、監視処理によってモーターが停止すること
