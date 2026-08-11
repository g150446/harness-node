# HarnessNode / nordic-main ファームウェア運用ガイド

`harness-node` リポジトリで現在メインとなっている、XIAO nRF52840 Sense 向けのジェスチャートリガー式 BLE 音声ファームウェアです。腕の動きを IMU で検知し、録音の開始・停止を自動制御します。

---

## 概要

| 項目 | 内容 |
|------|------|
| ターゲットボード | `xiao_ble/nrf52840/sense`（Seeed XIAO nRF52840 Sense） |
| BLE デバイス名 | `HarnessNode` |
| ブートローダ | MCUboot（Adafruit UF2 ブートローダ経由で 0x27000 に配置） |
| OTA | MCUboot + BLE SMP（MCUmgr） |
| PDM マイク | DMIC（Zephyr DMIC API、RIGHT チャンネル、マイク電源制御あり） |
| IMU | LSM6DS3TR-C（LSM6DSL ドライバ経由、Zephyr センサー API） |
| 音声フォーマット | 16 kHz / 16-bit / モノラル PCM |
| LED 方針 | 起動後は待機時消灯。録音ジェスチャー成立後の録音中のみ赤 |

---

## ファイル構成

```
nordic-main/
├── src/
│   ├── main.c                    # BLE サービス + ジェスチャー検出 + DMIC 制御
│   ├── audio_capture.c/h         # DMIC キャプチャ（16 kHz / 16-bit / モノラル）
│   └── adpcm.c/h                 # ADPCM コーデック（互換用）
├── boards/
│   └── xiao_ble_nrf52840_sense.overlay  # PDM + IMU + マイク電源 + フラッシュパーティション
├── sysbuild/mcuboot/             # MCUboot 設定
├── prj.conf                      # Zephyr 設定（BLE, Audio, MCUmgr）
├── sysbuild.conf                 # SB_CONFIG_BOOTLOADER_MCUBOOT=y
├── pm_static.yml                 # フラッシュパーティション定義
├── build_and_flash.sh            # 初回 UF2 フラッシュスクリプト
└── build_and_package_ota.sh      # OTA バイナリ生成スクリプト（→ ota_update.bin）
```

---

## フラッシュレイアウト

| パーティション | 開始アドレス | 備考 |
|--------------|------------|------|
| Adafruit UF2 ブートローダ | `0x000000` | 書き換え不要 |
| MCUboot | `0x027000` | Adafruit がジャンプするアドレス |
| slot0（稼働中アプリ） | `0x033000` | 署名済みアプリイメージ |
| slot1（OTA 受信バッファ） | `0x085000` | OTA 転送先、MCUboot がスワップ |

---

## BLE サービス仕様

### Battery Service（標準 UUID）

**サービス UUID**: `0000180f-0000-1000-8000-00805f9b34fb`（Bluetooth SIG 標準 Battery Service）

| キャラクタリスティック | UUID | プロパティ | 説明 |
|----------------------|------|-----------|------|
| Battery Level | `00002a19-0000-1000-8000-00805f9b34fb` | Read, Notify | バッテリー残量（0〜100%） |

- nRF Connect や iOS/Android の標準 API で直接読み取り可能
- 1 分ごとに更新。録音停止直後にも即時更新。変化があると Notify が発火する
- 詳細な実装・回路知見は `docs/nrf52_battery_guide.md` を参照

### Audio Service

**サービス UUID**: `00000001-0000-1000-8000-00805f9b34fb`

| キャラクタリスティック | UUID | プロパティ | 説明 |
|----------------------|------|-----------|------|
| TX（送信） | `00000002-0000-1000-8000-00805f9b34fb` | Notify | 音声 PCM パケット / イベントパケット |
| RX（受信） | `00000003-0000-1000-8000-00805f9b34fb` | Write | 制御コマンド |

### RX コマンド（ホスト → ファームウェア）

| バイト値 | 動作 |
|---------|------|
| `0x01` | 録音開始 |
| `0x00` | 録音停止 |

### TX パケット形式（ファームウェア → ホスト）

#### 音声 PCM パケット

```
[seq: 1 byte][0xAA: 1 byte][PCM data: 16-bit LE samples...]
```

- `seq`: シーケンス番号（0–255、ロールオーバー）
- `0xAA`: 音声パケット識別バイト
- `PCM data`: 16-bit リトルエンディアン PCM サンプル列

#### イベントパケット

```
[0x00: 1 byte][0x55: 1 byte][code: 1 byte][optional data: 4 bytes]
```

| コード | イベント名 | オプションデータ | 説明 |
|--------|-----------|----------------|------|
| `0x01` | `recording_start` | なし | 録音開始（ジェスチャートリガー後） |
| `0x02` | `recording_stop` | なし | 録音停止 |
| `0x10` | `motion_active` | x, y, z f32 LE（各 4 byte） | モーション検出開始、xyz 加速度値 |
| `0x11` | `motion_settled` | x, y, z f32 LE + elapsed_ms u32 + avg/peak_speed/distance f32 LE（計 28 bytes） | モーション静定、詳細メトリクス |
| `0x20` | `sleep_enter` | なし | ライトスリープ移行（10 秒無動作） |
| `0x21` | `sleep_wake` | なし | ライトスリープ復帰（モーション検出） |
| `0x30` | `gesture_diag` | stage/reason u8 + value1/2/3 f32 LE | USBなしのジェスチャー内部診断 |

---

## ジェスチャー検出アルゴリズム

この章は運用時の概要です。軸調査の根拠、判定式、状態遷移、全閾値、既知の
制約、実機テスト項目は [屈曲→回内ジェスチャー仕様](flex_pronation_gesture.md)
を参照してください。

### IMU 軸と取り付け方向

Seeed Studio 公式の XIAO nRF52840 Sense KiCad 基板データと ST の LSM6DS3TR-C 軸図を照合すると、基板上のセンサー軸は次の向きになります。

| 軸 | XIAO 基板上の向き | リストバンド装着時の用途 |
|----|------------------|------------------------|
| `+X` | 基板長手方向、USB 端子から離れる向き | 前腕長手軸。回内・回外の回転軸 |
| `+Y` | 基板横方向、5V/GND/3V 側 | 肘屈曲の主な横軸 |
| `+Z` | 部品面から外向き | 部品面を内側に装着するため手首方向 |

資料: [Seeed Studio XIAO nRF52840 Series](https://wiki.seeedstudio.com/XIAO_BLE/)、[LSM6DS3TR-C datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)

### 録音開始トリガー（屈曲 → 回内）

絶対姿勢や重力の向きを使わず、加速度センサーとジャイロの短時間の相対変化を使います。これにより立位、寝た姿勢、走行中の車内で同じ条件を適用できます。

1. 角速度が 200 ms 以上安定している状態を開始姿勢とし、その後1000 msを屈曲開始受付期間とする。この間のX軸先行成分も保持する。
2. **肘屈曲フェーズ**: X 軸に直交する角速度 `sqrt(gyro_y² + gyro_z²)` が35°/s以上になると回転を積分する。
3. 180〜1600 ms の間に、屈曲角 ≥ 35°、ピーク角速度 ≥ 50°/s、加速度エビデンス ≥ 0.5 m/s²を満たす。
4. **回内待ちフェーズ**へ移り、1500 ms 以内の20°/s以上の X 軸角速度を、正負方向別に積算する。
5. X 軸の方向別積分角 ≥ 30°、ピーク角速度 ≥ 55°/s、3 サンプル以上を満たすと `recording_requested = true` とし、録音開始 + `0x01` を送信する。

加速度エビデンスには `abs(|accel| - 1g)` と開始姿勢からの3軸加速度ベクトル差の大きい方を使います。固定の x/y/z 値を要求しないため姿勢依存がなく、ジャイロだけのノイズでも成立しません。

屈曲は Y-Z の合成角、回内は X の正負方向別積分角で評価するため、右手・左手でジャイロの符号が反転しても同じ条件で発動します。自然な連続動作を許容するため、開始受付中のX軸成分と、屈曲開始50 ms後かつ屈曲角8°以上のX軸回転を屈曲成立後へ引き継ぎます。全シーケンスの上限は3000 msです。成立後に腕を戻す動作は、6000 ms以内に現れる逆向きX回転として1回だけ除外します。1200 msのガードは同一動作内の重複防止用であり、腕戻し対策として長時間の全面禁止は行いません。

単一の手首 IMU だけでは、立位・臥位を問わず肘の絶対伸展角を直接観測できません。そのため「伸展状態」は開始前 200 ms の角速度安定で代用し、その後に十分な Y-Z 回転が生じたことを屈曲として判定します。

同様に、左右どちらの手かという設定を持たずに両手対応するため、X 軸回転は正負の両方向を許容します。この構成では回内と同程度の逆方向回転（回外）も区別できません。回内だけに限定するには、装着する手を設定として追加し、手ごとに X 軸角度の符号を選ぶ必要があります。

### 録音停止トリガー

録音中に次の `motion_active` イベントが発生すると `stop_requested = true` となり、DMIC を停止して `0x02` を送信します。

### ライトスリープ

10 秒間 `motion_active` でなく録音中でもない場合、ライトスリープに移行します。

| 状態 | IMU ポーリング | BLE 送信 |
|------|-------------|---------|
| 通常（アクティブ） | 25 ms | — |
| ライトスリープ移行時 | → 50 ms | `0x20 sleep_enter` |
| ライトスリープ中 | 50 ms | — |
| ライトスリープ復帰時 | → 25 ms | `0x21 sleep_wake` |

BLE 接続はスリープ中も維持されます。録音停止後もタイマーはリセットされ、即座にスリープに入ることはありません。

---

## モーション検出パラメータ

### サンプリング / キャリブレーション

| パラメータ | 値 | 説明 |
|-----------|---|------|
| `ACCEL_ODR_HZ` | 416 | 加速度センサ ODR（Hz） |
| `GYRO_ODR_HZ` | 104 | ジャイロ ODR（Hz） |
| `GYRO_FULL_SCALE_DPS` | ±500°/s | ジャイロ測定レンジ |
| `MOTION_SAMPLE_INTERVAL_MS` | 25 | ソフトウェアポーリング間隔（ms） |
| `CALIBRATION_SAMPLES` | 25 | 起動時ベースライン計測サンプル数 |
| `ACTIVITY_WINDOW_SAMPLES` | 4 | アクティビティ判定ウィンドウ（サンプル数） |

### モーション検出しきい値

| パラメータ | 値（m/s²） | 説明 |
|-----------|----------|------|
| `MOTION_ENTRY_ACTIVITY_MS2` | 8.0 | モーション開始判定：ウィンドウ内の活動量 |
| `MOTION_ENTRY_PEAK_MS2` | 2.4 | モーション開始判定：ピーク加速度 |
| `MOTION_SETTLE_ACTIVITY_MS2` | 4.0 | 静定判定：ウィンドウ内の活動量 |
| `MOTION_SETTLE_PEAK_MS2` | 1.4 | 静定判定：ピーク加速度 |
| `MOTION_START_WINDOWS` | 2 | モーション開始に必要な連続ウィンドウ数 |
| `MOTION_SETTLE_WINDOWS` | 2 | 静定判定に必要な連続ウィンドウ数 |
| `BASELINE_ALPHA` | 0.03 | ベースライン更新の指数移動平均係数 |
| `REPORT_COOLDOWN_MS` | 700 | 連続レポートのクールダウン（ms） |

### ジェスチャー判定しきい値

| パラメータ | 値 | 説明 |
|-----------|---|------|
| `GESTURE_QUIET_HOLD_MS` | 200 ms | 開始前に必要な角速度安定時間 |
| `GESTURE_START_ARM_MS` | 1000 ms | 静止成立後に保持する屈曲開始受付時間とX軸先行成分 |
| `GESTURE_FLEX_START_RATE_DPS` | 35°/s | 屈曲フェーズ開始角速度 |
| `GESTURE_FLEX_ANGLE_MIN_DEG` | 35° | Y-Z 合成屈曲角の下限 |
| `GESTURE_FLEX_PEAK_RATE_MIN_DPS` | 50°/s | 屈曲ピーク角速度の下限 |
| `GESTURE_FLEX_ACCEL_EVIDENCE_MIN_MS2` | 0.5 m/s² | 加速度による実動作確認の下限 |
| `GESTURE_FLEX_MIN_DURATION_MS` | 180 ms | 屈曲の最短時間 |
| `GESTURE_FLEX_MAX_DURATION_MS` | 1600 ms | 屈曲の最長時間 |
| `GESTURE_PRONATION_INTEGRATE_RATE_DPS` | 20°/s | X 軸積分対象の最小角速度 |
| `GESTURE_PRONATION_ANGLE_MIN_DEG` | 30° | X 軸回内角の下限 |
| `GESTURE_PRONATION_EARLY_MIN_FLEX_DEG` | 8° | 屈曲中の回内先行積分を許可する最小屈曲角 |
| `GESTURE_PRONATION_EARLY_MIN_MS` | 50 ms | 屈曲中の回内先行積分を許可する最短時間 |
| `GESTURE_PRONATION_PEAK_MIN_DPS` | 55°/s | 回内ピーク角速度の下限 |
| `GESTURE_PRONATION_WINDOW_MS` | 1500 ms | 屈曲成立後の回内待ち時間 |
| `GESTURE_RETURN_SIGNATURE_WINDOW_MS` | 6000 ms | 逆向きXの腕戻しを除外する履歴期間 |
| `GESTURE_SEQUENCE_TIMEOUT_MS` | 3000 ms | 全シーケンスの最大時間 |
| `SLEEP_IDLE_TIMEOUT_MS` | 10000 ms | ライトスリープ移行までの無動作時間 |
| `SLEEP_POLL_INTERVAL_MS` | 50 ms | スリープ中の IMU ポーリング間隔 |

---

## ビルドと OTA

NCS v2.9.2 を使用します。SDK が標準の `/opt/nordic/ncs/v2.9.2` または
`/opt/nordic/ncs/2.9.2` 以外にある場合は、SDK workspace を `NCS_BASE` で
指定してください。`west` が PATH にない場合は、実行可能な nRF Util を
`NRFUTIL` で指定すると SDK Manager のツールチェーン環境を使用できます。

### 初回フラッシュ（MCUboot + アプリを UF2 で書き込み）

```bash
cd nordic-main
./build_and_flash.sh
```

XIAO のリセットボタンをダブルタップして UF2 ブートローダに入ると
（XIAO-SENSE ドライブが出現）、スクリプトが merged UF2 を書き込み、
アプリのUSBシリアル再列挙まで確認します。

### OTA バイナリのビルド

```bash
cd nordic-main
./build_and_package_ota.sh
# → nordic-main/ota_update.bin が生成される
```

OTA バイナリのバージョンは稼働中ファームウェアより新しくする必要があります。`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を更新してからビルドしてください（例: `"0.0.2+0"`）。

### BLE OTA アップデート（2 回目以降）

```bash
cd mac_client
python3 -m venv venv
venv/bin/pip install bleak cbor2 pyserial
venv/bin/python ota_updater.py --device HarnessNode ../nordic-main/ota_update.bin
```

正常終了時の出力例:

```
Scanning for 'HarnessNode'...
Found: <address>
Connected. MTU=244
Upload complete in ~104s
Querying image state...
Setting image test flag...
Image test flag set.
Sending reset command...
Device reset. MCUboot will swap slots on next boot.
Waiting for the updated device to return...
OTA verified: uploaded image is active and confirmed in slot 0 (version=...).
```

---

## Mac クライアント

### xiao_voice_client.py — BLE 録音クライアント

HarnessNode に接続し、`0x01` 受信で WAV 録音を自動開始、`0x02` 受信で自動停止します。motion_active / motion_settled / sleep_enter / sleep_wake イベントも画面表示します。

```bash
cd mac_client
source venv/bin/activate
python3 xiao_voice_client.py
```

録音ファイルは `mac_client/output/xiao_recording_YYYYMMDD_HHMMSS.wav` に保存されます（16 kHz / 16-bit / モノラル）。

### gesture_monitor.py — ジェスチャーモニター

録音機能なし。BLE イベントをタイムスタンプ付きで表示するだけのミニマルモニターです。ジェスチャー動作確認やデバッグに使用します。

```bash
cd mac_client
source venv/bin/activate
python3 gesture_monitor.py
```

表示イベント: `motion_active`（x/y/z）、`motion_settled`（x/y/z + elapsed/peak/dist）、`recording_start`、`recording_stop`、`sleep_enter`、`sleep_wake`

### gesture_classifier.py — オフライン分類器（検証用）

CSV ファイル（`gesture_data.csv`）を読み込み、2 特徴量ベースのジェスチャー分類を実行します。ジェスチャーしきい値のチューニング・検証に使用します。

```bash
cd mac_client
source venv/bin/activate
python3 gesture_classifier.py
```

---

## LED 状態

| 状態 | LED 色 / パターン |
|------|----------------|
| 起動直後 | 白（1 秒点灯） |
| BLE アドバタイジング中 | 消灯 |
| BLE 接続済み（待機中） | 消灯 |
| 単純なモーション検出中 | 消灯 |
| 録音中 | 赤（常時点灯） |

省発光のため、BLE のみで待機している間や単純な `motion_active` 検出中は LED を点灯しません。LED が点灯するのは録音ジェスチャーが成立して `is_recording` に入ったときだけです。リモート未接続でも録音ジェスチャー成立後は赤点灯を維持し、停止ジェスチャーで消灯します。
