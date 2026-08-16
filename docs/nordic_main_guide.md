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
 制約、実機テスト項目は [水平回内→屈曲・回外→垂直静止仕様](flex_pronation_gesture.md)
 を参照してください。

### IMU 軸と取り付け方向

Seeed Studio 公式の XIAO nRF52840 Sense KiCad 基板データと ST の LSM6DS3TR-C 軸図を照合すると、基板上のセンサー軸は次の向きになります。

| 軸 | XIAO 基板上の向き | リストバンド装着時の用途 |
|----|------------------|------------------------|
| `+X` | 基板長手方向、USB 端子から離れる向き | 前腕を横切る方向。復路の肘屈曲の合成判定 |
| `+Y` | 基板横方向、5V/GND/3V 側 | 前腕に沿う方向。回内・回外と水平／垂直判定 |
| `+Z` | 部品面から外向き | 手のひら上向き水平の開始判定 |

資料: [Seeed Studio XIAO nRF52840 Series](https://wiki.seeedstudio.com/XIAO_BLE/)、[LSM6DS3TR-C datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)

### 録音開始トリガー（水平回内 → 屈曲・回外 → 垂直静止）

XIAOをリストバンドの掌側に置き、部品面を皮膚側、基板X軸を前腕と直交、Y軸を
前腕に沿わせる。右手・左手とUSB端子方向の違いは、往路と復路の相対符号で吸収する。

1. 手のひら上向き水平で、補正Z比0.90以上、Y比絶対値0.17以下（±10°）、角速度19°/s以下を200 ms連続して満たすと1秒間armする。
2. 前腕を水平±10°に保ったまま回内する。180〜2000 msでY軸積分角30°以上、Y軸ピーク55°/s以上、往路中の最大`|y_ratio|`0.17以下を要求する。
3. 750 ms以内に回転停止またはY軸回転方向の反転を検出し、肘屈曲＋回外へ切り返す。
4. 復路にもX-Z合成角60°以上などの実動作条件を要求し、Y軸積分を往路と逆符号にする。復路の加速度変位と有効腕長15 cmの円弧距離を融合し、推定移動距離5 cm以上を要求する。
5. `abs(accel_y) / |accel| >= 0.94`、角速度19°/s以下、線形加速度1.2 m/s²以下を500 ms連続して満たすと`recording_requested = true`とし、録音開始＋`0x01`を送信する。

最終姿勢は垂直から20°まで許容する。保持は復路終了から1.5秒以内、全動作は5秒以内に
成立する必要がある。距離推定とBLE診断を含む詳細は
`docs/flex_pronation_gesture.md`を参照する。

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
| `GESTURE_START_ARM_MS` | 1000 ms | 静止成立後の開始受付時間 |
| `GESTURE_START_PALM_UP_Z_MIN_RATIO` | 0.90 | 手のひら上向き開始時の補正Z比下限 |
| `GESTURE_START_Y_MAX_RATIO` | 0.17 | 開始時のY比絶対値上限（±10°） |
| `GESTURE_HORIZONTAL_Y_MAX_RATIO` | 0.17 | 往路中の水平維持上限（±10°） |
| `GESTURE_ROLL_START_RATE_DPS` | 35°/s | 往路のY軸開始角速度 |
| `GESTURE_TRANSVERSE_START_RATE_DPS` | 35°/s | 復路のX-Z合成開始角速度 |
| `GESTURE_TRANSVERSE_ANGLE_MIN_DEG` | 60° | 復路のX-Z合成角下限 |
| `GESTURE_TRANSVERSE_PEAK_MIN_DPS` | 50°/s | 復路ピーク角速度下限 |
| `GESTURE_ACCEL_EVIDENCE_MIN_MS2` | 0.5 m/s² | 復路の加速度による実動作確認下限 |
| `GESTURE_PHASE_MIN_DURATION_MS` | 180 ms | 往路・復路の最短時間 |
| `GESTURE_OUTBOUND_MAX_DURATION_MS` | 2000 ms | 往路の最長時間 |
| `GESTURE_RETURN_MAX_DURATION_MS` | 2500 ms | 復路の最長時間 |
| `GESTURE_ROLL_ANGLE_MIN_DEG` | 30° | 回内・回外のY軸積分角下限 |
| `GESTURE_ROLL_PEAK_MIN_DPS` | 55°/s | 回内・回外のY軸ピーク下限 |
| `GESTURE_FINAL_Y_MIN_RATIO` | 0.94 | 最終Y比絶対値（垂直から20°以内） |
| `GESTURE_TURNAROUND_TIMEOUT_MS` | 750 ms | 切り返し期限 |
| `GESTURE_DISTANCE_MIN_M` | 0.05 m | 加速度・ジャイロ融合距離の下限 |
| `GESTURE_EFFECTIVE_ARM_LENGTH_M` | 0.15 m | 円弧距離推定に使う有効腕長 |
| `GESTURE_DISTANCE_LP_TAU_S` | 0.55 s | 車両加速度を追従除去する時定数 |
| `GESTURE_FINAL_QUIET_RATE_DPS` | 19°/s | 垂直保持中の角速度上限 |
| `GESTURE_FINAL_LINEAR_ACCEL_MAX_MS2` | 1.2 m/s² | 垂直保持中の線形加速度上限 |
| `GESTURE_FINAL_HOLD_MS` | 500 ms | 垂直姿勢の連続保持時間 |
| `GESTURE_FINAL_HOLD_TIMEOUT_MS` | 1500 ms | 復路終了後の保持成立期限 |
| `GESTURE_SEQUENCE_TIMEOUT_MS` | 5000 ms | 全シーケンス期限 |
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

### gesture_validator.py — 水平回内ジェスチャー検証

試行ごとにカウントダウンと Ping 音を合図に、水平回内→屈曲・回外→垂直静止を
対話検証する。条件ごとの `[OK]` / `[NG]` / `[--]` を表示し、生の診断ログは JSON へ保存する。

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 1 \
  --json-output /private/tmp/harness-node-volar-sequence.json
venv/bin/python gesture_validator.py --self-test
```

詳細は `docs/flex_pronation_gesture.md` を参照。

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
