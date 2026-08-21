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
| IMU | LSM6DS3TR-C（加速度 ODR 416 Hz。ジャイロは無効） |
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
├── build_and_package_ota.sh      # OTA バイナリ生成スクリプト（→ ota_update.bin）
└── AGENTS.md                     # エージェント向け board / ビルド注意
```

---

## ビルド手順とよくある失敗

### 正しいボードターゲット

| 正しい | 誤り（よくある） |
|--------|------------------|
| **`xiao_ble/nrf52840/sense`** | `xiao_ble/nrf52840`（`/sense` なし） |
| | `xiao_ble` のみ |

- NCS 上の identifier: `zephyr/boards/seeed/xiao_ble/xiao_ble_nrf52840_sense.yaml`
- アプリ overlay は board 名に紐づく:
  `boards/xiao_ble_nrf52840_sense.overlay`
  → **`/sense` 付きターゲットのときだけ**自動適用される
- overlay が載らないと `imu0`・マイク電源・バッテリ ADC・slot パーティションが
  DT 上に存在せず、`main.c` や flash_map 周りで **未定義マクロの嵐**になる。
  これは C ロジックのバグではなく **ボード指定ミス**である

### 推奨コマンド

```bash
cd nordic-main
./build_and_package_ota.sh    # sysbuild + 署名 OTA → ota_update.bin（書込みなし）
./build_and_flash.sh          # 初回 UF2 向け（ある場合）
```

手動 west（スクリプトと同等）:

```bash
# NCS v2.9.2 の west ワークスペースで実行すること
west build -p always --sysbuild -b xiao_ble/nrf52840/sense \
  /path/to/harness-node/nordic-main \
  --build-dir /path/to/harness-node/nordic-main/build
```

- **必ず** `--sysbuild`（`sysbuild.conf` で MCUboot 有効）
- sysbuild なしのアプリ単体ビルドはパーティション / MCUmgr 前提が崩れやすい
- NCS: **v2.9.2**（`NCS_BASE` でパス上書き可）

### 失敗の切り分け

| 症状 | 原因 | 対処 |
|------|------|------|
| `DT_N_ALIAS_imu0_*` / `zephyr_user` / mic GPIO 未定義 | board に `/sense` がない | `xiao_ble/nrf52840/sense` |
| `slot0_partition` / flash_map 関連 | board 誤り or sysbuild なし | sense + `--sysbuild`、必要なら `-p always` |
| `west: unknown command "build"` | NCS 外で west を実行 | NCS ルートへ移動 or 付属スクリプト |
| `ccache: command not found` | ツールチェーン PATH 不足 | NCS toolchain の `bin` を PATH へ |

エージェント向けの短縮版はリポジトリ直下 `AGENTS.md` / `CLAUDE.md`、
および `nordic-main/AGENTS.md` を参照。

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
制約、実機テスト項目は [シェイク→掌上→挙上→掌下静止仕様](flex_pronation_gesture.md)
を参照してください。

### IMU 軸と取り付け方向

Seeed Studio 公式の XIAO nRF52840 Sense KiCad 基板データと ST の LSM6DS3TR-C 軸図を照合すると、基板上のセンサー軸は次の向きになります。

| 軸 | XIAO 基板上の向き | リストバンド装着時の用途 |
|----|------------------|------------------------|
| `+X` | 基板長手方向、USB 端子から離れる向き | 前腕を横切る方向 |
| `+Y` | 基板短い辺に平行（5V/GND/3V 側） | 前腕に沿う＝回内・回外軸 |
| `+Z` | 部品面から外向き | 基板水平（\|Z 比\|）と掌向きの相対判定 |

資料: [Seeed Studio XIAO nRF52840 Series](https://wiki.seeedstudio.com/XIAO_BLE/)、[LSM6DS3TR-C datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf)

### 録音開始トリガー（シェイク → 掌上 → 挙上 → 掌下静止）

XIAOをリストバンドの手の甲側に置き、部品面を皮膚側、基板X軸を前腕と直交、Y軸を
前腕に沿わせる。右手・左手とUSB端子方向の違いは、シェイク後の相対符号で吸収する。

1. 甲側装着で手のひら下向き、Z絶対比0.80以上の水平姿勢で短く1回シェイクする。重力直交の線形加速度について、500 ms窓の峰間が5.0 m/s²以上かつ`|平均| < 0.4 × 峰間`なら成立。持続する同じ符号の横Gでは不成立。
2. シェイク後1.5秒以内に掌上。基準はシェイク窓開始時の重力LP。3D角20°以上、phi 12°以上、Z比0.35以上の変化、またはZ符号反転で成立。成立時に基準を取り直す。
3. 重力LPで姿勢成分を除いた線形加速度を現在の重力方向へ投影し、上向き加速パルスを検出する（掌向きは見ない。物理動作の目安は約5 cm上昇）。
4. holdは掌上基準からの反転（phi 20° / Δz 0.50 / Z符号）が取れてから開始する。線形加速度RMS ≤ 2.5 m/s² が静止を示した時点の重力方向を固定し、そこからの角度差10°以内を500 ms維持すると録音を開始する（掌上後 4 s 以内）。

回外・最終仰角帯・甲上条件は要求しない。詳細は`docs/flex_pronation_gesture.md`を参照する。

### 録音停止トリガー

録音開始時点の重力 LP（掌下）を基準に保存する。開始後 1200 ms を過ぎたあと、シェイク後掌上と同じ反転（3D 角 ≥ 20°、phi ≥ 12°、Z 比変化 ≥ 0.35、または Z 符号反転、かつ `|a|` が 7.5–12.5 m/s²）で `stop_requested = true` となり、DMIC を停止して `0x02` を送信する。手を下ろすだけでは停止しない。`motion_active` は睡眠タイマーとイベント通知用に残し、録音停止には使わない。ホスト `0x00` とシリアル `'s'` による停止は従来どおり。

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
| `GESTURE_START_PALM_UP_Z_MIN_RATIO` | 0.80 | 開始時の基板水平（\|Z比\|） |
| `GESTURE_SHAKE_WINDOW_SAMPLES` | 20 | シェイク窓（約500 ms） |
| `GESTURE_SHAKE_PTP_MIN_MS2` | 5.0 m/s² | 重力直交成分の峰間 |
| `GESTURE_SHAKE_MEAN_RATIO_MAX` | 0.4 | \|平均\| / 峰間 の上限 |
| `GESTURE_SHAKE_AXIS_MIN_MS2` | 2.0 m/s² | シェイク軸固定の直交成分下限 |
| `GESTURE_QUIET_ACCEL_MS2` | 3.0 m/s² | 静止判定の線形加速度上限 |
| `GESTURE_PRONATION_MIN_DEG` | 20° | hold 反転の重力phi |
| `GESTURE_PRONATION_Z_RATIO_DONE` | 0.50 | hold 反転のZ比変化 |
| `GESTURE_PRONATION_Z_SIGN_MIN_MS2` | 2.0 m/s² | Z符号反転と認める\|az\|下限 |
| `GESTURE_OUTBOUND_MIN_DEG` | 12° | シェイク後掌上のXZ phi |
| `GESTURE_OUTBOUND_TILT_MIN_DEG` | 20° | シェイク後掌上の重力3D角 |
| `GESTURE_OUTBOUND_Z_RATIO_DONE` | 0.35 | シェイク後掌上のZ比変化 |
| `GESTURE_PHASE_MIN_DURATION_MS` | 120 ms | 掌上フェーズ最短時間 |
| `GESTURE_OUTBOUND_MAX_DURATION_MS` | 1500 ms | シェイク後の掌上期限 |
| `GESTURE_LIFT_ACCEL_MIN_MS2` | 0.40 m/s² | 上向き加速パルス下限 |
| `GESTURE_LIFT_BRAKE_MIN_MS2` | 0.15 m/s² | 逆向き減速パルス下限 |
| `GESTURE_LIFT_POS_IMPULSE_MIN_MS` | 0.04 m/s | 正インパルス下限 |
| `GESTURE_LIFT_NEG_IMPULSE_MIN_MS` | 0.015 m/s | 負インパルス下限 |
| `GESTURE_LIFT_BRAKE_RATIO_MIN` | 0.05 | 減速/加速インパルス比下限 |
| `GESTURE_LIFT_PULSE_MIN_MS` / `MAX_MS` | 150 / 1800 ms | 双極パルスの時間窓 |
| `GESTURE_LIFT_FINAL_TILT_MAX_DEG` | 10° | 静止開始時の重力方向からの保持中姿勢差上限 |
| `GESTURE_FINAL_STILL_RMS_MS2` | 2.5 m/s² | 4サンプル静止RMS上限 |
| `GESTURE_FINAL_HOLD_MS` | 500 ms | 最終静止保持時間 |
| `GESTURE_GRAVITY_LP_TAU_S` | 0.30 s | 重力推定の低通時定数 |
| `GESTURE_FINAL_HOLD_TIMEOUT_MS` | 4000 ms | 掌上成立後の最終成立期限 |
| `GESTURE_SEQUENCE_TIMEOUT_MS` | 6000 ms | 全シーケンス期限 |
| `GESTURE_RETRIGGER_BLOCK_MS` | 1200 ms | 開始直後の停止抑制 / 停止後の再開始抑制 |
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

### gesture_validator.py — シェイク→掌上→挙上→掌下静止ジェスチャー検証

試行ごとにカウントダウンと Ping 音を合図に、シェイク→掌上→挙上→掌下で0.5秒静止、
続けて掌上で録音終了するかを対話検証する。条件ごとの `[OK]` / `[NG]` / `[--]` を表示し、
生の診断ログは JSON へ保存する。GO後の判定時間は既定15秒。開始後はホスト `0x00` を送らず、
掌上の `recording_stop` を待つ。

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 1 --window 15 \
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
