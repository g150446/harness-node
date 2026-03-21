# nrf52-handy ファームウェア 運用ガイド

XIAO nRF52840 Sense 向けのジェスチャートリガー式 BLE 音声ファームウェアです。腕の動きを IMU で検知し、録音の開始・停止を自動制御します。

---

## 概要

| 項目 | 内容 |
|------|------|
| ターゲットボード | `xiao_ble/nrf52840/sense`（Seeed XIAO nRF52840 Sense） |
| BLE デバイス名 | `XIAOVoice` |
| ブートローダ | MCUboot（Adafruit UF2 ブートローダ経由で 0x27000 に配置） |
| OTA | MCUboot + BLE SMP（MCUmgr） |
| PDM マイク | DMIC（Zephyr DMIC API、RIGHT チャンネル、マイク電源制御あり） |
| IMU | LSM6DS3TR-C（LSM6DSL ドライバ経由、Zephyr センサー API） |
| 音声フォーマット | 16 kHz / 16-bit / モノラル PCM |

---

## ファイル構成

```
nrf52-handy/
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
| `0x10` | `motion_active` | z-axis 加速度（f32 LE） | モーション検出開始、z 軸加速度値 |
| `0x11` | `motion_settled` | z-axis 加速度（f32 LE） | モーション静定、z 軸加速度値 |

---

## ジェスチャー検出アルゴリズム

### 録音開始トリガー（3 条件の AND）

ジェスチャー判定はファームウェア内で完結します。以下の 3 条件がすべて成立したとき `recording_requested = true` となり、DMIC 録音を開始して `0x01` イベントを送信します。

| 条件 | 判定内容 | しきい値（通常） | しきい値（sleep wake 直後） |
|------|---------|----------------|--------------------------|
| 1. `motion_active` の z 値 | 腕の姿勢チェック | `\|z\| < 5.0 m/s²` | `z > 6.0 m/s²`（腕が持ち上がっている方向のみ） |
| 2. `motion_settled` の z 値 | 静定時に腕が上がっている | `z ≥ 8.0 m/s²` | 同左 |
| 3. 経過時間 | active → settled の間隔 | `≤ 2000 ms` | 同左 |

> **sleep wake 直後の挙動**: スリープ解除後の最初の `motion_settled` のみ条件 1 を `z > 6.0` に差し替えます。
> 腕を持ち上げる動き（z 正方向が大きい）以外は録音トリガーにならず、誤検知を抑制します。
> 2 回目以降のモーションからは通常条件（`|z| < 5.0`）に戻ります。

**シーケンス例（腕を持ち上げるジェスチャー）:**

1. 腕が横方向に動き始める → `motion_active` 検出（通常: z が ±5.0 以内 / wake 直後: z > 6.0）
2. 腕を持ち上げて静止する → `motion_settled` 検出（z ≥ 8.0）
3. 1→2 の経過時間が 2000 ms 以内
4. 3 条件成立 → 録音開始 + `0x01` 送信

### 録音停止トリガー

録音中に次の `motion_active` イベントが発生すると `stop_requested = true` となり、DMIC を停止して `0x02` を送信します。

---

## モーション検出パラメータ

### サンプリング / キャリブレーション

| パラメータ | 値 | 説明 |
|-----------|---|------|
| `ACCEL_ODR_HZ` | 52 | 加速度センサ ODR（Hz） |
| `MOTION_SAMPLE_INTERVAL_MS` | 25 | ソフトウェアポーリング間隔（ms） |
| `CALIBRATION_SAMPLES` | 25 | 起動時ベースライン計測サンプル数 |
| `ACTIVITY_WINDOW_SAMPLES` | 4 | アクティビティ判定ウィンドウ（サンプル数） |

### モーション検出しきい値

| パラメータ | 値（m/s²） | 説明 |
|-----------|----------|------|
| `MOTION_ENTRY_ACTIVITY_MS2` | 8.0 | モーション開始判定：ウィンドウ内の活動量 |
| `MOTION_ENTRY_PEAK_MS2` | 2.4 | モーション開始判定：ピーク加速度 |
| `MOTION_SETTLE_ACTIVITY_MS2` | 2.0 | 静定判定：ウィンドウ内の活動量 |
| `MOTION_SETTLE_PEAK_MS2` | 0.8 | 静定判定：ピーク加速度 |
| `MOTION_START_WINDOWS` | 2 | モーション開始に必要な連続ウィンドウ数 |
| `MOTION_SETTLE_WINDOWS` | 4 | 静定判定に必要な連続ウィンドウ数 |
| `BASELINE_ALPHA` | 0.03 | ベースライン更新の指数移動平均係数 |
| `REPORT_COOLDOWN_MS` | 700 | 連続レポートのクールダウン（ms） |

### ジェスチャー判定しきい値

| パラメータ | 値 | 説明 |
|-----------|---|------|
| `GESTURE_ACTIVE_Z_ABS_MAX_MS2` | 5.0 m/s² | motion_active 時の z 絶対値上限（通常） |
| `GESTURE_WAKE_ACTIVE_Z_MIN_MS2` | 6.0 m/s² | motion_active 時の z 下限（sleep wake 直後のみ） |
| `GESTURE_SETTLE_Z_MIN_MS2` | 8.0 m/s² | motion_settled 時の z 軸下限 |
| `GESTURE_WINDOW_MS` | 2000 ms | active → settled の最大許容時間 |

---

## ビルドと OTA

### 初回フラッシュ（MCUboot + アプリを UF2 で書き込み）

```bash
cd nrf52-handy
./build_and_flash.sh
```

XIAO のリセットボタンをダブルタップして UF2 ブートローダに入ると（XIAO-SENSE ドライブが出現）、スクリプトが自動でフラッシュします。

### OTA バイナリのビルド

```bash
cd nrf52-handy
./build_and_package_ota.sh
# → nrf52-handy/ota_update.bin が生成される
```

OTA バイナリのバージョンは稼働中ファームウェアより新しくする必要があります。`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を更新してからビルドしてください（例: `"0.0.2+0"`）。

### BLE OTA アップデート（2 回目以降）

```bash
cd mac_client
source venv/bin/activate
python3 ota_updater.py --device XIAOVoice ../nrf52-handy/ota_update.bin
```

正常終了時の出力例:

```
Scanning for 'XIAOVoice'...
Found: <address>
Connected. MTU=244
Upload complete in ~104s
Querying image state...
Setting image test flag...
Image test flag set.
Sending reset command...
Device reset. MCUboot will swap slots on next boot.
```

---

## Mac クライアント

### nrf52_voice_client.py — BLE 録音クライアント

XIAOVoice に接続し、`0x01` 受信で WAV 録音を自動開始、`0x02` 受信で自動停止します。motion_active / motion_settled イベントと z 値も画面表示します。

```bash
cd mac_client
source venv/bin/activate
python3 nrf52_voice_client.py
```

録音ファイルは `mac_client/output/nrf52voice_YYYYMMDD_HHMMSS.wav` に保存されます（16 kHz / 16-bit / モノラル）。

### gesture_monitor.py — ジェスチャーモニター

録音機能なし。BLE イベントをタイムスタンプ付きで表示するだけのミニマルモニターです。ジェスチャー動作確認やデバッグに使用します。

```bash
cd mac_client
source venv/bin/activate
python3 gesture_monitor.py
```

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
| BLE アドバタイジング中 | 青（500 ms ON / 1500 ms OFF 点滅） |
| BLE 接続済み | 緑（常時点灯） |
| 録音中 | 赤（常時点灯） |
