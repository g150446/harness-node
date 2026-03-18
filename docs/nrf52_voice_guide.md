# nrf52-voice 保守・運用ガイド

XIAO nRF52840 Sense 向け音声+モーション検出+OTA 統合ファームウェア `nrf52-voice`（BLE デバイス名: `VoiceBridge52`）の保守・運用に必要な情報をまとめます。

---

## 概要

| 項目 | 内容 |
|------|------|
| ターゲットボード | `xiao_ble/nrf52840/sense`（Seeed XIAO nRF52840 Sense） |
| BLE デバイス名 | `VoiceBridge52` |
| ブートローダ構成 | Adafruit UF2 (0xF4000) + MCUboot (0x27000) |
| OTA トランスポート | BLE SMP（MCUmgr） |
| PDM マイク | MSM261D3526H1CPM（RIGHT チャンネル、P1.10 電源制御） |
| マイク電源 | 録音時のみ ON（GPIO P1.10）、アイドル時は OFF |
| IMU | LSM6DS3TR-C（Zephyr LSM6DSL ドライバ互換） |
| コンソール | USB CDC ACM（115200 baud） |
| 音声フォーマット | 16kHz / 16bit / モノラル PCM |
| 転送速度（OTA） | 約 5.6 KB/s（227 KB イメージで約 39 秒） |
| ウォッチドッグ | 5秒タイムアウト（イメージ確定後にアーム） |
| 録音トリガー | Mac クライアントが TILT / WAKEUP / MOTION ACTIVE を組み合わせて BLE write を送信 |

---

## ファイル構成

```
nrf52-voice/
├── CMakeLists.txt                      # PM_STATIC_YML_FILE を find_package 前に設定
├── prj.conf                            # Zephyr 設定（Audio, BLE, MCUmgr, WDT, HWINFO）
├── sysbuild.conf                       # SB_CONFIG_BOOTLOADER_MCUBOOT=y
├── pm_static.yml                       # フラッシュパーティション定義
├── build_and_flash.sh                  # 初回 UF2 フラッシュスクリプト
├── build_and_package_ota.sh            # OTA バイナリ生成スクリプト
├── src/
│   ├── main.c                          # BLE サービス・モーション検出・音声制御・WDT
│   ├── audio_capture.c / .h            # PDM マイク入力（Zephyr DMIC API）
│   └── adpcm.c / .h                    # ADPCM コーデック（予備）
├── boards/
│   └── xiao_ble_nrf52840_sense.overlay # PDM ピン・マイク電源GPIO・フラッシュパーティション
└── sysbuild/
    └── mcuboot/
        ├── prj.conf
        └── boards/
            ├── xiao_ble_nrf52840_sense.conf
            └── xiao_ble_nrf52840_sense.overlay
```

---

## フラッシュレイアウト

```
0x000000  Adafruit MBR / SoftDevice相当  ~156 KB  (上書き不可)
0x027000  MCUboot                          48 KB  ← Adafruit がここにジャンプ
0x02f000  mcuboot_pad                       0.5 KB
0x033000  slot0_partition (primary)        328 KB  ← 稼働中ファームウェア
0x085000  slot1_partition (secondary)      328 KB  ← OTA 書き込み先
0x0ec000  storage_partition (settings)      32 KB  ← イメージ確認状態を保存
0x0f4000  Adafruit UF2 bootloader           48 KB  (上書き不可)
```

---

## BLE サービス仕様

### Audio Service (`00000001-0000-1000-8000-00805f9b34fb`)

| Characteristic | UUID (末尾 2 bytes) | プロパティ | 内容 |
|----------------|---------------------|-----------|------|
| Audio TX | `...0002...` | Notify | PCM 音声パケット（16kHz/16bit/mono） |
| Audio RX | `...0003...` | Write | `0x01` = 録音開始 / `0x00` = 録音停止 |

**PCM パケット形式:**

```
Byte 0: シーケンス番号 (0-255, ロールオーバー)
Byte 1: 0xAA（同期バイト）
Bytes 2..: 16bit LE PCM サンプル（最大 200 バイト = 100 サンプル）
```

### Motion Service (`00000010-0000-1000-8000-00805f9b34fb`)

| Characteristic | UUID (末尾 2 bytes) | プロパティ | 内容 |
|----------------|---------------------|-----------|------|
| Motion TX | `...0011...` | Notify | モーションイベント通知 |
| Build Info | `...0012...` | Read | ビルドタイムスタンプ文字列 |
| Wakeup TX | `...0013...` | Notify | IMU ハードウェア wakeup 通知 |
| Tilt TX | `...0014...` | Notify | IMU tilt 通知（`FUNC_SRC1` ミラー） |

**Motion TX パケット形式（18 バイト）:**

```
Byte 0:    イベントタイプ (0x01 = motion active / 0x00 = motion settled)
Byte 1:    モーション検出累計カウント
Bytes 2-5:   activity (float LE, m/s²) — 直近ウィンドウのサンプル間差分合計
Bytes 6-9:   peak (float LE, m/s²) — 直近ウィンドウのピーク差分
Bytes 10-13: elapsed_ms (uint32 LE, 前回のモーションからの経過時間)
Bytes 14-17: z_excursion_peak (float LE, m/s²) — モーション開始からの Z 軸最大偏差
```

**Wakeup TX パケット形式（2 バイト）:**

```
Byte 0: axes    bit2=Z, bit1=Y, bit0=X（閾値超過した軸）
Byte 1: count   累積 wakeup 数（8bit ロールオーバー）
```

**Tilt TX パケット形式（2 バイト）:**

```
Byte 0: tilt_src  LSM6DS3TR-C の FUNC_SRC1 値をそのまま通知（bit5 = tilt_ia）
Byte 1: count     累積 tilt 数（8bit ロールオーバー）
```

---

## 録音制御

録音の開始・停止そのものは **BLE クライアントからの Audio RX write** で行う。ただし現在の `mac_client/nrf52_voice_client.py` は IMU 通知を解釈して、自動的に `0x01` / `0x00` を送る。

```
BLE write 0x01 → mic_power_on() (P1.10 HIGH, 50ms 待ち)
              → audio_capture_start() (DMIC_TRIGGER_START, DC 収束待ち)
             → Audio TX 通知 開始

BLE write 0x00 → audio_capture_stop() (DMIC_TRIGGER_STOP)
             → mic_power_off() (P1.10 LOW)

BLE 切断時   → audio_capture_stop()
              → mic_power_off()
```

### Mac クライアントの自動録音ルール

1. `MOTION ACTIVE` 中に最初の `WAKEUP` が来ると `DOUBLE_CLENCH` とみなす。
2. `TILT` 通知の `since_wakeup < 2000ms` なら「有効な TILT」とみなす。
3. 直近 `2000ms` 以内に `DOUBLE_CLENCH` があり、かつ有効な `TILT` が来たら録音開始。
4. 録音開始後 `5000ms` は停止トリガーを無視する。
5. 録音開始後 `2000ms` 未満では最低録音時間のため停止しない。
6. `5000ms` 経過後に `MOTION ACTIVE` が来ると録音停止。

### 運用上の注意

- 手動録音 (`r` / `s`) は引き続き利用できる。
- `MOTION ACTIVE` 停止に変えたため、録音終了動作の確認では「軽く持ち上げて即終了」ではなく、5 秒以上録音した後に明確なモーションを入れる。
- 切断時は最低録音時間に関係なく強制停止して WAV をクローズする。

---

## モーション検出パラメータ

`src/main.c` の定数で調整可能：

| 定数 | デフォルト値 | 説明 |
|------|------------|------|
| `MOTION_ENTRY_ACTIVITY_MS2` | 8.0 | モーション開始判定のアクティビティ閾値（m/s²） |
| `MOTION_ENTRY_PEAK_MS2` | 2.4 | モーション開始判定のピーク閾値 |
| `MOTION_CONTINUE_ACTIVITY_MS2` | 4.0 | モーション継続判定の閾値 |
| `MOTION_SETTLE_ACTIVITY_MS2` | 0.9 | 停止判定のアクティビティ閾値 |
| `MOTION_SETTLE_PEAK_MS2` | 0.35 | 停止判定のピーク閾値 |
| `MOTION_START_WINDOWS` | 2 | 開始判定に必要な連続ウィンドウ数 |
| `MOTION_SETTLE_WINDOWS` | 4 | 停止判定に必要な連続ウィンドウ数 |
| `ACCEL_ODR_HZ` | 52 | IMU サンプリングレート（Hz） |
| `MOTION_SAMPLE_INTERVAL_MS` | 100 | モーション特徴量更新の周期（ms） |
| `EVENT_POLL_INTERVAL_MS` | 25 | Wakeup / Tilt レジスタ確認周期（ms） |

---

## Wakeup / Tilt 検出

LSM6DS3TR-C のハードウェア wakeup / tilt 検出を I2C 直接アクセスで設定する。`25ms` ごとにイベント系レジスタを確認し、`100ms` ごとにモーション特徴量を更新する。

**レジスタ設定（`configure_wakeup_detection()`）:**

| レジスタ | アドレス | 値 | 内容 |
|----------|----------|----|------|
| `TAP_CFG` | 0x58 | `0x80` | INTERRUPTS_ENABLE=1（wakeup イベント有効化） |
| `WAKE_UP_THS` | 0x5B | `0x04` | WK_THS=4 → 4 × (2g/64) = 125mg 閾値 |
| `WAKE_UP_DUR` | 0x5C | `0x00` | 最短 wakeup 持続（1 サンプル @ 26Hz ≈ 38ms） |
| `CTRL10_C` | 0x19 | `0x0C` | `FUNC_EN=1`, `TILT_EN=1` |
| `MD1_CFG` | 0x5E | `0x22` | `INT1_WU=1`, `INT1_TILT=1` |

**ポーリングループ（25ms 毎）:**

```
check_wakeup_src() → WAKE_UP_SRC レジスタ読み出し
  → WU_IA(bit3)=1 なら axes(bit2:Z, bit1:Y, bit0:X) を通知

check_tilt_src() → FUNC_SRC1 レジスタ読み出し
  → TILT_IA(bit5)=1 なら Tilt TX に通知
```

---

## ジェスチャー判別

ジェスチャー判別は **Mac クライアント（Python）側** で完結する。ファームウェアは Motion TX / Wakeup TX / Tilt TX の生データを送る。

### 判別アルゴリズム

`MOTION ACTIVE` / `WAKEUP` / `TILT` の到着順で逐次評価する：

```
MOTION ACTIVE 中に最初の WAKEUP が届いた
  → その時点で DOUBLE_CLENCH（ダブルクレンチ）

TILT が届き、直前 WAKEUP から 2000ms 未満
  → 有効な TILT として記録

有効な TILT の時点で、直前 2000ms 以内に DOUBLE_CLENCH がある
  → 自動録音開始

録音開始から 5000ms 経過後に MOTION ACTIVE
  → 自動録音停止
```

**ログの補助情報**

- `since_wakeup`: TILT 受信時点から直近 WAKEUP までの経過時間
- `since_tilt`: DOUBLE_CLENCH 判定時点から直近の有効な TILT までの経過時間
- `N/A`: その条件に使えるイベントが時間窓内にまだ無いことを表す

**現場でよく見るログ**

```text
[WAKEUP] axes=ZYX count=61
[TILT] active=YES src=0x20 count=172 since_wakeup=136ms
[DOUBLE_CLENCH] count=12 since_tilt=412ms
[AUTO] Qualified tilt matched recent DOUBLE_CLENCH (since_double_clench=412ms) → start recording
[AUTO] MOTION ACTIVE during gesture recording ignored (grace 1834ms/5000ms)
[AUTO] MOTION ACTIVE detected after grace period → stop recording
```

### データ収集ツール

`mac_client/gesture_collector.py` で2ジェスチャーの比較データを収集できる：

```bash
cd mac_client
source venv/bin/activate
python3 gesture_collector.py
```

サマリーに `wakeups_after` 統計が表示され、閾値調整に利用できる。

### 主要定数（`mac_client/nrf52_voice_client.py`）

```python
TILT_WAKEUP_MAX_MS = 2000
DOUBLE_CLENCH_TILT_MAX_MS = 2000
GESTURE_EVENT_RETENTION_S = 5.0
RECORDING_WAKEUP_GRACE_MS = 5000
MIN_RECORDING_DURATION_MS = 2000
```

---

## 保守・運用チェックリスト

### ビルド

```bash
cd /opt/nordic/ncs/v2.9.2
/opt/nordic/ncs/toolchains/b8efef2ad5/bin/python3 -m west build -p always --sysbuild \
  -b xiao_ble/nrf52840/sense /Users/g150446/projects/voice-harness/voice-bridge-ble/nrf52-voice \
  --build-dir $HOME/nrf52-voice-build \
  -- -DBOARD_ROOT=/Users/g150446/projects/voice-harness/voice-bridge-ble
```

### OTA 配布

```bash
cd /Users/g150446/projects/voice-harness/voice-bridge-ble/mac_client
../venv/bin/python3 ota_updater.py $HOME/nrf52-voice-build/nrf52-voice/zephyr/zephyr.signed.bin
```

### クライアントの最低限確認

```bash
cd /Users/g150446/projects/voice-harness/voice-bridge-ble
venv/bin/python3 -m py_compile mac_client/nrf52_voice_client.py
```

### 動作確認の順序

1. クライアント起動後、`Gesture mode: ON` を確認。
2. `DOUBLE_CLENCH` ログが先に出ることを確認。
3. その後の有効な `TILT` で録音開始することを確認。
4. 開始後 5 秒未満の `MOTION ACTIVE` では停止しないことを確認。
5. 5 秒経過後の `MOTION ACTIVE` で停止し、`mac_client/output/` に WAV が保存されることを確認。

---

## ウォッチドッグ・リセット原因

### ウォッチドッグ（WDT）

5秒タイムアウト。ファームウェアのハング（無限ループ、デッドロック）を検出して自動リセット。

- イメージ確定（起動 3 秒後）の後にアーム
- メインループが 100ms ごとにキック
- `WDT_OPT_PAUSE_HALTED_BY_DBG` 設定により、デバッガ一時停止中は WDT 停止

### リセット原因の確認

起動直後のシリアルログに出力される：

```
Reset cause: 0x00000000 [POR — power-on / battery removed]
Reset cause: 0x00020000 [WATCHDOG — firmware hang]
Reset cause: 0x00040000 [SOFTWARE]
Reset cause: 0x00010000 [PIN — external reset]
```

nRF52840 の `RESETREAS` レジスタは POR（完全電源断→再投入）のときのみ `0x00000000` になる。WDT やソフトウェアリセットは必ずいずれかのビットを立てる。

### 電池切れ vs クラッシュの判別

```
デバイスが突然切断された
  ├─ Mac クライアントが数秒以内に自動再接続できた
  │    → シリアルで [WATCHDOG] を確認 → ファームウェアクラッシュ
  │
  └─ 何度スキャンしても見つからない
       → 電池切れ
       → 充電後に再接続し、シリアルで [POR] を確認
```

---

## 電力消費

| コンポーネント | 録音なし | 録音中 |
|---------------|---------|--------|
| nRF52840 CPU (64MHz) | 4.6 mA | 4.6 mA |
| BLE ラジオ（アイドル） | 0.4 mA | — |
| BLE ラジオ（音声送信中） | — | 2–3 mA |
| PDM マイク | **0 mA**（電源 OFF） | 0.65 mA |
| LSM6DS3TR-C (26Hz 通常) | ~0.075 mA | ~0.075 mA |
| 基板クワイエセント | 0.5 mA | 0.5 mA |
| **合計** | **~5.6 mA** | **~8.8 mA** |

**40mAh 理論稼働時間:** モーション監視のみ ~7.1 時間 / 連続録音 ~4.5 時間

> タップ検出（416Hz 高性能モード）を廃止し 26Hz 通常モードに戻したことで IMU 電流が約 0.65 mA → 0.075 mA に削減された。

---

## ビルドとフラッシュ

### 環境

- NCS v2.9.2（パス: `/opt/nordic/ncs/v2.9.2`）
- ビルドディレクトリ: `~/nrf52-voice-build`（`build_and_package_ota.sh` に定義）

### 初回フラッシュ（MCUboot + アプリを UF2 で書き込み）

```bash
cd nrf52-voice
./build_and_flash.sh
```

XIAO をリセットボタンのダブルタップで UF2 ブートローダに入れます。緑 LED が点滅して XIAO-SENSE ドライブが現れたらスクリプトが自動でコピーします。

### OTA バイナリ生成

```bash
cd nrf52-voice
./build_and_package_ota.sh
```

`nrf52-voice/ota_update.bin`（約 227 KB）が生成されます。

**バージョン管理:** `Image test set failed: rc=1` が出た場合（同一ハッシュ）は `prj.conf` のバージョンを上げてリビルドする：

```ini
CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="0.0.2+0"
```

---

## OTA アップデート手順

```bash
# 実行中の BLE クライアントを停止してから行うこと
pkill -f nrf52_voice_client.py

cd mac_client
source venv/bin/activate
python3 ota_updater.py ../nrf52-voice/ota_update.bin
```

正常終了時の出力例：

```
Found: <address>
Connected. MTU=244
  227219/227219 bytes  100%  5.6 KB/s
Upload complete in 39.4s
Sending reset command...
Device reset. MCUboot will swap slots on next boot.
```

デバイスが再起動し MCUboot がスロット swap を実行します。起動 3 秒後にファームウェアが自動確定（`boot_write_img_confirmed()`）します。シリアルモニタで確認：

```
Image confirmed (permanent)
```

---

## Mac クライアント（nrf52_voice_client.py）使用方法

```bash
cd mac_client
source venv/bin/activate
python3 nrf52_voice_client.py
```

### コマンド

| コマンド | 動作 |
|----------|------|
| `r` | 録音開始（BLE write 0x01 → ファームウェアがマイク ON → PDM 起動） |
| `s` | 録音停止（BLE write 0x00 → ファームウェアが PDM 停止 → マイク OFF） |
| `a` | 自動モード ON/OFF（モーション通知で録音を自動開始・停止） |
| `t` | ステータス表示 |
| `q` | 終了 |

### 自動再接続

予期しない切断（電池切れ以外）が発生すると、クライアントは 3 秒ごとに `VoiceBridge52` をスキャンして自動再接続します。

```
  [DISCONNECT] After 64s connected
  [RECONNECT] Waiting for device to reappear...
  [RECONNECT] Scanning for 'VoiceBridge52'...
  [RECONNECT] Device found after 8s offline — reconnecting
```

WDT リセット後のファームウェア再起動なら数秒で再接続される。電池切れなら再接続されない。

### 録音ファイル

`mac_client/output/nrf52voice_YYYYMMDD_HHMMSS.wav`

- フォーマット: 16kHz / 16bit / モノラル PCM
- 典型的なパケット受信数: ~200 パケット/秒（パケットロス 0%）

---

## PDM マイク技術メモ

### マイク電源制御

P1.10（GPIO HIGH = 電源 ON）で制御。`main.c` の `mic_power_on()` / `mic_power_off()` で管理。

- 録音開始時: P1.10 HIGH → 50ms 安定待ち → `DMIC_TRIGGER_START`
- 録音停止時: `DMIC_TRIGGER_STOP` → P1.10 LOW
- **必ず `mic_power_on()` を先に呼ぶこと。** 電源 OFF のまま `DMIC_TRIGGER_START` すると全サンプルが `-8`（定数）になる。

### PDM チャンネル

XIAO nRF52840 Sense のマイクは **RIGHT チャンネル**に接続されている（SELECT ピンが HIGH）。

```c
.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT),
```

`PDM_CHAN_LEFT` を使うと全サンプルが `-8` になる。

### PDM CIC フィルタの DC オフセット問題

`DMIC_TRIGGER_START` 直後は CIC フィルタの過渡応答により大きな DC オフセットが発生する（約 -30000 LSB から開始）。`audio_capture_start()` 内で DC 収束を待つループを実装済み（最大 80フレーム = 1.6秒、通常は 4フレーム = 80ms で収束）：

```c
for (int i = 0; i < 80; i++) {
    /* dmic_read でフレームを取得 */
    /* DC = フレーム平均を計算 */
    /* |DC| < 200 で break */
}
```

### 常時 PDM 駆動は避けること

PDM を常時駆動してアイドル時に drain するアプローチは **slab が枯渇して DMA がストールする** ため機能しない。録音ごとに `DMIC_TRIGGER_START`/`STOP` を実行し、start 時に DC 収束を待つのが正しい実装。

---

## デバッグ

### シリアルモニタ接続

```bash
ls /dev/cu.usbmodem*

source mac_client/venv/bin/activate
python3 -c "
import serial, time
s = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=0.5)
while True:
    line = s.readline()
    if line: print(line.decode('utf-8', errors='replace').rstrip())
"
```

### 起動時のログ例

```
*** Booting nRF Connect SDK v2.9.2 ***
Voice Bridge BLE - XIAO nRF52840 Sense
Build: Mar 17 2026 09:00:00
Reset cause: 0x00000000 [POR — power-on / battery removed]
[inf] nrf52_voice: Starting VoiceBridge52
Wakeup configured (125mg threshold)
[inf] nrf52_voice: Calibrating for 2.5 s; keep the board still
[inf] nrf52_voice: BLE advertising started
Image confirmed (permanent)
Watchdog armed (5000 ms)
```

### 録音開始・停止のログ例

```
>>> Manual START command
Mic power ON
PDM settled after 4 frames (DC=166)
Audio capture started
Recording started

>>> Manual STOP command
Audio capture stopped
Mic power OFF
Recording stopped
```

---

## トラブルシューティング

### `Device 'VoiceBridge52' not found`

- 起動直後（キャリブレーション中）はアドバタイズしていない。起動から 3 秒待つ。
- 前回の接続が残っている場合、Mac の Bluetooth を OFF/ON してから再試行。
- シリアルモニタで `BLE advertising started` が出ているか確認。

### 全サンプルが `-8`（無音 WAV）

1. マイク電源の順序を確認 → `mic_power_on()` → 50ms → `audio_capture_start()` の順になっているか
2. PDM チャンネルが `PDM_CHAN_RIGHT` になっているか（`src/audio_capture.c`）
3. シリアルログで `PDM settled after N frames` が出ているか確認

### ジェスチャーが正しく判別されない

`gesture_collector.py` でデータを収集し、サマリーの `wakeups_after` を確認する：

- **DOUBLE_CLENCH** が ARM_LIFT と誤認される → `wakeups_after` が 0 になっている
  - 2 回目のクレンチを ACTIVE 後に行えているか確認（速すぎると両方 ACTIVE 前に来る）
- **ARM_LIFT** が DOUBLE_CLENCH と誤認される → `wakeups_after` が 1 以上になっている
  - 腕を持ち上げる動作中に意図せず 2 回目の wakeup が発生している
  - 腕をよりゆっくり持ち上げるか、WK_THS 閾値を上げることで改善できる（`WAKE_UP_THS` を `0x06`（187.5mg）等に変更）

### OTA `Image test set failed: rc=1`

OTA バイナリのハッシュが稼働中のファームウェアと同じ場合に発生。`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を上げてリビルドする。

### OTA `Upload error at offset N: rc=6`

スロット 0 の稼働イメージが「test モード（未確定）」の場合に発生。対処法：

1. **恒久対処**（実装済み）: 起動 3 秒後に `boot_write_img_confirmed()` で自動確定。
2. OTA 実行前にシリアルで `Image confirmed (permanent)` が出ているか確認。

### OTA 中にタイムアウトする

録音中に OTA を実行すると音声スレッドが BLE 帯域を消費して失敗することがある。OTA 前に録音を停止すること。

---

## 関連ドキュメント

| ファイル | 内容 |
|----------|------|
| `docs/nrf52_ota_guide.md` | `nrf52-motion` の OTA 手順（基礎となったファームウェア） |
| `docs/nrf52_porting_lessons.md` | nRF52840 移植で得られた技術的知見（DTS/PM partition ID 問題など） |
| `docs/nrf54l15_ota_guide.md` | nRF54L15 の OTA 手順 |
