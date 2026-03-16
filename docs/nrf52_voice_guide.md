# nrf52-voice 保守・運用ガイド

XIAO nRF52840 Sense 向け音声+モーション+OTA 統合ファームウェア `nrf52-voice`（BLE デバイス名: `VoiceBridge52`）の保守・運用に必要な情報をまとめます。

---

## 概要

| 項目 | 内容 |
|------|------|
| ターゲットボード | `xiao_ble/nrf52840/sense`（Seeed XIAO nRF52840 Sense） |
| BLE デバイス名 | `VoiceBridge52` |
| ブートローダ構成 | Adafruit UF2 (0xF4000) + MCUboot (0x27000) |
| OTA トランスポート | BLE SMP（MCUmgr） |
| PDM マイク | MSM261D3526H1CPM（RIGHT チャンネル、P1.10 電源有効） |
| IMU | LSM6DS3TR-C（Zephyr LSM6DSL ドライバ互換） |
| コンソール | USB CDC ACM（115200 baud） |
| 音声フォーマット | 16kHz / 16bit / モノラル PCM |
| 転送速度（OTA） | 約 5.8 KB/s（225 KB イメージで約 39 秒） |

---

## ファイル構成

```
nrf52-voice/
├── CMakeLists.txt                      # PM_STATIC_YML_FILE を find_package 前に設定
├── prj.conf                            # Zephyr 設定（Audio, BLE, MCUmgr, USB CDC ACM）
├── sysbuild.conf                       # SB_CONFIG_BOOTLOADER_MCUBOOT=y
├── pm_static.yml                       # フラッシュパーティション定義（nrf52-motion から流用）
├── build_and_flash.sh                  # 初回 UF2 フラッシュスクリプト
├── build_and_package_ota.sh            # OTA バイナリ生成スクリプト
├── src/
│   ├── main.c                          # BLE サービス + モーション検出 + 音声制御統合
│   ├── audio_capture.c / .h            # PDM マイク入力（Zephyr DMIC API）
│   ├── adpcm.c / .h                    # ADPCM コーデック（予備）
├── boards/
│   └── xiao_ble_nrf52840_sense.overlay # PDM ピン + マイク電源 + フラッシュパーティション
└── sysbuild/
    └── mcuboot/
        ├── prj.conf
        └── boards/
            ├── xiao_ble_nrf52840_sense.conf     # MCUboot ボード設定
            └── xiao_ble_nrf52840_sense.overlay  # MCUboot 用パーティション定義
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

**Motion TX パケット形式（14 バイト）:**

```
Byte 0:    イベントタイプ (0x01 = motion active / 0x00 = motion settled)
Byte 1:    モーション検出累計カウント
Bytes 2-5: activity (float LE, m/s²)
Bytes 6-9: peak (float LE, m/s²)
Bytes 10-13: elapsed_ms (uint32 LE, 前回のモーションからの経過時間)
```

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
| `MOTION_AUDIO_COOLDOWN_MS` | 2000 | 音声開始/停止後の再トリガー禁止時間（ms） |
| `ACCEL_ODR_HZ` | 26 | IMU サンプリングレート（Hz） |
| `POLL_INTERVAL_MS` | 100 | メインループのポーリング間隔（ms） |

---

## ビルドとフラッシュ

### 環境

- NCS v2.9.2（パス: `/opt/nordic/ncs/v2.9.2`）
- ビルドディレクトリ: `/Users/g150446/nrf52-voice-build`（`build_and_package_ota.sh` に定義）

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

`nrf52-voice/ota_update.bin`（約 225 KB）が生成されます。

**バージョン管理:** 毎回 OTA を行う場合、`prj.conf` の以下の値を変更する必要はありません（MCUmgr は異なるハッシュを持つイメージであれば受け付けます）。ただし `Image test set failed: rc=1` が出た場合はバージョンを上げてください：

```ini
CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="0.0.2+0"
```

---

## OTA アップデート手順

```bash
cd mac_client
source venv/bin/activate
python3 ota_updater.py ../nrf52-voice/ota_update.bin
```

正常終了時の出力例：

```
Scanning for 'VoiceBridge52'...
Found: <address>
Connected. MTU=244
Chunk size: first=163, rest=213 bytes
  224983/224983 bytes  100%  5.8 KB/s
Upload complete in 38.7s
Querying image state...
Slot 1 image hash: <hash>
Setting image test flag...
Image test flag set.
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

### 手動モード

| コマンド | 動作 |
|----------|------|
| `r` | 録音開始（BLE write 0x01 → ファームウェア PDM 起動） |
| `s` | 録音停止 |
| `t` | ステータス表示 |
| `q` | 終了 |

### 自動モード（モーション連動）

`a` で自動モードをトグルします。

- モーション検出通知（`event_type=0x01`）受信 → WAV 録音開始 + BLE start 送信
- モーション停止通知（`event_type=0x00`）受信 → WAV 録音停止 + BLE stop 送信

自動モードと手動モードは同時使用可能（`r`/`s` でオーバーライドできる）。

### 録音ファイル

`mac_client/output/nrf52voice_YYYYMMDD_HHMMSS.wav`

- フォーマット: 16kHz / 16bit / モノラル PCM
- 典型的なパケット受信数: ~200 パケット/秒（パケットロス 0%）

---

## PDM マイク技術メモ

### マイク電源有効化（重要）

XIAO nRF52840 Sense の MSM261D3526H1CPM マイクは **P1.10 を HIGH にしないと電源が入らない**。

Board DTS の `msm261d3526hicpm-c-en` ノードは `regulator-boot-on` なしで定義されているため、overlay で明示的に追加する必要がある：

```dts
/* boards/xiao_ble_nrf52840_sense.overlay */
msm261d3526hicpm_c_en: msm261d3526hicpm-c-en {
    compatible = "regulator-fixed-sync", "regulator-fixed";
    enable-gpios = <&gpio1 10 (NRF_GPIO_DRIVE_S0H1 | GPIO_ACTIVE_HIGH)>;
    regulator-name = "MSM261D3526HICPM-C-EN";
    regulator-boot-on;  /* ← これがないとマイクが無音になる */
};
```

電源なしの場合の症状: 全サンプルが `-8`（定数）になる。

### PDM チャンネル

XIAO nRF52840 Sense のマイクは **RIGHT チャンネル**に接続されている（SELECT ピンが HIGH）。

```c
/* src/audio_capture.c */
.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT),
```

`PDM_CHAN_LEFT` を使うと全サンプルが `-8` になる。

### PDM CIC フィルタの DC オフセット問題

`DMIC_TRIGGER_START` 直後は CIC フィルタの過渡応答により大きな DC オフセットが発生する（約 -30000 LSB から開始）。マイク電源が有効な状態では **4フレーム（80ms）程度で収束**する。

`audio_capture_start()` 内で DC 収束を待つループを実装済み：

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
# ポート確認
ls /dev/cu.usbmodem*

# Python で接続
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
Build: Mar 16 2026 13:39:32
[inf] nrf52_voice: Starting VoiceBridge52
[inf] nrf52_voice: Calibrating for 2.5 s; keep the board still
[inf] nrf52_voice: Motion detection ready: ODR=26 Hz, poll=100 ms
[inf] nrf52_voice: BLE advertising started
Image confirmed (permanent)   ← OTA後、3秒で自動確定
```

### 録音開始時のログ例

```
>>> Manual START command           ← BLE write 0x01 受信
PDM settled after 4 frames (DC=166)  ← DC収束 (マイク電源有効時は4フレーム程度)
[inf] audio_capture: Audio capture started
Audio started
Recording started
```

### モーション検出のログ例

```
Motion! count=1 elapsed=0 ms activity=16.2 peak=6.4
>>> Motion started → requesting audio start
Motion! count=2 elapsed=784 ms activity=12.5 peak=4.2
Motion settled: baseline=(0.012, -0.003, 9.814)
>>> Motion settled → requesting audio stop
```

---

## トラブルシューティング

### `Device 'VoiceBridge52' not found`

- デバイスが起動直後（キャリブレーション中）は BLE アドバタイズしていない。起動から 3 秒待つ。
- 前回の接続が残っている場合、Mac の Bluetooth を OFF/ON してから再試行。
- シリアルモニタで `BLE advertising started` が出ているか確認。

### 全サンプルが `-8`（無音 WAV）

1. マイク電源が有効か確認 → `boards/xiao_ble_nrf52840_sense.overlay` に `regulator-boot-on` があるか
2. PDM チャンネルが `PDM_CHAN_RIGHT` になっているか確認（`src/audio_capture.c`）
3. シリアルログで `PDM settled after N frames` が出ているか確認

### `dmic_nrfx_pdm: No audio data to be read` が大量に出る

PDM DMA がストールしている。常時 PDM 駆動 + idle drain のコードが残っていないか確認。録音開始時のみ `DMIC_TRIGGER_START` を呼ぶ実装になっているか確認。

### OTA `Image test set failed: rc=1`

OTA バイナリのハッシュが稼働中のファームウェアと同じ場合に発生。`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を上げてリビルドする。

### OTA `Upload error at offset N: rc=6`

スロット 0 の稼働イメージが「test モード（未確定）」の場合に発生。MCUmgr はアクティブスロットへの上書きを拒否する。対処法：

1. **恒久対処**（実装済み）: 起動 3 秒後に `boot_write_img_confirmed()` で自動確定。
2. **手動確定**: `mac_client/ota_updater.py` の直前の実行で `confirm=True` の SMP コマンドを送信。

### 録音開始後しばらくして OTA がタイムアウトする

OTA 中に録音が開始されると音声スレッドが BLE バンド幅を消費し OTA が失敗することがある。OTA 実行時は録音を停止してから行うこと。

---

## 関連ドキュメント

| ファイル | 内容 |
|----------|------|
| `docs/nrf52_ota_guide.md` | `nrf52-motion` の OTA 手順（基礎となったファームウェア） |
| `docs/nrf52_porting_lessons.md` | nRF52840 移植で得られた技術的知見（DTS/PM partition ID 問題など） |
| `docs/nrf54l15_ota_guide.md` | nRF54L15 の OTA 手順 |
