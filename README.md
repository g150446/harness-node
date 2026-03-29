# Harness Node

Bidirectional BLE Audio Streaming for XIAO Boards.

This repository was renamed from `voice-bridge-ble` to `harness-node`.
The current nRF52840 Sense firmware path is `nordic-main/`, and its BLE
device name is `HarnessNode`.

Supports:
- **XIAO ESP32S3 Sense** (ESP-IDF)
- **XIAO nRF52840 Sense** (Zephyr/NCS) — `nordic-main`（現行: ジェスチャー録音 + OTA + Battery Service）
- **XIAO nRF54L15 Sense** (Zephyr/NCS) - Latest!

## Project Structure

```
harness-node/
├── esp32s3/                   # ESP-IDF firmware (ESP32S3)
│   ├── main.c                 # Main application
│   ├── adpcm.c/h              # IMA ADPCM codec
│   └── CMakeLists.txt
├── nrf54l15/                  # Zephyr firmware (nRF54L15)
│   ├── src/
│   │   ├── main.c             # BLE service + audio streaming loop
│   │   ├── adpcm.c/h          # ADPCM codec (互換用)
│   │   └── audio_capture.c/h  # PDM microphone capture
│   ├── boards/
│   │   └── xiao_nrf54l15_sense.overlay
│   ├── README.md              # nRF54L15 implementation notes
│   ├── prj.conf               # Zephyr config with Audio
│   └── west.yml               # Zephyr manifest
├── nordic-main/               # Zephyr ジェスチャートリガー音声+OTA for XIAO nRF52840 Sense (HarnessNode) ← 現行
│   ├── src/
│   │   ├── main.c             # BLE サービス + ジェスチャー検出 + DMIC 制御
│   │   ├── audio_capture.c/h  # DMIC キャプチャ（16 kHz / 16-bit / モノラル）
│   │   └── adpcm.c/h          # ADPCM コーデック（互換用）
│   ├── boards/
│   │   └── xiao_ble_nrf52840_sense.overlay  # PDM + IMU + マイク電源 + フラッシュパーティション
│   ├── sysbuild/mcuboot/      # MCUboot 設定
│   ├── prj.conf               # Zephyr 設定（BLE, Audio, MCUmgr）
│   ├── sysbuild.conf          # SB_CONFIG_BOOTLOADER_MCUBOOT=y
│   ├── pm_static.yml          # フラッシュパーティション定義
│   ├── build_and_flash.sh     # 初回 UF2 フラッシュ
│   └── build_and_package_ota.sh  # OTA バイナリ生成（→ ota_update.bin）
├── nrf52-voice/               # Zephyr 音声+モーション+OTA for XIAO nRF52840 Sense (VoiceBridge52) ← レガシー
│   ├── src/
│   │   ├── main.c             # BLE audio/motion service + motion→audio制御
│   │   ├── audio_capture.c/h  # PDM マイク（RIGHT ch, mic電源有効）
│   │   ├── adpcm.c/h          # ADPCM コーデック
│   ├── boards/
│   │   └── xiao_ble_nrf52840_sense.overlay  # PDM + IMU + マイク電源 + フラッシュパーティション
│   ├── sysbuild/mcuboot/      # MCUboot 設定
│   ├── prj.conf               # Zephyr 設定（BLE, Audio, MCUmgr, USB CDC ACM）
│   ├── sysbuild.conf          # SB_CONFIG_BOOTLOADER_MCUBOOT=y
│   ├── pm_static.yml          # フラッシュパーティション定義
│   ├── build_and_flash.sh     # 初回 UF2 フラッシュ
│   └── build_and_package_ota.sh  # OTA バイナリ生成
├── nrf52-motion/              # Zephyr IMU motion detection + BLE OTA for XIAO nRF52840 Sense
│   ├── src/
│   │   └── main.c             # IMU motion detection + BLE notify + MCUmgr OTA
│   ├── boards/
│   │   └── xiao_ble_nrf52840_sense.overlay
│   ├── sysbuild/mcuboot/      # MCUboot configuration for nested boot at 0x27000
│   ├── prj.conf               # Zephyr config (BLE, MCUmgr, USB CDC ACM)
│   ├── sysbuild.conf          # SB_CONFIG_BOOTLOADER_MCUBOOT=y
│   ├── pm_static.yml          # Flash partition layout (MCUboot + OTA slots)
│   ├── build_and_flash.sh     # Initial UF2 flash (MCUboot + app merged)
│   └── build_and_package_ota.sh  # Build OTA binary only (no USB flash)
├── nrf54-motion/              # Zephyr IMU motion detection test for XIAO nRF54L15 Sense
│   ├── src/
│   │   └── main.c             # IMU wake-up / motion interrupt test
│   ├── boards/
│   │   ├── xiao_nrf54l15_nrf54l15_cpuapp.overlay
│   │   └── xiao_nrf54l15_sense.overlay
│   ├── prj.conf               # Zephyr config for the onboard LSM6DS3TR-C
│   ├── build_and_flash.sh     # Build + USB flash + OTA payload helper
│   ├── build_and_package_ota.sh # Build + OTA payload helper (no USB flash)
│   └── flash.sh               # Compatibility wrapper to build_and_flash.sh
├── boards/seeed/xiao_nrf54l15/ # Custom XIAO nRF54L15 board definition
├── mac_client/                # Mac Python client (shared)
│   ├── nrf52_voice_client.py  # HarnessNode BLE クライアント（自動録音、ジェスチャーイベント表示）
│   ├── gesture_monitor.py     # ジェスチャーイベントモニター（タイムスタンプ付き、録音なし）
│   ├── gesture_classifier.py  # オフライン CSV ジェスチャー分類器（検証用）
│   ├── gesture_collector.py   # ジェスチャーデータ収集ツール（ARM_LIFT / DOUBLE_CLENCH）
│   ├── ota_updater.py         # BLE OTA アップデータ（HarnessNode / nRF52840 用）
│   ├── nrf54_controller.py
│   ├── voice_bridge_client.py
│   └── requirements.txt
└── README.md
```

## Hardware Requirements

- **Seeed Studio XIAO ESP32S3 Sense**  or **XIAO nRF54L15 Sense**
- **Mac** with Bluetooth 5.0 support

## Quick Start

### For XIAO ESP32S3 Sense

See `esp32s3/` or the ESP-IDF section below.

### For XIAO nRF52840 Sense — HarnessNode (`nordic-main`)

`nordic-main/` is the primary nRF52840 Sense target in this repository.
It advertises as `HarnessNode` and is the firmware used by the current
Android app and Handy integration.

PDM マイク音声 + LSM6DS3TR-C ジェスチャー検出 + BLE OTA + **バッテリー残量通知** を統合したファームウェアです。腕を水平から持ち上げるジェスチャーで録音を自動開始・停止します。ジェスチャー判別は **ファームウェア側で完結** しており、Mac クライアントは BLE イベント（`0x01` / `0x02`）を受け取るだけです。

LED は省電力・省発光寄りの挙動にしてあり、起動直後の白色点灯後は **アドバタイジング中 / BLE 接続済み待機中 / 単純なモーション検出中は消灯** します。LED が点灯するのは **録音ジェスチャー成立後の録音中（赤）** のみで、リモート未接続でも録音ジェスチャーが成立すれば赤点灯します。

バッテリー残量は **BLE Battery Service（UUID 0x180F）** で 1 分ごとに通知されます（録音停止直後にも即時更新）。nRF Connect アプリ等から Battery Level キャラクタリスティック（UUID 0x2A19）で読み取り可能です。

詳細は `docs/nordic_main_guide.md` を参照してください。

#### ジェスチャー検出アルゴリズム（概要）

録音開始は以下の **4 条件**がすべて成立したとき:

1. 静定時の z 軸加速度が ≥ 8.0 m/s²（腕が上がった状態で静定）
2. モーション開始 → 静定の経過時間が ≤ 2000 ms
3. モーション中のピーク速度 ≥ 2.5 m/s（通常）/ ≥ 1.25 m/s（ライトスリープ復帰直後）
4. モーション中の総移動距離 ≥ 0.25 m（通常）/ ≥ 0.125 m（ライトスリープ復帰直後）

録音停止は、録音中に次のモーション開始が検出されたとき。

#### ライトスリープ

10 秒間モーションがなく録音中でもない場合、IMU ポーリング間隔を 25 ms → 500 ms に切り替え（BLE 接続は維持）。スリープ移行時に `0x20`、復帰時に `0x21` イベントを送信。スリープ復帰直後の最初のジェスチャーのみ条件 3・4 の閾値を半分に緩和。

#### 初回フラッシュ（MCUboot + アプリを UF2 で書き込み）

```bash
cd nordic-main
./build_and_flash.sh
```

XIAO のリセットボタンをダブルタップして UF2 ブートローダに入ると（XIAO-SENSE ドライブが出現）、スクリプトが自動でフラッシュします。

#### BLE OTA アップデート（2 回目以降）

```bash
cd nordic-main
./build_and_package_ota.sh           # OTA バイナリをビルド → ota_update.bin

cd mac_client
source venv/bin/activate
python3 ota_updater.py --device HarnessNode ../nordic-main/ota_update.bin
```

#### Mac クライアントで接続・録音

```bash
cd mac_client
source venv/bin/activate
python3 xiao_voice_client.py
```

`0x01` イベント受信で WAV 録音を自動開始、`0x02` 受信で自動停止します。motion_active / motion_settled / sleep_enter / sleep_wake イベントも表示されます。

録音ファイルは `mac_client/output/xiao_recording_YYYYMMDD_HHMMSS.wav` に保存されます（16 kHz / 16-bit / モノラル）。

#### ジェスチャーモニター（録音なし）

```bash
cd mac_client
source venv/bin/activate
python3 gesture_monitor.py
```

BLE イベントをタイムスタンプ付きで表示するだけのミニマルモニターです。ジェスチャー動作確認やデバッグに使用します。

---

<details>
<summary>レガシー: VoiceBridge52 (<code>nrf52-voice</code>)</summary>

`nrf52-voice/` は旧ファームウェアです。nRF52840 Sense 向けには `nordic-main/` の使用を推奨します。`nrf52-voice/` は ARM_LIFT / DOUBLE_CLENCH の 2 ジェスチャーを IMU の wakeup パターンで判別する方式でした。

初回フラッシュ:

```bash
cd nrf52-voice
./build_and_flash.sh
```

BLE OTA:

```bash
cd nrf52-voice
./build_and_package_ota.sh
cd mac_client
source venv/bin/activate
python3 ota_updater.py ../nrf52-voice/ota_update.bin
```

詳細は `docs/nrf52_voice_guide.md` を参照してください。

</details>

### For XIAO nRF54L15 Sense (Latest - Recommended)

#### 自動フラッシュ

```bash
cd nrf54l15
./flash.sh
```

#### 手動フラッシュ

XIAO nRF54L15 は CMSIS-DAP インターフェースのため、pyocd でフラッシュします。

```bash
# 前提: NCS v2.9.2, pyocd がインストール済み
pip install pyocd  # 未インストールの場合

# ビルド
export PATH="/opt/nordic/ncs/toolchains/b8efef2ad5/bin:$PATH"
cd /opt/nordic/ncs/v2.9.2
west build -b xiao_nrf54l15/nrf54l15/cpuapp nrf/samples/voice_bridge_nrf54l15 -- -DBOARD_ROOT=<project_dir>

# フラッシュ (pyocd 経由, ターゲット名は nrf54l)
pyocd flash -t nrf54l build/merged.hex
```

**Key Features of nRF54L15:**
- Latest Nordic MCU (Cortex-M33 128MHz)
- Bluetooth 6.0 support
- Zephyr Audio Subsystem for PDM
- Better audio quality with hardware PDM interface

See `nrf54l15/README.md` for details.

### nRF54L15 implementation overview

The working nRF54L15 path now uses a custom XIAO board definition imported from newer Zephyr board support, because NCS v2.9.2 did not include the XIAO board files by default.

- `boards/seeed/xiao_nrf54l15/`
  - Defines the real XIAO nRF54L15 hardware: UART routing, fixed regulators, IMU alias, and the onboard PDM microphone pins.
  - Uses the correct microphone pins: `P1.12` for `PDM_CLK` and `P1.13` for `PDM_DIN`.
- `nrf54l15/src/audio_capture.c`
  - Initializes the DMIC device through `DT_ALIAS(dmic20)`.
  - Configures 16 kHz / 16-bit / mono capture.
  - Starts and stops the Zephyr DMIC stream and returns PCM buffers to the BLE thread.
- `nrf54l15/src/main.c`
  - Exposes the custom BLE service used by the Mac client.
  - Starts recording on BLE write `0x01`, stops on `0x00`.
  - Streams raw PCM frames in packets formatted as `[sequence][0xAA][PCM data...]`.
  - Falls back to a generated 440 Hz tone only if DMIC initialization or reading fails.
- `mac_client/nrf54_controller.py`
  - Connects over BLE, sends start/stop commands, and stores the received PCM stream as a 16 kHz WAV file.
  - This was updated to match the firmware sample rate, so saved files now have the correct timing.

### nRF52840 IMU motion detection + BLE OTA (`nrf52-motion`)

`nrf52-motion/` は Seeed XIAO nRF52840 Sense 向けの Zephyr アプリです。`nrf54-motion/` と同じモーション検出・BLE Notify 機能に加え、MCUboot + MCUmgr/SMP による BLE OTA ファームウェア更新に対応しています。

#### 初回フラッシュ（MCUboot + アプリを UF2 で書き込み）

nRF52840 Sense は Adafruit UF2 ブートローダを使用します。MCUboot は Adafruit がジャンプする 0x27000 に配置されます。初回プロビジョニングでは、MCUboot + 署名済みアプリの merged UF2 を UF2 ドライブ経由で書き込みます。

```bash
cd nrf52-motion
./build_and_flash.sh   # sysbuild ビルド後、XIAO-SENSE ドライブを待って自動コピー
```

リセットボタンをダブルタップして UF2 ブートローダに入ると (XIAO-SENSE ドライブが出現)、スクリプトが自動でフラッシュします。

#### BLE OTA アップデート

OTA 対応ファームウェアが稼働中の場合、以降の更新は BLE のみで完結します。

**前準備（初回のみ）:**

```bash
cd mac_client
python3 -m venv ../venv
../venv/bin/pip install -r requirements.txt
../venv/bin/pip install cbor2
```

**OTA バイナリをビルド:**

```bash
cd nrf52-motion
./build_and_package_ota.sh   # ビルドして ota_update.bin を生成
```

OTA バイナリのバージョンは稼働中のファームウェアより新しくする必要があります。`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を更新してからビルドしてください（例: `"0.0.2+0"`）。

**BLE 経由で送信:**

```bash
cd mac_client
../venv/bin/python3 ota_updater.py ../nrf52-motion/ota_update.bin
```

正常終了時の出力例:

```text
Scanning for 'MotionBridge'...
Found: <address>
Connected. MTU=244
Upload complete in ~104s
Querying image state...
Setting image test flag...
Image test flag set.
Sending reset command...
Device reset. MCUboot will swap slots on next boot.
```

デバイスが自動的に再起動して新しいファームウェアで動作します。転送速度は約 2 KB/s（217 KB イメージで約 104 秒）。

実装詳細とトラブルシューティングは `docs/nrf52_ota_guide.md` を参照してください。

### IMU motion detection test app

The repository also includes `nrf54-motion/`, a standalone Zephyr app for the Seeed XIAO nRF54L15 Sense onboard LSM6DS3TR-C IMU.

```bash
cd nrf54-motion
./build_and_flash.sh
```

The app polls `imu0` at 26 Hz, calibrates a baseline for about 2.5 seconds after boot, and then prints `Motion detected!` messages on the serial console at 115200 baud when the board is moved. Once the board becomes still again, it logs `Motion settled` and updates the baseline to the new resting orientation.

### nRF54L15 BLE OTA update (`nrf54-motion`)

`nrf54-motion/` supports BLE OTA using MCUboot + MCUmgr/SMP. The OTA target device name is `MotionBridge`.

#### Prerequisites

- NCS v2.9.2
- Python environment with `bleak`
- `cbor2` for `mac_client/ota_updater.py`

```bash
cd mac_client
python3 -m venv ../venv  # if needed
../venv/bin/pip install -r requirements.txt
../venv/bin/pip install cbor2
```

#### Case 1: the device does not yet have OTA-capable firmware

Use USB once to install the OTA-capable firmware and generate the first OTA payload:

- Requires `pyocd`

```bash
cd nrf54-motion
./build_and_flash.sh
```

This command builds with sysbuild + MCUboot, flashes `merged.hex` over USB via `pyocd`, and refreshes `nrf54-motion/ota_update.bin`.

After that initial USB step, later updates can use BLE OTA only.

#### Case 2: the device already has OTA-capable firmware

1. Build a new OTA payload without USB flashing:

- `pyocd` is not required for this step

```bash
cd nrf54-motion
./build_and_package_ota.sh
```

This command only rebuilds the firmware and refreshes `nrf54-motion/ota_update.bin`. It does **not** flash over USB.

2. Make the firmware newer than the running image.

In practice, change the firmware, or rebuild after your intended modifications so the generated signed image differs from the version already running on the board. The firmware exposes a Build Info GATT characteristic, so the build timestamp can be used to verify the switch after reboot.

3. Run the OTA updater:

```bash
cd mac_client
../venv/bin/python3 ota_updater.py ../nrf54-motion/ota_update.bin
```

Expected successful tail output:

```text
Upload complete in ...
Querying image state...
Setting image test flag...
Image test flag set.
Sending reset command...
Device reset. MCUboot will swap slots on next boot.
```

4. Verify that the device came back on the new image.

The `Build Info` characteristic UUID is `00000012-0000-1000-8000-00805f9b34fb`. Read it before and after OTA to confirm that the build timestamp changed.

#### Notes

- OTA support described here is for `nrf54-motion/`, not `nrf54l15/`.
- The updater currently performs upload + test boot request + reset. It does not automatically reconnect after reboot to confirm the new image.
- Use `build_and_flash.sh` only for USB provisioning / recovery.
- Use `build_and_package_ota.sh` when the board already has OTA-capable firmware and you only need a new OTA payload.
- `nrf54-motion/flash.sh` still works, but it is now only a compatibility wrapper around `build_and_flash.sh`.
- For implementation details and maintenance notes, see `docs/nrf54l15_ota_guide.md` and `docs/nrf54l15_ota_status.md`.

## Firmware Setup (ESP-IDF)

### Prerequisites

- ESP-IDF v5.x installed
- XIAO ESP32S3 board support configured

### Build and Flash

```bash
# Set target to ESP32S3
idf.py set-target esp32s3

# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyACM0 flash monitor
```

Replace `/dev/ttyACM0` with your actual serial port.

### Pin Configuration

- **PDM CLK**: GPIO 42
- **PDM DATA**: GPIO 41

## Mac Client Setup

### Install Dependencies

```bash
cd mac_client
pip3 install -r requirements.txt
```

### Recording Mode (High Quality)

Record audio to WAV file with control:

```bash
cd mac_client
python3 nrf54_controller.py
```

Then use commands:
- `r` - Start recording via BLE
- `s` - Stop recording via BLE
- `q` - Quit

Recorded files are saved to `mac_client/output/recording_YYYYMMDD_HHMMSS.wav`.

**Note**: The nRF54L15 path records and saves `16kHz / 16-bit / mono` PCM to match the firmware output.

### Real-time Playback Mode

Stream audio to Mac speakers:

```bash
python3 voice_bridge_client.py
```

### Serial Monitor

To view XIAO's serial output for debugging:

```bash
# Auto-detect XIAO and connect
python3 serial_monitor.py

# Specify port
python3 serial_monitor.py -p /dev/tty.usbmodem1101

# List available ports
python3 serial_monitor.py --list

# Reset device
python3 serial_monitor.py --reset
```

On the Atom Echo S3R firmware, the USB serial console at `115200` baud also accepts
simple control commands:

- `r` / `R`: request recording start
- `s` / `S`: request recording stop
- `c` / `C` / `1`: emulate a **single-click**
- `d` / `D` / `2`: emulate a **double-click**
- `h` / `H`: print command help

This is useful for reproducing Handy-side BLE button handling without physically
pressing the device button.

## Technical Specifications

### Audio Format
- Sample Rate: 16,000 Hz
- Bit Depth: 16-bit (PCM) / 4-bit (ADPCM compressed)
- Channels: Mono

### BLE Configuration
- MTU Size: 247 bytes on the current nRF54L15 path
- PHY: 2M PHY (for low latency)
- Connection Interval: 7.5ms - 15ms

### PCM Packet Format (nRF54L15)
```
Byte 0: Sequence number (0-255)
Byte 1: Sync byte (0xAA)
Bytes 2..N: 16-bit little-endian PCM samples
```

### ADPCM Frame Format
```
Byte 0: Sequence number (0-255)
Bytes 1-60: ADPCM encoded audio data (120 samples)
```

## Troubleshooting

### Firmware Issues

1. **No audio data on nRF54L15**: Confirm the custom XIAO board definition is being used and that the firmware is built for `xiao_nrf54l15/nrf54l15/cpuapp`
2. **BLE connection fails**: Reset the XIAO board
3. **Audio quality issues**: Verify sample rate configuration

### Mac Client Issues

1. **Device not found**: Ensure XIAO is advertising (check serial monitor)
2. **Audio playback fails**: Check PyAudio installation
   ```bash
   # macOS: Install portaudio
   brew install portaudio
   pip3 install pyaudio
   ```

## License

MIT License
