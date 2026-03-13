# Voice Bridge BLE

Bidirectional BLE Audio Streaming for XIAO Boards.

Supports:
- **XIAO ESP32S3 Sense** (ESP-IDF)
- **XIAO nRF52840 Sense** (Zephyr/NCS)
- **XIAO nRF54L15 Sense** (Zephyr/NCS) - Latest!

## Project Structure

```
voice-bridge-ble/
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

### For XIAO nRF52840 Sense

```bash
cd nrf52840
west init -l .
cd ..
west update
cd nrf52840
west build -b xiao_nrf52840_sense
west flash
```

See `nrf52840/README.md` for details.

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
