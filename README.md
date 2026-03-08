# Voice Bridge BLE

Bidirectional BLE Audio Streaming for XIAO Boards.

Supports:
- **XIAO ESP32S3 Sense** (ESP-IDF)
- **XIAO nRF52840 Sense** (Zephyr/NCS)
- **XIAO nRF54L15 Sense** (Zephyr/NCS) - Latest!

## Project Structure

```
voice-bridge-ble/
├── main/                      # ESP-IDF firmware (ESP32S3)
│   ├── main.c                 # Main application
│   ├── adpcm.c/h              # IMA ADPCM codec
│   └── CMakeLists.txt
├── nrf52840/                  # Zephyr firmware (nRF52840)
│   ├── src/
│   │   ├── main.c             # Main application
│   │   └── adpcm.c/h          # IMA ADPCM codec
│   ├── boards/
│   │   └── xiao_nrf52840_sense.overlay
│   ├── prj.conf               # Zephyr config
│   └── west.yml               # Zephyr manifest
├── nrf54l15/                  # Zephyr firmware (nRF54L15) - NEW!
│   ├── src/
│   │   ├── main.c             # Main application with BLE Audio
│   │   ├── adpcm.c/h          # IMA ADPCM codec
│   │   └── audio_capture.c/h  # PDM audio capture
│   ├── boards/
│   │   └── xiao_nrf54l15_sense.overlay
│   ├── prj.conf               # Zephyr config with Audio
│   └── west.yml               # Zephyr manifest
├── mac_client/                # Mac Python client (shared)
│   ├── voice_bridge_recorder.py
│   ├── voice_bridge_client.py
│   └── requirements.txt
└── README.md
```

## Hardware Requirements

- **Seeed Studio XIAO ESP32S3 Sense** or **XIAO nRF52840 Sense** or **XIAO nRF54L15 Sense**
- **Mac** with Bluetooth 5.0 support

## Quick Start

### For XIAO ESP32S3 Sense

See `main/README.md` or the ESP-IDF section below.

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
west build -b nrf54l15dk/nrf54l15/cpuapp nrf/samples/voice_bridge_nrf54l15

# フラッシュ (pyocd 経由, ターゲット名は nrf54l)
pyocd flash -t nrf54l build/merged.hex
```

**Key Features of nRF54L15:**
- Latest Nordic MCU (Cortex-M33 128MHz)
- Bluetooth 6.0 support
- Zephyr Audio Subsystem for PDM
- Better audio quality with hardware PDM interface

See `nrf54l15/README.md` for details.

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
python3 voice_bridge_recorder.py
```

Then use commands:
- `r` - Start recording via BLE
- `s` - Stop recording via BLE
- `q` - Quit

Recorded files are saved to `mac_client/output/recording_YYYYMMDD_HHMMSS.wav`.

**Note**: The recording uses 24kHz/16-bit mono PCM for better quality.

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

## Technical Specifications

### Audio Format
- Sample Rate: 16,000 Hz
- Bit Depth: 16-bit (PCM) / 4-bit (ADPCM compressed)
- Channels: Mono

### BLE Configuration
- MTU Size: 512 bytes
- PHY: 2M PHY (for low latency)
- Connection Interval: 7.5ms - 15ms

### ADPCM Frame Format
```
Byte 0: Sequence number (0-255)
Bytes 1-60: ADPCM encoded audio data (120 samples)
```

## Troubleshooting

### Firmware Issues

1. **No audio data**: Check PDM pin connections (GPIO 41/42)
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
