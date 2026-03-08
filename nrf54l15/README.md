# Voice Bridge BLE - XIAO nRF54L15 Sense

Seeed Studio XIAO nRF54L15 Sense 用の BLE 音声ストリーミングファームウェア

## 概要

XIAO nRF54L15 Sense のオンボード PDM マイクから音声を収集し、BLE を介して Mac にストリーミングします。

nRF54L15 は Nordic の最新 MCU で、以下の特徴があります：
- Arm Cortex-M33 128MHz
- Bluetooth 6.0 対応
- 1.5MB Flash + 256KB RAM
- PDM マイクインターフェース内蔵

## ハードウェア要件

- Seeed Studio XIAO nRF54L15 Sense
- Mac (Bluetooth 5.0 以上)
- USB ケーブル

## ソフトウェア要件

- Nordic Connect SDK (NCS) v2.9.2 以降
- west (Zephyr ビルドツール)
- pyocd (`pip install pyocd`) — フラッシュ用
- Python 3.10+ / bleak / pyserial — Mac クライアント用

## クイックスタート

### 1. 自動フラッシュスクリプト

```bash
cd nrf54l15
./flash.sh
```

スクリプトが自動的に以下の処理を実行します：
1. ソースファイルのコピー
2. ビルド（数分かかります）
3. フラッシュ

### 2. Mac クライアントの実行

```bash
cd mac_client
python3 voice_bridge_recorder.py
```

### 3. 操作

- 接続後、`r` キーを押して録音開始
- `s` キーで録音停止
- `q` キーで終了

録音ファイルは `mac_client/output/recording_*.wav` に保存されます。

## 仕様

### 音声
- サンプリングレート：16kHz
- ビット深度：16bit
- チャンネル：モノラル
- マイク：オンボード PDM (MSM261DGT006)

### PDM マイクピン
- PDM_CLK: P1.06 (D2)
- PDM_DATA: P1.07 (D3)

### BLE
- デバイス名：VoiceBridge
- サービス UUID: 00000001-0000-1000-8000-00805f9b34fb
- TX 特性 (Notify): 00000002-0000-1000-8000-00805f9b34fb
- RX 特性 (Write): 00000003-0000-1000-8000-00805f9b34fb
- デフォルト MTU: 23（ATT ペイロード最大 20 バイト）

### 送信データ
- パケットサイズ：20 バイト（ヘッダー 2 バイト + PCM データ 18 バイト）
- パケット形式：[シーケンス番号][同期バイト 0xAA][PCM データ...]

### 現在の制限事項
- 音声キャプチャはシミュレーション（440Hz 正弦波）。実 PDM マイク対応は `audio_capture.c` の実装が必要
- MTU 23 のため帯域が限られる（MTU 拡張で改善可能）

## 手動ビルド・フラッシュ

### 前提条件

- Nordic Connect SDK (NCS) v2.9.2: `/opt/nordic/ncs/v2.9.2`
- NCS ツールチェイン: `/opt/nordic/ncs/toolchains/b8efef2ad5`
- pyocd (`pip install pyocd`)

### ビルド

```bash
export PATH="/opt/nordic/ncs/toolchains/b8efef2ad5/bin:$PATH"
cd /opt/nordic/ncs/v2.9.2

# ソースファイルを NCS サンプルディレクトリにコピー
mkdir -p nrf/samples/voice_bridge_nrf54l15/src
cp <project_dir>/nrf54l15/src/*.c nrf/samples/voice_bridge_nrf54l15/src/
cp <project_dir>/nrf54l15/src/*.h nrf/samples/voice_bridge_nrf54l15/src/
cp <project_dir>/nrf54l15/CMakeLists.txt nrf/samples/voice_bridge_nrf54l15/
cp <project_dir>/nrf54l15/prj.conf nrf/samples/voice_bridge_nrf54l15/

# ビルド (nrf54l15dk ボードターゲット)
rm -rf build
west build -b nrf54l15dk/nrf54l15/cpuapp nrf/samples/voice_bridge_nrf54l15
```

### フラッシュ

XIAO nRF54L15 は CMSIS-DAP インターフェースを使用しているため、**pyocd** でフラッシュします。
`west flash --runner nrfutil` は CMSIS-DAP デバイスでは動作しません。

```bash
# pyocd でフラッシュ（推奨）
pyocd flash -t nrf54l /opt/nordic/ncs/v2.9.2/build/merged.hex

# デバイスの接続確認
pyocd list
nrfutil device list
```

### デバイスの確認

```bash
# pyocd でデバイスが見えるか確認
$ pyocd list
  #   Probe/Board                                      Unique ID   Target
---------------------------------------------------------------------------
  0   Seeed Studio Seeed Studio XIAO nrf54 CMSIS-DAP   81370703    n/a

# nrfutil でデバイス情報を確認
$ nrfutil device list
81370703
Product         Seeed Studio XIAO nrf54 CMSIS-DAP
Ports           /dev/tty.usbmodem813707033
Traits          serialPorts, usb
```

## 動作確認済みテスト結果

2026-03-09 に以下の環境で動作確認済み：

- **ボード**: Seeed Studio XIAO nRF54L15 Sense (CMSIS-DAP, ID: 81370703)
- **NCS**: v2.9.2 / ツールチェイン b8efef2ad5
- **ボードターゲット**: `nrf54l15dk/nrf54l15/cpuapp`
- **フラッシュ**: pyocd 0.43.1 (`pyocd flash -t nrf54l`)
- **Mac クライアント**: bleak (BLE), Python 3.12

| 項目 | 結果 |
|------|------|
| BLE アドバタイズ | OK — "VoiceBridge" で検出 |
| BLE 接続 | OK — MTU 23 |
| サービス/Characteristics 認識 | OK |
| 録音コマンド送信 (Write) | OK |
| 音声データ受信 (Notify) | OK — 5秒で493パケット/9,690バイト |
| WAV ファイル保存 | OK |

### 実行に必要だった修正

- `bt_enable` を同期呼び出し (`NULL`) に変更（非同期だと main 終了後にコールバックが呼ばれない）
- `main()` で `while(1) k_sleep` により永続化
- UUID マクロの中括弧 `{}` を除去（`BT_UUID_DECLARE_128` の仕様）
- `CONFIG_BT_DEVICE_NAME_DYNAMIC=y` を prj.conf に追加
- scan response にデバイス名を含めるよう `BT_LE_ADV_CONN` + `scan_rsp` を使用
- パケットサイズを MTU 23 に合わせて 20 バイトに調整
- `flash.sh` のランナーを `nrfutil` → `pyocd` に変更（CMSIS-DAP 対応）

## トラブルシューティング

### デバイスが見つからない

1. XIAO nRF54L15 がフラッシュされていることを確認
2. XIAO をリセット（リセットボタンを押す）
3. Mac の Bluetooth がオンになっていることを確認
4. 数秒待ってから再度試行

### フラッシュできない

**pyocd がインストールされていない場合:**
```bash
pip3 install pyocd
```

**`pyocd flash` で "Target type nrf54l15 not recognized" エラー:**
ターゲット名は `nrf54l` です（`nrf54l15` ではない）。
```bash
# 正しいコマンド
pyocd flash -t nrf54l firmware.hex

# 対応ターゲット一覧の確認
pyocd list --targets | grep nrf
```

**`west flash --runner nrfutil` が失敗する場合:**
XIAO nRF54L15 は CMSIS-DAP を使用しているため、nrfutil ランナーでは動作しません。
代わりに pyocd を使用してください。

**"NRF54L15 is not in a secure state" 警告:**
正常な動作です。フラッシュは問題なく実行されます。

### ビルドエラー

**Zephyr モジュールの更新:**
```bash
cd /opt/nordic/ncs/v2.9.2
west update
```

**ボード定義の確認:**
nRF54L15 のボード定義が NCS に含まれていることを確認してください。

## ファイル構造

```
nrf54l15/
├── CMakeLists.txt              # ビルド設定
├── prj.conf                    # Zephyr 設定
├── west.yml                    # Zephyr マニフェスト
├── flash.sh                    # フラッシュスクリプト
├── README.md                   # このファイル
├── src/
│   ├── main.c                  # メインアプリケーション
│   ├── adpcm.c/h               # ADPCM コーデック
│   └── audio_capture.c/h       # PDM オーディオキャプチャ
└── boards/
    └── xiao_nrf54l15_sense.overlay  # ボード設定
```

## 関連プロジェクト

- **nrf52840/**: XIAO nRF52840 Sense 用ファームウェア
- **main/**: XIAO ESP32S3 Sense 用ファームウェア
- **mac_client/**: Mac 用 Python クライアント（共通）

## ライセンス

MIT License
