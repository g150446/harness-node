Seeed Studioの公式Wiki（MSM261D4030H1AP PDMマイクの仕様）の情報を反映し、ピンアサインやI2S設定をより厳密に指定した、AIコーディングエージェント向けのシステム設計指示書（Markdown）を再作成しました。

このプロジェクトでは、**「XIAO（マイク）→ Mac」**のストリーミングだけでなく、**「Mac → XIAO（外部スピーカー）」**の双方向伝送を想定した構成にしています。
※XIAO ESP32S3 Senseにはスピーカーが内蔵されていないため、外部のI2S DAC（MAX98357A等）を接続する前提の記述を含めています。

---

# Project: Bidirectional BLE Audio Streaming (XIAO ESP32S3 Sense ↔ Mac)

## 1. 概要

Seeed Studio XIAO ESP32S3 SenseのオンボードPDMマイクと外部I2Sスピーカーを使用し、BLE 5.0（カスタムGATT）を介してMac上のPythonスクリプトと双方向で音声を送受信するシステムを構築する。

## 2. ターゲット環境

* **デバイス**: Seeed Studio XIAO ESP32S3 Sense
* **MCU開発環境**: ESP-IDF (v5.x推奨)
* **Mac側環境**: Python 3.10+, macOS
* **通信方式**: BLE 5.0 (Custom GATT Profile / NimBLE Stack)

## 3. ハードウェア詳細仕様

公式Wiki（[https://wiki.seeedstudio.com/xiao_esp32s3_sense_mic/](https://wiki.seeedstudio.com/xiao_esp32s3_sense_mic/)）に基づく。

### A. オンボードマイク (Input)

* **型番**: MSM261D4030H1AP (Digital PDM Microphone)
* **接続方式**: I2S PDM Mode
* **ピンアサイン**:
* `PDM_CLK`: **GPIO 42**
* `PDM_DATA`: **GPIO 41**


* **設定パラメータ**:
* Sample Rate: 16,000 Hz (16bit Mono)
* VDD: 3.3V



### B. 外部スピーカー出力 (Output - 推奨構成)

※双方向通信のため、D0-D2ピンに外部I2S DACを接続する想定。

* **ピンアサイン (例)**:
* `I2S_BCLK`: **GPIO 1 (D0)**
* `I2S_LRCK`: **GPIO 2 (D1)**
* `I2S_DATA`: **GPIO 3 (D2)**



## 4. 技術的要件

### A. ESP-IDF 実装要件

1. **I2S PDMマイク入力**:
* `i2s_pdm_rx_config_t` を使用し、GPIO 41/42を割り当て。
* 取得したPCMデータをバッファリングする。


2. **I2S スピーカー出力**:
* `i2s_std_tx_config_t` を使用。マイクと同じ16kHz/16bitで設定。


3. **コーデック (双方向)**:
* BLE帯域制限のため、**IMA ADPCM (4-bit)** を使用。
* エンコーダ（Mic -> BLE）とデコーダ（BLE -> Speaker）を実装。


4. **BLE NimBLE スタック**:
* **MTUサイズ**: 512 bytes以上に拡張。
* **PHY**: 2M PHYを選択（低遅延・高帯域）。
* **GATT Service/Characteristic**:
* `TX_Characteristic (Notify)`: マイク音声をMacへ。
* `RX_Characteristic (WriteWithoutResponse)`: Macからの音声をスピーカーへ。




5. **マルチコア最適化**:
* Core 0: Bluetoothスタック処理。
* Core 1: I2S PDM/STD 制御およびコーデック処理。



### B. Mac Python 実装要件

1. **ライブラリ**: `bleak` (BLE), `pyaudio` (音声入出力), `numpy` (ADPCM演算)。
2. **処理フロー**:
* XIAOと接続し、MTU 512を要求。2M PHYへの切替を試行。
* マイク入力をADPCMエンコードし、XIAOのRX characteristicへ送信。
* XIAOからのNotificationをADPCMデコードし、スピーカーで再生。



## 5. エージェントへの指示ステップ

### Step 1: PDMマイク初期化 (ESP-IDF)

公式Wikiの仕様に基づき、GPIO 41(Data)とGPIO 42(Clk)を使用したI2S PDM受信の初期化コードを生成してください。`i2s_new_channel` および `i2s_channel_enable` を使用した最新のドライバ形式で記述してください。

### Step 2: ADPCMコーデックの実装

C言語でIMA ADPCMのエンコード/デコード関数を実装してください。16bit PCMを4bitに圧縮し、BLEパケット（シーケンス番号1byte + データ）を構成できるようにしてください。

### Step 3: NimBLE GATTサーバーの構築

カスタムサービスを定義し、MTU拡張と2M PHYの設定を含むNimBLEサーバーを実装してください。特に、大量の音声パケットを扱うためのバッファ管理に注意してください。

### Step 4: Pythonクライアントの実装

`bleak`を用いたMac用スクリプトを生成してください。`pyaudio`のコールバック関数内で、録音・再生・ADPCM変換・BLE送受信を並列処理する構造にしてください。

## 6. 特記事項

* **重要**: ESP32-S3はBluetooth ClassicのHFP/A2DPが使えません。必ずGATT Notify/Writeを用いたカスタムプロトコルにすること。
* **低遅延化**: Connection Intervalを 7.5ms ~ 15ms に設定するようホストに要求するコードを含めてください。
