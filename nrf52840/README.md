# Voice Bridge BLE - XIAO nRF52840 Sense

Seeed Studio XIAO nRF52840 Sense 用の BLE 音声ストリーミングファームウェア

## 概要

XIAO nRF52840 Sense の BLE を使用して、Mac に音声データ（またはダミーデータ）をストリーミングします。

**注意**: 現在のバージョンはダミーデータを送信します。実際の PDM マイクを使用するには、I2S/PDM ドライバーの設定が必要です。

## ハードウェア要件

- Seeed Studio XIAO nRF52840 Sense
- Mac (Bluetooth 5.0 以上)
- USB ケーブル

## ソフトウェア要件

- Nordic Connect SDK (NCS) v2.9.2
- west (Zephyr ビルドツール)

## クイックスタート

### 1. 自動フラッシュスクリプト

```bash
cd nrf52840
./flash.sh
```

スクリプトが自動的に以下の処理を実行します：
1. ソースファイルのコピー
2. ビルド（数分かかります）
3. ブートローダーモードへの指示
4. UF2 ファイルのコピー

**ブートローダーモードの手順**:
1. リセットボタンを押しながら
2. ブートボタンを押す（またはリセットを 2 回素早く押す）
3. リセットボタンを離す
4. 「XIAO-NRF52」というドライブが現れる

スクリプトが自動的に UF2 ファイルをコピーします。

### 3. Mac クライアントの実行

```bash
cd mac_client
python3 voice_bridge_recorder.py
```

### 4. 操作

- 接続後、`r` キーを押して録音開始
- `s` キーで録音停止
- `q` キーで終了

録音ファイルは `mac_client/output/recording_*.wav` に保存されます。

## 手動ビルド・フラッシュ

### ビルド

```bash
export PATH="/opt/nordic/ncs/toolchains/b8efef2ad5/bin:$PATH"
cd /opt/nordic/ncs/v2.9.2
rm -rf build
west build -b xiao_ble/nrf52840/sense nrf/samples/voice_bridge
```

### フラッシュ（UF2 ブートローダー）

1. **ブートローダーモードにする**:
   - リセットボタンを押しながら
   - ブートボタンを押す（またはリセットを 2 回素早く押す）
   - 「XIAO-NRF52」ドライブが現れる

2. **UF2 ファイルをコピー**:
```bash
cp /opt/nordic/ncs/v2.9.2/build/voice_bridge/zephyr/zephyr.uf2 /Volumes/XIAO-NRF52/
```

3. **自動リセット**: デバイスが自動的にリセットされ、新しいファームウェアが起動します。

## 仕様

### BLE
- デバイス名：VoiceBridge
- サービス UUID: 00000000-0000-0000-0000-000000000000
- TX 特性 (Notify): 00000002-0000-1000-8000-00805f9b34fb
- RX 特性 (Write): 00000003-0000-1000-8000-00805f9b34fb

### 送信データ
- パケットサイズ：202 バイト（ヘッダー 2 バイト + PCM データ 200 バイト）
- パケット形式：[シーケンス番号][同期バイト 0xAA][PCM データ...]
- 送信間隔：10ms（100 パケット/秒）

**注意**: 現在のバージョンはダミーデータを送信します。

## シリアルコマンド

デバッグ用シリアル接続でも制御できます：

- `r` - 録音開始
- `s` - 録音停止
- `h` - ヘルプ表示

## トラブルシューティング

### デバイスが見つからない

1. XIAO nRF52840 がフラッシュされていることを確認
2. XIAO をリセット（リセットボタンを押す）
3. Mac の Bluetooth がオンになっていることを確認
4. 数秒待ってから再度試行

### フラッシュできない

**ブートローダーモードの確認**:
1. リセットボタンを押しながら
2. ブートボタンを押す（またはリセットを 2 回素早く押す）
3. 「XIAO-NRF52」ドライブが表示されることを確認

**手動で UF2 ファイルをコピー**:
```bash
cp /opt/nordic/ncs/v2.9.2/build/voice_bridge/zephyr/zephyr.uf2 /Volumes/XIAO-NRF52/
```

### ビルドエラー

Kconfig エラーが発生する場合は、既存の BLE サンプルをベースにしてください：

```bash
cp -r /opt/nordic/ncs/v2.9.2/nrf/samples/bluetooth/peripheral_lbs \
      /opt/nordic/ncs/v2.9.2/nrf/samples/voice_bridge
```

その後、ソースファイルをコピー：
```bash
cp /path/to/nrf52840/src/main.c /opt/nordic/ncs/v2.9.2/nrf/samples/voice_bridge/src/
cp /path/to/nrf52840/src/adpcm.* /opt/nordic/ncs/v2.9.2/nrf/samples/voice_bridge/src/
```

## 実際の PDM マイクを使用するには

現在のバージョンはダミーデータを送信しています。実際の PDM マイクを使用するには、以下の設定が必要です：

1. **prj.conf に PDM/I2S を追加**:
```
CONFIG_PDM=y
CONFIG_PDM_NRFQDEC=y
```

2. **main.c を修正**: I2S/PDM ドライバーを使用して音声データを取得

3. **XIAO nRF52840 Sense のピン設定**:
   - PDM_CLK: P1.00 (GPIO 32)
   - PDM_DATA: P0.16 (GPIO 16)

## ライセンス

MIT License
