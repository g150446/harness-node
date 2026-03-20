# nrf54-speaker 保守・運用ガイド

Seeed XIAO nRF54L15 Sense + M5Stack SPK2 (MAX98357A) でインターネットラジオを BLE 経由でリアルタイム再生するシステム。

---

## システム概要

```
インターネットラジオ (MP3 ストリーム)
    │
    ▼  ffmpeg subprocess (Mac)
  16kHz モノラル 16-bit PCM
    │
    ▼  BLE Write Without Response  [seq:1][0xAA:1][PCM:238 bytes]
  nRF54L15 GATT Audio RX characteristic ("SpeakerBridge")
    │
    ▼  Ring buffer (16384 bytes = ~500ms)
  I2S feeder スレッド (mono → stereo upmix, underrun → silence)
    │
    ▼  i2s20 → MAX98357A → スピーカー
```

**BLE デバイス名**: `SpeakerBridge`
**実効サンプルレート**: 15873 Hz (nRF54L15 の MCLK 制約による実際のレート)
**BLE スループット**: MTU=244, DLE=251 → ~31 KB/s

---

## ハードウェア配線

| XIAO ピン | SoC    | SPK2        | 用途                |
|-----------|--------|-------------|---------------------|
| GND (2)   | GND    | GND (pin2)  | グランド共通         |
| 3V3 (3)   | 3V3OUT | 3V3 (pin7)  | IC 電源 3.3V        |
| D0 (4)    | P1.04  | BCLK (pin3) | I2S ビットクロック   |
| D1 (5)    | P1.05  | LRCK (pin4) | I2S ワードセレクト   |
| D2 (6)    | P1.06  | SDATA (pin5)| I2S データ           |

> **重要**: SPK2 の 5VIN (pin8) は**接続不要**。SD_MODE は基板上 10kΩ プルアップ済みで配線不要。

---

## クイックスタート

### 1. 依存ツールのインストール

```bash
# Mac
brew install ffmpeg
pip install pyocd
cd mac_client && pip install bleak
```

### 2. ファームウェアの書き込み（初回 / USB）

```bash
cd nrf54-speaker
./build_and_flash.sh
```

### 3. シリアルモニタで起動確認

```bash
screen $(ls /dev/tty.usbmodem* | head -1) 115200
```

正常起動時:
```
nrf54-speaker BLE audio bridge (build: Mar 20 2026 ...)
I2S configured at 16000 Hz
I2S started
BLE advertising as SpeakerBridge
```

### 4. Mac からストリーミング再生

```bash
cd mac_client
python3 speaker_client.py                                    # デフォルト: Capital Radio
python3 speaker_client.py http://your.radio.stream/path      # 任意の URL
```

正常動作時の表示:
```
Connected to SpeakerBridge (MTU=244)
Streaming: 133.4 pkt/s, 31.0 KB/s  (queue=30)
```

シリアルモニタ側:
```
BLE connected
audio RX: 501 pkts, ring=12000/16384 bytes   ← ring が満杯にならず変動していれば正常
```

---

## BLE OTA アップデート

```bash
# ビルドのみ
cd nrf54-speaker
./build_and_package_ota.sh

# OTA 転送
cd ../mac_client
python3 ota_updater.py ../nrf54-speaker/ota_update.bin --device SpeakerBridge
```

---

## プロジェクト構成

```
nrf54-speaker/
├── CMakeLists.txt
├── prj.conf                              # Kconfig (BLE + I2S + MCUmgr)
├── sysbuild.conf                         # MCUboot 有効化
├── sysbuild/
│   └── mcuboot/boards/
│       └── xiao_nrf54l15_nrf54l15_cpuapp.conf  # MCUboot 用 nRF54L15 設定
├── boards/
│   └── xiao_nrf54l15_nrf54l15_cpuapp.overlay   # I2S20 ピン + UART コンソール
├── src/
│   └── main.c                            # BLE GATT + I2S feeder
├── build_and_flash.sh                    # ビルド + USB フラッシュ
├── build_and_package_ota.sh             # ビルド + OTA バイナリ生成
└── ota_update.bin                        # 最新の OTA ペイロード

mac_client/
├── speaker_client.py                     # BLE オーディオストリーミングクライアント
└── ota_updater.py                        # BLE OTA アップデーター (--device SpeakerBridge)
```

---

## パケット仕様

| フィールド | サイズ | 値       |
|-----------|--------|----------|
| seq       | 1 byte | 0x00〜FF (ループ) |
| magic     | 1 byte | 0xAA     |
| PCM data  | 238 bytes | 16kHz モノラル 16-bit LE |
| 合計      | 240 bytes | ≤ MTU-3 = 241 bytes |

**送信レート**: 238 / (15873 × 2) ≒ **7.498ms / パケット = 133.4 pkt/s**

---

## GATT プロファイル

| 項目 | UUID |
|------|------|
| Speaker Service   | `00000020-0000-1000-8000-00805f9b34fb` |
| Audio RX char     | `00000021-...` Write / Write Without Response |
| Build Info char   | `00000022-...` Read (ビルドタイムスタンプ) |

BLE 接続時に接続インターバル 7.5〜10ms を要求（スループット確保のため）。

---

## 重要な技術的知見

### 1. I2S20 は Port 1 (P1) ピン専用【最重要】

nRF54L15 の `i2s20` はグローバルドメイン (0x500DD000) のペリフェラル。
グローバルドメインの GPIO はすべて **Port 1 (P1)** に接続されており、
Port 2 (P2) のピンを割り当てても BCLK が生成されず音が出ない。

```
P0, P1 → グローバルドメイン → I2S20 ○
P2     → ローカルドメイン専用 → I2S20 ✗
```

公式テストオーバーレイ参考:
`/opt/nordic/ncs/v2.9.2/zephyr/tests/drivers/i2s/*/boards/nrf54l15dk_nrf54l15_cpuapp.overlay`

### 2. I2S の実際のサンプルレートは 15873 Hz

ファームウェアの設定値は 16000 Hz だが、nRF54L15 の MCLK 制約により
実際の出力は **15873 Hz** になる。Mac 側のペーシングはこれに合わせること。

```python
# speaker_client.py
I2S_ACTUAL_HZ = 15873
PACKET_INTERVAL = CHUNK_BYTES / (I2S_ACTUAL_HZ * 2)  # 7.498ms
```

### 3. BLE MTU とパケットサイズの制約

```
MTU = 244
最大ペイロード = MTU - 3 = 241 bytes
パケット構成 = 1 (seq) + 1 (magic) + 238 (PCM) = 240 bytes  ≤ 241 ✓

240 bytes を超えると CoreBluetooth が書き込みをサイレントドロップする
```

### 4. I2S feeder スラブタイムアウト問題

**症状**: `ring=16384/16384` が変わらない、音が出ない
**原因**: I2S ドライバがスラブを全保持したまま返さなくなる → feeder が `k_mem_slab_alloc(K_FOREVER)` でデッドロック
**対処**: `K_FOREVER` を `K_MSEC(200)` に変更し、タイムアウト時に `i2s_recover()` を呼ぶ

```c
// src/main.c i2s_feeder_thread 内
int ret = k_mem_slab_alloc(&i2s_tx_slab, &slab, K_MSEC(200));
if (ret < 0) {
    LOG_ERR("slab alloc timeout, recovering I2S");
    i2s_recover(dev);  // DROP → PREPARE → START
    continue;
}
```

### 5. BLE 接続時にリングバッファをリセット

再接続時に前セッションの古いデータが残っていると初期バーストで ring が溢れる。
`ble_connected()` で `ring_buf_reset()` を呼ぶことで解消。

### 6. リングバッファのオーバーフロー保護を入れない

`ring_buf_space_get()` でチェックしてドロップすると、初期バースト時に
全パケットがドロップされて完全無音になる。`ring_buf_put()` を無条件で呼ぶ
（Zephyr の実装が収まる分だけ書く）。

### 7. pyocd のターゲット名

```bash
pyocd flash -t nrf54l <hex_file>   # ○
pyocd flash -t nrf54l15 ...        # ✗ (認識されない)
```

---

## トラブルシューティング

### 音が出ない

| シリアルログ | 原因 | 対処 |
|-------------|------|------|
| `ring=16384/16384` が変わらない | I2S feeder デッドロック | ファームを `K_MSEC(200)` 版にアップデート |
| `ring=` が 0 のまま | BLE パケットが届いていない | speaker_client.py の UUID / MTU を確認 |
| `slab alloc timeout, recovering I2S` が繰り返す | I2S ハードウェア異常 | デバイスを再起動 |
| `bad packet` ログ | パケット形式の不一致 | magic byte が 0xAA か確認 |

### 音が割れる / 途切れる

| 症状 | 原因 | 対処 |
|------|------|------|
| 断続的にブツブツ | BLE コネクションインターバルが長い | `bt_conn_le_param_update(6, 8, ...)` で 7.5ms を確保 |
| 最初の数秒だけ割れる | 初期バーストで ring が一時的に溢れる | queue=30 の pre-buffer が安定するまで待つ |
| `queue=0` が続く | ffmpeg が遅い / ネットワーク不安定 | URL を変えるかローカルファイルで試す |

### BLE が見つからない

```bash
# デバイスが "SpeakerBridge" でアドバタイズしているか確認
# iPhone の LightBlue アプリ等でスキャン
```

---

## 動作確認済み環境

- NCS: v2.9.2
- ボード: `xiao_nrf54l15/nrf54l15/cpuapp`
- ホスト OS: macOS
- Python: 3.x, bleak, ffmpeg
- 確認日: 2026-03-20
