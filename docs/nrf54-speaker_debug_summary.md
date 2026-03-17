# nrf54-speaker デバッグまとめ

## 概要

`nrf54-speaker` は Seeed XIAO nRF54L15 Sense から M5Stack SPK2(MAX98357A) へ I2S で音声再生するサンプルです。

今回の調査では、BLE 再生コマンド送信から I2S 書き込み開始まではソフトウェア上で正常に到達していることを確認しました。一方で、実機では音が聞こえない状態が継続しており、現時点ではハードウェア接続または SPK2 側の受け条件に起因する可能性が高いです。

## 接続前提

期待している配線は以下です。

| XIAO | SoC Pin | SPK2 | 用途 |
|---|---|---|---|
| D8 | P2.01 | BCLK | I2S bit clock |
| D9 | P2.04 | LRCLK | I2S word select |
| D10 | P2.02 | DIN | I2S data |
| 5V | VBUS | 5VIN | 電源 |
| GND | GND | GND | GND |

補足:

- SPK2 は `5V` 駆動前提
- `SD_MODE` は基板側プルアップ済み前提
- MCLK は未使用

## 確認できた事実

### 1. BLE 側は正常

- `SpeakerBridge` は BLE 広告される
- `mac_client/nrf54_controller.py --play-once` で接続成功
- 再生 characteristic へ `0x01` を送信成功
- GDB で `write_play_cmd()` にブレークし、`len=1`, `buf[0]=0x01` を確認

結論: BLE コマンドはファームウェアまで届いている

### 2. I2S 初期化は正常

- `audio_thread_fn()` は起動している
- `device_is_ready(i2s_dev)` で停止していない
- `i2s_configure(..., I2S_DIR_TX, ...)` の戻り値は `0`

結論: I2S ドライバ初期化は成功している

### 3. 再生ループにも入っている

- GDB で `i2s_write()` 到達確認
- `i2s_trigger(I2S_TRIGGER_START)` の戻り値 `0` を確認

結論: ソフトウェア上は BLE → semaphore → audio thread → I2S TX 開始まで進んでいる

## 修正・変更した内容

### 1. `mac_client/nrf54_controller.py` 修正

- デバイス名を `VoiceBridge` → `SpeakerBridge` に更新
- 再生 characteristic UUID を追加
- `p / play` コマンドを追加
- `--play-once` を追加
- `--address` 指定を追加
- 広告名が見えない場合のサービス UUID プローブを追加

### 2. 埋め込み音声データの正規化

元の PCM はピークが約 `4.9%FS` しかなく、非常に小音量でした。

- `tools/wav_to_c.py` に正規化処理を追加
- `audio_data.c` を再生成
- 正規化後ピークは約 `87.8%FS`

### 3. sysbuild の正しいイメージで再書き込み

`nrf54-speaker/zephyr/zephyr.hex` ではなく、sysbuild の `merged.hex` を書き込む必要がありました。

有効だったコマンド:

```bash
pyocd flash -t nrf54l /tmp/nrf54-speaker-build/merged.hex
```

この書き込み後に `SpeakerBridge` の BLE 広告が安定して確認できました。

### 4. 再生を聞き取りやすくする追加調整

実機確認用として以下を試しました。

- 先頭に大きい `1kHz` テストトーンを追加
- 出力サンプルレートを `32kHz` に変更
- I2S を stereo 複製出力から mono 出力へ変更

これらを適用したファームはビルド・書き込み済みです。

## それでも未解決の点

ソフトウェア経路は正常に見えるにもかかわらず、実音が確認できていません。

このため、残る有力候補は以下です。

### 1. 物理配線の不一致

最有力です。

- D8/D9/D10 のどれかが実配線で入れ替わっている
- XIAO の右側ピン番号と実際の基板ピンを取り違えている
- GND 共通が不十分

### 2. SPK2 への電源条件

- `5VIN` に十分な 5V が来ていない
- `VBUS` 供給が不安定
- SPK2 が電源投入されていない

### 3. SPK2 の入力モード相性

MAX98357A 自体は I2S を受けますが、SPK2 基板の `SD_MODE` 固定状態と現在の I2S スロット構成の相性が残っています。

今回、stereo 複製と mono 出力の両方を試しましたが、まだ実音確認には至っていません。

### 4. 実ボード側ピン multiplex の差異

devicetree 上では XIAO board 定義と overlay は整合しています。

- D8 = `gpio2.1`
- D9 = `gpio2.4`
- D10 = `gpio2.2`

ただし、実機の取り出しと Seeed の表記が異なる場合は、I2S はソフト上で正常でも物理的には別ピンへ出ている可能性があります。

## 現時点の結論

現状の最大結論は次のとおりです。

1. `mac_client` 側の BLE 再生コマンド送信は修正済みで、実動確認済み
2. `nrf54-speaker` 側は BLE 受信から I2S TX 開始までソフト的には正常
3. 無音の主原因は、現時点ではソフトよりもハード配線・電源・SPK2 側条件にある可能性が高い

## 次にやるべきこと

### 優先度高

1. XIAO ↔ SPK2 の実配線写真を見ながら、D8/D9/D10/5V/GND を再確認する
2. SPK2 の `5VIN` と GND をテスタで確認する
3. BCLK / LRCLK / DIN をロジアナまたはオシロで観測し、BLE 再生時に信号が出ているか確認する

### 優先度中

4. SPK2 の `SD_MODE` 周辺回路を実基板で再確認する
5. 必要なら LRCLK/BCLK/DIN の候補ピンを入れ替えて最小構成で再検証する

### 優先度低

6. `uart20` ログを確実に読める別経路を用意し、ランタイムログを常時見えるようにする

## 関連ファイル

- `nrf54-speaker/src/main.c`
- `nrf54-speaker/tools/wav_to_c.py`
- `nrf54-speaker/src/audio_data.c`
- `mac_client/nrf54_controller.py`
- `nrf54-speaker/boards/xiao_nrf54l15_nrf54l15_cpuapp.overlay`

