# M5StickC Plus2 / HarnessNode-Plus2

ESP-IDF ファーム: `stickc_plus2/`  
BLE 名: **`HarnessNode-Plus2`**  
Audio Service UUID: XIAO `HarnessNode` と同じ  
`00000001/0002/0003-0000-1000-8000-00805f9b34fb`

Handy / Android は `HarnessNode` 接頭辞で接続可能。

---

## ハードウェア

| 項目 | 値 |
|------|-----|
| SoC | ESP32-PICO-V3-02（**esp32s3 ではない**） |
| Flash / PSRAM | 8 MB / 2 MB |
| USB-UART | CH9102 → `/dev/cu.usbserial-…` |
| Mic | SPM1423 **PDM** CLK=**G0**, DIN=**G34** |
| BtnA | **G37** active-low（single / double） |
| POWER_HOLD | **G4 = HIGH 必須**（起動直後に保持） |
| Status LED | G19（IR と共有、active-high） |
| 電池 | 200 mAh（ADC G38、v1 では BAS 未実装） |

---

## ビルド / USB フラッシュ

```bash
source ~/esp/esp-idf/export.sh   # 例: ~/esp/esp-idf
cd harness-node

# ボード切替（デフォルトも stickc_plus2）
idf.py -DHN_BOARD=stickc_plus2 set-target esp32
idf.py -DHN_BOARD=stickc_plus2 build
idf.py -DHN_BOARD=stickc_plus2 -p /dev/cu.usbserial-XXXX flash monitor
```

| 変数 / ファイル | 意味 |
|-----------------|------|
| `-DHN_BOARD=stickc_plus2` | コンポーネント `stickc_plus2/` をリンク |
| `sdkconfig.defaults.esp32` | Flash 8MB、dual OTA、NimBLE、UART コンソール |
| `stickc_plus2/VERSION` | `esp_app_desc.version`（例 `0.1.3`） |

**注意:** 以前 `esp32s3` でビルドした `build/` がある場合は退避してから `set-target esp32` する。

CH9102 が見えないとき: USB 抜き → 電源長押し（緑 LED）→ 再挿し。

---

## BLE OTA（nordic と同じ `ota_updater.py`）

パーティション: `ota_0` / `ota_1`（`stickc_plus2/partitions_ota.csv`）  
SMP UUID: nordic MCUmgr と同じ（`8D53DC1D-…` / `DA2E7828-…`）

```bash
# 1. VERSION を上げる
# 2. パッケージ
./stickc_plus2/build_and_package_ota.sh
# → stickc_plus2/ota_update.bin

# 3. macOS は Terminal で（Bluetooth TCC）
python3 mac_client/ota_updater.py --device HarnessNode-Plus2 stickc_plus2/ota_update.bin
```

- dual-OTA レイアウトに変えた**直後**は USB full flash が必要（bootloader + table + app）。
- OTA 中は Handy / Android など他 BLE クライアントを切る。
- 成功: `OTA verified: uploaded image is active and confirmed in slot 0`
- 起動約 3 秒後に auto-confirm（それ以前のクラッシュは rollback）
- 成果物 `ota_update.bin` は git 管理外（ビルドで生成）

詳細・nRF との共通注意: [`ota_update_notes.md`](ota_update_notes.md)

---

## プロトコル（v1）

| 入力 | 動作 |
|------|------|
| BtnA **single** / serial `c` | BLE 接続中に録音トグル。TX `0x14` のあと `0x01` or `0x02` |
| BtnA **double** / serial `d` | TX `0x12` のみ（録音しない） |
| RX `0x01` / `0x00` | ホスト start / stop |
| RX `0x05 mode` | モード（0=NORMAL, 1=DRIVING）。TX `0x40` |
| 音声 | 16 kHz mono PCM `[seq][0xAA][i16 LE…]`、ペイロード最大 200 B |

### マイク処理（Handy STT 用）

内蔵 SPM1423（`pin_ws = G0`, `pin_data_in = G34`, M5Unified `M5Unified.cpp:2143`）。

**確定した設定**（`MIC_PDM_DEFAULT`, `stickc_plus2/main.c`）:

| 項目 | 値 |
|---|---|
| クロック経路 | `MIC_CLK_IDF`（`i2s_channel_init_pdm_rx_mode()` に任せる） |
| `dn_sample_mode` | `I2S_PDM_DSR_16S`（PDM CLK 2.048 MHz） |
| `slot_mask` | **`I2S_PDM_SLOT_LEFT`** |
| `slot_mode` | `I2S_SLOT_MODE_MONO` |
| gain | ×4（`MIC_GAIN_DEFAULT`、シリアル `g` で変更可） |

#### 真因は slot_mask だった（M5Unified の既定は移植できない）

M5Unified の `mic_config_t` 既定は `input_only_right`（`Mic_Class.hpp`）で、
Plus2 でも上書きしていない。しかしこれを IDF のクロック経路にそのまま持ち込むと
**マイクが載っていない側のスロットを読む**ことになる。

2026-09-02 のシリアル `m` スイープ実測（9 構成、発話しながら各 0.3 s 計測）:

| # | 構成 | verdict | rms | zcr |
|---|---|---|---|---|
| 1 | IDF DSR_8S **RIGHT** | `CONST`（-30935 固定） | 30935 | 0 Hz |
| 2 | IDF DSR_16S **RIGHT** | `NOISE` | 21414 | 3362 Hz |
| **3** | IDF DSR_8S **LEFT** | **`SIGNAL`** | 1108 | 472 Hz |
| **4** | IDF DSR_16S **LEFT** | **`SIGNAL`** | 1157 | 431 Hz |
| 5 | IDF DSR_8S STEREO | **L=`SIGNAL`** / R=`CONST` | 1022 | 234 Hz |
| 6 | IDF DSR_16S STEREO | **L=`SIGNAL`** / R=`NOISE` | 1312 | 293 Hz |
| 7 | M5-raw DSR_16S **RIGHT** | `SIGNAL` | 924 | 382 Hz |
| 8 | M5-raw DSR_16S STEREO | L=`CONST` / **R=`SIGNAL`** | 1231 | 378 Hz |
| 9 | M5-raw DSR_8S RIGHT | `SIGNAL`（ただし出力 32 kHz） | 970 | 336 Hz |

**IDF クロックでは LEFT、M5 生レジスタクロックでは RIGHT** に音がある。
`bclk_div` が違う（IDF は 8 以上を強制 / M5 は `div_m = 2`）ため WS の位相が
1 スロットずれるのが理由。M5Unified の `input_only_right` は
**M5 自身のクロックとセットでのみ正しく**、IDF クロックには成り立たない。

`#9`（M5 クロック + DSR_8S）は M5 のクロックが DSR128 前提で固定されているため
デシメーションだけが半分になり、出力が 32 kHz になる（同じ 0.3 s で n が 2 倍）。使わない。

`DSR_8S`（1.024 MHz）ではなく `DSR_16S`（2.048 MHz）を選ぶのは、SPM1423 の
normal mode 下限が 1.0 MHz で `DSR_8S` がその縁に乗るため。

#### 判定は peak ではなくゼロ交差レートで

16 kHz なのでナイキストは 8 kHz。白色雑音の zcr は約 4000 Hz に張り付く:

| verdict | 条件 | 意味 |
|---|---|---|
| `CONST` | `zcr = 0` / distinct 一桁 | PDM DOUT がアイドル。無音のスロットを読んでいる |
| `NOISE` | `zcr > 2500 Hz` | 広帯域雑音。マイクが載っていないスロット、またはデコード不成立 |
| `SIGNAL` | `zcr` 100–2500 Hz かつ distinct ≥ 100 | 実音声。目安は 300–1500 Hz |
| `WEAK` | 上記以外 | レベル不足 |

**注意**: この verdict は DC 除去**前**の生ストリームに対して計算する。
DC オフセットが振幅より大きいと符号が反転しなくなり、正常な音声でも
`zcr ≈ 0` → `WEAK` と出る（実測例: `mean=-1925 rms=2398 zcr=1Hz` だが
処理後は `peak=8972 rms=453 clip=0%` で正常）。スイープでは DC が小さいので有効、
録音時のログでは `Mic proc` 側と `head` ダンプを併せて見ること。

#### 実績値

| | 張り付き | rms | peak | zcr | distinct |
|---|---|---|---|---|---|
| XIAO `HarnessNode`（基準） | 0 % | 560–692 | — | 800–1004 Hz | — |
| Plus2 `RIGHT`（不良） | 84–90 % | 30995 | 32768 | 3810–3909 Hz | 2783 |
| **Plus2 `LEFT` + gain 4** | **0.00 %** | 1345 | 13236 | 1723 Hz | 3057 |

シリアル側: `Mic proc peak=8972 rms=453 gain=4 clip=0%`、
`Mic raw head: 4023 4033 4000 4017 3955 3987 3898 …`（滑らかな波形）。

#### PDM クロックの 2 系統

`mic_pdm_cfg_t.clk_path` で切り替える。既定は `MIC_CLK_IDF`。

| | `MIC_CLK_IDF` | `MIC_CLK_M5` |
|---|---|---|
| 実装 | `i2s_channel_reconfig_pdm_rx_clock()` | `apply_m5_mic_clock()`（生レジスタ） |
| bclk_div | **8 以上を強制**（`i2s_pdm.c` `I2S_PDM_RX_BCLK_DIV_MIN`） | `div_m = 2` |
| mclk | 16.384 MHz（160 MHz / 9.7656） | 4.096 MHz（160 MHz / 39.0625） |
| 音のあるスロット | **LEFT** | **RIGHT** |

M5Unified は `Mic_Class.cpp:317` で `sample_rate_hz = 48000; // dummy setting` と
書いたうえで生レジスタで上書きしており、**IDF のクロック計算を使っていない**。
`MIC_CLK_M5` はその ESP32-classic (I2S HW v1) 分岐（`Mic_Class.cpp:528-548`）の移植で、
`atom_echo_s3r/main.c:621` の同名関数は I2S HW v2 用なのでレジスタ配置が異なる。
`i2s_channel_enable()` の後に適用する必要がある（`set_microphone_enabled()` が自動で呼ぶ）。

両方 `SIGNAL` になるが、生レジスタを叩かず IDF 更新にも強い `MIC_CLK_IDF` を既定にしている。

#### ファーム処理

- 起動時 DC 収束待ち（`|x-offset|` 閾値、上限 ~400 ms）のあと `recording_started`
  （実測 63 ms で収束）
- DC IIR 除去 + デジタル gain ×4 + ハードクリップ
- STEREO 取り込み時は `mic_stereo_pick` のスロットだけを詰めてから DSP/BLE へ
- 録音開始後 ~1 s の serial:
  - `Mic raw n=… min=… max=… mean=… rms=… zcr=…Hz distinct=… CONST|NOISE|WEAK|SIGNAL`
  - `Mic raw head: …`（生サンプル先頭 32 個。波形か全振幅ジャンプかが一目で分かる）
  - `Mic proc peak=… rms=… gain=… clip=…%`
- **`Mic raw` / `Mic proc` は BLE 接続中しか出ない。** 録音ループは
  `audio_conn_handle == BLE_HS_CONN_HANDLE_NONE` のとき読み出しごとスキップするので、
  シリアル `r` だけでは統計が出ない。BLE クライアントを繋いだ状態で測ること。

#### 落とし穴: `i2s_channel_read()` のタイムアウトは「ミリ秒」

```c
i2s_channel_read(h, buf, bytes, &read, MIC_READ_TIMEOUT_MS);   /* 正 */
i2s_channel_read(h, buf, bytes, &read, pdMS_TO_TICKS(50));     /* 誤 */
```

第 5 引数は **ミリ秒**で、IDF が内部で `pdMS_TO_TICKS()` を適用する
（`esp_driver_i2s/i2s_common.c`）。二重に変換すると
`pdMS_TO_TICKS(50)` = 5 → 内部で `pdMS_TO_TICKS(5)` = **0 tick**
（`CONFIG_FREERTOS_HZ=100`）となり、ノンブロッキング読み出しになって
DMA にデータが揃っていなければ即 `ESP_ERR_TIMEOUT` を返す。

実害（0.1.2 で発生）:

- 診断スイープが全構成 `no samples` で 140 ms で終わる（測定が一切走らない）。
- **録音開始時の DC 収束ループが即 `break`** し、`mic_dc_offset = 0` のまま録音開始。

read の失敗は握りつぶさずログすること（`Sweep #n read failed:` / `DC settle read failed:`）。

#### 落とし穴: `VERSION` は configure 時にしか読まれない

`CMakeLists.txt` は `file(READ stickc_plus2/VERSION)` で読むため、`VERSION` を
書き換えただけの `idf.py build` では `esp_app_desc.version` が古いままになる
（OTA のバージョン確認が通らない）。`stickc_plus2/build_and_package_ota.sh` は
`PROJECT_VER` の export と `reconfigure` を行うので、**OTA は必ずこのスクリプト経由**で
パッケージすること。USB フラッシュ前に手で `idf.py build` する場合は
`idf.py -DHN_BOARD=stickc_plus2 reconfigure` を挟む。

#### 診断スイープ

シリアル `m` で 9 構成（クロック経路 × DSR × slot）を各 0.5 s 測る。約 5 秒間
**途切れず**喋り続けること（黙った構成は `CONST` と誤判定される）。
STEREO 構成は L / R を別行で出すので、**どちらのスロットが実マイクか**が直接分かる。
ゲインはシリアル `g` で 1→2→4→8→16 と巡回でき、較正に再フラッシュは不要。

BLE 側の確認には `mac_client/esp32_controller.py` を使うが、
`find_device()` はスキャンで**最初に見つかった既知デバイス**を掴むため、
XIAO `HarnessNode` が同時に起動しているとそちらに繋がる。
`VoiceBridgeClient(device_names=("HarnessNode-Plus2",))` で固定するか、
XIAO の電源を切ること。

---

## シリアル（115200）

| キー | 動作 |
|------|------|
| `r` | 録音開始 |
| `s` | 録音停止 |
| `c` / `1` | single-click 相当 |
| `d` / `2` | double-click 相当 |
| `m` | PDM 設定スイープ 9 構成（録音中は不可） |
| `g` | mic gain 巡回 1→2→4→8→16 |
| `h` | ヘルプ |

---

## ボード切替（Atom Echo S3R）

```bash
idf.py -DHN_BOARD=atom_echo_s3r set-target esp32s3 build
```

Echo は別コンポーネント `atom_echo_s3r/`（保留中のデスクノード）。  
詳細のたたき台: [`atom_echo_s3r_guide.md`](atom_echo_s3r_guide.md)

---

## ファイル一覧

| パス | 役割 |
|------|------|
| `stickc_plus2/main.c` | HOLD, PDM, BtnA, NimBLE audio, LED |
| `stickc_plus2/smp_ota.c` | MCUmgr 互換 SMP OTA |
| `stickc_plus2/partitions_ota.csv` | dual OTA |
| `stickc_plus2/build_and_package_ota.sh` | OTA bin 生成 |
| `stickc_plus2/VERSION` | アプリバージョン |
| `CMakeLists.txt` | `HN_BOARD` で EXTRA_COMPONENT_DIRS 切替 |
