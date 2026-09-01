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
| `stickc_plus2/VERSION` | `esp_app_desc.version`（例 `0.1.1`） |

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

ESP32 PDM は nRF DMIC より静か。ファームで:

- 起動 discard ~400 ms
- DC IIR 除去
- **デジタル gain ×16**（M5Unified 相当）+ soft clip

録音開始後 ~1 s の serial に `Mic level peak=… rms=… gain=16` が出る。  
peak が極端に小さい（~0）場合は slot/配線、数百未満なら gain 再調整を検討。

---

## シリアル（115200）

| キー | 動作 |
|------|------|
| `r` | 録音開始 |
| `s` | 録音停止 |
| `c` / `1` | single-click 相当 |
| `d` / `2` | double-click 相当 |
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
