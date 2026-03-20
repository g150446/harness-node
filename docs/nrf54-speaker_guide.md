# nrf54-speaker 保守・運用ガイド

Seeed XIAO nRF54L15 Sense + M5Stack SPK2 (MAX98357A) で 440 Hz 正弦波を I2S 再生するファームウェア。

---

## ハードウェア配線

| XIAO ピン | SoC | SPK2 | 用途 |
|-----------|-----|------|------|
| 2 (GND)   | GND     | GND (pin2) | グランド共通 |
| 3 (3V3)   | 3V3_OUT | 3V3 (pin7) | IC 電源 3.3V |
| 4 (D0)    | P1.04   | BCLK (pin3) | I2S ビットクロック |
| 5 (D1)    | P1.05   | LRCLK (pin4) | I2S ワードセレクト |
| 6 (D2)    | P1.06   | SDATA (pin5) | I2S データ |

> **注意**: SPK2 の 5VIN (pin8) は **接続不要**。3V3 (pin7) に 3.3V を供給する。
> SD_MODE は SPK2 基板上で 10kΩ プルアップ済みのため配線不要。
> MCLK は MAX98357A では不要。

---

## ビルド・書き込み

### 初回 or USB 書き込み

```bash
cd nrf54-speaker
./build_and_flash.sh
```

pyocd が必要:

```bash
pip install pyocd
```

### BLE OTA 更新

```bash
cd nrf54-speaker
./build_and_package_ota.sh
cd ../mac_client
python3 ota_updater.py ../nrf54-speaker/ota_update.bin
```

> OTA は MCUboot が不要なシンプル構成のため、このプロジェクトでは OTA 非対応。
> `build_and_package_ota.sh` はビルドのみ行い、signed binary がなければ警告を出して終了する。

### シリアルモニタ

```bash
screen $(ls /dev/tty.usbmodem* | head -1) 115200
```

正常起動時の出力:

```
I2S configured OK, starting 440 Hz tone
```

エラー例:

```
I2S device not ready       ← overlay/ピン設定が間違っている
I2S configure failed: -22  ← i2s_config の設定値が不正
slab alloc timeout: -11    ← I2S TX が詰まっている（ピン不一致が多い）
```

---

## プロジェクト構成

```
nrf54-speaker/
├── CMakeLists.txt               # Zephyr ビルド設定
├── prj.conf                     # Kconfig
├── boards/
│   └── xiao_nrf54l15_nrf54l15_cpuapp.overlay  # I2S20 ピン設定
├── src/
│   └── main.c                   # 440Hz 正弦波ループ再生
├── build_and_flash.sh           # ビルド + USB 書き込み
└── build_and_package_ota.sh     # ビルドのみ
```

---

## 重要な技術的知見

### 1. I2S20 は Port 1 (P1) ピンしか使えない【最重要】

nRF54L15 の `i2s20` は **グローバルドメイン** のペリフェラル (0x500DD000) であり、
グローバルドメインの GPIO はすべて **Port 1 (P1)** に接続されている。

Port 2 (P2) はローカルドメイン専用であり、`i2s20` に P2 ピンを割り当てても
ピンマックスが無効になり BCLK が生成されない。初期化は成功するが音は出ない。

| Port | 用途 | I2S20 対応 |
|------|------|----------|
| P0   | グローバルドメイン | ○ |
| P1   | グローバルドメイン | ○ ← 今回使用 |
| P2   | ローカルドメイン専用 | **✗ 使用不可** |

**旧配線 (D8/D9/D10 = P2 系) では絶対に動作しない。**

参考: NCS の公式テストオーバーレイ
`/opt/nordic/ncs/v2.9.2/zephyr/tests/drivers/i2s/*/boards/nrf54l15dk_nrf54l15_cpuapp.overlay`
→ P1.08/P1.09/P1.11/P1.12 を使用している

### 2. nRF54L15 の I2S ノードは `i2s20`

nRF52 では `i2s0` だが、nRF54L15 では `i2s20`。

- `main.c`: `DT_NODELABEL(i2s20)`
- overlay: `&i2s20 { ... }`
- pinctrl: `i2s20_default`

### 3. pyocd のターゲット名は `nrf54l`

```bash
pyocd flash -t nrf54l <hex_file>
```

`nrf54l15` は認識されない。

### 4. ピンコントロールは全ピンを単一グループに

```dts
group1 {
    psels = <NRF_PSEL(I2S_SCK_M, 1, 4)>,
            <NRF_PSEL(I2S_LRCK_M, 1, 5)>,
            <NRF_PSEL(I2S_SDOUT, 1, 6)>;
};
```

3つに分けると `pinctrl_apply_state` でエラーになる場合がある。

### 5. TX バッファカウントは余裕を持たせる

```kconfig
CONFIG_I2S_NRFX_TX_BLOCK_COUNT=8
```

デフォルト値だと `slab alloc timeout` が発生しやすい。

### 6. サンプルレートは 48000 Hz 固定

nRF54L15 の HFXO から割り切れるサンプルレートは 48000 Hz が安定。
44100 Hz は若干の誤差が生じる可能性がある。

---

## デバッグ手順

### I2S が動作しているか確認する方法

起動直後にシリアルログで以下を確認:

1. `I2S configured OK` が出ているか → OK なら初期化成功
2. `slab alloc timeout: -11` が出続けているか → TX が詰まっている（ピン不一致）
3. ログが出ない / `I2S device not ready` → overlay の `i2s20` 設定が誤り

### TX が詰まっているときのチェックリスト

- [ ] ピンが P1 系か（P2 系では動作しない）
- [ ] SPK2 の BCLK/LRCLK/SDATA の接続順が正しいか
- [ ] GND が XIAO と SPK2 の間で共通になっているか
- [ ] 3V3 が SPK2 の pin7 (3V3) に接続されているか（5VIN ではない）
- [ ] overlay で `spi00` や他のペリフェラルが同じピンを使っていないか

### シリアルログの取り方（ロストなし）

```bash
# リセットと同時にシリアルを開く
pyocd reset -t nrf54l & python3 -c "
import serial, time
s = serial.Serial('$(ls /dev/tty.usbmodem* | head -1)', 115200, timeout=5)
time.sleep(0.1)
while True:
    l = s.readline()
    if not l: break
    print(l.decode(errors='replace'), end='')
"
```

---

## 設定ファイル詳細

### prj.conf

```kconfig
CONFIG_I2S=y
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_I2S_NRFX_TX_BLOCK_COUNT=8
```

### boards/xiao_nrf54l15_nrf54l15_cpuapp.overlay

```dts
/* I2S20 only works with Port 1 pins on nRF54L15 (hardware restriction).
 * Using D0/D1/D2 (P1.04/05/06) — rewire SPK2 from D8-D10 to D0-D2.
 */

&pinctrl {
    i2s20_default: i2s20_default {
        group1 {
            psels = <NRF_PSEL(I2S_SCK_M, 1, 4)>,
                    <NRF_PSEL(I2S_LRCK_M, 1, 5)>,
                    <NRF_PSEL(I2S_SDOUT, 1, 6)>;
        };
    };
};

&i2s20 {
    status = "okay";
    pinctrl-0 = <&i2s20_default>;
    pinctrl-names = "default";
};
```

---

## 動作確認済み環境

- NCS: v2.9.2
- ボード: `xiao_nrf54l15/nrf54l15/cpuapp`
- pyocd: `pip install pyocd` でインストール
- ホスト OS: macOS
- 確認日: 2026-03-20

---

## 旧デバッグ記録

旧バージョン（D8/D9/D10 使用・音が出なかった経緯）は
`docs/nrf54-speaker_debug_summary.md` を参照。
