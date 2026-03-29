# HarnessNode (`nordic-main`) バッテリー残量実装ガイド

XIAO nRF52840 Sense 向け `nordic-main` ファームウェア（BLE デバイス名: `HarnessNode`）のバッテリー残量 BLE 通知機能の実装詳細と、実機診断で判明した回路知見をまとめます。

---

## 機能概要

| 項目 | 内容 |
|------|------|
| BLE サービス | Battery Service（UUID `0x180F`、Bluetooth SIG 標準） |
| キャラクタリスティック | Battery Level（UUID `0x2A19`）、Read / Notify |
| 更新間隔 | 30 秒ごと（`BATTERY_POLL_INTERVAL_MS = 30000`） |
| ADC ピン | P0.31（AIN7） |
| 分圧比 | 1/3（2MΩ : 1MΩ、補正係数 ×3） |
| Zephyr Kconfig | `CONFIG_ADC=y`、`CONFIG_BT_BAS=y` |

---

## XIAO nRF52840 Sense のバッテリー回路

```
LiPo Battery (+) ── VBAT
                       │
                    [2MΩ相当]   ← R_top（複合抵抗）
                       │
                    P0.31 (AIN7)  ← ADC 入力
                       │
                    [1MΩ相当]   ← R_bot
                       │
                     GND

P0.14 ── PMOS ゲート制御（LOW=ON、HIGH=OFF）
```

### 重要な回路知見（実機診断で確認）

ネット上のドキュメントや一部のサンプルコードには誤りが含まれているため注意が必要です。

#### 誤り 1: ADC ピンが P0.14（AIN2）と記載されている

P0.14 は nRF52840 の SAADC 非対応ピンです（AIN2 = P0.04）。
**実際の ADC 入力は P0.31（AIN7）** です。

#### 誤り 2: P0.14 を HIGH にすると測定が有効になる

PMOS 回路のため、**P0.14=LOW で PMOS ON（VBAT が分圧回路に接続）** となります。
P0.14=HIGH にすると VBAT が切断され、P0.31 が VCC 方向にフロートして ADC が飽和します。

実機診断データ（USB 接続中、VBAT ≈ 4200mV）:

```
P0.14=HIGH: raw=4042  pin_mv=3552  → ×3 = 10656mV  ← 飽和・誤値
P0.14=LOW:  raw=1591  pin_mv=1398  → ×3 = 4194mV   ← 正しい
```

**P0.14 は常に LOW（GPIO_OUTPUT_INACTIVE）のまま使用します。**

#### 誤り 3: 補正係数が ×2（分圧比 1/2）

実測で確認した分圧比:
- USB 接続中（VBAT = 4200mV）の pin_mv = 1398mV
- 4200 / 1398 ≈ **3.0** → 分圧比は 1/3（R_top ≈ 2MΩ、R_bot ≈ 1MΩ）

**補正係数は ×3 です（×2 ではない）。**

---

## デバイスツリー設定

`boards/xiao_ble_nrf52840_sense.overlay`:

```dts
/ {
    zephyr,user {
        /* P0.31 = AIN7 — actual battery voltage ADC input (1/3 divider) */
        io-channels = <&adc 7>;
        /* P0.14 = PMOS gate: LOW=ON(measure), HIGH=OFF. Keep LOW at all times. */
        bat-read-enable-gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;
    };
};

&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@7 {
        reg = <7>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";  /* 0.6V → full scale 3.6V */
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN7>; /* P0.31 */
        zephyr,resolution = <12>;
        zephyr,oversampling = <4>;
    };
};
```

ADC 設定:
- `ADC_GAIN_1_6` + `ADC_REF_INTERNAL(0.6V)` → 入力フルスケール = **3.6V**
- P0.31 の最大電圧 = 4.2V / 3 = 1.4V（3.6V より十分低く、飽和しない）

---

## prj.conf

```conf
CONFIG_ADC=y
CONFIG_BT_BAS=y
```

---

## 実装コード要点

`src/main.c` のバッテリー読み取り関数:

```c
static int battery_get_millivolt(void)
{
    int err;
    int32_t val_mv;

    /* P0.14 must remain LOW — HIGH disconnects VBAT (PMOS OFF) and saturates ADC */

    err = adc_sequence_init_dt(&adc_bat, &bat_sequence);
    if (err < 0) { return err; }

    err = adc_read(adc_bat.dev, &bat_sequence);
    if (err < 0) { return err; }

    val_mv = bat_sample_buf;
    err = adc_raw_to_millivolts_dt(&adc_bat, &val_mv);
    if (err < 0) { return err; }

    /*
     * Correction factor ×3: divider is 2MΩ:1MΩ (not 1:1).
     * Verified: pin_mv=1398mV × 3 = 4194mV ≈ 4200mV (USB charge, VBAT=4.2V).
     */
    return (int)(val_mv * 3);
}
```

### LiPo 放電カーブ近似 LUT

```c
{ 4200, 100 }, { 4100, 90 }, { 4000, 80 },
{ 3900,  70 }, { 3800, 60 }, { 3700, 50 },
{ 3600,  40 }, { 3500, 30 }, { 3400, 20 },
{ 3300,  10 }, { 3000,  0 },
```

隣接エントリ間は線形補間。

### BAS への反映

```c
bt_bas_set_battery_level(pct);  /* 0〜100 の uint8_t */
```

---

## 診断スクリプト

`mac_client/battery_check.py` で BLE 経由のバッテリー残量を確認できます:

```bash
cd mac_client
source venv/bin/activate
python3 battery_check.py
```

出力例:

```
Scanning for 'HarnessNode'...
Found: C57DBDCF-8ACD-BBA7-5675-A3843B609520
Connected. MTU=244

=== Battery Level ===
  Battery Level: 87%  (raw byte: 0x57)
```

---

## 動作確認手順

1. `battery_check.py` または nRF Connect アプリで Battery Service (0x180F) を読み取る
2. シリアルモニターで `Battery: XXXX mV (XX%)` ログを確認する（30 秒ごとに出力）
3. USB を外してバッテリー駆動に切り替え、しばらく後に残量が変化することを確認する

---

## 関連ファイル

| ファイル | 役割 |
|---------|------|
| `nordic-main/src/main.c` | バッテリー読み取り実装（`battery_init`, `battery_get_millivolt`, `battery_millivolt_to_percent`, `battery_update`） |
| `nordic-main/boards/xiao_ble_nrf52840_sense.overlay` | ADC チャンネル（AIN7/P0.31）と P0.14 GPIO 定義 |
| `nordic-main/prj.conf` | `CONFIG_ADC=y`, `CONFIG_BT_BAS=y` |
| `mac_client/battery_check.py` | BLE 経由の診断スクリプト |
