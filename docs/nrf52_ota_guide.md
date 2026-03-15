# nRF52840 BLE OTA ガイド (`nrf52-motion`)

## 概要

`nrf52-motion/` は Seeed XIAO nRF52840 Sense 向けのモーション検出アプリで、BLE OTA ファームウェア更新に対応しています。

| 項目 | 内容 |
|------|------|
| ターゲットボード | `xiao_ble/nrf52840/sense` |
| BLE デバイス名 | `MotionBridge` |
| ブートローダ構成 | Adafruit UF2 (0xF4000) + MCUboot (0x27000) |
| OTA トランスポート | BLE SMP (MCUmgr) |
| OTA クライアント | `mac_client/ota_updater.py` |
| 転送速度 | 約 2 KB/s（217 KB イメージで約 104 秒） |

---

## フラッシュレイアウト

```
0x000000  softdevice/Adafruit MBR  156 KB  (Adafruit 管理、上書き不可)
0x027000  MCUboot                   48 KB  ← Adafruit がここにジャンプ
0x033000  mcuboot_pad                0.5 KB
0x033200  app primary slot         最大 327 KB
0x085000  mcuboot_secondary         328 KB  ← OTA 書き込み先
0x0D7000  settings_storage           84 KB  (イメージ確認状態を保存)
0x0F4000  Adafruit UF2 bootloader    48 KB  (上書き不可)
```

---

## 手順

### 1. 初回フラッシュ（UF2 経由）

OTA 対応ファームウェアを初めて書き込む場合は UF2 を使います。

```bash
cd nrf52-motion
./build_and_flash.sh
```

スクリプトが自動で:
1. sysbuild（MCUboot + アプリ）をビルド
2. XIAO-SENSE ドライブ（UF2 ブートローダ）を待機
3. merged.uf2 を書き込み

**デバイスを UF2 ブートローダモードにする方法:** リセットボタンをダブルタップ（0.5 秒以内に 2 回押す）。緑 LED が点滅し XIAO-SENSE ドライブが現れます。

### 2. OTA バイナリのビルド

```bash
cd nrf52-motion
./build_and_package_ota.sh
```

`ota_update.bin` が生成されます（`nrf52-motion/zephyr.signed.bin` のコピー）。

**重要:** OTA バイナリのバージョンは稼働中のファームウェアと異なる必要があります。`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` を更新してからビルドしてください：

```
CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="0.0.2+0"  # 毎回インクリメント
```

### 3. BLE OTA 実行

```bash
cd mac_client
../venv/bin/python3 ota_updater.py ../nrf52-motion/ota_update.bin
```

### 4. 更新確認

デバイスの USB シリアル（115200 baud）でビルド時刻が新しくなっていることを確認：

```
[nrf52_motion] XIAO nRF52840 Sense motion detection (build: Mar 15 2026 21:10:37)
```

---

## トラブルシューティング

### `Device 'MotionBridge' not found`

- デバイスがキャリブレーション完了後に BLE アドバタイズを開始するため、起動から約 3 秒待ってから実行
- シリアルモニタで `BLE advertising started` が出ているか確認

### `Image test set failed: rc=1`

OTA バイナリのハッシュが稼働中のファームウェアと同一の場合に発生します。`prj.conf` でバージョンを上げてリビルドしてください。

```
CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION="0.0.2+0"
```

### `No image in slot 1 found`

DTS partition ID と PM partition ID のミスマッチが原因の可能性があります。`docs/nrf52_porting_lessons.md` の「DTS/PM partition ID マッピング」を参照してください。

### デバイスが UF2 ブートローダに入らない

- Zephyr USB CDC ACM ファームウェアでは 1200 baud トリガーは機能しません
- 物理的なリセットボタンのダブルタップのみ有効です
- ダブルタップのタイミング: 0.5 秒以内に 2 回

---

## 関連ファイル

```
nrf52-motion/
├── prj.conf                    # アプリ設定（MCUmgr, BLE, version）
├── sysbuild.conf               # SB_CONFIG_BOOTLOADER_MCUBOOT=y
├── pm_static.yml               # フラッシュパーティション定義
├── CMakeLists.txt              # PM_STATIC_YML_FILE を設定
├── boards/
│   └── xiao_ble_nrf52840_sense.overlay  # DTS パーティション ID 調整
├── sysbuild/mcuboot/
│   ├── prj.conf                # MCUboot 設定
│   └── boards/
│       ├── xiao_ble_nrf52840_sense.conf     # MCUboot ボード設定
│       └── xiao_ble_nrf52840_sense.overlay  # MCUboot DTS（slot0/slot1 定義）
├── src/main.c
├── build_and_flash.sh          # 初回 UF2 フラッシュスクリプト
└── build_and_package_ota.sh    # OTA バイナリ生成スクリプト
```
