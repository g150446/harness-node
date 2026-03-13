# nRF54L15 OTA (BLE SMP) 実装状況メモ

作成日：2026-03-12

---

## 概要

XIAO nRF54L15 Sense ボードに MCUboot + MCUmgr/SMP over BLE によるファームウェア OTA 更新機能を実装中。
NCS v2.9.2、sysbuild 構成。

---

## 動作確認済み

| 項目 | 状況 |
|---|---|
| MCUboot ビルド・フラッシュ | ✅ 動作 (merged.hex = MCUboot 62KB + 署名済みアプリ) |
| MCUboot からアプリ起動 | ✅ 動作 |
| BLE アドバタイズ (MotionBridge) | ✅ 起動後即時 |
| Build Info characteristic (日時読み取り) | ✅ `Mar 12 2026 21:27:40` など |
| SMP characteristic (OTA用) | ✅ UUID: `da2e7828-...` / MTU=244 |
| OTA ファームウェアアップロード | ✅ 206KB を BLE 経由 約90秒で転送 |
| Image state クエリ (slot 1 確認) | ✅ slot 1 に有効なイメージを確認 |

---

## 解決した問題

### `boot_set_next` が失敗していた問題 (解決済み)

アップロード完了後にイメージを「次回テストブート対象」に設定する SMP コマンド (`IMG_STATE` write) が `rc=1` で失敗していたが、**アプリ側の RRAM 設定を MCUboot 側と揃えることで解決した**。

**根本原因:**
- `nrf54-motion/prj.conf` のアプリ側では `CONFIG_NRF_RRAM_READYNEXT_TIMEOUT_VALUE` がデフォルトの `128` になっていた
- `img_mgmt_state_write()` → `boot_set_next()` → `boot_write_magic()` により secondary slot trailer へ書き込む際、nRF54L15 の RRAMC まわりでこの設定が悪影響を出していた
- MCUboot 側ではすでに `CONFIG_NRF_RRAM_READYNEXT_TIMEOUT_VALUE=0` を入れていたが、**実際に trailer write を行うのは再起動前のアプリ側**なので、アプリ側にも同じ回避設定が必要だった

**修正内容:**
- `nrf54-motion/prj.conf` に以下を追加

```kconfig
CONFIG_NRF_RRAM_READYNEXT_TIMEOUT_VALUE=0
```

**結果:**
- `IMG_STATE` write が成功
- `Image test flag set.` まで進行
- reset 後に MCUboot が swap を実行
- 再起動後の Build Info characteristic が OTA イメージのビルド時刻に更新されることを確認

**エラーの流れ：**
```
SMP IMG_STATE write {"hash": <slot1_hash>, "confirm": false}
→ img_mgmt_state_write()
→ img_mgmt_set_next_boot_slot(slot=1, confirm=false)
→ img_mgmt_set_next_boot_slot_common(slot=1, active_slot=0, confirm=false)
→ boot_set_next(fa=secondary_slot, active=false, confirm=false)
→ boot_read_swap_state() → BOOT_MAGIC_UNSET (正常)
→ boot_write_magic(fa) → flash_area_write() → 失敗?
→ BOOT_EFLASH → IMG_MGMT_ERR_FLASH_WRITE_FAILED → MGMT_ERR_EUNKNOWN=1
```

**OTA アップロード自体は成功しているため flash driver は動作している**が、
MCUboot トレーラー領域 (アドレス `0x102FF0` = secondary slot 末尾 16B) への書き込みが失敗する疑い。

**調査済みの内容：**
- スロット1トレーラー (`0x102FF0`) は `0xFFFFFFFF` (= UNSET 状態) ✓
- RRAM の書き込みアライメント (16B) も問題なし ✓
- `flash_area_open(area_id=7, ...)` は state read では成功している ✓
- RRAM アドレス範囲 (0xB2000〜0x103000) は有効 ✓

**確認できたこと：**
- OTA upload 自体は引き続き成功
- 問題は transport ではなく pending/test 設定の最終ステップに限定されていた
- `swap/test + rollback` 方針のまま修正できたため、`overwrite-only` への変更は不要だった

---

## パーティションレイアウト (`pm_static.yml`)

```
0x000000  mcuboot            62 KB   (fprotect: 2×31KB RRAMC combined)
0x00F800  mcuboot_pad         2 KB   ─┐ mcuboot_primary (324 KB)
0x010000  app              322 KB   ─┘
0x060800  EMPTY_gap           2 KB   (アライン用)
0x061000  EMPTY_0 (slot0_ns) 324 KB  (未使用 TrustZone NS 領域)
0x0B2000  mcuboot_secondary  324 KB  ← OTA スロット
0x103000  EMPTY_1            356 KB  (slot1_ns + 予備)
0x15C000  settings_storage    36 KB
0x165000  end
```

---

## MCUboot ビルドで必要だったワークアラウンド

`sysbuild/mcuboot/boards/xiao_nrf54l15_nrf54l15_cpuapp.conf` に追加：

```kconfig
CONFIG_FPROTECT=n                        # nRF54L15 では未サポート (公式DK設定に準拠)
CONFIG_FLASH=y
CONFIG_SOC_FLASH_NRF_RRAM=y
CONFIG_NRF_RRAM_READYNEXT_TIMEOUT_VALUE=0  # RRAMC レジスタ書き込みがフォルト回避
CONFIG_NRF_GRTC_TIMER=n                  # onoff/clock クラッシュ回避
CONFIG_NRF_GRTC_START_SYSCOUNTER=n
CONFIG_SYS_CLOCK_EXISTS=n               # SysTick がフォルト回避 (最重要)
CONFIG_SPI_NOR=n
CONFIG_BOOT_WATCHDOG_FEED=n
CONFIG_ARM_MPU=n
CONFIG_HW_STACK_PROTECTION=n
CONFIG_BUILTIN_STACK_GUARD=n
CONFIG_BOOT_MAX_IMG_SECTORS=256
CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=0
```

**`CONFIG_SYS_CLOCK_EXISTS=n` が最も重要。**
理由: `sys_clock_driver_init` が SysTick を有効化するが、SysTick ベクターが汎用フォルトハンドラ (`z_arm_bus_fault`) を指しているため、SysTick 割り込みが発生するとフォルトループに入る。

### MCUboot でクラッシュしていた問題一覧（解決済み）

| 問題 | 症状 | 解決策 |
|---|---|---|
| `fprotect_area()` 失敗 | RRAMC 保護領域が 64KB > RRAMC 最大 62KB | MCUboot パーティションを 62KB に縮小 |
| onoff/clock クラッシュ | GRTC タイマー初期化でフォルト | `CONFIG_NRF_GRTC_TIMER=n` |
| malloc アリーナクラッシュ | 175KB 動的アリーナ確保で失敗 | `CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=0` |
| regulator_fixed_init クラッシュ | XIAO ボードの固定レギュレータ初期化失敗 | DTS overlay で disabled に |
| RRAMC READYNEXTTIMEOUT クラッシュ | `0x5004B50C` 書き込み中に SysTick フォルト | `CONFIG_NRF_RRAM_READYNEXT_TIMEOUT_VALUE=0` |
| SysTick フォルト (メイン) | `sys_clock_driver_init` が SysTick 有効化 → フォルトハンドラ呼び出し | `CONFIG_SYS_CLOCK_EXISTS=n` |

---

## ファイル構成

```
nrf54-motion/
├── prj.conf                   # MCUmgr/SMP, MTU拡張, SETTINGS等
├── sysbuild.conf              # SB_CONFIG_BOOTLOADER_MCUBOOT=y
├── pm_static.yml              # パーティションレイアウト
├── build_and_flash.sh         # USB導入用: build + pyocd flash + OTA payload更新
├── build_and_package_ota.sh   # OTA継続更新用: build + OTA payload更新
├── flash.sh                   # 互換ラッパー (build_and_flash.shへ委譲)
├── sysbuild/
│   └── mcuboot/
│       ├── prj.conf           # (空)
│       └── boards/
│           ├── xiao_nrf54l15_nrf54l15_cpuapp.conf   # MCUbootワークアラウンド
│           └── xiao_nrf54l15_nrf54l15_cpuapp.overlay # レギュレータ無効化
└── src/
    └── main.c                 # Build Info characteristic追加済み

mac_client/
└── ota_updater.py             # BLE SMP OTAスクリプト (cbor2, bleak使用)
```

---

## BLE UUIDs

| 用途 | UUID |
|---|---|
| Motion Service | `00000010-0000-1000-8000-00805f9b34fb` |
| Motion Notify (TX) | `00000011-0000-1000-8000-00805f9b34fb` |
| **Build Info (Read)** | `00000012-0000-1000-8000-00805f9b34fb` |
| SMP Service | `8D53DC1D-1DB7-4CD3-868B-8A527460AA84` |
| SMP Characteristic | `DA2E7828-FBCE-4E01-AE9E-261174997C48` |

---

## OTA スクリプトの使い方

### すでに OTA 対応ファームが入っている場合

```bash
cd nrf54-motion
./build_and_package_ota.sh

cd ../mac_client
venv/bin/pip install cbor2
venv/bin/python3 ota_updater.py ../nrf54-motion/ota_update.bin
```

### まだ OTA 対応ファームが入っていない場合

```bash
cd nrf54-motion
./build_and_flash.sh
```

その後、以降の更新は上の OTA 手順に切り替える。

### OTA 実行コマンド

```bash
cd mac_client
venv/bin/pip install cbor2
venv/bin/python3 ota_updater.py ../nrf54-motion/ota_update.bin
```

ステップ:
1. BLE スキャン → MotionBridge に接続
2. MTU ネゴシエーション (244B)
3. SMP image upload (first chunk: 163B, rest: 213B)
4. Image state クエリ (slot 1 ハッシュ取得)
5. Image test flag 設定
6. OS reset → MCUboot が swap 実行

---

## 実機検証ログ (2026-03-13)

1. `nrf54-motion` を sysbuild + MCUboot でビルド
2. 生成した `merged.hex` を USB 接続した XIAO nRF54L15 Sense に書き込み
3. 実機の Build Info characteristic を読み取り、基準ビルドが `Mar 13 2026 08:36:20` であることを確認
4. 同じソースを再ビルドし、OTA 用 signed image の Build Info が `Mar 13 2026 08:37:57` であることを確認
5. `venv/bin/pip install cbor2`
6. `mac_client/ota_updater.py` を実行して OTA 実施

実行結果:
- `Upload complete in 88.3s`
- `Image test flag set.`
- `Device reset. MCUboot will swap slots on next boot.`
- 再起動後の Build Info characteristic が `Mar 13 2026 08:37:57` になり、OTA イメージへ切り替わったことを確認

## 今後の改善候補

1. `ota_updater.py` に reset 後の再接続と Build Info 再読取を組み込み、更新完了確認まで自動化する
2. 追加で active image の confirm / rollback シナリオも自動検証できるようにする
