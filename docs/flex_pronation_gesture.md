# 回内→上昇→静止ジェスチャー仕様

対象デバイス: Seeed Studio XIAO nRF52840 Sense  
対象ファームウェア: HarnessNode `0.0.41` 以降

## 発動条件

- XIAO をリストバンドの**手の甲側**に置く
- 部品面を皮膚側にし、基板 X 軸を前腕と直交、Y 軸を前腕に沿わせる
- 手のひらを上向きにし、120 ms 以上静止する（角速度 70°/s 以下）
  - 掌上は補正 Z 比 ≥ 0.90 のみ（開始時の |Y| 上限は設けない。車載の横 G を許容）
- 静止成立後 1 秒以内に、前腕軸まわり（`gyro_y`）で回内する
- 回内後、重力に逆らって手を上げる（物理目安 **約 5 cm 以上**）
- 上げた位置で **500 ms** 静止する（線形加速度 ≤ 3.0 m/s²。最終静止に角速度は使わない）

右手・左手および USB 向きの差は、回内の相対符号で吸収する。  
最終姿勢の仰角帯・甲上条件・回外は要求しない。

## 装着と軸

| 軸 | 基板上 | 役割 |
|---|---|---|
| `X` | 長手（USB から離れる） | 前腕を横切る |
| `Y` | 短い辺方向（5V/GND 側） | 前腕軸＝回内軸（`gyro_y`） |
| `Z` | 部品面外向き | 掌上開始判定 |

実測（甲側・部品面皮膚）: 掌上で raw `z` は**負**。  
このため `GESTURE_PALM_UP_Z_SIGN = -1`。

```text
palm_up_z_ratio = z_ratio * (-1)
```

- 掌上: `palm_up_z_ratio` が大きく正

## 上昇量の測り方（IMU 換算）

回内完了後:

1. 角速度が落ちるまで短い整定待ち（約 80 ms）
2. その時点の重力方向（低通した世界座標加速度）を基準として固定
3. 線形加速度を「重力の逆方向」へ投影し、動作中のみ二重積分
4. **ピーク上昇量（cm）** が閾値以上なら上昇成立

しきい値 `GESTURE_FINAL_LIFT_MIN_CM = 2.5` は **IMU 積分 cm**。  
短時間積分のため物理変位より小さめに出る。実機では物理約 5 cm 以上の明確な上昇で、ピークが 2.5 cm を超えることを確認済み（例: 29 cm ピークで PASS）。

## 状態遷移

```text
WAITING → OUTBOUND(回内) → HOLDING_FINAL(上昇積分 → 静止保持)
```

## しきい値

| 項目 | 値 |
|---|---:|
| 開始補正 Z | ≥ 0.90 |
| 開始 \|Y\| | **なし**（撤廃） |
| 開始静止 | ≤ 70°/s を 120 ms |
| 回内開始 \|gyro_y\| | ≥ 25°/s |
| 回内積分 / ピーク | ≥ 15° / ≥ 30°/s |
| 上昇アーム整定 | gyro ≤ 80°/s を 80 ms |
| 上昇 IMU 換算下限 | ≥ 2.5 cm（ピーク） |
| 最終静止 | 線形加速度 ≤ 3.0 m/s² を 500 ms |
| 最終到達期限 | 回内後 3000 ms |
| 全シーケンス | 5000 ms |

## 診断イベント（抜粋）

| stage | 意味 |
|---|---|
| `outbound_start` / `outbound_ready` | 回内開始 / 回内完了 |
| `final_sample` (0x21) | 上昇中 100 ms 周期: peak_cm, a_up, gyro |
| `final_hold_start` / `final_ready` | 静止保持開始 / 完了 |
| `match` | 発動 |
| `reset` reason `final_lift_too_short` | 上昇不足でタイムアウト |

## 検証

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 1 \
  --json-output /tmp/gesture-lift.json
```

表示条件: 掌上開始、開始静止、回内、重力に逆らう上昇、0.5 秒静止、単一動作、発動。

## ビルド・OTA

```bash
# 必要に応じて NCS / west を PATH に
export NCS_BASE=/opt/nordic/ncs/v2.9.2
export PATH="/opt/nordic/ncs/toolchains/<id>/bin:$PATH"

nordic-main/build_and_package_ota.sh
cd mac_client && venv/bin/python ota_updater.py --device HarnessNode \
  ../nordic-main/ota_update.bin
```

`prj.conf` の `CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION` と `VERSION` を揃えてからビルドすること。  
更新後 version `0.0.41` が slot0 active+confirmed であること。
