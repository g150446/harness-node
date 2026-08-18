# 回内→上昇→静止ジェスチャー仕様

対象デバイス: Seeed Studio XIAO nRF52840 Sense  
対象ファームウェア: HarnessNode `0.0.41` 以降

## 発動条件

- XIAO をリストバンドの**手の甲側**に置く
- 部品面を皮膚側にし、基板 X 軸を前腕と直交、Y 軸を前腕に沿わせる
- 手のひらを上向きにし、50 ms 以上静止する（線形加速度 ≤ 3.0 m/s²）
  - 掌上は補正 Z 比 ≥ 0.85 のみ（開始時の |Y| 上限は設けない。車載の横 G を許容）
- 静止成立後 1 秒以内に、重力ベクトルの XZ 回転（回内）を検出する
- 回内後、重力に逆らって手を上げる（物理目安 **約 5 cm**）
- 上げた位置で **500 ms** 静止する（最終静止に角速度は使わない）

右手・左手および USB 向きの差は、回内の相対符号で吸収する。  
最終姿勢の仰角帯・甲上条件・回外は要求しない。

## 装着と軸

| 軸 | 基板上 | 役割 |
|---|---|---|
| `X` | 長手（USB から離れる） | 前腕を横切る |
| `Y` | 短い辺方向（5V/GND 側） | 前腕軸＝回内・回外軸 |
| `Z` | 部品面外向き | 掌上開始判定 |

実機検証（2026-08-18）では装着ごとに raw `z` の符号が反転したため、
開始姿勢はZの絶対比で判定する。

```text
palm_up_z_ratio = abs(z_ratio)
```

- 掌上開始目安: 基板面がほぼ水平で `abs(z_ratio)` が大きい

## 上昇動作の測り方（双極パルス）

回内完了後:

1. 低通加速度を重力推定とし、rawとの差から線形加速度を得る
2. 線形加速度をその時点の低通重力方向へ投影し、上向き加速度 `a_up` を得る
3. 上向きの加速パルスに続き、停止時の逆向き減速パルスが来ることを確認する
4. 正負インパルス、減速/加速比、150–900 ms の時間形状を満たしたら上昇成立
5. 短い加速度 RMS 窓が静止を示した時点の重力方向を保持基準とし、その後の角度差が 10° 以内か確認する

変位の二重積分は使わない。約 5 cm はユーザー動作の目安であり、判定閾値ではない。
単なる小振動を通しにくくするため、加速・減速・姿勢安定・500 ms 静止を
すべて満たす必要がある。

## 状態遷移

```text
WAITING → OUTBOUND(回内)
        → HOLDING_FINAL(WAIT_ACCEL → WAIT_BRAKE → WAIT_HOLD)
```

## しきい値

| 項目 | 値 |
|---|---:|
| 開始補正 Z | ≥ 0.85 |
| 開始 \|Y\| | **なし**（撤廃） |
| 開始静止 | 線形加速度 ≤ 3.0 m/s² を 50 ms |
| 回内開始（重力 phi） | arm時の実測phiから ≥ 10° |
| 回内完了（重力 phi） | arm時の実測phiから ≥ 20° |
| 上向き加速度 | `a_up` ≥ 0.80 m/s² を2サンプル、正インパルス ≥ 0.08 m/s |
| 逆向き減速 | `a_up` ≤ -0.30 m/s² を2サンプル、負インパルス ≥ 0.03 m/s |
| パルス形状 | 150–900 ms、減速/加速インパルス比 ≥ 0.08 |
| 保持中の姿勢差 | 静止開始時の重力方向から ≤ 10° |
| 静止進入 | 4サンプルの線形加速度 RMS ≤ 0.50 m/s² |
| 最終静止 | 500 ms |
| 最終到達期限 | 回内後 3000 ms |
| 全シーケンス | 5000 ms |

## 診断イベント（抜粋）

| stage | 意味 |
|---|---|
| `outbound_start` / `outbound_ready` | 回内開始 / 回内完了 |
| `final_sample` (0x21) | 100 ms 周期: pulse_stage, a_up, net_impulse |
| `final_hold_start` / `final_ready` | 静止保持開始 / 完了 |
| `match` | 発動 |
| `reset` reason `final_accel_missing` | 上向き加速が不足 |
| `reset` reason `final_brake_missing` | 逆向き減速が不足 |
| `reset` reason `final_brake_ratio_low` | 減速/加速インパルス比が不足 |
| `reset` reason `final_pulse_duration_invalid` | パルスが150 ms未満 |
| `reset` reason `final_tilt_unstable` | 上昇後の姿勢差が過大 |

## 検証

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 1 \
  --json-output /tmp/gesture-lift.json
```

表示条件: 掌上開始、開始静止、回内、加速→減速パルス、姿勢安定、
0.5 秒静止、単一動作、発動。

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
