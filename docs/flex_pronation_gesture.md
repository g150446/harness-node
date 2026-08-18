# 回内→上昇→静止ジェスチャー仕様

対象デバイス: Seeed Studio XIAO nRF52840 Sense  
対象ファームウェア: HarnessNode `0.0.42` 以降

## 発動条件

- XIAO をリストバンドの**手の甲側**に置く
- 部品面を皮膚側にし、基板 X 軸を前腕と直交、Y 軸を前腕に沿わせる
- 手のひらを上向きにし、50 ms 以上静止する（線形加速度 ≤ 3.0 m/s²）
  - 掌上は補正 Z 比 ≥ 0.80 のみ（開始時の |Y| 上限は設けない。車載の横 G を許容）
  - 掌上静止で基準 phi / az を一度だけ固定する。その後の静止中に基準を追随しない
- 静止成立後 2.5 秒以内に回内を検出する。armed 中は静止していても判定する
  - 重力ベクトルの XZ 回転（phi ≥ 8°）、または Z 方向の明確な変化
- 回内完了後に一度静止してよい。その後、重力に逆らって手を上げる（物理目安 **約 5 cm**）
- 上げた位置で **500 ms** 静止する（最終静止に角速度は使わない）

右手・左手および USB 向きの差は、回内の相対符号で吸収する。  
最終姿勢の仰角帯・甲上条件・回外は要求しない。

ゆっくり回内すると線形加速度が 3.0 m/s² 以下のまま Z が回る。
基準 phi を静止中に更新したり、Z が掌上閾値を下回っただけで arm を解除したりすると、
その回転が消える。基準は初回 arm で固定し、armed 中は静止していても phi と Z を見る。

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
3. 上向きの加速パルスを確認する。減速パルスがあれば早期に hold へ進む
4. 正インパルスと 150–1800 ms の時間形状を満たしたら上昇成立。減速パルスは必須ではない
5. 短い加速度 RMS 窓が静止を示した時点の重力方向を保持基準とし、その後の角度差が 10° 以内か確認する

変位の二重積分は使わない。約 5 cm はユーザー動作の目安であり、判定閾値ではない。
加速インパルスと姿勢安定・500 ms 静止を満たす必要がある。減速は任意。

## 状態遷移

```text
WAITING → OUTBOUND(回内)
        → HOLDING_FINAL(WAIT_ACCEL → WAIT_BRAKE → WAIT_HOLD)
WAIT_BRAKE で 1800 ms 経過し正インパルスがあれば WAIT_HOLD へ。不足なら WAIT_ACCEL 再試行
```

## しきい値

| 項目 | 値 |
|---|---:|
| 開始補正 Z | ≥ 0.80（初回 arm のみ。arm 後は掌上を再要求しない） |
| 開始 \|Y\| | **なし**（撤廃） |
| 開始静止 | 線形加速度 ≤ 3.0 m/s² を 50 ms |
| 参照 phi / az | 初回 arm で固定。静止中に追随しない |
| arm 窓 | 2500 ms |
| 回内開始（重力 phi） | 固定基準から ≥ 8° |
| 回内開始（Z） | `|Δz比| ≥ 0.25`、Z 符号反転（\|az\| ≥ 2 m/s²）、または Z ピーク間 ≥ 4.0 m/s² |
| 回内完了 | phi ≥ 20°、または `|Δz比| ≥ 0.50`、または Z 符号反転 |
| 回内中の未完了静止 | 800 ms で reset。完了後の静止は HOLDING_FINAL で許容 |
| 上向き加速度 | `a_up` ≥ 0.40 m/s² を2サンプル、正インパルス ≥ 0.04 m/s |
| 逆向き減速 | 任意。`a_up` ≤ -0.15 m/s² なら早期に hold へ |
| パルス形状 | 150–1800 ms。1800 ms 時点で正インパルスがあれば hold へ |
| 保持中の姿勢差 | 静止開始時の重力方向から ≤ 10° |
| 静止進入 | 4サンプルの線形加速度 RMS ≤ 2.0 m/s² |
| 最終静止 | 500 ms |
| 最終到達期限 | 回内後 3000 ms |
| 全シーケンス | 5000 ms |

## 診断イベント（抜粋）

| stage | 意味 |
|---|---|
| `outbound_start` / `outbound_ready` | 回内開始 / 回内完了。`outbound_ready` の v3 は Δz 比 |
| `wait_reject` reason `outbound_rate_low` | armed 中だが phi / Z が開始閾値未満。v1=phi、v2=Δz比、v3=Zピーク間 |
| `final_sample` (0x21) | 100 ms 周期: pulse_stage, a_up, net_impulse |
| `final_hold_start` / `final_ready` | 静止保持開始 / 完了 |
| `match` | 発動 |
| `wait_reject` reason `final_brake_missing` | 逆向き減速が不足。パルス再試行 |
| `wait_reject` reason `final_brake_ratio_low` | 減速/加速比不足。パルス再試行 |
| `wait_reject` reason `final_pulse_duration_invalid` | パルスが150 ms未満。パルス再試行 |
| `reset` reason `final_accel_missing` | 回内後 3000 ms 以内に上向き加速が不足 |
| `reset` reason `final_brake_missing` | 回内後 3000 ms 以内に減速パルス不成 |
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
更新後 version `0.0.42` が slot0 active+confirmed であること。
