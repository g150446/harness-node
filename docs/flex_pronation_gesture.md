# シェイク→掌上→挙上→掌下静止ジェスチャー仕様

対象デバイス: Seeed Studio XIAO nRF52840 Sense  
対象ファームウェア: HarnessNode `0.0.47` 以降

## 発動条件

- XIAO をリストバンドの**手の甲側**に置く
- 部品面を皮膚側にし、基板 X 軸を前腕と直交、Y 軸を前腕に沿わせる
- 手のひらを下向きにし、基板がほぼ水平（|Z 比| ≥ 0.80）のまま短く **1 回シェイク**する
  - シェイクは重力に直交する線形加速度だけを見る
  - 500 ms 窓の峰間 ≥ 5.0 m/s²、かつ `|平均| < 0.4 × 峰間`
  - 持続する同じ符号の横 G（運転）では平均 ≈ ピークとなり不成立
- シェイク後 2.5 秒以内に掌を上へ反転させる（掌上）
  - 基準はシェイク窓の最初のサンプル時点の重力 LP（振った瞬間の raw ではない）
  - 相対変化: 重力ベクトルの 3D 角 ≥ 15°、または phi ≥ 8°、または `|Δz比| ≥ 0.25`、または Z 符号反転
  - 口元への掌上は前腕ピッチになりやすく、XZ の phi だけでは拾えない
  - 装着ごとの raw Z 符号に依存する絶対掌上は使わない
  - 掌上成立時に基準 phi / az を取り直す
- 掌向きを問わず、口元へ上げる（物理目安 **約 5 cm**）
- 上げたあと、**掌上基準から手首を回して掌下** にし、**500 ms** 静止する
  - 静止の掌下はシェイク時の重力半球ではなく、掌上姿勢からの反転で見る
- 録音中に手のひらを上へ向けると録音終了する。手を下ろすだけでは終了しない

右手・左手および USB 向きの差は、シェイク後の相対符号で吸収する。  
最終姿勢の仰角帯・甲上条件・回外は要求しない。

運転の横 G は重力 LP（tau 0.30 s）で遅い車体加速度を姿勢から除き、
シェイク判定では直交成分の直流（平均 ≈ ピーク）を落とす。

## 装着と軸

| 軸 | 基板上 | 役割 |
|---|---|---|
| `X` | 長手（USB から離れる） | 前腕を横切る |
| `Y` | 短い辺方向（5V/GND 側） | 前腕軸＝回内・回外軸 |
| `Z` | 部品面外向き | 基板水平（|Z 比|）の判定 |

実機検証では装着ごとに raw `z` の符号が反転したため、
水平姿勢は Z の絶対比、掌上はシェイク前 LP からの 3D / phi / Z 変化、
掌下 hold は掌上基準からの相対 phi / Z 変化で判定する。

```text
board_flat = abs(z / |a|) >= 0.80
palm_up_after_shake = 3D tilt >= 15 deg or phi >= 8 deg
  or |dz_ratio| >= 0.25 or Z sign flip
  (reference = gravity LP at first shake-window sample)
palm_down_after_lift = flip from palm-up reference
  (phi >= 20 deg or |dz_ratio| >= 0.50 or Z sign flip)
```

## 上昇動作の測り方（双極パルス）

掌上成立後:

1. 低通加速度を重力推定とし、rawとの差から線形加速度を得る
2. 線形加速度をその時点の低通重力方向へ投影し、上向き加速度 `a_up` を得る
3. 上向きの加速パルスを確認する。減速パルスがあれば早期に hold へ進む
4. 正インパルスと 150–1800 ms の時間形状を満たしたら上昇成立。減速パルスは必須ではない。掌向きは見ない
5. 掌上基準から手首が反転してから、短い加速度 RMS 窓が静止を示した時点の重力方向を保持基準とし、その後の角度差が 10° 以内か確認する

変位の二重積分は使わない。約 5 cm はユーザー動作の目安であり、判定閾値ではない。
加速インパルスと、掌上基準からの反転後の姿勢安定・500 ms 静止を満たす必要がある。減速は任意。

## 録音停止（開始姿勢からの掌上）

録音開始時の重力 LP（掌下静止）を基準として保存する。`reset_gesture_sequence()` では消さない。録音中も重力LPは継続する。

開始後 1200 ms を過ぎたあと、シェイク後掌上と同じ緩いゲートで反転したら即座に停止する（hold は待たない）:

1. 重力ベクトルの 3D 角 ≥ 15°、または phi ≥ 8°、または `|Δz比| ≥ 0.25`、または Z 符号反転
2. `|a|` が 7.5–12.5 m/s²

手を重力方向へ下ろすだけでは停止しない。`motion_active` でも止めない。ホスト `0x00` とシリアル `'s'` は従来どおり。

## 状態遷移

```text
WAITING → OUTBOUND(掌上)
        → HOLDING_FINAL(WAIT_ACCEL → WAIT_BRAKE → WAIT_HOLD)
WAIT_HOLD は掌上基準からの反転が取れるまで開始しない
WAIT_BRAKE で 1800 ms 経過し正インパルスがあれば WAIT_HOLD へ。不足なら WAIT_ACCEL 再試行
```

## しきい値

| 項目 | 値 |
|---|---:|
| 開始水平 | \|Z 比\| ≥ 0.80 |
| シェイク窓 | 20 サンプル（約 500 ms） |
| シェイク峰間 | 重力直交成分 ≥ 5.0 m/s² |
| シェイク平均比 | \|平均\| < 0.4 × 峰間 |
| シェイク軸下限 | 直交成分 ≥ 2.0 m/s² で軸を固定 |
| 掌上 | シェイク窓開始時の重力 LP から 3D ≥ 15°、phi ≥ 8°、\|Δz比\| ≥ 0.25、または Z 符号反転 |
| 掌上窓 | シェイク後 2500 ms |
| 上向き加速度 | `a_up` ≥ 0.40 m/s² を2サンプル、正インパルス ≥ 0.04 m/s |
| 逆向き減速 | 任意。`a_up` ≤ -0.15 m/s² なら早期に hold へ |
| パルス形状 | 150–1800 ms。1800 ms 時点で正インパルスがあれば hold へ |
| hold の掌 | 掌上基準からの反転（phi ≥ 20° / \|Δz比\| ≥ 0.50 / Z 符号）。挙上中の掌向きは見ない |
| 保持中の姿勢差 | 静止開始時の重力方向から ≤ 10° |
| 静止進入 | 4サンプルの線形加速度 RMS ≤ 2.0 m/s² |
| 最終静止 | 500 ms |
| 最終到達期限 | 掌上後 5000 ms |
| 全シーケンス | 9000 ms |
| 録音停止 | 開始姿勢から 3D ≥ 15°、phi ≥ 8°、\|Δz比\| ≥ 0.25、または Z 符号反転。\|a\| 7.5–12.5 m/s² |
| 録音停止 開始抑制 | 1200 ms |

## 診断イベント（抜粋）

| stage | 意味 |
|---|---|
| `outbound_start` | シェイク成立。v1=峰間、v2=Z絶対比、v3=平均 |
| `outbound_ready` | 掌上反転。v1=phi、v2=3D角、v3=Δz 比 |
| `wait_reject` reason `start_not_palm_up` | 基板が水平でない。v1=Z比、v2=下限、v3=線形加速度 |
| `wait_reject` reason `quiet_not_ready` | シェイク窓未充足。v1=Z比、v2=件数、v3=必要件数 |
| `wait_reject` reason `shake_not_oscillatory` | 直流横Gなど。v1=峰間、v2=平均、v3=\|平均\|/峰間 |
| `reset` reason `lift_palm_still_up` | 挙上後に掌上基準から反転できず期限切れ |
| `final_sample` (0x21) | 100 ms 周期: pulse_stage, a_up, net_impulse |
| `final_hold_start` / `final_ready` | 掌下反転後の静止保持開始 / 完了 |
| `match` | 発動 |
| `stop_palm_up` (0x0C) | 録音中の掌上停止。v1=phi、v2=3D角、v3=Δz |
| `wait_reject` reason `final_brake_missing` | 逆向き減速が不足。パルス再試行 |
| `wait_reject` reason `final_brake_ratio_low` | 減速/加速比不足。パルス再試行 |
| `wait_reject` reason `final_pulse_duration_invalid` | パルスが150 ms未満。パルス再試行 |
| `reset` reason `outbound_timeout` | シェイク後 2500 ms 以内に掌上不成。v1=phi、v2=3D角、v3=Δz |
| `reset` reason `final_accel_missing` | 掌上後 5000 ms 以内に上向き加速が不足 |
| `reset` reason `final_brake_missing` | 掌上後 5000 ms 以内に減速パルス不成 |
| `reset` reason `final_tilt_unstable` | 上昇後の姿勢差が過大 |

## 検証

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 1 --window 15 \
  --json-output /tmp/gesture-lift.json
```

表示条件: 掌下シェイク、掌上、挙上、掌下で静止、姿勢安定、
単一動作、発動、掌上で録音終了。
`recording_start` のあとホスト `0x00` では止めず、掌上による `recording_stop` を待つ。
判定時間内に停止がなければ後始末として `0x00` を送る。

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
更新後 version `0.0.47` が slot0 active+confirmed であること。
