# シェイク→掌上→挙上→掌下静止ジェスチャー仕様

対象デバイス: Seeed Studio XIAO nRF52840 Sense  
対象ファームウェア: HarnessNode `0.0.52` 以降

## 発動条件

- XIAO をリストバンドの**手の甲側**に置く
- 部品面を皮膚側にし、基板 X 軸を前腕と直交、Y 軸を前腕に沿わせる
- 手のひらを下向きにし、基板がほぼ水平（|Z 比| ≥ 0.80）のまま短く **1 回シェイク**する
  - シェイクは重力に直交する線形加速度だけを見る
  - 500 ms 窓の峰間 ≥ 5.0 m/s²、かつ `|平均| < 0.4 × 峰間`
  - 持続する同じ符号の横 G（運転）では平均 ≈ ピークとなり不成立
  - シェイク成立でジャイロ ON（104 Hz）。掌上窓 1.5 s 内に ~100 ms 整定
- シェイク後 1.5 秒以内に掌を上へ反転させる（掌上）
  - 基準はシェイク窓の最初のサンプル時点の重力 LP（振った瞬間の raw ではない）
  - 相対変化: 重力 3D 角 ≥ 20°、phi ≥ 12°、`|Δz比| ≥ 0.35`、Z 符号反転、
    **または** `|∫gyro_y dt| ≥ 25°` / `peak|gyro_y| ≥ 35°/s`（重力 OR ジャイロ）
  - 口元への掌上は前腕ピッチになりやすく、XZ の phi だけでは拾えない
  - 装着ごとの raw Z 符号に依存する絶対掌上は使わない
  - 掌上成立時に基準 phi / az を取り直し、outbound の `gyro_y` 符号を記憶する
- 掌向きを問わず、口元へ上げる（物理目安 **約 5 cm**）
- 上げたあと、**掌上基準から手首を回して掌下** にし、**400 ms** 静止する
  - 掌下: 重力反転 **または** outbound と逆符号の `gyro_y`（角 ≥ 20° / peak ≥ 30°/s）
  - 静止進入: 線形加速度 RMS ≤ 3.0 m/s²。進入時のみ `|gyro_y| ≤ 90°/s`
  - 保持中の姿勢差 ≤ 15°（静止開始時の重力方向から）
- 録音中に手のひらを上へ向けると録音終了する。手を下ろすだけでは終了しない
- ジャイロは録音終了（またはシーケンス失敗）で OFF

右手・左手および USB 向きの差は、シェイク後の相対符号で吸収する。  
最終姿勢の仰角帯・甲上条件・回外は要求しない。

運転の横 G は重力 LP（tau 0.30 s）で遅い車体加速度を姿勢から除き、
シェイク判定では直交成分の直流（平均 ≈ ピーク）を落とす。

## 装着と軸

| 軸 | 基板上 | 役割 |
|---|---|---|
| `X` | 長手（USB から離れる） | 前腕を横切る |
| `Y` | 短い辺方向（5V/GND 側） | 前腕軸＝回内・回外軸（**gyro_y**） |
| `Z` | 部品面外向き | 基板水平（|Z 比|）の判定 |

実機検証では装着ごとに raw `z` の符号が反転したため、
水平姿勢は Z の絶対比、掌上はシェイク前 LP からの 3D / phi / Z 変化、
掌下 hold は掌上基準からの相対 phi / Z 変化で判定する。角速度は **gyro_y**。

```text
board_flat = abs(z / |a|) >= 0.80
palm_up_after_shake = gravity gates OR |∫ω_y|>=25° OR peak|ω_y|>=35 dps
palm_down_after_lift = gravity flip OR opposite-sign gyro_y gates
hold_entry = palm_down AND rms<=3.0 AND tilt<=15° AND (timer running OR |ω_y|<=90)
```

## 上昇動作の測り方（双極パルス）

掌上成立後:

1. 低通加速度を重力推定とし、rawとの差から線形加速度を得る
2. 線形加速度をその時点の低通重力方向へ投影し、上向き加速度 `a_up` を得る
3. 上向きの加速パルスを確認する。減速パルスがあれば早期に hold へ進む
4. 正インパルスと 150–1800 ms の時間形状を満たしたら上昇成立。減速パルスは必須ではない。掌向きは見ない
5. 掌上基準から手首が反転してから、短い加速度 RMS 窓が静止を示した時点の重力方向を保持基準とし、その後の角度差が 15° 以内か確認する

変位の二重積分は使わない。約 5 cm はユーザー動作の目安であり、判定閾値ではない。
加速インパルスと、掌上基準からの反転後の姿勢安定・400 ms 静止を満たす必要がある。減速は任意。

## 録音停止（開始姿勢からの掌上）

録音開始時の重力 LP（掌下静止）を基準として保存する。`reset_gesture_sequence()` では消さない。録音中も重力LPとジャイロは継続する。

開始後 1200 ms を過ぎたあと、次のいずれかで即座に停止する（hold は待たない）:

1. 重力: 3D 角 ≥ 20°、phi ≥ 12°、`|Δz比| ≥ 0.35`、または Z 符号反転
2. ジャイロ: `|∫gyro_y dt| ≥ 25°` または `peak|gyro_y| ≥ 35°/s`
3. 共通: `|a|` が 7.5–12.5 m/s²

手を重力方向へ下ろすだけでは停止しない。`motion_active` でも止めない。ホスト `0x00` とシリアル `'s'` は従来どおり。停止後にジャイロ OFF。

## 状態遷移

```text
WAITING → OUTBOUND(掌上)  [shake accept → gyro ON]
        → HOLDING_FINAL(WAIT_ACCEL → WAIT_BRAKE → WAIT_HOLD)
WAIT_HOLD は掌上基準からの反転が取れるまで開始しない
WAIT_BRAKE で 1800 ms 経過し正インパルスがあれば WAIT_HOLD へ。不足なら WAIT_ACCEL 再試行
MATCH → recording (gyro stays ON) → stop palm-up → gyro OFF
```

## しきい値

| 項目 | 値 |
|---|---:|
| 開始水平 | \|Z 比\| ≥ 0.80 |
| シェイク窓 | 20 サンプル（約 500 ms） |
| シェイク峰間 | 重力直交成分 ≥ 5.0 m/s² |
| シェイク平均比 | \|平均\| < 0.4 × 峰間 |
| シェイク軸下限 | 直交成分 ≥ 2.0 m/s² で軸を固定 |
| 掌上 | 重力ゲート **または** \|∫ω_y\|≥25° / peak≥35 dps |
| 掌上窓 | シェイク後 1500 ms |
| ジャイロ | シェイク成立で ON、録音終了で OFF。ODR 104 Hz、FS ±500 dps、整定 100 ms |
| 上向き加速度 | `a_up` ≥ 0.40 m/s² を2サンプル、正インパルス ≥ 0.04 m/s |
| 逆向き減速 | 任意。`a_up` ≤ -0.15 m/s² なら早期に hold へ |
| パルス形状 | 150–1800 ms。1800 ms 時点で正インパルスがあれば hold へ |
| hold の掌 | 重力反転 **または** 逆符号 gyro_y（角≥20° / peak≥30 dps） |
| 保持中の姿勢差 | 静止開始時の重力方向から ≤ **15°** |
| 静止進入 | 4サンプル RMS ≤ **3.0** m/s²。進入時のみ \|ω_y\|≤**90** dps |
| 最終静止 | **400** ms |
| 最終到達期限 | 掌上後 **5000** ms |
| 全シーケンス | 6000 ms |
| 録音停止 | 重力ゲート **または** gyro_y ゲート。\|a\| 7.5–12.5 m/s² |
| 録音停止 開始抑制 | 1200 ms |

## 診断イベント（抜粋）

| stage | 意味 |
|---|---|
| `outbound_start` | シェイク成立。v1=峰間、v2=Z絶対比、v3=平均 |
| `outbound_ready` | 掌上。v1=phi、v2=3D角、v3=Δz 比 |
| `outbound_gyro` (0x0F) | 掌上/停止時の gyro。v1=∫ω_y、v2=peak dps、v3=sign |
| `gyro_enabled` (0x0D) / `gyro_disabled` (0x0E) | ジャイロ電源 |
| `wait_reject` reason `start_not_palm_up` | 基板が水平でない。v1=Z比、v2=下限、v3=線形加速度 |
| `wait_reject` reason `quiet_not_ready` | シェイク窓未充足。v1=Z比、v2=件数、v3=必要件数 |
| `wait_reject` reason `shake_not_oscillatory` | 直流横Gなど。v1=峰間、v2=平均、v3=\|平均\|/峰間 |
| `wait_reject` reason `final_hold_interrupted` | hold 中断。v1=RMS、v2=tilt、v3=\|gy\| |
| `final_sample` (0x21) | 挙上中 100 ms 周期: pulse_stage, a_up, net_impulse |
| `hold_sample` (0x22) | hold 中 100 ms 周期: rms, tilt, \|gy\| |
| `final_hold_start` / `final_ready` | 静止保持開始 / 完了（v1=pos imp、v2=neg imp or hold_ms、v3=tilt） |
| `match` | 発動 |
| `stop_palm_up` (0x0C) | 録音中の掌上停止。v1=phi、v2=3D角、v3=Δz（直後に `outbound_gyro`） |
| `reset` reason `outbound_timeout` | 掌上不成 |
| `reset` reason `final_hold_timeout` | 掌上後 5000 ms 以内に hold 不成 |
| `reset` reason `final_accel_missing` | 掌上後 5000 ms 以内に上向き加速が不足 |

### 履歴バッチ（`GESTURE_DEBUG_HISTORY=1` のときのみ）

録音終了またはシーケンス失敗後にまとめて送信（録音中は送らない）:

| event | 内容 |
|---|---|
| `0x33` history_begin | count, session_id |
| `0x34` history_entry | u16 t_ms, stage, reason, f32×3（19 B） |
| `0x35` history_end | count, session_id |

## 検証

```bash
cd mac_client
venv/bin/python gesture_validator.py --trials 1 --window 18 \
  --json-output /tmp/gesture-lift.json
```

表示条件: 掌下シェイク、掌上、挙上、掌下で静止、姿勢安定、
単一動作、発動、掌上で録音終了。各行に **実測値と閾値** を出す。
停止は重力 OR gyro_y で判定する。
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
更新後 version `0.0.52` が slot0 active+confirmed であること。
