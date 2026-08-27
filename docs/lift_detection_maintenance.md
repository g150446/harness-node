# 腕上げ検出の保守ガイド

## 概要

`nordic-main` の録音開始ジェスチャは、手の甲側に装着した XIAO
nRF52840 Sense の **加速度（常時）** と **ジャイロ（オンデマンド）** を使用する。

意図する操作:

1. 掌上候補で 0.5 秒静止する（成立時にジャイロ ON）
2. 手を上げる（掌向きは問わない）
3. 掌を下にして 500 ms 静止
4. 録音中に挙上と逆向きの線形パルス（手下ろし）で録音終了
5. 録音終了でジャイロ OFF

物理的な上昇距離は操作の目安であり、判定では変位を積分しない。

## 現行アルゴリズム

`nordic-main/src/main.c` の `process_gesture_sample()` は、次の順で判定する。

1. `GESTURE_WAITING`
   - `|Z 比| ≥ 0.75`、`|a|` 8.5–11.5 m/s²、RMS ≤ 4.0 m/s²、姿勢差 ≤ 20° を 500 ms 維持。
   - 成立時 `gyro_set_enabled(true)`（104 Hz / ±500 dps）し、その姿勢を掌上基準とする。
2. `GESTURE_HOLDING_FINAL`
   - `WAIT_ACCEL` / `WAIT_BRAKE`: `a_up` パルス。掌向きは見ない。
   - `WAIT_HOLD`: 掌上基準からの反転（重力 OR 逆符号 gyro_y）後、RMS 静止と 500 ms hold。
   - 掌下の重力＋gyro連動条件は一度成立したらその試行中ラッチする。
   - hold **進入時のみ** `|ω_y| ≤ 90 dps`、RMS ≤ 3.0。進入後はRMS > 3.5が2サンプル連続した場合だけ中断する。
   - gyro起動後、5000 ms以内に挙上が始まらなければ `final_accel_missing`。
   - 挙上開始から4500 ms時点で掌下連動が未成立なら`palm_down_gate_failed`、成立済みで500 ms holdが未完了なら`motion_too_slow`。
3. 録音中
   - MATCH 時に挙上軸 `L` を固定し、逆向き線形パルス + settle で録音停止。
   - 掌上のみ・静止のみでは止めない。停止後ジャイロ OFF。

上向き加速と逆向き減速は、連続サンプル数、正負インパルス、
減速/加速インパルス比を組み合わせて判定する。明瞭な減速が取れなくても、
掌下・gyro静定・4サンプルRMS静止が揃えばholdへ進む。

## 主要閾値

| 定数 | 値 | 用途 |
|------|---:|------|
| `GESTURE_START_PALM_UP_Z_MIN_RATIO` | 0.75 | 開始時の基板水平（\|Z 比\|） |
| `GESTURE_PALM_UP_DWELL_MS` | 500 ms | 掌上候補の連続静止 |
| `GESTURE_PALM_UP_DWELL_TILT_MAX_DEG` | 20° | 候補開始姿勢からの許容差 |
| `GESTURE_START_QUIET_ACCEL_MS2` | 4.0 m/s² | 掌上候補の線形加速度/RMS上限 |
| `GYRO_ODR_HZ` / `GYRO_FULL_SCALE_DPS` | 104 / 500 | オンデマンドジャイロ |
| `GESTURE_GYRO_SETTLE_MS` | 50 ms | ジャイロ整定 |
| `GESTURE_STOP_OPP_ACCEL_MIN_MS2` | 0.25 | 録音停止の逆向き a 下限（0.0.71） |
| `GESTURE_STOP_OPP_IMPULSE_MIN_MS` | 0.10 | 録音停止の負インパルス下限 |
| `GESTURE_STOP_OPP_IMPULSE_LIFT_RATIO` | 0.20 | 負インパルス ≥ ratio×lift |
| `GESTURE_STOP_OPP_IMPULSE_LIFT_CAP_MS` | 0.35 | lift 相対閾値の上限 |
| `GESTURE_STOP_OPP_PULSE_MIN_MS` / `MAX` | 60 / 2000 | パルス時間窓（車両の長G除外） |
| `GESTURE_STOP_SETTLE_MS` | 80 ms | パルス後の quiet 保持 |
| `GESTURE_HOLD_GYRO_INTEGRATE_RATE_DPS` | 10 dps | hold 回内の積分対象レート |
| `GESTURE_HOLD_GYRO_ANGLE_MIN_DEG` | 30° | hold 回内の ∫ω_y（0.0.70） |
| `GESTURE_HOLD_GYRO_XY_PEAK_RATIO_MIN` | 0.42 | peak \|ω_x\| / peak \|ω_y\| |
| `GESTURE_LIFT_PREFLIP_MAX_DEG` | 50° | 挙上成立時の \|∫ωy\| 上限（未満=回内前） |
| `GESTURE_LIFT_XY_WAIVER_IMPULSE_MIN_MS` | 0.30 | XY 免除に必要な入口 +imp |
| `GESTURE_LIFT_POS_IMPULSE_MIN_MS` | 0.30 | 挙上正インパルス下限（0.0.68） |
| `GESTURE_FINAL_HOLD_MS` | 500 ms | 最終静止（0.0.68） |
| `GESTURE_FINAL_QUIET_RATE_DPS` | 90 dps | hold **進入時のみ** |
| `GESTURE_PRONATION_MIN_DEG` | 15° | hold 反転角（phi）。Z 比 0.40 または符号反転でも成立 |
| `GESTURE_PRONATION_Z_RATIO_DONE` | 0.40 | hold 反転の Z 比変化 |
| `GESTURE_LIFT_ACCEL_MIN_MS2` | 0.40 m/s² | 上向き加速パルス下限 |
| `GESTURE_LIFT_PULSE_MIN_MS` | 150 ms | 短すぎるパルスの下限 |
| `GESTURE_LIFT_FINAL_TILT_MAX_DEG` | 15° | hold 中の姿勢差上限 |
| `GESTURE_FINAL_STILL_RMS_MS2` | 3.0 m/s² | 静止進入 RMS 上限 |
| `GESTURE_FINAL_HOLD_RMS_EXIT_MS2` | 3.5 m/s² | hold中のRMS中断閾値 |
| `GESTURE_FINAL_HOLD_RMS_EXIT_SAMPLES` | 2 | 中断に必要な連続超過数 |
| `GESTURE_LIFT_START_TIMEOUT_MS` | 5000 ms | gyro起動後に挙上を開始するまでの期限 |
| `GESTURE_MOTION_COMPLETE_MAX_MS` | 4500 ms | 挙上開始から最終静止完了まで。4.5秒以上は不成立 |
| `GESTURE_RETRIGGER_BLOCK_MS` | 3000 ms | 開始直後の停止抑制（基準再ロック）/ 停止後の再開始抑制 |

実装変更時は `docs/flex_pronation_gesture.md` と
`docs/nordic_main_guide.md` の値も同時に更新する。

## 診断と validator

`mac_client/gesture_validator.py` は BLE 診断を条件別に **実測と閾値** を並べて表示し、JSONへ保存する。
主なイベント:

- `outbound_start`: 掌上候補の Z 比、線形加速度
- `outbound_ready`: 掌上静止時間、Z 比、線形加速度
- `outbound_gyro`: ∫ω_y、peak dps、sign（開始 hold 時）
- `gyro_enabled` / `gyro_disabled`
- `final_sample` / `hold_sample`
- `final_hold_start` / `final_ready` / `match`
- `motion_complete`: 挙上開始から最終静止完了までの時間、gyro Y peak、積分角
- `palm_down_gate`: 重力反転、gyro Y積分角、peak gyro X/Y比の不足理由
- `stop_hand_lower` (0x0C): opp_imp / peak a_opp / pulse_ms（手下ろし停止）
- `wait_reject` reason `final_hold_interrupted`: RMS / tilt / \|gy\|
- `reset` reason `palm_down_gate_failed`: 掌下連動条件が期限までに一度も成立しなかった
- `reset` reason `motion_too_slow`: 掌下成立後も最終静止が期限内に完了しなかった

validatorは開始と停止を二段階で案内する。Ping音の`GO`後は`START OK`まで掌下を維持し、
Glass音と`STOP GO`のあとに**腕を下ろす**。STOP GO前の停止は失敗として区別する。
開始イベントが発生しなかった試行では停止合図も出ず、停止動作の所要時間は開始判定に混ぜない。

履歴有効ファームのPNGは、上段が加速度XYZ、下段がgyro XYZ、灰色部分がgyro未取得期間である。
gyro Yの回内は `START OK` 前後（録音開始）で確認する。停止は加速度の逆向きパルス側を見る。
gyro X/Zのピークは挙上に伴う複合回転でも生じるため、単独で掌下成立とは判定しない。

変更後は少なくとも次を実行する。

```bash
cd mac_client
venv/bin/python gesture_validator.py --self-test
python3 -m py_compile gesture_validator.py
```

閾値変更前の少数サンプル収集は `gesture_dataset_collector.py` で1試行ずつ行う。
最初は左右それぞれ、正例（自然・遅い・速い）3回と負例（挙上のみ・回転のみ・日常動作）
3回の計12回とする。`gesture_dataset_analyzer.py` の正例ばらつきまたは正負分離フラグが
立った場合だけ、該当条件を左右各1回ずつ追加し、上限16回とする。車・電車の実測が
できない段階では、疑似水平加速度の結果を実環境での合格証明には使わない。

## ビルドと実機確認

Board は必ず `xiao_ble/nrf52840/sense`、ビルドは sysbuild を使用する。
詳細な環境設定は `AGENTS.md`、`nordic-main/AGENTS.md`、
`docs/nordic_main_guide.md` を参照する。

```bash
cd nordic-main
./build_and_package_ota.sh
```

実機試験では、正例を 5 回実施して 4/5 以上を目安とする。続いて、
掌上のまま上げる、持続横 G、歩行で no-match を確認する。車載振動は安全に
実施できる場合だけ追加する。停止は**手下ろし**で確認する（掌上では止めない）。

2026-08-27 の確認結果（`0.0.70`→`0.0.71`）:

- validator self-test: PASS
- Mac 通し: 開始 + 手下ろし停止 PASS（opp_imp≈1.4）
- Android「リンゴ」履歴: 旧閾値では約 13 s で opp_imp≈0.189 の弱い停止。0.0.71 で緩和
- hold `|∫gyro_y|` は 50°→30°（0.0.70）。自然な掌下 ∫≈35° を通す
- 録音停止は挙上軸逆向きパルス + settle（0.0.69+）。掌上経路は廃止

## 保守上の注意

- ジャイロはオンデマンド（掌上0.5秒静止成立〜録音終了）。待機時は ODR=0。
- hold 中の gyro quiet は **進入時のみ**。進入後に \|ω_y\| で中断しない。
- 掌下の重力＋gyro連動成立は試行中ラッチし、停止時の逆回転で積分角が減っても取り消さない。
- 単一閾値を下げるだけで合格率を上げない。開始の水平・静止と掌下 hold を維持する。
- 掌上基準は0.5秒静止成立時の重力 LP。成立瞬間の raw では武装しない。
- 重力 LP はパルス開始まで追従し、hold の姿勢基準は静止進入時に固定する。
- 録音停止は MATCH 時の挙上軸 `L` を固定し、開始後 3000 ms 抑制のあと逆向きパルス + settle。
  `reset_gesture_sequence()` では消さない。掌上・静止のみでは止めない。
- 最終静止は進入RMS 3.0 m/s²以下、保持中は3.5 m/s²超過2サンプル連続で中断、継続500 ms。
  挙上開始待ちは5 s、動作完了は4.5 s未満。
- 減速パルスは任意。取れない場合も掌下・gyro静定・4サンプルRMS静止でholdへ進む。
- 実機ログでは掌上候補の Z 比・静止時間、加速度/ジャイロ6軸、正負インパルス、姿勢差、
  hold 時間、`stop_hand_lower` の opp_imp/peak/pulse、reset reason を確認する。

## 関連ファイル

- `nordic-main/src/main.c`: 状態機械、閾値、BLE 診断
- `mac_client/gesture_validator.py`: BLE 試験、診断表示、JSON / 6軸グラフ、`--start-only`
- `mac_client/imu_trajectory.py`: 6軸バッチ復元・CSV・PNG
- `docs/flex_pronation_gesture.md`: 仕様正本（現行 0.0.68）
- `mac_client/gesture_dataset_collector.py`: 6秒のラベル付き6軸収集
- `mac_client/gesture_dataset_analyzer.py`: 左右比較、特徴抽出、疑似横加速度感度解析
- `mac_client/imu_trajectory.py`: 6軸バッチ復元、CSV、グラフ共通処理
- `docs/flex_pronation_gesture.md`: ジェスチャ仕様
- `docs/nordic_main_guide.md`: ビルド、OTA、運用
