# 腕上げ検出の保守ガイド

## 概要

`nordic-main` の録音開始ジェスチャは、手の甲側に装着した XIAO
nRF52840 Sense の加速度センサだけを使用する。

意図する操作:

1. 掌上で静止
2. 回内
3. 腕を約 5 cm 持ち上げる
4. 上昇位置で 0.5 秒静止

物理的な上昇距離は操作の目安であり、判定では変位を積分しない。

## 現行アルゴリズム

`nordic-main/src/main.c` の `process_gesture_sample()` は、次の順で判定する。

1. `GESTURE_WAITING`
   - 重力に対する Z 絶対比と線形加速度から掌上静止を確認する。
   - arm 成立時の重力 phi を回内角の基準にする。
2. `GESTURE_OUTBOUND`
   - XZ 平面の重力 phi が基準から規定角以上変化すると回内成立。
3. `GESTURE_HOLDING_FINAL`
   - `WAIT_ACCEL`: 重力方向へ投影した線形加速度 `a_up` の上向きパルスを検出。
   - `WAIT_BRAKE`: 続く逆向き減速パルスを検出。
   - `WAIT_HOLD`: 短時間 RMS で静止進入を確認し、その時点の重力方向を固定。
   - 固定姿勢からの傾きが上限内のまま 500 ms 静止すると録音開始。

上向き加速と逆向き減速は、連続サンプル数、正負インパルス、
減速/加速インパルス比、150–900 ms の時間窓を組み合わせて判定する。
これにより、以前の変位二重積分にあったドリフトとスケールのばらつきを避ける。

## 主要閾値

| 定数 | 値 | 用途 |
|------|---:|------|
| `GESTURE_QUIET_ACCEL_MS2` | 3.0 m/s² | 開始静止の線形加速度上限 |
| `GESTURE_QUIET_HOLD_MS` | 50 ms | 開始静止の継続時間 |
| `GESTURE_START_PALM_UP_Z_MIN_RATIO` | 0.85 | 掌上開始時の Z 絶対比 |
| `GESTURE_PRONATION_START_DEG` | 10° | 回内開始角 |
| `GESTURE_PRONATION_MIN_DEG` | 20° | 回内成立角 |
| `GESTURE_LIFT_ACCEL_MIN_MS2` | 0.80 m/s² | 上向き加速パルス下限 |
| `GESTURE_LIFT_POS_IMPULSE_MIN_MS` | 0.08 m/s | 正インパルス下限 |
| `GESTURE_LIFT_BRAKE_MIN_MS2` | 0.30 m/s² | 逆向き減速パルス下限 |
| `GESTURE_LIFT_NEG_IMPULSE_MIN_MS` | 0.03 m/s | 負インパルス下限 |
| `GESTURE_LIFT_BRAKE_RATIO_MIN` | 0.08 | 減速/加速インパルス比下限 |
| `GESTURE_LIFT_PULSE_MIN_MS` / `MAX_MS` | 150 / 900 ms | パルス時間窓 |
| `GESTURE_LIFT_FINAL_TILT_MAX_DEG` | 10° | hold 中の姿勢差上限 |
| `GESTURE_FINAL_STILL_RMS_MS2` | 0.50 m/s² | 静止進入 RMS 上限 |
| `GESTURE_FINAL_HOLD_MS` | 500 ms | 最終静止時間 |

実装変更時は `docs/flex_pronation_gesture.md` と
`docs/nordic_main_guide.md` の値も同時に更新する。

## 診断と validator

`mac_client/gesture_validator.py` は BLE 診断を条件別に表示し、JSONへ保存する。
主なイベント:

- `final_sample`: pulse stage、`a_up`、インパルス
- `final_hold_start`: 正負インパルス、静止開始時の姿勢差
- `final_ready`: 正インパルス、hold 時間、姿勢差
- `match`: 回内角、正インパルス、hold 時間
- `reset`: 加速不足、減速不足、比率不足、時間窓、姿勢、hold 失敗理由

変更後は少なくとも次を実行する。

```bash
cd mac_client
venv/bin/python gesture_validator.py --self-test
python3 -m py_compile gesture_validator.py
```

## ビルドと実機確認

Board は必ず `xiao_ble/nrf52840/sense`、ビルドは sysbuild を使用する。
詳細な環境設定は `AGENTS.md`、`nordic-main/AGENTS.md`、
`docs/nordic_main_guide.md` を参照する。

```bash
cd nordic-main
./build_and_package_ota.sh
```

実機試験では、正例を 5 回実施して 4/5 以上を目安とする。続いて、
回内のみ、小刻み動作、歩行で no-match を確認する。車載振動は安全に
実施できる場合だけ追加する。

2026-08-18 の確認結果:

- validator self-test: PASS
- `xiao_ble/nrf52840/sense` + sysbuild: PASS
- OTA version `0.0.41`: slot 0 で active / confirmed
- 正例確認: PASS
- 正例安定性: 4/5 PASS
- 回内のみ、小刻み動作、歩行: すべて誤発動なし
- 車載振動: 未実施

## 保守上の注意

- ジャイロは省電力方針により無効。再有効化は要件確認後に行う。
- 単一閾値を下げるだけで合格率を上げない。双極パルスと姿勢・静止ガードを維持する。
- 重力 LP はパルス開始まで追従し、hold の姿勢基準は静止進入時に固定する。
- Z 軸の装着符号は固定せず、開始判定では絶対比を使う。
- 25 ms のサンプル周期、3 秒の最終期限、5 秒の全体期限を前提とする。
- 実機ログでは正負インパルス、比率、姿勢差、hold 時間、reset reasonを確認する。

## 関連ファイル

- `nordic-main/src/main.c`: 状態機械、閾値、BLE 診断
- `mac_client/gesture_validator.py`: BLE 試験、診断表示、JSON 保存
- `docs/flex_pronation_gesture.md`: ジェスチャ仕様
- `docs/nordic_main_guide.md`: ビルド、OTA、運用
