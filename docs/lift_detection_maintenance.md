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
   - 初回 arm で重力 phi と az を固定し、静止中に追随しない。
   - armed 中は静止していても、phi または Z 方向変化で回内開始する。
2. `GESTURE_OUTBOUND`
   - XZ 平面の重力 phi、または Z 比 / 符号の明確な変化で回内成立。
   - 成立後の静止は `HOLDING_FINAL` で許容する。
3. `GESTURE_HOLDING_FINAL`
   - `WAIT_ACCEL`: 重力方向へ投影した線形加速度 `a_up` の上向きパルスを検出。
   - `WAIT_BRAKE`: 続く逆向き減速があれば早期に hold へ進む。
   - 1800 ms 以内に減速がなくても、正インパルスが下限以上なら hold へ進む。
   - `WAIT_HOLD`: 短時間 RMS で静止進入を確認し、その時点の重力方向を固定。
   - 固定姿勢からの傾きが上限内のまま 500 ms 静止すると録音開始。

上向き加速と逆向き減速は、連続サンプル数、正負インパルス、
減速/加速インパルス比、150–1800 ms の時間窓を組み合わせて判定する。
これにより、以前の変位二重積分にあったドリフトとスケールのばらつきを避ける。

## 主要閾値

| 定数 | 値 | 用途 |
|------|---:|------|
| `GESTURE_QUIET_ACCEL_MS2` | 3.0 m/s² | 開始静止の線形加速度上限 |
| `GESTURE_QUIET_HOLD_MS` | 50 ms | 開始静止の継続時間 |
| `GESTURE_START_PALM_UP_Z_MIN_RATIO` | 0.80 | 掌上開始時の Z 絶対比（初回 arm のみ） |
| `GESTURE_START_ARM_MS` | 2500 ms | 静止成立後の開始受付時間 |
| `GESTURE_PRONATION_START_DEG` | 8° | 回内開始角 |
| `GESTURE_PRONATION_MIN_DEG` | 20° | 回内成立角（phi）。Z 比 0.50 または符号反転でも成立 |
| `GESTURE_PRONATION_Z_RATIO_START` | 0.25 | 回内開始の Z 比変化 |
| `GESTURE_PRONATION_Z_RATIO_DONE` | 0.50 | 回内成立の Z 比変化 |
| `GESTURE_PRONATION_Z_PTP_START_MS2` | 4.0 m/s² | 回内開始の Z ピーク間 |
| `GESTURE_PRONATION_Z_SIGN_MIN_MS2` | 2.0 m/s² | Z 符号反転と認める \|az\| 下限 |
| `GESTURE_INCOMPLETE_SETTLE_MS` | 800 ms | 回内未完了の静止リセット |
| `GESTURE_LIFT_ACCEL_MIN_MS2` | 0.40 m/s² | 上向き加速パルス下限 |
| `GESTURE_LIFT_POS_IMPULSE_MIN_MS` | 0.04 m/s | 正インパルス下限 |
| `GESTURE_LIFT_BRAKE_MIN_MS2` | 0.15 m/s² | 逆向き減速パルス下限 |
| `GESTURE_LIFT_NEG_IMPULSE_MIN_MS` | 0.015 m/s | 負インパルス下限 |
| `GESTURE_LIFT_BRAKE_RATIO_MIN` | 0.05 | 減速/加速インパルス比下限 |
| `GESTURE_LIFT_PULSE_MIN_MS` / `MAX_MS` | 150 / 1800 ms | パルス時間窓 |
| `GESTURE_LIFT_FINAL_TILT_MAX_DEG` | 10° | hold 中の姿勢差上限 |
| `GESTURE_FINAL_STILL_RMS_MS2` | 2.0 m/s² | 静止進入 RMS 上限 |
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
- `wait_reject` reason `outbound_rate_low`: armed だが回内開始未満。v1=phi、v2=Δz比、v3=Zピーク間
- `outbound_ready`: v1=phi、v2=peak phi、v3=Δz比
- `wait_reject`（パルス再試行）: 減速不足、比率不足、パルス時間不足。一連動作は継続
- `reset`: 回内失敗、全体/最終期限切れ、姿勢・hold 失敗。掌上からやり直し

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

2026-08-18 の確認結果（`0.0.42`、回内の Z/phi 条件）:

- validator self-test: PASS
- `xiao_ble/nrf52840/sense` + sysbuild: PASS
- OTA version `0.0.42`: slot 0 で active / confirmed
- 正例 2 試行: いずれも PASS
  - 1 回目: phi 開始 9.6° → ピーク 175°、Δz 比 1.76、4984 ms
  - 2 回目: phi 開始 9.7° → ピーク 96°、Δz 比 1.46、3995 ms
- 本変更後の回内のみ / 小刻み動作 / 歩行 / 車載の誤発動確認は未実施

## 保守上の注意

- ジャイロは省電力方針により無効。再有効化は要件確認後に行う。
- 単一閾値を下げるだけで合格率を上げない。双極パルスと姿勢・静止ガードを維持する。
- 重力 LP はパルス開始まで追従し、hold の姿勢基準は静止進入時に固定する。
- Z 軸の装着符号は固定せず、開始判定では絶対比を使う。arm 後の回内は符号付き Z 変化とピーク間も使う。
- 最終静止の RMS 上限は運転中の振動を見込んで 2.0 m/s²。500 ms の継続は維持する。
- 減速パルス不足は `wait_reject` の再試行であり、`reset` ではない。
- 実機ログでは正負インパルス、比率、姿勢差、hold 時間、reset / wait_reject reasonを確認する。

## 関連ファイル

- `nordic-main/src/main.c`: 状態機械、閾値、BLE 診断
- `mac_client/gesture_validator.py`: BLE 試験、診断表示、JSON 保存
- `docs/flex_pronation_gesture.md`: ジェスチャ仕様
- `docs/nordic_main_guide.md`: ビルド、OTA、運用
