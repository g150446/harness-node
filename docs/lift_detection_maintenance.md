# 腕上げ検出の保守ガイド

## 概要

`nordic-main` の録音開始ジェスチャは、手の甲側に装着した XIAO
nRF52840 Sense の加速度センサだけを使用する。

意図する操作:

1. 掌下で短く 1 回シェイクする
2. 手のひらを上向きにする
3. 手を上げる（掌向きは問わない）
4. 掌を下にして 0.5 秒静止
5. 録音中に手のひらを上へ向けると録音終了（手下ろしでは終了しない）

物理的な上昇距離は操作の目安であり、判定では変位を積分しない。

## 現行アルゴリズム

`nordic-main/src/main.c` の `process_gesture_sample()` は、次の順で判定する。

1. `GESTURE_WAITING`
   - `|Z 比| ≥ 0.80` で基板が水平なときだけ、重力直交の線形加速度をシェイク窓へ入れる。
   - 500 ms 窓の峰間と平均比で 1 回の短い振りを確認し、持続横 G を落とす。
2. `GESTURE_OUTBOUND`
   - シェイク窓開始時の重力 LP 基準から、3D 角 / 緩い phi / Δz / Z 符号反転で掌上成立。
   - 成立時に phi / az を掌上基準として取り直す。未完了のまま 2500 ms で reset。
3. `GESTURE_HOLDING_FINAL`
   - `WAIT_ACCEL` / `WAIT_BRAKE`: `a_up` パルス。掌向きは見ない。
   - `WAIT_HOLD`: 掌上基準からの反転が取れてから RMS 静止と 500 ms hold。
   - 反転できずに期限切れなら `lift_palm_still_up`。
4. 録音中
   - 開始時の重力 LP を基準に保存し、緩い掌上反転で録音停止。
   - 手下ろしでは止めない。`motion_active` でも止めない。

上向き加速と逆向き減速は、連続サンプル数、正負インパルス、
減速/加速インパルス比、150–1800 ms の時間窓を組み合わせて判定する。
これにより、以前の変位二重積分にあったドリフトとスケールのばらつきを避ける。

## 主要閾値

| 定数 | 値 | 用途 |
|------|---:|------|
| `GESTURE_START_PALM_UP_Z_MIN_RATIO` | 0.80 | 開始時の基板水平（\|Z 比\|） |
| `GESTURE_SHAKE_WINDOW_SAMPLES` | 20 | シェイク窓（約 500 ms） |
| `GESTURE_SHAKE_PTP_MIN_MS2` | 5.0 m/s² | 重力直交成分の峰間 |
| `GESTURE_SHAKE_MEAN_RATIO_MAX` | 0.4 | \|平均\| / 峰間 の上限 |
| `GESTURE_SHAKE_AXIS_MIN_MS2` | 2.0 m/s² | シェイク軸を固定する直交成分下限 |
| `GESTURE_PRONATION_MIN_DEG` | 20° | hold 反転角（phi）。Z 比 0.50 または符号反転でも成立 |
| `GESTURE_PRONATION_Z_RATIO_DONE` | 0.50 | hold 反転の Z 比変化 |
| `GESTURE_PRONATION_Z_SIGN_MIN_MS2` | 2.0 m/s² | Z 符号反転と認める \|az\| 下限 |
| `GESTURE_OUTBOUND_MIN_DEG` | 8° | シェイク後掌上の XZ phi |
| `GESTURE_OUTBOUND_TILT_MIN_DEG` | 15° | シェイク後掌上の重力 3D 角 |
| `GESTURE_OUTBOUND_Z_RATIO_DONE` | 0.25 | シェイク後掌上の Z 比変化 |
| `GESTURE_OUTBOUND_MAX_DURATION_MS` | 2500 ms | シェイク後の掌上期限 |
| `GESTURE_LIFT_ACCEL_MIN_MS2` | 0.40 m/s² | 上向き加速パルス下限 |
| `GESTURE_LIFT_POS_IMPULSE_MIN_MS` | 0.04 m/s | 正インパルス下限 |
| `GESTURE_LIFT_BRAKE_MIN_MS2` | 0.15 m/s² | 逆向き減速パルス下限 |
| `GESTURE_LIFT_NEG_IMPULSE_MIN_MS` | 0.015 m/s | 負インパルス下限 |
| `GESTURE_LIFT_BRAKE_RATIO_MIN` | 0.05 | 減速/加速インパルス比下限 |
| `GESTURE_LIFT_PULSE_MIN_MS` / `MAX_MS` | 150 / 1800 ms | パルス時間窓 |
| `GESTURE_LIFT_FINAL_TILT_MAX_DEG` | 10° | hold 中の姿勢差上限 |
| `GESTURE_FINAL_STILL_RMS_MS2` | 2.0 m/s² | 静止進入 RMS 上限 |
| `GESTURE_FINAL_HOLD_MS` | 500 ms | 最終静止時間 |
| `GESTURE_FINAL_HOLD_TIMEOUT_MS` | 5000 ms | 掌上後の最終成立期限 |
| `GESTURE_SEQUENCE_TIMEOUT_MS` | 9000 ms | 全シーケンス期限 |
| `GESTURE_RETRIGGER_BLOCK_MS` | 1200 ms | 開始直後の停止抑制 / 停止後の再開始抑制 |

実装変更時は `docs/flex_pronation_gesture.md` と
`docs/nordic_main_guide.md` の値も同時に更新する。

## 診断と validator

`mac_client/gesture_validator.py` は BLE 診断を条件別に表示し、JSONへ保存する。
主なイベント:

- `outbound_start`: シェイク峰間、Z 比、平均
- `outbound_ready`: phi、3D 角、Δz 比
- `final_sample`: pulse stage、`a_up`、インパルス
- `final_hold_start`: 正負インパルス、静止開始時の姿勢差
- `final_ready`: 正インパルス、hold 時間、姿勢差
- `match`: 掌上角、正インパルス、hold 時間
- `stop_palm_up`: 録音中掌上停止。phi、3D 角、Δz
- `wait_reject` reason `shake_not_oscillatory`: 直流横 G
- `reset` reason `lift_palm_still_up`: 挙上後に掌上基準から反転できず期限切れ
- `wait_reject`（パルス再試行）: 減速不足、比率不足、パルス時間不足。一連動作は継続
- `reset`: 掌上失敗、全体/最終期限切れ、掌上のまま hold、姿勢・hold 失敗。シェイクからやり直し

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
掌上のまま上げる、持続横 G、歩行で no-match を確認する。車載振動は安全に
実施できる場合だけ追加する。停止は手下ろしではなく掌上で確認する。

2026-08-20 の確認結果（`0.0.47`、slot0 active+confirmed）:

- validator self-test: PASS
- `xiao_ble/nrf52840/sense` + sysbuild: PASS
- 正例（シェイク→掌上→挙上→掌下静止→掌上停止）: 3/3 PASS
  - 開始 5.5–6.3 s。停止は開始姿勢からの緩い phi（9.7–15.2°）。手下ろしでは止めていない

## 保守上の注意

- ジャイロは省電力方針により無効。再有効化は要件確認後に行う。
- 単一閾値を下げるだけで合格率を上げない。シェイクの平均比と掌下 hold（20°）を維持する。
- シェイク後の掌上だけ 3D 傾きと緩い phi を使う。hold の反転ゲートは共有しない。
- 掌上基準はシェイク窓開始時の重力 LP。成立瞬間の raw では武装しない。
- 重力 LP はパルス開始まで追従し、hold の姿勢基準は静止進入時に固定する。
- シェイク成立後に重力 LP をリセットしない。
- 録音停止の基準は開始時の重力 LP。`reset_gesture_sequence()` では消さない。
- 録音停止は開始姿勢からの緩い掌上反転。手下ろしと 3軸 `motion_active` では止めない。
- 掌上/掌下は装着 Z 符号に依存させず、シェイク基準と掌上基準からの相対変化で見る。
- 最終静止の RMS 上限は運転中の振動を見込んで 2.0 m/s²。500 ms の継続は維持する。
- 減速パルス不足は `wait_reject` の再試行であり、`reset` ではない。
- 実機ログではシェイク峰間/平均、掌上基準からの phi/Δz、正負インパルス、姿勢差、hold 時間、reset / wait_reject reasonを確認する。

## 関連ファイル

- `nordic-main/src/main.c`: 状態機械、閾値、BLE 診断
- `mac_client/gesture_validator.py`: BLE 試験、診断表示、JSON 保存
- `docs/flex_pronation_gesture.md`: ジェスチャ仕様
- `docs/nordic_main_guide.md`: ビルド、OTA、運用
