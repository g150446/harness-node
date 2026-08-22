# 腕上げ検出の保守ガイド

## 概要

`nordic-main` の録音開始ジェスチャは、手の甲側に装着した XIAO
nRF52840 Sense の **加速度（常時）** と **ジャイロ（オンデマンド）** を使用する。

意図する操作:

1. 掌下で短く 1 回シェイクする（成立時にジャイロ ON）
2. 手のひらを上向きにする（重力 OR gyro_y）
3. 手を上げる（掌向きは問わない）
4. 掌を下にして 400 ms 静止
5. 録音中に手のひらを上へ向けると録音終了（手下ろしでは終了しない。重力 OR gyro_y）
6. 録音終了でジャイロ OFF

物理的な上昇距離は操作の目安であり、判定では変位を積分しない。

## 現行アルゴリズム

`nordic-main/src/main.c` の `process_gesture_sample()` は、次の順で判定する。

1. `GESTURE_WAITING`
   - `|Z 比| ≥ 0.80` で基板が水平なときだけ、重力直交の線形加速度をシェイク窓へ入れる。
   - 500 ms 窓の峰間と平均比で 1 回の短い振りを確認し、持続横 G を落とす。
   - 成立時 `gyro_set_enabled(true)`（104 Hz / ±500 dps）。
2. `GESTURE_OUTBOUND`
   - シェイク窓開始時の重力 LP 基準から、3D 角 / phi / Δz / Z 符号 **または** gyro_y で掌上成立。
   - 成立時に phi / az を掌上基準として取り直し、outbound の gyro 符号を記憶。未完了のまま 1500 ms で reset。
3. `GESTURE_HOLDING_FINAL`
   - `WAIT_ACCEL` / `WAIT_BRAKE`: `a_up` パルス。掌向きは見ない。
   - `WAIT_HOLD`: 掌上基準からの反転（重力 OR 逆符号 gyro_y）後、RMS 静止と 400 ms hold。
   - hold **進入時のみ** `|ω_y| ≤ 90 dps`。進入後は RMS + 姿勢差で維持。
   - 掌上後 5000 ms 以内に成立しなければ `final_hold_timeout`。
4. 録音中
   - 開始時の重力 LP を基準に保存し、掌上反転（重力 OR gyro_y）で録音停止。
   - 手下ろしでは止めない。`motion_active` でも止めない。停止後ジャイロ OFF。

上向き加速と逆向き減速は、連続サンプル数、正負インパルス、
減速/加速インパルス比、150–1800 ms の時間窓を組み合わせて判定する。

## 主要閾値

| 定数 | 値 | 用途 |
|------|---:|------|
| `GESTURE_START_PALM_UP_Z_MIN_RATIO` | 0.80 | 開始時の基板水平（\|Z 比\|） |
| `GESTURE_SHAKE_WINDOW_SAMPLES` | 20 | シェイク窓（約 500 ms） |
| `GESTURE_SHAKE_PTP_MIN_MS2` | 5.0 m/s² | 重力直交成分の峰間 |
| `GESTURE_SHAKE_MEAN_RATIO_MAX` | 0.4 | \|平均\| / 峰間 の上限 |
| `GESTURE_SHAKE_AXIS_MIN_MS2` | 2.0 m/s² | シェイク軸を固定する直交成分下限 |
| `GYRO_ODR_HZ` / `GYRO_FULL_SCALE_DPS` | 104 / 500 | オンデマンドジャイロ |
| `GESTURE_GYRO_SETTLE_MS` | 100 ms | ジャイロ整定 |
| `GESTURE_OUTBOUND_GYRO_ANGLE_MIN_DEG` | 25° | 掌上/停止の ∫ω_y |
| `GESTURE_OUTBOUND_GYRO_PEAK_DPS` | 35 dps | 掌上/停止の peak \|ω_y\| |
| `GESTURE_HOLD_GYRO_ANGLE_MIN_DEG` | 20° | hold 反転の ∫ω_y |
| `GESTURE_HOLD_GYRO_PEAK_DPS` | 30 dps | hold 反転の peak |
| `GESTURE_FINAL_QUIET_RATE_DPS` | 90 dps | hold **進入時のみ** |
| `GESTURE_PRONATION_MIN_DEG` | 20° | hold 反転角（phi）。Z 比 0.50 または符号反転でも成立 |
| `GESTURE_PRONATION_Z_RATIO_DONE` | 0.50 | hold 反転の Z 比変化 |
| `GESTURE_OUTBOUND_MIN_DEG` | 12° | シェイク後掌上の XZ phi |
| `GESTURE_OUTBOUND_TILT_MIN_DEG` | 20° | シェイク後掌上の重力 3D 角 |
| `GESTURE_OUTBOUND_Z_RATIO_DONE` | 0.35 | シェイク後掌上の Z 比変化 |
| `GESTURE_OUTBOUND_MAX_DURATION_MS` | 1500 ms | シェイク後の掌上期限 |
| `GESTURE_LIFT_ACCEL_MIN_MS2` | 0.40 m/s² | 上向き加速パルス下限 |
| `GESTURE_LIFT_POS_IMPULSE_MIN_MS` | 0.04 m/s | 正インパルス下限 |
| `GESTURE_LIFT_PULSE_MIN_MS` / `MAX_MS` | 150 / 1800 ms | パルス時間窓 |
| `GESTURE_LIFT_FINAL_TILT_MAX_DEG` | 15° | hold 中の姿勢差上限 |
| `GESTURE_FINAL_STILL_RMS_MS2` | 3.0 m/s² | 静止進入 RMS 上限 |
| `GESTURE_FINAL_HOLD_MS` | 400 ms | 最終静止時間 |
| `GESTURE_FINAL_HOLD_TIMEOUT_MS` | 5000 ms | 掌上後の最終成立期限 |
| `GESTURE_SEQUENCE_TIMEOUT_MS` | 6000 ms | 全シーケンス期限 |
| `GESTURE_RETRIGGER_BLOCK_MS` | 1200 ms | 開始直後の停止抑制 / 停止後の再開始抑制 |

実装変更時は `docs/flex_pronation_gesture.md` と
`docs/nordic_main_guide.md` の値も同時に更新する。

## 診断と validator

`mac_client/gesture_validator.py` は BLE 診断を条件別に **実測と閾値** を並べて表示し、JSONへ保存する。
主なイベント:

- `outbound_start`: シェイク峰間、Z 比、平均
- `outbound_ready`: phi、3D 角、Δz 比
- `outbound_gyro`: ∫ω_y、peak dps、sign（掌上時と停止時）
- `gyro_enabled` / `gyro_disabled`
- `final_sample` / `hold_sample`
- `final_hold_start` / `final_ready` / `match`
- `stop_palm_up` + 直後の `outbound_gyro`（停止は重力 OR gyro）
- `wait_reject` reason `final_hold_interrupted`: RMS / tilt / \|gy\|
- `reset` reason `final_hold_timeout` など

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

2026-08-22 の確認結果（`0.0.52`、slot0 active+confirmed）:

- validator self-test: PASS
- `xiao_ble/nrf52840/sense` + sysbuild: PASS
- 正例（1 回ずつ）: 開始〜掌上停止の完全合格を確認（例: 4.1 s / 9.3 s）
- hold 期限 5 s、静止 400 ms / RMS 3.0 / tilt 15°、進入時 gyro quiet 90 dps
- 掌上・停止は gyro_y peak で成立するケースあり（validator は重力 OR gyro を表示）

## 保守上の注意

- ジャイロはオンデマンド（シェイク成立〜録音終了）。待機時は ODR=0。閾値変更時は重力 OR ジャイロの両方を確認する。
- hold 中の gyro quiet は **進入時のみ**。進入後に \|ω_y\| で中断しない。
- 単一閾値を下げるだけで合格率を上げない。シェイクの平均比と掌下 hold（20°）を維持する。
- 運転誤検出対策として掌上窓 1.5 s・掌上重力ゲート（phi 12° / 3D 20° / Δz 0.35）を厳しめに保つ。
- 掌上基準はシェイク窓開始時の重力 LP。成立瞬間の raw では武装しない。
- 重力 LP はパルス開始まで追従し、hold の姿勢基準は静止進入時に固定する。
- 録音停止の基準は開始時の重力 LP。`reset_gesture_sequence()` では消さない。
- 録音停止は開始姿勢からの掌上（重力 OR gyro）。手下ろしと 3軸 `motion_active` では止めない。
- 最終静止の RMS 上限は 3.0 m/s²、継続 400 ms、掌上後期限 5 s。
- 減速パルス不足は `wait_reject` の再試行であり、`reset` ではない。
- 実機ログではシェイク峰間/平均、掌上の phi/Δz/∫ω_y/peak、正負インパルス、姿勢差、hold 時間、reset reason を確認する。

## 関連ファイル

- `nordic-main/src/main.c`: 状態機械、閾値、BLE 診断
- `mac_client/gesture_validator.py`: BLE 試験、診断表示、JSON 保存
- `docs/flex_pronation_gesture.md`: ジェスチャ仕様
- `docs/nordic_main_guide.md`: ビルド、OTA、運用
