#!/usr/bin/env python3
"""HarnessNode のシェイク→掌上→挙上→掌下静止をBLEで対話検証する。

人が画面のカウントダウンに合わせて動作し、ファームウェアから届く
``recording_start`` イベント (0x01) の有無を試行ごとに判定する。
録音終了は開始時の掌下姿勢からの緩い掌上反転であり、手を下ろすだけでは
``recording_stop`` しない。ホスト ``0x00`` による停止は従来どおり。

画面には試行ごとの判定条件だけを読みやすく表示し、生の診断イベントはJSON
レポートだけに保存する。標準入力は使わない。

Examples:
    venv/bin/python gesture_validator.py --trials 3
    venv/bin/python gesture_validator.py --expect no-match --trials 3 \
        --instruction "水平姿勢のまま手を動かしてください"
    venv/bin/python gesture_validator.py --self-test
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import struct
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional


DEVICE_NAME = "HarnessNode"
DEFAULT_CUE_SOUND = "/System/Library/Sounds/Ping.aiff"
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"
AUDIO_RX_UUID = "00000003-0000-1000-8000-00805f9b34fb"

SYNC0 = 0x00
SYNC1 = 0x55
EVT_RECORDING_START = 0x01
EVT_RECORDING_STOP = 0x02
EVT_MOTION_ACTIVE = 0x10
EVT_MOTION_SETTLED = 0x11
EVT_SLEEP_ENTER = 0x20
EVT_SLEEP_WAKE = 0x21
EVT_GESTURE_DIAG = 0x30
EVT_DISCONNECTED = -1

EVENT_NAMES = {
    EVT_RECORDING_START: "recording_start",
    EVT_RECORDING_STOP: "recording_stop",
    EVT_MOTION_ACTIVE: "motion_active",
    EVT_MOTION_SETTLED: "motion_settled",
    EVT_SLEEP_ENTER: "sleep_enter",
    EVT_SLEEP_WAKE: "sleep_wake",
    EVT_GESTURE_DIAG: "gesture_diag",
    EVT_DISCONNECTED: "disconnected",
}

DIAG_STAGE_NAMES = {
    0x01: "outbound_start",
    0x02: "outbound_ready",
    0x03: "turnaround_ready",
    0x04: "return_start",
    0x05: "return_ready",
    0x06: "distance_ready",
    0x07: "final_hold_start",
    0x08: "final_ready",
    0x09: "match",
    0x0A: "gyro_bias_ready",
    0x0C: "stop_palm_up",
    0x20: "gyro_y_sample",
    0x21: "final_sample",
    0x10: "wait_reject",
    0x80: "reset",
}

DIAG_REASON_NAMES = {
    0x00: "none",
    0x01: "quiet_not_ready",
    0x02: "start_not_palm_up",
    0x03: "outbound_rate_low",
    0x11: "outbound_timeout",
    0x12: "incomplete_outbound",
    0x1A: "final_hold_interrupted",
    0x1B: "final_hold_timeout",
    0x1C: "sequence_timeout",
    0x1D: "final_accel_missing",
    0x1E: "final_brake_missing",
    0x1F: "final_brake_ratio_low",
    0x20: "final_tilt_unstable",
    0x21: "final_pulse_duration_invalid",
    0x22: "shake_not_oscillatory",
    0x23: "lift_palm_still_up",
}

START_BOARD_FLAT_Z_MIN_RATIO = 0.80
START_PALM_UP_Z_MIN_RATIO = START_BOARD_FLAT_Z_MIN_RATIO
START_QUIET_HOLD_MS = 50
START_QUIET_ACCEL_MAX_MS2 = 3.0
SHAKE_PTP_MIN_MS2 = 5.0
SHAKE_MEAN_RATIO_MAX = 0.4
# Outbound palm-up (hold flip stays 20° / 0.50 in firmware).
PRONATION_ANGLE_MIN_DEG = 8.0
PRONATION_TILT_MIN_DEG = 15.0
PRONATION_START_DEG = 8.0
PRONATION_Z_RATIO_START = 0.25
PRONATION_Z_RATIO_DONE = 0.25
HOLD_PRONATION_ANGLE_MIN_DEG = 20.0
HOLD_PRONATION_Z_RATIO_DONE = 0.50
# Final: upward acceleration pulse, braking pulse, stable pose, then hold.
FINAL_POS_IMPULSE_MIN_MS = 0.04
FINAL_NEG_IMPULSE_MIN_MS = 0.015
FINAL_BRAKE_RATIO_MIN = 0.05
FINAL_TILT_MAX_DEG = 10.0
FINAL_HOLD_MIN_MS = 500


@dataclass(frozen=True)
class GestureEvent:
    code: int
    name: str
    monotonic_s: float
    wall_time: str
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    elapsed_ms: Optional[int] = None
    diag_stage: Optional[int] = None
    diag_reason: Optional[int] = None
    value1: Optional[float] = None
    value2: Optional[float] = None
    value3: Optional[float] = None


@dataclass(frozen=True)
class ConditionResult:
    label: str
    status: str
    detail: str


@dataclass
class TrialResult:
    trial: int
    expected: str
    result: str
    matched: bool
    latency_ms: Optional[int]
    motion_active_seen: bool
    motion_settled_seen: bool
    sequence_reset_seen: bool
    reason: str
    diagnostics: list[dict[str, Any]]
    conditions: list[ConditionResult]


def gesture_gate_eligible(
    board_flat_z_ratio: float,
    palm_up_deg: float,
    positive_impulse_ms: float,
    negative_impulse_ms: float,
    final_tilt_deg: float,
    final_hold_ms: int,
    z_ratio_delta: float = 0.0,
    z_sign_flip: bool = False,
    shake_ptp_ms2: float = SHAKE_PTP_MIN_MS2,
    shake_mean_ms2: float = 0.0,
    palm_up_tilt_deg: float = 0.0,
) -> bool:
    """Mirror the firmware's shake, outbound palm-up, lift-pulse and hold gates.

    Outbound palm-up uses PRONATION_ANGLE_MIN_DEG / PRONATION_TILT_MIN_DEG /
    PRONATION_Z_RATIO_DONE. Hold flip in firmware remains
    HOLD_PRONATION_ANGLE_MIN_DEG / HOLD_PRONATION_Z_RATIO_DONE.
    """
    del negative_impulse_ms
    shake_ok = (
        shake_ptp_ms2 >= SHAKE_PTP_MIN_MS2
        and abs(shake_mean_ms2) < SHAKE_MEAN_RATIO_MAX * max(shake_ptp_ms2, 1e-6)
    )
    palm_up_ok = (
        abs(palm_up_deg) >= PRONATION_ANGLE_MIN_DEG
        or palm_up_tilt_deg >= PRONATION_TILT_MIN_DEG
        or z_ratio_delta >= PRONATION_Z_RATIO_DONE
        or z_sign_flip
    )
    return (
        board_flat_z_ratio >= START_BOARD_FLAT_Z_MIN_RATIO
        and shake_ok
        and palm_up_ok
        and positive_impulse_ms >= FINAL_POS_IMPULSE_MIN_MS
        and final_tilt_deg <= FINAL_TILT_MAX_DEG
        and final_hold_ms >= FINAL_HOLD_MIN_MS
    )


def recording_stop_palm_up_eligible(
    palm_up_deg: float,
    palm_up_tilt_deg: float = 0.0,
    z_ratio_delta: float = 0.0,
    z_sign_flip: bool = False,
) -> bool:
    """Mirror the firmware's recording-stop palm-up gates (same as outbound)."""
    return (
        abs(palm_up_deg) >= PRONATION_ANGLE_MIN_DEG
        or palm_up_tilt_deg >= PRONATION_TILT_MIN_DEG
        or z_ratio_delta >= PRONATION_Z_RATIO_DONE
        or z_sign_flip
    )


def sequence_reset_seen(diagnostics: list[dict[str, Any]]) -> bool:
    """Return whether the one-action trial reset before its eventual match."""
    return any(record.get("stage") == "reset" for record in diagnostics)


def _latest_diagnostic(
    diagnostics: list[dict[str, Any]],
    *,
    stage: Optional[str] = None,
    reasons: tuple[str, ...] = (),
) -> Optional[dict[str, Any]]:
    for record in reversed(diagnostics):
        if stage is not None and record.get("stage") != stage:
            continue
        if reasons and record.get("reason") not in reasons:
            continue
        return record
    return None


def build_condition_results(
    diagnostics: list[dict[str, Any]], matched: bool
) -> list[ConditionResult]:
    """Turn raw diagnostics into the firmware gate checklist shown per trial."""
    outbound_start = _latest_diagnostic(diagnostics, stage="outbound_start")
    outbound_ready = _latest_diagnostic(diagnostics, stage="outbound_ready")
    final_start = _latest_diagnostic(diagnostics, stage="final_hold_start")
    final_ready = _latest_diagnostic(diagnostics, stage="final_ready")
    wait_quiet = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("quiet_not_ready",),
    )
    wait_pose = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("start_not_palm_up",),
    )
    wait_shake = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("shake_not_oscillatory",),
    )
    outbound_reset = _latest_diagnostic(
        diagnostics,
        stage="reset",
        reasons=(
            "outbound_timeout",
            "incomplete_outbound",
        ),
    )
    final_reset = _latest_diagnostic(
        diagnostics,
        stage="reset",
        reasons=(
            "final_accel_missing",
            "final_brake_missing",
            "final_brake_ratio_low",
            "final_pulse_duration_invalid",
            "final_tilt_unstable",
            "final_hold_timeout",
            "lift_palm_still_up",
        ),
    )
    pulse_retry = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=(
            "final_brake_missing",
            "final_brake_ratio_low",
            "final_pulse_duration_invalid",
        ),
    )

    conditions: list[ConditionResult] = []

    # wait_reject encoding (firmware accel-only):
    #   start_not_palm_up: v1=z_ratio, v2=min_ratio, v3=linear_accel (board not flat)
    #   quiet_not_ready: v1=z_ratio, v2=shake_count, v3=need_samples
    #   shake_not_oscillatory: v1=ptp, v2=mean, v3=|mean|/ptp
    pose_fail_detail: Optional[str] = None
    shake_fail_detail: Optional[str] = None
    best_z = -1.0
    best_ptp = -1.0
    worst_mean_ratio = -1.0
    shake_samples = -1.0

    for rec in diagnostics:
        if rec.get("stage") != "wait_reject":
            continue
        reason = rec.get("reason")
        v1 = float(rec.get("value1") or 0.0)
        v2 = float(rec.get("value2") or 0.0)
        v3 = float(rec.get("value3") or 0.0)
        if reason == "start_not_palm_up":
            best_z = max(best_z, v1)
            thr = v2 if v2 > 0.1 else START_BOARD_FLAT_Z_MIN_RATIO
            pose_fail_detail = (
                f"Z絶対比 {v1:.2f} < {thr:.2f}"
                f"（線形加速度 {v3:.2f} m/s²）"
            )
        elif reason == "quiet_not_ready":
            best_z = max(best_z, v1)
            shake_samples = max(shake_samples, v2)
            shake_fail_detail = (
                f"シェイク窓 {v2:.0f}/{v3:.0f} サンプル"
                f"（Z絶対比 {v1:.2f}）"
            )
        elif reason == "shake_not_oscillatory":
            best_ptp = max(best_ptp, v1)
            worst_mean_ratio = max(worst_mean_ratio, v3)
            shake_fail_detail = (
                f"峰間 {v1:.2f} m/s²、平均 {v2:.2f} m/s²"
                f"（要 峰間 ≥ {SHAKE_PTP_MIN_MS2:.1f} かつ "
                f"|平均| < {SHAKE_MEAN_RATIO_MAX:.1f}×峰間）"
            )

    if outbound_start is not None:
        ptp = float(outbound_start["value1"])
        z_ratio = float(outbound_start["value2"])
        mean = float(outbound_start["value3"])
        shake_ok = (
            ptp >= SHAKE_PTP_MIN_MS2
            and abs(mean) < SHAKE_MEAN_RATIO_MAX * max(ptp, 1e-6)
            and z_ratio >= START_BOARD_FLAT_Z_MIN_RATIO
        )
        conditions.append(
            ConditionResult(
                "掌下の短いシェイク",
                "PASS" if shake_ok else "FAIL",
                f"峰間 {ptp:.2f} m/s²、平均 {mean:.2f} m/s²、"
                f"Z絶対比 {z_ratio:.2f}",
            )
        )
    elif wait_shake is not None or wait_quiet is not None or wait_pose is not None:
        if best_z < 0.0 and wait_pose is not None:
            best_z = float(wait_pose.get("value1") or 0.0)
        if best_z < 0.0 and wait_quiet is not None:
            best_z = float(wait_quiet.get("value1") or 0.0)
        if pose_fail_detail:
            detail = pose_fail_detail
        elif shake_fail_detail:
            detail = shake_fail_detail
        else:
            detail = "シェイク未成立"
        conditions.append(
            ConditionResult(
                "掌下の短いシェイク",
                "FAIL",
                detail,
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "掌下の短いシェイク",
                "NOT_REACHED",
                "判定データなし",
            )
        )

    if outbound_ready is not None:
        phi = float(outbound_ready["value1"])
        tilt = abs(float(outbound_ready["value2"]))
        z_delta = abs(float(outbound_ready.get("value3") or 0.0))
        phi_ok = (
            abs(phi) >= PRONATION_ANGLE_MIN_DEG
            or tilt >= PRONATION_TILT_MIN_DEG
            or z_delta >= PRONATION_Z_RATIO_DONE
        )
        conditions.append(
            ConditionResult(
                "掌上への反転",
                "PASS" if phi_ok else "FAIL",
                f"phi {phi:.1f}°、3D {tilt:.1f}°、"
                f"Δz比 {z_delta:.2f}"
                f"（要 phi ≥ {PRONATION_ANGLE_MIN_DEG:.0f}° または "
                f"3D ≥ {PRONATION_TILT_MIN_DEG:.0f}° または "
                f"Δz ≥ {PRONATION_Z_RATIO_DONE:.2f}）",
            )
        )
    elif outbound_reset is not None:
        conditions.append(
            ConditionResult(
                "掌上への反転",
                "FAIL",
                f"phi {float(outbound_reset['value1']):+.1f}°、"
                f"3D {float(outbound_reset.get('value2') or 0.0):.1f}°、"
                f"Δz {float(outbound_reset.get('value3') or 0.0):.2f} "
                f"({outbound_reset['reason']})",
            )
        )
    elif outbound_start is not None:
        conditions.append(
            ConditionResult("掌上への反転", "FAIL", "掌上完了条件まで到達せず")
        )
    else:
        conditions.append(
            ConditionResult("掌上への反転", "NOT_REACHED", "シェイク後の掌上を検出せず")
        )

    # final_hold_start: v1=positive impulse, v2=negative impulse, v3=tilt
    # final_ready:      v1=positive impulse, v2=hold_ms, v3=tilt
    pulse_src = final_ready or final_start
    if pulse_src is not None:
        pos_impulse = float(pulse_src["value1"])
        neg_impulse = (
            float(final_start["value2"])
            if final_start is not None
            else FINAL_NEG_IMPULSE_MIN_MS
        )
        pulse_ok = pos_impulse >= FINAL_POS_IMPULSE_MIN_MS
        brake_ratio = neg_impulse / pos_impulse if pos_impulse > 0.0 else 0.0
        conditions.append(
            ConditionResult(
                "挙上",
                "PASS" if pulse_ok else "FAIL",
                f"加速 {pos_impulse:.3f} m/s ≥ {FINAL_POS_IMPULSE_MIN_MS:.2f}、"
                f"減速 {neg_impulse:.3f} m/s"
                + (
                    f"、比率 {brake_ratio:.2f}"
                    if pos_impulse > 0.0
                    else ""
                ),
            )
        )
    elif final_reset is not None and final_reset["reason"] in (
        "final_tilt_unstable",
        "final_hold_timeout",
        "lift_palm_still_up",
    ):
        pos_impulse = float(final_reset.get("value1") or 0.0)
        neg_impulse = float(final_reset.get("value2") or 0.0)
        conditions.append(
            ConditionResult(
                "挙上",
                "PASS",
                f"加速 {pos_impulse:.3f} m/s、減速 {neg_impulse:.3f} m/s（成立済み）",
            )
        )
    elif final_reset is not None:
        reason = str(final_reset["reason"])
        pos_impulse = float(final_reset.get("value1") or 0.0)
        neg_impulse = float(final_reset.get("value2") or 0.0)
        if reason == "final_brake_ratio_low":
            detail = (
                f"減速/加速比 {float(final_reset.get('value3') or 0.0):.2f} "
                f"< {FINAL_BRAKE_RATIO_MIN:.2f}"
            )
        elif reason == "final_pulse_duration_invalid":
            detail = (
                f"パルス時間 {pos_impulse:.0f} ms "
                "（許容 150–1800 ms）"
            )
        else:
            detail = (
                f"{reason}: 加速 {pos_impulse:.3f} m/s、"
                f"減速 {neg_impulse:.3f} m/s"
            )
        conditions.append(
            ConditionResult(
                "挙上",
                "FAIL",
                detail,
            )
        )
    elif pulse_retry is not None:
        reason = str(pulse_retry["reason"])
        pos_impulse = float(pulse_retry.get("value1") or 0.0)
        neg_impulse = float(pulse_retry.get("value2") or 0.0)
        if reason == "final_brake_ratio_low":
            detail = (
                f"減速/加速比 {float(pulse_retry.get('value3') or 0.0):.2f} "
                f"< {FINAL_BRAKE_RATIO_MIN:.2f}（パルス再試行）"
            )
        elif reason == "final_pulse_duration_invalid":
            detail = (
                f"パルス時間 {pos_impulse:.0f} ms "
                "（許容 150–1800 ms、パルス再試行）"
            )
        else:
            detail = (
                f"{reason}: 加速 {pos_impulse:.3f} m/s、"
                f"減速 {neg_impulse:.3f} m/s（パルス再試行）"
            )
        conditions.append(
            ConditionResult(
                "挙上",
                "FAIL",
                detail,
            )
        )
    elif outbound_ready is not None:
        conditions.append(
            ConditionResult(
                "挙上",
                "FAIL",
                "掌上後に必要な挙上が成立せず",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "挙上",
                "NOT_REACHED",
                "挙上判定まで到達せず",
            )
        )

    tilt_src = final_ready or final_start
    if final_reset is not None and final_reset["reason"] == "final_tilt_unstable":
        tilt_deg = float(final_reset.get("value3") or 0.0)
        conditions.append(
            ConditionResult(
                "上昇後の姿勢安定",
                "FAIL",
                f"静止開始姿勢から {tilt_deg:.1f}° > {FINAL_TILT_MAX_DEG:.1f}°",
            )
        )
    elif tilt_src is not None:
        tilt_deg = float(tilt_src["value3"])
        conditions.append(
            ConditionResult(
                "上昇後の姿勢安定",
                "PASS" if tilt_deg <= FINAL_TILT_MAX_DEG else "FAIL",
                f"静止開始姿勢から {tilt_deg:.1f}° ≤ {FINAL_TILT_MAX_DEG:.1f}°",
            )
        )
    elif final_reset is not None and final_reset["reason"] == "final_hold_timeout":
        tilt_deg = float(final_reset.get("value3") or 0.0)
        conditions.append(
            ConditionResult(
                "上昇後の姿勢安定",
                "PASS",
                f"静止開始姿勢から {tilt_deg:.1f}° ≤ {FINAL_TILT_MAX_DEG:.1f}°",
            )
        )
    else:
        conditions.append(
            ConditionResult("上昇後の姿勢安定", "NOT_REACHED", "パルス成立前")
        )

    if final_ready is not None:
        hold_ms = float(final_ready["value2"])
        conditions.append(
            ConditionResult(
                "掌下で静止",
                "PASS",
                f"{hold_ms:.0f} ms ≥ {FINAL_HOLD_MIN_MS} ms",
            )
        )
    elif final_reset is not None and final_reset["reason"] == "lift_palm_still_up":
        conditions.append(
            ConditionResult(
                "掌下で静止",
                "FAIL",
                "掌上基準から掌下へ回せなかった",
            )
        )
    elif final_start is not None:
        conditions.append(
            ConditionResult(
                "掌下で静止",
                "FAIL",
                f"{FINAL_HOLD_MIN_MS} msの連続静止を確認できず",
            )
        )
    elif final_reset is not None and final_reset["reason"] == "final_hold_timeout":
        conditions.append(
            ConditionResult(
                "掌下で静止",
                "FAIL",
                f"{FINAL_HOLD_MIN_MS} msの静止を開始できずタイムアウト",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "掌下で静止", "NOT_REACHED", "上昇が未成立"
            )
        )

    conditions.append(
        ConditionResult(
            "1回の動作で完結",
            "FAIL" if sequence_reset_seen(diagnostics) else "PASS",
            (
                "途中で判定がリセットされた"
                if sequence_reset_seen(diagnostics)
                else "途中の判定リセットなし"
            ),
        )
    )
    conditions.append(
        ConditionResult(
            "ジェスチャー発動",
            "PASS" if matched else "FAIL",
            "recording_startを受信" if matched else "recording_startなし",
        )
    )
    return conditions


def print_condition_results(conditions: list[ConditionResult]) -> None:
    markers = {"PASS": "[OK]", "FAIL": "[NG]", "NOT_REACHED": "[--]"}
    print("  判定条件:", flush=True)
    for condition in conditions:
        print(
            f"    {markers[condition.status]} {condition.label}: {condition.detail}",
            flush=True,
        )


def summarize_gyro_y_samples(diagnostics: list[dict[str, Any]]) -> None:
    """Print a short summary of debug gyro_y_sample stream if present."""
    samples = [r for r in diagnostics if r.get("stage") == "gyro_y_sample"]
    if not samples:
        return
    ys = [float(r.get("value1") or 0.0) for r in samples]
    print(
        f"  gyro_y debug: n={len(samples)} "
        f"min={min(ys):+.1f} max={max(ys):+.1f} "
        f"last={ys[-1]:+.1f} dps",
        flush=True,
    )


def write_gyro_csv(path: str, diagnostics: list[dict[str, Any]]) -> None:
    samples = [r for r in diagnostics if r.get("stage") == "gyro_y_sample"]
    if not samples:
        return
    lines = ["elapsed_ms,gyro_y_dps,y_ratio,time"]
    for r in samples:
        lines.append(
            f"{float(r.get('value3') or 0.0):.0f},"
            f"{float(r.get('value1') or 0.0):.3f},"
            f"{float(r.get('value2') or 0.0):.4f},"
            f"{r.get('time') or ''}"
        )
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"  gyro CSV: {path} ({len(samples)} samples)", flush=True)


def parse_event_packet(data: bytes, now: Optional[float] = None) -> Optional[GestureEvent]:
    """BLE notificationからイベントだけを抽出する。音声パケットは無視する。"""
    if len(data) < 3 or data[0] != SYNC0 or data[1] != SYNC1:
        return None

    code = data[2]
    event = GestureEvent(
        code=code,
        name=EVENT_NAMES.get(code, f"unknown_0x{code:02x}"),
        monotonic_s=time.monotonic() if now is None else now,
        wall_time=datetime.now().isoformat(timespec="milliseconds"),
    )

    if code in (EVT_MOTION_ACTIVE, EVT_MOTION_SETTLED) and len(data) >= 15:
        x, y, z = struct.unpack_from("<fff", data, 3)
        elapsed_ms = None
        if code == EVT_MOTION_SETTLED and len(data) >= 19:
            elapsed_ms = struct.unpack_from("<I", data, 15)[0]
        event = GestureEvent(
            code=code,
            name=event.name,
            monotonic_s=event.monotonic_s,
            wall_time=event.wall_time,
            x=x,
            y=y,
            z=z,
            elapsed_ms=elapsed_ms,
        )
    elif code == EVT_GESTURE_DIAG and len(data) >= 17:
        stage = data[3]
        reason = data[4]
        value1, value2, value3 = struct.unpack_from("<fff", data, 5)
        event = GestureEvent(
            code=code,
            name=event.name,
            monotonic_s=event.monotonic_s,
            wall_time=event.wall_time,
            diag_stage=stage,
            diag_reason=reason,
            value1=value1,
            value2=value2,
            value3=value3,
        )
    return event


def format_event(event: GestureEvent) -> str:
    ts = event.wall_time[11:23]
    if event.code == EVT_GESTURE_DIAG:
        stage = DIAG_STAGE_NAMES.get(
            event.diag_stage or 0, f"stage_0x{(event.diag_stage or 0):02x}"
        )
        reason = DIAG_REASON_NAMES.get(
            event.diag_reason or 0, f"reason_0x{(event.diag_reason or 0):02x}"
        )
        v1 = event.value1 or 0.0
        v2 = event.value2 or 0.0
        v3 = event.value3 or 0.0
        if stage == "outbound_start":
            suffix = (
                f" shake_ptp={v1:.2f}m/s^2 z={v2:.2f} "
                f"mean={v3:.2f}m/s^2"
            )
        elif stage == "outbound_ready":
            suffix = (
                f" pronation={v1:+.1f}deg tilt_3d={v2:.1f}deg "
                f"z_delta={v3:.2f}"
            )
        elif stage == "stop_palm_up":
            suffix = (
                f" pronation={v1:+.1f}deg tilt_3d={v2:.1f}deg "
                f"z_delta={v3:.2f}"
            )
        elif stage == "turnaround_ready":
            suffix = (
                f" elapsed={v1:.0f}ms gyro_y={v2:+.1f}dps y={v3:+.2f}"
            )
        elif stage == "return_start":
            suffix = (
                f" roll_rate={v1:.1f}dps gyro_y={v2:+.1f}dps "
                f"outbound_roll={v3:+.1f}deg"
            )
        elif stage == "return_ready":
            suffix = (
                f" supination={v1:+.1f}deg peak={v2:.1f}dps "
                f"final_y={v3:+.2f}"
            )
        elif stage == "final_sample":
            suffix = (
                f" pulse_stage={v1:.0f} a_up={v2:+.2f}m/s^2 "
                f"net_impulse={v3:+.3f}m/s"
            )
        elif stage == "final_hold_start":
            suffix = (
                f" positive_impulse={v1:.3f}m/s "
                f"negative_impulse={v2:.3f}m/s tilt={v3:.1f}deg"
            )
        elif stage == "match":
            suffix = (
                f" pronation={v1:+.1f}deg positive_impulse={v2:.3f}m/s "
                f"hold={v3:.0f}ms"
            )
        elif stage == "distance_ready":
            suffix = (
                f" fused={v1:.1f}cm accel={v2:.1f}cm arc={v3:.1f}cm"
            )
        elif stage == "final_ready":
            suffix = (
                f" positive_impulse={v1:.3f}m/s hold={v2:.0f}ms "
                f"tilt={v3:.1f}deg"
            )
        elif stage == "gyro_bias_ready":
            suffix = (
                f" correction={v1:.1f}dps plane_bias={v2:.1f}dps "
                f"z_bias={v3:.1f}dps"
            )
        elif stage == "gyro_y_sample":
            suffix = (
                f" gyro_y={v1:+.1f}dps y_ratio={v2:+.2f} "
                f"elapsed={v3:.0f}ms"
            )
        elif stage == "wait_reject":
            suffix = (
                f" reason={reason} value1={v1:.2f} "
                f"value2={v2:.2f} value3={v3:.1f}"
            )
        elif reason == "turnaround_timeout":
            suffix = (
                f" reason={reason} elapsed={v1:.0f}ms "
                f"max_pos_gy={v2:+.1f}dps min_neg_gy={v3:+.1f}dps"
            )
        elif reason == "distance_too_short":
            suffix = (
                f" reason={reason} fused={v1:.1f}cm "
                f"accel={v2:.1f}cm arc={v3:.1f}cm"
            )
        else:
            suffix = f" reason={reason} values={v1:.2f},{v2:.2f},{v3:.2f}"
        return f"[{ts}] gesture_{stage}{suffix}"
    if event.x is not None:
        suffix = f" x={event.x:+.2f} y={event.y:+.2f} z={event.z:+.2f}"
        if event.elapsed_ms is not None:
            suffix += f" elapsed={event.elapsed_ms}ms"
    else:
        suffix = ""
    marker = " ★" if event.code == EVT_RECORDING_START else ""
    return f"[{ts}] {event.name}{suffix}{marker}"


def diagnostic_record(event: GestureEvent) -> dict[str, Any]:
    record: dict[str, Any] = {
        "time": event.wall_time,
        "stage": DIAG_STAGE_NAMES.get(event.diag_stage or 0, "unknown"),
        "reason": DIAG_REASON_NAMES.get(event.diag_reason or 0, "unknown"),
        "value1": event.value1,
        "value2": event.value2,
        "value3": event.value3,
    }
    stage = record["stage"]
    reason = record["reason"]
    if stage == "distance_ready" or reason == "distance_too_short":
        record["metrics"] = {
            "fused_distance_cm": event.value1,
            "accel_distance_cm": event.value2,
            "arc_distance_cm": event.value3,
        }
    elif stage == "outbound_start":
        record["metrics"] = {
            "shake_ptp_ms2": event.value1,
            "z_ratio": event.value2,
            "shake_mean_ms2": event.value3,
        }
    elif stage == "outbound_ready":
        record["metrics"] = {
            "pronation_angle_deg": event.value1,
            "tilt_3d_deg": event.value2,
            "z_ratio_delta": event.value3,
        }
    elif stage == "stop_palm_up":
        record["metrics"] = {
            "pronation_angle_deg": event.value1,
            "tilt_3d_deg": event.value2,
            "z_ratio_delta": event.value3,
        }
    elif stage == "return_ready":
        record["metrics"] = {
            "supination_angle_deg": event.value1,
            "supination_peak_dps": event.value2,
            "final_y_ratio": event.value3,
        }
    elif stage == "gyro_y_sample":
        record["metrics"] = {
            "gyro_y_dps": event.value1,
            "y_ratio": event.value2,
            "elapsed_ms": event.value3,
        }
    elif stage == "reset" and reason == "turnaround_timeout":
        record["metrics"] = {
            "elapsed_ms": event.value1,
            "max_pos_gyro_y_dps": event.value2,
            "min_neg_gyro_y_dps": event.value3,
        }
    elif stage == "reset" and reason == "final_pulse_duration_invalid":
        record["metrics"] = {
            "pulse_duration_ms": event.value1,
            "positive_impulse_ms": event.value2,
            "negative_impulse_ms": event.value3,
        }
    elif stage == "reset" and reason == "final_brake_ratio_low":
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "negative_impulse_ms": event.value2,
            "brake_ratio": event.value3,
        }
    elif stage == "reset" and reason in (
        "final_accel_missing",
        "final_brake_missing",
        "final_tilt_unstable",
        "final_hold_timeout",
        "lift_palm_still_up",
    ):
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "negative_impulse_ms": event.value2,
            "tilt_deg": event.value3,
        }
    elif stage == "final_sample":
        record["metrics"] = {
            "pulse_stage": event.value1,
            "a_up_ms2": event.value2,
            "net_impulse_ms": event.value3,
        }
    elif stage == "final_hold_start":
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "negative_impulse_ms": event.value2,
            "tilt_deg": event.value3,
        }
    elif stage == "final_ready":
        record["metrics"] = {
            "positive_impulse_ms": event.value1,
            "final_hold_ms": event.value2,
            "tilt_deg": event.value3,
        }
    elif stage == "match":
        record["metrics"] = {
            "pronation_angle_deg": event.value1,
            "positive_impulse_ms": event.value2,
            "final_hold_ms": event.value3,
        }
    elif stage == "gyro_bias_ready":
        record["metrics"] = {
            "correction_dps": event.value1,
            "plane_bias_dps": event.value2,
            "z_bias_dps": event.value3,
        }
    return record


class GestureValidator:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.client: Any = None
        self.queue: asyncio.Queue[GestureEvent] = asyncio.Queue()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.audio_seen = asyncio.Event()
        self.results: list[TrialResult] = []
        self.all_diagnostics: list[dict[str, Any]] = []
        self.address = ""
        self.closing = False

    def _enqueue(self, event: GestureEvent) -> None:
        self.queue.put_nowait(event)

    def _on_notify(self, _sender: Any, data: bytearray) -> None:
        raw = bytes(data)
        event = parse_event_packet(raw)
        if event is not None:
            if event.code == EVT_GESTURE_DIAG:
                record = diagnostic_record(event)
                self.all_diagnostics.append(record)
            if self.loop is not None:
                self.loop.call_soon_threadsafe(self._enqueue, event)
            return

        # Audio packet: [seq][0xAA][PCM...]. 既に録音中かの検出だけに使う。
        if len(raw) >= 2 and raw[1] == 0xAA and self.loop is not None:
            self.loop.call_soon_threadsafe(self.audio_seen.set)

    def _on_disconnected(self, _client: Any) -> None:
        if self.closing:
            return
        print("\n  BLE接続が切断されました。", flush=True)
        if self.loop is None:
            return
        event = GestureEvent(
            code=EVT_DISCONNECTED,
            name=EVENT_NAMES[EVT_DISCONNECTED],
            monotonic_s=time.monotonic(),
            wall_time=datetime.now().isoformat(timespec="milliseconds"),
        )
        self.loop.call_soon_threadsafe(self._enqueue, event)

    async def _write_command(self, command: int) -> None:
        if self.client is None or not self.client.is_connected:
            return
        await self.client.write_gatt_char(
            AUDIO_RX_UUID, bytes([command]), response=True
        )

    def _drain_events(self) -> None:
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break

    async def _wait_for_event(self, code: int, timeout: float) -> bool:
        deadline = self.loop.time() + timeout if self.loop is not None else 0.0
        while self.loop is not None:
            remaining = deadline - self.loop.time()
            if remaining <= 0:
                return False
            try:
                event = await asyncio.wait_for(self.queue.get(), timeout=remaining)
            except asyncio.TimeoutError:
                return False
            if event.code == EVT_DISCONNECTED:
                raise ConnectionError("BLE connection lost")
            if event.code == code:
                return True
        return False

    async def connect(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as exc:
            raise RuntimeError(
                "bleak がありません。mac_client/venv/bin/pip install -r "
                "mac_client/requirements.txt を実行してください。"
            ) from exc

        self.loop = asyncio.get_running_loop()
        if self.args.address:
            target: Any = self.args.address
            self.address = self.args.address
        else:
            print(f"BLE接続準備中: {self.args.device}", flush=True)
            target = await BleakScanner.find_device_by_name(
                self.args.device, timeout=self.args.scan_timeout
            )
            if target is None:
                raise RuntimeError(
                    f"デバイス '{self.args.device}' が見つかりません。"
                )
            self.address = target.address

        self.client = BleakClient(
            target,
            timeout=self.args.connect_timeout,
            disconnected_callback=self._on_disconnected,
        )
        await self.client.connect()
        await self.client.start_notify(AUDIO_TX_UUID, self._on_notify)

        # イベントはprimary接続だけへ送られるため、検証中だけprimaryを引き受ける。
        await self._write_command(0x02)
        print("BLE接続: OK", flush=True)

        # 接続前から録音中なら音声パケットが届く。その場合だけ停止して初期化する。
        await asyncio.sleep(0.8)
        if self.audio_seen.is_set():
            await self._write_command(0x00)
            await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
        self._drain_events()

        if not self.args.skip_preflight:
            await self.preflight()

    async def preflight(self) -> None:
        """BLE writeとevent notifyの往復を実動作の前に検証する。"""
        self._drain_events()
        await self._write_command(0x01)
        started = await self._wait_for_event(EVT_RECORDING_START, timeout=3.0)
        if not started:
            raise RuntimeError("preflightでrecording_startを受信できません")

        await self._write_command(0x00)
        stopped = await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
        if not stopped:
            raise RuntimeError("preflightでrecording_stopを受信できません")

        self._drain_events()
        print("BLEイベント通信: OK", flush=True)
        await asyncio.sleep(0.5)

    async def disconnect(self) -> None:
        if self.client is None:
            return
        self.closing = True
        if self.client.is_connected:
            try:
                # 他の接続があればprimaryを返す。
                await self._write_command(0x03)
            except Exception:
                pass
            try:
                await self.client.stop_notify(AUDIO_TX_UUID)
            except Exception:
                pass
            await self.client.disconnect()

    async def _countdown(self, trial: int) -> None:
        print("", flush=True)
        print(f"試行 {trial}/{self.args.trials}", flush=True)
        print(f"  指示: {self.args.instruction}", flush=True)
        print("  MacのPing音が鳴ってから動作を始めてください。", flush=True)
        for remaining in range(self.args.countdown, 0, -1):
            print(f"  {remaining}...", flush=True)
            await asyncio.sleep(1.0)

    async def _play_start_cue(self) -> None:
        if self.args.no_cue_sound:
            return
        sound_path = Path(self.args.cue_sound).expanduser()
        if not sound_path.is_file():
            raise RuntimeError(f"開始合図の音声ファイルがありません: {sound_path}")
        await asyncio.create_subprocess_exec(
            "/usr/bin/afplay",
            str(sound_path),
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )

    async def _announce_go(self, trial: int) -> None:
        await self._play_start_cue()
        print("  >>> GO <<<", flush=True)

    async def run_trial(self, trial: int) -> TrialResult:
        self._drain_events()
        await self._countdown(trial)
        assert self.loop is not None

        # GO前に届いたイベントを試行成立として数えない。早すぎる発動で録音中に
        # なっていた場合は停止し、この試行を明示的に失敗とする。
        early_match = False
        while not self.queue.empty():
            try:
                early_event = self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break
            if early_event.code == EVT_RECORDING_START:
                early_match = True
        if early_match:
            await self._write_command(0x00)
            await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
            early_conditions = build_condition_results([], matched=False)
            early_conditions[-2] = ConditionResult(
                "1回の動作で完結", "FAIL", "GO表示前に発動したため試行無効"
            )
            early_conditions[-1] = ConditionResult(
                "ジェスチャー発動", "FAIL", "GO表示前の発動は判定対象外"
            )
            result = TrialResult(
                trial=trial,
                expected=self.args.expect,
                result="FAIL",
                matched=True,
                latency_ms=0,
                motion_active_seen=False,
                motion_settled_seen=False,
                sequence_reset_seen=False,
                reason="GO表示前にrecording_startを受信",
                diagnostics=[],
                conditions=early_conditions,
            )
            print_condition_results(result.conditions)
            print(f"  結果: FAIL — {result.reason}", flush=True)
            return result

        start = self.loop.time()
        await self._announce_go(trial)

        deadline = start + self.args.window
        matched = False
        gesture_stopped = False
        latency_ms: Optional[int] = None
        motion_active_seen = False
        motion_settled_seen = False
        disconnected = False
        diagnostics: list[dict[str, Any]] = []

        while self.loop.time() < deadline:
            remaining = deadline - self.loop.time()
            try:
                event = await asyncio.wait_for(self.queue.get(), timeout=remaining)
            except asyncio.TimeoutError:
                break

            if event.code == EVT_DISCONNECTED:
                disconnected = True
                break
            if event.code == EVT_MOTION_ACTIVE:
                motion_active_seen = True
            elif event.code == EVT_MOTION_SETTLED:
                motion_settled_seen = True
            elif event.code == EVT_GESTURE_DIAG:
                diagnostics.append(diagnostic_record(event))
            elif event.code == EVT_RECORDING_START:
                matched = True
                latency_ms = max(0, round((event.monotonic_s - start) * 1000))
            elif event.code == EVT_RECORDING_STOP:
                if matched:
                    gesture_stopped = True
                    break

        if matched and not gesture_stopped and not disconnected:
            await self._write_command(0x00)
            try:
                await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
            except ConnectionError:
                disconnected = True

        reset_seen = sequence_reset_seen(diagnostics)

        if disconnected:
            result = "ERROR"
            reason = "BLE接続が切断されました"
        elif self.args.expect == "match":
            if matched and reset_seen:
                result = "FAIL"
                reason = (
                    "recording_start前にgesture resetを検出"
                    "（単一動作試行として無効）"
                )
            else:
                result = "PASS" if matched and gesture_stopped else "FAIL"
                if matched and gesture_stopped:
                    reason = f"recording_startを{latency_ms} msで受信し、掌上で停止"
                elif matched:
                    reason = (
                        f"recording_startを{latency_ms} msで受信したが"
                        "掌上停止なし"
                    )
                else:
                    reason = f"{self.args.window:g}秒以内にrecording_startなし"
        else:
            result = "FAIL" if matched else "PASS"
            reason = (
                f"意図しないrecording_startを{latency_ms} msで受信"
                if matched
                else f"{self.args.window:g}秒間、誤発動なし"
            )

        conditions = build_condition_results(diagnostics, matched)
        stop_diag = _latest_diagnostic(diagnostics, stage="stop_palm_up")
        if stop_diag is not None:
            phi = float(stop_diag.get("value1") or 0.0)
            tilt = float(stop_diag.get("value2") or 0.0)
            z_delta = abs(float(stop_diag.get("value3") or 0.0))
            stop_ok = recording_stop_palm_up_eligible(phi, tilt, z_delta)
            conditions.append(
                ConditionResult(
                    "掌上で録音終了",
                    "PASS" if stop_ok and gesture_stopped else "FAIL",
                    f"phi {phi:.1f}°、3D {tilt:.1f}°、Δz比 {z_delta:.2f}",
                )
            )
        elif gesture_stopped:
            conditions.append(
                ConditionResult(
                    "掌上で録音終了",
                    "PASS",
                    "recording_stopを受信",
                )
            )
        elif matched:
            conditions.append(
                ConditionResult(
                    "掌上で録音終了",
                    "FAIL",
                    "判定時間内に掌上停止なし",
                )
            )
        else:
            conditions.append(
                ConditionResult(
                    "掌上で録音終了",
                    "NOT_REACHED",
                    "録音開始前",
                )
            )
        trial_result = TrialResult(
            trial=trial,
            expected=self.args.expect,
            result=result,
            matched=matched,
            latency_ms=latency_ms,
            motion_active_seen=motion_active_seen,
            motion_settled_seen=motion_settled_seen,
            sequence_reset_seen=reset_seen,
            reason=reason,
            diagnostics=diagnostics,
            conditions=conditions,
        )
        print_condition_results(trial_result.conditions)
        if getattr(self.args, "show_gyro", False):
            summarize_gyro_y_samples(diagnostics)
        gyro_csv = getattr(self.args, "gyro_csv", None)
        if gyro_csv:
            stem = Path(gyro_csv)
            out = (
                stem
                if self.args.trials == 1
                else stem.with_name(f"{stem.stem}_t{trial}{stem.suffix}")
            )
            write_gyro_csv(str(out), diagnostics)
        # Always summarize briefly when debug samples arrived.
        if any(r.get("stage") == "gyro_y_sample" for r in diagnostics):
            if not getattr(self.args, "show_gyro", False):
                summarize_gyro_y_samples(diagnostics)
        print(f"  結果: {result} — {reason}", flush=True)
        return trial_result

    async def run(self) -> int:
        started_at = datetime.now().isoformat(timespec="seconds")
        await self.connect()
        print("", flush=True)
        print("=" * 64, flush=True)
        print("HarnessNode 回内→上昇パルス→0.5秒静止 BLE検証", flush=True)
        print(f"期待結果: {self.args.expect}", flush=True)
        print(f"試行回数: {self.args.trials} / 判定時間: {self.args.window:g}秒", flush=True)
        print("=" * 64, flush=True)

        try:
            for trial in range(1, self.args.trials + 1):
                result = await self.run_trial(trial)
                self.results.append(result)
                if result.result == "ERROR":
                    break
                if trial < self.args.trials:
                    await asyncio.sleep(self.args.interval)
        finally:
            await self.disconnect()

        passed = sum(result.result == "PASS" for result in self.results)
        failed = sum(result.result == "FAIL" for result in self.results)
        errors = sum(result.result == "ERROR" for result in self.results)
        total = len(self.results)
        overall = "PASS" if total == self.args.trials and failed == 0 and errors == 0 else "FAIL"

        print("", flush=True)
        print("=" * 64, flush=True)
        print(f"総合結果: {overall}  PASS={passed} FAIL={failed} ERROR={errors}", flush=True)
        print("=" * 64, flush=True)

        if self.args.json_output:
            report = {
                "started_at": started_at,
                "finished_at": datetime.now().isoformat(timespec="seconds"),
                "device": self.args.device,
                "address": self.address,
                "expected": self.args.expect,
                "instruction": self.args.instruction,
                "window_s": self.args.window,
                "overall": overall,
                "results": [asdict(result) for result in self.results],
                "diagnostics": self.all_diagnostics,
            }
            output_path = Path(self.args.json_output).expanduser()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"レポート保存: {output_path}", flush=True)

        return 0 if overall == "PASS" else 1


def run_self_test() -> int:
    start = parse_event_packet(bytes([0x00, 0x55, EVT_RECORDING_START]), now=12.5)
    assert start is not None
    assert start.code == EVT_RECORDING_START
    assert start.monotonic_s == 12.5

    xyz_packet = bytearray([0x00, 0x55, EVT_MOTION_ACTIVE])
    xyz_packet.extend(struct.pack("<fff", 1.25, -2.5, 9.75))
    motion = parse_event_packet(bytes(xyz_packet), now=13.0)
    assert motion is not None
    assert motion.name == "motion_active"
    assert motion.x is not None and math.isclose(motion.x, 1.25)
    assert motion.y is not None and math.isclose(motion.y, -2.5)
    assert motion.z is not None and math.isclose(motion.z, 9.75)

    assert parse_event_packet(bytes([0x01, 0xAA, 0x00, 0x00])) is None
    assert parse_event_packet(bytes([0x00, 0x55])) is None

    diag_packet = bytearray([0x00, 0x55, EVT_GESTURE_DIAG, 0x02, 0x00])
    diag_packet.extend(struct.pack("<fff", 42.0, 70.0, 0.12))
    diag = parse_event_packet(bytes(diag_packet), now=14.0)
    assert diag is not None
    assert diag.diag_stage == 0x02
    assert diag.value1 is not None and math.isclose(diag.value1, 42.0)
    assert "pronation=+42.0deg" in format_event(diag)
    assert "tilt_3d=70.0deg" in format_event(diag)
    assert math.isclose(
        diagnostic_record(diag)["metrics"]["tilt_3d_deg"], 70.0
    )

    stop_packet = bytearray([0x00, 0x55, EVT_GESTURE_DIAG, 0x0C, 0x00])
    stop_packet.extend(struct.pack("<fff", 9.6, 16.0, 0.28))
    stop_diag = parse_event_packet(bytes(stop_packet), now=14.2)
    assert stop_diag is not None
    assert stop_diag.diag_stage == 0x0C
    assert "stop_palm_up" in format_event(stop_diag)
    assert math.isclose(
        diagnostic_record(stop_diag)["metrics"]["z_ratio_delta"], 0.28, abs_tol=1e-6
    )

    gyro_packet = bytearray([0x00, 0x55, EVT_GESTURE_DIAG, 0x20, 0x00])
    gyro_packet.extend(struct.pack("<fff", -12.5, 0.15, 250.0))
    gyro = parse_event_packet(bytes(gyro_packet), now=14.5)
    assert gyro is not None
    assert gyro.diag_stage == 0x20
    assert "gyro_y=-12.5dps" in format_event(gyro)
    assert math.isclose(
        diagnostic_record(gyro)["metrics"]["gyro_y_dps"], -12.5
    )

    final_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x08, 0x00]
    )
    final_packet.extend(struct.pack("<fff", 0.12, 525.0, 8.2))
    final = parse_event_packet(bytes(final_packet), now=16.0)
    assert final is not None
    assert final.diag_stage == 0x08
    assert "hold=525ms" in format_event(final)

    bias_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x0A, 0x00]
    )
    bias_packet.extend(struct.pack("<fff", 22.0, 21.5, 4.5))
    bias = parse_event_packet(bytes(bias_packet), now=17.0)
    assert bias is not None
    assert bias.diag_stage == 0x0A
    assert "correction=22.0dps" in format_event(bias)
    bias_record = diagnostic_record(bias)
    assert math.isclose(bias_record["metrics"]["correction_dps"], 22.0)

    # Firmware boundary: board-flat shake, palm-up flip, lift pulse, pose, hold.
    assert HOLD_PRONATION_ANGLE_MIN_DEG == 20.0
    assert HOLD_PRONATION_Z_RATIO_DONE == 0.50
    eligible = (0.90, 20.0, 0.04, 0.015, 10.0, 500)
    assert gesture_gate_eligible(*eligible)
    assert gesture_gate_eligible(0.80, *eligible[1:])
    assert not gesture_gate_eligible(0.79, *eligible[1:])
    assert not gesture_gate_eligible(0.90, 7.9, *eligible[2:])
    assert gesture_gate_eligible(0.90, 8.0, *eligible[2:])
    assert gesture_gate_eligible(0.90, 4.0, 0.04, 0.015, 10.0, 500, 0.25)
    assert not gesture_gate_eligible(0.90, 4.0, 0.04, 0.015, 10.0, 500, 0.24)
    assert gesture_gate_eligible(0.90, 4.0, 0.04, 0.015, 10.0, 500, 0.0, True)
    assert gesture_gate_eligible(
        0.90, 4.0, 0.04, 0.015, 10.0, 500, palm_up_tilt_deg=15.0
    )
    assert not gesture_gate_eligible(
        0.90, 4.0, 0.04, 0.015, 10.0, 500, palm_up_tilt_deg=14.9
    )
    assert recording_stop_palm_up_eligible(8.0)
    assert recording_stop_palm_up_eligible(4.0, palm_up_tilt_deg=15.0)
    assert recording_stop_palm_up_eligible(4.0, z_ratio_delta=0.25)
    assert recording_stop_palm_up_eligible(4.0, z_sign_flip=True)
    assert not recording_stop_palm_up_eligible(7.9)
    assert not gesture_gate_eligible(0.90, 20.0, 0.039, *eligible[3:])
    assert not gesture_gate_eligible(0.90, 20.0, 0.04, 0.015, 10.1, 500)
    assert not gesture_gate_eligible(*eligible[:-1], 499)
    assert not gesture_gate_eligible(*eligible, shake_ptp_ms2=4.9)
    assert not gesture_gate_eligible(*eligible, shake_ptp_ms2=6.0, shake_mean_ms2=3.0)
    assert gesture_gate_eligible(*eligible, shake_ptp_ms2=6.0, shake_mean_ms2=2.0)
    assert not sequence_reset_seen([])
    assert not sequence_reset_seen([{"stage": "outbound_ready"}])
    assert sequence_reset_seen(
        [{"stage": "outbound_ready"}, {"stage": "reset"}]
    )
    retry_then_match = [
        {
            "stage": "outbound_start",
            "reason": "none",
            "value1": 6.2,
            "value2": 0.92,
            "value3": 0.20,
        },
        {
            "stage": "outbound_ready",
            "reason": "none",
            "value1": 35.0,
            "value2": 45.0,
            "value3": 0.55,
        },
        {
            "stage": "wait_reject",
            "reason": "final_brake_missing",
            "value1": 1.96,
            "value2": 0.0,
            "value3": 0.0,
        },
        {
            "stage": "final_hold_start",
            "reason": "none",
            "value1": 0.12,
            "value2": 0.10,
            "value3": 1.0,
        },
        {
            "stage": "final_ready",
            "reason": "none",
            "value1": 0.12,
            "value2": 525.0,
            "value3": 1.0,
        },
        {
            "stage": "match",
            "reason": "none",
            "value1": 35.0,
            "value2": 0.12,
            "value3": 525.0,
        },
    ]
    assert not sequence_reset_seen(retry_then_match)
    retry_conditions = build_condition_results(
        retry_then_match, matched=True
    )
    assert all(
        condition.status == "PASS" for condition in retry_conditions
    )

    passing_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start",
                "reason": "none",
                "value1": 6.4,
                "value2": 0.92,
                "value3": 0.20,
            },
            {
                "stage": "outbound_ready",
                "reason": "none",
                "value1": 35.0,
                "value2": 45.0,
                "value3": 0.55,
            },
            {
                "stage": "final_hold_start",
                "reason": "none",
                "value1": 0.12,
                "value2": 0.10,
                "value3": 6.0,
            },
            {
                "stage": "final_ready",
                "reason": "none",
                "value1": 0.12,
                "value2": 525.0,
                "value3": 6.0,
            },
        ],
        matched=True,
    )
    assert all(condition.status == "PASS" for condition in passing_conditions)

    rejected_conditions = build_condition_results(
        [
            {
                "stage": "wait_reject",
                "reason": "start_not_palm_up",
                "value1": 0.40,
                "value2": 0.50,
                "value3": 5.0,
            }
        ],
        matched=False,
    )
    assert rejected_conditions[0].status == "FAIL"
    assert rejected_conditions[-1].status == "FAIL"
    for reason, values in (
        ("final_accel_missing", (0.02, 0.0, 2.0)),
        ("final_brake_missing", (0.10, 0.01, 2.0)),
        ("final_brake_ratio_low", (0.50, 0.06, 0.12)),
        ("final_pulse_duration_invalid", (100.0, 0.10, 0.08)),
        ("final_tilt_unstable", (0.12, 0.10, 18.0)),
        ("final_hold_timeout", (0.12, 0.10, 5.0)),
        ("lift_palm_still_up", (0.12, 0.10, 0.0)),
    ):
        conditions = build_condition_results(
            [
                {
                    "stage": "outbound_ready",
                    "reason": "none",
                    "value1": 35.0,
                    "value2": 35.0,
                    "value3": 0.55,
                },
                {
                    "stage": "reset",
                    "reason": reason,
                    "value1": values[0],
                    "value2": values[1],
                    "value3": values[2],
                },
            ],
            matched=False,
        )
        assert any(condition.status == "FAIL" for condition in conditions)
    z_only_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start",
                "reason": "none",
                "value1": 6.0,
                "value2": 0.92,
                "value3": 0.18,
            },
            {
                "stage": "outbound_ready",
                "reason": "none",
                "value1": 12.0,
                "value2": 12.0,
                "value3": 0.55,
            },
            {
                "stage": "final_hold_start",
                "reason": "none",
                "value1": 0.12,
                "value2": 0.10,
                "value3": 6.0,
            },
            {
                "stage": "final_ready",
                "reason": "none",
                "value1": 0.12,
                "value2": 525.0,
                "value3": 6.0,
            },
        ],
        matched=True,
    )
    assert all(condition.status == "PASS" for condition in z_only_conditions)

    shake_fail_conditions = build_condition_results(
        [
            {
                "stage": "wait_reject",
                "reason": "shake_not_oscillatory",
                "value1": 6.0,
                "value2": 5.0,
                "value3": 0.83,
            }
        ],
        matched=False,
    )
    assert shake_fail_conditions[0].label == "掌下の短いシェイク"
    assert shake_fail_conditions[0].status == "FAIL"

    palm_up_lift_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start",
                "reason": "none",
                "value1": 6.2,
                "value2": 0.92,
                "value3": 0.20,
            },
            {
                "stage": "outbound_ready",
                "reason": "none",
                "value1": 35.0,
                "value2": 45.0,
                "value3": 0.55,
            },
            {
                "stage": "wait_reject",
                "reason": "lift_palm_still_up",
                "value1": -0.80,
                "value2": 0.50,
                "value3": 0.12,
            },
            {
                "stage": "reset",
                "reason": "lift_palm_still_up",
                "value1": 0.12,
                "value2": 0.10,
                "value3": 0.0,
            },
        ],
        matched=False,
    )
    assert any(
        condition.label == "掌下で静止" and condition.status == "FAIL"
        for condition in palm_up_lift_conditions
    )
    assert any(
        condition.label == "挙上" and condition.status == "PASS"
        for condition in palm_up_lift_conditions
    )
    print("SELF_TEST: PASS", flush=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="HarnessNodeのシェイク→掌上→挙上→掌下静止をBLEで検証"
    )
    parser.add_argument("--device", default=DEVICE_NAME, help="BLEデバイス名")
    parser.add_argument("--address", help="スキャンせず接続するBLEアドレス/UUID")
    parser.add_argument("--trials", type=int, default=3, help="試行回数 (default: 3)")
    parser.add_argument(
        "--expect",
        choices=("match", "no-match"),
        default="match",
        help="各試行の期待結果 (default: match)",
    )
    parser.add_argument(
        "--instruction",
        help="試行時にユーザーへ表示する動作指示",
    )
    parser.add_argument(
        "--countdown", type=int, default=3, help="GOまでの秒数 (default: 3)"
    )
    parser.add_argument(
        "--window", type=float, default=15.0, help="GO後の判定秒数 (default: 15)"
    )
    parser.add_argument(
        "--interval", type=float, default=2.0, help="試行間隔秒 (default: 2)"
    )
    parser.add_argument(
        "--cue-sound",
        default=DEFAULT_CUE_SOUND,
        help=f"GO時に再生する音声ファイル (default: {DEFAULT_CUE_SOUND})",
    )
    parser.add_argument(
        "--no-cue-sound",
        action="store_true",
        help="GO時のMacの合図音を無効化",
    )
    parser.add_argument(
        "--scan-timeout", type=float, default=15.0, help="スキャン秒数"
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=10.0, help="接続タイムアウト秒"
    )
    parser.add_argument("--json-output", help="JSONレポートの保存先")
    parser.add_argument(
        "--gyro-csv",
        help="デバッグ用 gyro_y_sample をCSV保存（ファーム GESTURE_DEBUG_GYRO_Y=1 時）",
    )
    parser.add_argument(
        "--show-gyro",
        action="store_true",
        help="gyro_y_sample の min/max 要約を画面表示",
    )
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="BLE start/stopイベントの事前往復確認を省略",
    )
    parser.add_argument(
        "--self-test", action="store_true", help="BLE接続なしの内部テストを実行"
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.trials < 1:
        parser.error("--trials は1以上にしてください")
    if args.countdown < 0:
        parser.error("--countdown は0以上にしてください")
    if args.window <= 0 or args.interval < 0:
        parser.error("--window は正、--interval は0以上にしてください")
    if args.instruction is None:
        args.instruction = (
            "手の甲側装着で短く1回振り、手のひらを上へ向け、手を上げてから、"
            "掌を下にして0.5秒静止してください。"
            "録音開始後、手のひらを上へ向けると録音が終了します"
            if args.expect == "match"
            else "指定された非対象動作を行ってください"
        )

    try:
        return asyncio.run(GestureValidator(args).run())
    except KeyboardInterrupt:
        print("\n中断しました。", flush=True)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
