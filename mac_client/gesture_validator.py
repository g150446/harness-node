#!/usr/bin/env python3
"""HarnessNode の水平回内→屈曲・回外→垂直静止をBLEで対話検証する。

人が画面のカウントダウンに合わせて動作し、ファームウェアから届く
``recording_start`` イベント (0x01) の有無を試行ごとに判定する。

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
    0x13: "turnaround_timeout",
    0x14: "wrong_roll_direction",
    0x15: "horizontal_lost",
    0x16: "return_timeout",
    0x17: "incomplete_return",
    0x18: "distance_too_short",
    0x19: "final_y_not_reached",
    0x1A: "final_hold_interrupted",
    0x1B: "final_hold_timeout",
    0x1C: "sequence_timeout",
}

START_PALM_UP_Z_MIN_RATIO = 0.90
START_Y_MAX_RATIO = 0.17  # ~sin(10°)
HORIZONTAL_Y_MAX_RATIO = 0.17
START_QUIET_HOLD_MS = 200
START_QUIET_RATE_MAX_DPS = 19.0
ROLL_START_RATE_MIN_DPS = 35.0
TRANSVERSE_START_RATE_MIN_DPS = 35.0
TRANSVERSE_ANGLE_MIN_DEG = 60.0
TRANSVERSE_PEAK_RATE_MIN_DPS = 50.0
ACCEL_MIN_MS2 = 0.50
PHASE_DURATION_MIN_MS = 180
OUTBOUND_DURATION_MAX_MS = 2000
RETURN_DURATION_MAX_MS = 2500
ROLL_ANGLE_MIN_DEG = 30.0
ROLL_PEAK_RATE_MIN_DPS = 55.0
FINAL_Y_MIN_RATIO = 0.94
TURNAROUND_TIMEOUT_MS = 750
DISTANCE_MIN_CM = 5.0
FINAL_RATE_MAX_DPS = 19.0
FINAL_LINEAR_ACCEL_MAX_MS2 = 1.20
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
    fused_distance_cm: float,
    palm_up_z_ratio: float,
    start_y_ratio: float,
    outbound_max_abs_y_ratio: float,
    outbound_roll_deg: float,
    return_roll_deg: float,
    final_y_ratio: float,
    final_hold_ms: int,
) -> bool:
    """Mirror the firmware's public pose, roll, distance and hold gates."""
    return (
        fused_distance_cm >= DISTANCE_MIN_CM
        and palm_up_z_ratio >= START_PALM_UP_Z_MIN_RATIO
        and abs(start_y_ratio) <= START_Y_MAX_RATIO
        and outbound_max_abs_y_ratio <= HORIZONTAL_Y_MAX_RATIO
        and abs(outbound_roll_deg) >= ROLL_ANGLE_MIN_DEG
        and abs(return_roll_deg) >= ROLL_ANGLE_MIN_DEG
        and outbound_roll_deg * return_roll_deg < 0.0
        and abs(final_y_ratio) >= FINAL_Y_MIN_RATIO
        and final_hold_ms >= FINAL_HOLD_MIN_MS
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
    turnaround_ready = _latest_diagnostic(diagnostics, stage="turnaround_ready")
    return_start = _latest_diagnostic(diagnostics, stage="return_start")
    return_ready = _latest_diagnostic(diagnostics, stage="return_ready")
    distance = _latest_diagnostic(diagnostics, stage="distance_ready")
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
    wait_rate = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("outbound_rate_low",),
    )
    outbound_reset = _latest_diagnostic(
        diagnostics,
        stage="reset",
        reasons=(
            "outbound_timeout",
            "incomplete_outbound",
            "horizontal_lost",
        ),
    )
    return_reset = _latest_diagnostic(
        diagnostics,
        stage="reset",
        reasons=(
            "return_timeout",
            "incomplete_return",
            "wrong_roll_direction",
            "final_y_not_reached",
        ),
    )

    conditions: list[ConditionResult] = []

    start_evidence = outbound_start or wait_rate or wait_quiet or wait_pose
    if outbound_start is not None:
        z_ratio = float(outbound_start["value2"])
        y_ratio = float(outbound_start["value3"])
        conditions.append(
            ConditionResult(
                "手のひら上向きの水平開始姿勢",
                "PASS",
                f"補正Z比 {z_ratio:.2f} ≥ {START_PALM_UP_Z_MIN_RATIO:.2f}、"
                f"|Y比| {abs(y_ratio):.2f} ≤ {START_Y_MAX_RATIO:.2f}",
            )
        )
        conditions.append(
            ConditionResult(
                "開始前の静止",
                "PASS",
                f"{START_QUIET_HOLD_MS} ms以上・角速度 "
                f"≤ {START_QUIET_RATE_MAX_DPS:.0f}°/s が成立",
            )
        )
    elif start_evidence is not None:
        z_ratio = float(start_evidence["value1"])
        y_ratio = float(start_evidence["value2"])
        pose_passed = (
            z_ratio >= START_PALM_UP_Z_MIN_RATIO
            and abs(y_ratio) <= START_Y_MAX_RATIO
        )
        conditions.append(
            ConditionResult(
                "手のひら上向きの水平開始姿勢",
                "PASS" if pose_passed else "FAIL",
                f"補正Z比 {z_ratio:.2f}、|Y比| {abs(y_ratio):.2f}",
            )
        )
        quiet_passed = wait_rate is not None
        conditions.append(
            ConditionResult(
                "開始前の静止",
                "PASS" if quiet_passed else "FAIL",
                (
                    f"{START_QUIET_HOLD_MS} ms以上・角速度 "
                    f"≤ {START_QUIET_RATE_MAX_DPS:.0f}°/s が成立"
                    if quiet_passed
                    else f"{START_QUIET_HOLD_MS} msの静止を確認できず"
                ),
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "手のひら上向きの水平開始姿勢",
                "NOT_REACHED",
                "判定データなし",
            )
        )
        conditions.append(
            ConditionResult("開始前の静止", "NOT_REACHED", "判定データなし")
        )

    if outbound_ready is not None and outbound_start is not None:
        start_rate = float(outbound_start["value1"])
        roll = float(outbound_ready["value1"])
        peak = float(outbound_ready["value2"])
        max_abs_y = float(outbound_ready["value3"])
        roll_ok = (
            abs(roll) >= ROLL_ANGLE_MIN_DEG
            and peak >= ROLL_PEAK_RATE_MIN_DPS
            and start_rate >= ROLL_START_RATE_MIN_DPS
        )
        horizontal_ok = max_abs_y <= HORIZONTAL_Y_MAX_RATIO
        conditions.append(
            ConditionResult(
                "水平を維持した回内",
                "PASS" if roll_ok and horizontal_ok else "FAIL",
                f"開始 {start_rate:.1f} ≥ {ROLL_START_RATE_MIN_DPS:.0f}°/s、"
                f"Y軸積分 {roll:+.1f}°（|角度|≥{ROLL_ANGLE_MIN_DEG:.0f}°・"
                f"peak≥{ROLL_PEAK_RATE_MIN_DPS:.0f}°/s）、"
                f"最大|Y比| {max_abs_y:.2f} ≤ {HORIZONTAL_Y_MAX_RATIO:.2f}",
            )
        )
    elif outbound_reset is not None:
        conditions.append(
            ConditionResult(
                "水平を維持した回内",
                "FAIL",
                f"Y軸積分 {float(outbound_reset['value1']):+.1f}° "
                f"({outbound_reset['reason']})",
            )
        )
    elif outbound_start is not None:
        conditions.append(
            ConditionResult(
                "水平を維持した回内", "FAIL", "回内完了条件まで到達せず"
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "水平を維持した回内", "NOT_REACHED", "回内開始を検出せず"
            )
        )

    conditions.append(
        ConditionResult(
            "回内から屈曲への切り返し",
            "PASS" if turnaround_ready is not None else "NOT_REACHED",
            (
                f"{float(turnaround_ready['value1']):.0f} msで検出"
                if turnaround_ready is not None
                else "切り返し判定まで到達せず"
            ),
        )
    )

    if return_ready is not None and return_start is not None:
        conditions.append(
            ConditionResult(
                "肘の屈曲動作",
                "PASS",
                f"開始 {float(return_start['value1']):.1f}°/s、"
                f"合成角 {float(return_ready['value1']):.1f}° ≥ "
                f"{TRANSVERSE_ANGLE_MIN_DEG:.0f}°、加速度 "
                f"{float(return_ready['value3']):.2f} m/s²",
            )
        )
    elif return_reset is not None:
        conditions.append(
            ConditionResult(
                "肘の屈曲動作",
                "FAIL",
                f"合成角 {float(return_reset['value1']):.1f}° "
                f"({return_reset['reason']})",
            )
        )
    else:
        conditions.append(
            ConditionResult("肘の屈曲動作", "NOT_REACHED", "復路完了まで到達せず")
        )

    if outbound_ready is not None and return_ready is not None:
        outbound_roll = float(outbound_ready["value1"])
        return_roll = float(return_ready["value2"])
        opposite = outbound_roll * return_roll < 0.0
        conditions.append(
            ConditionResult(
                "屈曲中の回外",
                "PASS" if opposite else "FAIL",
                f"往路 {outbound_roll:+.1f}° → 復路 {return_roll:+.1f}° "
                f"（逆符号・|復路|≥{ROLL_ANGLE_MIN_DEG:.0f}°）",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "屈曲中の回外", "NOT_REACHED", "回外判定まで到達せず"
            )
        )

    if distance is not None:
        fused_cm = float(distance["value1"])
        conditions.append(
            ConditionResult(
                "復路で5 cm以上の移動",
                "PASS" if fused_cm >= DISTANCE_MIN_CM else "FAIL",
                f"融合推定 {fused_cm:.1f} cm "
                f"{'≥' if fused_cm >= DISTANCE_MIN_CM else '<'} {DISTANCE_MIN_CM:.1f} cm",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "復路で5 cm以上の移動", "NOT_REACHED", "距離判定まで到達せず"
            )
        )

    final_evidence = final_ready or final_start
    if final_evidence is not None:
        final_y = float(final_evidence["value1"])
        vertical_ok = abs(final_y) >= FINAL_Y_MIN_RATIO
        conditions.append(
            ConditionResult(
                "垂直終了姿勢",
                "PASS" if vertical_ok else "FAIL",
                f"最終Y比 {final_y:+.2f}、"
                f"|Y比| {'≥' if vertical_ok else '<'} "
                f"{FINAL_Y_MIN_RATIO:.2f}（垂直から20°以内）",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "垂直終了姿勢",
                "NOT_REACHED",
                "最終姿勢判定まで到達せず",
            )
        )

    if final_ready is not None:
        hold_ms = float(final_ready["value2"])
        tilt_deg = float(final_ready["value3"])
        conditions.append(
            ConditionResult(
                "垂直位置での静止保持",
                "PASS",
                f"{hold_ms:.0f} ms ≥ {FINAL_HOLD_MIN_MS} ms、傾き {tilt_deg:.1f}° "
                f"（角速度≤{FINAL_RATE_MAX_DPS:.0f}°/s・"
                f"線形加速度≤{FINAL_LINEAR_ACCEL_MAX_MS2:.2f} m/s²も成立）",
            )
        )
    elif final_start is not None:
        conditions.append(
            ConditionResult(
                "垂直位置での静止保持",
                "FAIL",
                f"{FINAL_HOLD_MIN_MS} msの連続静止を確認できず",
            )
        )
    else:
        conditions.append(
            ConditionResult(
                "垂直位置での静止保持", "NOT_REACHED", "垂直姿勢が未成立"
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
                f" roll_rate={v1:.1f}dps palm_up_z={v2:.2f} "
                f"start_y={v3:+.2f}"
            )
        elif stage == "outbound_ready":
            suffix = (
                f" pronation={v1:+.1f}deg peak={v2:.1f}dps "
                f"max_abs_y={v3:.2f}"
            )
        elif stage == "turnaround_ready":
            suffix = (
                f" elapsed={v1:.0f}ms y={v2:+.2f} gyro={v3:.1f}dps"
            )
        elif stage == "return_start":
            suffix = (
                f" transverse_rate={v1:.1f}dps gyro_y={v2:+.1f}dps "
                f"outbound_roll={v3:+.1f}deg"
            )
        elif stage == "return_ready":
            suffix = (
                f" flexion={v1:.1f}deg supination={v2:+.1f}deg "
                f"accel={v3:.2f}m/s^2"
            )
        elif stage == "final_hold_start":
            suffix = (
                f" final_y={v1:+.2f} gyro={v2:.1f}dps "
                f"linear_accel={v3:.2f}m/s^2"
            )
        elif stage == "match":
            suffix = (
                f" distance={v1:.1f}cm final_y={v2:+.2f} "
                f"hold={v3:.0f}ms"
            )
        elif stage == "distance_ready":
            suffix = (
                f" fused={v1:.1f}cm accel={v2:.1f}cm arc={v3:.1f}cm"
            )
        elif stage == "final_ready":
            suffix = (
                f" final_y={v1:+.2f} hold={v2:.0f}ms tilt={v3:.1f}deg"
            )
        elif stage == "gyro_bias_ready":
            suffix = (
                f" correction={v1:.1f}dps plane_bias={v2:.1f}dps "
                f"z_bias={v3:.1f}dps"
            )
        elif stage == "wait_reject":
            suffix = (
                f" reason={reason} value1={v1:.2f} "
                f"value2={v2:.2f} value3={v3:.1f}"
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
    elif stage == "outbound_ready":
        record["metrics"] = {
            "pronation_angle_deg": event.value1,
            "pronation_peak_dps": event.value2,
            "max_abs_y_ratio": event.value3,
        }
    elif stage == "return_ready":
        record["metrics"] = {
            "flexion_angle_deg": event.value1,
            "supination_angle_deg": event.value2,
            "accel_evidence_ms2": event.value3,
        }
    elif stage == "final_hold_start":
        record["metrics"] = {
            "final_y_ratio": event.value1,
            "gyro_dps": event.value2,
            "linear_accel_ms2": event.value3,
        }
    elif stage == "final_ready":
        record["metrics"] = {
            "final_y_ratio": event.value1,
            "final_hold_ms": event.value2,
            "tilt_deg": event.value3,
        }
    elif stage == "match":
        record["metrics"] = {
            "fused_distance_cm": event.value1,
            "final_y_ratio": event.value2,
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
                break

        if matched:
            # 次の試行に備えて録音を明示停止する。
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
                result = "PASS" if matched else "FAIL"
                reason = (
                    f"recording_startを{latency_ms} msで受信"
                    if matched
                    else f"{self.args.window:g}秒以内にrecording_startなし"
                )
        else:
            result = "FAIL" if matched else "PASS"
            reason = (
                f"意図しないrecording_startを{latency_ms} msで受信"
                if matched
                else f"{self.args.window:g}秒間、誤発動なし"
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
            conditions=build_condition_results(diagnostics, matched),
        )
        print_condition_results(trial_result.conditions)
        print(f"  結果: {result} — {reason}", flush=True)
        return trial_result

    async def run(self) -> int:
        started_at = datetime.now().isoformat(timespec="seconds")
        await self.connect()
        print("", flush=True)
        print("=" * 64, flush=True)
        print("HarnessNode 水平回内→屈曲・回外→垂直静止 BLE検証", flush=True)
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
    assert "max_abs_y=0.12" in format_event(diag)

    distance_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x06, 0x00]
    )
    distance_packet.extend(struct.pack("<fff", 5.25, 4.75, 9.0))
    distance = parse_event_packet(bytes(distance_packet), now=15.0)
    assert distance is not None
    assert distance.diag_stage == 0x06
    assert "fused=5.2cm" in format_event(distance)
    distance_record = diagnostic_record(distance)
    assert math.isclose(
        distance_record["metrics"]["fused_distance_cm"], 5.25
    )

    final_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x08, 0x00]
    )
    final_packet.extend(struct.pack("<fff", -0.95, 525.0, 18.2))
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

    # Firmware boundary: start pose, horizontal outbound, opposite roll, final.
    eligible = (5.0, 0.90, 0.17, 0.17, 30.0, -30.0, -0.94, 500)
    assert gesture_gate_eligible(*eligible)
    assert not gesture_gate_eligible(4.9, *eligible[1:])
    assert not gesture_gate_eligible(5.0, 0.89, *eligible[2:])
    assert not gesture_gate_eligible(5.0, 0.90, 0.18, *eligible[3:])
    assert not gesture_gate_eligible(5.0, 0.90, 0.17, 0.18, *eligible[4:])
    assert not gesture_gate_eligible(
        5.0, 0.90, 0.17, 0.17, 30.0, 30.0, -0.94, 500
    )
    assert not gesture_gate_eligible(
        5.0, 0.90, 0.17, 0.17, 30.0, -30.0, -0.93, 500
    )
    assert not gesture_gate_eligible(*eligible[:-1], 499)
    assert not sequence_reset_seen([])
    assert not sequence_reset_seen([{"stage": "outbound_ready"}])
    assert sequence_reset_seen(
        [{"stage": "outbound_ready"}, {"stage": "reset"}]
    )

    passing_conditions = build_condition_results(
        [
            {
                "stage": "outbound_start",
                "reason": "none",
                "value1": 40.0,
                "value2": 0.92,
                "value3": 0.05,
            },
            {
                "stage": "outbound_ready",
                "reason": "none",
                "value1": 45.0,
                "value2": 70.0,
                "value3": 0.12,
            },
            {
                "stage": "turnaround_ready",
                "reason": "none",
                "value1": 125.0,
                "value2": 0.10,
                "value3": 20.0,
            },
            {
                "stage": "return_start",
                "reason": "none",
                "value1": 42.0,
                "value2": -30.0,
                "value3": 45.0,
            },
            {
                "stage": "return_ready",
                "reason": "none",
                "value1": 88.0,
                "value2": -48.0,
                "value3": 4.0,
            },
            {
                "stage": "distance_ready",
                "reason": "none",
                "value1": 12.0,
                "value2": 10.0,
                "value3": 14.0,
            },
            {
                "stage": "final_hold_start",
                "reason": "none",
                "value1": -0.95,
                "value2": 8.0,
                "value3": 0.50,
            },
            {
                "stage": "final_ready",
                "reason": "none",
                "value1": -0.96,
                "value2": 525.0,
                "value3": 16.3,
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
    print("SELF_TEST: PASS", flush=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="HarnessNodeの水平回内→屈曲・回外→垂直静止をBLEで検証"
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
        "--window", type=float, default=10.0, help="GO後の判定秒数 (default: 10)"
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
            "手のひらを上向き水平で静止し、前腕を水平に保ったまま回内した後、"
            "切り返して肘を曲げながら回外し、手・前腕を垂直にして0.5秒静止してください"
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
