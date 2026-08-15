#!/usr/bin/env python3
"""HarnessNode の水平開始→屈曲→垂直静止ジェスチャーをBLEで対話検証する。

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
    0x01: "flex_start",
    0x02: "flex_complete",
    0x03: "vertical_hold_start",
    0x04: "match",
    0x05: "distance_ready",
    0x06: "vertical_ready",
    0x07: "gyro_bias_ready",
    0x10: "wait_reject",
    0x80: "reset",
}

DIAG_REASON_NAMES = {
    0x00: "none",
    0x01: "quiet_not_ready",
    0x02: "start_not_horizontal",
    0x03: "flex_rate_low",
    0x11: "flex_timeout",
    0x13: "incomplete_flex",
    0x14: "endpoint_not_vertical",
    0x16: "distance_too_short",
    0x17: "vertical_hold_interrupted",
    0x18: "vertical_hold_timeout",
}

START_HORIZONTAL_Z_MIN_RATIO = 0.80
START_QUIET_HOLD_MS = 200
START_QUIET_RATE_MAX_DPS = 19.0
FLEX_START_RATE_MIN_DPS = 35.0
FLEX_ANGLE_MIN_DEG = 60.0
FLEX_PEAK_RATE_MIN_DPS = 50.0
FLEX_ACCEL_MIN_MS2 = 0.50
FLEX_DURATION_MIN_MS = 180
FLEX_DURATION_MAX_MS = 2000
DISTANCE_MIN_CM = 5.0
VERTICAL_PLANE_MIN_RATIO = 0.80
VERTICAL_Z_MAX_RATIO = 0.45
VERTICAL_RATE_MAX_DPS = 19.0
VERTICAL_LINEAR_ACCEL_MAX_MS2 = 1.20
VERTICAL_HOLD_MIN_MS = 250


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
    start_z_ratio: float,
    end_plane_ratio: float,
    end_z_ratio: float,
    vertical_hold_ms: int,
) -> bool:
    """Mirror the firmware's public posture, distance, and hold boundaries."""
    return (
        fused_distance_cm >= DISTANCE_MIN_CM
        and start_z_ratio >= START_HORIZONTAL_Z_MIN_RATIO
        and end_plane_ratio >= VERTICAL_PLANE_MIN_RATIO
        and end_z_ratio <= VERTICAL_Z_MAX_RATIO
        and vertical_hold_ms >= VERTICAL_HOLD_MIN_MS
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
    flex_start = _latest_diagnostic(diagnostics, stage="flex_start")
    flex_complete = _latest_diagnostic(diagnostics, stage="flex_complete")
    distance = _latest_diagnostic(diagnostics, stage="distance_ready")
    vertical_start = _latest_diagnostic(
        diagnostics, stage="vertical_hold_start"
    )
    vertical_ready = _latest_diagnostic(diagnostics, stage="vertical_ready")
    wait_quiet = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("quiet_not_ready",),
    )
    wait_horizontal = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("start_not_horizontal",),
    )
    wait_flex_rate = _latest_diagnostic(
        diagnostics,
        stage="wait_reject",
        reasons=("flex_rate_low",),
    )
    flex_reset = _latest_diagnostic(
        diagnostics,
        stage="reset",
        reasons=("flex_timeout", "incomplete_flex"),
    )
    endpoint_reset = _latest_diagnostic(
        diagnostics,
        stage="reset",
        reasons=("endpoint_not_vertical", "vertical_hold_timeout"),
    )

    conditions: list[ConditionResult] = []

    if flex_start is not None:
        z_ratio = float(flex_start["value2"])
        conditions.append(
            ConditionResult(
                "水平な開始姿勢",
                "PASS",
                f"Z比 {z_ratio:.2f} ≥ {START_HORIZONTAL_Z_MIN_RATIO:.2f}",
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
    elif wait_flex_rate is not None or wait_quiet is not None:
        wait_horizontal_proof = wait_flex_rate or wait_quiet
        assert wait_horizontal_proof is not None
        z_ratio = float(wait_horizontal_proof["value1"])
        conditions.append(
            ConditionResult(
                "水平な開始姿勢",
                "PASS",
                f"Z比 {z_ratio:.2f} ≥ {START_HORIZONTAL_Z_MIN_RATIO:.2f}",
            )
        )
        quiet_passed = wait_flex_rate is not None
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
    elif wait_horizontal is not None:
        z_ratio = float(wait_horizontal["value1"])
        conditions.append(
            ConditionResult(
                "水平な開始姿勢",
                "FAIL",
                f"Z比 {z_ratio:.2f} < {START_HORIZONTAL_Z_MIN_RATIO:.2f}",
            )
        )
        conditions.append(
            ConditionResult("開始前の静止", "NOT_REACHED", "開始姿勢が未成立")
        )
    else:
        conditions.append(
            ConditionResult("水平な開始姿勢", "NOT_REACHED", "判定データなし")
        )
        conditions.append(
            ConditionResult("開始前の静止", "NOT_REACHED", "判定データなし")
        )

    if flex_complete is not None and flex_start is not None:
        start_rate = float(flex_start["value1"])
        angle = float(flex_complete["value1"])
        peak = float(flex_complete["value2"])
        accel = float(flex_complete["value3"])
        conditions.append(
            ConditionResult(
                "肘の屈曲動作",
                "PASS",
                f"開始 {start_rate:.1f} ≥ {FLEX_START_RATE_MIN_DPS:.0f}°/s、"
                f"角度 {angle:.1f} ≥ {FLEX_ANGLE_MIN_DEG:.0f}°、"
                f"peak {peak:.1f} ≥ {FLEX_PEAK_RATE_MIN_DPS:.0f}°/s、"
                f"加速度 {accel:.2f} ≥ {FLEX_ACCEL_MIN_MS2:.2f} m/s²、"
                f"時間 {FLEX_DURATION_MIN_MS}–{FLEX_DURATION_MAX_MS} ms内",
            )
        )
    elif flex_reset is not None:
        conditions.append(
            ConditionResult(
                "肘の屈曲動作",
                "FAIL",
                f"角度 {float(flex_reset['value1']):.1f}°、"
                f"peak {float(flex_reset['value2']):.1f}°/s、"
                f"加速度 {float(flex_reset['value3']):.2f} m/s² "
                f"({flex_reset['reason']})",
            )
        )
    elif flex_start is not None:
        conditions.append(
            ConditionResult("肘の屈曲動作", "FAIL", "屈曲完了条件まで到達せず")
        )
    else:
        conditions.append(
            ConditionResult("肘の屈曲動作", "NOT_REACHED", "屈曲開始を検出せず")
        )

    if distance is not None:
        fused_cm = float(distance["value1"])
        conditions.append(
            ConditionResult(
                "5 cm以上の移動",
                "PASS" if fused_cm >= DISTANCE_MIN_CM else "FAIL",
                f"融合推定 {fused_cm:.1f} cm "
                f"{'≥' if fused_cm >= DISTANCE_MIN_CM else '<'} {DISTANCE_MIN_CM:.1f} cm",
            )
        )
    else:
        conditions.append(
            ConditionResult("5 cm以上の移動", "NOT_REACHED", "距離判定まで到達せず")
        )

    vertical_evidence = vertical_ready or vertical_start
    if vertical_evidence is not None:
        plane_ratio = float(vertical_evidence["value1"])
        z_ratio = float(vertical_evidence["value2"])
        conditions.append(
            ConditionResult(
                "垂直な終了姿勢",
                "PASS",
                f"XY面比 {plane_ratio:.2f} ≥ {VERTICAL_PLANE_MIN_RATIO:.2f}、"
                f"Z比 {z_ratio:.2f} ≤ {VERTICAL_Z_MAX_RATIO:.2f}",
            )
        )
    elif endpoint_reset is not None:
        plane_ratio = float(endpoint_reset["value1"])
        z_ratio = float(endpoint_reset["value2"])
        posture_passed = (
            plane_ratio >= VERTICAL_PLANE_MIN_RATIO
            and z_ratio <= VERTICAL_Z_MAX_RATIO
        )
        conditions.append(
            ConditionResult(
                "垂直な終了姿勢",
                "PASS" if posture_passed else "FAIL",
                f"XY面比 {plane_ratio:.2f} "
                f"{'≥' if plane_ratio >= VERTICAL_PLANE_MIN_RATIO else '<'} "
                f"{VERTICAL_PLANE_MIN_RATIO:.2f}、Z比 {z_ratio:.2f} "
                f"{'≤' if z_ratio <= VERTICAL_Z_MAX_RATIO else '>'} "
                f"{VERTICAL_Z_MAX_RATIO:.2f}",
            )
        )
    else:
        conditions.append(
            ConditionResult("垂直な終了姿勢", "NOT_REACHED", "姿勢判定まで到達せず")
        )

    if vertical_ready is not None:
        hold_ms = float(vertical_ready["value3"])
        conditions.append(
            ConditionResult(
                "垂直位置での静止保持",
                "PASS",
                f"{hold_ms:.0f} ms ≥ {VERTICAL_HOLD_MIN_MS} ms "
                f"（角速度 ≤ {VERTICAL_RATE_MAX_DPS:.0f}°/s・"
                f"線形加速度 ≤ {VERTICAL_LINEAR_ACCEL_MAX_MS2:.2f} m/s²も成立）",
            )
        )
    elif vertical_start is not None or (
        endpoint_reset is not None
        and endpoint_reset["reason"] == "vertical_hold_timeout"
    ):
        conditions.append(
            ConditionResult(
                "垂直位置での静止保持",
                "FAIL",
                f"{VERTICAL_HOLD_MIN_MS} msの連続静止を確認できず",
            )
        )
    else:
        conditions.append(
            ConditionResult("垂直位置での静止保持", "NOT_REACHED", "垂直姿勢が未成立")
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
        if stage == "flex_start":
            suffix = (
                f" plane_rate={v1:.1f}dps start_z_ratio={v2:.2f} "
                f"accel={v3:.2f}m/s^2"
            )
        elif stage == "flex_complete":
            suffix = (
                f" angle={v1:.1f}deg peak={v2:.1f}dps "
                f"accel={v3:.2f}m/s^2"
            )
        elif stage == "vertical_hold_start":
            suffix = (
                f" plane_ratio={v1:.2f} z_ratio={v2:.2f} "
                f"linear_accel={v3:.2f}m/s^2"
            )
        elif stage == "match":
            suffix = (
                f" distance={v1:.1f}cm plane_ratio={v2:.2f} "
                f"hold={v3:.0f}ms"
            )
        elif stage == "distance_ready":
            suffix = (
                f" fused={v1:.1f}cm accel={v2:.1f}cm arc={v3:.1f}cm"
            )
        elif stage == "vertical_ready":
            suffix = (
                f" plane_ratio={v1:.2f} z_ratio={v2:.2f} hold={v3:.0f}ms"
            )
        elif stage == "gyro_bias_ready":
            suffix = (
                f" correction={v1:.1f}dps plane_bias={v2:.1f}dps "
                f"z_bias={v3:.1f}dps"
            )
        elif stage == "wait_reject":
            suffix = (
                f" reason={reason} z_ratio={v1:.2f} "
                f"plane_ratio={v2:.2f} gyro={v3:.1f}dps"
            )
        elif reason in ("flex_timeout", "incomplete_flex"):
            suffix = (
                f" reason={reason} angle={v1:.1f}deg peak={v2:.1f}dps "
                f"accel={v3:.2f}m/s^2"
            )
        elif reason == "distance_too_short":
            suffix = (
                f" reason={reason} fused={v1:.1f}cm "
                f"accel={v2:.1f}cm arc={v3:.1f}cm"
            )
        elif reason in ("endpoint_not_vertical", "vertical_hold_timeout"):
            suffix = (
                f" reason={reason} plane_ratio={v1:.2f} "
                f"z_ratio={v2:.2f} distance={v3:.1f}cm"
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
    elif stage == "vertical_hold_start":
        record["metrics"] = {
            "plane_gravity_ratio": event.value1,
            "z_gravity_ratio": event.value2,
            "linear_accel_ms2": event.value3,
        }
    elif stage == "vertical_ready":
        record["metrics"] = {
            "plane_gravity_ratio": event.value1,
            "z_gravity_ratio": event.value2,
            "vertical_hold_ms": event.value3,
        }
    elif stage == "match":
        record["metrics"] = {
            "fused_distance_cm": event.value1,
            "plane_gravity_ratio": event.value2,
            "vertical_hold_ms": event.value3,
        }
    elif stage == "gyro_bias_ready":
        record["metrics"] = {
            "correction_dps": event.value1,
            "plane_bias_dps": event.value2,
            "z_bias_dps": event.value3,
        }
    elif reason in ("endpoint_not_vertical", "vertical_hold_timeout"):
        record["metrics"] = {
            "plane_gravity_ratio": event.value1,
            "z_gravity_ratio": event.value2,
            "fused_distance_cm": event.value3,
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
        print("HarnessNode 水平開始→屈曲→垂直静止 BLE検証", flush=True)
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
    diag_packet.extend(struct.pack("<fff", 42.0, 75.0, 1.5))
    diag = parse_event_packet(bytes(diag_packet), now=14.0)
    assert diag is not None
    assert diag.diag_stage == 0x02
    assert diag.value1 is not None and math.isclose(diag.value1, 42.0)

    distance_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x05, 0x00]
    )
    distance_packet.extend(struct.pack("<fff", 5.25, 4.75, 9.0))
    distance = parse_event_packet(bytes(distance_packet), now=15.0)
    assert distance is not None
    assert distance.diag_stage == 0x05
    assert "fused=5.2cm" in format_event(distance)
    distance_record = diagnostic_record(distance)
    assert math.isclose(
        distance_record["metrics"]["fused_distance_cm"], 5.25
    )

    vertical_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x06, 0x00]
    )
    vertical_packet.extend(struct.pack("<fff", 0.91, 0.12, 275.0))
    vertical = parse_event_packet(bytes(vertical_packet), now=16.0)
    assert vertical is not None
    assert vertical.diag_stage == 0x06
    assert "hold=275ms" in format_event(vertical)

    bias_packet = bytearray(
        [0x00, 0x55, EVT_GESTURE_DIAG, 0x07, 0x00]
    )
    bias_packet.extend(struct.pack("<fff", 22.0, 21.5, 4.5))
    bias = parse_event_packet(bytes(bias_packet), now=17.0)
    assert bias is not None
    assert bias.diag_stage == 0x07
    assert "correction=22.0dps" in format_event(bias)
    bias_record = diagnostic_record(bias)
    assert math.isclose(bias_record["metrics"]["correction_dps"], 22.0)

    # Firmware boundary semantics for horizontal start and vertical endpoint.
    assert not gesture_gate_eligible(4.9, 0.80, 0.80, 0.45, 250)
    assert gesture_gate_eligible(5.0, 0.80, 0.80, 0.45, 250)
    assert gesture_gate_eligible(5.1, 0.81, 0.81, 0.44, 251)
    assert not gesture_gate_eligible(5.0, 0.79, 0.80, 0.45, 250)
    assert not gesture_gate_eligible(5.0, 0.80, 0.79, 0.45, 250)
    assert not gesture_gate_eligible(5.0, 0.80, 0.80, 0.46, 250)
    assert not gesture_gate_eligible(5.0, 0.80, 0.80, 0.45, 249)
    assert not sequence_reset_seen([])
    assert not sequence_reset_seen([{"stage": "flex_complete"}])
    assert sequence_reset_seen(
        [{"stage": "flex_complete"}, {"stage": "reset"}]
    )

    passing_conditions = build_condition_results(
        [
            {
                "stage": "flex_start",
                "reason": "none",
                "value1": 40.0,
                "value2": 0.90,
                "value3": 2.0,
            },
            {
                "stage": "flex_complete",
                "reason": "none",
                "value1": 90.0,
                "value2": 180.0,
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
                "stage": "vertical_hold_start",
                "reason": "none",
                "value1": 0.95,
                "value2": 0.10,
                "value3": 0.50,
            },
            {
                "stage": "vertical_ready",
                "reason": "none",
                "value1": 0.96,
                "value2": 0.08,
                "value3": 260.0,
            },
        ],
        matched=True,
    )
    assert all(condition.status == "PASS" for condition in passing_conditions)

    rejected_conditions = build_condition_results(
        [
            {
                "stage": "wait_reject",
                "reason": "start_not_horizontal",
                "value1": 0.40,
                "value2": 0.92,
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
        description="HarnessNodeの水平開始→屈曲→垂直静止をBLEイベントで検証"
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
        "--window", type=float, default=7.0, help="GO後の判定秒数 (default: 7)"
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
            "手のひらを水平にして静止し、肘を屈曲して手を垂直位置に留めてください"
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
