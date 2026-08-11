#!/usr/bin/env python3
"""HarnessNode の屈曲→回内ジェスチャーを BLE で対話検証する。

人が画面のカウントダウンに合わせて動作し、ファームウェアから届く
``recording_start`` イベント (0x01) の有無を試行ごとに判定する。

Codex から macOS Terminal に起動して進捗を読み取れるよう、重要な状態は
``CODEX_*`` で始まる一行にも出力する。標準入力は使わない。

Examples:
    venv/bin/python gesture_validator.py --trials 3
    venv/bin/python gesture_validator.py --expect no-match --trials 3 \
        --instruction "回内だけを行ってください"
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
    0x03: "pronation_start",
    0x04: "match",
    0x10: "wait_reject",
    0x80: "reset",
}

DIAG_REASON_NAMES = {
    0x00: "none",
    0x01: "quiet_not_ready",
    0x02: "axis_not_transverse",
    0x03: "flex_rate_low",
    0x11: "flex_timeout",
    0x12: "wrong_order",
    0x13: "incomplete_flex",
    0x14: "pronation_timeout",
    0x15: "return_motion",
}


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


@dataclass
class TrialResult:
    trial: int
    expected: str
    result: str
    matched: bool
    latency_ms: Optional[int]
    motion_active_seen: bool
    motion_settled_seen: bool
    reason: str
    diagnostics: list[dict[str, Any]]


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
                f" transverse={v1:.1f}dps longitudinal={v2:.1f}dps "
                f"accel={v3:.2f}m/s^2"
            )
        elif stage == "flex_complete":
            suffix = (
                f" angle={v1:.1f}deg peak={v2:.1f}dps "
                f"accel={v3:.2f}m/s^2"
            )
        elif stage == "pronation_start":
            suffix = (
                f" x_rate={v1:.1f}dps transverse={v2:.1f}dps "
                f"angle={v3:.1f}deg"
            )
        elif stage == "match":
            suffix = f" angle={v1:.1f}deg peak={v2:.1f}dps samples={v3:.0f}"
        elif stage == "wait_reject":
            suffix = (
                f" reason={reason} transverse={v1:.1f}dps "
                f"longitudinal={v2:.1f}dps norm={v3:.1f}dps"
            )
        elif reason in ("flex_timeout", "incomplete_flex"):
            suffix = (
                f" reason={reason} angle={v1:.1f}deg peak={v2:.1f}dps "
                f"accel={v3:.2f}m/s^2"
            )
        elif reason == "wrong_order":
            suffix = (
                f" reason={reason} flex_angle={v1:.1f}deg "
                f"x_rate={v2:.1f}dps transverse={v3:.1f}dps"
            )
        elif reason == "pronation_timeout":
            suffix = (
                f" reason={reason} angle={v1:.1f}deg peak={v2:.1f}dps "
                f"samples={v3:.0f}"
            )
        elif reason == "return_motion":
            suffix = (
                f" reason={reason} x_direction_relation={v1:.0f} "
                f"x_angle={v2:.1f}deg elapsed={v3:.0f}ms"
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
    return {
        "time": event.wall_time,
        "stage": DIAG_STAGE_NAMES.get(event.diag_stage or 0, "unknown"),
        "reason": DIAG_REASON_NAMES.get(event.diag_reason or 0, "unknown"),
        "value1": event.value1,
        "value2": event.value2,
        "value3": event.value3,
    }


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
            print(f"  {format_event(event)}", flush=True)
            if event.code == EVT_GESTURE_DIAG:
                record = diagnostic_record(event)
                self.all_diagnostics.append(record)
                print(
                    f"CODEX_DIAG stage={record['stage']} reason={record['reason']} "
                    f"v1={record['value1']:.3f} v2={record['value2']:.3f} "
                    f"v3={record['value3']:.3f}",
                    flush=True,
                )
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
            print(f"指定アドレスへ接続: {self.address}", flush=True)
        else:
            print(
                f"'{self.args.device}' をスキャン中 "
                f"({self.args.scan_timeout:.0f}秒)...",
                flush=True,
            )
            target = await BleakScanner.find_device_by_name(
                self.args.device, timeout=self.args.scan_timeout
            )
            if target is None:
                raise RuntimeError(
                    f"デバイス '{self.args.device}' が見つかりません。"
                )
            self.address = target.address

        print(f"検出: {self.address}。接続中...", flush=True)
        self.client = BleakClient(
            target,
            timeout=self.args.connect_timeout,
            disconnected_callback=self._on_disconnected,
        )
        await self.client.connect()
        await self.client.start_notify(AUDIO_TX_UUID, self._on_notify)

        # イベントはprimary接続だけへ送られるため、検証中だけprimaryを引き受ける。
        await self._write_command(0x02)
        print(f"接続完了 (MTU={self.client.mtu_size})。通知を購読しました。", flush=True)
        print(f"CODEX_CONNECTED address={self.address} mtu={self.client.mtu_size}", flush=True)

        # 接続前から録音中なら音声パケットが届く。その場合だけ停止して初期化する。
        await asyncio.sleep(0.8)
        if self.audio_seen.is_set():
            print("既存の録音を検出したため停止します。", flush=True)
            await self._write_command(0x00)
            await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
        self._drain_events()

        if not self.args.skip_preflight:
            await self.preflight()

    async def preflight(self) -> None:
        """BLE writeとevent notifyの往復を実動作の前に検証する。"""
        print("BLEイベント経路をpreflight確認中...", flush=True)
        self._drain_events()
        await self._write_command(0x01)
        started = await self._wait_for_event(EVT_RECORDING_START, timeout=3.0)
        if not started:
            print("CODEX_PREFLIGHT result=FAIL reason=no_recording_start", flush=True)
            raise RuntimeError("preflightでrecording_startを受信できません")

        await self._write_command(0x00)
        stopped = await self._wait_for_event(EVT_RECORDING_STOP, timeout=3.0)
        if not stopped:
            print("CODEX_PREFLIGHT result=FAIL reason=no_recording_stop", flush=True)
            raise RuntimeError("preflightでrecording_stopを受信できません")

        self._drain_events()
        print("preflight成功: BLE start/stopイベントを往復確認しました。", flush=True)
        print("CODEX_PREFLIGHT result=PASS", flush=True)
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
        print(
            f"CODEX_TRIAL trial={trial} state=GO expected={self.args.expect} "
            f"window_s={self.args.window:g}",
            flush=True,
        )

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
            result = TrialResult(
                trial=trial,
                expected=self.args.expect,
                result="FAIL",
                matched=True,
                latency_ms=0,
                motion_active_seen=False,
                motion_settled_seen=False,
                reason="GO表示前にrecording_startを受信",
                diagnostics=[],
            )
            print(f"  結果: FAIL — {result.reason}", flush=True)
            print(
                f"CODEX_TRIAL trial={trial} result=FAIL expected={self.args.expect} "
                "matched=true latency_ms=0 early=true motion_active=false "
                "motion_settled=false",
                flush=True,
            )
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

        if disconnected:
            result = "ERROR"
            reason = "BLE接続が切断されました"
        elif self.args.expect == "match":
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
            reason=reason,
            diagnostics=diagnostics,
        )
        print(f"  結果: {result} — {reason}", flush=True)
        print(
            f"CODEX_TRIAL trial={trial} result={result} expected={self.args.expect} "
            f"matched={str(matched).lower()} latency_ms={latency_ms} "
            f"motion_active={str(motion_active_seen).lower()} "
            f"motion_settled={str(motion_settled_seen).lower()}",
            flush=True,
        )
        return trial_result

    async def run(self) -> int:
        started_at = datetime.now().isoformat(timespec="seconds")
        await self.connect()
        print("", flush=True)
        print("=" * 64, flush=True)
        print("HarnessNode 屈曲→回内ジェスチャー BLE検証", flush=True)
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
        print(
            f"CODEX_SUMMARY result={overall} passed={passed} failed={failed} "
            f"errors={errors} trials={self.args.trials}",
            flush=True,
        )

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
    print("SELF_TEST: PASS", flush=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="HarnessNodeの屈曲→回内ジェスチャーをBLEイベントで検証"
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
            "肘を伸ばして静止し、肘を屈曲してから前腕を回内してください"
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
        print(f"CODEX_SUMMARY result=ERROR reason={type(exc).__name__}", flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
