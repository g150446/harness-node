#!/usr/bin/env python3
"""
Gesture Data Collector for VoiceBridge52

腕を持ち上げる / ダブルクレンチ の2ジェスチャーのデータを収集し、
判別閾値の調整に使用するJSONファイルを生成する。

各ジェスチャーは SETTLED 時に1レコードとして記録する（ピーク値を集計）。
wakeup_count = ジェスチャー開始から SETTLED までのwakeup回数。

Usage:
    python3 gesture_collector.py
    python3 gesture_collector.py --output my_data.json
"""

import asyncio
import struct
import time
import json
import argparse
import statistics
from datetime import datetime
from typing import Optional
from bleak import BleakClient, BleakScanner

DEVICE_NAME   = "VoiceBridge52"
MOTION_TX_UUID = "00000011-0000-1000-8000-00805f9b34fb"
WAKEUP_TX_UUID = "00000013-0000-1000-8000-00805f9b34fb"

# ============================================================
# Collector state
# ============================================================

class Collector:
    def __init__(self):
        self.current_label: str = ""
        self.collecting: bool = False
        self.records: list = []

        # Wakeup tracking
        self._wakeup_log: list = []         # [(timestamp, axes), ...]

        # Per-gesture accumulators (reset on each SETTLED)
        self._gesture_start_time: Optional[float] = None
        self._gesture_activity: float = 0.0
        self._gesture_peak: float = 0.0
        self._gesture_z_excursion: float = 0.0

    def on_wakeup(self, sender, data: bytes):
        if len(data) < 2:
            return
        axes = data[0]
        now = time.time()
        self._wakeup_log.append((now, axes))
        # Trim entries older than 15s
        self._wakeup_log = [(t, a) for t, a in self._wakeup_log if now - t < 15.0]

        if not self.collecting:
            return

        axis_str = (
            ("Z" if axes & 0x04 else "") +
            ("Y" if axes & 0x02 else "") +
            ("X" if axes & 0x01 else "")
        ) or "?"
        t = time.strftime("%H:%M:%S")
        print(f"  {t}  [WAKEUP]  axes={axis_str}")

    def on_motion(self, sender, data: bytes):
        if len(data) < 14 or not self.collecting:
            return

        event_type  = data[0]
        activity    = struct.unpack_from("<f", data, 2)[0]
        peak        = struct.unpack_from("<f", data, 6)[0]
        z_excursion = struct.unpack_from("<f", data, 14)[0] if len(data) >= 18 else 0.0

        t = time.strftime("%H:%M:%S")

        if event_type == 0x01:  # ACTIVE
            # First ACTIVE of this gesture: record start time
            if self._gesture_start_time is None:
                self._gesture_start_time = time.time()
            # Track peak values over entire gesture duration
            if activity > self._gesture_activity:
                self._gesture_activity = activity
            if peak > self._gesture_peak:
                self._gesture_peak = peak
            if z_excursion > self._gesture_z_excursion:
                self._gesture_z_excursion = z_excursion
            print(f"  {t}  [ACTIVE ]  activity={activity:6.3f}  peak={peak:6.3f}  z_exc={z_excursion:6.3f}")

        else:  # SETTLED
            # Count wakeups that arrived AFTER the first ACTIVE event (post-active wakeups)
            if self._gesture_start_time is not None:
                wakeups_after_active = sum(1 for wt, _ in self._wakeup_log
                                           if wt > self._gesture_start_time)
            else:
                wakeups_after_active = 0

            print(f"  {t}  [SETTLED]  activity={self._gesture_activity:6.3f}  "
                  f"peak={self._gesture_peak:6.3f}  z_exc={self._gesture_z_excursion:6.3f}  "
                  f"wakeups_after={wakeups_after_active}")

            # Record one entry per gesture (only if we saw at least one ACTIVE)
            if self._gesture_start_time is not None:
                self.records.append({
                    "label":              self.current_label,
                    "activity":           round(self._gesture_activity, 4),
                    "peak":               round(self._gesture_peak, 4),
                    "z_excursion":        round(self._gesture_z_excursion, 4),
                    "wakeups_after_active": wakeups_after_active,
                    "ts":                 t,
                })

            # Reset for next gesture
            self._gesture_start_time = None
            self._gesture_activity = 0.0
            self._gesture_peak = 0.0
            self._gesture_z_excursion = 0.0


# ============================================================
# Statistics helper
# ============================================================

def summarize(records: list, label: str):
    subset = [r for r in records if r["label"] == label]
    if not subset:
        print(f"  ({label}: データなし)")
        return

    acts   = [r["activity"]    for r in subset]
    peaks  = [r["peak"]        for r in subset]
    zexcs  = [r["z_excursion"] for r in subset]
    wakes  = [r.get("wakeups_after_active", 0) for r in subset]

    print(f"  {label} — {len(subset)}件")
    print(f"    activity         : min={min(acts):.3f}  max={max(acts):.3f}  "
          f"mean={statistics.mean(acts):.3f}  median={statistics.median(acts):.3f}")
    print(f"    peak             : min={min(peaks):.3f}  max={max(peaks):.3f}  "
          f"mean={statistics.mean(peaks):.3f}  median={statistics.median(peaks):.3f}")
    print(f"    z_excursion      : min={min(zexcs):.3f}  max={max(zexcs):.3f}  "
          f"mean={statistics.mean(zexcs):.3f}  median={statistics.median(zexcs):.3f}")
    print(f"    wakeups_after    : min={min(wakes)}  max={max(wakes)}  "
          f"mean={statistics.mean(wakes):.1f}  median={statistics.median(wakes):.1f}")
    w1 = sum(1 for w in wakes if w >= 1)
    print(f"    wakeups_after>=1 : {w1}/{len(subset)} 件")


# ============================================================
# Main
# ============================================================

async def run(output_path: str):
    print("=" * 60)
    print("Gesture Data Collector")
    print("=" * 60)
    print(f"Device: {DEVICE_NAME}")
    print()

    # Scan
    print("Scanning...")
    devices = await BleakScanner.discover(timeout=5.0)
    addr = next((d.address for d in devices if d.name == DEVICE_NAME), None)
    if not addr:
        print("ERROR: デバイスが見つかりません。電源とアドバタイズを確認してください。")
        return

    print(f"Found: {addr}")
    print()

    col = Collector()

    async with BleakClient(addr) as client:
        await client.start_notify(MOTION_TX_UUID, col.on_motion)
        await client.start_notify(WAKEUP_TX_UUID, col.on_wakeup)

        loop = asyncio.get_event_loop()

        # ── Phase 1: 腕を持ち上げる ────────────────────────────
        print("=" * 60)
        print("  Phase 1: 腕を持ち上げるジェスチャー")
        print("=" * 60)
        print("  ボードを手首に付けた状態で、腕をゆっくり持ち上げてください。")
        print("  5〜10回繰り返してください。")
        print()
        print("  >>> 準備ができたら Enter を押してください（収集開始）")
        await loop.run_in_executor(None, input, "")

        col.current_label = "arm_lift"
        col.collecting    = True
        print("  [収集中] ジェスチャーを行ってください...")
        print()

        print("  >>> 終わったら Enter を押してください（収集停止）")
        await loop.run_in_executor(None, input, "")
        col.collecting = False
        arm_count = sum(1 for r in col.records if r["label"] == "arm_lift")
        print(f"  → {arm_count} ジェスチャー記録")
        print()

        await asyncio.sleep(1.0)

        # ── Phase 2: ダブルクレンチ ──────────────────────────────
        print("=" * 60)
        print("  Phase 2: ダブルクレンチジェスチャー")
        print("=" * 60)
        print("  ボードを手首に付けた状態で、手を素早く2回握りしめてください。")
        print("  5〜10回繰り返してください。")
        print()
        print("  >>> 準備ができたら Enter を押してください（収集開始）")
        await loop.run_in_executor(None, input, "")

        col.current_label = "double_clench"
        col.collecting    = True
        print("  [収集中] ジェスチャーを行ってください...")
        print()

        print("  >>> 終わったら Enter を押してください（収集停止）")
        await loop.run_in_executor(None, input, "")
        col.collecting = False
        clench_count = sum(1 for r in col.records if r["label"] == "double_clench")
        print(f"  → {clench_count} ジェスチャー記録")
        print()

        await client.stop_notify(MOTION_TX_UUID)
        await client.stop_notify(WAKEUP_TX_UUID)

    # ── Summary ────────────────────────────────────────────────
    print("=" * 60)
    print("  Summary")
    print("=" * 60)
    summarize(col.records, "arm_lift")
    print()
    summarize(col.records, "double_clench")
    print()

    # Save JSON
    output = {
        "collected_at": datetime.now().isoformat(),
        "device": DEVICE_NAME,
        "records": col.records,
    }
    with open(output_path, "w") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    print(f"  データを保存しました: {output_path}")
    print()
    print("このファイルの内容をClaude Codeに貼り付けると閾値を調整します。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="gesture_data.json",
                        help="出力JSONファイル名 (default: gesture_data.json)")
    args = parser.parse_args()

    try:
        asyncio.run(run(args.output))
    except KeyboardInterrupt:
        print("\nInterrupted.")
