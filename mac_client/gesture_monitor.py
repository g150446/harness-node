#!/usr/bin/env python3
"""
gesture_monitor.py — BLE接続して IMU イベントを表示する

イベントコード:
  0x10  motion_active    動き検出開始（+z値 f32）
  0x11  motion_settled   動き収束（+z値 f32）
  0x01  recording_start  録音開始
  0x02  recording_stop   録音停止
"""

import asyncio
import struct
import sys
from datetime import datetime
from bleak import BleakClient, BleakScanner

DEVICE_NAME   = "XIAOVoice"
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"


def on_notify(sender, data: bytes):
    if len(data) < 3 or data[0] != 0x00 or data[1] != 0x55:
        return

    code = data[2]
    ts   = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    if code == 0x10 and len(data) >= 7:
        z = struct.unpack_from('<f', data, 3)[0]
        print(f"[{ts}]  motion_active   z={z:+.2f}")
    elif code == 0x11 and len(data) >= 7:
        z = struct.unpack_from('<f', data, 3)[0]
        print(f"[{ts}]  motion_settled  z={z:+.2f}")
    elif code == 0x01:
        print(f"[{ts}]  recording_start")
    elif code == 0x02:
        print(f"[{ts}]  recording_stop")
    else:
        print(f"[{ts}]  unknown 0x{code:02x}  raw={data.hex()}")


async def main():
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
    if device is None:
        print(f"Device '{DEVICE_NAME}' not found.")
        sys.exit(1)

    print(f"Found: {device.address}. Connecting...")
    async with BleakClient(device) as client:
        print("Connected.\n")
        await client.start_notify(AUDIO_TX_UUID, on_notify)
        try:
            while client.is_connected:
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            pass
    print("Disconnected.")


if __name__ == "__main__":
    asyncio.run(main())
