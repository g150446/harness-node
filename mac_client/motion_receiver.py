#!/usr/bin/env python3
"""
MotionBridge BLE receiver.

Connects to a MotionBridge device and prints motion events received
via BLE notifications.

Packet format (14 bytes):
  [0]    event_type : 0x01 = motion detected
  [1]    count      : uint8, wraps at 255
  [2:6]  activity   : float32 LE
  [6:10] peak       : float32 LE
  [10:14] elapsed_ms: uint32 LE (milliseconds since last motion event)
"""

import asyncio
import struct
import sys

from bleak import BleakClient, BleakScanner

DEVICE_NAME      = "MotionBridge"
SERVICE_UUID     = "00000010-0000-1000-8000-00805f9b34fb"
TX_CHAR_UUID     = "00000011-0000-1000-8000-00805f9b34fb"


def handle_notification(sender, data: bytearray) -> None:
    if len(data) < 14:
        print(f"[WARN] Short packet ({len(data)} bytes): {data.hex()}")
        return

    event_type, count = data[0], data[1]
    activity, = struct.unpack_from("<f", data, 2)
    peak,     = struct.unpack_from("<f", data, 6)
    elapsed_ms, = struct.unpack_from("<I", data, 10)

    if event_type == 0x01:
        print(f"[MOTION] count={count:3d}  elapsed={elapsed_ms:6d} ms  activity={activity:.3f}  peak={peak:.3f}")
    else:
        print(f"[UNKNOWN event=0x{event_type:02x}] {data.hex()}")


async def run() -> None:
    print(f"Scanning for '{DEVICE_NAME}'...")

    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        print(f"ERROR: '{DEVICE_NAME}' not found. Is the firmware running and advertising?")
        sys.exit(1)

    print(f"Found: {device.name} [{device.address}]")
    print("Connecting...")

    async with BleakClient(device) as client:
        print(f"Connected. Subscribing to TX characteristic {TX_CHAR_UUID}")
        await client.start_notify(TX_CHAR_UUID, handle_notification)
        print("Listening for motion events. Press Ctrl+C to stop.\n")
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            await client.stop_notify(TX_CHAR_UUID)
            print("\nDisconnected.")


if __name__ == "__main__":
    asyncio.run(run())
