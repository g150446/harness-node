#!/usr/bin/env python3
"""
speaker_client.py — BLE audio streaming client for SpeakerBridge (nRF54L15)

Usage:
    python3 speaker_client.py [URL]

Default URL: http://vis.media-ice.musicradio.com/CapitalMP3

Dependencies:
    pip install bleak
    brew install ffmpeg
"""

import asyncio
import subprocess
import sys
import time

from bleak import BleakClient, BleakScanner

# ─── Configuration ────────────────────────────────────────────────────────────

DEFAULT_URL = "http://vis.media-ice.musicradio.com/CapitalMP3"

DEVICE_NAME      = "SpeakerBridge"
AUDIO_RX_UUID    = "00000021-0000-1000-8000-00805f9b34fb"

SAMPLE_RATE      = 16000   # Hz fed to ffmpeg
# nRF54L15 I2S actual PCM rate is 15873 Hz (hardware MCLK constraint).
# Pace BLE sends to match 15873 Hz so the firmware ring buffer stays stable.
I2S_ACTUAL_HZ    = 15873
CHUNK_BYTES      = 238     # PCM bytes per BLE packet (total = 240, within MTU-3=241)
PACKET_INTERVAL  = CHUNK_BYTES / (I2S_ACTUAL_HZ * 2)  # 7.498 ms → 133.4 pkt/s

QUEUE_MAXSIZE    = 30      # ~0.2 s pre-buffer — prevents initial burst overflow

# ─── BLE scan ─────────────────────────────────────────────────────────────────

async def scan_for_device(name: str, timeout: float = 15.0):
    print(f"Scanning for '{name}'...")
    device = await BleakScanner.find_device_by_name(name, timeout=timeout)
    if device is None:
        raise RuntimeError(f"Device '{name}' not found within {timeout}s")
    print(f"Found: {device.address}")
    return device

# ─── ffmpeg reader ────────────────────────────────────────────────────────────

async def ffmpeg_reader(url: str, queue: asyncio.Queue):
    """Run ffmpeg as a subprocess, push 240-byte PCM chunks into queue."""
    cmd = [
        "ffmpeg",
        "-reconnect", "1",
        "-reconnect_streamed", "1",
        "-reconnect_delay_max", "5",
        "-i", url,
        "-f", "s16le",
        "-ar", str(SAMPLE_RATE),
        "-ac", "1",
        "-loglevel", "error",
        "pipe:1",
    ]

    print(f"Starting ffmpeg: {url}")
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    loop = asyncio.get_event_loop()

    try:
        while True:
            chunk = await loop.run_in_executor(None, proc.stdout.read, CHUNK_BYTES)
            if not chunk:
                print("ffmpeg EOF — stream ended")
                break
            if len(chunk) < CHUNK_BYTES:
                # Pad last chunk
                chunk = chunk + b"\x00" * (CHUNK_BYTES - len(chunk))
            await queue.put(chunk)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

# ─── BLE sender ───────────────────────────────────────────────────────────────

async def ble_sender(client: BleakClient, queue: asyncio.Queue):
    """Pull chunks from queue and send via BLE Write Without Response."""
    seq = 0
    packets_sent = 0
    bytes_sent = 0
    report_time = time.monotonic()

    deadline = time.monotonic()

    while True:
        chunk = await queue.get()

        packet = bytes([seq & 0xFF, 0xAA]) + chunk
        await client.write_gatt_char(AUDIO_RX_UUID, packet, response=False)

        seq = (seq + 1) & 0xFF
        packets_sent += 1
        bytes_sent += len(chunk)

        # Deadline-based pacing
        deadline += PACKET_INTERVAL
        now = time.monotonic()
        if deadline > now:
            await asyncio.sleep(deadline - now)

        # Stats every 5 seconds
        now = time.monotonic()
        if now - report_time >= 5.0:
            elapsed = now - report_time
            pps = packets_sent / elapsed
            kbps = bytes_sent / elapsed / 1024
            qsize = queue.qsize()
            print(f"Streaming: {pps:.1f} pkt/s, {kbps:.1f} KB/s  (queue={qsize})")
            packets_sent = 0
            bytes_sent = 0
            report_time = now

# ─── Main ─────────────────────────────────────────────────────────────────────

async def main():
    url = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URL

    device = await scan_for_device(DEVICE_NAME)

    async with BleakClient(device, mtu_size=244) as client:
        print(f"Connected to {DEVICE_NAME} (MTU={client.mtu_size})")

        queue: asyncio.Queue = asyncio.Queue(maxsize=QUEUE_MAXSIZE)

        reader_task = asyncio.create_task(ffmpeg_reader(url, queue))
        sender_task = asyncio.create_task(ble_sender(client, queue))

        try:
            await asyncio.gather(reader_task, sender_task)
        except asyncio.CancelledError:
            pass
        finally:
            reader_task.cancel()
            sender_task.cancel()
            await asyncio.gather(reader_task, sender_task, return_exceptions=True)
            print("Stopped.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except RuntimeError as e:
        print(f"Error: {e}")
        sys.exit(1)
