#!/usr/bin/env python3
"""
Voice Bridge BLE - Mac Client for nRF52840 Sense (VoiceBridge52)

Connects to XIAO nRF52840 Sense running nrf52-voice firmware.
Receives audio notifications and motion event notifications via BLE.

Auto mode: motion detection triggers WAV recording start/stop automatically.
Manual mode: 'r'/'s' commands send BLE RX write to start/stop audio.
"""

import asyncio
import logging
import os
import struct
import sys
import wave
import time
from typing import Optional
from datetime import datetime

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Please install bleak: pip install bleak")
    sys.exit(1)


# ============================================================================
# Configuration
# ============================================================================

DEVICE_NAME = "VoiceBridge52"

# Audio Service UUIDs
AUDIO_TX_UUID = "00000002-0000-1000-8000-00805f9b34fb"  # Notify
AUDIO_RX_UUID = "00000003-0000-1000-8000-00805f9b34fb"  # Write

# Motion Service UUIDs
MOTION_TX_UUID = "00000011-0000-1000-8000-00805f9b34fb"  # Notify
WAKEUP_TX_UUID = "00000013-0000-1000-8000-00805f9b34fb"  # Notify

BLE_MTU_SIZE = 512

# Audio config (must match firmware)
SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2   # 16-bit
CHANNELS = 1

PACKET_HEADER_SIZE = 2
PACKET_SYNC_BYTE = 0xAA


# ============================================================================
# Logging
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# WAV Recorder
# ============================================================================

class WAVRecorder:
    def __init__(self, sample_rate=SAMPLE_RATE, sample_width=SAMPLE_WIDTH, channels=CHANNELS):
        self.sample_rate = sample_rate
        self.sample_width = sample_width
        self.channels = channels
        self.wav_file: Optional[wave.Wave_write] = None
        self.is_recording = False
        self.start_time: Optional[float] = None
        self.bytes_recorded = 0
        self.last_duration = 0.0
        self._filename: Optional[str] = None

    def start_recording(self, filename: Optional[str] = None) -> bool:
        if self.is_recording:
            logger.warning("Already recording")
            return False

        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, 'output')
        os.makedirs(output_dir, exist_ok=True)

        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(output_dir, f"nrf52voice_{ts}.wav")
        elif not os.path.isabs(filename):
            filename = os.path.join(output_dir, filename)

        try:
            self.wav_file = wave.open(filename, 'wb')
            self.wav_file.setnchannels(self.channels)
            self.wav_file.setsampwidth(self.sample_width)
            self.wav_file.setframerate(self.sample_rate)
            self.is_recording = True
            self.start_time = time.time()
            self.bytes_recorded = 0
            self.last_duration = 0.0
            self._filename = filename
            print(f"  [WAV] Recording: {os.path.basename(filename)}")
            return True
        except Exception as e:
            logger.error(f"Failed to start recording: {e}")
            return False

    def stop_recording(self):
        if not self.is_recording:
            return
        self.last_duration = time.time() - (self.start_time or time.time())
        if self.wav_file:
            self.wav_file.close()
            self.wav_file = None
        self.is_recording = False
        print(f"  [WAV] Saved {self.last_duration:.1f}s ({self.bytes_recorded} bytes) → {os.path.basename(self._filename or '')}")

    def write_samples(self, pcm_data: bytes):
        if not self.is_recording or not self.wav_file:
            return
        try:
            self.wav_file.writeframes(pcm_data)
            self.bytes_recorded += len(pcm_data)
        except Exception as e:
            logger.error(f"Error writing samples: {e}")

    def get_duration(self) -> float:
        if self.is_recording and self.start_time:
            return time.time() - self.start_time
        return self.last_duration


# ============================================================================
# BLE Client
# ============================================================================

# ============================================================================
# Gesture Classifier
# ============================================================================
#
# 判別ロジック（gesture_collector.py の実測データから導出）
#
# 判別アルゴリズム（SETTLED時に評価）:
#
#   ダブルクレンチ: 2回目のクレンチがMOTION ACTIVE検出より後に来る
#     → ACTIVE後にwakeupが1回以上発生する
#
#   arm_lift: 腕を持ち上げる動作でwakeupが複数発生することがあるが、
#     それらはすべてACTIVEの前（動作開始時の同一モーションから）
#     → ACTIVE後にwakeupは発生しない
#
def classify_gesture(activity: float, peak: float,
                     z_excursion: float = 0.0,
                     wakeups_after_active: int = 0) -> str:
    """
    Returns: "ARM_LIFT" | "DOUBLE_CLENCH"

    判定ロジック（SETTLED時のみ呼び出す）:
      MOTION ACTIVE受信後にwakeupが1回以上あれば DOUBLE_CLENCH。
      arm_liftではACTIVE後にwakeupは発生しない。
    """
    if wakeups_after_active >= 1:
        return "DOUBLE_CLENCH"
    return "ARM_LIFT"


# ============================================================================
# BLE Client
# ============================================================================

class VoiceBridge52Client:
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.recorder = WAVRecorder()
        self.is_connected = False
        self._last_seq = -1
        self._packets_received = 0
        self._packet_loss = 0

        # Auto mode: motion triggers recording
        self.auto_mode = False

        # Disconnect tracking (battery death vs firmware crash)
        self._connect_time: Optional[float] = None
        self._disconnect_time: Optional[float] = None
        self._address: Optional[str] = None

        # Gesture classification state
        self._last_wakeup_axes: int = 0
        self._last_wakeup_time: float = 0.0
        self._wakeup_log: list = []          # [timestamp, ...] — all wakeup times
        self._motion_start_time: Optional[float] = None  # time of first ACTIVE in gesture

    async def find_device(self) -> Optional[str]:
        print(f"Scanning for '{DEVICE_NAME}'...")
        devices = await BleakScanner.discover(timeout=5.0)
        for d in devices:
            if d.name == DEVICE_NAME:
                print(f"  Found: {d.address}")
                return d.address
        print("  Device not found.")
        return None

    def _on_disconnected(self, client: "BleakClient"):
        elapsed = time.time() - (self._connect_time or time.time())
        self._disconnect_time = time.time()
        self.is_connected = False
        print(f"\n  [DISCONNECT] After {elapsed:.0f}s connected")
        if self.recorder.is_recording:
            self.recorder.stop_recording()

    async def connect(self, address: str):
        print(f"Connecting to {address}...")
        self._address = address
        self.client = BleakClient(
            address,
            mtu_size=BLE_MTU_SIZE,
            timeout=10.0,
            disconnected_callback=self._on_disconnected,
        )
        await self.client.connect()
        self._connect_time = time.time()
        self.is_connected = True
        print(f"  Connected (MTU={self.client.mtu_size})")

        await self.client.start_notify(AUDIO_TX_UUID, self._on_audio)
        await self.client.start_notify(MOTION_TX_UUID, self._on_motion)
        await self.client.start_notify(WAKEUP_TX_UUID, self._on_wakeup)
        print("  Notifications enabled (audio + motion + wakeup)")
        await asyncio.sleep(0.5)

    async def disconnect(self):
        if self.client:
            try:
                await self.client.stop_notify(AUDIO_TX_UUID)
                await self.client.stop_notify(MOTION_TX_UUID)
                await self.client.stop_notify(WAKEUP_TX_UUID)
            except Exception:
                pass
            await self.client.disconnect()
            self.is_connected = False
            print("Disconnected.")

    def _on_audio(self, sender, data: bytes):
        if len(data) < PACKET_HEADER_SIZE:
            return
        seq = data[0]
        if data[1] != PACKET_SYNC_BYTE:
            return
        if self._last_seq >= 0:
            expected = (self._last_seq + 1) % 256
            if seq != expected:
                self._packet_loss += 1
        self._last_seq = seq
        self._packets_received += 1
        self.recorder.write_samples(data[PACKET_HEADER_SIZE:])

    async def _on_motion(self, sender, data: bytes):
        if len(data) < 14:
            return
        event_type = data[0]   # 0x01 = motion active, 0x00 = motion settled
        count = data[1]
        activity = struct.unpack_from('<f', data, 2)[0]
        peak = struct.unpack_from('<f', data, 6)[0]
        elapsed_ms = struct.unpack_from('<I', data, 10)[0]
        z_excursion = struct.unpack_from('<f', data, 14)[0] if len(data) >= 18 else 0.0

        state = "ACTIVE" if event_type == 0x01 else "SETTLED"

        if event_type == 0x01:
            # Record start of gesture (first ACTIVE only)
            if self._motion_start_time is None:
                self._motion_start_time = time.time()
            print(f"  [MOTION {state}] count={count} activity={activity:.2f} peak={peak:.2f} "
                  f"z_exc={z_excursion:.2f} elapsed={elapsed_ms}ms")
        else:
            # SETTLED: count wakeups that arrived AFTER the first ACTIVE event
            if self._motion_start_time is not None:
                wakeups_after_active = sum(1 for t in self._wakeup_log
                                           if t > self._motion_start_time)
            else:
                wakeups_after_active = 0
            gesture = classify_gesture(activity, peak, z_excursion, wakeups_after_active)
            print(f"  [MOTION {state}] count={count} activity={activity:.2f} peak={peak:.2f} "
                  f"z_exc={z_excursion:.2f} elapsed={elapsed_ms}ms "
                  f"wakeups_after={wakeups_after_active} → [{gesture}]")
            self._motion_start_time = None

        if self.auto_mode:
            if event_type == 0x01 and not self.recorder.is_recording:
                print("  [AUTO] Motion detected → start recording")
                self.recorder.start_recording()
                try:
                    if self.client and self.client.is_connected:
                        await self.client.write_gatt_char(AUDIO_RX_UUID, bytes([0x01]))
                except Exception as e:
                    logger.error(f"BLE start failed: {e}")
            elif event_type == 0x00 and self.recorder.is_recording:
                print("  [AUTO] Motion settled → stop recording")
                self.recorder.stop_recording()
                try:
                    if self.client and self.client.is_connected:
                        await self.client.write_gatt_char(AUDIO_RX_UUID, bytes([0x00]))
                except Exception as e:
                    logger.error(f"BLE stop failed: {e}")

    def _on_wakeup(self, sender, data: bytes):
        if len(data) < 2:
            return
        axes = data[0]
        count = data[1]
        now = time.time()
        self._last_wakeup_axes = axes
        self._last_wakeup_time = now
        self._wakeup_log.append(now)
        # Trim entries older than 15s
        self._wakeup_log = [t for t in self._wakeup_log if now - t < 15.0]
        axis_str = "".join([
            "Z" if axes & 0x04 else "",
            "Y" if axes & 0x02 else "",
            "X" if axes & 0x01 else "",
        ]) or "?"
        print(f"  [WAKEUP] axes={axis_str} count={count}")

    async def _send_ble_start(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.write_gatt_char(AUDIO_RX_UUID, bytes([0x01]))
        except Exception as e:
            logger.error(f"BLE start failed: {e}")

    async def _send_ble_stop(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.write_gatt_char(AUDIO_RX_UUID, bytes([0x00]))
        except Exception as e:
            logger.error(f"BLE stop failed: {e}")

    def start_recording(self):
        self.recorder.start_recording()
        t = asyncio.create_task(self._send_ble_start())
        t.add_done_callback(lambda _: None)  # keep reference alive

    def stop_recording(self):
        self.recorder.stop_recording()
        t = asyncio.create_task(self._send_ble_stop())
        t.add_done_callback(lambda _: None)  # keep reference alive

    def print_stats(self):
        print(f"  Packets received: {self._packets_received}")
        print(f"  Packets lost:     {self._packet_loss}")
        if self._packets_received > 0:
            rate = self._packet_loss / (self._packets_received + self._packet_loss) * 100
            print(f"  Loss rate:        {rate:.1f}%")
        print(f"  Recording:        {'YES ({:.1f}s)'.format(self.recorder.get_duration()) if self.recorder.is_recording else 'NO'}")
        print(f"  Auto mode:        {'ON' if self.auto_mode else 'OFF'}")


# ============================================================================
# Interactive Control
# ============================================================================

async def interactive_control(client: VoiceBridge52Client):
    print()
    print("=" * 60)
    print("VoiceBridge52 - nRF52840 Sense Recording Control")
    print("=" * 60)
    print("Commands:")
    print("  r  - Manual: start recording")
    print("  s  - Manual: stop recording")
    print("  a  - Toggle auto mode (motion-triggered recording)")
    print("  t  - Show status")
    print("  q  - Quit")
    print()

    loop = asyncio.get_event_loop()

    while True:
        try:
            cmd = await loop.run_in_executor(None, input, "Command: ")
            cmd = cmd.strip().lower()

            if cmd in ('q', 'quit', 'exit'):
                break
            elif cmd == 'r':
                client.start_recording()
            elif cmd == 's':
                client.stop_recording()
            elif cmd == 'a':
                client.auto_mode = not client.auto_mode
                print(f"  Auto mode: {'ON' if client.auto_mode else 'OFF'}")
            elif cmd == 't':
                client.print_stats()
            elif cmd == '':
                client.print_stats()
            else:
                print(f"  Unknown command: '{cmd}'")

        except EOFError:
            await asyncio.sleep(0.5)
        except Exception as e:
            logger.error(f"Error: {e}")
            await asyncio.sleep(0.5)


# ============================================================================
# Main
# ============================================================================

async def main():
    print("=" * 60)
    print("VoiceBridge52 - Mac Client")
    print("=" * 60)
    print()

    client = VoiceBridge52Client()

    try:
        address = await client.find_device()
        if not address:
            print("Device not found. Make sure XIAO nRF52840 Sense is advertising 'VoiceBridge52'.")
            return

        await client.connect(address)

        # Run interactive control; it exits on 'q' or when the connection drops
        control_task = asyncio.create_task(interactive_control(client))

        # Wait for either the user quitting or an unexpected disconnect
        while not control_task.done():
            if not client.is_connected:
                print("  [RECONNECT] Waiting for device to reappear...")
                control_task.cancel()
                try:
                    await control_task
                except asyncio.CancelledError:
                    pass

                # Poll until device reappears (battery recharge / reboot)
                reconnect_address = None
                while reconnect_address is None:
                    await asyncio.sleep(3.0)
                    print(f"  [RECONNECT] Scanning for '{DEVICE_NAME}'...")
                    try:
                        reconnect_address = await client.find_device()
                    except Exception:
                        pass

                gap = time.time() - (client._disconnect_time or time.time())
                print(f"  [RECONNECT] Device found after {gap:.0f}s offline — reconnecting")
                await client.connect(reconnect_address)
                control_task = asyncio.create_task(interactive_control(client))

            await asyncio.sleep(0.5)

        await control_task  # propagate any exception

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if client.recorder.is_recording:
            client.stop_recording()
            await asyncio.sleep(0.5)
        client.print_stats()
        await client.disconnect()
        print("Done.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
