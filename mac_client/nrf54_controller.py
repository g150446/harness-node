#!/usr/bin/env python3
"""
Voice Bridge BLE - Mac Client for nRF54L15

Receives audio from XIAO nRF54L15 Sense via BLE and saves to WAV file.
Uses serial commands to control recording start/stop.

This is compatible with the nRF54L15 firmware that sends PCM data
(same format as ESP32).
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

import serial
import serial.tools.list_ports

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Please install bleak: pip install bleak")
    sys.exit(1)


# ============================================================================
# Configuration
# ============================================================================

# BLE Device name
DEVICE_NAME = "VoiceBridge"

# BLE Service and Characteristic UUIDs
SERVICE_UUID = "00000000-0000-0000-0000-000000000000"
TX_CHARACTERISTIC_UUID = "00000002-0000-1000-8000-00805f9b34fb"  # XIAO -> Mac (Notify)
RX_CHARACTERISTIC_UUID = "00000003-0000-1000-8000-00805f9b34fb"  # Mac -> XIAO (Write)

# BLE configuration
BLE_MTU_SIZE = 512

# Audio configuration (must match XIAO firmware)
SAMPLE_RATE = 16000  # 16kHz
SAMPLE_WIDTH = 2  # 16-bit
CHANNELS = 1

# Recording packet format: [seq_num][sync_byte][data...]
PACKET_HEADER_SIZE = 2
PACKET_SYNC_BYTE = 0xAA


# ============================================================================
# Logging Setup
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# Serial Port Functions
# ============================================================================

def find_xiao_serial_port():
    """Find XIAO nRF54L15 serial port."""
    ports = serial.tools.list_ports.comports()

    for port in ports:
        # XIAO nRF54L15 uses CMSIS-DAP
        if 'XIAO' in (port.manufacturer or '') or \
           'nRF' in (port.description or '') or \
           'CMSIS' in (port.description or '') or \
           'usbmodem' in port.device:
            return port.device

    return None


# ============================================================================
# WAV File Recorder
# ============================================================================

class WAVRecorder:
    """Records audio data to WAV file."""

    def __init__(self, sample_rate: int = SAMPLE_RATE,
                 sample_width: int = SAMPLE_WIDTH,
                 channels: int = CHANNELS):
        self.sample_rate = sample_rate
        self.sample_width = sample_width
        self.channels = channels
        self.wav_file: Optional[wave.Wave_write] = None
        self.is_recording = False
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.bytes_recorded = 0
        self.last_duration = 0.0  # Store last recording duration

    def start_recording(self, filename: Optional[str] = None):
        """Start recording to WAV file."""
        if self.is_recording:
            logger.warning("Already recording")
            return False

        # Create output directory if it doesn't exist
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, 'output')
        os.makedirs(output_dir, exist_ok=True)

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(output_dir, f"recording_{timestamp}.wav")
        elif not os.path.isabs(filename):
            # If relative path, save to output directory
            filename = os.path.join(output_dir, filename)

        try:
            self.wav_file = wave.open(filename, 'wb')
            self.wav_file.setnchannels(self.channels)
            self.wav_file.setsampwidth(self.sample_width)
            self.wav_file.setframerate(self.sample_rate)

            self.is_recording = True
            self.start_time = time.time()
            self.end_time = None
            self.bytes_recorded = 0
            self.last_duration = 0.0

            logger.info(f"Started recording: {filename}")
            return True

        except Exception as e:
            logger.error(f"Failed to start recording: {e}")
            return False

    def stop_recording(self):
        """Stop recording."""
        if not self.is_recording:
            return

        self.end_time = time.time()
        self.last_duration = self.end_time - self.start_time if self.start_time else 0.0
        self.is_recording = False

        if self.wav_file:
            self.wav_file.close()
            self.wav_file = None

        logger.info(f"Stopped recording. Duration: {self.last_duration:.1f}s, Bytes: {self.bytes_recorded}")

    def write_samples(self, pcm_data: bytes):
        """Write PCM samples to WAV file."""
        if not self.is_recording or not self.wav_file:
            return

        try:
            self.wav_file.writeframes(pcm_data)
            self.bytes_recorded += len(pcm_data)
        except Exception as e:
            logger.error(f"Error writing samples: {e}")

    def get_recording_duration(self) -> float:
        """Get current recording duration in seconds."""
        if self.is_recording and self.start_time:
            return time.time() - self.start_time
        return self.last_duration


# ============================================================================
# BLE Client
# ============================================================================

class VoiceBridgeClient:
    """BLE client for Voice Bridge nRF54L15."""

    def __init__(self, device_name: str = DEVICE_NAME):
        self.device_name = device_name
        self.client: Optional[BleakClient] = None
        self.recorder = WAVRecorder()
        self.is_connected = False
        self._last_seq_num = -1
        self._packet_loss_count = 0
        self._packets_received = 0
        self._serial_port: Optional[serial.Serial] = None

    async def find_device(self) -> Optional[str]:
        """Find the Voice Bridge device."""
        logger.info(f"Scanning for device: {self.device_name}")

        devices = await BleakScanner.discover(timeout=5.0)

        for device in devices:
            if device.name == self.device_name:
                logger.info(f"Found device: {device.address}")
                return device.address

        logger.error("Device not found")
        return None

    async def connect(self, address: str):
        """Connect to the device."""
        logger.info(f"Connecting to {address}...")

        self.client = BleakClient(
            address,
            mtu_size=BLE_MTU_SIZE,
            timeout=10.0
        )

        await self.client.connect()
        self.is_connected = True

        logger.info(f"Connected: {self.client.is_connected}")
        logger.info(f"MTU size: {self.client.mtu_size}")

        # Start notifications
        await self.client.start_notify(
            TX_CHARACTERISTIC_UUID,
            self.notification_handler
        )
        logger.info("Notifications enabled")

        # Wait for BLE to stabilize
        await asyncio.sleep(0.5)

    async def disconnect(self):
        """Disconnect from the device."""
        if self.client:
            await self.client.stop_notify(TX_CHARACTERISTIC_UUID)
            await self.client.disconnect()
            self.is_connected = False
            logger.info("Disconnected")

    def notification_handler(self, sender, data: bytes):
        """Handle incoming BLE notifications (audio data)."""
        if not self.is_connected:
            return

        # Check packet format
        if len(data) < PACKET_HEADER_SIZE:
            return

        seq_num = data[0]
        sync_byte = data[1]

        # Verify sync byte
        if sync_byte != PACKET_SYNC_BYTE:
            logger.warning(f"Invalid sync byte: {sync_byte:02X}")
            return

        # Check sequence number for packet loss detection
        if self._last_seq_num >= 0:
            expected_seq = (self._last_seq_num + 1) % 256
            if seq_num != expected_seq:
                self._packet_loss_count += 1
                logger.debug(f"Packet loss: expected={expected_seq}, got={seq_num}, total_lost={self._packet_loss_count}")
        self._last_seq_num = seq_num
        self._packets_received += 1

        # Extract PCM payload (skip header)
        pcm_data = data[PACKET_HEADER_SIZE:]

        # Write to WAV file
        self.recorder.write_samples(pcm_data)

    def send_serial_command(self, command: str):
        """Send command via serial."""
        if self._serial_port and self._serial_port.is_open:
            self._serial_port.write(command.encode())
            logger.info(f"Sent serial command: {command}")
        else:
            logger.warning("Serial port not open")

    def open_serial(self, port: Optional[str] = None):
        """Open serial port for control commands."""
        if port is None:
            port = find_xiao_serial_port()

        if port is None:
            logger.warning("No serial port found")
            return False

        try:
            self._serial_port = serial.Serial(port, 115200, timeout=1)
            time.sleep(0.5)  # Wait for connection
            logger.info(f"Serial port opened: {port}")
            return True
        except Exception as e:
            logger.error(f"Failed to open serial port: {e}")
            return False

    def close_serial(self):
        """Close serial port."""
        if self._serial_port:
            self._serial_port.close()
            self._serial_port = None

    def start_recording(self):
        """Start recording via BLE command."""
        if self.client and self.client.is_connected:
            # Start local WAV recording
            self.recorder.start_recording()

            # Send start command (0x01)
            asyncio.create_task(
                self.client.write_gatt_char(RX_CHARACTERISTIC_UUID, bytes([0x01]))
            )
            logger.info("Sent start recording command via BLE")

    def stop_recording(self):
        """Stop recording via BLE command."""
        if self.client and self.client.is_connected:
            # Stop local WAV recording
            self.recorder.stop_recording()

            # Send stop command (0x00)
            asyncio.create_task(
                self.client.write_gatt_char(RX_CHARACTERISTIC_UUID, bytes([0x00]))
            )
            logger.info("Sent stop recording command via BLE")

    def print_stats(self):
        """Print recording statistics."""
        logger.info(f"Packets received: {self._packets_received}")
        logger.info(f"Packets lost: {self._packet_loss_count}")
        if self._packets_received > 0:
            loss_rate = self._packet_loss_count / (self._packets_received + self._packet_loss_count) * 100
            logger.info(f"Packet loss rate: {loss_rate:.1f}%")
        logger.info(f"Recording duration: {self.recorder.get_recording_duration():.1f}s")
        logger.info(f"Bytes recorded: {self.recorder.bytes_recorded}")


# ============================================================================
# Interactive Control
# ============================================================================

async def interactive_control(client: VoiceBridgeClient):
    """Interactive control loop."""
    print("\n" + "=" * 60)
    print("Voice Bridge BLE - nRF54L15 Recording Control")
    print("=" * 60)
    print("\nCommands:")
    print("  r / start  - Start recording")
    print("  s / stop   - Stop recording")
    print("  R / record - Start recording via serial")
    print("  S / stop_serial - Stop recording via serial")
    print("  t / status - Show status")
    print("  q / quit   - Quit")
    print()

    # Try to open serial port
    if client.open_serial():
        print("Serial control: Available")
    else:
        print("Serial control: Not available")
    print()

    # Show initial status
    client.print_stats()
    print()

    loop = asyncio.get_event_loop()

    while True:
        try:
            command = await loop.run_in_executor(None, input, "Command: ")
            command = command.strip().lower()

            if command in ['q', 'quit', 'exit']:
                break
            elif command in ['r', 'start']:
                client.start_recording()
            elif command in ['s', 'stop']:
                client.stop_recording()
            elif command in ['R', 'record']:
                client.send_serial_command('r')
            elif command in ['S', 'stop_serial']:
                client.send_serial_command('s')
            elif command in ['t', 'status']:
                client.print_stats()
                if client.recorder.is_recording:
                    print(f"  Recording: YES ({client.recorder.get_recording_duration():.1f}s)")
                else:
                    print(f"  Recording: NO")
            elif command == '':
                # Empty input (just Enter key) - show status
                client.print_stats()
            else:
                print(f"Unknown command: {command}")

        except EOFError:
            # EOF received - just wait
            await asyncio.sleep(0.5)
        except Exception as e:
            logger.error(f"Error: {e}")
            await asyncio.sleep(0.5)


# ============================================================================
# Main Entry Point
# ============================================================================

async def main():
    """Main entry point."""
    print("=" * 60)
    print("Voice Bridge BLE - Mac Client (nRF54L15)")
    print("=" * 60)
    print()

    client = VoiceBridgeClient()

    try:
        # Find device
        address = await client.find_device()
        if not address:
            print("Device not found. Make sure XIAO is powered and advertising.")
            return

        # Connect
        await client.connect(address)

        # Start interactive control
        await interactive_control(client)

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Cleanup
        if client.recorder.is_recording:
            client.stop_recording()
            await asyncio.sleep(0.5)

        client.print_stats()
        await client.disconnect()
        client.close_serial()

        print("\nDone!")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
