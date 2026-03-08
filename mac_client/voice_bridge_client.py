#!/usr/bin/env python3
"""
Voice Bridge BLE - Mac Client

Receives audio from XIAO ESP32S3 Sense via BLE and plays it on Mac.
Uses bleak for BLE communication and pyaudio for audio playback.
"""

import asyncio
import logging
import struct
import sys
from typing import Optional

import numpy as np

try:
    import pyaudio
except ImportError:
    print("Please install pyaudio: pip install pyaudio")
    sys.exit(1)

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
# Custom service UUID (all zeros in the example, should be customized)
SERVICE_UUID = "00000000-0000-0000-0000-000000000000"
TX_CHARACTERISTIC_UUID = "00000002-0000-1000-8000-00805f9b34fb"  # XIAO -> Mac (Notify)
RX_CHARACTERISTIC_UUID = "00000003-0000-1000-8000-00805f9b34fb"  # Mac -> XIAO (Write)

# Audio configuration
SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2  # 16-bit
CHANNELS = 1

# ADPCM configuration
SAMPLES_PER_FRAME = 120
FRAME_SIZE = (SAMPLES_PER_FRAME * 4) // 8  # 60 bytes of ADPCM data


# ============================================================================
# Logging Setup
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# IMA ADPCM Decoder
# ============================================================================

class IMAADPCMDecoder:
    """IMA ADPCM decoder for 16-bit audio."""
    
    def __init__(self):
        self.step_size_table = [
            7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
            34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143,
            157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544,
            598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878,
            2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894,
            6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818,
            18500, 20350, 22385, 24623, 27086, 29794, 32767
        ]
        self.index_table = [-1, -1, -1, -1, 2, 4, 6, 8]
        self.reset()
    
    def reset(self):
        """Reset decoder state."""
        self.predictor = 0
        self.step_size = 7
    
    def decode(self, adpcm_data: bytes) -> np.ndarray:
        """
        Decode ADPCM data to PCM samples.
        
        Args:
            adpcm_data: ADPCM encoded bytes (first byte is sequence number)
            
        Returns:
            numpy array of int16 PCM samples
        """
        if len(adpcm_data) < 2:
            return np.array([], dtype=np.int16)
        
        # Skip sequence number (first byte)
        adpcm_bytes = adpcm_data[1:]
        
        output_samples = []
        
        for byte in adpcm_bytes:
            # Process two 4-bit nibbles
            for nibble_idx in range(2):
                adpcm_value = (byte >> (nibble_idx * 4)) & 0x0F
                
                # Extract sign and value
                sign = (adpcm_value >> 3) & 0x01
                value = adpcm_value & 0x07
                
                # Calculate step
                step = self.step_size
                if value & 0x04:
                    step += self.step_size >> 2
                if value & 0x02:
                    step += self.step_size >> 1
                if value & 0x01:
                    step += self.step_size >> 3
                
                # Update predictor
                if sign:
                    self.predictor -= step
                else:
                    self.predictor += step
                
                # Clamp predictor to 16-bit range
                self.predictor = max(-32768, min(32767, self.predictor))
                
                # Update step size
                index_change = self.index_table[value]
                step_index = 0
                for i, step_val in enumerate(self.step_size_table):
                    if step_val >= self.step_size:
                        step_index = i
                        break
                
                step_index += index_change
                step_index = max(0, min(88, step_index))
                self.step_size = self.step_size_table[step_index]
                
                # Output decoded sample
                output_samples.append(self.predictor)
        
        return np.array(output_samples, dtype=np.int16)


# ============================================================================
# Audio Player
# ============================================================================

class AudioPlayer:
    """Handles audio playback using PyAudio."""
    
    def __init__(self, sample_rate: int = SAMPLE_RATE):
        self.sample_rate = sample_rate
        self.pa = pyaudio.PyAudio()
        self.stream: Optional[pyaudio.Stream] = None
        self.adpcm_decoder = IMAADPCMDecoder()
        
    def start(self):
        """Start audio playback stream."""
        self.stream = self.pa.open(
            format=pyaudio.paInt16,
            channels=CHANNELS,
            rate=self.sample_rate,
            output=True,
            frames_per_buffer=1024
        )
        logger.info("Audio playback started")
    
    def stop(self):
        """Stop audio playback."""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None
        logger.info("Audio playback stopped")
    
    def play_samples(self, samples: np.ndarray):
        """Play PCM samples."""
        if self.stream and len(samples) > 0:
            audio_data = samples.tobytes()
            self.stream.write(audio_data)
    
    def decode_and_play(self, adpcm_frame: bytes):
        """Decode ADPCM frame and play the audio."""
        samples = self.adpcm_decoder.decode(adpcm_frame)
        self.play_samples(samples)
    
    def cleanup(self):
        """Clean up PyAudio resources."""
        self.stop()
        self.pa.terminate()


# ============================================================================
# BLE Client
# ============================================================================

class VoiceBridgeClient:
    """BLE client for Voice Bridge."""
    
    def __init__(self, device_name: str = DEVICE_NAME):
        self.device_name = device_name
        self.client: Optional[BleakClient] = None
        self.audio_player = AudioPlayer()
        self.is_connected = False
        self._last_seq_num = -1
    
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
            mtu_size=512,
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
        
        # Check sequence number for packet loss detection
        seq_num = data[0]
        if self._last_seq_num >= 0:
            expected_seq = (self._last_seq_num + 1) % 256
            if seq_num != expected_seq:
                logger.debug(f"Packet loss detected: expected={expected_seq}, got={seq_num}")
        self._last_seq_num = seq_num
        
        # Decode and play audio
        try:
            self.audio_player.decode_and_play(data)
        except Exception as e:
            logger.error(f"Error decoding audio: {e}")
    
    async def run(self):
        """Main run loop."""
        # Find device
        address = await self.find_device()
        if not address:
            return
        
        try:
            # Connect
            await self.connect(address)
            
            # Start audio playback
            self.audio_player.start()
            
            # Keep running until interrupted
            logger.info("Press Ctrl+C to stop")
            while True:
                await asyncio.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            # Cleanup
            self.audio_player.stop()
            await self.disconnect()
            self.audio_player.cleanup()


# ============================================================================
# Main Entry Point
# ============================================================================

async def main():
    """Main entry point."""
    print("=" * 60)
    print("Voice Bridge BLE - Mac Client")
    print("=" * 60)
    print()
    
    client = VoiceBridgeClient()
    await client.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
